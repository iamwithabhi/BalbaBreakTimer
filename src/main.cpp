// Balba break timer - LilyGO T-Display-S3 (ESP32-S3, 8 MB PSRAM, 170x320 ST7789 on an 8-bit parallel bus).
//
// Bulbasaur works at a laptop, then takes a break and gazes at the horizon, on a loop. Everything is drawn by the
// CPU (software rasteriser in balba_core.h) into a RGB565 framebuffer that is then pushed to the display.
//
// Controls:
//   Top button    (GPIO14)  run one camera pan lap now (the timer is not touched; a lap near the break is shortened)
//   Bottom button (GPIO0)   switch timer preset:  1 = test 10s/10s   2 = 10 min/20 s   3 = 20 min/20 s, then back to 1
//   Serial monitor: '1' '2' '3' pick a preset, 'p' pan, 'r' restart the cycle
//   If your board is mounted the other way round, swap the two pin numbers below.
#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include "esp_heap_caps.h"

#ifndef BALBA_RENDER_DIV
#define BALBA_RENDER_DIV 2
#endif
#define BALBA_FB_W (320 / BALBA_RENDER_DIV)
#define BALBA_FB_H (170 / BALBA_RENDER_DIV)
#include "balba_core.h"
#include "balba_blob.h"

extern const uint8_t blobStart[] asm("_binary_assets_balba_bin_start");
extern const uint8_t blobEnd[] asm("_binary_assets_balba_bin_end");

namespace {
constexpr int DISPLAY_POWER_PIN = 15;
constexpr int BACKLIGHT_PIN = 38;
constexpr int BUTTON_PAN_PIN = 14;      // top button: camera pan
constexpr int BUTTON_PRESET_PIN = 0;    // bottom button: next timer preset
constexpr int SCREEN_W = 320;
constexpr int SCREEN_H = 170;
constexpr uint32_t DEBOUNCE_MS = 40;

Arduino_DataBus* bus = new Arduino_ESP32PAR8Q(7, 6, 8, 9, 39, 40, 41, 42, 45, 46, 47, 48);
Arduino_GFX* display = new Arduino_ST7789(bus, 5, 0, true, 170, 320, 35, 0, 35, 0);

Scene scene;
uint16_t* strip = nullptr;          // 2x upscale buffer (only used when BALBA_RENDER_DIV == 2)
uint32_t cycleStartMs = 0;
int preset = 1;

struct Button {
  int pin;
  bool down;
  uint32_t changedMs;
  explicit Button(int p) : pin(p), down(false), changedMs(0) {}
  // true once per press (falling edge, debounced)
  bool pressed() {
    bool now = digitalRead(pin) == LOW;
    uint32_t t = millis();
    if (now != down && t - changedMs > DEBOUNCE_MS) {
      down = now;
      changedMs = t;
      return now;
    }
    return false;
  }
};
Button panButton(BUTTON_PAN_PIN), presetButton(BUTTON_PRESET_PIN);

uint16_t* allocFrameBuffer(size_t bytes) {
  void* p = heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);   // fast internal RAM first
  if (!p) p = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  return static_cast<uint16_t*>(p);
}

void halt(const char* message) {
  Serial.println(message);
  display->fillScreen(0x0000);
  display->setTextColor(0xF800, 0x0000);
  display->setTextSize(2);
  display->setCursor(6, 6);
  display->print(message);
  while (true) delay(1000);
}

void selectPreset(int p) {
  preset = ((p - 1 + 3) % 3) + 1;
  applyPreset(preset);
  cycleStartMs = millis();   // restart the cycle at the beginning of a work period
  Serial.printf("preset %d: work %.0f s, break %.0f s\n", preset, g_workSecs, g_breakSecs);
}

// Push the finished framebuffer to the display.
void present() {
#if BALBA_RENDER_DIV == 1
  display->draw16bitRGBBitmap(0, 0, g_color, SCREEN_W, SCREEN_H);
#else
  // 2x pixel doubling, 8 source rows (16 screen rows) at a time
  for (int sy = 0; sy < FB_H; sy += 8) {
    int rows = min(8, FB_H - sy);
    uint16_t* out = strip;
    for (int r = 0; r < rows; r++) {
      const uint16_t* src = g_color + (sy + r) * FB_W;
      uint16_t* row0 = out;
      for (int x = 0; x < FB_W; x++) {
        row0[2 * x] = src[x];
        row0[2 * x + 1] = src[x];
      }
      memcpy(row0 + SCREEN_W, row0, SCREEN_W * sizeof(uint16_t));
      out += 2 * SCREEN_W;
    }
    display->draw16bitRGBBitmap(0, sy * 2, strip, SCREEN_W, rows * 2);
  }
#endif
}
}  // namespace

void setup() {
  Serial.begin(115200);
  pinMode(DISPLAY_POWER_PIN, OUTPUT);
  digitalWrite(DISPLAY_POWER_PIN, HIGH);
  pinMode(BUTTON_PAN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PRESET_PIN, INPUT_PULLUP);
  delay(10);
  if (!display->begin()) {
    Serial.println("Display initialization failed");
    while (true) delay(1000);
  }
  display->setRotation(1);
  pinMode(BACKLIGHT_PIN, OUTPUT);
  digitalWrite(BACKLIGHT_PIN, HIGH);
  display->fillScreen(0x0000);
  display->setTextColor(0x7DFF, 0x0000);
  display->setTextSize(2);
  display->setCursor(6, 6);
  display->print("LOADING...");

  if (!psramFound()) Serial.println("warning: no PSRAM found - meshes and textures will not fit");
  const size_t fbBytes = static_cast<size_t>(FB_W) * FB_H * sizeof(uint16_t);
  g_color = allocFrameBuffer(fbBytes);
  g_depth = allocFrameBuffer(fbBytes);
  if (!g_color || !g_depth) halt("OUT OF MEMORY");
#if BALBA_RENDER_DIV != 1
  strip = allocFrameBuffer(static_cast<size_t>(SCREEN_W) * 16 * sizeof(uint16_t));
  if (!strip) halt("OUT OF MEMORY");
#endif

  if (!scene.loadBlob(blobStart, static_cast<size_t>(blobEnd - blobStart))) halt("BAD ASSET BLOB");
  Serial.printf("assets loaded: %u KB blob, render %dx%d, free heap %u, free psram %u\n",
                static_cast<unsigned>((blobEnd - blobStart) / 1024), FB_W, FB_H, ESP.getFreeHeap(), ESP.getFreePsram());
  selectPreset(1);
}

void loop() {
  if (presetButton.pressed()) selectPreset(preset + 1);
  if (panButton.pressed()) requestPan();
  while (Serial.available()) {
    int c = Serial.read();
    if (c >= '1' && c <= '3') selectPreset(c - '0');
    else if (c == 'p') requestPan();
    else if (c == 'r') selectPreset(preset);
  }

  const uint32_t frameStart = millis();
  const float t = (frameStart - cycleStartMs) / 1000.0f;
  renderFrame(scene, t, 65.0f, 24.0f);
  const uint32_t renderDone = millis();
  present();
  const uint32_t frameEnd = millis();

  static uint32_t frames = 0, renderMs = 0, presentMs = 0, lastReport = 0;
  frames++;
  renderMs += renderDone - frameStart;
  presentMs += frameEnd - renderDone;
  if (frameEnd - lastReport >= 2000) {
    Serial.printf("%.1f fps  (render %lu ms, display %lu ms per frame)\n", frames * 1000.0f / (frameEnd - lastReport),
                  static_cast<unsigned long>(renderMs / frames), static_cast<unsigned long>(presentMs / frames));
    frames = renderMs = presentMs = 0;
    lastReport = frameEnd;
  }
}
