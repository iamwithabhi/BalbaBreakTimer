# Balba Break Timer

A desk break timer for the **LilyGO T-Display-S3** (ESP32-S3, 170×320 ST7789) that reminds you to rest your eyes, following the [20-20-20 rule](https://www.healthline.com/health/eye-health/20-20-20-rule): every 20 minutes, look at something 20 feet away for 20 seconds.

The scene is animated by remixing assets from [Pokémon X / Y on The Models Resource](https://models.spriters-resource.com/3ds/pokemonxy).

A HUD shows the current state and a countdown (`SS` under a minute, `MM:SS` otherwise).

Everything is rendered on the CPU by a small software rasteriser (skinned meshes, textures, depth buffer) into an RGB565 framebuffer, which is then pushed to the display. No GPU is involved.

## Hardware

- LilyGO T-Display-S3 **with PSRAM** (the meshes and textures don't fit without it)
- USB-C cable

## Quick start

The project uses [PlatformIO](https://platformio.org/) with the Arduino framework on `espressif32`.

```bash
pio run -t upload      # build and flash
pio device monitor     # optional: serial console at 115200 baud
```

PlatformIO installs the only library dependency, `moononournation/GFX Library for Arduino@1.4.9`, automatically. The 3D assets are embedded in the firmware, so there is nothing else to upload.

The board starts on preset 1 (a 10-second test cycle). Press the bottom button to switch to a real preset.

## Timer presets

| Preset | Work | Break | Pan lap at end of work |
|--------|------|-------|------------------------|
| 1 (default, for testing) | 10 s | 10 s | no |
| 2 | 10 min | 20 s | yes |
| 3 | 20 min | 20 s | yes |

A **pan lap** is one full orbit of the camera around the scene, used as a heads-up that the break is about to start.

## Controls

| Button | Serial key | Action |
|--------|-----------|--------|
| Bottom (GPIO0) | — | Cycle presets 1 → 2 → 3 → 1 and restart the cycle |
| — | `1` `2` `3` | Select that preset |
| Top (GPIO14) | `p` | Run one pan lap now (the timer keeps running) |
| — | `r` | Restart the current cycle |

If your board is mounted upside down, swap `BUTTON_PAN_PIN` and `BUTTON_PRESET_PIN` in `src/main.cpp`.

Every two seconds the serial console also prints the frame rate and per-frame render and display times.

## Configuration

Set in the build flags in `platformio.ini`:

| Flag | Render size | Notes |
|------|-------------|-------|
| `BALBA_RENDER_DIV=2` (default) | 160×85, upscaled 2× | Fast |
| `BALBA_RENDER_DIV=1` | 320×170 (native) | Sharper, about 4× the pixel work |

## Project layout

```
platformio.ini      Board, partitions, build flags, embedded asset
assets/balba.bin    Baked meshes, skeleton and textures
src/main.cpp        Display setup, buttons, serial commands, frame loop
src/balba_core.h    Renderer core: math, rasteriser, skinning, choreography, camera, HUD
src/balba_blob.h    Loader for the baked asset blob (binary layout documented at the top)
```

`balba_core.h` has no OS or display dependencies and is shared with a desktop build of the same scene.

## Assets

You only need this section if you want to change the models or textures.

`assets/balba.bin` holds three meshes (Bulbasaur, laptop, Pikachu) and 13 textures already decoded to RGB565, so the board needs no image decoders or filesystem.

It is generated on a PC from the original SMD / OBJ / PNG / JPG files by the desktop build:

```bash
bulbasaur.exe --export balba.bin
```

(`export_to_board.bat` in the desktop project wraps this.) Copy the result to `assets/balba.bin` and rebuild.
