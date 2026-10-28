// Shared renderer core for the Bulbasaur scene: math, rasteriser, skinning, choreography, camera, HUD.
// Pure CPU, no OS or display dependencies - used by both the desktop build (src/main.cpp) and the ESP32 firmware.
// The platform supplies: framebuffer memory (g_color / g_depth), the clock, and the asset loading (Scene::load*).
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

// ----------------------------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------------------------
#ifndef BALBA_FB_W
#define BALBA_FB_W 320
#endif
#ifndef BALBA_FB_H
#define BALBA_FB_H 240
#endif
static const int FB_W = BALBA_FB_W;
static const int FB_H = BALBA_FB_H;
static float g_workSecs = 10.0f;     // changed at run time by the 1/2/3 presets
static float g_breakSecs = 10.0f;
static bool g_panEnabled = false;    // slow orbit around Bulbasaur at the end of long work periods (presets 2 and 3)
// Pan lap state (see renderFrame). requestPan() starts one lap right now without touching the timer.
static bool g_panRequest = false, g_panActive = false;
static float g_panStart = 0, g_panLen = 0;      // lap start (seconds into the work period) and length
static float g_panAzNow = 0;                      // current pan offset in degrees (for tests / debugging)
static long g_panAutoPeriod = -1;               // work/break cycle in which the end-of-work lap has already run
static void requestPan() { g_panRequest = true; }

// Framebuffer (RGB565) and 16-bit depth buffer. Allocated by the platform: FB_W * FB_H entries each.
static uint16_t* g_color = nullptr;
static uint16_t* g_depth = nullptr;

// ----------------------------------------------------------------------------------------
// Math
// ----------------------------------------------------------------------------------------
struct V3 { float x, y, z; };
static inline V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static inline V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static inline V3 operator*(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
static inline float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
static inline float len(V3 a) { return sqrtf(dot(a, a)); }
static inline V3 norm(V3 a) { float l = len(a); return l > 1e-12f ? a * (1.0f / l) : V3{0, 0, 1}; }
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float smooth01(float t) { t = clampf(t, 0, 1); return t * t * (3 - 2 * t); }
static float wrapPi(float a) {
    const float TAU_F = 6.28318530718f;
    a = fmodf(a + 3.14159265359f, TAU_F);
    if (a < 0) a += TAU_F;
    return a - 3.14159265359f;
}

// Affine transform: 3x3 rotation/scale + translation (what a skinning palette needs).
struct M34 { float m[3][4]; };
static M34 identity() { M34 r{}; r.m[0][0] = r.m[1][1] = r.m[2][2] = 1; return r; }
static M34 mul(const M34& a, const M34& b) {
    M34 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++) {
            r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j] + (j == 3 ? a.m[i][3] : 0.0f);
        }
    return r;
}
static V3 xformPoint(const M34& m, V3 p) {
    return {m.m[0][0] * p.x + m.m[0][1] * p.y + m.m[0][2] * p.z + m.m[0][3],
            m.m[1][0] * p.x + m.m[1][1] * p.y + m.m[1][2] * p.z + m.m[1][3],
            m.m[2][0] * p.x + m.m[2][1] * p.y + m.m[2][2] * p.z + m.m[2][3]};
}
static V3 xformDir(const M34& m, V3 p) {
    return {m.m[0][0] * p.x + m.m[0][1] * p.y + m.m[0][2] * p.z,
            m.m[1][0] * p.x + m.m[1][1] * p.y + m.m[1][2] * p.z,
            m.m[2][0] * p.x + m.m[2][1] * p.y + m.m[2][2] * p.z};
}
static M34 translation(V3 t) { M34 r = identity(); r.m[0][3] = t.x; r.m[1][3] = t.y; r.m[2][3] = t.z; return r; }
// Source-engine RadianEuler -> matrix: R = Rz * Ry * Rx
static M34 eulerSource(float rx, float ry, float rz) {
    float cx = cosf(rx), sx = sinf(rx), cy = cosf(ry), sy = sinf(ry), cz = cosf(rz), sz = sinf(rz);
    M34 X = identity(), Y = identity(), Z = identity();
    X.m[1][1] = cx; X.m[1][2] = -sx; X.m[2][1] = sx; X.m[2][2] = cx;
    Y.m[0][0] = cy; Y.m[0][2] = sy; Y.m[2][0] = -sy; Y.m[2][2] = cy;
    Z.m[0][0] = cz; Z.m[0][1] = -sz; Z.m[1][0] = sz; Z.m[1][1] = cz;
    return mul(Z, mul(Y, X));
}
// Rotation by `ang` around a (unit) axis through point `p` - a joint rotation in world axes.
static M34 rotateAbout(V3 axis, float ang, V3 p) {
    float c = cosf(ang), s = sinf(ang), t = 1 - c;
    M34 r = identity();
    r.m[0][0] = t * axis.x * axis.x + c;          r.m[0][1] = t * axis.x * axis.y - s * axis.z; r.m[0][2] = t * axis.x * axis.z + s * axis.y;
    r.m[1][0] = t * axis.x * axis.y + s * axis.z; r.m[1][1] = t * axis.y * axis.y + c;          r.m[1][2] = t * axis.y * axis.z - s * axis.x;
    r.m[2][0] = t * axis.x * axis.z - s * axis.y; r.m[2][1] = t * axis.y * axis.z + s * axis.x; r.m[2][2] = t * axis.z * axis.z + c;
    V3 rp = xformDir(r, p);
    r.m[0][3] = p.x - rp.x; r.m[1][3] = p.y - rp.y; r.m[2][3] = p.z - rp.z;
    return r;
}


// ----------------------------------------------------------------------------------------
// Textures: downsampled to RGB565 at load time (small enough for ESP32 PSRAM / flash).
// texel 0 means "transparent" (alpha test).
// ----------------------------------------------------------------------------------------
struct Texture {
    int w = 0, h = 0, wshift = 0;
    std::vector<uint16_t> px;
    bool valid() const { return !px.empty(); }
};
static inline uint16_t pack565(int r, int g, int b) { return (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)); }


// ----------------------------------------------------------------------------------------
// Meshes
// ----------------------------------------------------------------------------------------
struct Vert { V3 p, n; float u, v; int nl; int bone[3]; float w[3]; };
struct Tri { uint32_t i[3]; int mat; };
struct Mesh {
    std::vector<Vert> v;
    std::vector<Tri> t;
    std::vector<V3> wp, wn;     // posed position / normal, rebuilt every frame
    std::vector<float> lit;     // per-vertex light intensity
    float orient = 1;           // +1 if CCW winding faces along the vertex normal
    void finalize() {
        wp.resize(v.size()); wn.resize(v.size()); lit.resize(v.size());
        double s = 0;
        for (auto& tr : t) {
            V3 g = cross(v[tr.i[1]].p - v[tr.i[0]].p, v[tr.i[2]].p - v[tr.i[0]].p);
            s += dot(g, v[tr.i[0]].n + v[tr.i[1]].n + v[tr.i[2]].n);
        }
        orient = s >= 0 ? 1.0f : -1.0f;
    }
};

struct Skeleton {
    std::vector<std::string> name;
    std::vector<int> parent;
    std::vector<V3> bindPos;   // joint position in bind pose (world space)
    int find(const char* n) const { for (size_t i = 0; i < name.size(); i++) if (name[i] == n) return (int)i; return -1; }
};


// ----------------------------------------------------------------------------------------
// Camera (orthographic - keeps texture mapping affine, which is cheap on a microcontroller)
// ----------------------------------------------------------------------------------------
struct Camera {
    V3 toCam, right, up, fwd, center;
    float scale, ox, oy;
    void set(float azDeg, float elDeg, V3 c, float s, float screenX, float screenY) {
        float az = azDeg * 0.0174532925f, el = elDeg * 0.0174532925f;
        toCam = {sinf(az) * cosf(el), cosf(az) * cosf(el), sinf(el)};
        fwd = toCam * -1.0f;
        right = norm(cross(fwd, V3{0, 0, 1}));
        up = cross(right, fwd);
        center = c; scale = s; ox = screenX; oy = screenY;
    }
};
struct SV { float x, y, z, u, v, l; };
static const float Z_NEAR = -450.0f, Z_RANGE = 900.0f;

static inline SV project(const Camera& c, V3 p, float u, float v, float l) {
    V3 d = p - c.center;
    return {c.ox + dot(d, c.right) * c.scale, c.oy - dot(d, c.up) * c.scale, dot(d, c.fwd), u, v, l};
}

// ----------------------------------------------------------------------------------------
// Rasteriser: z-buffered, Gouraud-lit, affine-textured triangles. Integer-friendly inner loop.
// ----------------------------------------------------------------------------------------
static unsigned long g_stTris = 0, g_stPixels = 0, g_stTests = 0;   // per-frame statistics for --bench
static void drawTri(SV a, SV b, SV c, const Texture* tex, uint16_t flat) {
    g_stTris++;
    float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    if (fabsf(area) < 1e-5f) return;
    if (area < 0) { std::swap(b, c); area = -area; }
    int minx = std::max(0, (int)floorf(std::min(a.x, std::min(b.x, c.x))));
    int maxx = std::min(FB_W - 1, (int)ceilf(std::max(a.x, std::max(b.x, c.x))));
    int miny = std::max(0, (int)floorf(std::min(a.y, std::min(b.y, c.y))));
    int maxy = std::min(FB_H - 1, (int)ceilf(std::max(a.y, std::max(b.y, c.y))));
    if (minx > maxx || miny > maxy) return;
    float inv = 1.0f / area;
    float w0dx = b.y - c.y, w0dy = c.x - b.x;
    float w1dx = c.y - a.y, w1dy = a.x - c.x;
    float w2dx = a.y - b.y, w2dy = b.x - a.x;
    float px = minx + 0.5f, py = miny + 0.5f;
    float w0r = (c.x - b.x) * (py - b.y) - (c.y - b.y) * (px - b.x);
    float w1r = (a.x - c.x) * (py - c.y) - (a.y - c.y) * (px - c.x);
    float w2r = (b.x - a.x) * (py - a.y) - (b.y - a.y) * (px - a.x);
    // attribute gradients
    auto grad = [&](float A, float B, float C, float& dx, float& dy, float& row) {
        dx = (w0dx * A + w1dx * B + w2dx * C) * inv;
        dy = (w0dy * A + w1dy * B + w2dy * C) * inv;
        row = (w0r * A + w1r * B + w2r * C) * inv;
    };
    float zdx, zdy, zr, udx, udy, ur, vdx, vdy, vr, ldx, ldy, lr;
    float tw = tex ? (float)tex->w : 0, th = tex ? (float)tex->h : 0;
    grad(a.z, b.z, c.z, zdx, zdy, zr);
    grad(a.u * tw, b.u * tw, c.u * tw, udx, udy, ur);
    grad((1 - a.v) * th, (1 - b.v) * th, (1 - c.v) * th, vdx, vdy, vr);
    grad(a.l, b.l, c.l, ldx, ldy, lr);
    const float zscale = 65535.0f / Z_RANGE;
    for (int y = miny; y <= maxy; y++, w0r += w0dy, w1r += w1dy, w2r += w2dy, zr += zdy, ur += udy, vr += vdy, lr += ldy) {
        float w0 = w0r, w1 = w1r, w2 = w2r, z = zr, u = ur, v = vr, l = lr;
        size_t row = (size_t)y * FB_W;
        for (int x = minx; x <= maxx; x++, w0 += w0dx, w1 += w1dx, w2 += w2dx, z += zdx, u += udx, v += vdx, l += ldx) {
            if (w0 < 0 || w1 < 0 || w2 < 0) continue;
            g_stTests++;
            int zq = (int)((z - Z_NEAR) * zscale);
            if (zq < 0) zq = 0; else if (zq > 65535) zq = 65535;
            if (zq > g_depth[row + x]) continue;
            uint16_t col;
            if (tex) {
                int tx = ((int)(u + tex->w * 64.0f)) & (tex->w - 1);
                int ty = ((int)(v + tex->h * 64.0f)) & (tex->h - 1);
                uint16_t t = tex->px[((size_t)ty << tex->wshift) + tx];
                if (!t) continue;
                int li = (int)(l * 256.0f);
                if (li < 0) li = 0;
                int r = ((t >> 11) * li) >> 8, g = (((t >> 5) & 63) * li) >> 8, bl = ((t & 31) * li) >> 8;
                col = (uint16_t)(((r > 31 ? 31 : r) << 11) | ((g > 63 ? 63 : g) << 5) | (bl > 31 ? 31 : bl));
            } else col = flat;
            g_stPixels++;
            g_depth[row + x] = (uint16_t)zq;
            g_color[row + x] = col;
        }
    }
}

// ----------------------------------------------------------------------------------------
// Text (5x7 bitmap font) for the HUD timer
// ----------------------------------------------------------------------------------------
static const uint8_t* glyph(char c) {
    static const uint8_t G[][7] = {
        {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}, {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}, {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F},
        {0x1E,0x01,0x01,0x0E,0x01,0x01,0x1E}, {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}, {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},
        {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}, {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}, {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},
        {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C},
        {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}, {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}, {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F},
        {0x0E,0x11,0x10,0x17,0x11,0x11,0x0E}, {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}, {0x11,0x12,0x14,0x18,0x14,0x12,0x11},
        {0x11,0x19,0x15,0x13,0x11,0x11,0x11}, {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}, {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11},
        {0x11,0x11,0x11,0x15,0x15,0x1B,0x11}, {0x00,0x00,0x00,0x00,0x00,0x00,0x00}, {0x00,0x04,0x04,0x00,0x04,0x04,0x00}};
    if (c >= '0' && c <= '9') return G[c - '0'];
    switch (c) {
        case 'A': return G[10]; case 'B': return G[11]; case 'E': return G[12]; case 'G': return G[13]; case 'I': return G[14];
        case 'K': return G[15]; case 'N': return G[16]; case 'O': return G[17]; case 'R': return G[18]; case 'W': return G[19];
        case ':': return G[21];
        default: return G[20];
    }
}
static void fillRect(int x, int y, int w, int h, uint16_t c) {
    for (int j = std::max(0, y); j < std::min(FB_H, y + h); j++)
        for (int i = std::max(0, x); i < std::min(FB_W, x + w); i++) g_color[j * FB_W + i] = c;
}
static void drawText(int x, int y, const char* s, int sc, uint16_t c, uint16_t shadow) {
    for (int pass = 0; pass < 2; pass++) {
        int cx = x + (pass == 0 ? sc : 0), cy = y + (pass == 0 ? sc : 0);
        for (const char* p = s; *p; p++, cx += 6 * sc) {
            const uint8_t* g = glyph(*p);
            for (int r = 0; r < 7; r++)
                for (int col = 0; col < 5; col++)
                    if (g[r] & (0x10 >> col)) fillRect(cx + col * sc, cy + r * sc, sc, sc, pass == 0 ? shadow : c);
        }
    }
}

// ----------------------------------------------------------------------------------------
// Scene
// ----------------------------------------------------------------------------------------
static const float WALK_DIST = 42.0f;
static const float WALK_SIDE = 30.0f;   // he also drifts to his right (+Y) so the laptop stays out of the gaze shot

// Body yaw (about Z, 0 = facing the laptop) and X offset at `b` seconds into the break.
static void choreoAt(float b, float& yaw, float& x) {
    const float QUARTER_TURN = 1.5707963f;
    yaw = 0; x = 0;
    if (b < 0.9f) yaw = -QUARTER_TURN * smooth01(b / 0.9f);                                           // turn to face -X
    else if (b < 6.8f) { yaw = -QUARTER_TURN; x = -WALK_DIST * smooth01((b - 0.9f) / 2.3f); }         // walk out, then gaze
    else if (b < 7.7f) { yaw = -QUARTER_TURN + 2 * QUARTER_TURN * smooth01((b - 6.8f) / 0.9f); x = -WALK_DIST; }  // turn around
    else if (b < 9.0f) { yaw = QUARTER_TURN; x = -WALK_DIST * (1 - smooth01((b - 7.7f) / 1.3f)); }    // walk back
    else yaw = QUARTER_TURN * (1 - smooth01((b - 9.0f) / 1.0f));                                      // face the laptop again
}

// The break choreography is authored for 10 s. For other lengths the walk out / walk back are sped up (short breaks)
// and the gaze in the middle is stretched or shortened to fill the rest.
static float canonicalBreakTime(float br) {
    float B = g_breakSecs, k = std::min(1.0f, B / 10.0f), out = 3.2f * k, gaze = B - 6.4f * k;
    if (br < out) return br / k;
    if (br < out + gaze) return 3.2f + (br - out) * (3.6f / gaze);
    return 6.8f + (br - out - gaze) / k;
}

struct Scene {
    Mesh bulb, laptop;
    Skeleton sk;
    Texture bulbTex[4];                  // BodyA1, BodyB1, Eye1, Iris1
    Texture laptopKeys, screenWork, screenBreak, wallTex, floorTex;
    Mesh pika;                           // rigid (bind-pose) Pikachu, animated as a hopping body
    Texture pikaTex[4];                  // BodyA1, BodyB1, Mouth1, Eye1
    uint16_t skyColor = 0;
    int bShoulder[2], bHand[2], bThigh[2], bHead, bSpine2, bOrigin;
    float rootYaw = 0, rootX = 0, rootY = 0, camBlend = 0;   // set by pose(), read by the renderer
    float armTarget[2];                  // working pose: hand target, per arm (yz-plane direction angle)
    std::vector<M34> palette;
    std::vector<M34> joint;
    bool load(const std::string& root);                       // desktop: build from the original asset files
    bool loadBlob(const uint8_t* data, size_t size);          // desktop + board: load the baked asset blob (balba_blob.h)

    // ---- animation: procedural joint rotations in world axes about each bind-pose joint ----
    void pose(float cycleT, float time) {
        const float PI_F = 3.14159265f;
        bool working = cycleT < g_workSecs;
        float br = working ? -1.0f : cycleT - g_workSecs;        // real seconds into the break
        float b = working ? -1.0f : canonicalBreakTime(br);       // matching point on the 10 s choreography
        for (auto& j : joint) j = identity();

        // Break choreography: turn away from the laptop, walk out, gaze at the horizon, walk back, sit down again.
        float yaw = 0, x = 0, yaw2 = 0, x2 = 0;
        if (!working) { choreoAt(b, yaw, x); choreoAt(canonicalBreakTime(br + 0.03f), yaw2, x2); }
        float speed = fabsf(x2 - x) / 0.03f, turnRate = fabsf(yaw2 - yaw) / 0.03f;
        float amp = std::max(clampf(speed / 9.0f, 0, 1), clampf(turnRate / 4.0f, 0, 0.6f));
        float walkDist = b < 6.8f ? -x : 2 * WALK_DIST + x;
        float gait = walkDist * 2 * PI_F / 14.0f + 4.0f * std::max(br, 0.0f);
        float atKeys = 1.0f;
        if (!working) atKeys = b < 0.5f ? 1 - smooth01(b / 0.5f) : (b > 9.1f ? smooth01((b - 9.1f) / 0.9f) : 0.0f);
        float gaze = working ? 0.0f : smooth01((b - 3.0f) / 0.8f) * (1 - smooth01((b - 6.4f) / 0.6f));
        float y = -x / WALK_DIST * WALK_SIDE;
        rootYaw = yaw; rootX = x; rootY = y;
        camBlend = working ? 0.0f : (b < 1.4f ? smooth01(b / 1.4f) : (b > 8.6f ? 1 - smooth01((b - 8.6f) / 1.4f) : 1.0f));

        // arms: typing taps on the keys, swinging while walking
        float tapHz = 5.5f;
        for (int a = 0; a < 2; a++) {
            V3 sh = sk.bindPos[bShoulder[a]];
            float tap = std::max(0.0f, sinf(2 * PI_F * tapHz * time + a * PI_F));
            float typeAng = armTarget[a] - 0.16f * tap;
            float restAng = 0.55f * amp * sinf(gait + (a ? PI_F : 0.0f));
            joint[bShoulder[a]] = rotateAbout({1, 0, 0}, restAng + (typeAng - restAng) * atKeys, sh);
            joint[bThigh[a]] = rotateAbout({1, 0, 0}, 0.45f * amp * sinf(gait + (a ? 0.0f : PI_F)), sk.bindPos[bThigh[a]]);
        }
        // head: looks down at the screen while working, looks far out (and slowly around) during the gaze
        float headPitch = atKeys * (0.10f + 0.03f * sinf(2 * PI_F * 2.2f * time)) - (1 - atKeys) * 0.03f - gaze * (0.22f + 0.04f * sinf(2 * PI_F * 0.5f * time));
        float headYaw = gaze * 0.45f * sinf(2 * PI_F * 0.22f * time);
        V3 hp = sk.bindPos[bHead];
        joint[bHead] = mul(rotateAbout({0, 0, 1}, headYaw, hp), rotateAbout({1, 0, 0}, headPitch, hp));
        joint[bSpine2] = rotateAbout({1, 0, 0}, 0.015f * sinf(2 * PI_F * 0.4f * time) - 0.05f * gaze, sk.bindPos[bSpine2]);
        // whole body: yaw around its centre, walk along -X, small bob in step
        joint[0] = mul(translation({x, y, 1.2f * amp * fabsf(sinf(gait))}), rotateAbout({0, 0, 1}, yaw, {0, -8, 0}));

        for (size_t i = 0; i < sk.parent.size(); i++)
            palette[i] = sk.parent[i] >= 0 ? mul(palette[sk.parent[i]], joint[i]) : joint[i];
    }

    // Pikachu: a small figure far out on the horizon, hopping on the spot and facing Bulbasaur (+X).
    void hopPika(float time) {
        const float PERIOD = 0.8f, SCALE = 0.18f, HOP_HEIGHT = 9.0f, TAU_F = 6.2831853f;
        const V3 spot = {-300.0f, 85.0f, 0.0f};
        float ph = fmodf(time, PERIOD) / PERIOD;
        float h = 4 * ph * (1 - ph);                                    // 0 on the ground, 1 at the top of the hop
        float sz = 1.0f + 0.18f * h - 0.22f * (1 - smooth01(h / 0.2f));  // crouch before take-off / on landing, stretch in the air
        float sxy = 1.0f / sqrtf(sz);
        float yaw = 1.5707963f + 0.25f * sinf(TAU_F * ph);              // slight wiggle while airborne
        float cy = cosf(yaw), sy = sinf(yaw);
        for (size_t i = 0; i < pika.v.size(); i++) {
            const Vert& v = pika.v[i];
            float x = v.p.x * sxy * SCALE, y = v.p.y * sxy * SCALE, z = v.p.z * sz * SCALE;
            pika.wp[i] = {spot.x + x * cy - y * sy, spot.y + x * sy + y * cy, z + HOP_HEIGHT * h};
            pika.wn[i] = {v.n.x * cy - v.n.y * sy, v.n.x * sy + v.n.y * cy, v.n.z};
        }
    }

    void skin() {
        for (size_t i = 0; i < bulb.v.size(); i++) {
            const Vert& v = bulb.v[i];
            V3 p{0, 0, 0}, n{0, 0, 0};
            for (int k = 0; k < v.nl; k++) {
                const M34& m = palette[v.bone[k]];
                p = p + xformPoint(m, v.p) * v.w[k];
                n = n + xformDir(m, v.n) * v.w[k];
            }
            bulb.wp[i] = p; bulb.wn[i] = norm(n);
        }
    }
};

static void lightMesh(Mesh& m, V3 L) {
    for (size_t i = 0; i < m.v.size(); i++) {
        float d = dot(m.wn[i], L);
        m.lit[i] = 0.50f + 0.62f * (d > 0 ? d : 0);
    }
}

static void drawMesh(Mesh& m, const Camera& cam, const Texture* const* mats, int nmats) {
    for (const Tri& t : m.t) {
        const V3 &a = m.wp[t.i[0]], &b = m.wp[t.i[1]], &c = m.wp[t.i[2]];
        if (dot(cross(b - a, c - a), cam.toCam) * m.orient <= 0) continue;
        const Texture* tex = t.mat < nmats ? mats[t.mat] : nullptr;
        if (!tex || !tex->valid()) continue;
        const Vert &va = m.v[t.i[0]], &vb = m.v[t.i[1]], &vc = m.v[t.i[2]];
        drawTri(project(cam, a, va.u, va.v, m.lit[t.i[0]]), project(cam, b, vb.u, vb.v, m.lit[t.i[1]]),
                project(cam, c, vc.u, vc.v, m.lit[t.i[2]]), tex, 0);
    }
}

// The environment, built from Backdrop.jpg so it reads as one continuous place:
//  - floor: a patch of the painted grass, mirror-tiled so it repeats without seams
//  - wall:  the sky/cloud/hill strip bent into a ring around the scene; the image is mirrored on every
//           other segment so neighbouring edges match and the ring closes with no visible join.
//  The wall's bottom edge is grass painted in the same colours as the floor, hiding the horizon seam.
static void drawWorld(const Camera& cam, const Scene& sc) {
    const float CX = 0, CY = -20, R = 335, WALL_H = 78, HALF = 365, TILE_X = 160, TILE_Y = 80;
    auto fv = [&](float x, float y) { return project(cam, {x, y, -0.4f}, (x - CX) / TILE_X, 1 - (y - CY) / TILE_Y, 0.97f); };
    SV f00 = fv(CX - HALF, CY - HALF), f10 = fv(CX + HALF, CY - HALF), f11 = fv(CX + HALF, CY + HALF), f01 = fv(CX - HALF, CY + HALF);
    drawTri(f00, f10, f11, &sc.floorTex, 0);
    drawTri(f00, f11, f01, &sc.floorTex, 0);

    const int SEGMENTS = 10, QUADS_PER_SEG = 12;                  // 10 x 36 degrees, even count so mirroring closes the ring
    const float STEP = 6.2831853f / (SEGMENTS * QUADS_PER_SEG), START = 90.0f * 0.0174532925f;
    for (int i = 0; i < SEGMENTS * QUADS_PER_SEG; i++) {
        int k = i / QUADS_PER_SEG;
        float f0 = (float)(i % QUADS_PER_SEG) / QUADS_PER_SEG, f1 = (float)(i % QUADS_PER_SEG + 1) / QUADS_PER_SEG;
        float u0 = (k & 1) ? 1 - f0 : f0, u1 = (k & 1) ? 1 - f1 : f1;
        float a0 = START + i * STEP, a1 = a0 + STEP;
        float am = a0 + 0.5f * STEP;
        if (-(cosf(am) * cam.toCam.x + sinf(am) * cam.toCam.y) <= 0) continue;   // near side of the ring: faces away, would block the view
        float x0 = CX + R * cosf(a0), y0 = CY + R * sinf(a0), x1 = CX + R * cosf(a1), y1 = CY + R * sinf(a1);
        SV b0 = project(cam, {x0, y0, -1.0f}, u0, 0, 1), b1 = project(cam, {x1, y1, -1.0f}, u1, 0, 1);
        SV t0 = project(cam, {x0, y0, WALL_H}, u0, 1, 1), t1 = project(cam, {x1, y1, WALL_H}, u1, 1, 1);
        drawTri(b0, b1, t1, &sc.wallTex, 0);
        drawTri(b0, t1, t0, &sc.wallTex, 0);
    }
}

// 1 when the camera (azimuth in degrees) looks at the back of the laptop, fading to 0 about 60 degrees either side.
static float laptopBackness(float azDeg) {
    float d = fabsf(wrapPi((azDeg - 180.0f) * 0.0174532925f)) * 57.29578f;
    return 1.0f - smooth01(d / 60.0f);
}

// Camera orbit angle (degrees, 0..360) after fraction f (0..1) of the pan time. The orbit is slow in general but
// laptopBackness() makes it up to 4x faster while the laptop's back is in view. The speed profile is tabulated per
// starting azimuth: cumulative time per degree, normalised so f = 1 lands exactly on 360 degrees.
static float panAngle(float f, float baseAz) {
    const float BOOST = 3.0f;
    static float cum[361];
    static float tableAz = -1e9f;
    if (tableAz != baseAz) {
        cum[0] = 0;
        for (int i = 0; i < 360; i++) cum[i + 1] = cum[i] + 1.0f / (1.0f + BOOST * laptopBackness(baseAz + i + 0.5f));
        for (int i = 1; i <= 360; i++) cum[i] /= cum[360];
        tableAz = baseAz;
    }
    f = clampf(f, 0.0f, 1.0f);
    int i = 0;
    while (i < 359 && cum[i + 1] < f) i++;
    float span = cum[i + 1] - cum[i];
    return i + (span > 1e-9f ? (f - cum[i]) / span : 0.0f);
}
static void renderFrame(Scene& sc, float time, float az, float el) {
    float cycle = fmodf(time, g_workSecs + g_breakSecs);
    bool working = cycle < g_workSecs;
    sc.pose(cycle, time);
    sc.skin();

    // During the break the camera swings round behind Bulbasaur and pulls back to show the world.
    float cb = sc.camBlend;
    Camera cam;
    // Gaze shot: camera behind Bulbasaur's right shoulder, low, looking along his heading (-X) at the horizon.
    V3 center = {(sc.rootX + 22.0f) * cb, -22.0f * (1 - cb) + (sc.rootY + 45.0f) * cb, 20.0f + 10.0f * cb};   // follows him, held to his right
    // Pre-break pan: one slow 360-degree orbit that rises and falls twice, ends back on the default angle shortly before
    // the break, and hurries past the angles where the back of the laptop fills the frame. It starts by itself
    // 25.5 s before the break (presets 2 and 3), or on request (requestPan) at any time while working. The lap
    // never touches the work/break timer; if there is little time left it is compressed so it finishes in time.
    float panAz = 0, panEl = 0;
    if (working) {
        const float PAN_LEN = 24.0f, PAN_LEAD = 1.5f, PAN_MIN = 1.5f;
        float untilBreak = g_workSecs - cycle;
        long period = (long)(time / (g_workSecs + g_breakSecs));
        bool autoDue = g_panEnabled && untilBreak <= PAN_LEN + PAN_LEAD && g_panAutoPeriod != period;
        if ((g_panRequest || autoDue) && !g_panActive) {
            float lap = std::min(PAN_LEN, untilBreak - PAN_LEAD);
            if (lap >= PAN_MIN) {
                g_panActive = true; g_panStart = cycle; g_panLen = lap;
                if (cycle + lap > g_workSecs - (PAN_LEN + PAN_LEAD)) g_panAutoPeriod = period;   // a lap reaching into the end-of-work window counts as that period's lap
            }
        }
        g_panRequest = false;
    }
    if (g_panActive) {
        float p = (cycle - g_panStart) / g_panLen;
        if (!working || p < 0.0f || p >= 1.0f) g_panActive = false;
        else {
            float f = smooth01(p);                                              // eased progress through the orbit
            panAz = panAngle(f, az);
            float behind = laptopBackness(az + panAz);
            float side = sinf((az + panAz) * 0.0174532925f);                    // +-1 when the camera is level with Bulbasaur's flanks
            side *= side;
            float envelope = smooth01(std::min(f, 1.0f - f) / 0.12f);          // fade in/out so the lap starts and ends on the default view
            panEl = envelope * (6.0f * behind - 20.0f * side);                  // low alongside Bulbasaur, high at front/back, lifted over the laptop
        }
    }
    g_panAzNow = panAz;
    cam.set(az + panAz + 15.0f * cb, clampf(el + panEl - 15.0f * cb, 4.0f, 85.0f), center, (2.35f - 0.55f * cb) * (FB_H / 240.0f), FB_W * 0.5f, FB_H * (0.60f + 0.12f * cb));
    V3 L = norm(cam.toCam * 0.7f + cam.up * 0.55f + cam.right * -0.45f);

    // sky colour + z clear
    for (int i = 0; i < FB_W * FB_H; i++) { g_color[i] = sc.skyColor; g_depth[i] = 0xFFFF; }
    drawWorld(cam, sc);
    // blob shadow under Bulbasaur (follows the walk and turn)
    {
        float cy = cosf(sc.rootYaw), sy = sinf(sc.rootYaw);
        auto at = [&](float lx, float ly) { return V3{sc.rootX + lx * cy - (ly + 8) * sy, sc.rootY - 8 + lx * sy + (ly + 8) * cy, -0.2f}; };
        SV c0 = project(cam, at(0, -5), 0, 0, 1);
        for (int i = 0; i < 24; i++) {
            float a0 = i * 6.2831853f / 24, a1 = (i + 1) * 6.2831853f / 24;
            drawTri(c0, project(cam, at(38 * cosf(a0), -5 + 52 * sinf(a0)), 0, 0, 1),
                    project(cam, at(38 * cosf(a1), -5 + 52 * sinf(a1)), 0, 0, 1), nullptr, pack565(88, 128, 45));
        }
    }

    lightMesh(sc.laptop, L);   // laptop is static: wp/wn are its fixed vertices
    lightMesh(sc.bulb, L);
    const Texture* laptopMats[2] = {working ? &sc.screenWork : &sc.screenBreak, &sc.laptopKeys};
    const Texture* bulbMats[4] = {&sc.bulbTex[0], &sc.bulbTex[1], &sc.bulbTex[2], &sc.bulbTex[3]};
    drawMesh(sc.laptop, cam, laptopMats, 2);
    drawMesh(sc.bulb, cam, bulbMats, 4);
    if (cb > 0.01f) {   // Pikachu only exists out on the horizon, so skip him while the camera is on the desk
        const Texture* pikaMats[4] = {&sc.pikaTex[0], &sc.pikaTex[1], &sc.pikaTex[2], &sc.pikaTex[3]};
        sc.hopPika(time);
        lightMesh(sc.pika, L);
        drawMesh(sc.pika, cam, pikaMats, 4);
    }

    // HUD: state + countdown (SS under a minute, MM:SS above)
    float phaseLen = working ? g_workSecs : g_breakSecs;
    float inPhase = working ? cycle : cycle - g_workSecs;
    int remain = (int)ceilf(phaseLen - inPhase);
    if (remain < 0) remain = 0;
    char num[8];
    bool minutes = remain >= 60;
    if (minutes) snprintf(num, sizeof(num), "%02d:%02d", remain / 60, remain % 60);
    else snprintf(num, sizeof(num), "%02d", remain);
    uint16_t accent = working ? pack565(120, 255, 120) : pack565(255, 190, 70);
    const int hs = FB_W >= 320 ? 2 : 1;   // HUD size: 2 = full-size layout, 1 = half-resolution render (board)
    fillRect(2 * hs, 2 * hs, hs == 2 ? 112 : 68, 26 * hs, pack565(16, 24, 32));
    drawText(5 * hs, 4 * hs + (hs == 1 ? 1 : 1), working ? "WORKING" : "BREAK", hs, accent, 0);
    drawText(5 * hs, (minutes ? 14 : 13) * hs + (hs == 1 ? 1 : 0), num, minutes ? (hs == 2 ? 3 : 2) : 2 * hs, pack565(255, 255, 255), 0);
}

// Timer presets. Platform-independent so a hardware button can call it later.
static void applyPreset(int preset) {
    switch (preset) {
        case 1: g_workSecs = 10;   g_breakSecs = 10; g_panEnabled = false; break;   // test
        case 2: g_workSecs = 600;  g_breakSecs = 20; g_panEnabled = true; break;   // work setting 1: 10 min / 20 s
        case 3: g_workSecs = 1200; g_breakSecs = 20; g_panEnabled = true; break;   // work setting 2: 20 min / 20 s
        default: return;
    }
    g_panActive = false; g_panRequest = false; g_panAutoPeriod = -1;   // the cycle restarts, so forget any lap in progress
    printf("preset %d: work %.0f s, break %.0f s\n", preset, g_workSecs, g_breakSecs);
}
