// Baked asset blob: everything the renderer needs, already decoded and shrunk for a microcontroller.
//
// The desktop program reads the original SMD / OBJ / PNG / JPG assets and bakes them into one file
// (`bulbasaur.exe --export balba.bin`). The ESP32 firmware embeds that file and loads it with Scene::loadBlob,
// so the board needs no PNG/JPEG decoders and no filesystem.
//
// Layout (little-endian, packed, read sequentially):
//   u32 magic 'BALB', u32 version, u16 skyColor, u16 pad
//   i32 bShoulder[2], bHand[2], bThigh[2], bHead, bSpine2, bOrigin;  f32 armTarget[2]
//   u32 nBones; i32 parent[nBones]; f32 bindPos[nBones][3]
//   3 meshes (Bulbasaur, laptop, Pikachu): u32 nVerts, u32 nTris, f32 orient, then
//        vertex: f32 pos[3], i8 normal[3] (/127), u8 nLinks, i16 uv[2] (/2048), u8 bone[3], u8 weight[3] (/255)
//        tri:    u16 index[3], u8 material
//   13 textures (bulb x4, laptop keys, screen work, screen break, wall, floor, pikachu x4):
//        u32 w, u32 h, u16 rgb565[w * h]   (texel 0 = transparent)
#pragma once
#include "balba_core.h"

static const uint32_t BLOB_MAGIC = 0x424C4142;   // 'BALB'
static const uint32_t BLOB_VERSION = 1;

namespace blob {
struct Reader {
    const uint8_t *p, *end;
    bool ok = true;
    Reader(const uint8_t* begin, const uint8_t* stop) : p(begin), end(stop) {}
    template <class T> T get() {
        T v{};
        if (p + sizeof(T) > end) { ok = false; return v; }
        memcpy(&v, p, sizeof(T));
        p += sizeof(T);
        return v;
    }
};

__attribute__((noinline)) static bool readMesh(Reader& r, Mesh& m) {
    uint32_t nv = r.get<uint32_t>(), nt = r.get<uint32_t>();
    m.orient = r.get<float>();
    if (!r.ok || nv > 65535 || nt > 200000) return false;
    m.v.resize(nv);
    for (uint32_t i = 0; i < nv && r.ok; i++) {
        Vert& v = m.v[i];
        v.p = {r.get<float>(), r.get<float>(), r.get<float>()};
        v.n = {r.get<int8_t>() / 127.0f, r.get<int8_t>() / 127.0f, r.get<int8_t>() / 127.0f};
        v.nl = r.get<uint8_t>();
        v.u = r.get<int16_t>() / 2048.0f;
        v.v = r.get<int16_t>() / 2048.0f;
        float sum = 0;
        for (int k = 0; k < 3; k++) v.bone[k] = r.get<uint8_t>();
        for (int k = 0; k < 3; k++) { v.w[k] = r.get<uint8_t>() / 255.0f; if (k < v.nl) sum += v.w[k]; }
        if (sum > 0) for (int k = 0; k < v.nl; k++) v.w[k] /= sum;
    }
    m.t.resize(nt);
    for (uint32_t i = 0; i < nt && r.ok; i++) {
        for (int k = 0; k < 3; k++) m.t[i].i[k] = r.get<uint16_t>();
        m.t[i].mat = r.get<uint8_t>();
    }
    m.wp.resize(nv); m.wn.resize(nv); m.lit.resize(nv);
    return r.ok;
}

__attribute__((noinline)) static bool readTexture(Reader& r, Texture& t) {
    uint32_t w = r.get<uint32_t>(), h = r.get<uint32_t>();
    if (!r.ok || w == 0 || h == 0 || (w & (w - 1)) || (h & (h - 1)) || w > 1024 || h > 1024) return false;
    size_t bytes = (size_t)w * h * 2;
    if (r.p + bytes > r.end) return false;
    t.w = (int)w; t.h = (int)h; t.wshift = 0;
    while ((1u << t.wshift) < w) t.wshift++;
    t.px.resize((size_t)w * h);
    memcpy(t.px.data(), r.p, bytes);
    r.p += bytes;
    return true;
}
}  // namespace blob

__attribute__((noinline)) inline bool Scene::loadBlob(const uint8_t* data, size_t size) {
    blob::Reader r(data, data + size);
    if (r.get<uint32_t>() != BLOB_MAGIC || r.get<uint32_t>() != BLOB_VERSION) return false;
    skyColor = r.get<uint16_t>();
    r.get<uint16_t>();
    bShoulder[0] = r.get<int32_t>(); bShoulder[1] = r.get<int32_t>();
    bHand[0] = r.get<int32_t>(); bHand[1] = r.get<int32_t>();
    bThigh[0] = r.get<int32_t>(); bThigh[1] = r.get<int32_t>();
    bHead = r.get<int32_t>(); bSpine2 = r.get<int32_t>(); bOrigin = r.get<int32_t>();
    armTarget[0] = r.get<float>(); armTarget[1] = r.get<float>();
    uint32_t nb = r.get<uint32_t>();
    if (!r.ok || nb == 0 || nb > 256) return false;
    sk.parent.resize(nb); sk.bindPos.resize(nb);
    for (uint32_t i = 0; i < nb; i++) sk.parent[i] = r.get<int32_t>();
    for (uint32_t i = 0; i < nb; i++) sk.bindPos[i] = {r.get<float>(), r.get<float>(), r.get<float>()};
    if (!blob::readMesh(r, bulb) || !blob::readMesh(r, laptop) || !blob::readMesh(r, pika)) return false;
    Texture* tex[13] = {&bulbTex[0], &bulbTex[1], &bulbTex[2], &bulbTex[3], &laptopKeys, &screenWork, &screenBreak,
                        &wallTex, &floorTex, &pikaTex[0], &pikaTex[1], &pikaTex[2], &pikaTex[3]};
    for (Texture* t : tex) if (!blob::readTexture(r, *t)) return false;
    for (size_t i = 0; i < laptop.v.size(); i++) { laptop.wp[i] = laptop.v[i].p; laptop.wn[i] = norm(laptop.v[i].n); }   // laptop is static
    palette.resize(nb); joint.resize(nb);
    return true;
}

#ifdef BALBA_BLOB_WRITER
// Desktop only: bake a fully loaded Scene into the blob format above.
static bool writeBlob(const Scene& sc, const char* path) {
    std::vector<uint8_t> out;
    auto put = [&](const void* p, size_t n) { const uint8_t* b = (const uint8_t*)p; out.insert(out.end(), b, b + n); };
    auto u8 = [&](uint8_t v) { put(&v, 1); };
    auto u16 = [&](uint16_t v) { put(&v, 2); };
    auto i16 = [&](int16_t v) { put(&v, 2); };
    auto u32 = [&](uint32_t v) { put(&v, 4); };
    auto i32 = [&](int32_t v) { put(&v, 4); };
    auto f32 = [&](float v) { put(&v, 4); };
    auto q8 = [&](float v) { int8_t x = (int8_t)lroundf(clampf(v, -1, 1) * 127.0f); put(&x, 1); };
    u32(BLOB_MAGIC); u32(BLOB_VERSION); u16(sc.skyColor); u16(0);
    i32(sc.bShoulder[0]); i32(sc.bShoulder[1]); i32(sc.bHand[0]); i32(sc.bHand[1]); i32(sc.bThigh[0]); i32(sc.bThigh[1]);
    i32(sc.bHead); i32(sc.bSpine2); i32(sc.bOrigin);
    f32(sc.armTarget[0]); f32(sc.armTarget[1]);
    u32((uint32_t)sc.sk.parent.size());
    for (int p : sc.sk.parent) i32(p);
    for (const V3& b : sc.sk.bindPos) { f32(b.x); f32(b.y); f32(b.z); }
    auto mesh = [&](const Mesh& m) {
        u32((uint32_t)m.v.size()); u32((uint32_t)m.t.size()); f32(m.orient);
        for (const Vert& v : m.v) {
            f32(v.p.x); f32(v.p.y); f32(v.p.z);
            q8(v.n.x); q8(v.n.y); q8(v.n.z);
            u8((uint8_t)v.nl);
            i16((int16_t)lroundf(clampf(v.u, -15.9f, 15.9f) * 2048.0f)); i16((int16_t)lroundf(clampf(v.v, -15.9f, 15.9f) * 2048.0f));
            for (int k = 0; k < 3; k++) u8((uint8_t)(k < v.nl ? v.bone[k] : 0));
            for (int k = 0; k < 3; k++) u8((uint8_t)(k < v.nl ? lroundf(v.w[k] * 255.0f) : 0));
        }
        for (const Tri& t : m.t) { u16((uint16_t)t.i[0]); u16((uint16_t)t.i[1]); u16((uint16_t)t.i[2]); u8((uint8_t)t.mat); }
    };
    mesh(sc.bulb); mesh(sc.laptop); mesh(sc.pika);
    const Texture* tex[13] = {&sc.bulbTex[0], &sc.bulbTex[1], &sc.bulbTex[2], &sc.bulbTex[3], &sc.laptopKeys, &sc.screenWork, &sc.screenBreak,
                              &sc.wallTex, &sc.floorTex, &sc.pikaTex[0], &sc.pikaTex[1], &sc.pikaTex[2], &sc.pikaTex[3]};
    for (const Texture* t : tex) { u32((uint32_t)t->w); u32((uint32_t)t->h); put(t->px.data(), t->px.size() * 2); }
    FILE* f = fopen(path, "wb");
    if (!f) return false;
    fwrite(out.data(), 1, out.size(), f);
    fclose(f);
    printf("wrote %s (%.1f KB)\n", path, out.size() / 1024.0);
    return true;
}
#endif
