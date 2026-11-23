#include "Render/Renderer.h"
#include <algorithm>
#include <cmath>
#include <cstring>

static const int   CELL   = 4;        // water field cell = 4x4 pixels
static const float KR     = 1.25f;    // splat radius in world units
static const float ISO    = 1.5f;     // field value where the water surface sits
static const float DEEP   = 6.0f;     // field value that counts as "deep water"
static const int   WALL_D = 36;       // how far back the wall is (pixels): it shifts up and right by this much
static const int   SLAB_D = 12;       // how thick the water slab is (pixels), a multiple of CELL
static const int   SHADOW_X = 12, SHADOW_Y = 12;   // the light comes from the upper left, shadows fall right and down

// light direction, and the half-vector for the shiny highlight (viewer looks straight at the screen)
static const float Lx = -0.45f, Ly = -0.55f, Lz = 0.70f;
static const float Hx = -0.244f, Hy = -0.298f, Hz = 0.923f;

static inline float clamp01(float v) { return v < 0.f ? 0.f : (v > 1.f ? 1.f : v); }
static inline float mix(float a, float b, float t) { return a + (b - a) * t; }
static inline unsigned int chan(float v) { return v <= 0.f ? 0u : (v >= 255.f ? 255u : (unsigned int)(v + 0.5f)); }
static inline unsigned int pack(float r, float g, float b) { return (chan(r) << 16) | (chan(g) << 8) | chan(b); }

static inline void blend(unsigned int* p, float r, float g, float b, float a) {
    unsigned int bg = *p;
    float br = (float)((bg >> 16) & 255), bgc = (float)((bg >> 8) & 255), bb = (float)(bg & 255);
    *p = pack(mix(br, r, a), mix(bgc, g, a), mix(bb, b, a));
}

// a lit sphere (a == b) or a lit rod with round ends (a != b), hw = half width, all in pixels
static void capsule(unsigned int* pix, int pw, int ph, float ax, float ay, float bx, float by,
                    float hw, float cr, float cg, float cb) {
    int x0 = std::max(0, (int)std::floor(std::min(ax, bx) - hw - 1)), x1 = std::min(pw - 1, (int)std::ceil(std::max(ax, bx) + hw + 1));
    int y0 = std::max(0, (int)std::floor(std::min(ay, by) - hw - 1)), y1 = std::min(ph - 1, (int)std::ceil(std::max(ay, by) + hw + 1));
    float vx = bx - ax, vy = by - ay, len2 = vx * vx + vy * vy + 1e-6f;
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++) {
            float px = x + 0.5f - ax, py = y + 0.5f - ay;
            float t = clamp01((px * vx + py * vy) / len2);
            float qx = px - vx * t, qy = py - vy * t;                 // from the axis to this pixel
            float d = std::sqrt(qx * qx + qy * qy);
            float cov = clamp01(hw - d + 0.5f);                       // soft 1-pixel edge
            if (cov <= 0.f) continue;
            float u = std::min(1.f, d / hw);                          // 0 on the axis, 1 at the edge
            float nx = d > 1e-4f ? qx / d * u : 0.f, ny = d > 1e-4f ? qy / d * u : 0.f;
            float nz = std::sqrt(std::max(0.f, 1.f - u * u));        // surface normal of a sphere / cylinder
            float diff = std::max(0.f, nx * Lx + ny * Ly + nz * Lz);
            float hd = std::max(0.f, nx * Hx + ny * Hy + nz * Hz);
            float sp = hd * hd; sp *= sp; sp *= sp; sp *= sp; sp *= sp;  // hd^32: a small sharp highlight
            float shade = 0.30f + 0.70f * diff;
            blend(pix + y * pw + x, cr * shade + 255.f * sp * 0.6f, cg * shade + 255.f * sp * 0.6f,
                  cb * shade + 255.f * sp * 0.6f, cov);
        }
}

// a soft dark patch (the shadow of a sphere or rod on the back wall)
static void shadowCapsule(unsigned int* pix, int pw, int ph, float ax, float ay, float bx, float by,
                          float hw, float soft, float alpha) {
    float reach = hw + soft + 1.f;
    int x0 = std::max(0, (int)std::floor(std::min(ax, bx) - reach)), x1 = std::min(pw - 1, (int)std::ceil(std::max(ax, bx) + reach));
    int y0 = std::max(0, (int)std::floor(std::min(ay, by) - reach)), y1 = std::min(ph - 1, (int)std::ceil(std::max(ay, by) + reach));
    float vx = bx - ax, vy = by - ay, len2 = vx * vx + vy * vy + 1e-6f;
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++) {
            float px = x + 0.5f - ax, py = y + 0.5f - ay;
            float t = clamp01((px * vx + py * vy) / len2);
            float qx = px - vx * t, qy = py - vy * t;
            float d = std::sqrt(qx * qx + qy * qy);
            if (d >= hw + soft) continue;
            float s = clamp01((d - hw * 0.5f) / (hw * 0.5f + soft));
            blend(pix + y * pw + x, 0.f, 0.f, 0.f, alpha * (1.f - s) * (1.f - s));
        }
}

Renderer::Renderer(int w, int h, float s) : pw(w), ph(h), scale(s) {
    fw = w / CELL + 2;
    fh = h / CELL + 2;
    dens.assign(fw * fh, 0.f);
    spd.assign(fw * fh, 0.f);
    ext.assign(fw * fh, 0.f);
    buildWall();
}

// ---------- the room ----------
void Renderer::buildWall() {
    wall.assign(pw * ph, 0);
    for (int y = 0; y < ph; y++)
        for (int x = 0; x < pw; x++) {
            float r, g, b;
            bool left = (x < WALL_D && x + y <= ph);
            bool floorPart = !left && (y > ph - WALL_D);
            if (left) {
                float t = (float)x / WALL_D;                          // 0 at the front edge, 1 at the back
                r = mix(12.f, 24.f, t); g = mix(15.f, 29.f, t); b = mix(28.f, 50.f, t);
                if (((y + x) % 48) < 1) { r += 8.f; g += 8.f; b += 10.f; }
            } else if (floorPart) {
                float t = (float)(ph - y) / WALL_D;                   // 0 at the front edge, 1 at the back
                r = mix(62.f, 40.f, t); g = mix(68.f, 46.f, t); b = mix(98.f, 74.f, t);
                int x0 = x - (ph - y);
                if (((x0 % 48) + 48) % 48 < 1) { r += 10.f; g += 10.f; b += 14.f; }
                if (((ph - y) % 12) < 1)       { r += 6.f;  g += 6.f;  b += 8.f; }
            } else {
                float t = (float)y / ph;
                r = mix(36.f, 22.f, t); g = mix(43.f, 27.f, t); b = mix(70.f, 46.f, t);
                int bx = x - WALL_D, by = y + WALL_D;                 // position on the wall itself
                if (((bx % 48) + 48) % 48 < 1 || (by % 48) < 1) { r += 12.f; g += 12.f; b += 16.f; }
                if (std::abs(x - WALL_D) < 2 || std::abs(y - (ph - WALL_D)) < 2) { r += 14.f; g += 14.f; b += 18.f; }
            }
            float cx = (x - pw * 0.5f) / (pw * 0.5f), cy = (y - ph * 0.5f) / (ph * 0.5f);
            float vig = 1.f - 0.30f * (cx * cx + cy * cy) * 0.5f;     // darker toward the corners
            wall[y * pw + x] = pack(r * vig, g * vig, b * vig);
        }
}

// ---------- water fields ----------
void Renderer::fillFields(const EntityManager& em) {
    std::fill(dens.begin(), dens.end(), 0.f);
    std::fill(spd.begin(), spd.end(), 0.f);
    const float rk = KR * scale / CELL;                       // splat radius in field cells
    const float inv = 1.f / (rk * rk);
    int n = em.count();
    for (int i = 0; i < n; i++) {
        if (em.kind[i] != WATER) continue;
        float fx = em.pos[i].x * scale / CELL;
        float fy = em.pos[i].y * scale / CELL;
        float speed = length(em.vel[i]);
        int x0 = std::max(0, (int)std::floor(fx - rk)), x1 = std::min(fw - 1, (int)std::ceil(fx + rk));
        int y0 = std::max(0, (int)std::floor(fy - rk)), y1 = std::min(fh - 1, (int)std::ceil(fy + rk));
        for (int y = y0; y <= y1; y++)
            for (int x = x0; x <= x1; x++) {
                float dx = x - fx, dy = y - fy;
                float q = 1.f - (dx * dx + dy * dy) * inv;
                if (q <= 0.f) continue;
                float w = q * q;
                dens[y * fw + x] += w;
                spd[y * fw + x]  += w * speed;
            }
    }
    // the slab: the same water drawn again further back (shifted up and right), joined to the front
    const int K = SLAB_D / CELL;
    for (int y = 0; y < fh; y++)
        for (int x = 0; x < fw; x++) {
            float m = 0.f;
            for (int k = 0; k <= K; k++) {
                int sx = x - k, sy = y + k;
                if (sx >= 0 && sy < fh) m = std::max(m, dens[sy * fw + sx]);
            }
            ext[y * fw + x] = m;
        }
}

// visits every pixel where the field f could be above the surface, smoothly interpolated.
// (offCx, offCy) moves the picture by whole cells. fn(X, Y, d, gx, gy, cx, cy, u, v)
template <class F>
void Renderer::scan(const std::vector<float>& f, float minCorner, int offCx, int offCy, F fn) const {
    for (int cy = 0; cy < fh - 1; cy++)
        for (int cx = 0; cx < fw - 1; cx++) {
            float d00 = f[cy * fw + cx],       d10 = f[cy * fw + cx + 1];
            float d01 = f[(cy + 1) * fw + cx], d11 = f[(cy + 1) * fw + cx + 1];
            if (std::max(std::max(d00, d10), std::max(d01, d11)) < minCorner) continue;
            float gx = (d10 + d11 - d00 - d01) * 0.5f;
            float gy = (d01 + d11 - d00 - d10) * 0.5f;
            for (int py = 0; py < CELL; py++) {
                int Y = (cy + offCy) * CELL + py;
                if (Y < 0 || Y >= ph) continue;
                float v = (py + 0.5f) / CELL;
                for (int px = 0; px < CELL; px++) {
                    int X = (cx + offCx) * CELL + px;
                    if (X < 0 || X >= pw) continue;
                    float u = (px + 0.5f) / CELL;
                    float d = mix(mix(d00, d10, u), mix(d01, d11, u), v);
                    fn(X, Y, d, gx, gy, cx, cy, u, v);
                }
            }
        }
}

// ---------- layers ----------
void Renderer::shadows(const EntityManager& em, const Joints& jt, unsigned int* pix) const {
    const float ox = (float)(WALL_D + SHADOW_X), oy = (float)(-WALL_D + SHADOW_Y);
    auto at = [&](int i) { return em.pos[i] * scale + Vec2(ox, oy); };
    for (const auto& j : jt.rods) {
        Vec2 a = at(j.a), b = at(j.b);
        shadowCapsule(pix, pw, ph, a.x, a.y, b.x, b.y, j.oneSided ? 1.5f : 3.f, 6.f, 0.30f);
    }
    for (const auto& p : jt.pulleys) {
        Vec2 c = at(p.c), a = at(p.a), b = at(p.b);
        shadowCapsule(pix, pw, ph, a.x, a.y, c.x, c.y, 1.5f, 6.f, 0.30f);
        shadowCapsule(pix, pw, ph, c.x, c.y, b.x, b.y, 1.5f, 6.f, 0.30f);
    }
    for (int i = 0; i < em.count(); i++) {
        if (em.kind[i] == WATER) continue;
        Vec2 p = at(i);
        float hw = em.radius[i] > 0.5f ? em.radius[i] * scale : (em.kind[i] == KINEMATIC ? em.radius[i] * scale : 5.f);
        shadowCapsule(pix, pw, ph, p.x, p.y, p.x, p.y, hw, 7.f, 0.38f);
    }
}

void Renderer::waterShadow(unsigned int* pix) const {
    scan(dens, ISO * 0.4f, (WALL_D + SHADOW_X) / CELL, (-WALL_D + SHADOW_Y) / CELL,
         [&](int X, int Y, float d, float, float, int, int, float, float) {
             float a = 0.30f * clamp01((d - ISO * 0.5f) / ISO);
             if (a > 0.f) blend(pix + Y * pw + X, 0.f, 0.f, 0.f, a);
         });
}

void Renderer::waterBack(unsigned int* pix) const {
    scan(ext, ISO * 0.5f, 0, 0,
         [&](int X, int Y, float d, float gx, float gy, int, int, float, float) {
             float a = clamp01((d - (ISO - 0.3f)) / 0.6f);
             if (a <= 0.f) return;
             float gl = std::sqrt(gx * gx + gy * gy) + 1e-4f;
             float edge = 1.f - clamp01((d - ISO) / 1.5f);             // only faces near the surface are lit
             float up = std::max(0.f, gy / gl) * edge;                 // the face looks upward: catches the light
             blend(pix + Y * pw + X, 20.f + 40.f * up, 80.f + 70.f * up, 150.f + 70.f * up, a * 0.95f);
         });
}

void Renderer::solids(const EntityManager& em, const Joints& jt, unsigned int* pix) const {
    for (const auto& j : jt.rods) {
        Vec2 a = em.pos[j.a] * scale, b = em.pos[j.b] * scale;
        if (j.oneSided) capsule(pix, pw, ph, a.x, a.y, b.x, b.y, 1.8f, 235.f, 200.f, 90.f);
        else            capsule(pix, pw, ph, a.x, a.y, b.x, b.y, 3.0f, 205.f, 210.f, 228.f);
    }
    for (const auto& p : jt.pulleys) {
        Vec2 c = em.pos[p.c] * scale, a = em.pos[p.a] * scale, b = em.pos[p.b] * scale;
        capsule(pix, pw, ph, a.x, a.y, c.x, c.y, 1.8f, 235.f, 200.f, 90.f);
        capsule(pix, pw, ph, c.x, c.y, b.x, b.y, 1.8f, 235.f, 200.f, 90.f);
    }
    for (int i = 0; i < em.count(); i++) {
        if (em.kind[i] == WATER) continue;
        Vec2 p = em.pos[i] * scale;
        if (em.kind[i] == KINEMATIC) {
            capsule(pix, pw, ph, p.x, p.y, p.x, p.y, em.radius[i] * scale, 120.f, 130.f, 158.f);
        } else if (em.radius[i] > 0.5f) {                             // a ball: gold if light, steel if heavy
            float mass = em.restInv[i] > 0.f ? 1.f / em.restInv[i] : 100.f;
            if (mass < 15.f) capsule(pix, pw, ph, p.x, p.y, p.x, p.y, em.radius[i] * scale, 240.f, 190.f, 70.f);
            else             capsule(pix, pw, ph, p.x, p.y, p.x, p.y, em.radius[i] * scale, 135.f, 145.f, 170.f);
        } else if (em.invMass[i] == 0.f) {
            capsule(pix, pw, ph, p.x, p.y, p.x, p.y, 6.6f, 170.f, 172.f, 188.f);   // fixed point
        } else {
            capsule(pix, pw, ph, p.x, p.y, p.x, p.y, 5.0f, 255.f, 140.f, 60.f);    // movable bead
        }
    }
}

void Renderer::waterFront(unsigned int* pix) const {
    scan(dens, ISO * 0.5f, 0, 0,
         [&](int X, int Y, float d, float gx, float gy, int cx, int cy, float u, float v) {
             float a = clamp01((d - (ISO - 0.3f)) / 0.6f);             // soft edge
             if (a <= 0.f) return;
             float s00 = spd[cy * fw + cx],       s10 = spd[cy * fw + cx + 1];
             float s01 = spd[(cy + 1) * fw + cx], s11 = spd[(cy + 1) * fw + cx + 1];
             float sp = mix(mix(s00, s10, u), mix(s01, s11, u), v) / (d + 1e-4f);

             float gl = std::sqrt(gx * gx + gy * gy) + 1e-4f;
             float light = std::max(0.f, (0.6f * gx + 0.8f * gy) / gl);  // light from the upper left
             light = light * light * light;

             float depth = clamp01((d - ISO) / (DEEP - ISO));
             float rim   = 1.f - clamp01((d - ISO) / 1.5f);              // 1 at the surface, 0 deep inside

             float r = mix(80.f, 14.f, depth);
             float g = mix(190.f, 72.f, depth);
             float b = mix(240.f, 165.f, depth);

             float spec = light * rim * 0.9f;                            // shiny edge facing the light
             r += 255.f * spec * 0.55f; g += 255.f * spec * 0.55f; b += 255.f * spec * 0.45f;

             float foam = clamp01((sp - 10.f) / 25.f) * (0.35f + 0.65f * rim);   // fast water turns white
             r = mix(r, 238.f, foam * 0.6f);
             g = mix(g, 247.f, foam * 0.6f);
             b = mix(b, 255.f, foam * 0.6f);

             // see-through: things under the water show through, more so deeper down
             blend(pix + Y * pw + X, r, g, b, a * mix(0.90f, 0.50f, depth));
         });
}

void Renderer::draw(const EntityManager& em, const Joints& jt, unsigned int* pix) {
    std::memcpy(pix, wall.data(), sizeof(unsigned int) * pw * ph);   // the room
    fillFields(em);
    shadows(em, jt, pix);                                            // objects' shadows on the back wall
    waterShadow(pix);                                                // the pool's shadow
    waterBack(pix);                                                  // the water slab's top and side faces
    solids(em, jt, pix);                                             // balls, beads, rods
    waterFront(pix);                                                 // the front of the water, over the solids
}