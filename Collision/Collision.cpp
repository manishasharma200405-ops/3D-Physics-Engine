#include "Collision/Collision.h"

#include <cmath>

namespace eng {

namespace {

void addContact(Manifold& m, const Vec3& p, float depth) {
    if (m.count < 8) m.contacts[m.count++] = {p, depth};
}

// Keep at most 4 contacts that span the largest area.
void reduceContacts(Manifold& m) {
    if (m.count <= 4) return;

    int i0 = 0;
    for (int i = 1; i < m.count; i++)
        if (m.contacts[i].depth > m.contacts[i0].depth) i0 = i;

    int i1 = -1;
    float best = -1.f;
    for (int i = 0; i < m.count; i++) {
        if (i == i0) continue;
        float d = (m.contacts[i].point - m.contacts[i0].point).lengthSq();
        if (d > best) { best = d; i1 = i; }
    }

    int i2 = -1;
    best = -1.f;
    Vec3 edge = m.contacts[i1].point - m.contacts[i0].point;
    for (int i = 0; i < m.count; i++) {
        if (i == i0 || i == i1) continue;
        float d = cross(edge, m.contacts[i].point - m.contacts[i0].point).lengthSq();
        if (d > best) { best = d; i2 = i; }
    }

    int i3 = -1;
    best = -1.f;
    for (int i = 0; i < m.count; i++) {
        if (i == i0 || i == i1 || i == i2) continue;
        float d = std::min({(m.contacts[i].point - m.contacts[i0].point).lengthSq(),
                            (m.contacts[i].point - m.contacts[i1].point).lengthSq(),
                            (m.contacts[i].point - m.contacts[i2].point).lengthSq()});
        if (d > best) { best = d; i3 = i; }
    }

    Contact keep[4] = {m.contacts[i0], m.contacts[i1], m.contacts[i2], m.contacts[i3]};
    for (int i = 0; i < 4; i++) m.contacts[i] = keep[i];
    m.count = 4;
}

// ---- sphere / sphere -------------------------------------------------------
bool sphereSphere(const RigidBody& A, const RigidBody& B, Manifold& m) {
    Vec3 d = B.position - A.position;
    float rsum = A.shape.radius + B.shape.radius;
    float lim = rsum + kContactMargin;
    float d2 = d.lengthSq();
    if (d2 > lim * lim) return false;

    float dist = std::sqrt(d2);
    Vec3 n = dist > 1e-6f ? d / dist : Vec3(0.f, 1.f, 0.f);
    float depth = rsum - dist;
    m.normal = n;
    addContact(m, A.position + n * (A.shape.radius - depth * 0.5f), depth);
    return true;
}

// ---- sphere / plane --------------------------------------------------------
bool spherePlane(const RigidBody& S, const RigidBody& P, Manifold& m) {
    const Vec3& pn = P.shape.normal;
    float r = S.shape.radius;
    float d = dot(pn, S.position) - P.shape.offset;
    if (d > r + kContactMargin) return false;

    m.normal = -pn;  // from sphere (above) down into the plane
    addContact(m, S.position - pn * ((d + r) * 0.5f), r - d);
    return true;
}

// ---- sphere / box ----------------------------------------------------------
bool sphereBox(const RigidBody& S, const RigidBody& B, Manifold& m) {
    const Vec3& h = B.shape.halfExtents;
    float r = S.shape.radius;

    Vec3 lc = B.rotation.transposed() * (S.position - B.position);  // sphere centre in box space
    Vec3 closest(std::max(-h.x, std::min(h.x, lc.x)),
                 std::max(-h.y, std::min(h.y, lc.y)),
                 std::max(-h.z, std::min(h.z, lc.z)));
    Vec3 diff = lc - closest;
    float d2 = diff.lengthSq();
    float lim = r + kContactMargin;
    if (d2 > lim * lim) return false;

    Vec3 nLocal;  // from box towards sphere
    float depth;
    if (d2 > 1e-10f) {
        float dist = std::sqrt(d2);
        nLocal = diff / dist;
        depth = r - dist;
    } else {
        // Centre inside the box: push out through the nearest face.
        int axis = 0;
        float minGap = h.x - std::fabs(lc.x);
        for (int i = 1; i < 3; i++) {
            float gap = h[i] - std::fabs(lc[i]);
            if (gap < minGap) { minGap = gap; axis = i; }
        }
        float sgn = lc[axis] >= 0.f ? 1.f : -1.f;
        nLocal = Vec3();
        nLocal[axis] = sgn;
        closest = lc;
        closest[axis] = sgn * h[axis];
        depth = r + minGap;
    }

    m.normal = -(B.rotation * nLocal);  // A (sphere) -> B (box)
    addContact(m, B.position + B.rotation * closest, depth);
    return true;
}

// ---- box / plane -----------------------------------------------------------
bool boxPlane(const RigidBody& Bx, const RigidBody& P, Manifold& m) {
    const Vec3& pn = P.shape.normal;
    const Vec3& h = Bx.shape.halfExtents;
    m.normal = -pn;

    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2)
            for (int sz = -1; sz <= 1; sz += 2) {
                Vec3 v = Bx.position + Bx.rotation * Vec3(sx * h.x, sy * h.y, sz * h.z);
                float d = dot(pn, v) - P.shape.offset;
                if (d < kContactMargin) addContact(m, v - pn * (d * 0.5f), -d);
            }
    return m.count > 0;
}

// ---- box / box (SAT + face clipping) --------------------------------------
int clipPolygon(const Vec3* in, int n, const Vec3& pn, float off, Vec3* out) {
    int c = 0;
    for (int i = 0; i < n; i++) {
        const Vec3& a = in[i];
        const Vec3& b = in[(i + 1) % n];
        float da = dot(a, pn) - off;
        float db = dot(b, pn) - off;
        if (da <= 0.f) out[c++] = a;
        if ((da < 0.f && db > 0.f) || (da > 0.f && db < 0.f)) {
            float t = da / (da - db);
            out[c++] = a + (b - a) * t;
        }
    }
    return c;
}

bool boxBox(const RigidBody& A, const RigidBody& B, Manifold& m) {
    Vec3 ua[3], ub[3];
    for (int i = 0; i < 3; i++) { ua[i] = A.rotation.col(i); ub[i] = B.rotation.col(i); }
    const Vec3& ha = A.shape.halfExtents;
    const Vec3& hb = B.shape.halfExtents;
    Vec3 t = B.position - A.position;

    float best = 1e30f;
    int bestAxis = -1;
    Vec3 bestN;

    // Returns false when the axis separates the boxes.
    auto test = [&](Vec3 axis, int idx, bool isEdge) -> bool {
        float len = axis.length();
        if (len < 1e-5f) return true;  // parallel edges: no information
        axis = axis / len;
        float ra = ha.x * std::fabs(dot(ua[0], axis)) + ha.y * std::fabs(dot(ua[1], axis)) + ha.z * std::fabs(dot(ua[2], axis));
        float rb = hb.x * std::fabs(dot(ub[0], axis)) + hb.y * std::fabs(dot(ub[1], axis)) + hb.z * std::fabs(dot(ub[2], axis));
        float overlap = ra + rb - std::fabs(dot(t, axis));
        if (overlap < -kContactMargin) return false;
        // Prefer face axes: an edge axis must be clearly better to win.
        if (overlap < best - (isEdge ? 0.01f : 0.f)) {
            best = overlap;
            bestAxis = idx;
            bestN = dot(axis, t) < 0.f ? -axis : axis;  // point A -> B
        }
        return true;
    };

    for (int i = 0; i < 3; i++) if (!test(ua[i], i, false)) return false;
    for (int i = 0; i < 3; i++) if (!test(ub[i], 3 + i, false)) return false;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            if (!test(cross(ua[i], ub[j]), 6 + i * 3 + j, true)) return false;
    if (bestAxis < 0) return false;

    m.normal = bestN;

    if (bestAxis >= 6) {
        // Edge-edge: closest points between the two supporting edges.
        int ei = (bestAxis - 6) / 3, ej = (bestAxis - 6) % 3;
        Vec3 pa = A.position, pb = B.position;
        for (int k = 0; k < 3; k++) {
            if (k != ei) pa += ua[k] * (dot(ua[k], bestN) > 0.f ? ha[k] : -ha[k]);
            if (k != ej) pb += ub[k] * (dot(ub[k], bestN) > 0.f ? -hb[k] : hb[k]);
        }
        Vec3 d1 = ua[ei], d2 = ub[ej], r = pa - pb;
        float b = dot(d1, d2), c = dot(d1, r), f = dot(d2, r);
        float denom = 1.f - b * b;
        Vec3 p;
        if (denom > 1e-8f) {
            float s = (b * f - c) / denom;
            float u = b * s + f;
            p = ((pa + d1 * s) + (pb + d2 * u)) * 0.5f;
        } else {
            p = (pa + pb) * 0.5f;
        }
        addContact(m, p, best);
        return true;
    }

    // Face contact: clip the incident face against the reference face's side planes.
    bool refIsA = bestAxis < 3;
    int k = refIsA ? bestAxis : bestAxis - 3;
    const RigidBody& R = refIsA ? A : B;
    const RigidBody& I = refIsA ? B : A;
    const Vec3* ur = refIsA ? ua : ub;
    const Vec3* ui = refIsA ? ub : ua;
    const Vec3& hr = refIsA ? ha : hb;
    const Vec3& hi = refIsA ? hb : ha;
    Vec3 nr = refIsA ? bestN : -bestN;  // outward normal of the reference face

    int ii = 0;
    float maxd = -1.f;
    for (int i = 0; i < 3; i++) {
        float d = std::fabs(dot(ui[i], nr));
        if (d > maxd) { maxd = d; ii = i; }
    }
    Vec3 ni = ui[ii] * (dot(ui[ii], nr) > 0.f ? -1.f : 1.f);  // incident face normal (opposes nr)
    Vec3 ci = I.position + ni * hi[ii];
    int j = (ii + 1) % 3, l = (ii + 2) % 3;
    Vec3 ej = ui[j] * hi[j], el = ui[l] * hi[l];

    Vec3 bufA[16], bufB[16];
    bufA[0] = ci + ej + el;
    bufA[1] = ci - ej + el;
    bufA[2] = ci - ej - el;
    bufA[3] = ci + ej - el;
    int n = 4;

    Vec3* src = bufA;
    Vec3* dst = bufB;
    for (int a = 1; a <= 2; a++) {
        int axis = (k + a) % 3;
        for (int s = -1; s <= 1; s += 2) {
            Vec3 pn = ur[axis] * (float)s;
            float off = dot(R.position, pn) + hr[axis];
            n = clipPolygon(src, n, pn, off, dst);
            std::swap(src, dst);
            if (n == 0) return false;
        }
    }

    Vec3 cr = R.position + nr * hr[k];
    for (int i = 0; i < n; i++) {
        float sep = dot(src[i] - cr, nr);
        if (sep <= kContactMargin) addContact(m, src[i] - nr * (sep * 0.5f), -sep);
    }
    return m.count > 0;
}

}  // namespace

bool collide(const RigidBody& A, const RigidBody& B, Manifold& out) {
    out.count = 0;
    out.friction = std::sqrt(A.friction * B.friction);
    out.restitution = std::max(A.restitution, B.restitution);

    const RigidBody* a = &A;
    const RigidBody* b = &B;
    bool swapped = false;
    if (a->shape.type > b->shape.type) {
        std::swap(a, b);
        swapped = true;
    }

    bool hit = false;
    switch (a->shape.type) {
        case ShapeType::Sphere:
            if (b->shape.type == ShapeType::Sphere) hit = sphereSphere(*a, *b, out);
            else if (b->shape.type == ShapeType::Box) hit = sphereBox(*a, *b, out);
            else hit = spherePlane(*a, *b, out);
            break;
        case ShapeType::Box:
            if (b->shape.type == ShapeType::Box) hit = boxBox(*a, *b, out);
            else if (b->shape.type == ShapeType::Plane) hit = boxPlane(*a, *b, out);
            break;
        default:
            break;  // plane vs plane: never collides
    }

    if (!hit || out.count == 0) return false;
    if (swapped) out.normal = -out.normal;
    reduceContacts(out);
    return true;
}

}  // namespace eng
