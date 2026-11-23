#include "Solver/ConstraintSolver.h"

#include <algorithm>
#include <cmath>

namespace eng {

namespace {

// Applies impulse P at B (and -P at A), at offsets rb and ra from their centres of mass.
inline void applyImpulse(RigidBody& A, RigidBody& B, const Vec3& ra, const Vec3& rb, const Vec3& P) {
    A.linearVelocity -= P * A.invMass;
    A.angularVelocity -= A.invInertiaWorld * cross(ra, P);
    B.linearVelocity += P * B.invMass;
    B.angularVelocity += B.invInertiaWorld * cross(rb, P);
}

inline float effectiveMass(const RigidBody& A, const RigidBody& B, const Vec3& ra, const Vec3& rb, const Vec3& dir) {
    Vec3 raxd = cross(ra, dir), rbxd = cross(rb, dir);
    float k = A.invMass + B.invMass +
              dot(cross(A.invInertiaWorld * raxd, ra), dir) +
              dot(cross(B.invInertiaWorld * rbxd, rb), dir);
    return k > 1e-12f ? 1.f / k : 0.f;
}

inline void tangentBasis(const Vec3& n, Vec3& t1, Vec3& t2) {
    if (std::fabs(n.x) >= 0.57735f) t1 = normalize(Vec3(n.y, -n.x, 0.f));
    else t1 = normalize(Vec3(0.f, n.z, -n.y));
    t2 = cross(n, t1);
}

inline uint64_t pairKey(int a, int b) {
    return ((uint64_t)(uint32_t)a << 32) | (uint32_t)b;
}

}  // namespace

void ConstraintSolver::solve(std::vector<RigidBody>& bodies, const std::vector<Manifold>& manifolds, float dt) {
    const SolverSettings& S = settings;
    work_.clear();
    work_.reserve(manifolds.size());

    // ---- prepare ----
    for (const Manifold& m : manifolds) {
        RigidBody& A = bodies[m.a];
        RigidBody& B = bodies[m.b];

        WorkManifold w;
        w.a = m.a;
        w.b = m.b;
        w.n = m.normal;
        w.friction = m.friction;
        w.count = m.count;
        tangentBasis(w.n, w.t1, w.t2);

        const CachedManifold* prev = nullptr;
        if (S.warmStart) {
            auto it = cache_.find(pairKey(m.a, m.b));
            if (it != cache_.end()) prev = &it->second;
        }

        for (int i = 0; i < m.count; i++) {
            WorkContact& c = w.c[i];
            c.point = m.contacts[i].point;
            c.ra = c.point - A.position;
            c.rb = c.point - B.position;
            c.normalMass = effectiveMass(A, B, c.ra, c.rb, w.n);
            c.tangentMass1 = effectiveMass(A, B, c.ra, c.rb, w.t1);
            c.tangentMass2 = effectiveMass(A, B, c.ra, c.rb, w.t2);

            float depth = m.contacts[i].depth;
            float vs0 = dot(B.velocityAt(c.rb) - A.velocityAt(c.ra), w.n);
            if (depth < 0.f) {
                // Speculative contact: allow closing the gap, but not more...
                c.vsMin = depth / dt;
                // ...unless a fast impact will reach the surface this step: then it bounces.
                if (vs0 < -S.restitutionThreshold && -vs0 * dt >= -depth)
                    c.vsMin = -m.restitution * vs0;
            } else {
                c.vsMin = std::min(S.baumgarte * std::max(depth - S.penetrationSlop, 0.f) / dt, S.maxCorrectionSpeed);
                if (vs0 < -S.restitutionThreshold) c.vsMin = std::max(c.vsMin, -m.restitution * vs0);
            }

            // Warm start from the nearest contact of the previous step.
            if (prev) {
                float bestD = 0.05f * 0.05f;
                const CachedContact* match = nullptr;
                for (int k = 0; k < prev->count; k++) {
                    float d = (prev->c[k].point - c.point).lengthSq();
                    if (d < bestD) { bestD = d; match = &prev->c[k]; }
                }
                if (match) {
                    c.accN = match->accN;
                    c.accT1 = dot(match->tangentImpulse, w.t1);
                    c.accT2 = dot(match->tangentImpulse, w.t2);
                    Vec3 P = w.n * c.accN + w.t1 * c.accT1 + w.t2 * c.accT2;
                    applyImpulse(A, B, c.ra, c.rb, P);
                }
            }
        }
        work_.push_back(w);
    }

    // ---- iterate ----
    for (int it = 0; it < S.iterations; it++) {
        for (WorkManifold& w : work_) {
            RigidBody& A = bodies[w.a];
            RigidBody& B = bodies[w.b];

            for (int i = 0; i < w.count; i++) {
                WorkContact& c = w.c[i];

                // Non-penetration.
                Vec3 dv = B.velocityAt(c.rb) - A.velocityAt(c.ra);
                float vs = dot(dv, w.n);
                float lambda = (c.vsMin - vs) * c.normalMass;
                float newAcc = std::max(c.accN + lambda, 0.f);
                lambda = newAcc - c.accN;
                c.accN = newAcc;
                applyImpulse(A, B, c.ra, c.rb, w.n * lambda);

                // Friction (pyramid), limited by the accumulated normal impulse.
                float maxF = w.friction * c.accN;

                dv = B.velocityAt(c.rb) - A.velocityAt(c.ra);
                float l1 = -dot(dv, w.t1) * c.tangentMass1;
                float n1 = std::max(-maxF, std::min(maxF, c.accT1 + l1));
                l1 = n1 - c.accT1;
                c.accT1 = n1;
                applyImpulse(A, B, c.ra, c.rb, w.t1 * l1);

                dv = B.velocityAt(c.rb) - A.velocityAt(c.ra);
                float l2 = -dot(dv, w.t2) * c.tangentMass2;
                float n2 = std::max(-maxF, std::min(maxF, c.accT2 + l2));
                l2 = n2 - c.accT2;
                c.accT2 = n2;
                applyImpulse(A, B, c.ra, c.rb, w.t2 * l2);
            }
        }
    }

    // ---- store for next step ----
    cache_.clear();
    for (const WorkManifold& w : work_) {
        CachedManifold cm;
        cm.count = w.count;
        for (int i = 0; i < w.count; i++) {
            cm.c[i].point = w.c[i].point;
            cm.c[i].accN = w.c[i].accN;
            cm.c[i].tangentImpulse = w.t1 * w.c[i].accT1 + w.t2 * w.c[i].accT2;
        }
        cache_[pairKey(w.a, w.b)] = cm;
    }
}

}  // namespace eng
