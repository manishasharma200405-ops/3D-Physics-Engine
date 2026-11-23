#pragma once
// Sequential-impulse contact solver (projected Gauss-Seidel) with Coulomb friction,
// restitution, penetration correction and warm starting.
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "Collision/Collision.h"

namespace eng {

struct SolverSettings {
    int iterations = 20;
    float baumgarte = 0.2f;            // fraction of penetration corrected per step
    float penetrationSlop = 0.005f;    // allowed overlap
    float maxCorrectionSpeed = 4.f;    // cap on the position-correction velocity
    float restitutionThreshold = 1.f;  // slower impacts do not bounce
    bool warmStart = true;
};

class ConstraintSolver {
public:
    SolverSettings settings;

    // Modifies body velocities in place.  Manifold normals must point a -> b.
    void solve(std::vector<RigidBody>& bodies, const std::vector<Manifold>& manifolds, float dt);

    void clearCache() { cache_.clear(); }

private:
    struct WorkContact {
        Vec3 point, ra, rb;
        float normalMass, tangentMass1, tangentMass2;
        float vsMin;  // minimum allowed separating speed along the normal
        float accN = 0.f, accT1 = 0.f, accT2 = 0.f;
    };
    struct WorkManifold {
        int a, b;
        Vec3 n, t1, t2;
        float friction;
        int count;
        WorkContact c[4];
    };
    struct CachedContact {
        Vec3 point;
        float accN;
        Vec3 tangentImpulse;  // world space, so it survives a tangent-basis change
    };
    struct CachedManifold {
        int count = 0;
        CachedContact c[4];
    };

    std::vector<WorkManifold> work_;
    std::unordered_map<uint64_t, CachedManifold> cache_;
};

}  // namespace eng
