#pragma once
// Semi-implicit (symplectic) Euler integration for rigid bodies.
// Both functions work on a [begin, end) range so they can be split across threads.
#include <cstddef>

#include "Core/EntityManager.h"

namespace eng {

class Integrator {
public:
    // v += (g + F/m) dt ; w += I^-1 * torque * dt ; then damping.
    static void integrateVelocities(RigidBody* bodies, size_t begin, size_t end, float dt, const Vec3& gravity);

    // x += v dt ; q += 1/2 (0,w) q dt (normalised) ; refresh derived data ; clear forces.
    static void integratePositions(RigidBody* bodies, size_t begin, size_t end, float dt);
};

}  // namespace eng
