#pragma once
// Narrow-phase collision detection for sphere, box and plane shapes.
#include "Core/EntityManager.h"

namespace eng {

struct Contact {
    Vec3 point;     // world-space contact position
    float depth;    // penetration depth; slightly negative = speculative (within margin)
};

struct Manifold {
    int a = -1, b = -1;       // body indices
    Vec3 normal;              // unit vector pointing from body a towards body b
    int count = 0;
    Contact contacts[8];      // reduced to at most 4 before returning from collide()
    float friction = 0.5f;
    float restitution = 0.f;
};

// Distance within which near-touching shapes still generate contacts.
constexpr float kContactMargin = 0.01f;

// Tests two bodies.  Returns true and fills `out` (normal points A -> B) when
// they overlap or are within kContactMargin.  out.a / out.b are left for the caller.
bool collide(const RigidBody& A, const RigidBody& B, Manifold& out);

}  // namespace eng
