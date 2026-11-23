#pragma once
// Rigid body storage.  Bodies are addressed by integer handle (index) and are
// never removed individually; use clear() to reset.
#include <vector>

#include "Core/EngineMath.h"

namespace eng {

// Order matters: collision dispatch relies on Sphere < Box < Plane.
enum class ShapeType { Sphere = 0, Box = 1, Plane = 2 };

struct Shape {
    ShapeType type = ShapeType::Sphere;
    float radius = 0.5f;                    // Sphere
    Vec3 halfExtents{0.5f, 0.5f, 0.5f};     // Box (local axes)
    Vec3 normal{0.f, 1.f, 0.f};             // Plane: solid side is dot(n,p) < offset
    float offset = 0.f;                     // Plane
};

struct RigidBody {
    Vec3 position;
    Quat orientation;
    Vec3 linearVelocity;
    Vec3 angularVelocity;
    Vec3 force;    // accumulated, cleared every step
    Vec3 torque;

    float invMass = 0.f;          // 0 => static
    Vec3 invInertiaLocal;         // diagonal, body space
    Mat3 rotation;                // derived
    Mat3 invInertiaWorld = Mat3::zero();  // derived

    float restitution = 0.3f;
    float friction = 0.5f;
    float linearDamping = 0.01f;
    float angularDamping = 0.05f;

    Shape shape;
    int proxy = -1;               // broadphase tree proxy, -1 for planes

    bool isDynamic() const { return invMass > 0.f; }

    // Recompute rotation matrix and world-space inverse inertia.
    void updateDerived();
    // World-space bounding box (not valid for planes).
    AABB computeAABB() const;
    // Velocity of the point at offset r from the centre of mass.
    Vec3 velocityAt(const Vec3& r) const { return linearVelocity + cross(angularVelocity, r); }
};

class EntityManager {
public:
    // mass <= 0 creates a static body.
    int createSphere(const Vec3& pos, float radius, float mass);
    int createBox(const Vec3& pos, const Vec3& halfExtents, float mass);
    // Static infinite plane: solid below, dot(normal, p) = offset on the surface.
    int createPlane(const Vec3& normal, float offset);

    RigidBody& get(int i) { return bodies_[i]; }
    const RigidBody& get(int i) const { return bodies_[i]; }
    std::vector<RigidBody>& bodies() { return bodies_; }
    const std::vector<RigidBody>& bodies() const { return bodies_; }
    int size() const { return (int)bodies_.size(); }
    void clear() { bodies_.clear(); }

private:
    std::vector<RigidBody> bodies_;
};

}  // namespace eng
