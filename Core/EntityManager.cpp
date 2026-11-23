#include "Core/EntityManager.h"

#include <cmath>

namespace eng {

void RigidBody::updateDerived() {
    rotation = orientation.toMat3();
    invInertiaWorld = rotation * Mat3::diag(invInertiaLocal) * rotation.transposed();
}

AABB RigidBody::computeAABB() const {
    switch (shape.type) {
        case ShapeType::Sphere: {
            Vec3 r(shape.radius, shape.radius, shape.radius);
            return {position - r, position + r};
        }
        case ShapeType::Box: {
            Vec3 e;
            for (int i = 0; i < 3; i++)
                e[i] = std::fabs(rotation.m[i][0]) * shape.halfExtents.x +
                       std::fabs(rotation.m[i][1]) * shape.halfExtents.y +
                       std::fabs(rotation.m[i][2]) * shape.halfExtents.z;
            return {position - e, position + e};
        }
        default:
            return {position, position};
    }
}

int EntityManager::createSphere(const Vec3& pos, float radius, float mass) {
    RigidBody b;
    b.position = pos;
    b.shape.type = ShapeType::Sphere;
    b.shape.radius = radius;
    if (mass > 0.f) {
        b.invMass = 1.f / mass;
        float i = 0.4f * mass * radius * radius;
        b.invInertiaLocal = Vec3(1.f / i, 1.f / i, 1.f / i);
    }
    b.updateDerived();
    bodies_.push_back(b);
    return (int)bodies_.size() - 1;
}

int EntityManager::createBox(const Vec3& pos, const Vec3& h, float mass) {
    RigidBody b;
    b.position = pos;
    b.shape.type = ShapeType::Box;
    b.shape.halfExtents = h;
    if (mass > 0.f) {
        b.invMass = 1.f / mass;
        // Solid cuboid: I_x = m/3 (hy^2 + hz^2), etc.
        float ix = mass / 3.f * (h.y * h.y + h.z * h.z);
        float iy = mass / 3.f * (h.x * h.x + h.z * h.z);
        float iz = mass / 3.f * (h.x * h.x + h.y * h.y);
        b.invInertiaLocal = Vec3(1.f / ix, 1.f / iy, 1.f / iz);
    }
    b.updateDerived();
    bodies_.push_back(b);
    return (int)bodies_.size() - 1;
}

int EntityManager::createPlane(const Vec3& normal, float offset) {
    RigidBody b;
    b.shape.type = ShapeType::Plane;
    b.shape.normal = normalize(normal);
    b.shape.offset = offset;
    b.updateDerived();
    bodies_.push_back(b);
    return (int)bodies_.size() - 1;
}

}  // namespace eng
