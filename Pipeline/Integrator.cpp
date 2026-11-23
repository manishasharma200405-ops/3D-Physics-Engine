#include "Pipeline/Integrator.h"

namespace eng {

namespace {
constexpr float kMaxAngularSpeed = 100.f;  // rad/s, keeps the quaternion update sane
}

void Integrator::integrateVelocities(RigidBody* bodies, size_t begin, size_t end, float dt, const Vec3& gravity) {
    for (size_t i = begin; i < end; i++) {
        RigidBody& b = bodies[i];
        if (!b.isDynamic()) continue;

        b.linearVelocity += (gravity + b.force * b.invMass) * dt;
        b.angularVelocity += (b.invInertiaWorld * b.torque) * dt;

        // Implicit damping: v *= 1 / (1 + c dt), stable for any dt.
        b.linearVelocity *= 1.f / (1.f + b.linearDamping * dt);
        b.angularVelocity *= 1.f / (1.f + b.angularDamping * dt);
    }
}

void Integrator::integratePositions(RigidBody* bodies, size_t begin, size_t end, float dt) {
    for (size_t i = begin; i < end; i++) {
        RigidBody& b = bodies[i];
        if (!b.isDynamic()) continue;

        float w = b.angularVelocity.length();
        if (w > kMaxAngularSpeed) b.angularVelocity *= kMaxAngularSpeed / w;

        b.position += b.linearVelocity * dt;

        Quat spin(0.f, b.angularVelocity.x, b.angularVelocity.y, b.angularVelocity.z);
        Quat dq = spin * b.orientation;
        b.orientation = Quat(b.orientation.w + 0.5f * dt * dq.w,
                             b.orientation.x + 0.5f * dt * dq.x,
                             b.orientation.y + 0.5f * dt * dq.y,
                             b.orientation.z + 0.5f * dt * dq.z).normalized();

        b.updateDerived();
        b.force = Vec3();
        b.torque = Vec3();
    }
}

}  // namespace eng
