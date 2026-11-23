#pragma once
// Ties everything together: one call to step() runs the whole pipeline.
//
//   forces -> velocities -> broad phase (AABB tree) -> narrow phase -> solver -> positions
#include <memory>
#include <utility>
#include <vector>

#include "Collision/AABBTree.h"
#include "Collision/Collision.h"
#include "Core/EntityManager.h"
#include "Pipeline/Integrator.h"
#include "Pipeline/ThreadPool.h"
#include "Solver/ConstraintSolver.h"

namespace eng {

struct WorldSettings {
    Vec3 gravity{0.f, -9.81f, 0.f};
    int solverIterations = 20;
    unsigned threads = 0;  // worker threads; 0 = run single-threaded
};

class World {
public:
    explicit World(const WorldSettings& s = WorldSettings());

    // mass <= 0 creates a static body.  All return the body handle.
    int addSphere(const Vec3& pos, float radius, float mass);
    int addBox(const Vec3& pos, const Vec3& halfExtents, float mass);
    int addPlane(const Vec3& normal, float offset);

    RigidBody& body(int i) { return entities_.get(i); }
    const RigidBody& body(int i) const { return entities_.get(i); }
    int bodyCount() const { return entities_.size(); }
    const std::vector<RigidBody>& bodies() const { return entities_.bodies(); }

    void step(float dt);
    void clear();

    // Diagnostics from the most recent step.
    const std::vector<Manifold>& manifolds() const { return manifolds_; }
    int pairCount() const { return pairCount_; }

    WorldSettings settings;
    ConstraintSolver& solver() { return solver_; }
    const AABBTree& broadphase() const { return tree_; }

private:
    void registerBody(int index);

    EntityManager entities_;
    AABBTree tree_;
    ConstraintSolver solver_;
    std::unique_ptr<ThreadPool> pool_;
    std::vector<int> planes_;
    std::vector<std::pair<int, int>> pairs_;
    std::vector<Manifold> manifolds_;
    std::vector<Manifold> scratch_;
    std::vector<char> hit_;
    int pairCount_ = 0;
};

}  // namespace eng
