#include "Pipeline/World.h"

namespace eng {

namespace {
constexpr size_t kParallelThreshold = 256;  // below this, threading costs more than it saves
}

World::World(const WorldSettings& s) : settings(s), pool_(new ThreadPool(s.threads)) {
    solver_.settings.iterations = s.solverIterations;
}

int World::addSphere(const Vec3& pos, float radius, float mass) {
    int i = entities_.createSphere(pos, radius, mass);
    registerBody(i);
    return i;
}

int World::addBox(const Vec3& pos, const Vec3& h, float mass) {
    int i = entities_.createBox(pos, h, mass);
    registerBody(i);
    return i;
}

int World::addPlane(const Vec3& normal, float offset) {
    int i = entities_.createPlane(normal, offset);
    planes_.push_back(i);
    return i;
}

void World::registerBody(int index) {
    RigidBody& b = entities_.get(index);
    b.proxy = tree_.createProxy(b.computeAABB(), index);
}

void World::clear() {
    entities_.clear();
    tree_.clear();
    solver_.clearCache();
    planes_.clear();
    pairs_.clear();
    manifolds_.clear();
    pairCount_ = 0;
}

void World::step(float dt) {
    std::vector<RigidBody>& bodies = entities_.bodies();
    const size_t n = bodies.size();
    RigidBody* data = bodies.data();
    solver_.settings.iterations = settings.solverIterations;

    // 1. Apply forces to velocities.
    pool_->parallelFor(n, kParallelThreshold, [&](size_t b, size_t e) {
        Integrator::integrateVelocities(data, b, e, dt, settings.gravity);
    });

    // 2. Broad phase: refresh the tree, then collect candidate pairs.
    for (size_t i = 0; i < n; i++) {
        RigidBody& b = bodies[i];
        if (b.proxy >= 0 && b.isDynamic()) tree_.moveProxy(b.proxy, b.computeAABB(), b.linearVelocity * dt);
    }

    pairs_.clear();
    for (size_t i = 0; i < n; i++) {
        const RigidBody& bi = bodies[i];
        if (bi.proxy < 0 || !bi.isDynamic()) continue;

        tree_.query(tree_.fatAABB(bi.proxy), [&](int proxy) {
            int j = tree_.userData(proxy);
            if (j == (int)i) return;
            // Dynamic-dynamic pairs are reported once (lower index owns them);
            // dynamic-static pairs are only ever found from the dynamic side.
            if (bodies[j].isDynamic() && j < (int)i) return;
            pairs_.emplace_back(std::min((int)i, j), std::max((int)i, j));
        });

        for (int p : planes_) pairs_.emplace_back(std::min((int)i, p), std::max((int)i, p));
    }
    pairCount_ = (int)pairs_.size();

    // 3. Narrow phase (parallel, each pair writes only its own slot).
    scratch_.resize(pairs_.size());
    hit_.assign(pairs_.size(), 0);
    pool_->parallelFor(pairs_.size(), kParallelThreshold, [&](size_t b, size_t e) {
        for (size_t k = b; k < e; k++) {
            Manifold& m = scratch_[k];
            m.a = pairs_[k].first;
            m.b = pairs_[k].second;
            hit_[k] = collide(bodies[m.a], bodies[m.b], m) ? 1 : 0;
        }
    });
    manifolds_.clear();
    for (size_t k = 0; k < pairs_.size(); k++)
        if (hit_[k]) manifolds_.push_back(scratch_[k]);

    // 4. Solve contacts.
    solver_.solve(bodies, manifolds_, dt);

    // 5. Move bodies.
    pool_->parallelFor(n, kParallelThreshold, [&](size_t b, size_t e) {
        Integrator::integratePositions(data, b, e, dt);
    });
}

}  // namespace eng
