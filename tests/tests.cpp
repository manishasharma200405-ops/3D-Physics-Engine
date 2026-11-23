// Headless physics tests.  No framework: each CHECK prints on failure and the
// process exit code is the number of failed checks.
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "Collision/AABBTree.h"
#include "Pipeline/ThreadPool.h"
#include "Pipeline/World.h"

using namespace eng;

static int g_failed = 0;
static int g_checks = 0;

#define CHECK(cond, ...)                                              \
    do {                                                              \
        g_checks++;                                                   \
        if (!(cond)) {                                                \
            g_failed++;                                               \
            std::printf("  FAIL %s:%d  %s\n    ", __FILE__, __LINE__, #cond); \
            std::printf(__VA_ARGS__);                                 \
            std::printf("\n");                                        \
        }                                                             \
    } while (0)

static const float DT = 1.f / 120.f;

static void run(World& w, float seconds) {
    int steps = (int)(seconds / DT);
    for (int i = 0; i < steps; i++) w.step(DT);
}

static void testMath() {
    std::printf("math\n");
    Quat q = Quat::fromAxisAngle(Vec3(0, 0, 1), kPi / 2.f);
    Vec3 v = q.rotate(Vec3(1, 0, 0));
    CHECK(std::fabs(v.x) < 1e-5f && std::fabs(v.y - 1.f) < 1e-5f, "rotated (%f,%f,%f)", v.x, v.y, v.z);

    Mat3 r = q.toMat3();
    Mat3 id = r * r.transposed();
    CHECK(std::fabs(id.m[0][0] - 1.f) < 1e-5f && std::fabs(id.m[0][1]) < 1e-5f, "R*R^T not identity");

    Vec3 c = cross(Vec3(1, 0, 0), Vec3(0, 1, 0));
    CHECK(std::fabs(c.z - 1.f) < 1e-6f, "cross product");
}

static void testThreadPool() {
    std::printf("thread pool\n");
    ThreadPool pool(3);
    std::vector<int> data(10000, 0);
    pool.parallelFor(data.size(), 64, [&](size_t b, size_t e) {
        for (size_t i = b; i < e; i++) data[i] = (int)i;
    });
    long long sum = 0;
    for (int x : data) sum += x;
    CHECK(sum == 49995000LL, "sum=%lld", sum);

    // Reuse repeatedly to catch lifetime / wake-up races.
    std::atomic<int> counter(0);
    for (int rep = 0; rep < 200; rep++)
        pool.parallelFor(100, 10, [&](size_t b, size_t e) { counter += (int)(e - b); });
    CHECK(counter == 200 * 100, "counter=%d", counter.load());
}

static void testAABBTree() {
    std::printf("aabb tree\n");
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> pos(-50.f, 50.f), size(0.2f, 3.f);

    AABBTree tree(0.1f);
    std::vector<AABB> boxes;
    std::vector<int> proxies;
    for (int i = 0; i < 300; i++) {
        Vec3 c(pos(rng), pos(rng), pos(rng)), h(size(rng), size(rng), size(rng));
        AABB b(c - h, c + h);
        boxes.push_back(b);
        proxies.push_back(tree.createProxy(b, i));
    }
    CHECK(tree.height() < 40, "height %d", tree.height());

    // Move a third of them, remove a few, then compare against brute force.
    for (int i = 0; i < 100; i++) {
        Vec3 d(pos(rng) * 0.1f, pos(rng) * 0.1f, pos(rng) * 0.1f);
        boxes[i] = AABB(boxes[i].min + d, boxes[i].max + d);
        tree.moveProxy(proxies[i], boxes[i], d);
    }
    std::vector<bool> alive(300, true);
    for (int i = 290; i < 300; i++) { tree.destroyProxy(proxies[i]); alive[i] = false; }
    CHECK(tree.proxyCount() == 290, "proxy count %d", tree.proxyCount());

    int mismatches = 0;
    for (int q = 0; q < 200; q++) {
        Vec3 c(pos(rng), pos(rng), pos(rng));
        AABB query(c - Vec3(8, 8, 8), c + Vec3(8, 8, 8));
        std::vector<int> found;
        tree.query(query, [&](int p) { found.push_back(tree.userData(p)); });
        // The tree uses fat boxes, so it must find AT LEAST every true overlap.
        for (int i = 0; i < 300; i++) {
            if (!alive[i] || !boxes[i].overlaps(query)) continue;
            if (std::find(found.begin(), found.end(), i) == found.end()) mismatches++;
        }
    }
    CHECK(mismatches == 0, "%d true overlaps missed by the tree", mismatches);
}

static void testSphereBounce() {
    std::printf("sphere bounce\n");
    WorldSettings s;
    World w(s);
    w.addPlane(Vec3(0, 1, 0), 0.f);
    int ball = w.addSphere(Vec3(0, 5.5f, 0), 0.5f, 1.f);
    w.body(ball).restitution = 0.8f;
    w.body(ball).linearDamping = 0.f;

    // Drop from 5 m of clearance; after the first bounce the peak should be ~ e^2 * 5 = 3.2 m.
    bool bounced = false;
    float peak = 0.f;
    for (int i = 0; i < 6 * 120; i++) {
        float vyBefore = w.body(ball).linearVelocity.y;
        w.step(DT);
        float vy = w.body(ball).linearVelocity.y;
        if (!bounced && vyBefore < -1.f && vy > 0.f) bounced = true;
        if (bounced) peak = std::max(peak, w.body(ball).position.y - 0.5f);
        if (bounced && vy < 0.f && peak > 1.f) break;
    }
    CHECK(bounced, "ball never bounced");
    CHECK(peak > 2.7f && peak < 3.5f, "bounce peak %.3f m (expected ~3.2)", peak);
}

static void testSphereRest() {
    std::printf("sphere comes to rest\n");
    World w;
    w.addPlane(Vec3(0, 1, 0), 0.f);
    int ball = w.addSphere(Vec3(0, 3.f, 0), 0.5f, 1.f);
    run(w, 6.f);
    const RigidBody& b = w.body(ball);
    CHECK(std::fabs(b.position.y - 0.5f) < 0.02f, "y = %.4f", b.position.y);
    CHECK(b.linearVelocity.length() < 0.05f, "speed = %.4f", b.linearVelocity.length());
}

static void testBoxRest() {
    std::printf("box lands flat and rests\n");
    World w;
    w.addPlane(Vec3(0, 1, 0), 0.f);
    int box = w.addBox(Vec3(0, 2.f, 0), Vec3(0.5f, 0.5f, 0.5f), 2.f);
    w.body(box).orientation = Quat::fromAxisAngle(Vec3(1, 0, 1), 0.3f).normalized();
    w.body(box).updateDerived();
    run(w, 6.f);
    const RigidBody& b = w.body(box);
    Vec3 up = b.rotation.col(1);
    CHECK(std::fabs(b.position.y - 0.5f) < 0.03f, "y = %.4f", b.position.y);
    CHECK(std::fabs(up.y) > 0.99f, "up.y = %.4f (box not flat)", up.y);
    CHECK(b.linearVelocity.length() < 0.1f, "speed = %.4f", b.linearVelocity.length());
}

static void testFriction() {
    std::printf("friction stops a sliding box\n");
    World w;
    w.addPlane(Vec3(0, 1, 0), 0.f);
    int box = w.addBox(Vec3(0, 0.5f, 0), Vec3(0.5f, 0.5f, 0.5f), 1.f);
    w.body(box).friction = 0.8f;
    w.body(box).linearVelocity = Vec3(4.f, 0, 0);
    run(w, 3.f);
    // Deceleration mu*g = 7.85 m/s^2 -> stops after ~0.51 s, ~1.0 m.
    const RigidBody& b = w.body(box);
    CHECK(b.linearVelocity.length() < 0.05f, "still moving at %.3f m/s", b.linearVelocity.length());
    CHECK(b.position.x > 0.7f && b.position.x < 1.4f, "slid to x = %.3f (expected ~1.0)", b.position.x);
}

static void testElasticCollision() {
    std::printf("head-on elastic sphere collision conserves momentum\n");
    WorldSettings s;
    s.gravity = Vec3(0, 0, 0);
    World w(s);
    int a = w.addSphere(Vec3(-2, 0, 0), 0.5f, 1.f);
    int b = w.addSphere(Vec3(2, 0, 0), 0.5f, 1.f);
    for (int i : {a, b}) {
        w.body(i).restitution = 1.f;
        w.body(i).linearDamping = 0.f;
    }
    w.body(a).linearVelocity = Vec3(5, 0, 0);
    w.body(b).linearVelocity = Vec3(-3, 0, 0);
    float pBefore = 5.f - 3.f;
    run(w, 1.5f);
    float pAfter = w.body(a).linearVelocity.x + w.body(b).linearVelocity.x;
    CHECK(std::fabs(pAfter - pBefore) < 0.01f, "momentum %.4f -> %.4f", pBefore, pAfter);
    // Equal masses, e = 1: velocities swap.
    CHECK(std::fabs(w.body(a).linearVelocity.x + 3.f) < 0.1f, "a vx = %.3f (expected -3)", w.body(a).linearVelocity.x);
    CHECK(std::fabs(w.body(b).linearVelocity.x - 5.f) < 0.1f, "b vx = %.3f (expected 5)", w.body(b).linearVelocity.x);
}

static void testSphereOnBox() {
    std::printf("sphere rests on a static box\n");
    World w;
    w.addBox(Vec3(0, -0.5f, 0), Vec3(5, 0.5f, 5), 0.f);   // top surface at y = 0
    int ball = w.addSphere(Vec3(0.3f, 2.f, 0.2f), 0.5f, 1.f);
    run(w, 5.f);
    CHECK(std::fabs(w.body(ball).position.y - 0.5f) < 0.03f, "y = %.4f", w.body(ball).position.y);
}

static void testStack() {
    std::printf("stack of 5 boxes stays standing\n");
    World w;
    w.addPlane(Vec3(0, 1, 0), 0.f);
    std::vector<int> boxes;
    for (int i = 0; i < 5; i++) boxes.push_back(w.addBox(Vec3(0, 0.5f + i * 1.0f, 0), Vec3(0.5f, 0.5f, 0.5f), 1.f));
    run(w, 8.f);
    float maxDrift = 0.f, maxHeightErr = 0.f;
    for (int i = 0; i < 5; i++) {
        const RigidBody& b = w.body(boxes[i]);
        maxDrift = std::max(maxDrift, std::sqrt(b.position.x * b.position.x + b.position.z * b.position.z));
        maxHeightErr = std::max(maxHeightErr, std::fabs(b.position.y - (0.5f + i)));
    }
    CHECK(maxDrift < 0.1f, "horizontal drift %.4f", maxDrift);
    CHECK(maxHeightErr < 0.1f, "height error %.4f", maxHeightErr);
}

static void testOffsetStackAndTopple() {
    std::printf("box balanced half off the edge topples, centred box does not\n");
    World w;
    w.addPlane(Vec3(0, 1, 0), 0.f);
    int base = w.addBox(Vec3(0, 0.5f, 0), Vec3(1.f, 0.5f, 1.f), 0.f);  // static pedestal, top at y = 1
    (void)base;
    int safe = w.addBox(Vec3(0.2f, 1.5f, 0), Vec3(0.5f, 0.5f, 0.5f), 1.f);
    int tip = w.addBox(Vec3(5.f + 1.4f, 1.5f, 0), Vec3(0.5f, 0.5f, 0.5f), 1.f);
    // Second pedestal for the tipping box: centre of mass beyond the edge.
    w.addBox(Vec3(5.f, 0.5f, 0), Vec3(1.f, 0.5f, 1.f), 0.f);
    run(w, 5.f);
    CHECK(w.body(safe).position.y > 1.4f, "safe box fell: y = %.3f", w.body(safe).position.y);
    CHECK(w.body(tip).position.y < 1.2f, "overhanging box did not fall: y = %.3f", w.body(tip).position.y);
}

static void testManyBodies() {
    std::printf("200 random bodies: no NaNs, nothing leaves the world, threaded == serial\n");
    auto build = [](World& w) {
        w.addPlane(Vec3(0, 1, 0), 0.f);
        w.addPlane(Vec3(1, 0, 0), -10.f);   // wall: solid for x < -10
        std::mt19937 rng(99);
        std::uniform_real_distribution<float> px(-4.f, 4.f), py(1.f, 30.f), sz(0.25f, 0.6f);
        for (int i = 0; i < 200; i++) {
            Vec3 p(px(rng), py(rng), px(rng));
            if (i % 2) w.addBox(p, Vec3(sz(rng), sz(rng), sz(rng)), 1.f);
            else w.addSphere(p, sz(rng), 1.f);
        }
    };
    WorldSettings serial, threaded;
    threaded.threads = 3;
    World a(serial), b(threaded);
    build(a);
    build(b);
    for (int i = 0; i < 600; i++) { a.step(DT); b.step(DT); }

    bool bad = false;
    float maxDiff = 0.f;
    for (int i = 0; i < a.bodyCount(); i++) {
        const Vec3& p = a.body(i).position;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) bad = true;
        if (a.body(i).isDynamic() && (p.y < -1.f || p.y > 40.f || std::fabs(p.x) > 30.f)) bad = true;
        maxDiff = std::max(maxDiff, (p - b.body(i).position).length());
    }
    CHECK(!bad, "NaN or escaped body");
    CHECK(maxDiff < 1e-4f, "threaded and serial runs differ by %.6f", maxDiff);
}

int main() {
    testMath();
    testThreadPool();
    testAABBTree();
    testSphereBounce();
    testSphereRest();
    testBoxRest();
    testFriction();
    testElasticCollision();
    testSphereOnBox();
    testStack();
    testOffsetStackAndTopple();
    testManyBodies();

    std::printf("\n%d checks, %d failed\n", g_checks, g_failed);
    return g_failed;
}
