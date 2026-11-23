// OpenGL viewer for the physics engine.
//
//   Left mouse drag  orbit camera        Scroll   zoom
//   B  drop a box    N  drop a sphere    S  shoot a sphere from the camera
//   C  toggle contact points             Space  pause     R  reset scene     Esc  quit
#include <GLFW/glfw3.h>

#include <chrono>
#include <cstdio>
#include <random>

#include "Pipeline/World.h"
#include "Render/Renderer.h"

using namespace eng;

static std::mt19937 rng(2024);

static float rnd(float lo, float hi) {
    return std::uniform_real_distribution<float>(lo, hi)(rng);
}

static void buildScene(World& w) {
    w.clear();
    w.addPlane(Vec3(0, 1, 0), 0.f);

    // A wall of boxes to knock over.
    for (int y = 0; y < 6; y++)
        for (int x = 0; x < 6; x++) {
            float off = (y % 2) ? 0.5f : 0.f;
            w.addBox(Vec3(-3.f + x * 1.02f + off, 0.5f + y * 1.0f, -4.f), Vec3(0.5f, 0.5f, 0.5f), 1.f);
        }

    // A short pyramid.
    for (int level = 0; level < 4; level++)
        for (int i = 0; i < 4 - level; i++)
            w.addBox(Vec3(4.f + (i - (3 - level) * 0.5f) * 1.02f, 0.5f + level, 2.f), Vec3(0.5f, 0.5f, 0.5f), 1.f);

    // A couple of balls to start the action.
    for (int i = 0; i < 4; i++) {
        int s = w.addSphere(Vec3(rnd(-2, 2), 8.f + i * 1.5f, rnd(-1, 1)), 0.5f, 1.f);
        w.body(s).restitution = 0.5f;
    }
}

static void spawn(World& w, bool box, const Vec3& pos, const Vec3& vel) {
    int i;
    if (box) {
        i = w.addBox(pos, Vec3(rnd(0.3f, 0.7f), rnd(0.3f, 0.7f), rnd(0.3f, 0.7f)), 1.f);
        w.body(i).orientation = Quat::fromAxisAngle(Vec3(rnd(-1, 1), rnd(-1, 1), rnd(-1, 1)), rnd(0, 6.28f)).normalized();
        w.body(i).updateDerived();
    } else {
        i = w.addSphere(pos, rnd(0.3f, 0.6f), 1.f);
        w.body(i).restitution = 0.5f;
    }
    w.body(i).linearVelocity = vel;
}

int main() {
    Renderer renderer;
    if (!renderer.init(1280, 720, "3D Physics Engine")) {
        std::fprintf(stderr, "Could not create an OpenGL window.\n");
        return 1;
    }

    WorldSettings settings;
    settings.threads = 2;
    World world(settings);
    buildScene(world);

    const float dt = 1.f / 120.f;
    double accumulator = 0.0;
    bool paused = false, showContacts = false;
    auto last = std::chrono::steady_clock::now();
    double fpsTimer = 0.0;
    int frames = 0;

    while (!renderer.shouldClose()) {
        renderer.pollEvents();
        if (renderer.keyDown(GLFW_KEY_ESCAPE)) break;
        if (renderer.keyPressedOnce(GLFW_KEY_SPACE)) paused = !paused;
        if (renderer.keyPressedOnce(GLFW_KEY_C)) showContacts = !showContacts;
        if (renderer.keyPressedOnce(GLFW_KEY_R)) buildScene(world);
        if (renderer.keyPressedOnce(GLFW_KEY_B)) spawn(world, true, Vec3(rnd(-3, 3), 12.f, rnd(-3, 3)), Vec3());
        if (renderer.keyPressedOnce(GLFW_KEY_N)) spawn(world, false, Vec3(rnd(-3, 3), 12.f, rnd(-3, 3)), Vec3());
        if (renderer.keyPressedOnce(GLFW_KEY_S))
            spawn(world, false, renderer.cameraPosition() + renderer.cameraForward() * 2.f, renderer.cameraForward() * 25.f);

        auto now = std::chrono::steady_clock::now();
        double frame = std::chrono::duration<double>(now - last).count();
        last = now;
        if (frame > 0.1) frame = 0.1;  // avoid a spiral of death after a stall

        if (!paused) {
            accumulator += frame;
            while (accumulator >= dt) {
                world.step(dt);
                accumulator -= dt;
            }
        }

        renderer.draw(world, showContacts);

        fpsTimer += frame;
        frames++;
        if (fpsTimer >= 0.5) {
            char title[160];
            std::snprintf(title, sizeof(title), "3D Physics Engine | %d bodies | %d contacts | %.0f fps%s",
                          world.bodyCount(), (int)world.manifolds().size(), frames / fpsTimer, paused ? " | PAUSED" : "");
            renderer.setTitle(title);
            fpsTimer = 0.0;
            frames = 0;
        }
    }

    renderer.shutdown();
    return 0;
}
