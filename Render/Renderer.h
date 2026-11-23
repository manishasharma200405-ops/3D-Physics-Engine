#pragma once
// Small OpenGL 2.1 (fixed-function) renderer + orbit camera.  Only used by the viewer;
// the physics library never includes this.
#include "Pipeline/World.h"

struct GLFWwindow;

class Renderer {
public:
    bool init(int width, int height, const char* title);
    void shutdown();

    bool shouldClose() const;
    void pollEvents();
    void setTitle(const char* title);
    bool keyPressedOnce(int glfwKey);   // true once per physical key press
    bool keyDown(int glfwKey) const;

    // Draws the whole world and swaps buffers.
    void draw(const eng::World& world, bool showContacts);

    // Camera ray through the screen centre, used for shooting objects.
    eng::Vec3 cameraPosition() const;
    eng::Vec3 cameraForward() const;

private:
    static void onCursor(GLFWwindow*, double x, double y);
    static void onScroll(GLFWwindow*, double dx, double dy);

    GLFWwindow* window_ = nullptr;
    float yaw_ = 0.6f, pitch_ = 0.35f, distance_ = 22.f;
    eng::Vec3 target_{0.f, 3.f, 0.f};
    double lastX_ = 0, lastY_ = 0;
    bool keyLatch_[512] = {};
};
