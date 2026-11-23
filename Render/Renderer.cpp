#include "Render/Renderer.h"

#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>

using namespace eng;

namespace {

Renderer* g_self = nullptr;  // GLFW callbacks are plain functions

// Column-major 4x4 for glMultMatrixf from a body's rotation and position.
void bodyMatrix(const RigidBody& b, float out[16]) {
    for (int c = 0; c < 3; c++)
        for (int r = 0; r < 3; r++) out[c * 4 + r] = b.rotation.m[r][c];
    out[3] = out[7] = out[11] = 0.f;
    out[12] = b.position.x;
    out[13] = b.position.y;
    out[14] = b.position.z;
    out[15] = 1.f;
}

void setColor(int index, bool dynamic) {
    if (!dynamic) { glColor3f(0.45f, 0.47f, 0.52f); return; }
    // Cheap hash -> pleasant pastel colour.
    float h = std::fmod(index * 0.61803398f, 1.f) * 6.f;
    float x = 1.f - std::fabs(std::fmod(h, 2.f) - 1.f);
    float r = 0, g = 0, b = 0;
    if (h < 1) { r = 1; g = x; } else if (h < 2) { r = x; g = 1; }
    else if (h < 3) { g = 1; b = x; } else if (h < 4) { g = x; b = 1; }
    else if (h < 5) { r = x; b = 1; } else { r = 1; b = x; }
    glColor3f(0.35f + 0.55f * r, 0.35f + 0.55f * g, 0.35f + 0.55f * b);
}

void drawUnitCube() {
    static const float n[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    glBegin(GL_QUADS);
    for (int f = 0; f < 6; f++) {
        int a = f / 2;                 // face axis
        float s = (f % 2 == 0) ? 1.f : -1.f;
        int u = (a + 1) % 3, v = (a + 2) % 3;
        glNormal3fv(n[f]);
        static const float cu[4] = {-1, 1, 1, -1}, cv[4] = {-1, -1, 1, 1};
        for (int k = 0; k < 4; k++) {
            float p[3];
            p[a] = s;
            p[u] = cu[k];
            p[v] = cv[k];
            // Flip winding on negative faces so all quads face outward.
            if (s < 0) { p[u] = cu[3 - k]; p[v] = cv[3 - k]; }
            glVertex3f(p[0], p[1], p[2]);
        }
    }
    glEnd();
}

void drawSphere(float r) {
    const int stacks = 14, slices = 24;
    for (int i = 0; i < stacks; i++) {
        float p0 = kPi * i / stacks - kPi / 2, p1 = kPi * (i + 1) / stacks - kPi / 2;
        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= slices; j++) {
            float t = 2 * kPi * j / slices;
            for (float p : {p1, p0}) {
                float x = std::cos(p) * std::cos(t), y = std::sin(p), z = std::cos(p) * std::sin(t);
                glNormal3f(x, y, z);
                glVertex3f(x * r, y * r, z * r);
            }
        }
        glEnd();
    }
}

void drawPlane(const Shape& s) {
    Vec3 n = s.normal;
    Vec3 t1 = std::fabs(n.y) < 0.9f ? normalize(cross(n, Vec3(0, 1, 0))) : Vec3(1, 0, 0);
    Vec3 t2 = cross(n, t1);
    Vec3 c = n * s.offset;
    const int half = 30;

    glDisable(GL_LIGHTING);
    glColor3f(0.16f, 0.17f, 0.20f);
    glBegin(GL_QUADS);
    for (Vec3 corner : {Vec3(-1, -1, 0), Vec3(1, -1, 0), Vec3(1, 1, 0), Vec3(-1, 1, 0)}) {
        Vec3 p = c + t1 * (corner.x * half) + t2 * (corner.y * half) - n * 0.01f;
        glVertex3f(p.x, p.y, p.z);
    }
    glEnd();

    glColor3f(0.28f, 0.30f, 0.34f);
    glBegin(GL_LINES);
    for (int i = -half; i <= half; i += 2) {
        Vec3 a = c + t1 * (float)i + t2 * (float)-half, b = c + t1 * (float)i + t2 * (float)half;
        Vec3 d = c + t2 * (float)i + t1 * (float)-half, e = c + t2 * (float)i + t1 * (float)half;
        glVertex3f(a.x, a.y, a.z); glVertex3f(b.x, b.y, b.z);
        glVertex3f(d.x, d.y, d.z); glVertex3f(e.x, e.y, e.z);
    }
    glEnd();
    glEnable(GL_LIGHTING);
}

}  // namespace

bool Renderer::init(int width, int height, const char* title) {
    if (!glfwInit()) return false;
    glfwWindowHint(GLFW_SAMPLES, 4);
    window_ = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!window_) { glfwTerminate(); return false; }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    g_self = this;
    glfwSetCursorPosCallback(window_, onCursor);
    glfwSetScrollCallback(window_, onScroll);
    glfwGetCursorPos(window_, &lastX_, &lastY_);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_NORMALIZE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);  // winding-agnostic lighting
    glClearColor(0.07f, 0.08f, 0.10f, 1.f);
    return true;
}

void Renderer::shutdown() {
    if (window_) glfwDestroyWindow(window_);
    window_ = nullptr;
    glfwTerminate();
}

bool Renderer::shouldClose() const { return glfwWindowShouldClose(window_); }
void Renderer::pollEvents() { glfwPollEvents(); }
void Renderer::setTitle(const char* title) { glfwSetWindowTitle(window_, title); }
bool Renderer::keyDown(int key) const { return glfwGetKey(window_, key) == GLFW_PRESS; }

bool Renderer::keyPressedOnce(int key) {
    bool down = glfwGetKey(window_, key) == GLFW_PRESS;
    bool fired = down && !keyLatch_[key];
    keyLatch_[key] = down;
    return fired;
}

void Renderer::onCursor(GLFWwindow* w, double x, double y) {
    Renderer* r = g_self;
    if (!r) return;
    double dx = x - r->lastX_, dy = y - r->lastY_;
    r->lastX_ = x;
    r->lastY_ = y;
    if (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        r->yaw_ -= (float)dx * 0.008f;
        r->pitch_ = std::max(-1.5f, std::min(1.5f, r->pitch_ + (float)dy * 0.008f));
    }
}

void Renderer::onScroll(GLFWwindow*, double, double dy) {
    Renderer* r = g_self;
    if (r) r->distance_ = std::max(3.f, std::min(80.f, r->distance_ * std::pow(0.9f, (float)dy)));
}

Vec3 Renderer::cameraPosition() const {
    return target_ + Vec3(std::cos(pitch_) * std::sin(yaw_), std::sin(pitch_), std::cos(pitch_) * std::cos(yaw_)) * distance_;
}

Vec3 Renderer::cameraForward() const { return normalize(target_ - cameraPosition()); }

void Renderer::draw(const World& world, bool showContacts) {
    int w, h;
    glfwGetFramebufferSize(window_, &w, &h);
    if (h == 0) h = 1;
    glViewport(0, 0, w, h);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Projection.
    const float nearZ = 0.1f, farZ = 300.f, fov = 60.f * kPi / 180.f;
    float top = nearZ * std::tan(fov * 0.5f), right = top * (float)w / (float)h;
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-right, right, -top, top, nearZ, farZ);

    // View (look-at).
    Vec3 eye = cameraPosition();
    Vec3 f = normalize(target_ - eye);
    Vec3 s = normalize(cross(f, Vec3(0, 1, 0)));
    Vec3 u = cross(s, f);
    float m[16] = {s.x, u.x, -f.x, 0, s.y, u.y, -f.y, 0, s.z, u.z, -f.z, 0, 0, 0, 0, 1};
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(m);
    glTranslatef(-eye.x, -eye.y, -eye.z);

    float lightPos[4] = {0.4f, 1.f, 0.3f, 0.f};
    float ambient[4] = {0.28f, 0.28f, 0.30f, 1.f};
    float diffuse[4] = {0.85f, 0.85f, 0.80f, 1.f};
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);

    const std::vector<RigidBody>& bodies = world.bodies();
    for (size_t i = 0; i < bodies.size(); i++) {
        const RigidBody& b = bodies[i];
        if (b.shape.type == ShapeType::Plane) { drawPlane(b.shape); continue; }

        glPushMatrix();
        float mat[16];
        bodyMatrix(b, mat);
        glMultMatrixf(mat);
        setColor((int)i, b.isDynamic());
        if (b.shape.type == ShapeType::Box) {
            glScalef(b.shape.halfExtents.x, b.shape.halfExtents.y, b.shape.halfExtents.z);
            drawUnitCube();
        } else {
            drawSphere(b.shape.radius);
        }
        glPopMatrix();
    }

    if (showContacts) {
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glPointSize(6.f);
        glColor3f(1.f, 0.2f, 0.2f);
        glBegin(GL_POINTS);
        for (const Manifold& mf : world.manifolds())
            for (int k = 0; k < mf.count; k++) glVertex3f(mf.contacts[k].point.x, mf.contacts[k].point.y, mf.contacts[k].point.z);
        glEnd();
        glBegin(GL_LINES);
        for (const Manifold& mf : world.manifolds())
            for (int k = 0; k < mf.count; k++) {
                Vec3 p = mf.contacts[k].point, q = p + mf.normal * 0.4f;
                glVertex3f(p.x, p.y, p.z);
                glVertex3f(q.x, q.y, q.z);
            }
        glEnd();
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LIGHTING);
    }

    glfwSwapBuffers(window_);
}
