# 3D Physics Engine

A rigid-body physics engine in C++17 with an OpenGL viewer. The physics library has no graphics dependencies; the viewer is optional.

## Features

- **Rigid bodies** with position, orientation (quaternion), linear/angular velocity, and inertia tensors for boxes and spheres
- **Shapes:** sphere, oriented box, infinite plane
- **Integrator:** semi-implicit Euler, with implicit damping
- **Broad phase:** dynamic AABB tree with fat boxes and motion prediction
- **Narrow phase:**
  - sphere-sphere, sphere-plane, sphere-box, box-plane
  - box-box with the Separating Axis Theorem (15 axes) and Sutherland-Hodgman face clipping for contact manifolds; edge-edge contacts handled separately
- **Solver:** sequential impulses (projected Gauss-Seidel) with Coulomb friction (pyramid), restitution, Baumgarte penetration correction, speculative contacts and warm starting
- **Thread pool:** `parallelFor` used for integration and narrow phase (see Performance)

## Build

Requires CMake 3.16+ and a C++17 compiler. The viewer downloads GLFW automatically; on Linux you also need the X11/OpenGL dev packages (`libgl1-mesa-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev`).

```bash
cmake -S . -B build
cmake --build build --config Release
./build/physics_viewer      # on Windows: build\Release\physics_viewer.exe
ctest --test-dir build -C Release --output-on-failure
```

Physics library and tests only (no window, no GLFW download):

```bash
cmake -S . -B build -DBUILD_VIEWER=OFF
```

## Viewer controls

| Input | Action |
|---|---|
| Left mouse drag | Orbit camera |
| Scroll | Zoom |
| `B` / `N` | Drop a random box / sphere |
| `S` | Shoot a sphere from the camera |
| `C` | Show contact points and normals |
| `Space` | Pause |
| `R` | Reset scene |
| `Esc` | Quit |

## Using the library

```cpp
#include "Pipeline/World.h"
using namespace eng;

World world;
world.addPlane(Vec3(0, 1, 0), 0.f);                         // ground
int box = world.addBox(Vec3(0, 5, 0), Vec3(.5f, .5f, .5f), 1.f);   // mass 1
world.body(box).friction = 0.6f;

for (int i = 0; i < 600; i++) world.step(1.f / 120.f);      // fixed time step
```

Mass `<= 0` makes a body static. Always use a fixed time step (1/120 s works well).

## Layout

```
Core/        EngineMath.h (Vec3, Mat3, Quat, AABB), EntityManager (rigid bodies)
Collision/   Collision (narrow phase), AABBTree (broad phase)
Solver/      ConstraintSolver (contacts, friction, warm starting)
Pipeline/    Integrator, ThreadPool, World (the step pipeline)
Render/      OpenGL renderer (viewer only)
tests/       headless physics tests
examples/    the original ASCII particle demo (Windows only)
```

## Tests

`tests/tests.cpp` checks, among other things: bounce height against the restitution coefficient, sliding distance against the friction coefficient, momentum conservation in elastic collisions, boxes landing flat, a 5-box stack staying upright, an overhanging box falling while a centred one stays, AABB tree results against brute force, and 200 random bodies giving identical results threaded and single-threaded.

## Performance

Measured on a 2-core cloud VM, 1500 mixed bodies (about 2200 contacts), 120 Hz steps: roughly 5 ms per step single-threaded. The thread pool did not speed this up there, because the solver (serial) dominates and the machine has few cores. Parallelising the solver (for example by graph-colouring contacts into islands) is the next big win.

## Known limitations / ideas

- No sleeping: resting bodies are still simulated every step
- No gyroscopic term, so very fast spinning, elongated bodies are not exact
- No capsules or convex meshes; no joints/constraints between bodies
- Bodies cannot be removed individually (use `World::clear()`)
- The viewer uses fixed-function OpenGL, which works everywhere but is deprecated on macOS
