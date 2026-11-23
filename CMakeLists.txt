cmake_minimum_required(VERSION 3.16)
project(Physics3D LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(BUILD_VIEWER "Build the OpenGL viewer (downloads GLFW)" ON)
option(BUILD_TESTS  "Build the physics tests" ON)

find_package(Threads REQUIRED)

# ---- physics library (no graphics dependencies) ----
add_library(physics STATIC
  Core/EntityManager.cpp
  Collision/Collision.cpp
  Collision/AABBTree.cpp
  Solver/ConstraintSolver.cpp
  Pipeline/Integrator.cpp
  Pipeline/ThreadPool.cpp
  Pipeline/World.cpp)
target_include_directories(physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(physics PUBLIC Threads::Threads)
if(MSVC)
  target_compile_options(physics PRIVATE /W4)
else()
  target_compile_options(physics PRIVATE -Wall -Wextra)
endif()

# ---- tests ----
if(BUILD_TESTS)
  enable_testing()
  add_executable(physics_tests tests/tests.cpp)
  target_link_libraries(physics_tests PRIVATE physics)
  add_test(NAME physics_tests COMMAND physics_tests)
endif()

# ---- viewer ----
if(BUILD_VIEWER)
  include(FetchContent)
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4)
  FetchContent_MakeAvailable(glfw)
  find_package(OpenGL REQUIRED)

  add_executable(physics_viewer main.cpp Render/Renderer.cpp)
  target_link_libraries(physics_viewer PRIVATE physics glfw OpenGL::GL)
endif()
