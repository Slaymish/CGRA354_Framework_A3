# Boids simulation

Coursework for CGRA354 computer graphics, assignment 3: a flocking simulation in C++ and OpenGL, with an ImGui panel for changing the forces while it runs.

## What it does

The scene has boids, a predator and static obstacles. Each boid steers by separation, alignment and cohesion, weighted by the panel sliders, and avoids obstacles with a repulsion force. The predator chases the nearest boid and the flock reacts to it. Rendering is OpenGL with a skybox, and the models are in `work/res/models/`.

`images/` holds three screenshots: the core requirement, the completion task and the challenge extension.

## Building it

Needs CMake and a C++ toolchain. GLEW, GLFW, GLM, ImGui and stb are vendored under `work/ext/`, so there is nothing to install on macOS.

```bash
cd work
mkdir -p build && cd build
cmake ..
make
./bin/a3
```

## State

Submitted in May 2024 and unchanged since, apart from this README. It is a record of the assignment rather than a maintained project.

MIT licensed.
