# Aerolite Engine

PhysicsEngine is a lightweight 2D and 3D physics simulation library written in C++. It can be used to add physical behaviors like collisions, rigid body dynamics, and particle systems to games and other graphical applications. 

## About

Key features of PhysicsEngine:

- Uses a particle-based system to simulate physical objects
- Supports basic rigid bodies, joints, particle systems, and aggregate bodies
- Integration functions for simulating Newtonian physics
- Collision detection and response 
- Forces like gravity, drag, friction, springs, attraction, etc.
- Written in C++ for maximum performance

The goal of this project is to provide a simple yet powerful physics engine that can be easily integrated into games, simulations, and other graphical apps built with C/C++ and OpenGL. It aims to strike a balance between performance, flexibility, and ease-of-use.

## Installation

PhysicsEngine depends on a few libraries:

### Windows

- SDL2
- SDL2_image
- SDL2_gfx
- vcpkg (for managing dependencies)

Run the following in a command prompt to install required libraries using vcpkg:

```
vcpkg install sdl2 sdl2-image sdl2-gfx
```

Then build the project in your preferred IDE like Visual Studio.

### Mac/Linux

- SDL2 
- SDL2_image
- SDL2_gfx

Install these libraries using your preferred package manager like Homebrew or APT.

Then build the project using Make/CMake.

## Usage

See the scenes/ folder for sample code snippets showing how to:

- Create particles, rigid bodies, and joints
- Apply forces like gravity and springs 
- Perform collision detection
- Integrate physics over time
- Render physics objects using OpenGL

## Contributing

Contributions are welcome! Please create an issue or PR for any enhancements or fixes.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
