# AeroLite Physics Engine

AeroLite is a lightweight 2D physics engine designed to facilitate game development and graphical simulations. This engine provides a comprehensive set of features and algorithms to simulate realistic interactions between objects in a 2D space, with plans for future expansions into 3D physics.

## Features

### Collision Detection

AeroLite employs efficient algorithms for detecting collisions between various shapes, including circles, rectangles, and polygons. The collision detection system ensures accurate and reliable handling of object interactions.

### Rigid Body Dynamics

Simulate rigid body motion with AeroLite, allowing objects to move, rotate, and interact with each other based on physical principles. The engine supports a variety of shapes, enabling diverse and dynamic gameplay scenarios.

### Gravity Simulation

Gravity plays a crucial role in creating a realistic environment within your simulations. AeroLite implements gravitational forces based on Newton's law of universal gravitation, allowing objects to attract each other with appropriate forces.

### Particle System

A basic particle system is included in AeroLite, providing a framework for simulating individual particles with customizable forces and interactions. This feature is particularly useful for creating dynamic effects within your graphical simulations.

## Algorithms and Techniques

### Collision Resolution

AeroLite utilizes impulse-based resolution techniques to handle collisions and resolve interpenetrations between objects. This ensures stable and accurate collision responses, crucial for creating realistic and visually appealing simulations.

### Verlet Integration

For stable and accurate simulation of rigid body dynamics, AeroLite employs the Verlet integration method. This technique ensures the preservation of energy and stability during the simulation, leading to more realistic motion.

### Gravity Calculation

The engine calculates gravitational forces between objects using Newton's law of universal gravitation. This allows you to create engaging scenarios where objects interact with each other through gravitational attraction.

## Getting Started

### Installation

Clone the repository to your local machine:

```bash
git clone https://github.com/your-username/aerolite-physics.git
