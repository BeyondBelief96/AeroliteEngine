#include <iostream>
#include <math.h>
#include "Body2D.h"
#include "Precision.h"

namespace Aerolite {

    // Constructor for Body2D.
    // Takes a shared pointer to Shape, position coordinates (x, y), and mass.
    Body2D::Body2D(const std::shared_ptr<Shape> shape, const Aerolite::real x, const Aerolite::real y, const Aerolite::real mass)
    {
        this->shape = shape;
        this->position = Vec2(x, y);
        this->velocity = Vec2(0.0f, 0.0f);
        this->acceleration = Vec2(0.0f, 0.0f);
        this->mass = mass;
        this->rotation = 0.0f;
        this->angularVelocity = 0.0f;
        this->angularAcceleration = 0.0f;
        this->sumForces = Vec2(0.0f, 0.0f);
        this->sumTorque = 0.0f;
        this->restitution = 0.99;

        if(mass != 0.0) {
            this-> invMass = 1.0 / mass;
        } else {
            this->invMass = 0.0;
        }

        this->I = shape->GetMomentOfInertia() * this->mass;

        if(I != 0.0) {
            this-> invI = 1.0 / I;
        } else {
            invI = 0.0;
        }

        std::cout << "Body constructor called!" << std::endl;
    }

    // Method to update the body's state over time (dt).
    void Body2D::Update(const Aerolite::real dt)
    {
        IntegrateLinear(dt);
        IntegrateAngular(dt);
        bool isPolygon = shape->GetType() == Polygon || shape->GetType() == Box;

        if(isPolygon) {
            auto polygonShape = std::dynamic_pointer_cast<PolygonShape>(shape);
            polygonShape->UpdateVertices(rotation, position);
        }
    }

    // Destructor for Body2D.
    Body2D::~Body2D()
    {
        std::cout << "Destroying Body2D!" << std::endl;
    }

    bool Body2D::IsStatic(void) const
    {
        return Aerolite::AreFloatsEqual(mass, 0.0, Aerolite::epsilon);
    }

    // Method to add a force vector to the body.
    void Body2D::AddForce(const Vec2 &force)
    {
        sumForces += force;
    }

    // Method to add a torque to the body.
    void Body2D::AddTorque(const Aerolite::real torque)
    {
        sumTorque += torque;
    }

    void Body2D::ApplyImpulse(const Vec2& j)
    {
        if (IsStatic()) return;

        velocity += j * invMass;
    }

    // Method to clear all forces acting on the body.
    void Body2D::ClearForces(void)
    {
        sumForces = Vec2(0, 0);
    }

    // Method to clear all torques acting on the body.
    void Body2D::ClearTorque(void)
    {
        sumTorque = 0;
    }

    // Method to integrate the body's linear position and velocity over time (dt).
    void Body2D::IntegrateLinear(const Aerolite::real dt)
    {
        if(IsStatic()) return;

        // Find the acceleration based on forces being applied.
        acceleration = sumForces * invMass;

        // Integrate acceleration to find velocity;
        velocity += acceleration * dt;

        // Integrate velocity to find position.
        position += velocity * dt;
    }

    // Method to integrate the body's angular position and angular velocity over time (dt).
    void Body2D::IntegrateAngular(const Aerolite::real dt)
    {
        if(IsStatic()) return;
        angularAcceleration = sumTorque * invI;
        angularVelocity += angularAcceleration * dt;
        rotation += angularVelocity * dt;
        ClearTorque();
    }
}
