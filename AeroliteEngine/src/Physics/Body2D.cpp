#include <iostream>
#include <math.h>
#include "Body2D.h"
#include "Precision.h"

namespace Aerolite {

    // Constructor for Body2D.
    // Takes a pointer to a Shape, position coordinates (x, y), and mass.
    Body2D::Body2D(Shape* shape, const Aerolite::real x, const Aerolite::real y, const Aerolite::real mass)
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

    AABB2D Body2D::GetAABB(void)
    {
        AABB2D aabb;
        if (shape->GetType() == ShapeType::Circle)
        {
            CircleShape* circleShape = dynamic_cast<CircleShape*>(shape);
            Aerolite::real radius = circleShape->radius;
            aabb.min = position - Vec2(radius, radius);
            aabb.max = position + Vec2(radius, radius);

        }
        else if (shape->GetType() == ShapeType::Box || shape->GetType() == ShapeType::Polygon)
        {
            PolygonShape* polygon = dynamic_cast<PolygonShape*>(shape);
            aabb.min = Vec2(std::numeric_limits<Aerolite::real>::max(), std::numeric_limits<Aerolite::real>::max());
            aabb.max = Vec2(std::numeric_limits<Aerolite::real>::lowest(), std::numeric_limits<Aerolite::real>::lowest());

            for (const auto& vertex : polygon->worldVertices)
            {
                aabb.min.x = std::min(aabb.min.x, vertex.x);
                aabb.min.y = std::min(aabb.min.y, vertex.y);
                aabb.max.x = std::max(aabb.max.x, vertex.x);
                aabb.max.y = std::max(aabb.max.y, vertex.y);
            }
        }

        return aabb;
    }

    // Method to update the body's state over time (dt).
    void Body2D::Update(const Aerolite::real dt)
    {
        IntegrateLinear(dt);
        IntegrateAngular(dt);
        bool isPolygon = shape->GetType() == Polygon || shape->GetType() == Box;

        if(isPolygon) {
            auto polygonShape = (PolygonShape*) shape;
            polygonShape->UpdateVertices(rotation, position);
        }
    }

    // Destructor for Body2D.
    Body2D::~Body2D()
    {
        delete(shape);
        shape = nullptr;
        std::cout << "Destroying Body2D!" << std::endl;
    }

    bool Body2D::IsStatic(void) const
    {
        return Aerolite::AreEqual(mass, 0.0, Aerolite::epsilon);
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
        position += velocity * dt + (acceleration * dt * dt) / 2.0f;

        // Clear all the forces on the body before next frame.
        ClearForces();
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
