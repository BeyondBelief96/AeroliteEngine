#include "AeroBody2D.h"
#include "Precision.h"


namespace Aerolite {

    // Constructor for AeroBody2D.
    // Takes a pointer to a Shape, position coordinates (x, y), and mass.
    AeroBody2D::AeroBody2D(Shape* shape, const real x, const real y, const real mass)
    {
        this->shape = shape;
        this->position = AeroVec2(x, y);
        this->velocity = AeroVec2(0.0f, 0.0f);
        this->acceleration = AeroVec2(0.0f, 0.0f);
        this->mass = mass;
        this->rotation = 0.0f;
        this->angularVelocity = 0.0f;
        this->angularAcceleration = 0.0f;
        this->sumForces = AeroVec2(0.0f, 0.0f);
        this->sumTorque = 0.0f;
        this->restitution = 0.5;
        this->friction = make_real<real>(0.7);

        if(mass != 0.0) {
            this-> invMass = make_real<real>(1.0) / mass;
        } else {
            this->invMass = 0.0;
        }

        this->I = shape->GetMomentOfInertia() * this->mass;

        if(I != 0.0) {
            this-> invI = make_real<real>(1.0) / I;
        } else {
            invI = 0.0;
        }

        shape->UpdateVertices(rotation, position);
    }

    AABB2D AeroBody2D::GetAABB() const
    {
        AABB2D aabb;
        if (shape->GetType() == Circle)
        {
	        const CircleShape* circleShape = dynamic_cast<CircleShape*>(shape);
	        const real radius = circleShape->radius;
            aabb.min = position - AeroVec2(radius, radius);
            aabb.max = position + AeroVec2(radius, radius);

        }
        else if (shape->GetType() == Box || shape->GetType() == Polygon)
        {
	        const PolygonShape* polygon = dynamic_cast<PolygonShape*>(shape);
            aabb.min = AeroVec2(std::numeric_limits<real>::max(), std::numeric_limits<real>::max());
            aabb.max = AeroVec2(std::numeric_limits<real>::lowest(), std::numeric_limits<real>::lowest());

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

    void AeroBody2D::IntegrateForces(const real dt)
    {
        if (IsStatic()) return;

        // Find the acceleration based on the forces that are being applied this frame.
        acceleration = sumForces * invMass;

        // Integrate the acceleration to find the new velocity.
        velocity += acceleration * dt;

        // Integrate the torques to find the new angular acceleration.
        angularAcceleration = sumTorque * invI;

        // Integrate the angular acceleration to find the new angular velocity.
        angularVelocity += angularAcceleration * dt;

        // Clear all forces and torque acting on the body before the next physics simulation frame.
        ClearForces();
        ClearTorque();
    }

    void AeroBody2D::IntegrateVelocities(const real dt)
    {
        if (IsStatic()) return;

        // Integrate the velocity to find the new position.
        position += velocity * dt + (acceleration * dt * dt) / 2.0f;

        // Integrate the angular velocity to find the new rotation angle.
        rotation += angularVelocity * dt;

        // Update the vertices of the shape based on the position/rotation.
        shape->UpdateVertices(rotation, position);
    }

    // Destructor for AeroBody2D.
    AeroBody2D::~AeroBody2D()
    {
        delete(shape);
        shape = nullptr;
    }

    bool AeroBody2D::IsStatic(void) const
    {
        return AreEqual(mass, 0.0, epsilon);
    }

    // Method to add a force vector to the body.
    void AeroBody2D::AddForce(const AeroVec2 &force)
    {
        sumForces += force;
    }

    // Method to add a torque to the body.
    void AeroBody2D::AddTorque(const real torque)
    {
        sumTorque += torque;
    }

    void AeroBody2D::ApplyImpulseLinear(const AeroVec2& j)
    {
        if (IsStatic()) return;
        velocity += j * invMass;
    }

    void AeroBody2D::ApplyImpulseAngular(const real j)
    {
        if (IsStatic()) return;
        angularVelocity += j * invI;
    }

    void AeroBody2D::ApplyImpulseAtPoint(const AeroVec2& j, const AeroVec2& r)
    {
        if (IsStatic()) return;

        velocity += j * invMass;
        angularVelocity += r.Cross(j) * invI;
    }

    // Method to clear all forces acting on the body.
    void AeroBody2D::ClearForces(void)
    {
        sumForces = AeroVec2(0, 0);
    }

    // Method to clear all torques acting on the body.
    void AeroBody2D::ClearTorque(void)
    {
        sumTorque = 0;
    }

    AeroVec2 AeroBody2D::LocalSpaceToWorldSpace(const AeroVec2& point)
    {
        AeroVec2 rotated = point.Rotate(rotation);
        return rotated + position;
    }

    AeroVec2 AeroBody2D::WorldSpaceToLocalSpace(const AeroVec2& point)
    {
        real translatedX = point.x - position.x;
        real translatedY = point.y - position.y;
        real rotatedX = cos(-rotation) * translatedX - sin(-rotation) * translatedY;
        real rotatedY = cos(-rotation) * translatedY + sin(-rotation) * translatedX;
        return AeroVec2(rotatedX, rotatedY);
    }
}
