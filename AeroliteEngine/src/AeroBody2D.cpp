#include "AeroBody2D.h"
#include "Precision.h"

static Aerolite::aero_uint16 current_id = 0;

namespace Aerolite {

    // Constructor for AeroBody2D.
    // Takes a pointer to a Shape, position coordinates (x, y), and mass.
    AeroBody2D::AeroBody2D(const std::shared_ptr<Shape>& shape, const real x, const real y, const real mass)
	    : position({x, y}), mass(mass), shape(shape)
    {
        this->id = current_id++;
        this->linear_velocity = AeroVec2(0.0f, 0.0f);
        this->linear_acceleration = AeroVec2(0.0f, 0.0f);
        this->rotation = 0.0f;
        this->angular_velocity = 0.0f;
        this->angular_acceleration = 0.0f;
        this->linear_damping = 0.002;
        this->angular_damping = 0.002;
        this->sum_forces = AeroVec2(0.0f, 0.0f);
        this->sum_torque = 0.0f;
        this->restitution = 0.5;
        this->friction = make_real<real>(0.7);
        this->is_sleeping = false;
        this->sleep_timer = 0;

        if(mass != 0.0) {
            this-> inv_mass = make_real<real>(1.0) / mass;
        } else {
            this->inv_mass = 0.0;
        }

        this->inertia = shape->GetMomentOfInertia() * this->mass;

        if(inertia != 0.0) {
            this-> inv_inertia = make_real<real>(1.0) / inertia;
        } else {
            inv_inertia = 0.0;
        }

        shape->UpdateVertices(rotation, position);
    }

    AeroAABB2D AeroBody2D::GetAABB() const
    {
        AeroAABB2D aabb;
        if (shape->GetType() == Circle)
        {
	        const auto circleShape = std::dynamic_pointer_cast<CircleShape>(shape);
	        const real radius = circleShape->radius;
            aabb.min = position - AeroVec2(radius, radius);
            aabb.max = position + AeroVec2(radius, radius);

        }
        else if (shape->GetType() == Box || shape->GetType() == Polygon)
        {
	        const auto polygon = std::dynamic_pointer_cast<PolygonShape>(shape);
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

        // Find the linear_acceleration based on the forces that are being applied this frame.
        linear_acceleration = sum_forces * inv_mass;

        // Integrate the linear_acceleration to find the new linear_velocity.
        linear_velocity += linear_acceleration * dt;
        linear_velocity *= RealPow(0.98f, linear_damping);

        // Integrate the torques to find the new angular linear_acceleration.
        angular_acceleration = sum_torque * inv_inertia;

        // Integrate the angular linear_acceleration to find the new angular linear_velocity.
        angular_velocity += angular_acceleration * dt;
        angular_velocity *= RealPow(0.98f, angular_damping);

        // Clear all forces and torque acting on the body before the next physics simulation frame.
        ClearForces();
        ClearTorque();
    }

    void AeroBody2D::IntegrateVelocities(const real dt)
    {
        if (IsStatic()) return;

        // Integrate the linear_velocity to find the new position.
        position += linear_velocity * dt + (linear_acceleration * dt * dt) / 2.0f;

        // Integrate the angular linear_velocity to find the new rotation angle.
        rotation += angular_velocity * dt;

        // Update the vertices of the shape based on the position/rotation.
        shape->UpdateVertices(rotation, position);
    }

    bool AeroBody2D::IsStatic(void) const
    {
        return AreEqual(mass, 0.0, EPSILON);
    }

    // Method to add a force vector to the body.
    void AeroBody2D::AddForce(const AeroVec2 &force)
    {
        sum_forces += force;
    }

    // Method to add a torque to the body.
    void AeroBody2D::AddTorque(const real torque)
    {
        sum_torque += torque;
    }

    void AeroBody2D::ApplyImpulseLinear(const AeroVec2& j)
    {
        if (IsStatic()) return;
        linear_velocity += j * inv_mass;
    }

    void AeroBody2D::ApplyImpulseAngular(const real j)
    {
        if (IsStatic()) return;
        angular_velocity += j * inv_inertia;
    }

    void AeroBody2D::ApplyImpulseAtPoint(const AeroVec2& j, const AeroVec2& r)
    {
        if (IsStatic()) return;

        linear_velocity += j * inv_mass;
        angular_velocity += r.Cross(j) * inv_inertia;
    }

    void AeroBody2D::SetFriction(const real f)
    {
	    this->friction = f;
    }

    void AeroBody2D::SetRestitution(const real r)
    {
        this->restitution = r;
    }

    void AeroBody2D::Sleep()
    {
        if(!IsStatic())
        {
            is_sleeping = true;
            linear_velocity = { 0, 0 };
            angular_velocity = 0;
            ClearForces();
            ClearTorque();
        }
    }

    void AeroBody2D::Awake()
    {
        is_sleeping = false;
        sleep_timer = 0;
    }

    // Method to clear all forces acting on the body.
    void AeroBody2D::ClearForces(void)
    {
        sum_forces = AeroVec2(0, 0);
    }

    // Method to clear all torques acting on the body.
    void AeroBody2D::ClearTorque(void)
    {
        sum_torque = 0;
    }

    AeroVec2 AeroBody2D::LocalSpaceToWorldSpace(const AeroVec2& point) const
    {
        const AeroVec2 rotated = point.Rotate(rotation);
        return rotated + position;
    }

    AeroVec2 AeroBody2D::WorldSpaceToLocalSpace(const AeroVec2& point) const
    {
        const real translatedX = point.x - position.x;
        const real translatedY = point.y - position.y;
        const real rotatedX = cos(-rotation) * translatedX - sin(-rotation) * translatedY;
        const real rotatedY = cos(-rotation) * translatedY + sin(-rotation) * translatedX;
        return AeroVec2(rotatedX, rotatedY);
    }
}
