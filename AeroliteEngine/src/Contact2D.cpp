#include "Contact2D.h"
#include <iostream>

namespace Aerolite {

    void Contact2D::ResolveCollision()
    {
        // Apply positional correction using the projection method.
        // This is usually done to prevent objects from sinking into each other due to floating point inaccuracies.
        ResolvePenetration();

        // Resolve and apply the calculated impulse to each body.
        ResolveImpulse();
    }

    void Contact2D::ResolvePenetration(void) const
    {
        if(a->IsStatic() && b->IsStatic()) return;
        
        float da = depth / (a->inv_mass + b->inv_mass) * a->inv_mass;
        float db = depth / (a->inv_mass + b->inv_mass) * b->inv_mass;
        
        a->position -= (normal * da);
        b->position += (normal * db);

        a->shape->UpdateVertices(a->rotation, a->position);
        b->shape->UpdateVertices(b->rotation, b->position);
    }

    void Contact2D::ResolveImpulse()
    {
        // Calculate the coefficient of restitution, 'e', which is a measure of how bouncy the collision is.
        // The coefficient of restitution is the ratio of the final to initial relative linear_velocity 
        // between two objects after they collide. It ranges from 0 (perfectly inelastic collision, 
        // no bounce) to 1 (perfectly elastic collision).
        // Here, 'e' is determined by taking the minimum of the restitution coefficients 
        // of the two colliding objects.
        real e = std::min(a->restitution, b->restitution);
        real f = std::min(a->friction, b->friction);

        // Calculate the radius vectors from the center of mass of each object to the contact point.
        // These vectors are used in the torque calculations.
        AeroVec2 ra = end - a->position;
        AeroVec2 rb = start - b->position;

        // Calculate the velocities at the point of contact for each object.
        // This includes both linear and angular contributions.
        // The formula is: linear_velocity = linear_velocity + angular_velocity cross radius_vector
        AeroVec2 va = a->linear_velocity + AeroVec2(-a->angular_velocity * ra.y, a->angular_velocity * ra.x);
        AeroVec2 vb = b->linear_velocity + AeroVec2(-b->angular_velocity * rb.y, b->angular_velocity * rb.x);

        // Compute the relative linear_velocity at the point of contact.
        // This is the linear_velocity of object A relative to object B at the contact point.
        const AeroVec2 vrel = va - vb;

        // Calculate the dot product of the relative linear_velocity and the collision normal.
        // This value is a component of the relative linear_velocity along the normal direction,
        // which is used in calculating the impulse magnitude.
        float vrelDotNormal = vrel.Dot(normal);

        // Calculate the magnitude of the impulse to be applied to each object.
        // The impulse magnitude formula is derived from the law of conservation of momentum and
        // takes into account the masses and velocities of the colliding bodies, as well as the
        // coefficient of restitution.
        // The formula is: j = -(1 + e) * vrelDotNormal / (inv_massA + inv_massB + termA + termB)
        // where termA and termB are additional terms that account for the objects' rotational inertia.
        const float impulseMagnitudeN = -(1 + e) * vrelDotNormal / ((a->inv_mass + b->inv_mass)
            + (ra.Cross(normal) * ra.Cross(normal)) * a->inv_inertia
            + (rb.Cross(normal) * rb.Cross(normal)) * b->inv_inertia);

        // Calculate the normal component of the impulse.
        // jN is the impulse vector along the collision normal.
        // impulseMagnitudeN is the magnitude of the impulse calculated earlier, presumably using the collision normal.
        AeroVec2 jN = normal * impulseMagnitudeN;

        // Calculate the tangential component of the impulse, which is perpendicular to the normal.
        // This is used to simulate frictional effects at the point of collision.
        // First, obtain the tangent vector, which is perpendicular to the collision normal.
        const AeroVec2 tangent = normal.Normal();

        // Calculate the dot product of the relative linear_velocity and the tangent vector.
        // This represents the component of the relative linear_velocity in the direction of the tangent.
        real vrelDotTangent = vrel.Dot(tangent);

        // Calculate the magnitude of the tangential impulse.
        // This calculation is similar to the normal impulse, but uses the tangential component of the relative linear_velocity.
        // The coefficient 'f' represents the friction coefficient, which controls how much friction is applied.
        // The formula also includes restitution 'e' and the inverse masses and moment of inertia of the objects.
        const real impulseMagnitudeT = f * -(1 + e) * vrelDotTangent / ((a->inv_mass + b->inv_mass)
            + (ra.Cross(tangent) * ra.Cross(tangent)) * a->inv_inertia
            + (rb.Cross(tangent) * rb.Cross(tangent)) * b->inv_inertia);

        // Calculate the tangential impulse vector.
        // This is the impulse due to friction, acting along the tangent at the point of contact.
        AeroVec2 jT = tangent * impulseMagnitudeT;

        // Combine the normal and tangential impulses to get the total impulse to be applied.
        // This total impulse j is what will be applied to the objects to resolve the collision,
        // taking into account both collision response (normal impulse) and friction (tangential impulse).
        AeroVec2 j = jN + jT;

        a->ApplyImpulseAtPoint(j, ra);  // Apply impulse to object A
        b->ApplyImpulseAtPoint(-j, rb); // Apply opposite impulse to object B
    }
}
