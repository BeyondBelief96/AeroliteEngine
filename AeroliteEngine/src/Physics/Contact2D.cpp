#include "Contact2D.h"
#include <iostream>

namespace Aerolite {

    void Aerolite::Contact2D::ResolvePenetration(void)
    {
        if(a->IsStatic() && b->IsStatic()) return;
        
        float da = depth / (a->invMass + b->invMass) * a->invMass;
        float db = depth / (a->invMass + b->invMass) * b->invMass;
        
        a->position -= normal * da;
        b->position += normal * db;
    }

    void Contact2D::ResolveCollision()
    {
        // Apply positional correction using the projection method.
        // This is usually done to prevent objects from sinking into each other due to floating point inaccuracies.
        ResolvePenetration();

        // Calculate the coefficient of restitution, 'e', which is a measure of how bouncy the collision is.
        // This is determined by taking the minimum of the restitution coefficients of the two colliding objects.
        // A higher restitution value results in a bouncier collision.
        float e = std::min(a->restitution, b->restitution);

        // Compute the relative velocity vector of the two objects.
        // This is used to determine the direction and magnitude of the impulse to be applied.
        const Vec2 vrel = (a->velocity - b->velocity);

        // Calculate the dot product of the relative velocity and the collision normal.
        // This value is used in calculating the impulse magnitude.
        float vrelDotNormal = vrel.Dot(normal);

        // Define the direction of the impulse as the collision normal.
        const Vec2 impulseDirection = normal;
        // Calculate the magnitude of the impulse to be applied to each object.
        // The formula incorporates the relative velocity, the coefficient of restitution, and the inverse mass of the objects.
        const float impulseMagnitude = -(1 + e) * vrelDotNormal / (a->invMass + b->invMass);
        Vec2 j = impulseDirection * impulseMagnitude;

        // Apply the calculated impulse to both objects.
        // This changes their velocities according to their mass and the impulse magnitude and direction.
        a->ApplyImpulse(j);
        b->ApplyImpulse(-j);

    }

}
