#ifndef PFGEN_H
#define PFGEN_H

#include <vector>
#include <memory>
#include <algorithm>
#include "Particle2D.h"
#include "Precision.h"
#include "AeroVec2.h"

namespace Aerolite {

	struct Particle2DForceGenerators {
        static AeroVec2 GenerateDragForce(const Particle2D& particle, real k1, real k2) {
            AeroVec2 dragForce = AeroVec2(0, 0);
            // Only apply drag if the particle is moving (velocity magnitude is greater than zero).
            if (particle.velocity.MagnitudeSquared() > 0) {
                // Calculate the drag force direction, opposite to the particle's velocity.
                dragForce = particle.velocity.UnitVector() * -1.0;

                // Compute the magnitude of drag using the drag equation, which includes linear and quadratic components.
                real dragMagnitude = particle.velocity.Magnitude() * k1 + particle.velocity.MagnitudeSquared() * k2;

                // Scale the drag force by its magnitude.
                dragForce *= dragMagnitude;
            }

            return dragForce;
        }

        static AeroVec2 GenerateFrictionForce(const Particle2D& particle, real coefficientOfFriction) {
            // Calculate the direction of friction force, which is opposite to the particle's velocity direction.
            AeroVec2 frictionDirection = particle.velocity.UnitVector() * -1.0;

            // The magnitude of the friction force is determined by the friction coefficient.
            real frictionMagnitude = coefficientOfFriction;

            // Construct the friction force vector by combining its direction and magnitude.
            AeroVec2 frictionForce = frictionDirection * frictionMagnitude;

            return frictionForce;
        }

        static AeroVec2 GenerateGravitationalAttractionForce(const Particle2D& a, const Particle2D& b, real minDistance, real maxDistance, real gravConstant) {
            // Calculate the displacement vector between particleB and particleA.
            AeroVec2 d = (b.position - a.position);

            // Compute the squared distance between the particles.
            real distanceSquared = d.MagnitudeSquared();

            // Clamp the distance to be within the specified min and max range to avoid extreme forces.
            /*distanceSquared = std::clamp(distanceSquared, (real)minDistance, (real)maxDistance);*/

            // Calculate the magnitude of gravitational attraction using Newton's law of universal gravitation.
            real attractionMagnitude = gravConstant * (a.mass * b.mass) / distanceSquared;

            // Construct the gravitational force vector, directed towards particleB.
            AeroVec2 attractionForce = d.UnitVector() * attractionMagnitude;

            return attractionForce;
        }

        static AeroVec2 GenerateAnchoredSpringForce(const Particle2D& particle, const AeroVec2& anchor, real restLength, real springConstant) {
            // Calculate the distance between the anchor and the particle.
            AeroVec2 d = particle.position - anchor;

            // Find the spring displacement considering the rest length;
            real displacement = d.Magnitude() - restLength;

            // Calculate the direction & magnitude of the spring force.
            AeroVec2 springDirection = d.UnitVector();
            real springMagnitude = -springConstant * displacement;

            // Calculate the final resulting spring force vector.
            AeroVec2 springForce = springDirection * springMagnitude;
            return springForce;
        }

        static AeroVec2 GenerateSpringForce(const Particle2D& a, const Particle2D& b, real restLength, real springConstant) {
            // Calculate the distance between the anchor and the particle.
            AeroVec2 d = a.position - b.position;

            // Find the spring displacement considering the rest length;
            real displacement = d.Magnitude() - restLength;

            // Calculate the direction & magnitude of the spring force.
            AeroVec2 springDirection = d.UnitVector();
            real springMagnitude = -springConstant * displacement;

            // Calculate the final resulting spring force vector.
            AeroVec2 springForce = springDirection * springMagnitude;
            return springForce;
        }
    };
}

#endif // PFGEN_H
