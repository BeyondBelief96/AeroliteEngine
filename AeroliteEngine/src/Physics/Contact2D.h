#ifndef CONTACT2D_H
#define CONTACT2D_H

#include "Vec2.h"
#include "Body2D.h"
#include "Precision.h"

namespace Aerolite {

    // Represents a 2D contact point between two bodies in a physics simulation.
    // This struct is used to store information about the collision between two bodies.
    class Contact2D {

    public:
        // Pointers to the bodies involved in the contact.
        // These are raw pointers because Contact2D does not own the bodies.
        // The bodies are presumably managed elsewhere in your physics engine.
        Aerolite::Body2D* a = nullptr;
        Aerolite::Body2D* b = nullptr;

        // The start point of the contact in world coordinates.
        Aerolite::Vec2 start;

        // The end point of the contact in world coordinates. 
        Aerolite::Vec2 end;

        // A normalized vector representing the collision normal.
        // This vector points in the direction of the collision and is typically used to resolve the collision.
        Aerolite::Vec2 normal;

        // The depth of penetration between the two bodies.
        // A positive value typically indicates the amount by which the bodies are overlapping.
        Aerolite::real depth = 0;

        // Default constructor.
        Contact2D() = default;

        // Default destructor.
        // No custom destructor needed as raw pointers 'a' and 'b' do not own the bodies.
        ~Contact2D() = default;

        // This function resolves a collision between two objects in a 2D environment.
        // It applies both positional correction and an impulse based on the objects' properties.
        void ResolveCollision(void);

        // Copy Constructor
        Contact2D(const Contact2D& other)
        {
            a = other.a;
            b = other.b;
            start = other.start;
            end = other.end;
            normal = other.normal;
            depth = other.depth;
        }

        // Copy assignment operator.
        Contact2D& operator=(const Contact2D& other) {
            if (this != &other) {
                a = other.a;
                b = other.b;
                start = other.start;
                end = other.end;
                normal = other.normal;
                depth = other.depth;
            }
            return *this;
        }

        // Move constructor.
        Contact2D(Contact2D&& other) noexcept
            : a(other.a), b(other.b), start(std::move(other.start)), end(std::move(other.end)),
            normal(std::move(other.normal)), depth(other.depth) {
            other.a = nullptr;
            other.b = nullptr;
        }

        // Move assignment operator.
        Contact2D& operator=(Contact2D&& other) noexcept {
            if (this != &other) {
                a = other.a;
                b = other.b;
                start = std::move(other.start);
                end = std::move(other.end);
                normal = std::move(other.normal);
                depth = other.depth;

                other.a = nullptr;
                other.b = nullptr;
            }
            return *this;
        }

    private:
        // Resolves the penetration between the two Body2Ds.
        // This function adjusts the positions and/or velocities of the bodies to resolve the collision.
        void ResolvePenetration(void);

        /// <summary>
        /// This function resolves the impulse to apply to each body based on the linear and angular velocities
        /// of each body.
        /// </summary>
        void ResolveImpulse(void);
    };
} // namespace Aerolite

#endif
