#ifndef CONTACT2D_H
#define CONTACT2D_H

#include "AeroVec2.h"
#include "AeroBody2D.h"
#include "Precision.h"

namespace Aerolite {

    // Represents a 2D contact point between two bodies in our physics simulation.
    // This class is used to store information about the collision between two bodies.
    class Contact2D {

    public:
        // Pointers to the bodies involved in the contact.
        // These are raw pointers because Contact2D does not own the bodies.
        // The bodies are presumably managed elsewhere in your physics engine.
        AeroBody2D* a = nullptr;
        AeroBody2D* b = nullptr;

        // The start point of the contact in world coordinates.
        AeroVec2 start;

        // The end point of the contact in world coordinates. 
        AeroVec2 end;

        // A normalized vector representing the collision normal.
        // This vector points in the direction of the collision and is typically used to resolve the collision.
        AeroVec2 normal;

        // The depth of penetration between the two bodies.
        // A positive value typically indicates the amount by which the bodies are overlapping.
        real depth = 0;

        // Default constructor.
        Contact2D() = default;

        // Default destructor.
        // No custom destructor needed as raw pointers 'a' and 'b' do not own the bodies.
        ~Contact2D() = default;

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
    };
} // namespace Aerolite

#endif
