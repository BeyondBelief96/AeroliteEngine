#ifndef SHAPE_H
#define SHAPE_H

#include <vector>
#include "Vec2.h"
#include "Precision.h"

using Vec2 = Aerolite::Vec2;

namespace Aerolite {

    // Enumeration of available shape types.
    enum ShapeType {
        Circle,
        Box,
        Polygon
    };

    // Abstract base class for shapes. 
    // Defines the common interface for all concrete shape classes.
    struct Shape {
        virtual ~Shape() = default; // Virtual destructor for proper cleanup of derived classes.
        virtual ShapeType GetType() const = 0; // Pure virtual function to get the type of shape.
        virtual real GetMomentOfInertia() const = 0; // Pure virtual function to calculate the moment of inertia.
    };

    // CircleShape class inheriting from Polygon.
    struct CircleShape : public Shape {
            real radius; // Radius of the circle.

            CircleShape() = default; // Default constructor
            CircleShape(const real radius); // Constructor to initialize the circle with a radius.
            virtual ~CircleShape(); // Destructor.
            virtual ShapeType GetType() const override; // Override to return ShapeType::Circle.
            virtual real GetMomentOfInertia() const override; // Override to calculate moment of inertia for a circle.

    };

        // PolygonShape class inheriting from Shape.
    struct PolygonShape : public Shape {
            std::vector<Vec2> localVertices; // List of vertices defining the polygon in model space.
            std::vector<Vec2> worldVertices; // List of the polygons vertices in world space. Changes as the model transforms.

            PolygonShape() = default; // Default constructor
            PolygonShape(const std::vector<Vec2>& vertices); // Constructor to initialize the polygon with vertices.
            virtual ~PolygonShape(); // Destructor.
            virtual ShapeType GetType() const override; // Override to return ShapeType::Polygon.
            virtual real GetMomentOfInertia() const override; // Override to calculate moment of inertia for a polygon.
            Aerolite::Vec2 EdgeAt(int index) const; // Given a vertex index, finds a Vec2 representing an edge of the polygon 
                                                   // from the given vertex index to index + 1.

            // Updates the polygons vertices with the given rotation and translation.
            // Transforms the polygon from model space to world space.
            void UpdateVertices(const real angle, const Vec2& position);

    };

    // BoxShape class inheriting from Polygon.
    struct BoxShape : public PolygonShape {
            real width; // Width of the box.
            real height; // Height of the box.

            BoxShape() = default; // Default constructor
            BoxShape(const real width, const real height); // Constructor to initialize the box with width and height.
            virtual ~BoxShape(); // Destructor.
            virtual ShapeType GetType() const override; // Override to return ShapeType::Box.
            virtual real GetMomentOfInertia() const override; // Override to calculate moment of inertia for a box.

    };
}

#endif
