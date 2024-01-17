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
        virtual void UpdateVertices(Aerolite::real angle, const Aerolite::Vec2& position) = 0; // Used to update the vertices of the shape.
    };

    // CircleShape class inheriting from Polygon.
    struct CircleShape : public Shape {
            real radius; // Radius of the circle.

            CircleShape() = default; // Default constructor
            CircleShape(const real radius); // Constructor to initialize the circle with a radius.
            virtual ~CircleShape(); // Destructor.
            virtual ShapeType GetType() const override; // Override to return ShapeType::Circle.
            virtual real GetMomentOfInertia() const override; // Override to calculate moment of inertia for a circle.
            virtual void UpdateVertices(Aerolite::real angle, const Aerolite::Vec2& position) override;

    };

        // PolygonShape class inheriting from Shape.
    struct PolygonShape : public Shape {
            std::vector<Vec2> localVertices;
            std::vector<Vec2> worldVertices; 

            PolygonShape() = default; 
            PolygonShape(const std::vector<Vec2>& vertices);
            virtual ~PolygonShape(); // Destructor.
            virtual ShapeType GetType() const override;
            virtual real GetMomentOfInertia() const override;        
            virtual void UpdateVertices(const real angle, const Vec2& position) override; 
            Aerolite::Vec2 EdgeAt(int index) const; 
            Aerolite::Vec2 GeometricCenter(void) const;
            int FindIncidentEdgeIndex(const Aerolite::Vec2& referenceEdge);
            int ClipLineSegmentToLine(const std::vector<Vec2>& contactsIn, std::vector<Vec2>& contactsOut, const Vec2& c0, const Vec2& c1) const;
            static PolygonShape* CreateRegularPolygon(int sides, real sideLength);
            Aerolite::real FindMinimumSeparation(const Aerolite::PolygonShape& other, int& indexReferenceEdge, Vec2& supportPoint) const;
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
