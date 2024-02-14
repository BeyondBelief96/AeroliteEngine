#ifndef SHAPE_H
#define SHAPE_H

#include <vector>
#include <memory>
#include "AeroVec2.h"
#include "Precision.h"

using Vec2 = Aerolite::AeroVec2;

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
        virtual void UpdateVertices(real angle, const AeroVec2& position) = 0; // Used to update the vertices of the shape.
    };

    // CircleShape class inheriting from Polygon.
    struct CircleShape final : public Shape {
            real radius; // Radius of the circle.

            CircleShape() = default; // Default constructor
            CircleShape(const real radius); // Constructor to initialize the circle with a radius.
            virtual ShapeType GetType() const override; // Override to return ShapeType::Circle.
            virtual real GetMomentOfInertia() const override; // Override to calculate moment of inertia for a circle.
            virtual void UpdateVertices(real angle, const AeroVec2& position) override;

    };

        // PolygonShape class inheriting from Shape.
    struct PolygonShape : public Shape {
            std::vector<AeroVec2> localVertices;
            std::vector<AeroVec2> worldVertices; 

            PolygonShape() = default; 
            PolygonShape(const std::vector<AeroVec2>& vertices);
            virtual ShapeType GetType() const override;
            virtual real GetMomentOfInertia() const override;        
            virtual void UpdateVertices(const real angle, const AeroVec2& position) override;
            AeroVec2 EdgeAt(int index) const;
            AeroVec2 GeometricCenter(void) const;
            int FindIncidentEdgeIndex(const AeroVec2& referenceEdge) const;
            static int ClipLineSegmentToLine(const std::vector<AeroVec2>& contactsIn, std::vector<AeroVec2>& contactsOut, const AeroVec2& c0, const AeroVec2& c1);
            static std::shared_ptr<PolygonShape> CreateRegularPolygon(int sides, real sideLength);
            real FindMinimumSeparation(const PolygonShape& other, int& indexReferenceEdge, AeroVec2& supportPoint) const;
    };

    // BoxShape class inheriting from Polygon.
    struct BoxShape : public PolygonShape {
            real width; // Width of the box.
            real height; // Height of the box.

            BoxShape() = default; // Default constructor
            BoxShape(real width, real height); // Constructor to initialize the box with width and height.
            virtual ShapeType GetType() const override; // Override to return ShapeType::Box.
            virtual real GetMomentOfInertia() const override; // Override to calculate moment of inertia for a box.

    };
}

#endif
