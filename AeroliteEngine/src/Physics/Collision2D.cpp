#include <iostream>
#include <limits>
#include "Collision2D.h"
#include "Shape.h"

namespace Aerolite
{
    // Function: IsColliding
    // Purpose: Determines if two 2D bodies are colliding.
    // Parameters:
    //   - a, b: Pointers to the two Body2D objects being checked for collision.
    //   - contact: A reference to a Contact2D object where collision details will be stored.
    // Description: This function first checks the shape types of the bodies (circle or polygon).
    //              It then delegates the collision detection to the appropriate function based
    //              on the shape types.
    bool CollisionDetection2D::IsColliding(Aerolite::Body2D* a, Aerolite::Body2D* b, Aerolite::Contact2D& contact)
    {
        // Checking if each body is a circle.
        bool aIsCircle = a->shape->GetType() == Aerolite::ShapeType::Circle;
        bool bIsCircle = b->shape->GetType() == Aerolite::ShapeType::Circle;

        // If both bodies are circles, check for circle-circle collision.
        if (aIsCircle && bIsCircle)
        {
            return IsCollidingCircleCircle(a, b, contact);
        }
        // If neither body is a circle, they are polygons. Check for polygon-polygon collision.
        else if (!aIsCircle && !bIsCircle)
        {
            return IsCollidingPolygonPolygon(a, b, contact);
        }

        // If one body is a circle and the other is a polygon, no collision detection is implemented.
        return false;
    }

    // Function: IsCollidingCircleCircle
    // Purpose: Checks for collision between two circles.
    // Parameters:
    //   - a, b: Pointers to the circle Body2D objects.
    //   - contact: A reference to a Contact2D object to store collision details.
    // Description: This function calculates if two circles are colliding by comparing
    //              the distance between their centers to the sum of their radii.
    bool CollisionDetection2D::IsCollidingCircleCircle(Aerolite::Body2D* a, Aerolite::Body2D* b, Aerolite::Contact2D& contact)
    {
        // Casting the shape of each body to a CircleShape to access circle-specific properties.
        auto aCircleShape = dynamic_cast<Aerolite::CircleShape*>(a->shape);
        auto bCircleShape = dynamic_cast<Aerolite::CircleShape*>(b->shape);

        // Calculating the vector distance between the centers of the two circles.
        const Vec2 distanceBetweenCenters = b->position - a->position;
        // Adding up the radii of both circles.
        const Aerolite::real sumRadius = aCircleShape->radius + bCircleShape->radius;
        // Checking if the squares of distances and radii sum indicate a collision.
        bool isColliding = distanceBetweenCenters.MagnitudeSquared() <= (sumRadius * sumRadius);

        // If not colliding, return false immediately.
        if (!isColliding) {
            return false;
        }

        // If colliding, populate the contact object with collision details.
        contact.a = a;
        contact.b = b;
        contact.normal = distanceBetweenCenters.UnitVector();
        contact.start = b->position - (contact.normal * bCircleShape->radius);
        contact.end = a->position + (contact.normal * aCircleShape->radius);
        contact.depth = (contact.end - contact.start).Magnitude();

        return true;
    }

    // Function: IsCollidingPolygonPolygon
    // Purpose: Checks for collision between two polygons using the Separating Axis Theorem (SAT).
    // Parameters:
    //   - a, b: Pointers to the polygon Body2D objects.
    //   - contact: A reference to a Contact2D object, not used in this function as SAT does not provide contact points.
    // Description: This function checks for overlap along all possible axes formed by the edges of the polygons.
    //              If a separating axis is found (no overlap on an axis), the polygons are not colliding.
    bool CollisionDetection2D::IsCollidingPolygonPolygon(Aerolite::Body2D* a, Aerolite::Body2D* b, Aerolite::Contact2D& contact)
    {
        // Casting the shape of each body to a PolygonShape to access polygon-specific properties.
        auto aPolygonShape = dynamic_cast<PolygonShape*>(a->shape);
        auto bPolygonShape = dynamic_cast<PolygonShape*>(b->shape);

        auto contactDepth = std::numeric_limits<Aerolite::real>::max();
        auto contactNormal = Aerolite::Vec2();

        // Checking for overlap along all axes formed by the edges of polygon a.
        for (int i = 0; i < aPolygonShape->worldVertices.size(); i++)
        {
            // Compute the normal to the edge.
            Aerolite::Vec2 normal = aPolygonShape->EdgeAt(i).Normal().UnitVector();

            // Initialize min and max projection values.
            auto minA = std::numeric_limits<Aerolite::real>::max();
            auto maxA = std::numeric_limits<Aerolite::real>::min();
            auto minB = std::numeric_limits<Aerolite::real>::max();
            auto maxB = std::numeric_limits<Aerolite::real>::min();

            // Find the min and max projections of both polygons onto the axis.
            FindMinMaxProjections(aPolygonShape->worldVertices, normal, minA, maxA);
            FindMinMaxProjections(bPolygonShape->worldVertices, normal, minB, maxB);

            // If projections do not overlap, there is a separating axis, so no collision.
            if (minA >= maxB || minB >= maxA) return false;

            auto axisDepth = std::min(maxB - minA, maxA - minB);

            if (axisDepth < contactDepth)
            {
                contactDepth = axisDepth;
                contactNormal = normal;
            }
        }

        // Repeat the process for all axes formed by the edges of polygon b.
        for (int i = 0; i < bPolygonShape->worldVertices.size(); i++)
        {
            Aerolite::Vec2 normal = bPolygonShape->EdgeAt(i).Normal().UnitVector();

            auto minA = std::numeric_limits<Aerolite::real>::max();
            auto maxA = std::numeric_limits<Aerolite::real>::min();
            auto minB = std::numeric_limits<Aerolite::real>::max();
            auto maxB = std::numeric_limits<Aerolite::real>::min();

            FindMinMaxProjections(aPolygonShape->worldVertices, normal, minA, maxA);
            FindMinMaxProjections(bPolygonShape->worldVertices, normal, minB, maxB);

            if (minA >= maxB || minB >= maxA) return false;

            auto axisDepth = std::min(maxB - minA, maxA - minB);

            if (axisDepth < contactDepth)
            {
                contactDepth = axisDepth;
                contactNormal = normal;
            }
        }

        // Ensuring that the normal of the contact points from body A to body B.
        Aerolite::Vec2 centerA = aPolygonShape->GeometricCenter();
        Aerolite::Vec2 centerB = bPolygonShape->GeometricCenter();

        Aerolite::Vec2 direction = centerB - centerA;
        if (direction.Dot(contactNormal) < 0.0f)
            contactNormal *= -1;

        contactDepth /= contactNormal.Magnitude();
        contactNormal = contactNormal;

        contact.a = a;
        contact.b = b;
        contact.depth = contactDepth;
        contact.normal = contactNormal;

        // If no separating axis is found, the polygons are colliding.
        return true;
    }

    void CollisionDetection2D::FindContactPoint(PolygonShape* shapeA, PolygonShape* shapeB, Aerolite::Vec2& contactPoint)
    {
        Aerolite::real minDist = std::numeric_limits<Aerolite::real>::max();

        for (int i = 0; i < shapeA->worldVertices.size(); i++)
        {
            Aerolite::Vec2 va = shapeA->worldVertices[i];
            for (int j = 0; j < shapeB->worldVertices.size(); j++)
            {
                Aerolite::Vec2 edge1 = shapeB->worldVertices[j];
                Aerolite::Vec2 edge2 = shapeB->worldVertices[(j + 1) % shapeB->worldVertices.size()];
                Aerolite::Vec2 closestPoint = Vec2();
                Aerolite::real distance = 0.0f;
                PointLineSegmentDistance(va, edge1, edge2, distance, closestPoint);

                if (distance == minDist)
                {

                }
                else if (distance < minDist)
                {
                    minDist = distance;
                    contactPoint = closestPoint;
                }
            }
        }
    }

    void CollisionDetection2D::PointLineSegmentDistance(Aerolite::Vec2 p, Aerolite::Vec2 linePointA, Aerolite::Vec2 linePointB, Aerolite::real& distance, Aerolite::Vec2& closestPoint)
    {
        Aerolite::Vec2 ab = linePointB - linePointA;
        Aerolite::Vec2 ap = p - linePointA;

        auto proj = ap.Dot(ab);
        auto abMagSquared = ab.MagnitudeSquared();
        auto d = proj / abMagSquared;

        if (d <= 0)
            closestPoint = linePointA;
        else if (d >= 1)
            closestPoint = linePointB;
        else
        {
            closestPoint = linePointA + ab * d;
        }

        distance = p.DistanceTo(closestPoint);
    }

    // Function: FindMinMaxProjections
    // Purpose: Finds the minimum and maximum scalar projections of a set of vertices onto a given axis.
    // Parameters:
    //   - vertices: A vector of Vec2 representing the vertices of a polygon.
    //   - axis: The axis onto which the vertices are to be projected.
    //   - min, max: References to store the minimum and maximum projections.
    // Description: This function is a helper for the Separating Axis Theorem. It projects each vertex onto the axis
    //              and keeps track of the minimum and maximum values of these projections.
    void CollisionDetection2D::FindMinMaxProjections(std::vector<Aerolite::Vec2> vertices, Aerolite::Vec2 axis,
        Aerolite::real& min, Aerolite::real& max)
    {
        for (int j = 0; j < vertices.size(); j++)
        {
            auto currentVertex = vertices[j];
            // Projecting the vertex onto the axis.
            auto projection = currentVertex.Dot(axis);

            // Updating the min and max values based on the projection.
            if (projection < min) {
                min = projection;
            }
            if (projection > max) {
                max = projection;
            }
                
        }
    }
}
