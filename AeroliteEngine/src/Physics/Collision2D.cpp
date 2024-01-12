#include <iostream>
#include <limits>
#include "Collision2D.h"
#include "Precision.h"
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
    bool CollisionDetection2D::IsColliding(Aerolite::Body2D& a, Aerolite::Body2D& b, Aerolite::Contact2D& contact)
    {
        if (a.IsStatic() && b.IsStatic()) return false;
        bool aIsCircle = a.shape->GetType() == Aerolite::ShapeType::Circle;
        bool bIsCircle = b.shape->GetType() == Aerolite::ShapeType::Circle;

        if (aIsCircle && bIsCircle)
        {
            return IsCollidingCircleCircle(a, b, contact);
        }
        else if (!aIsCircle && !bIsCircle)
        {
            return IsCollidingPolygonPolygon(a, b, contact);
        }
        else if (aIsCircle && !bIsCircle)
        {
            return IsCollidingCirclePolygon(b, a, contact);
        }
        else if (!aIsCircle && bIsCircle)
        {
            return IsCollidingCirclePolygon(a, b, contact);
        }

        return false;
    }


    // Function: IsCollidingCircleCircle
    // Purpose: Checks for collision between two circles.
    // Parameters:
    //   - a, b: Pointers to the circle Body2D objects.
    //   - contact: A reference to a Contact2D object to store collision details.
    // Description: This function calculates if two circles are colliding by comparing
    //              the distance between their centers to the sum of their radii.
    bool CollisionDetection2D::IsCollidingCircleCircle(Aerolite::Body2D& a, Aerolite::Body2D& b, Aerolite::Contact2D& contact)
    {
        auto aCircleShape = dynamic_cast<Aerolite::CircleShape*>(a.shape);
        auto bCircleShape = dynamic_cast<Aerolite::CircleShape*>(b.shape);

        const Vec2 distanceBetweenCenters = b.position - a.position;
        const Aerolite::real sumRadius = aCircleShape->radius + bCircleShape->radius;
        bool isColliding = distanceBetweenCenters.MagnitudeSquared() <= (sumRadius * sumRadius);

        if (!isColliding) {
            return false;
        }

        contact.a = &a;
        contact.b = &b;
        contact.normal = distanceBetweenCenters.UnitVector();
        contact.start = b.position - (contact.normal * bCircleShape->radius);
        contact.end = a.position + (contact.normal * aCircleShape->radius);
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
    bool CollisionDetection2D::IsCollidingPolygonPolygon(Aerolite::Body2D& a, Aerolite::Body2D& b, Aerolite::Contact2D& contact)
    {
        return IsCollidingSATOptimized(a, b, contact);
    }

    bool CollisionDetection2D::IsCollidingSATBruteForce(Aerolite::Body2D& a, Aerolite::Body2D& b, Aerolite::Contact2D& contact)
    {
        // Casting the shape of each body to a PolygonShape to access polygon-specific properties.
        auto aPolygonShape = dynamic_cast<PolygonShape*>(a.shape);
        auto bPolygonShape = dynamic_cast<PolygonShape*>(b.shape);
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

        contact.a = &a;
        contact.b = &b;
        contact.depth = contactDepth;
        contact.normal = contactNormal;

        FindContactPointsPolygons(*aPolygonShape, *bPolygonShape, contact);

        // If no separating axis is found, the polygons are colliding.
        return true;
    }

    // Check for collision between two polygon shapes using the Separating Axis Theorem (SAT).
    bool CollisionDetection2D::IsCollidingSATOptimized(Aerolite::Body2D& a, Aerolite::Body2D& b, Aerolite::Contact2D& contact)
    {
        // Cast the shapes of the bodies to polygon shapes
        auto polygonShapeA = dynamic_cast<PolygonShape*>(a.shape);
        auto polygonShapeB = dynamic_cast<PolygonShape*>(b.shape);

        // Variables to store the axis of minimum separation and the corresponding points
        Aerolite::Vec2 aAxis, bAxis;
        Aerolite::Vec2 aMinPoint, bMinPoint;

        // Find the minimum separation from A to B, along with the separation axis and point
        Aerolite::real abSeparation = polygonShapeA->FindMinimumSeparation(*polygonShapeB, aAxis, aMinPoint);
        // Find the minimum separation from B to A, along with the separation axis and point
        Aerolite::real baSeparation = polygonShapeB->FindMinimumSeparation(*polygonShapeA, bAxis, bMinPoint);

        // If either separation is non-negative, no overlap occurs, thus no collision
        if (abSeparation >= 0) return false;
        if (baSeparation >= 0) return false;

        // Set the colliding bodies in the contact structure
        contact.a = &a;
        contact.b = &b;

        // Determine which polygon has the greatest penetration and set the contact details
        if (abSeparation > baSeparation)
        {
            contact.depth = -abSeparation;
            contact.normal = aAxis.Normal();
            contact.start = aMinPoint;
            contact.end = aMinPoint + contact.normal * contact.depth;
        }
        else
        {
            contact.depth = -baSeparation;
            contact.normal = -bAxis.Normal();
            contact.start = bMinPoint - contact.normal * contact.depth;;
            contact.end = bMinPoint;
        }

        // Collision detected
        return true;
    }

    // Check for collision between a circle and a polygon.
    bool CollisionDetection2D::IsCollidingCirclePolygon(Aerolite::Body2D& polygon, Aerolite::Body2D& circle, Aerolite::Contact2D& contact)
    {
        // Cast the shapes to their respective types
        const Aerolite::PolygonShape* polygonShape = dynamic_cast<Aerolite::PolygonShape*>(polygon.shape);
        const Aerolite::CircleShape* circleShape = dynamic_cast<Aerolite::CircleShape*>(circle.shape);

        // Initialization of variables to track the closest edge and if the circle is outside the polygon
        bool isOutside = false;
        Aerolite::Vec2 minCurrVertex;
        Aerolite::Vec2 minNextVertex;
        Aerolite::real distanceToCircleEdge = std::numeric_limits<Aerolite::real>::lowest();

        // Iterate over each edge of the polygon
        for (int i = 0; i < polygonShape->worldVertices.size(); i++)
        {
            int currVertex = i;
            int nextVertex = (i + 1) % polygonShape->worldVertices.size();
            Aerolite::Vec2 edge = polygonShape->EdgeAt(currVertex);
            Aerolite::Vec2 normal = edge.Normal();

            // Compute vector from the current vertex to the circle's center
            Aerolite::Vec2 vertexToCircleCenter = circle.position - polygonShape->worldVertices[currVertex];

            // Project the vertex-to-center vector onto the edge's normal
            Aerolite::real projection = vertexToCircleCenter.Dot(normal);

            // Determine if the circle is outside the polygon and find the closest edge
            if (projection > 0) {
                distanceToCircleEdge = projection;
                minCurrVertex = polygonShape->worldVertices[currVertex];
                minNextVertex = polygonShape->worldVertices[nextVertex];
                isOutside = true;
                break;
            }
            else {
                if (projection > distanceToCircleEdge) {
                    distanceToCircleEdge = projection;
                    minCurrVertex = polygonShape->worldVertices[currVertex];
                    minNextVertex = polygonShape->worldVertices[nextVertex];
                }
            }
        }

        // Check for collision based on the region where the circle is relative to the closest edge
        if (isOutside)
        {
            // Handle collision detection for region A
            Vec2 v1 = circle.position - minCurrVertex;
            Vec2 v2 = minNextVertex - minCurrVertex;
            if (v1.Dot(v2) < 0) {
                if (v1.Magnitude() > circleShape->radius) {
                    return false;
                }
                else {
                    SetContactDetails(contact, polygon, circle, v1, circleShape->radius);
                }
            }
            else {
                // Handle collision detection for region B
                v1 = circle.position - minNextVertex;
                v2 = minCurrVertex - minNextVertex;
                if (v1.Dot(v2) < 0) {
                    if (v1.Magnitude() > circleShape->radius) {
                        return false;
                    }
                    else {
                        SetContactDetails(contact, polygon, circle, v1, circleShape->radius);
                    }
                }
                else {
                    // Handle collision detection for region C
                    if (distanceToCircleEdge > circleShape->radius) {
                        return false;
                    }
                    else {
                        SetContactDetailsForRegionC(contact, polygon, circle, minCurrVertex, minNextVertex, circleShape->radius, distanceToCircleEdge);
                    }
                }
            }
        }
        else {
            // Circle's center is inside the polygon, indicating a definite collision
            SetContactDetailsForInsideCollision(contact, polygon, circle, minCurrVertex, minNextVertex, circleShape->radius, distanceToCircleEdge);
        }

        // Collision detected
        return true;
    }

    // Helper functions for setting contact details (to avoid code repetition)
    void CollisionDetection2D::SetContactDetails(Aerolite::Contact2D& contact, Aerolite::Body2D& polygon, Aerolite::Body2D& circle, Aerolite::Vec2& v1, Aerolite::real radius) {
        contact.a = &polygon;
        contact.b = &circle;
        contact.depth = radius - v1.Magnitude();
        contact.normal = v1.UnitVector();
        contact.start = circle.position + (contact.normal * -radius);
        contact.end = contact.start + (contact.normal * contact.depth);
    }

    void CollisionDetection2D::SetContactDetailsForRegionC(Aerolite::Contact2D& contact, Aerolite::Body2D& polygon, Aerolite::Body2D& circle, Aerolite::Vec2& minCurrVertex, Aerolite::Vec2& minNextVertex, Aerolite::real radius, Aerolite::real distanceToCircleEdge) {
        contact.a = &polygon;
        contact.b = &circle;
        contact.depth = radius - distanceToCircleEdge;
        contact.normal = (minNextVertex - minCurrVertex).Normal();
        contact.start = circle.position - (contact.normal * radius);
        contact.end = contact.start + (contact.normal * contact.depth);
    }

    void CollisionDetection2D::SetContactDetailsForInsideCollision(Aerolite::Contact2D& contact, Aerolite::Body2D& polygon, Aerolite::Body2D& circle, Aerolite::Vec2& minCurrVertex, Aerolite::Vec2& minNextVertex, Aerolite::real radius, Aerolite::real distanceToCircleEdge) {
        contact.a = &polygon;
        contact.b = &circle;
        contact.depth = radius - distanceToCircleEdge;
        contact.normal = (minNextVertex - minCurrVertex).Normal();
        contact.start = circle.position - (contact.normal * radius);
        contact.end = contact.start + (contact.start * contact.depth);
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

    void CollisionDetection2D::FindContactPointsPolygons(PolygonShape& shapeA, PolygonShape& shapeB, Aerolite::Contact2D& contact)
    {
        Aerolite::Vec2 c1;
        Aerolite::Vec2 c2;
        int contactCount = 0;

        Aerolite::real minDistance = std::numeric_limits<Aerolite::real>::max();

        for (int i = 0; i < shapeA.worldVertices.size(); i++)
        {
            Aerolite::Vec2 p = shapeA.worldVertices[i];

            for (int j = 0; j < shapeB.worldVertices.size(); j++)
            {
                Aerolite::Vec2 edgePoint1 = shapeB.worldVertices[j];
                Aerolite::Vec2 edgePoint2 = shapeB.worldVertices[(j + 1) % shapeB.worldVertices.size()];

                Aerolite::Vec2 closestPoint;
                Aerolite::real distance;

                PointLineSegmentDistance(p, edgePoint1, edgePoint2, distance, closestPoint);

                if (Aerolite::AreEqual(distance, minDistance, Aerolite::epsilon))
                {
                    if (closestPoint != contact.start)
                    {
                        contact.end = closestPoint;
                    }
                }
                else if (distance < minDistance)
                {
                    minDistance = distance;
                    contact.start = closestPoint;
                }      
            }
        }

        for (int i = 0; i < shapeB.worldVertices.size(); i++)
        {
            Aerolite::Vec2 p = shapeB.worldVertices[i];

            for (int j = 0; j < shapeA.worldVertices.size(); j++)
            {
                Aerolite::Vec2 edgePoint1 = shapeA.worldVertices[j];
                Aerolite::Vec2 edgePoint2 = shapeA.worldVertices[(j + 1) % shapeA.worldVertices.size()];

                Aerolite::Vec2 closestPoint;
                Aerolite::real distance;

                PointLineSegmentDistance(p, edgePoint1, edgePoint2, distance, closestPoint);

                if (Aerolite::AreEqual(distance, minDistance, Aerolite::epsilon))
                {
                    if (closestPoint != contact.start)
                    {
                        contact.end = closestPoint;
                    }
                }
                else if (distance < minDistance)
                {
                    minDistance = distance;
                    contact.start = closestPoint;
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
}
