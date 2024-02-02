#include <limits>
#include <complex>
#include "Collision2D.h"
#include "Precision.h"
#include "Shape.h"

namespace Aerolite
{
    // Function: IsColliding
    // Purpose: Determines if two 2D bodies are colliding.
    // Parameters:
    //   - a, b: Pointers to the two AeroBody2D objects being checked for collision.
    //   - contact: A reference to a Contact2D object where collision details will be stored.
    // Description: This function first checks the shape types of the bodies (circle or polygon).
    //              It then delegates the collision detection to the appropriate function based
    //              on the shape types.
    bool CollisionDetection2D::IsColliding(AeroBody2D& a, AeroBody2D& b, std::vector<Contact2D>& contacts)
    {
        if (a.IsStatic() && b.IsStatic()) return false;
        const bool aIsCircle = a.shape->GetType() == Circle;
        const bool bIsCircle = b.shape->GetType() == Circle;

        if (aIsCircle && bIsCircle)
        {
            return IsCollidingCircleCircle(a, b, contacts);
        }
        else if (!aIsCircle && !bIsCircle)
        {
            return IsCollidingPolygonPolygon(a, b, contacts);
        }
        else if (aIsCircle && !bIsCircle)
        {
            return IsCollidingCirclePolygon(b, a, contacts);
        }
        else if (!aIsCircle && bIsCircle)
        {
            return IsCollidingCirclePolygon(a, b, contacts);
        }

        return false;
    }


    bool CollisionDetection2D::IntersectAABBs(AABB2D& a, AABB2D& b)
    {
        if (a.max.x <= b.min.x || b.max.x <= a.min.x ||
            a.max.y <= b.min.y || b.max.y <= a.min.y) return false;

        return true;
    }

    // Function: IsCollidingCircleCircle
    // Purpose: Checks for collision between two circles.
    // Parameters:
    //   - a, b: Pointers to the circle AeroBody2D objects.
    //   - contact: A reference to a Contact2D object to store collision details.
    // Description: This function calculates if two circles are colliding by comparing
    //              the distance between their centers to the sum of their radii.
    bool CollisionDetection2D::IsCollidingCircleCircle(AeroBody2D& a, AeroBody2D& b, std::vector<Contact2D>& contacts)
    {
	    const auto aCircleShape = dynamic_cast<CircleShape*>(a.shape);
	    const auto bCircleShape = dynamic_cast<CircleShape*>(b.shape);

        const AeroVec2 distanceBetweenCenters = b.position - a.position;
        const real sumRadius = aCircleShape->radius + bCircleShape->radius;
	    const bool isColliding = distanceBetweenCenters.MagnitudeSquared() <= (sumRadius * sumRadius);

        if (!isColliding) {
            return false;
        }
        Contact2D contact;

        contact.a = &a;
        contact.b = &b;
        contact.normal = distanceBetweenCenters.UnitVector();
        contact.start = b.position - (contact.normal * bCircleShape->radius);
        contact.end = a.position + (contact.normal * aCircleShape->radius);
        contact.depth = (contact.end - contact.start).Magnitude();

        contacts.push_back(contact);
        return true;
    }

    // Function: IsCollidingPolygonPolygon
    // Purpose: Checks for collision between two polygons using the Separating Axis Theorem (SAT).
    // Parameters:
    //   - a, b: Pointers to the polygon AeroBody2D objects.
    //   - contact: A reference to a Contact2D object, not used in this function as SAT does not provide contact points.
    // Description: This function checks for overlap along all possible axes formed by the edges of the polygons.
    //              If a separating axis is found (no overlap on an axis), the polygons are not colliding.
    bool CollisionDetection2D::IsCollidingPolygonPolygon(AeroBody2D& a, AeroBody2D& b, std::vector<Contact2D>& contacts)
    {
        return IsCollidingSATOptimized(a, b, contacts);
    }

    bool CollisionDetection2D::IsCollidingSATBruteForce(AeroBody2D& a, AeroBody2D& b, Contact2D& contact)
    {
        // Casting the shape of each body to a PolygonShape to access polygon-specific properties.
        const auto aPolygonShape = dynamic_cast<PolygonShape*>(a.shape);
        const auto bPolygonShape = dynamic_cast<PolygonShape*>(b.shape);
        auto contactDepth = std::numeric_limits<real>::max();
        auto contactNormal = AeroVec2();

        // Checking for overlap along all axes formed by the edges of polygon a.
        for (int i = 0; i < aPolygonShape->worldVertices.size(); i++)
        {
            // Compute the normal to the edge.
            AeroVec2 normal = aPolygonShape->EdgeAt(i).Normal().UnitVector();

            // Initialize min and max projection values.
            auto minA = std::numeric_limits<real>::max();
            auto maxA = std::numeric_limits<real>::min();
            auto minB = std::numeric_limits<real>::max();
            auto maxB = std::numeric_limits<real>::min();

            // Find the min and max projections of both polygons onto the axis.
            FindMinMaxProjections(aPolygonShape->worldVertices, normal, minA, maxA);
            FindMinMaxProjections(bPolygonShape->worldVertices, normal, minB, maxB);

            // If projections do not overlap, there is a separating axis, so no collision.
            if (minA >= maxB || minB >= maxA) return false;

            const auto axisDepth = std::min(maxB - minA, maxA - minB);

            if (axisDepth < contactDepth)
            {
                contactDepth = axisDepth;
                contactNormal = normal;
            }
        }

        // Repeat the process for all axes formed by the edges of polygon b.
        for (int i = 0; i < bPolygonShape->worldVertices.size(); i++)
        {
	        AeroVec2 normal = bPolygonShape->EdgeAt(i).Normal().UnitVector();

            auto minA = std::numeric_limits<real>::max();
            auto maxA = std::numeric_limits<real>::min();
            auto minB = std::numeric_limits<real>::max();
            auto maxB = std::numeric_limits<real>::min();

            FindMinMaxProjections(aPolygonShape->worldVertices, normal, minA, maxA);
            FindMinMaxProjections(bPolygonShape->worldVertices, normal, minB, maxB);

            if (minA >= maxB || minB >= maxA) return false;

	        const auto axisDepth = std::min(maxB - minA, maxA - minB);

            if (axisDepth < contactDepth)
            {
                contactDepth = axisDepth;
                contactNormal = normal;
            }
        }

        // Ensuring that the normal of the contact points from body A to body B.
        const AeroVec2 centerA = aPolygonShape->GeometricCenter();
        const AeroVec2 centerB = bPolygonShape->GeometricCenter();

        const AeroVec2 direction = centerB - centerA;
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
    bool CollisionDetection2D::IsCollidingSATOptimized(AeroBody2D& a, AeroBody2D& b, std::vector<Contact2D>& contacts)
    {
        // Cast the shapes of the bodies to polygon shapes
        const auto polygonShapeA = dynamic_cast<PolygonShape*>(a.shape);
        const auto polygonShapeB = dynamic_cast<PolygonShape*>(b.shape);

        // Variables to store the axis of minimum separation and the corresponding points
        int aIndexReferenceEdge, bIndexReferenceEdge;
        AeroVec2 aSupportPoint, bSupportPoint;

        // Find the minimum separation from A to B, along with the separation axis and point
        const real abSeparation = polygonShapeA->FindMinimumSeparation(*polygonShapeB, aIndexReferenceEdge, aSupportPoint);
        // Find the minimum separation from B to A, along with the separation axis and point
        const real baSeparation = polygonShapeB->FindMinimumSeparation(*polygonShapeA, bIndexReferenceEdge, bSupportPoint);

        // If either separation is non-negative, no overlap occurs, thus no collision
        if (abSeparation >= 0) return false;
        if (baSeparation >= 0) return false;

        PolygonShape* referenceShape;
        PolygonShape* incidentShape;
        int indexReferenceEdge;
        if (abSeparation > baSeparation) {
            // Set "A" as our reference shape and "B" as incident shape.
            referenceShape = polygonShapeA;
            incidentShape = polygonShapeB;
            indexReferenceEdge = aIndexReferenceEdge;
        }
        else {
            // Set "B" as our reference shape and "A" as incident shape.
            referenceShape = polygonShapeB;
            incidentShape = polygonShapeA;
            indexReferenceEdge = bIndexReferenceEdge;
        }

        const AeroVec2 referenceEdge = referenceShape->EdgeAt(indexReferenceEdge);
        
        // Clipping
        const int incidentIndex = incidentShape->FindIncidentEdgeIndex(referenceEdge.Normal());
        const int incidentNextIndex = (incidentIndex + 1) % incidentShape->worldVertices.size();
        const AeroVec2 v0 = incidentShape->worldVertices[incidentIndex];
        const AeroVec2 v1 = incidentShape->worldVertices[incidentNextIndex];
        std::vector<AeroVec2> contactPoints = { v0, v1 };
        std::vector<AeroVec2> clippedPoints = contactPoints;
        for (int i = 0; i < referenceShape->worldVertices.size(); i++) {
            if (i == indexReferenceEdge) continue;

            AeroVec2 c0 = referenceShape->worldVertices[i];
            AeroVec2 c1 = referenceShape->worldVertices[(i + 1) % referenceShape->worldVertices.size()];
            const int numClipped = Aerolite::PolygonShape::ClipLineSegmentToLine(contactPoints, clippedPoints, c0, c1);
            if(numClipped < 2) {
                break;
            }

            contactPoints = clippedPoints; // Make the next contact points the ones that were just clipped.
        }

        const auto vref = referenceShape->worldVertices[indexReferenceEdge];

        for (auto& vclip : clippedPoints) {
	        const real separation = (vclip - vref).Dot(referenceEdge.Normal());
            if (separation <= 0) {
                Contact2D contact;
                contact.a = &a;
                contact.b = &b;
                contact.normal = referenceEdge.Normal();
                contact.start = vclip;
                contact.end = vclip + contact.normal * -separation;
                if (baSeparation >= abSeparation) {
                    std::swap(contact.start, contact.end);
                    contact.normal *= -1.0;
                }

                contacts.push_back(contact);
            }
        }

        return true;
    }

    // Check for collision between a circle and a polygon.
    bool CollisionDetection2D::IsCollidingCirclePolygon(AeroBody2D& polygon, AeroBody2D& circle, std::vector<Contact2D>& contacts)
    {
        // Cast the shapes to their respective types
        const PolygonShape* polygonShape = dynamic_cast<PolygonShape*>(polygon.shape);
        const CircleShape* circleShape = dynamic_cast<CircleShape*>(circle.shape);

        // Initialization of variables to track the closest edge and if the circle is outside the polygon
        bool isOutside = false;
        AeroVec2 minCurrVertex;
        AeroVec2 minNextVertex;
        real distanceToCircleEdge = std::numeric_limits<real>::lowest();

        // Iterate over each edge of the polygon
        for (int i = 0; i < polygonShape->worldVertices.size(); i++)
        {
	        const int currVertex = i;
	        const int nextVertex = (i + 1) % polygonShape->worldVertices.size();
            AeroVec2 edge = polygonShape->EdgeAt(currVertex);
            AeroVec2 normal = edge.Normal();

            // Compute vector from the current vertex to the circle's center
            AeroVec2 vertexToCircleCenter = circle.position - polygonShape->worldVertices[currVertex];

            // Project the vertex-to-center vector onto the edge's normal
	        const real projection = vertexToCircleCenter.Dot(normal);

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

        Contact2D contact;
        // Check for collision based on the region where the circle is relative to the closest edge
        if (isOutside)
        {
            // Handle collision detection for region A
            AeroVec2 v1 = circle.position - minCurrVertex;
            AeroVec2 v2 = minNextVertex - minCurrVertex;
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

        contacts.push_back(contact);

        // Collision detected
        return true;
    }

    // Helper functions for setting contact details (to avoid code repetition)
    void CollisionDetection2D::SetContactDetails(Contact2D& contact, AeroBody2D& polygon, AeroBody2D& circle, const AeroVec2& v1,
                                                 const real radius) {
        contact.a = &polygon;
        contact.b = &circle;
        contact.depth = radius - v1.Magnitude();
        contact.normal = v1.UnitVector();
        contact.start = circle.position + (contact.normal * -radius);
        contact.end = contact.start + (contact.normal * contact.depth);
    }

    void CollisionDetection2D::SetContactDetailsForRegionC(Contact2D& contact, AeroBody2D& polygon, AeroBody2D& circle,
                                                           const AeroVec2& minCurrVertex, const AeroVec2& minNextVertex, const real radius,
                                                           const real distanceToCircleEdge) {
        contact.a = &polygon;
        contact.b = &circle;
        contact.depth = radius - distanceToCircleEdge;
        contact.normal = (minNextVertex - minCurrVertex).Normal();
        contact.start = circle.position - (contact.normal * radius);
        contact.end = contact.start + (contact.normal * contact.depth);
    }

    void CollisionDetection2D::SetContactDetailsForInsideCollision(Contact2D& contact, AeroBody2D& polygon, AeroBody2D& circle,
                                                                   const AeroVec2& minCurrVertex, const AeroVec2& minNextVertex,
                                                                   const real radius, const real distanceToCircleEdge) {
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
    //   - vertices: A vector of AeroVec2 representing the vertices of a polygon.
    //   - axis: The axis onto which the vertices are to be projected.
    //   - min, max: References to store the minimum and maximum projections.
    // Description: This function is a helper for the Separating Axis Theorem. It projects each vertex onto the axis
    //              and keeps track of the minimum and maximum values of these projections.
    void CollisionDetection2D::FindMinMaxProjections(const std::vector<AeroVec2>& vertices, const AeroVec2& axis,
                                                     real& min, real& max)
    {
        for (auto currentVertex : vertices)
        {
	        // Projecting the vertex onto the axis.
            const auto projection = currentVertex.Dot(axis);

            // Updating the min and max values based on the projection.
            if (projection < min) {
                min = projection;
            }
            if (projection > max) {
                max = projection;
            }

        }
    }

    void CollisionDetection2D::FindContactPointsPolygons(PolygonShape& shapeA, PolygonShape& shapeB, Contact2D& contact)
    {
	    AeroVec2 c1;
	    AeroVec2 c2;
        int contactCount = 0;

	    real minDistance = std::numeric_limits<real>::max();

        for (const auto p : shapeA.worldVertices)
        {
	        for (int j = 0; j < shapeB.worldVertices.size(); j++)
            {
	            AeroVec2 edgePoint1 = shapeB.worldVertices[j];
	            AeroVec2 edgePoint2 = shapeB.worldVertices[(j + 1) % shapeB.worldVertices.size()];

	            AeroVec2 closestPoint;
	            real distance;

                PointLineSegmentDistance(p, edgePoint1, edgePoint2, distance, closestPoint);

                if (AreEqual(distance, minDistance, EPSILON))
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

        for (const auto p : shapeB.worldVertices)
        {
	        for (int j = 0; j < shapeA.worldVertices.size(); j++)
            {
	            AeroVec2 edgePoint1 = shapeA.worldVertices[j];
	            AeroVec2 edgePoint2 = shapeA.worldVertices[(j + 1) % shapeA.worldVertices.size()];

	            AeroVec2 closestPoint;
	            real distance;

                PointLineSegmentDistance(p, edgePoint1, edgePoint2, distance, closestPoint);

                if (AreEqual(distance, minDistance, EPSILON))
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

    void CollisionDetection2D::PointLineSegmentDistance(AeroVec2 p, const AeroVec2& linePointA, const AeroVec2& linePointB, real& distance,
                                                        AeroVec2& closestPoint)
    {
	    const AeroVec2 ab = linePointB - linePointA;
	    const AeroVec2 ap = p - linePointA;

	    const auto proj = ap.Dot(ab);
	    const auto abMagSquared = ab.MagnitudeSquared();
	    const auto d = proj / abMagSquared;

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
