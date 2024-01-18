#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include "Body2D.h"
#include "Contact2D.h"
#include "Precision.h"
#include "Shape.h"
#include "AeroVec2.h"

namespace Aerolite {
    /// @brief A static class with a set of utility functions for detecting 2D Collisions between two Body2D's.
    class CollisionDetection2D {

    public:
        /// @brief Detects if two Body2D's are colliding.
        /// @param a The first Body2D for detection.
        /// @param b The second Body2D for detection.
        /// @param contact A contact2D object to store colission information if one is detected.
        /// @return Returns true if collision is detected, false if not.
        static bool IsColliding(Aerolite::Body2D& a, Aerolite::Body2D& b, std::vector<Aerolite::Contact2D>& contacts);

        /// <summary>
        /// Determines if two axis-aligned bounding boxes for two bodies are intersecting.
        /// This helps reduce the amount of collision checks needed between bodies.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        static bool IntersectAABBs(AABB2D& a, AABB2D& b);

    private:
        /// @brief Detects if two circle Body2D's are colliding.
        /// @param a The first circle body for detection.
        /// @param b The second circle body for detection.
        /// @param contact A contact2D object to store colission information if one is detected.
        /// @return Returns true if the two circles are colliding, false if not.
        static bool IsCollidingCircleCircle(Aerolite::Body2D& a, Aerolite::Body2D& b, std::vector<Aerolite::Contact2D>& contacts);

        /// <summary>
        /// Detects if two polygons are colliding.
        /// </summary>
        /// <param name="a">The first polygon body for detection.</param>
        /// <param name="b">The second polygon body for detection.</param>
        /// <returns>Returns true if the two polygons are colliding, false if not.</returns>
        static bool IsCollidingPolygonPolygon(Aerolite::Body2D& a, Aerolite::Body2D& b, std::vector<Aerolite::Contact2D>& contacts);

        /// <summary>
        /// Detects if a circle and polygon are colliding.
        /// </summary>
        /// <param name="polygon">The polygon body for detection.</param>
        /// <param name="circle">The circle body for detection.</param>
        /// <param name="contact">A reference parameter to store collision and contact information.</param>
        /// <returns>Returns true if the polygon and circle are colliding, false if not.</returns>
        static bool IsCollidingCirclePolygon(Aerolite::Body2D& polygon, Aerolite::Body2D& circle, std::vector<Aerolite::Contact2D>& contacts);

        /// <summary>
        /// Helper function for setting the contact details of the circle polygon collision detection algorithm for regions A and B.
        /// </summary>
        static void SetContactDetails(Aerolite::Contact2D& contact, Aerolite::Body2D& polygon, Aerolite::Body2D& circle, Aerolite::AeroVec2& v1, Aerolite::real radius);

        /// <summary>
        /// Helper function for setting the contact details of the circle polygon collision detection algorithm for region C.
        /// </summary>
        static void SetContactDetailsForRegionC(Aerolite::Contact2D& contact, Aerolite::Body2D& polygon, Aerolite::Body2D& circle, Aerolite::AeroVec2& minCurrVertex, Aerolite::AeroVec2& minNextVertex, Aerolite::real radius, Aerolite::real distanceToCircleEdge);

        /// <summary>
        /// Helper function for setting the contact details of the cirlce polygon collision detection algorithm for when the 
        /// circle center is inside the polygon.
        /// </summary>
        static void SetContactDetailsForInsideCollision(Aerolite::Contact2D& contact, Aerolite::Body2D& polygon, Aerolite::Body2D& circle, Aerolite::AeroVec2& minCurrVertex, Aerolite::AeroVec2& minNextVertex, Aerolite::real radius, Aerolite::real distanceToCircleEdge);

        /// <summary>
        /// Uses a naive SAT algorithm approach to determine if two polygons are colliding.
        /// </summary>
        /// <param name="a">The first polygon for collision detection.</param>
        /// <param name="b">The second polygon for collision detection.</param>
        /// <param name="contact">A reference parameter to store collision and contact information.</param>
        /// <returns></returns>
        static bool IsCollidingSATBruteForce(Aerolite::Body2D& a, Aerolite::Body2D& b, Aerolite::Contact2D& contact);

        /// <summary>
        /// Uses an optimized version of the SAT algorithm to determine if two polygons are colliding.
        /// </summary>
        /// <param name="a">The first polygon for collision detection.</param>
        /// <param name="b">The second polygon for collision detection.</param>
        /// <param name="contact">A reference parameter to store collision and contact information.</param>
        /// <returns></returns>
        static bool IsCollidingSATOptimized(Aerolite::Body2D& a, Aerolite::Body2D& b, std::vector<Aerolite::Contact2D>& contacts);

        /// <summary>
        /// Projects the given vertices onto the given axis and find the minimum and maximum projections
        /// of the vertics onto the axis. Stores the min/max in the pass by reference values.
        /// </summary>
        /// <param name="vertices">The vertices to project</param>
        /// <param name="axis">The axis to project the vertices on.</param>
        /// <param name="min">The value to store the minimum projection value in.</param>
        /// <param name="max">The value to store the maximum projection value in.</param>
        static void FindMinMaxProjections(std::vector<Aerolite::AeroVec2> vertices, Aerolite::AeroVec2 axis, Aerolite::real& min, Aerolite::real& max);

        /// <summary>
        /// Finds the distance between a point p and the closest point on a line defined
        /// by the points linePointA and linePointB.
        /// </summary>
        /// <param name="p">The point to compute the distance from.</param>
        /// <param name="linePointA">The first endpoint of the line segment.</param>
        /// <param name="linePointB">The second endpoing of the line segment.</param>
        /// <param name="distance">Output parameter for the distance from p to the cloesestPoint.</param>
        /// <param name="closestPoint">The closest point on the line segment from the point p.</param>
        static void PointLineSegmentDistance(Aerolite::AeroVec2 p, Aerolite::AeroVec2 linePointA, Aerolite::AeroVec2 linePointB, Aerolite::real& distance, Aerolite::AeroVec2& closestPoint);

        /// <summary>
        /// Finds all contact points if any exist between two polygons.
        /// </summary>
        /// <param name="shapeA">The first polygon.</param>
        /// <param name="shapeB">The second polygon.</param>
        /// <param name="contact">The contact structure to store collision information in.</param>
        static void FindContactPointsPolygons(PolygonShape& shapeA, PolygonShape& shapeB, Aerolite::Contact2D& contact);
    };
}

#endif