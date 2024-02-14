#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include "AeroAABB2D.h"
#include "AeroBody2D.h"
#include "Contact2D.h"
#include "Precision.h"
#include "Shape.h"
#include "AeroVec2.h"

namespace Aerolite {
    /// @brief A static class with a set of utility functions for detecting 2D Collisions between two AeroBody2D's.
    class CollisionDetection2D {

    public:
        /// @brief Detects if two AeroBody2D's are colliding.
        /// @param a The first AeroBody2D for detection.
        /// @param b The second AeroBody2D for detection.
        /// @param contacts A vector of contact2D object to store collision information if one is detected.
        /// @return Returns true if collision is detected, false if not.
        static bool IsColliding(const std::shared_ptr<AeroBody2D>& a, const std::shared_ptr<AeroBody2D>& b, std::vector<Contact2D>& contacts);

        /// <summary>
        /// Determines if two axis-aligned bounding boxes for two bodies are intersecting.
        /// This helps reduce the amount of collision checks needed between bodies.
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns></returns>
        static bool IntersectAABBs(const AeroAABB2D& a, const AeroAABB2D& b);

    private:
        /// @brief Detects if two circle AeroBody2D's are colliding.
        /// @param a The first circle body for detection.
        /// @param b The second circle body for detection.
        /// @param contacts A vector of contact2D object to store collision information if one is detected.
        /// @return Returns true if the two circles are colliding, false if not.
        static bool IsCollidingCircleCircle(const std::shared_ptr<AeroBody2D>& a, const std::shared_ptr<AeroBody2D>& b, std::vector<Contact2D>& contacts);

        /// <summary>
        /// Detects if two polygons are colliding.
        /// </summary>
        /// <param name="a">The first polygon body for detection.</param>
        /// <param name="b">The second polygon body for detection.</param>\
        /// <param name="contacts"> contacts A vector of contact2D object to store collision information if one is detected.
        /// <returns>Returns true if the two polygons are colliding, false if not.</returns>
        static bool IsCollidingPolygonPolygon(const std::shared_ptr<AeroBody2D>& a, const std::shared_ptr<AeroBody2D>& b, std::vector<Contact2D>& contacts);

        /// <summary>
        /// Detects if a circle and polygon are colliding.
        /// </summary>
        /// <param name="polygon">The polygon body for detection.</param>
        /// <param name="circle">The circle body for detection.</param>
        /// <param name="contacts">A reference parameter to store collision and contact information.</param>
        /// <returns>Returns true if the polygon and circle are colliding, false if not.</returns>
        static bool IsCollidingCirclePolygon(const std::shared_ptr<AeroBody2D>& polygon, const std::shared_ptr<AeroBody2D>
                                             & circle, std::vector<Contact2D>& contacts);

        /// <summary>
        /// Helper function for setting the contact details of the circle polygon collision detection algorithm for regions A and B.
        /// </summary>
        static void SetContactDetails(Contact2D& contact, const std::shared_ptr<AeroBody2D>& polygon, const std::shared_ptr<AeroBody2D>
                                      & circle, const AeroVec2& v1, real radius);

        /// <summary>
        /// Helper function for setting the contact details of the circle polygon collision detection algorithm for region C.
        /// </summary>
        static void SetContactDetailsForRegionC(Contact2D& contact, const std::shared_ptr<AeroBody2D>& polygon, const std::shared_ptr<AeroBody2D>& circle, const AeroVec2& minCurrVertex,
                                                const AeroVec2& minNextVertex, real radius, real distanceToCircleEdge);

        /// <summary>
        /// Helper function for setting the contact details of the circle polygon collision detection algorithm for when the 
        /// circle center is inside the polygon.
        /// </summary>
        static void SetContactDetailsForInsideCollision(Contact2D& contact, const std::shared_ptr<AeroBody2D>& polygon, const std::shared_ptr<AeroBody2D>& circle, const AeroVec2& minCurrVertex,
                                                        const AeroVec2& minNextVertex, real radius, real distanceToCircleEdge);

        /// <summary>
        /// Uses a naive SAT algorithm approach to determine if two polygons are colliding.
        /// </summary>
        /// <param name="a">The first polygon for collision detection.</param>
        /// <param name="b">The second polygon for collision detection.</param>
        /// <param name="contact">A reference parameter to store collision and contact information.</param>
        /// <returns></returns>
        static bool IsCollidingSATBruteForce(std::shared_ptr<AeroBody2D> a, std::shared_ptr<AeroBody2D> b, Contact2D& contact);

        /// <summary>
        /// Uses an optimized version of the SAT algorithm to determine if two polygons are colliding.
        /// </summary>
        /// <param name="a">The first polygon for collision detection.</param>
        /// <param name="b">The second polygon for collision detection.</param>
         /// <param name="contacts">A reference parameter to store collision and contact information.</param>
        /// <returns></returns>
        static bool IsCollidingSATOptimized(std::shared_ptr<AeroBody2D> a, std::shared_ptr<AeroBody2D> b, std::vector<Contact2D>& contacts);

        /// <summary>
        /// Projects the given vertices onto the given axis and find the minimum and maximum projections
        /// of the vertices onto the axis. Stores the min/max in the pass by reference values.
        /// </summary>
        /// <param name="vertices">The vertices to project</param>
        /// <param name="axis">The axis to project the vertices on.</param>
        /// <param name="min">The value to store the minimum projection value in.</param>
        /// <param name="max">The value to store the maximum projection value in.</param>
        static void FindMinMaxProjections(const std::vector<AeroVec2>& vertices, const AeroVec2& axis, real& min, real& max);

        /// <summary>
        /// Finds the distance between a point p and the closest point on a line defined
        /// by the points linePointA and linePointB.
        /// </summary>
        /// <param name="p">The point to compute the distance from.</param>
        /// <param name="linePointA">The first endpoint of the line segment.</param>
        /// <param name="linePointB">The second endpoint of the line segment.</param>
        /// <param name="distance">Output parameter for the distance from p to the closestPoint.</param>
        /// <param name="closestPoint">The closest point on the line segment from the point p.</param>
        static void PointLineSegmentDistance(const AeroVec2& p, const AeroVec2& linePointA, const AeroVec2& linePointB, real& distance,
                                             AeroVec2& closestPoint);

        /// <summary>
        /// Finds all contact points if any exist between two polygons.
        /// </summary>
        /// <param name="shapeA">The first polygon.</param>
        /// <param name="shapeB">The second polygon.</param>
        /// <param name="contact">The contact structure to store collision information in.</param>
        static void FindContactPointsPolygons(const PolygonShape& shapeA, const PolygonShape& shapeB, Contact2D& contact);
    };
}

#endif