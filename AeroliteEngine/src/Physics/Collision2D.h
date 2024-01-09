#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include "Body2D.h"
#include "Contact2D.h"
#include "Precision.h"
#include "Shape.h"
#include "Vec2.h"

namespace Aerolite {
    /// @brief A static class with a set of utility functions for detecting 2D Collisions between two Body2D's.
    static class CollisionDetection2D {

    public:
        /// @brief Detects if two Body2D's are colliding.
        /// @param a The first Body2D for detection.
        /// @param b The second Body2D for detection.
        /// @param contact A contact2D object to store colission information if one is detected.
        /// @return Returns true if collision is detected, false if not.
        static bool IsColliding(Aerolite::Body2D* a, Aerolite::Body2D* b, Contact2D& contact);

    private:
        /// @brief Detects if two circle Body2D's are colliding.
        /// @param a The first circle body for detection.
        /// @param b The second circle body for detection.
        /// @param contact A contact2D object to store colission information if one is detected.
        /// @return Returns true if the two circles are colliding, false if not.
        static bool IsCollidingCircleCircle(Aerolite::Body2D* a, Aerolite::Body2D* b, Aerolite::Contact2D& contact);

        /// <summary>
        /// Detects if two polygons are colliding.
        /// </summary>
        /// <param name="a">The first polygon body for detection.</param>
        /// <param name="b">The second polygon body for detection.</param>
        /// <returns>Returns true if the two polygons are colliding, false if not.</returns>
        static bool IsCollidingPolygonPolygon(Aerolite::Body2D* a, Aerolite::Body2D* b, Aerolite::Contact2D& contact);

        /// <summary>
        /// Uses a naive SAT algorithm approach to determine if two polygons are colliding.
        /// </summary>
        /// <param name="a">The first polygon for collision detection.</param>
        /// <param name="b">The second polygon for collision detection.</param>
        /// <param name="contact">A reference parameter to store collision and contact information.</param>
        /// <returns></returns>
        static bool IsCollidingSATBruteForce(Aerolite::Body2D* a, Aerolite::Body2D* b, Aerolite::Contact2D& contact);

        /// <summary>
        /// Projects the given vertices onto the given axis and find the minimum and maximum projections
        /// of the vertics onto the axis. Stores the min/max in the pass by reference values.
        /// </summary>
        /// <param name="vertices">The vertices to project</param>
        /// <param name="axis">The axis to project the vertices on.</param>
        /// <param name="min">The value to store the minimum projection value in.</param>
        /// <param name="max">The value to store the maximum projection value in.</param>
        static void FindMinMaxProjections(std::vector<Aerolite::Vec2> vertices, Aerolite::Vec2 axis, Aerolite::real& min, Aerolite::real& max);

        /// <summary>
        /// Finds the distance between a point p and the closest point on a line defined
        /// by the points linePointA and linePointB.
        /// </summary>
        /// <param name="p">The point to compute the distance from.</param>
        /// <param name="linePointA">The first endpoint of the line segment.</param>
        /// <param name="linePointB">The second endpoing of the line segment.</param>
        /// <param name="distance">Output parameter for the distance from p to the cloesestPoint.</param>
        /// <param name="closestPoint">The closest point on the line segment from the point p.</param>
        static void PointLineSegmentDistance(Aerolite::Vec2 p, Aerolite::Vec2 linePointA, Aerolite::Vec2 linePointB, Aerolite::real& distance, Aerolite::Vec2& closestPoint);

        /// <summary>
        /// Finds all contact points if any exist between two circle shapes.
        /// </summary>
        /// <param name="shapeA">The first circle.</param>
        /// <param name="shapeB">The second circle.</param>
        /// <param name="contact">The contact structure to store collision information in.</param>
        static void FindContactPointsCircles(CircleShape* shapeA, CircleShape* shapeB, Aerolite::Contact2D& contact);

        /// <summary>
        /// Finds all contact points if any exist between two polygons.
        /// </summary>
        /// <param name="shapeA">The first polygon.</param>
        /// <param name="shapeB">The second polygon.</param>
        /// <param name="contact">The contact structure to store collision information in.</param>
        static void FindContactPointsPolygons(PolygonShape* shapeA, PolygonShape* shapeB, Aerolite::Contact2D& contact);
    };
}



#endif