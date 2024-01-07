#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include "Body2D.h"
#include "Contact2D.h"
#include "Precision.h"
#include "Shape.h"
#include "Vec2.h"

namespace Aerolite {
    /// @brief A static class with a set of utility functions for detecting 2-D Collisions between two Body2D's.
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
        /// Projects the given vertices onto the given axis and find the minimum and maximum projections
        /// of the vertics onto the axis. Stores the min/max in the pass by reference values.
        /// </summary>
        /// <param name="vertices">The vertices to project</param>
        /// <param name="axis">The axis to project the vertices on.</param>
        /// <param name="min">The value to store the minimum projection value in.</param>
        /// <param name="max">The value to store the maximum projection value in.</param>
        static void FindMinMaxProjections(std::vector<Aerolite::Vec2> vertices, Aerolite::Vec2 axis, Aerolite::real& min, Aerolite::real& max);
    };
}



#endif