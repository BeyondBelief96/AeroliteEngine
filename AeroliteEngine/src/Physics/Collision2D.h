#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include "Body2D.h"
#include "Contact2D.h"

namespace Aerolite {
    /// @brief A struct with a set of static utility functions for detecting 2-D Collisions between two Body2D's.
    struct CollisionDetection2D {
        
        /// @brief Detects if two Body2D's are colliding.
        /// @param a The first Body2D for detection.
        /// @param b The second Body2D for detection.
        /// @param contact A contact2D object to store colission information if one is detected.
        /// @return Returns true if collision is detected, false if not.
        static bool IsColliding(Aerolite::Body2D* a, Aerolite::Body2D* b, Contact2D& contact);

        /// @brief Detects if two circle Body2D's are colliding.
        /// @param a The first circle body for detection.
        /// @param b The second circle body for detection.
        /// @param contact A contact2D object to store colission information if one is detected.
        /// @return Returns true if the two circles are colliding, false if not.
        static bool IsCollidingCircleCircle(Aerolite::Body2D* a, Aerolite::Body2D* b, Aerolite::Contact2D& contact);
    };
}



#endif