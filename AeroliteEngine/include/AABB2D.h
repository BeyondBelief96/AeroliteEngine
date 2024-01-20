#ifndef AABB2D_H
#define AABB2D_H

#include "AeroVec2.h"
#include "Precision.h"

namespace Aerolite {

    struct AABB2D
    {
	    AeroVec2 min; // Minimum coordinate of the box (bottom-left corner)
	    AeroVec2 max; // Maximum coordinate of the box (top-right corner)

        // Default constructor
        AABB2D() = default;

        // Constructor with parameters
        AABB2D(AeroVec2 min, AeroVec2 max)
            : min(std::move(min)), max(std::move(max)) {}

        // Check if this AABB intersects with another AABB
	    [[nodiscard]] bool Intersects(const AABB2D& other) const
        {
            // No overlap if one box is on the left side of the other box's right side
            // or one box is above the other box's bottom side
            if (max.x < other.min.x || min.x > other.max.x) return false;
            if (max.y < other.min.y || min.y > other.max.y) return false;

            return true; // Overlapping on all axes means there's an intersection
        }

        // Compute the center of the AABB
	    [[nodiscard]] AeroVec2 Center() const
        {
            return (min + max) * 0.5;
        }

        // Compute the dimensions of the AABB (width and height)
	    [[nodiscard]] AeroVec2 Size() const
        {
            return max - min;
        }

        // Expand the AABB by a given amount in all directions
        void Expand(const AeroVec2& amount)
        {
            min -= amount;
            max += amount;
        }

        // Check if the AABB contains a point
	    [[nodiscard]] bool Contains(const AeroVec2& point) const
        {
            return (point.x >= min.x && point.x <= max.x) &&
                (point.y >= min.y && point.y <= max.y);
        }

        // Enlarge the AABB to include a given point
        void Enclose(const AeroVec2& point)
        {
            min.x = std::min(min.x, point.x);
            min.y = std::min(min.y, point.y);
            max.x = std::max(max.x, point.x);
            max.y = std::max(max.y, point.y);
        }
    };
}

#endif

