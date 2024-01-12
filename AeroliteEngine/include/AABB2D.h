#ifndef AABB2D_H
#define AABB2D_H

#include "Vec2.h"
#include "Precision.h"

namespace Aerolite {

    struct AABB2D
    {
        Aerolite::Vec2 min; // Minimum coordinate of the box (bottom-left corner)
        Aerolite::Vec2 max; // Maximum coordinate of the box (top-right corner)

        // Default constructor
        AABB2D() = default;

        // Constructor with parameters
        AABB2D(const Aerolite::Vec2& min, const Aerolite::Vec2& max)
            : min(min), max(max) {}

        // Check if this AABB intersects with another AABB
        bool Intersects(const AABB2D& other) const
        {
            // No overlap if one box is on the left side of the other box's right side
            // or one box is above the other box's bottom side
            if (max.x < other.min.x || min.x > other.max.x) return false;
            if (max.y < other.min.y || min.y > other.max.y) return false;

            return true; // Overlapping on all axes means there's an intersection
        }

        // Compute the center of the AABB
        Aerolite::Vec2 Center() const
        {
            return (min + max) * 0.5;
        }

        // Compute the dimensions of the AABB (width and height)
        Aerolite::Vec2 Size() const
        {
            return max - min;
        }

        // Expand the AABB by a given amount in all directions
        void Expand(const Aerolite::Vec2& amount)
        {
            min -= amount;
            max += amount;
        }

        // Check if the AABB contains a point
        bool Contains(const Aerolite::Vec2& point) const
        {
            return (point.x >= min.x && point.x <= max.x) &&
                (point.y >= min.y && point.y <= max.y);
        }

        // Enlarge the AABB to include a given point
        void Enclose(const Aerolite::Vec2& point)
        {
            min.x = std::min(min.x, point.x);
            min.y = std::min(min.y, point.y);
            max.x = std::max(max.x, point.x);
            max.y = std::max(max.y, point.y);
        }
    };
}

#endif

