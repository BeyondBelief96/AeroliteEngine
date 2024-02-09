#ifndef AABB2D_H
#define AABB2D_H

#include "AeroVec2.h"
#include "Precision.h"

namespace Aerolite {

    /**
     * @struct AeroAABB2D
     * @brief Represents an axis-aligned bounding box with minimum and maximum coordinates.
     */
    struct AeroAABB2D
    {
        AeroVec2 min; ///< Minimum coordinate of the box (bottom-left corner).
        AeroVec2 max; ///< Maximum coordinate of the box (top-right corner).

        /// Default constructor.
        AeroAABB2D() = default;

        /**
         * @brief Constructor with parameters to initialize the bounding box.
         * @param min The minimum coordinate (bottom-left corner) of the box.
         * @param max The maximum coordinate (top-right corner) of the box.
         */
        AeroAABB2D(AeroVec2 min, AeroVec2 max)
            : min(std::move(min)), max(std::move(max)) {}

        /**
         * @brief Checks if this AABB intersects with another AABB.
         * @param other The other AABB to check intersection with.
         * @return True if the AABBs intersect, false otherwise.
         */
        [[nodiscard]] bool Intersects(const AeroAABB2D& other) const
        {
            if (max.x < other.min.x || min.x > other.max.x) return false;
            if (max.y < other.min.y || min.y > other.max.y) return false;
            return true;
        }

        /**
         * @brief Computes the center point of the AABB.
         * @return The center point of the AABB.
         */
        [[nodiscard]] AeroVec2 Center() const
        {
            return (min + max) * 0.5;
        }

        /**
         * @brief Computes the size of the AABB (width and height).
         * @return The dimensions of the AABB.
         */
        [[nodiscard]] AeroVec2 Size() const
        {
            return max - min;
        }

        /**
         * @brief Returns the width of this bounding box.
         * @return Returns the width of the bounding box./
         */
        real Width() const
        {
            return max.x - min.x;
        }

        /**
        * @brief Returns the height of this bounding box.
        * @return Returns the height of the bounding box./
        */
        real Height() const
        {
            return max.y - min.y;
        }

        /**
         * @brief Expands the AABB by a given amount in all directions.
         * @param amount The amount to expand the AABB by.
         */
        void Expand(const AeroVec2& amount)
        {
            min -= amount;
            max += amount;
        }

        /**
         * @brief Checks if the AABB contains a given point.
         * @param point The point to check.
         * @return True if the AABB contains the point, false otherwise.
         */
        [[nodiscard]] bool Contains(const AeroVec2& point) const
        {
            return (point.x >= min.x && point.x <= max.x) &&
                (point.y >= min.y && point.y <= max.y);
        }

        /**
         * @brief Enlarges the AABB to include a given point.
         * @param point The point to enclose within the AABB.
         */
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
