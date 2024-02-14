#ifndef AEROLITE_SPATIAL_HASH_GRID_HPP
#define AEROLITE_SPATIAL_HASH_GRID_HPP

#include <unordered_map>
#include <vector>
#include "AeroBody2D.h" // Ensure this is the correct path
#include "Config.h"     // Ensure this is the correct path

namespace Aerolite {

    /**
     * @class AeroShg
     * @brief Implements a spatial hash grid for efficient collision detection in a 2D space.
     *
     * The spatial hash grid divides the space into cells, allowing for quick querying of bodies within a certain area.
     */
    class AeroShg {
    public:
	    /**
	     * @brief Default constructor. Initializes values to default values, however
	     * I recommend using setter functions to create grid with a size that makes sense
	     * for your simulation.
	     */
	    AeroShg();
        /**
         * @brief Constructs a new spatial hash grid with specified dimensions and cell size.
         * @param bounds The total bounding box of the grid.
         * @param cellWidth The width of each cell in the grid.
         * @param cellHeight The height of each cell in the grid.
         */
        AeroShg(const AeroAABB2D& bounds, real cellWidth, real cellHeight);

        // Default copy and move constructors and assignment operators for AeroShg
        AeroShg(const AeroShg&) = default;
        AeroShg(AeroShg&&) noexcept = delete;
        AeroShg& operator=(const AeroShg&) = delete;
        AeroShg& operator=(AeroShg&&) = delete;

        /** @brief Default destructor */
        ~AeroShg();

        real GetCellWidth() const;
        real GetCellHeight() const;
        void SetBounds(const AeroVec2& minPoint, const AeroVec2& maxPoint);
        void SetBounds(real x0, real y0, real x1, real y1);
        void SetCellWidth(real cellWidth);
        void SetCellHeight(real cellHeight);

        /**
         * @brief Places bodies into the spatial hash grid.
         * @param bodies Vector of pointers to AeroBody2D objects to be placed in the grid.
         */
        void Place(const std::vector<std::shared_ptr<AeroBody2D>>& bodies);

	    /**
	     * \brief Clears the contents of the cell content cache to prepare for next update.
	     */
	    void ClearCellContentCache() const;

        /**
         * @brief Retrieves neighbors close to a specific grid cell.
         * @param x0 The x-coordinate of the cell.
         * @param y0 The y-coordinate of the cell.
         * @return Vector of pointers to AeroBody2D objects that are neighbors to the specified cell.
         */
        std::vector<std::shared_ptr<AeroBody2D>> GetNeighbors(aero_int16 x0, aero_int16 y0) const;

        /**
        * @brief Computes the grid cell coordinates range covered by a given AABB.
        * @param aabb The AABB for which to compute the cell range.
        * @return A tuple containing the min and max cell coordinates (minX, minY, maxX, maxY).
        */
        std::tuple<int, int, int, int> ComputeCellRange(const AeroAABB2D& aabb) const;

        /**
         * @brief Gets the content of a specific cell.
         * @param x The x coordinate of the cell.
         * @param y The y coordinate of the cell.
         * @return Vector of pointers to AeroBody2D objects contained in the specified cell.
         */
        std::vector<std::shared_ptr<AeroBody2D>> GetCellContent(const aero_int16 x, const aero_int16 y) const;

    private:
        struct Cell {
            std::vector<std::shared_ptr<AeroBody2D>> bodies; ///< Bodies contained within a cell
        };

	    /**
	     * @brief Resizes the amount of columns and rows in the grid.
	     * This is usually called after the cell width/height or bounds change.
	     */
	    void ResizeGrid();

        AeroAABB2D m_bounds; ///< Total bounds of the spatial hash grid
        aero_uint32 m_cols, m_rows; ///< Number of columns and rows in the grid
        real m_cellWidth, m_cellHeight; ///< Width and height of each cell
        real m_invCellWidth, m_invCellHeight; ///<Inverse of the cell width/height for computational efficiency.
        std::unordered_map<aero_uint32, Cell*> m_cells; ///< Mapping from cell keys to cell data

        mutable std::unordered_map<int, std::vector<std::shared_ptr<AeroBody2D>>> m_cellContentCache; ///<Cached version of the cell contents to prevent multiple hash map lookups during update.

        /**
         * @brief Computes a unique key for a cell based on its x and y coordinates.
         * @param x The x-coordinate of the cell.
         * @param y The y-coordinate of the cell.
         * @return The computed cell key.
         */
        aero_uint32 ComputeCellKey(aero_int16 x, aero_int16 y) const;

        /**
         * @brief Inserts a body into a cell identified by the given key.
         * @param key The unique key of the cell.
         * @param body Shared pointer to the AeroBody2D object to insert.
         */
        void InsertBodyIntoCell(const aero_uint32 key, const std::shared_ptr<AeroBody2D>& body);
    };
}

#endif // AEROLITE_SPATIAL_HASH_GRID_HPP
