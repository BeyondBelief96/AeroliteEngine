#include "AeroShg.h"

namespace Aerolite
{
	AeroShg::AeroShg()
	{
		// Default values, however, I recommend setting these values to something
		// that makes sense for your simulation.
		m_bounds = AeroAABB2D({0, 0}, {1920, 1080});
		m_cellWidth = 10;
		m_cellHeight = 10;
		m_cols = static_cast<uint32_t>(std::ceil(m_bounds.Width() / m_cellWidth));
		m_rows = static_cast<uint32_t>(std::ceil(m_bounds.Height() / m_cellHeight));
	}

	AeroShg::AeroShg(const AeroAABB2D& bounds, const float cellWidth, const float cellHeight)
		: m_bounds(bounds), m_cellWidth(cellWidth), m_cellHeight(cellHeight)
	{
		m_cols = static_cast<uint32_t>(std::ceil(bounds.Width() / cellWidth));
		m_rows = static_cast<uint32_t>(std::ceil(bounds.Height() / cellHeight));
	}

	AeroShg::~AeroShg()
	{
		for(auto& cell : m_cells)
		{
			delete cell.second;
		}

		m_cells.clear();
	}

	real AeroShg::GetCellWidth() const
	{
		return m_cellWidth;
	}

	real AeroShg::GetCellHeight() const
	{
		return m_cellHeight;
	}

	void AeroShg::SetBounds(const AeroVec2& minPoint, const AeroVec2& maxPoint)
	{
		m_bounds = AeroAABB2D(minPoint, maxPoint);
		ResizeGrid();
	}

	void AeroShg::SetBounds(const real x0, const real y0, const real x1, const real y1)
	{
		m_bounds = AeroAABB2D({ x0, y0 }, { x1, y1 });
		ResizeGrid();
	}

	void AeroShg::SetCellWidth(const real cellWidth)
	{
		m_cellWidth = cellWidth;
		ResizeGrid();
	}

	void AeroShg::SetCellHeight(const real cellHeight)
	{
		m_cellHeight = cellHeight;
		ResizeGrid();
	}

	void AeroShg::Place(const std::vector<AeroBody2D*>& bodies) {
		for (auto* body : bodies) { // body is a raw pointer
			const auto bodyAABB = body->GetAABB();
			auto [minX, minY, maxX, maxY] = ComputeCellRange(bodyAABB);

			for (int y = minY; y <= maxY; ++y) {
				for (int x = minX; x <= maxX; ++x) {
					const auto key = ComputeCellKey(x, y);
					InsertBodyIntoCell(key, body); // Adjusted to accept raw pointer
				}
			}
		}
	}

	std::vector<AeroBody2D*> AeroShg::GetNeighbors(const aero_int16 x0, const aero_int16 y0) const
	{
		return GetCellContent(x0, y0);
	}

	std::tuple<int, int, int, int> AeroShg::ComputeCellRange(const AeroAABB2D& aabb) const
	{
		// Convert AABB min and max points to cell coordinates
		int minX = static_cast<int>((aabb.min.x - m_bounds.min.x) / m_cellWidth);
		int minY = static_cast<int>((aabb.min.y - m_bounds.min.y) / m_cellHeight);
		int maxX = static_cast<int>((aabb.max.x - m_bounds.min.x) / m_cellWidth);
		int maxY = static_cast<int>((aabb.max.y - m_bounds.min.y) / m_cellHeight);

		// Clamp values to ensure they're within the bounds of the grid
		minX = std::max(minX, 0);
		minY = std::max(minY, 0);
		maxX = std::min(maxX, static_cast<int>(m_cols) - 1);
		maxY = std::min(maxY, static_cast<int>(m_rows) - 1);

		return { minX, minY, maxX, maxY };
	}

	std::vector<AeroBody2D*> AeroShg::GetCellContent(const aero_int16 x, const aero_int16 y) const
	{
		const auto cellKey = ComputeCellKey(x, y);
		// Check if the cell content is already in the cache
		auto it = m_cellContentCache.find(cellKey);
		if (it != m_cellContentCache.end()) {
			return it->second; // Return the cached result
		}

		// If not cached, perform the usual lookup
		const auto contentIt = m_cells.find(cellKey);
		if (contentIt != m_cells.end()) {
			// Cache the result before returning
			m_cellContentCache[cellKey] = contentIt->second->bodies;
			return contentIt->second->bodies;
		}

		// Cache an empty vector for cells that are looked up but not found
		m_cellContentCache[cellKey] = {};
		return {};
	}

	void AeroShg::ClearCellContentCache() const
	{
		m_cellContentCache.clear();
	}

	void AeroShg::ResizeGrid()
	{
		m_cols = static_cast<uint32_t>(std::ceil(m_bounds.Width() / m_cellWidth));
		m_rows = static_cast<uint32_t>(std::ceil(m_bounds.Height() / m_cellHeight));
	}

	aero_uint32 AeroShg::ComputeCellKey(const aero_int16 x, const aero_int16 y) const
	{
		return static_cast<aero_uint32>(y * m_cols + x);
	}

	void AeroShg::InsertBodyIntoCell(const aero_uint32 key, AeroBody2D* body)
	{
		if (!m_cells.contains(key)) 
		{
			m_cells[key] = new Cell(); // Allocate a new Cell if it doesn't exist
		}
		m_cells[key]->bodies.push_back(body); // Add the body to the cell
	}
}
