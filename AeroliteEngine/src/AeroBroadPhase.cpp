#include "AeroWorld2D.h"
#include "AeroBroadPhase.h"

namespace Aerolite
{
	AeroBroadPhase::AeroBroadPhase()
	{
		m_algorithm = BroadPhaseAlg::SHG;
		InitializeAlgorithm();
	}

	AeroBroadPhase::AeroBroadPhase(const BroadPhaseAlg algorithm) : m_algorithm(algorithm)
	{
		InitializeAlgorithm();
	}

	void AeroBroadPhase::Execute(AeroWorld2D& world) const
	{
		m_algorithmFunction(world);
	}


	void AeroBroadPhase::InitializeAlgorithm()
	{
		switch (m_algorithm) {
		case BroadPhaseAlg::BruteForce:
			m_algorithmFunction = [this](AeroWorld2D& world) { BruteForce(world); };
			break;
		case BroadPhaseAlg::SHG:
			m_algorithmFunction = [this](AeroWorld2D& world) { Shg(world); };
			break;
		case BroadPhaseAlg::BVH:
			m_algorithmFunction = [this](AeroWorld2D& world) { Bvh(world); };
			break;
			// Add parallel versions if needed
		default:
			throw std::invalid_argument("Unsupported broad-phase algorithm.");
		}
	}

	bool AeroBroadPhase::EarlyOut(AeroWorld2D& world, const AeroBody2D& a, const AeroBody2D& b)
	{
		// Same body or B's ID is higher, so we don't check the same pairs twice.
		if (a.id >= b.id)
			return true;

#ifndef CHECK_STATIC_COLLISIONS
		// Both bodies are static
		if (a.IsStatic() && b.IsStatic()) return true;
#endif

		// Both bodies are asleep.
		if (a.is_sleeping && b.is_sleeping) return true;

		// One body is asleep, and the other is static.
		if ((a.is_sleeping && b.IsStatic()) ||
			b.is_sleeping && a.IsStatic()) return true;

		return false;
	}

	void AeroBroadPhase::BruteForce(AeroWorld2D& world)
	{
		world.ClearBroadPhasePairs();

		for(size_t i = 0; i < world.GetBodies().size(); i++)
		{
			const std::shared_ptr<AeroBody2D>& bodyA = world.GetBodies()[i];
			auto aBox = bodyA->GetAABB();
			for(size_t j = 0; j < world.GetBodies().size(); j++)
			{
				const std::shared_ptr<AeroBody2D>& bodyB = world.GetBodies()[j];
				if (EarlyOut(world, *bodyA, *bodyB)) continue;

				const auto bBox = bodyB->GetAABB();
				if (aBox.Intersects(bBox))
				{
					BroadPhasePair pair = { bodyA, bodyB, ComputeIdPair(bodyA->id, bodyB->id)};
					world.AddBroadPhasePair(pair);
				}
			}
		}
	}

	void AeroBroadPhase::Shg(AeroWorld2D& world)
	{
		world.ClearBroadPhasePairs();
		std::vector<std::shared_ptr<AeroBody2D>> bodies;
		for (const auto& sharedPtrBody : world.GetBodies()) {
			bodies.push_back(sharedPtrBody);
		}

		auto shg = world.GetShg();
		shg.ClearCellContentCache();
		shg.Place(bodies); 

		for(auto& a : bodies)
		{
			auto aBox = a->GetAABB();
			auto [minX, minY, maxX, maxY] = shg.ComputeCellRange(aBox);

			for(int y = minY; y <= maxY; ++y)
			{
				for(int x = minX; x <= maxX; ++x)
				{
					auto cellBodies = shg.GetCellContent(x, y);

					for(const auto& b : cellBodies)
					{
						if (EarlyOut(world, *a, *b)) continue;
						if (a == b) continue; // skip the same body
						const auto bBox = b->GetAABB();
						if (aBox.Intersects(bBox)) {
							world.AddBroadPhasePair({ a, b, ComputeIdPair(a->id, b->id) });
						}
					}
				}
			}
		}
	}

	void AeroBroadPhase::Bvh(AeroWorld2D& world)
	{
	}
}
