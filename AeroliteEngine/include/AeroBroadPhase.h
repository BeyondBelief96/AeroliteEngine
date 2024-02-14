#ifndef NOVAPHYSICS_BROADPHASE_HPP
#define NOVAPHYSICS_BROADPHASE_HPP

#include <functional>
#include "AeroBody2D.h"

namespace Aerolite {
    class AeroWorld2D; // Forward declaration
    /**
     * @struct BroadPhasePair
     * @brief Represents a pair of bodies that might be colliding.
     */
    struct BroadPhasePair {
        std::shared_ptr<AeroBody2D> a;
        std::shared_ptr<AeroBody2D> b;
        uint32_t id_pair;

        BroadPhasePair(const std::shared_ptr<AeroBody2D>& a, const std::shared_ptr<AeroBody2D>& b, const uint32_t idPair)
            : a(a), b(b), id_pair(idPair) {}
    };

    /**
     * @enum class BroadPhaseAlg
     * @brief Enumerates available algorithms for broad-phase collision detection.
     */
    enum class BroadPhaseAlg {
        BruteForce, ///< Naive brute-force approach.
        SHG,        ///< Spatial hash grid.
        BVH         ///< Bounding Volume Hierarchy tree.
    };

    class AeroBroadPhase {
    public:

        AeroBroadPhase();
        /**
         * @brief Constructs a BroadPhase object with a specified algorithm.
         * @param algorithm The algorithm to use for broad-phase collision detection.
         */
        explicit AeroBroadPhase(BroadPhaseAlg algorithm);

        /**
         * @brief Executes the selected broad-phase collision detection algorithm.
         * @param world The simulation space containing the bodies to be checked.
         */
        void Execute(AeroWorld2D& world) const;

    private:
        BroadPhaseAlg m_algorithm;
        std::function<void(AeroWorld2D&)> m_algorithmFunction;

        void InitializeAlgorithm();
        static bool EarlyOut(AeroWorld2D& world, const AeroBody2D& a, const AeroBody2D& b);
        static void BruteForce(AeroWorld2D& world);
        static void Shg(AeroWorld2D& world);
        static void Bvh(AeroWorld2D& world);
    };

}

#endif
