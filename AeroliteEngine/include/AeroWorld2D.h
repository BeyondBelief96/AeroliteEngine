#ifndef AERO_WORLD_2D_H
#define AERO_WORLD_2D_H

#include <chrono>
#include <vector>
#include "AeroBody2D.h"
#include "AeroBodyManager.h"
#include "AeroBroadPhase.h"
#include "AeroShg.h"
#include "Contact2D.h"
#include "Particle2D.h"
#include "Constraint2D.h"

namespace Aerolite {
    class AeroWorld2D {
    private:
        std::vector<std::unique_ptr<Particle2D>> m_particles;
        std::vector<std::unique_ptr<Constraint2D>> m_constraints;
        std::vector<AeroVec2> m_globalForces;
        real m_g = 9.8f;

        std::vector<BroadPhasePair> m_broadphasePairs;
        std::vector<Contact2D> m_contactsList;
        AeroBroadPhase m_broadPhasePipeline;
        AeroShg m_shg;
        BodyManager m_bodyManager;

        // Benchmarking 
        std::chrono::high_resolution_clock::time_point m_lastLogTime;
        std::chrono::duration<double> m_accumulatedTime = std::chrono::seconds(0);
        std::chrono::steady_clock::time_point lastLogTime = std::chrono::steady_clock::now();
        int m_frameCount = 0;
    public:
        AeroWorld2D() = default;
        explicit AeroWorld2D(real gravity);
        AeroWorld2D(AeroWorld2D&) = delete;
        AeroWorld2D(AeroWorld2D&&) = delete;
        void operator=(AeroWorld2D&) = delete;
		void operator=(AeroWorld2D&&) = delete;
        ~AeroWorld2D() = default;

        void ClearWorld();

        std::shared_ptr<AeroBody2D> CreateBody2D(const std::shared_ptr<Shape>& shape, const real x, const real y, const real mass);
    	const std::vector<std::shared_ptr<AeroBody2D>>& GetBodies() const;
        std::vector<std::shared_ptr<AeroBody2D>>& GetBodies();
        void RemoveBody2D(int index);
        void RemoveBody2D(AeroBody2D* bodyToRemove);

        void AddJointConstraint(const std::shared_ptr<AeroBody2D>& a, const std::shared_ptr<AeroBody2D>& b, const AeroVec2& anchorPoint);
        std::vector<std::unique_ptr<Constraint2D>>& GetConstraints(void);

        void SetBroadPhaseAlgorithm(BroadPhaseAlg alg);
        void AddBroadPhasePair(const BroadPhasePair& pair);
        void ClearBroadPhasePairs();

        const AeroShg& GetShg() const;
        void ShgSetBounds(const AeroVec2& minPoint, const AeroVec2& maxPoint);
        void ShgSetBounds(real x0, real y0, real x1, real y1);
        void ShgSetCellWidth(real cellWidth);
        void ShgSetCellHeight(real cellHeight);

        void AddParticle2D(std::unique_ptr<Particle2D> particle);
        void AddParticle2Ds(std::vector<std::unique_ptr<Particle2D>> particles);
        std::vector<Particle2D*> GetParticle2Ds() const;

        void AddGlobalForce(const AeroVec2& force);

        void Update(real dt);

        [[nodiscard]] std::vector<Contact2D> GetContacts(void) const;
    };
}

#endif
