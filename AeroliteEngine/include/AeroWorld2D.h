#ifndef AERO_WORLD_2D_H
#define AERO_WORLD_2D_H

#include <chrono>
#include <vector>
#include "Body2D.h"
#include "Contact2D.h"
#include "Particle2D.h"
#include "Constraint2D.h"

namespace Aerolite {
    class AeroWorld2D {
    private:
        std::vector<std::unique_ptr<Body2D>> m_bodies;
        std::vector<std::unique_ptr<Particle2D>> m_particles;
        std::vector<std::unique_ptr<Constraint2D>> m_constraints;
        std::vector<Contact2D> m_contactsList;
        std::vector<AeroVec2> m_bodyForces;
        std::vector<real> m_bodyTorques;
        std::vector<AeroVec2> m_particleForces;
        real m_g = 9.8f;

        std::chrono::high_resolution_clock::time_point m_lastLogTime;
        std::chrono::duration<double> m_accumulatedTime = std::chrono::seconds(0);
        int m_frameCount = 0;
    public:
        AeroWorld2D() = default;
        AeroWorld2D(AeroWorld2D& world) = default;

        AeroWorld2D(AeroWorld2D&& world) noexcept;
        explicit AeroWorld2D(real gravity);
        ~AeroWorld2D() = default;

        AeroWorld2D& operator=(AeroWorld2D&& world) noexcept = default;
        AeroWorld2D& operator=(const AeroWorld2D& world) = default;

        void AddBody2D(std::unique_ptr<Body2D> body);
        std::vector<std::unique_ptr<Body2D>>& GetBodies();
        void RemoveBody2D(int index);
        void RemoveBody2D(Body2D* bodyToRemove);

        void AddConstraint(std::unique_ptr<Constraint2D> constraint);
        std::vector<std::unique_ptr<Constraint2D>>& GetConstraints(void);

        void AddParticle2D(std::unique_ptr<Particle2D> particle);
        void AddParticle2Ds(std::vector<std::unique_ptr<Particle2D>> particles);
        std::vector<Particle2D*> GetParticle2Ds() const;

        void AddForceBody(const AeroVec2& force);
        void AddForceParticle2D(const AeroVec2& force);
        void AddTorque(const real torque);

        void Update(real dt);

        [[nodiscard]] const std::vector<Contact2D> GetContacts(void) const;
    };
}

#endif
