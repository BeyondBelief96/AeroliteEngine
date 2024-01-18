#ifndef AERO_WORLD_2D_H
#define AERO_WORLD_2D_H

#include <chrono>
#include <vector>
#include "Body2D.h"
#include "Contact2D.h"
#include "Particle2D.h"
#include "pfgen.h"
#include "Constraint2D.h"

namespace Aerolite {
    class AeroWorld2D {
    private:
        std::vector<std::unique_ptr<Aerolite::Body2D>> bodies;
        std::vector<std::unique_ptr<Aerolite::Particle2D>> particles;
        std::vector<std::unique_ptr<Aerolite::Constraint2D>> constraints;
        std::vector<Aerolite::Contact2D> contactsList;
        std::vector<Aerolite::AeroVec2> bodyForces;
        std::vector<Aerolite::real> bodyTorques;
        std::vector<Aerolite::AeroVec2> particleForces;
        Aerolite::real G = 9.8f;
        std::chrono::high_resolution_clock::time_point lastLogTime;
        std::chrono::duration<double> accumulatedTime = std::chrono::seconds(0);
        int frameCount = 0;
    public:
        AeroWorld2D();
        AeroWorld2D(Aerolite::real gravity);
        ~AeroWorld2D();
        AeroWorld2D(AeroWorld2D&& other) noexcept;
        AeroWorld2D& operator=(AeroWorld2D&& other) noexcept;

        void AddBody2D(std::unique_ptr<Aerolite::Body2D> body);
        std::vector<std::unique_ptr<Aerolite::Body2D>>& GetBodies();
        void RemoveBody2D(int index);
        void RemoveBody2D(Body2D* bodyToRemove);

        void AddConstraint(std::unique_ptr<Constraint2D> constraint);
        std::vector<std::unique_ptr<Aerolite::Constraint2D>>& GetConstraints(void);

        void AddParticle2D(std::unique_ptr<Aerolite::Particle2D> particle);
        void AddParticle2Ds(std::vector<std::unique_ptr<Aerolite::Particle2D>> particles);
        std::vector<Aerolite::Particle2D*> GetParticle2Ds();

        void AddForceBody(const Aerolite::AeroVec2& force);
        void AddForceParticle2D(const Aerolite::AeroVec2& force);
        void AddTorque(const Aerolite::real torque);

        void Update(Aerolite::real dt);

        const std::vector<Aerolite::Contact2D> GetContacts(void) const;
    };
}

#endif
