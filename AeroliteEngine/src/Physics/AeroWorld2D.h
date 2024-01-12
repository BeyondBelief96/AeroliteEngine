#ifndef AERO_WORLD_2D_H
#define AERO_WORLD_2D_H

#include <vector>
#include "Body2D.h"
#include "Contact2D.h"
#include "Particle.h"
#include "pfgen.h"

namespace Aerolite {
    class AeroWorld2D {
    private:
        std::vector<std::unique_ptr<Aerolite::Body2D>> bodies;
        std::vector<std::unique_ptr<Aerolite::Particle>> particles;
        std::vector<Aerolite::Contact2D> contacts;
        std::vector<Aerolite::Vec2> bodyForces;
        std::vector<Aerolite::real> bodyTorques;
        Aerolite::ParticleForceRegistry pfr;
        Aerolite::ParticleGravity particleGravityGenerator;
        Aerolite::real G = 9.8;

    public:
        AeroWorld2D(Aerolite::real gravity);
        ~AeroWorld2D();
        AeroWorld2D(AeroWorld2D&& other) noexcept;
        AeroWorld2D& operator=(AeroWorld2D&& other) noexcept;

        void AddBody2D(std::unique_ptr<Aerolite::Body2D> body);
        void AddParticle(std::unique_ptr<Aerolite::Particle> particle);
        void RemoveBody2D(int index);
        const std::vector<std::unique_ptr<Aerolite::Body2D>>& GetBodies(void) const;
        const std::vector<Aerolite::Contact2D> GetContacts(void) const;

        void AddForce(const Aerolite::Vec2& force);
        void AddTorque(const Aerolite::real torque);
        void AddParticleWithForce(Aerolite::ParticleForceGenerator& forceGenerator, std::unique_ptr<Aerolite::Particle> particle);

        void Update(Aerolite::real dt);

        void CheckCollisions(void);
    };
}

#endif
