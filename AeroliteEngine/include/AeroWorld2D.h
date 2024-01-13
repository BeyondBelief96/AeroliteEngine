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
        std::vector<Aerolite::Vec2> particleForces;
        Aerolite::real G = 9.8;

    public:
        AeroWorld2D();
        AeroWorld2D(Aerolite::real gravity);
        ~AeroWorld2D();
        AeroWorld2D(AeroWorld2D&& other) noexcept;
        AeroWorld2D& operator=(AeroWorld2D&& other) noexcept;

        void AddBody2D(std::unique_ptr<Aerolite::Body2D> body);
        void AddParticle(std::unique_ptr<Aerolite::Particle> particle);
        void AddParticles(std::vector<std::unique_ptr<Aerolite::Particle>> particles);
        void RemoveBody2D(int index);
        std::vector<Aerolite::Body2D*> GetBodies();
        std::vector<Aerolite::Particle*> GetParticles();
        const std::vector<Aerolite::Contact2D> GetContacts(void) const;

        void AddForceBody(const Aerolite::Vec2& force);
        void AddForceParticle(const Aerolite::Vec2& force);
        void AddTorque(const Aerolite::real torque);
        void Update(Aerolite::real dt);

        void CheckCollisions(void);
    };
}

#endif
