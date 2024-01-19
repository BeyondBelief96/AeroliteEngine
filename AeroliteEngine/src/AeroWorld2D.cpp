#include "AeroWorld2D.h"
#include "Collision2D.h"
#include "Constants.h"
#include <iostream>
#include <stdexcept>

namespace Aerolite {
    AeroWorld2D::AeroWorld2D(const real gravity) : m_g(-gravity) {}

    AeroWorld2D::AeroWorld2D(AeroWorld2D&& world) noexcept
        : m_bodies(std::move(world.m_bodies)), m_particles(std::move(world.m_particles)),
        m_constraints(std::move(world.m_constraints)), m_bodyForces(std::move(world.m_bodyForces)),
        m_bodyTorques(std::move(world.m_bodyTorques)), m_g(world.m_g) {
    }

    void AeroWorld2D::AddBody2D(std::unique_ptr<Body2D> body) {
        if (body != nullptr) {
            m_bodies.push_back(std::move(body));
        }
    }

    void AeroWorld2D::AddConstraint(std::unique_ptr<Constraint2D> constraint) {
        m_constraints.push_back(std::move(constraint));
    }

    std::vector<std::unique_ptr<Constraint2D>>& AeroWorld2D::GetConstraints(void) {
        return m_constraints;
    }

    void AeroWorld2D::AddParticle2D(std::unique_ptr<Particle2D> particle)
    {
        if (particle != nullptr) {
            m_particles.push_back(std::move(particle)); // Use std::move to transfer ownership
        }
    }

    void AeroWorld2D::AddParticle2Ds(std::vector<std::unique_ptr<Particle2D>> particles)
    {
        for (auto& particle : particles) {
            AddParticle2D(std::move(particle)); // Use std::move to transfer ownership
        }
    }

    void AeroWorld2D::RemoveBody2D(const int index)
    {
        // Check if the index is within bounds
        if (index < 0 || index >= m_bodies.size()) {
            throw std::out_of_range("Index is out of range in RemoveBody2D");
        }

        // Remove the element at the specified index
        m_bodies.erase(m_bodies.begin() + index);
    }

    void AeroWorld2D::RemoveBody2D(Body2D* bodyToRemove) {
        auto it = std::remove_if(m_bodies.begin(), m_bodies.end(),
            [bodyToRemove](const std::unique_ptr<Body2D>& body) {
                return body.get() == bodyToRemove;
            });

        if (it != m_bodies.end()) {
            m_bodies.erase(it, m_bodies.end());
        }
    }

    std::vector<std::unique_ptr<Body2D>>& AeroWorld2D::GetBodies() {
        return m_bodies;
    }

    std::vector<Particle2D*> AeroWorld2D::GetParticle2Ds() const
    {
        std::vector<Particle2D*> pointers;
        pointers.reserve(m_particles.size());
        for (auto& particle : m_particles) {
            pointers.push_back(particle.get());
        }
        return pointers;
    }

    const std::vector<Contact2D> AeroWorld2D::GetContacts(void) const {
        return m_contactsList;
    }

    void AeroWorld2D::AddForceBody(const AeroVec2& force) {
        m_bodyForces.push_back(force);
    }

    void AeroWorld2D::AddForceParticle2D(const AeroVec2& force)
    {
        m_particleForces.push_back(force);
    }

    void AeroWorld2D::AddTorque(const real torque) {
        m_bodyTorques.push_back(torque);
    }

    void AeroWorld2D::Update(real dt) {
	    const auto startTime = std::chrono::high_resolution_clock::now();
        // Create a vector of penetration constraint to be solved per frame
        std::vector<PenetrationConstraint> penetrations;

        m_contactsList.clear();
        for (const auto& body : m_bodies) {
	        auto weight = AeroVec2(0.0, body->mass * m_g * PIXELS_PER_METER);
            body->AddForce(weight);

            for (auto& force : m_bodyForces) {
                body->AddForce(force);
            }

            for (const auto& torque : m_bodyTorques) {
                body->AddTorque(torque);
            }
        }

        for (const auto& body : m_bodies) {
            body->IntegrateForces(dt);
        }

        // Collision Detection and Resolution.
        for (int i = 0; i < m_bodies.size(); ++i) {
            for (int j = i + 1; j < m_bodies.size(); ++j) {
                std::unique_ptr<Body2D>& a = m_bodies[i];
                std::unique_ptr<Body2D>& b = m_bodies[j];
                auto aAABB = a->GetAABB();
                auto bAABB = b->GetAABB();
                if (!CollisionDetection2D::IntersectAABBs(aAABB, bAABB)) continue;
                std::vector<Contact2D> contacts;
                if (CollisionDetection2D::IsColliding(*a, *b, contacts))
                {
                    m_contactsList.insert(m_contactsList.end(), contacts.begin(), contacts.end());
                    for (auto& contact : contacts) {
                        penetrations.emplace_back(*contact.a, *contact.b, contact.start, contact.end, contact.normal);
                    }
                }
            }
        }

        for (const auto& constraint : m_constraints) {
            constraint->PreSolve(dt);
        }

        for (auto& constraint : penetrations) {
            constraint.PreSolve(dt);
        }

        for (int i = 0; i < 8; i++) {
            for (const auto& constraint : m_constraints) {
                constraint->Solve();
            }

            for (auto& constraint : penetrations) {
                constraint.Solve();
            }
        }

        for (const auto& constraint : m_constraints) {
            constraint->PostSolve();
        }

        for (auto& constraint : penetrations) {
            constraint.PostSolve();
        }

        for (const auto& body : m_bodies) {
            body->IntegrateVelocities(dt);
        }

        for (const auto& particle : m_particles) {
	        auto weight = AeroVec2(0.0, particle->mass * m_g * PIXELS_PER_METER);
            particle->ApplyForce(weight);
            particle->Integrate(dt);
        }

	    const auto endTime = std::chrono::high_resolution_clock::now();
        m_accumulatedTime += endTime - startTime;
        m_frameCount++;

        // Log the average frame time and the number of bodies every second or after every 60 frames
        if (m_accumulatedTime >= std::chrono::seconds(1) || m_frameCount >= 60) {
	        const double averageTime = m_accumulatedTime.count() / m_frameCount;
	        const std::size_t bodyCount = m_bodies.size(); // Assuming bodies is a container like std::vector

            std::cout << "Average frame time: " << averageTime << " seconds, "
                << "Number of bodies: " << bodyCount << '\n';

            // Reset counters
            m_accumulatedTime = std::chrono::seconds(0);
            m_frameCount = 0;
        }
    }
}

