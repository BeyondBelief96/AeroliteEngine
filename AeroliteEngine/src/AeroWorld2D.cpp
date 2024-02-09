#include "AeroWorld2D.h"
#include "Collision2D.h"
#include "Constants.h"
#include <iostream>
#include <stdexcept>

namespace Aerolite {
    AeroWorld2D::AeroWorld2D(const real gravity)
	: m_g(-gravity)
    {
    }

    void AeroWorld2D::AddBody2D(std::unique_ptr<AeroBody2D> body) {
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

    void AeroWorld2D::SetBroadPhaseAlgorithm(const BroadPhaseAlg alg)
    {
        m_broadPhasePipeline = AeroBroadPhase(alg);
    }

    void AeroWorld2D::AddBroadPhasePair(const BroadPhasePair& pair)
    {
        m_broadphasePairs.push_back(pair);
    }

    void AeroWorld2D::ClearBroadPhasePairs()
    {
        m_broadphasePairs.clear();
    }

    const AeroShg& AeroWorld2D::GetShg() const
    {
        return m_shg;
    }

    void AeroWorld2D::ShgSetBounds(const AeroVec2& minPoint, const AeroVec2& maxPoint)
    {
        m_shg.SetBounds(minPoint, maxPoint);
    }

    void AeroWorld2D::ShgSetBounds(const real x0, const real y0, const real x1, const real y1)
    {
        m_shg.SetBounds(x0, y0, x1, y1);
    }

    void AeroWorld2D::ShgSetCellWidth(const real cellWidth)
    {
        m_shg.SetCellWidth(cellWidth);
    }

    void AeroWorld2D::ShgSetCellHeight(const real cellHeight)
    {
        m_shg.SetCellHeight(cellHeight);
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

    void AeroWorld2D::RemoveBody2D(AeroBody2D* bodyToRemove) {
        auto it = std::remove_if(m_bodies.begin(), m_bodies.end(),
            [bodyToRemove](const std::unique_ptr<AeroBody2D>& body) {
                return body.get() == bodyToRemove;
            });

        if (it != m_bodies.end()) {
            m_bodies.erase(it, m_bodies.end());
        }
    }

    std::vector<std::unique_ptr<AeroBody2D>>& AeroWorld2D::GetBodies() {
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

    void AeroWorld2D::AddGlobalForce(const AeroVec2& force) {
        m_globalForces.push_back(force);
    }

    void AeroWorld2D::Update(const real dt) {
	    const auto startTime = std::chrono::high_resolution_clock::now();
        // Create a vector of penetration constraint to be solved per frame
        std::vector<PenetrationConstraint> penetrations;

        m_contactsList.clear();
        for (const auto& body : m_bodies) {
	        auto weight = AeroVec2(0.0, body->mass * m_g * PIXELS_PER_METER);
            body->AddForce(weight);

            for (auto& force : m_globalForces) {
                body->AddForce(force);
            }
        }

        for (const auto& body : m_bodies) {
            body->IntegrateForces(dt);
        }

        // Broad phase detection
        m_broadPhasePipeline.Execute(*this);

        // Narrow phase detection
        for(const auto& pair : m_broadphasePairs)
        {
            std::vector<Contact2D> contacts;
            if (CollisionDetection2D::IsColliding(*pair.a, *pair.b, contacts))
            {
                m_contactsList.insert(m_contactsList.end(), contacts.begin(), contacts.end());
                for (auto& contact : contacts) {
					penetrations.emplace_back(*contact.a, *contact.b, contact.start, contact.end, contact.normal);
                }
            }
        }

        for (const auto& constraint : m_constraints) {
            constraint->PreSolve(dt);
        }

        for (auto& constraint : penetrations) {
            constraint.PreSolve(dt);
        }

        for (int i = 0; i < 12; i++) {
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

        // Check if more than a second has passed since the last log
        auto now = std::chrono::steady_clock::now();
        if (now - lastLogTime >= std::chrono::seconds(1)) {
            const double averageTime = m_accumulatedTime.count() / m_frameCount;
            const std::size_t bodyCount = m_bodies.size();

            std::cout << "Average frame time: " << averageTime << " seconds, "
                << "Number of bodies: " << bodyCount << std::endl;

            // Reset counters and update lastLogTime
            m_accumulatedTime = std::chrono::seconds(0);
            m_frameCount = 0;
            lastLogTime = now;
        }
    }
}

