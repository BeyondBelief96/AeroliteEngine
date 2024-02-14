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

    void AeroWorld2D::ClearWorld()
    {
        m_bodyManager.GetBodies().clear();
        m_broadphasePairs.clear();
        m_constraints.clear();
        m_contactsList.clear();
        m_globalForces.clear();
        m_particles.clear();
    }

    std::shared_ptr<AeroBody2D> AeroWorld2D::CreateBody2D(const std::shared_ptr<Shape>& shape, const real x, const real y, const real mass)
    {
        std::shared_ptr<AeroBody2D> body;
        // Check the type of the shape and create the corresponding derived shape
        if (std::dynamic_pointer_cast<CircleShape>(shape) != nullptr) {
	        const auto concreteShape = std::dynamic_pointer_cast<CircleShape>(shape);
            body = m_bodyManager.CreateBody(concreteShape, x, y, mass);
        }
        else if (std::dynamic_pointer_cast<PolygonShape>(shape) != nullptr) {
	        const auto concreteShape = std::dynamic_pointer_cast<PolygonShape>(shape);
            body = m_bodyManager.CreateBody(concreteShape, x, y, mass);
        }
        else if (std::dynamic_pointer_cast<BoxShape>(shape) != nullptr) {
	        const auto concreteShape = std::dynamic_pointer_cast<BoxShape>(shape);
            body = m_bodyManager.CreateBody(concreteShape, x, y, mass);
        }
        else {
            // Handle unknown shape type or add additional checks for other shape types
            // You may throw an exception, return nullptr, or handle it in another way.
        }

        return body;
    }

    void AeroWorld2D::AddJointConstraint(const std::shared_ptr<AeroBody2D>& a, const std::shared_ptr<AeroBody2D>& b, const AeroVec2& anchorPoint) {
        m_constraints.push_back(std::make_unique<JointConstraint>(a, b, anchorPoint));
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
        if (index < 0 || index >= m_bodyManager.GetBodies().size()) {
            throw std::out_of_range("Index is out of range in RemoveBody2D");
        }

        // Remove the element at the specified index
        m_bodyManager.GetBodies().erase(m_bodyManager.GetBodies().begin() + index);
    }

    void AeroWorld2D::RemoveBody2D(AeroBody2D* bodyToRemove) {
	    const auto it = std::remove_if(m_bodyManager.GetBodies().begin(), m_bodyManager.GetBodies().end(),
	                                   [bodyToRemove](const std::shared_ptr<AeroBody2D>& body) {
		                                   return body.get() == bodyToRemove;
	                                   });

        if (it != m_bodyManager.GetBodies().end()) {
            m_bodyManager.GetBodies().erase(it, m_bodyManager.GetBodies().end());
        }
    }

	const std::vector<std::shared_ptr<AeroBody2D>>& AeroWorld2D::GetBodies() const
    {
        return m_bodyManager.GetBodies();
    }

    std::vector<std::shared_ptr<AeroBody2D>>& AeroWorld2D::GetBodies()
    {
        return m_bodyManager.GetBodies();
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

	std::vector<Contact2D> AeroWorld2D::GetContacts() const
    {
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
        for (const auto& body : m_bodyManager.GetBodies()) {
	        auto weight = AeroVec2(0.0, body->mass * m_g * PIXELS_PER_METER);
            body->AddForce(weight);

            for (auto& force : m_globalForces) {
                body->AddForce(force);
            }
        }

        for (const auto& body : m_bodyManager.GetBodies()) {
            body->IntegrateForces(dt);
        }

        // Broad phase detection
        m_broadPhasePipeline.Execute(*this);

        // Narrow phase detection
        for(const auto& pair : m_broadphasePairs)
        {
            std::vector<Contact2D> contacts;
            if (CollisionDetection2D::IsColliding(pair.a, pair.b, contacts))
            {
                m_contactsList.insert(m_contactsList.end(), contacts.begin(), contacts.end());
                for (auto& contact : contacts) {
					penetrations.emplace_back(contact.a, contact.b, contact.start, contact.end, contact.normal);
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

        for (const auto& body : m_bodyManager.GetBodies()) {
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
            const std::size_t bodyCount = m_bodyManager.GetBodies().size();

            std::cout << "Average frame time: " << averageTime << " seconds, "
                << "Number of bodies: " << bodyCount << std::endl;

            // Reset counters and update lastLogTime
            m_accumulatedTime = std::chrono::seconds(0);
            m_frameCount = 0;
            lastLogTime = now;
        }
    }
}

