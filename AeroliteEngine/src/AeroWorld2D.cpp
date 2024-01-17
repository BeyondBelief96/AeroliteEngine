#include "AeroWorld2D.h"
#include "Collision2D.h"
#include "Constants.h"
#include <iostream>
#include <stdexcept>

namespace Aerolite {

    AeroWorld2D::AeroWorld2D() : G(9.8) {
        std::cout << "AeroWorld2D constructor called!" << std::endl;
    }

    AeroWorld2D::AeroWorld2D(Aerolite::real gravity) : G(-gravity) {
        // Initialization of other members, if needed
        std::cout << "AeroWorld2D constructor called!" << std::endl;
    }

    AeroWorld2D::~AeroWorld2D()
    {
        std::cout << "AeroWorld2D destructor called!" << std::endl;
    }

    AeroWorld2D::AeroWorld2D(AeroWorld2D&& other) noexcept
        : bodies(std::move(other.bodies)), particles(std::move(other.particles)),
        bodyForces(std::move(other.bodyForces)), bodyTorques(std::move(other.bodyTorques)),
        constraints(std::move(other.constraints)), G(other.G) {
    }

    AeroWorld2D& AeroWorld2D::operator=(AeroWorld2D&& other) noexcept {
        if (this != &other) {
            bodies = std::move(other.bodies);
            particles = std::move(other.particles);
            bodyForces = std::move(other.bodyForces);
            bodyTorques = std::move(other.bodyTorques);
            constraints = std::move(other.constraints);
            G = other.G;
        }
        return *this;
    }

    void AeroWorld2D::AddBody2D(std::unique_ptr<Aerolite::Body2D> body) {
        if (body != nullptr) {
            bodies.push_back(std::move(body));
        }
    }

    void AeroWorld2D::AddConstraint(std::unique_ptr<Aerolite::Constraint2D> constraint) {
        constraints.push_back(std::move(constraint));
    }

    std::vector<std::unique_ptr<Aerolite::Constraint2D>>& AeroWorld2D::GetConstraints(void) {
        return constraints;
    }

    void AeroWorld2D::AddParticle2D(std::unique_ptr<Aerolite::Particle2D> particle)
    {
        if (particle != nullptr) {
            particles.push_back(std::move(particle)); // Use std::move to transfer ownership
        }
    }

    void AeroWorld2D::AddParticle2Ds(std::vector<std::unique_ptr<Aerolite::Particle2D>> particles)
    {
        for (auto& particle : particles) {
            AddParticle2D(std::move(particle)); // Use std::move to transfer ownership
        }
    }

    void AeroWorld2D::RemoveBody2D(int index)
    {
        // Check if the index is within bounds
        if (index < 0 || index >= bodies.size()) {
            throw std::out_of_range("Index is out of range in RemoveBody2D");
        }

        // Remove the element at the specified index
        bodies.erase(bodies.begin() + index);
    }

    void AeroWorld2D::RemoveBody2D(Body2D* bodyToRemove) {
        auto it = std::remove_if(bodies.begin(), bodies.end(),
            [bodyToRemove](const std::unique_ptr<Body2D>& body) {
                return body.get() == bodyToRemove;
            });

        if (it != bodies.end()) {
            bodies.erase(it, bodies.end());
        }
    }

    std::vector<std::unique_ptr<Aerolite::Body2D>>& AeroWorld2D::GetBodies() {
        return bodies;
    }

    std::vector<Aerolite::Particle2D*> AeroWorld2D::GetParticle2Ds() {
        std::vector<Aerolite::Particle2D*> pointers;
        for (auto& particle : particles) {
            pointers.push_back(particle.get());
        }
        return pointers;
    }

    const std::vector<Aerolite::Contact2D> AeroWorld2D::GetContacts(void) const {
        return contactsList;
    }

    void AeroWorld2D::AddForceBody(const Aerolite::Vec2& force) {
        bodyForces.push_back(force);
    }

    void AeroWorld2D::AddForceParticle2D(const Aerolite::Vec2& force)
    {
        particleForces.push_back(force);
    }

    void AeroWorld2D::AddTorque(const Aerolite::real torque) {
        bodyTorques.push_back(torque);
    }

    void AeroWorld2D::Update(Aerolite::real dt) {
        // Create a vector of penetration constraint to be solved per frame
        std::vector<PenetrationConstraint> penetrations;

        contactsList.clear();
        for (auto& body : bodies) {
            Aerolite::Vec2 weight = Vec2(0.0, body->mass * G * PIXELS_PER_METER);
            body->AddForce(weight);

            for (auto& force : bodyForces) {
                body->AddForce(force);
            }

            for (auto& torque : bodyTorques) {
                body->AddTorque(torque);
            }
        }

        for (auto& body : bodies) {
            body->IntegrateForces(dt);
        }

        // Collision Detection and Resolution.
        for (int i = 0; i < bodies.size(); ++i) {
            for (int j = i + 1; j < bodies.size(); ++j) {
                std::unique_ptr<Aerolite::Body2D>& a = bodies[i];
                std::unique_ptr<Aerolite::Body2D>& b = bodies[j];
                std::vector<Aerolite::Contact2D> contacts;
                if (CollisionDetection2D::IsColliding(*a, *b, contacts))
                {
                    contactsList.insert(contactsList.end(), contacts.begin(), contacts.end());
                    for (auto& contact : contacts) {
                        penetrations.emplace_back(PenetrationConstraint(*contact.a, *contact.b, contact.start, contact.end, contact.normal));
                    }
                }
            }
        }

        for (auto& constraint : constraints) {
            constraint->PreSolve(dt);
        }

        for (auto& constraint : penetrations) {
            constraint.PreSolve(dt);
        }

        for (int i = 0; i < 8; i++) {
            for (auto& constraint : constraints) {
                constraint->Solve();
            }

            for (auto& constraint : penetrations) {
                constraint.Solve();
            }
        }

        for (auto& constraint : constraints) {
            constraint->PostSolve();
        }

        for (auto& constraint : penetrations) {
            constraint.PostSolve();
        }

        for (auto& body : bodies) {
            body->IntegrateVelocities(dt);
        }

        for (auto& particle : particles) {
            Aerolite::Vec2 weight = Vec2(0.0, particle->mass * G * PIXELS_PER_METER);
            particle->ApplyForce(weight);
            particle->Integrate(dt);
        }
    }
}

