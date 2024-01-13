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
        bodyForces(std::move(other.bodyForces)), bodyTorques(std::move(other.bodyTorques)), G(other.G) {
    }

    AeroWorld2D& AeroWorld2D::operator=(AeroWorld2D&& other) noexcept {
        if (this != &other) {
            bodies = std::move(other.bodies);
            particles = std::move(other.particles);
            bodyForces = std::move(other.bodyForces);
            bodyTorques = std::move(other.bodyTorques);
            G = other.G;
        }
        return *this;
    }

    void AeroWorld2D::AddBody2D(std::unique_ptr<Aerolite::Body2D> body) {
        if (body != nullptr) {
            bodies.push_back(std::move(body));
        }
    }

    void AeroWorld2D::AddParticle(std::unique_ptr<Aerolite::Particle> particle)
    {
        if (particle != nullptr) {
            particles.push_back(std::move(particle)); // Use std::move to transfer ownership
        }
    }

    void AeroWorld2D::AddParticles(std::vector<std::unique_ptr<Aerolite::Particle>> particles)
    {
        for (auto& particle : particles) {
            AddParticle(std::move(particle)); // Use std::move to transfer ownership
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

    std::vector<Aerolite::Body2D*> AeroWorld2D::GetBodies() {
        std::vector<Aerolite::Body2D*> pointers;
        for (auto& body : bodies) {
            pointers.push_back(body.get());
        }
        return pointers;
    }

    std::vector<Aerolite::Particle*> AeroWorld2D::GetParticles() {
        std::vector<Aerolite::Particle*> pointers;
        for (auto& particle : particles) {
            pointers.push_back(particle.get());
        }
        return pointers;
    }

    const std::vector<Aerolite::Contact2D> AeroWorld2D::GetContacts(void) const
    {
        return contacts;
    }

    void AeroWorld2D::AddForceBody(const Aerolite::Vec2& force) {
        bodyForces.push_back(force);
    }

    void AeroWorld2D::AddTorque(const Aerolite::real torque) {
        bodyTorques.push_back(torque);
    }

    void AeroWorld2D::Update(Aerolite::real dt) {
        contacts.clear();
        // Update bodies
        for (auto& body : bodies) {
            Aerolite::Vec2 weight = Vec2(0.0, body->mass * G * PIXELS_PER_METER);
            body->AddForce(weight);

            for (auto& force : bodyForces) {
                body->AddForce(force);
            }

            for (auto& torque : bodyTorques) {
                body->AddTorque(torque);
            }
            
            body->Update(dt);
        }

        // Update particles
        for (auto& particle : particles) {
            Aerolite::Vec2 weight = Vec2(0.0, particle->mass * G * PIXELS_PER_METER);
            particle->ApplyForce(weight);
            particle->Integrate(dt);
        }

        CheckCollisions();
    }

    void AeroWorld2D::CheckCollisions() {
        for (int i = 0; i < bodies.size(); ++i) {
            for (int j = i + 1; j < bodies.size(); ++j) {
                std::unique_ptr<Aerolite::Body2D>& a = bodies[i];
                std::unique_ptr<Aerolite::Body2D>& b = bodies[j];
                a->isColliding = false;
                b->isColliding = false;

                Aerolite::Contact2D contact;
                if (CollisionDetection2D::IsColliding(*a, *b, contact))
                {
                    contacts.emplace_back(contact);
                    contact.ResolveCollision();
                    a->isColliding = true;
                    b->isColliding = true;
                }
            }
        }
    }
}

