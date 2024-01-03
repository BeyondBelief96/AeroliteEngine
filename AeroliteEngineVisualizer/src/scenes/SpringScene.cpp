#include <random>
#include <iostream>
#include "pfgen.h"
#include "Constants.h"
#include "Scene.h"

void GenerateParticlesInLine(std::vector<std::shared_ptr<Particle>>& particles,
        int numParticles, float verticalSeparation, const Vec2& anchor,
        float width, float mass) 
    {
        for (int i = 0; i < numParticles; i++) {
            float xPosition = width / 2.0;

            std::cout << anchor.y;
            std::cout << verticalSeparation;
            std::cout << i+1 * verticalSeparation;
            float yPosition = anchor.y + ((i+1) * verticalSeparation);
            auto particle = std::make_shared<Particle>(xPosition, yPosition, mass);
            particles.push_back(particle);
        }
    };


auto dragGenerator = std::make_shared<ParticleDrag>(0.001, 0.005);

///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
void SpringScene::Setup() {
    running = Graphics::OpenWindow();
    particles = std::vector<std::shared_ptr<Particle>>();
    anchor = Vec2(Graphics::Width() / 2, 30);
    GenerateParticlesInLine(particles, NUM_PARTICLES, restLength, anchor, Graphics::Width(), PARTICLE_MASS);
}

///////////////////////////////////////////////////////////////////////////////
// Input processing
///////////////////////////////////////////////////////////////////////////////
void SpringScene::Input() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
        case SDL_QUIT:
            running = false;
            break;
        case SDL_KEYDOWN:
            if (event.key.keysym.sym == SDLK_ESCAPE)
                running = false;
            if (event.key.keysym.sym == SDLK_UP)
                pushForce.y = -50 * PIXELS_PER_METER;
            if (event.key.keysym.sym == SDLK_RIGHT)
                pushForce.x = 50 * PIXELS_PER_METER;
            if (event.key.keysym.sym == SDLK_DOWN)
                pushForce.y = 50 * PIXELS_PER_METER;
            if (event.key.keysym.sym == SDLK_LEFT)
                pushForce.x = -50 * PIXELS_PER_METER;
            break;
        case SDL_KEYUP:
            if (event.key.keysym.sym == SDLK_ESCAPE)
                running = false;
            if (event.key.keysym.sym == SDLK_UP)
                pushForce.y = 0;
            if (event.key.keysym.sym == SDLK_RIGHT)
                pushForce.x = 0;
            if (event.key.keysym.sym == SDLK_DOWN)
                pushForce.y = 0;
            if (event.key.keysym.sym == SDLK_LEFT)
                pushForce.x = 0;
            break;
        case SDL_MOUSEMOTION:
            mouseCursor.x = event.motion.x;
            mouseCursor.y = event.motion.y;
            break;
        case SDL_MOUSEBUTTONDOWN:
            if (!leftMouseButtonDown && event.button.button == SDL_BUTTON_LEFT) {
                leftMouseButtonDown = true;
                int x, y;
                SDL_GetMouseState(&x, &y);
                mouseCursor.x = x;
                mouseCursor.y = y;
            }
            break;
        case SDL_MOUSEBUTTONUP:
            if (leftMouseButtonDown && event.button.button == SDL_BUTTON_LEFT) {
                leftMouseButtonDown = false;
                Vec2 impulseDirection = (particles[particles.size() - 1]->position - mouseCursor).UnitVector();
                float impulseMagnitude = (particles[particles.size() - 1]->position - mouseCursor).Magnitude() * 6.0;
                particles[particles.size() - 1]->velocity = impulseDirection * impulseMagnitude;
            }
            break;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// Update function (called several times per second to update objects)
///////////////////////////////////////////////////////////////////////////////
void SpringScene::Update() {
    // Check if we are too fast, and if so, waste some milliseconds until we reach
    // MILLISECONDS_PER_FRAME.
    int timeToWait = MILLISECS_PER_FRAME - (SDL_GetTicks() - timePreviousFrame);
    if (timeToWait > 0) {
        SDL_Delay(timeToWait);
    }

    // Calculate the delta time factor in seconds to be used when we update objects
    float deltaTime = (SDL_GetTicks() - timePreviousFrame) / 1000.0f;
    if (deltaTime > 0.016) {
        deltaTime = 0.016;
    }

    // Set the time of the current frame to be used in the next one.
    timePreviousFrame = SDL_GetTicks();

    //Create Particle - Force Registration Pairs.
    for(int i = 0; i < particles.size(); i++)
    {
        // Applying weight force.
        auto weightForceGenerator = std::make_shared<ParticleGravity>(Vec2(0, 9.8 * particles[i]->mass * PIXELS_PER_METER));
        pfg.Add(particles[i], weightForceGenerator);
        
        // Apply drag force to each particle.
        particles[i]->ApplyForce(pushForce);
        pfg.Add(particles[i], dragGenerator);

        // Apply spring forces.
        if(i == 0)
        {
            auto anchorForceGenerator = std::make_shared<ParticleSpringAnchored>(anchor, restLength, k);
            pfg.Add(particles[i], anchorForceGenerator);
        }
        else
        {
            auto springForceGenerator = std::make_shared<ParticleSpring>(particles[i], particles[i - 1], restLength, k);
            auto invSpringForceGenerator = std::make_shared<ParticleSpring>(particles[i - 1], particles[i], restLength, k);
            pfg.Add(particles[i], springForceGenerator);
            pfg.Add(particles[i - 1], invSpringForceGenerator);
        }
    }

    
    // Update Forces From Particle Force Registry
    pfg.UpdateForces(deltaTime);

    // Preform integration for each particle.
    for (auto particle : particles)
    {
        // Integrate the accleration and velocity to find the new position.
        particle->Integrate(deltaTime);
    }

    // Check boundaries and keep particle inside window.
    for (auto particle : particles)
    {
        // Nasty hardcoded flip in velocity if it touches the limits of the screen
        if (particle->position.x - particle->radius <= 0) {
            particle->position.x = particle->radius;
            particle->velocity.x *= -0.95f;
        }
        else if (particle->position.x + particle->radius >= Graphics::Width()) {
            particle->position.x = Graphics::Width() - particle->radius;
            particle->velocity.x *= -0.95f;
        }
        if (particle->position.y - particle->radius <= 0) {
            particle->position.y = particle->radius;
            particle->velocity.y *= -0.95f;
        }
        else if (particle->position.y + particle->radius >= Graphics::Height()) {
            particle->position.y = Graphics::Height() - particle->radius;
            particle->velocity.y *= -0.95f;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// Render function (called several times per second to draw objects)
///////////////////////////////////////////////////////////////////////////////
void SpringScene::Render() {
    Graphics::ClearScreen(0xFF060224);

    if (leftMouseButtonDown)
        Graphics::DrawLine(particles[particles.size() - 1]->position.x, particles[particles.size() - 1]->position.y, mouseCursor.x, mouseCursor.y, 0xFFFFFFFF);

    // Draw anchor
    Graphics::DrawFillCircle(anchor.x, anchor.y, 10, 0xFFFFFFFF);

    for(int i = 0; i < particles.size(); i++)
    {
        if(i == 0)
        {
            // Draw line from anchor to first particle
            Graphics::DrawLine(anchor.x, anchor.y, particles[i]->position.x, particles[i]->position.y, 0xFFFFFFFF);
            Graphics::DrawFillCircle(particles[i]->position.x, particles[i]->position.y, particles[i]->radius, 0xFFFFFFFF);
        }
        else
        {
            // Draw line from current particle to previous particle.
            Graphics::DrawLine(particles[i]->position.x, particles[i]->position.y, particles[i-1]->position.x, particles[i-1]->position.y, 0xFFFFFFFF);
            Graphics::DrawFillCircle(particles[i]->position.x, particles[i]->position.y, particles[i]->radius, 0xFFFFFFFF);
        }
    }
        
    Graphics::RenderFrame();
}

///////////////////////////////////////////////////////////////////////////////
// Destroy function to delete objects and close the window
///////////////////////////////////////////////////////////////////////////////
void SpringScene::Destroy() {
    Graphics::CloseWindow();
}