#include <random>
#include <iostream>
#include "pfgen.h"
#include "Constants.h"
#include "Scene.h"

void GenerateParticlesInLine(std::vector<std::unique_ptr<Particle>>& particles,
        int numParticles, float verticalSeparation, const Vec2& anchor,
        float width, float mass) 
    {
        for (int i = 0; i < numParticles; i++) {
            float xPosition = width / 2.0;
            float yPosition = anchor.y + ((i+1) * verticalSeparation);
            auto particle = std::make_unique<Particle>(xPosition, yPosition, mass);
            particles.push_back(std::move(particle));
        }
    };

///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
void SpringScene::Setup() {
    running = Graphics::OpenWindow();
    world = std::make_unique<Aerolite::AeroWorld2D>();
    auto particles = std::vector<std::unique_ptr<Particle>>();
    anchor = Vec2(Graphics::Width() / 2, 30);
    GenerateParticlesInLine(particles, NUM_PARTICLES, restLength, anchor, Graphics::Width(), PARTICLE_MASS);
    world->AddParticles(std::move(particles));
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
                auto particles = world->GetParticles();
                Vec2 impulseDirection = (particles[particles.size() - 1]->position - mouseCursor).UnitVector();
                float impulseMagnitude = -(particles[particles.size() - 1]->position - mouseCursor).Magnitude() * 6.0;
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

    auto particles = world->GetParticles();
    Vec2 springForce = ParticleForceGenerators::GenerateAnchoredSpringForce(*particles[0], anchor, restLength, k);
    particles[0]->ApplyForce(springForce);
    for(int i = 1; i < particles.size(); i++)
    {
        int currParticle = i;
        int prevParticle = i - 1;
        Vec2 springForce = ParticleForceGenerators::GenerateSpringForce(*particles[currParticle],
            *particles[prevParticle], restLength, k);
        // Apply drag force to each particle.
        particles[i]->ApplyForce(pushForce);
        particles[i]->ApplyForce(ParticleForceGenerators::GenerateDragForce(*particles[i], 0.1, 0.002));
        particles[currParticle]->ApplyForce(springForce);
        particles[prevParticle]->ApplyForce(-springForce);
    }

    world->Update(deltaTime);

    // Check boundaries and keep particle inside window.
    for (auto& particle : particles)
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

    auto particles = world->GetParticles();
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