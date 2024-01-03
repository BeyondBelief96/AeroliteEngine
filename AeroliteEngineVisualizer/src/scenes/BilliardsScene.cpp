#include "Constants.h"
#include <random>
#include "pfgen.h"
#include "Scene.h"

// GLOBAL VARIABLES
auto frictionForceGenerator = std::make_shared<ParticleFriction>(10 * PIXELS_PER_METER);

///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
void BilliardScene::Setup() {
    running = Graphics::OpenWindow();

    auto smallBall = std::make_shared<Particle>(50, 100, 1.0);
    particles.push_back(smallBall);
}

///////////////////////////////////////////////////////////////////////////////
// Input processing
///////////////////////////////////////////////////////////////////////////////
void BilliardScene::Input() {
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
                Vec2 impulseDirection = (particles[0]->position - mouseCursor).UnitVector();
                float impulseMagnitude = (particles[0]->position - mouseCursor).Magnitude() * 5.0;
                particles[0]->velocity = impulseDirection * impulseMagnitude;
            }
            break;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// Update function (called several times per second to update objects)
///////////////////////////////////////////////////////////////////////////////
void BilliardScene::Update() {
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

    // Create Particle - Force Registration Pairs.
    for (auto particle : particles)
    {
        particle->ApplyForce(pushForce);
        pfg.Add(particle, frictionForceGenerator);
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
void BilliardScene::Render() {
    Graphics::ClearScreen(0xFF1C520C);
    
    if(leftMouseButtonDown)
        Graphics::DrawLine(particles[0]->position.x, particles[0]->position.y, mouseCursor.x, mouseCursor.y, 0xFF000000);
    for (auto particle : particles)
        Graphics::DrawFillCircle(particle->position.x, particle->position.y, particle->radius, 0xFFFFFFFF);
    Graphics::RenderFrame();
}

///////////////////////////////////////////////////////////////////////////////
// Destroy function to delete objects and close the window
///////////////////////////////////////////////////////////////////////////////
void BilliardScene::Destroy() {
    Graphics::CloseWindow();
}