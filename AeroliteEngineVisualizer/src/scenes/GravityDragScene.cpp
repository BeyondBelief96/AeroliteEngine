#include <Constants.h>
#include <random>
#include <pfgen.h>
#include "Scene.h"

// GLOBAL VARIABLES
Vec2 gravity(0.0, 9.8 * PIXELS_PER_METER);
auto gravityForceGenerator = std::make_shared<ParticleGravity>(gravity);
auto dragForceGenerator = std::make_shared<ParticleDrag>(1, 0.03f);

///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
void GravityDragScene::Setup() {
    running = Graphics::OpenWindow();
    //for (int i = 0; i < 7000; i++)
    //{
    //    int x, y;
    //    std::random_device rd;
    //    std::mt19937 gen(rd()); // Mersenne Twister engine

    //    // Define the range
    //    int lower_boundx = 0;
    //    int upper_boundx = Graphics::Width();
    //    int lower_boundy = 0;
    //    int upper_boundy = Graphics::Height();

    //    std::uniform_int_distribution<int> distx(lower_boundx, upper_boundx);
    //    std::uniform_int_distribution<int> disty(lower_boundy, upper_boundy);
    //    auto smallBall = std::make_shared<Particle>(distx(gen), disty(gen), 1.0);
    //    particles.push_back(smallBall);
    //}
    auto smallBall = std::make_shared<Particle>(50, 100, 1.0);
    particles.push_back(smallBall);

    // Initializing liquid object to simulate drag forces.
    liquid.x = 0;
    liquid.y = Graphics::Height() / 2;
    liquid.w = Graphics::Width();
    liquid.h = Graphics::Height() / 2;
}

///////////////////////////////////////////////////////////////////////////////
// Input processing
///////////////////////////////////////////////////////////////////////////////
void GravityDragScene::Input() {
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
            case SDL_MOUSEBUTTONDOWN:
                if (event.button.button == SDL_BUTTON_LEFT)
                {
                    int x, y;
                    SDL_GetMouseState(&x, &y);
                    auto particle = std::make_shared<Particle>(x, y, 1.0);
                    particle->radius = 5;
                    particles.push_back(particle);
                }
                break;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// Update function (called several times per second to update objects)
///////////////////////////////////////////////////////////////////////////////
void GravityDragScene::Update() {
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
        pfg.Add(particle, gravityForceGenerator);

        if (particle->position.y >= liquid.y) {
            pfg.Add(particle, dragForceGenerator);
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
void GravityDragScene::Render() {
    Graphics::ClearScreen(0xFF6f95d6);

    // Draw the liquid
    Graphics::DrawFillRect(liquid.x + liquid.w / 2, liquid.y + liquid.h / 2,
        liquid.w, liquid.h, 0xFF331504);

    for (auto particle : particles)
        Graphics::DrawFillCircle(particle->position.x, particle->position.y, particle->radius, 0xFFFFFFFF);
    Graphics::RenderFrame();
}

///////////////////////////////////////////////////////////////////////////////
// Destroy function to delete objects and close the window
///////////////////////////////////////////////////////////////////////////////
void GravityDragScene::Destroy() {
    Graphics::CloseWindow();
}