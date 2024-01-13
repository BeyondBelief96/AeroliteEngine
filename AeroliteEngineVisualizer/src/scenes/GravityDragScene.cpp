#include <random>
#include "Constants.h"
#include "AeroWorld2D.h"
#include "pfgen.h"
#include "Scene.h"

// GLOBAL VARIABLES
Vec2 gravity(0.0, 9.8 * PIXELS_PER_METER);

///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
void GravityDragScene::Setup() {
    running = Graphics::OpenWindow();
    world = std::make_unique<AeroWorld2D>();

    for (int i = 0; i < 10; i++)
    {
        int x, y;
        std::random_device rd;
        std::mt19937 gen(rd()); // Mersenne Twister engine

        // Define the range
        int lower_boundx = 0;
        int upper_boundx = Graphics::Width();
        int lower_boundy = 0;
        int upper_boundy = Graphics::Height();

        std::uniform_int_distribution<int> distx(lower_boundx, upper_boundx);
        std::uniform_int_distribution<int> disty(lower_boundy, upper_boundy);
        auto smallBall = std::make_unique<Particle>(distx(gen), disty(gen), 1.0);
        world->AddParticle(std::move(smallBall));
    }
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
                    auto particle = std::make_unique<Particle>(x, y, 1.0);
                    particle->radius = 5;
                    world->AddParticle(std::move(particle));
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

    for (auto& particle : world->GetParticles())
    {
        particle->ApplyForce(pushForce);
        if (particle->position.y >= liquid.y) {
            particle->ApplyForce(ParticleForceGenerators::GenerateDragForce(*particle, 0.03, 0.03));
        }
    }

    world->Update(deltaTime);

    // Check boundaries and keep particle inside window.
    for (auto& particle : world->GetParticles())
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

    for (auto& particle : world->GetParticles())
        Graphics::DrawFillCircle(particle->position.x, particle->position.y, particle->radius, 0xFFFFFFFF);
    Graphics::RenderFrame();
}

///////////////////////////////////////////////////////////////////////////////
// Destroy function to delete objects and close the window
///////////////////////////////////////////////////////////////////////////////
void GravityDragScene::Destroy() {
    Graphics::CloseWindow();
}