#include <Constants.h>
#include "Application.h"

bool Application::IsRunning() {
    return running;
}

///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
void Application::Setup() {
    running = Graphics::OpenWindow();
    particle = new Particle(50, 100, 1.0);
}

///////////////////////////////////////////////////////////////////////////////
// Input processing
///////////////////////////////////////////////////////////////////////////////
void Application::Input() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_QUIT:
                running = false;
                break;
            case SDL_KEYDOWN:
                if (event.key.keysym.sym == SDLK_ESCAPE)
                    running = false;
                break;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// Update function (called several times per second to update objects)
///////////////////////////////////////////////////////////////////////////////
void Application::Update() {
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
    
    // Integrate the accleration and velocity to find the new position.
    particle->acceleration = Vec2(15.0, 9.8 * PIXELS_PER_METER);
    particle->velocity += particle->acceleration * deltaTime;
    particle->position += particle->velocity * deltaTime;

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
        particle->velocity.x *= -0.95f;
    }
    else if (particle->position.y + particle->radius >= Graphics::Height()) {
        particle->position.y = Graphics::Height() - particle->radius;
        particle->velocity.y *= -0.95f;
    }
}

///////////////////////////////////////////////////////////////////////////////
// Render function (called several times per second to draw objects)
///////////////////////////////////////////////////////////////////////////////
void Application::Render() {
    Graphics::ClearScreen(0xFF056263);
    Graphics::DrawFillCircle(particle->position.x, particle->position.y, particle->radius, 0xFFFFFFFF);
    Graphics::RenderFrame();
}

///////////////////////////////////////////////////////////////////////////////
// Destroy function to delete objects and close the window
///////////////////////////////////////////////////////////////////////////////
void Application::Destroy() {
    delete particle;
    Graphics::CloseWindow();
}