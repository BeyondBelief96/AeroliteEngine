#include <random>
#include <iostream>
#include "Collision2D.h"
#include "Contact2D.h"
#include "Constants.h"
#include "pfgen.h"
#include "Scene.h"

// GLOBAL VARIABLES

///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
void CollisionProjectionResolutionScene::Setup() {
    running = true;

    world = std::make_unique<AeroWorld2D>();
    auto bigBall = std::make_unique<AeroBody2D>(new CircleShape(100), Graphics::Width() / make_real<real>(2.0), Graphics::Height() / make_real<real>(2.0), make_real<real>(0.0));
    world->AddBody2D(std::move(bigBall));
}

///////////////////////////////////////////////////////////////////////////////
// Input processing
///////////////////////////////////////////////////////////////////////////////
void CollisionProjectionResolutionScene::Input(SDL_Event event) {
     switch (event.type) {
     case SDL_QUIT:
         running = false;
         break;
     case SDL_KEYDOWN:
         if (event.key.keysym.sym == SDLK_ESCAPE)
             running = false;
         break;
     case SDL_MOUSEBUTTONDOWN:
         if(event.button.button == SDL_BUTTON_LEFT) {
             int x, y;
             SDL_GetMouseState(&x, &y);
             auto ball = std::make_unique<AeroBody2D>(new CircleShape(50), x, y, make_real<real>(1.0));
             world->AddBody2D(std::move(ball));
         }
         break;
     }
}

///////////////////////////////////////////////////////////////////////////////
// Update function (called several times per second to update objects)
///////////////////////////////////////////////////////////////////////////////
void CollisionProjectionResolutionScene::Update() {

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

    world->Update(deltaTime);
}

///////////////////////////////////////////////////////////////////////////////
// Render function (called several times per second to draw objects)
///////////////////////////////////////////////////////////////////////////////
void CollisionProjectionResolutionScene::Render() {
    Graphics::ClearScreen(0xFF000000);
    std::vector<std::unique_ptr<AeroBody2D>>& bodies = world->GetBodies();
    for(auto& body : bodies) {
        if(body->shape->GetType() == Circle) {
            auto circleShape = dynamic_cast<CircleShape*>(body->shape);
            Graphics::DrawFillCircle(body->position.x, body->position.y, circleShape->radius, 0xFFFFFFFF);
        }
        else  if(body->shape->GetType() == Box) {
            auto boxShape = dynamic_cast<BoxShape*>(body->shape);
            Graphics::DrawPolygon(body->position.x, body->position.y, boxShape->worldVertices, 0xFFFFFFFF);  
        }
    }

    Graphics::RenderFrame();
}

///////////////////////////////////////////////////////////////////////////////
// Destroy function to delete objects and close the window
///////////////////////////////////////////////////////////////////////////////
void CollisionProjectionResolutionScene::Destroy() {
    Graphics::CloseWindow();
}