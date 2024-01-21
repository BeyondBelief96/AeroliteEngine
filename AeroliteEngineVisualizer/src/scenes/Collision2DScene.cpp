#include <random>
#include <iostream>
#include "Collision2D.h"
#include "Contact2D.h"
#include "Constants.h"
#include "pfgen.h"
#include "Scene.h"

#pragma warning(disable : 4244)

// GLOBAL VARIABLES

///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
void Collision2DScene::Setup() {
    running = Graphics::OpenWindow();
    world = std::make_unique<AeroWorld2D>(0);
    auto smallBox = std::make_unique<Body2D>(new BoxShape(100, 100), 100, 100, make_real<real>(5.0));
    auto bigBox = std::make_unique<Body2D>(new BoxShape(200, 200), Graphics::Width() / make_real<real>(2.0), Graphics::Height() / make_real<real>(2.0), make_real<real>(0.0));

    world->AddBody2D(std::move(smallBox));
    world->AddBody2D(std::move(bigBox));
}

///////////////////////////////////////////////////////////////////////////////
// Input processing
///////////////////////////////////////////////////////////////////////////////
void Collision2DScene::Input() {
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
        case SDL_MOUSEMOTION:
            int x, y;
            SDL_GetMouseState(&x, &y);
            std::vector<std::unique_ptr<Body2D>>& bodies = world->GetBodies();
            bodies[0]->position.x = x;
            bodies[0]->position.y = y;
            break;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// Update function (called several times per second to update objects)
///////////////////////////////////////////////////////////////////////////////
void Collision2DScene::Update() {

    //TEMPORARY FOR DEBUGGIN PURPOSES. PUT BACK IN RENDER FUNCTION EVENTUALLY.
    Graphics::ClearScreen(0xFF000000);
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
void Collision2DScene::Render() {
    std::vector<std::unique_ptr<Body2D>>& bodies = world->GetBodies();
    for(auto& body : bodies) {
        if(body->shape->GetType() == Circle) {
            auto circleShape = dynamic_cast<CircleShape*>(body->shape);
            Graphics::DrawCircle(body->position.x, body->position.y, circleShape->radius, body->rotation, 0xFFFFFFFF);
        }
        else  if(body->shape->GetType() == Box) {
            auto boxShape = dynamic_cast<BoxShape*>(body->shape);
            Graphics::DrawPolygon(body->position.x, body->position.y, boxShape->worldVertices, 0xFFFFFFFF);  
        }

        auto contacts = world->GetContacts();

        for (auto& contact : contacts)
        {
            Graphics::DrawCircle(contact.start.x, contact.start.y, 5, make_real<real>(0.0), 0xFF00FFFF);
            Graphics::DrawCircle(contact.end.x, contact.end.y, 2, make_real<real>(0.0), 0xFF00FFFF);
        }
    }

    Graphics::RenderFrame();
}

///////////////////////////////////////////////////////////////////////////////
// Destroy function to delete objects and close the window
///////////////////////////////////////////////////////////////////////////////
void Collision2DScene::Destroy() {
    Graphics::CloseWindow();
}