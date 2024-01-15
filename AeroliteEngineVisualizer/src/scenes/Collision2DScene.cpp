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
void Collision2DScene::Setup() {
    running = Graphics::OpenWindow();
    auto bigBall = std::make_unique<Body2D>(new CircleShape(100), 100, 100, 5.0);
    auto smallBall = std::make_unique<Body2D>(new CircleShape(50), Graphics::Width() / 2.0, Graphics::Height() / 2.0, 0.0);

    bodies.push_back(std::move(bigBall));
    bodies.push_back(std::move(smallBall));
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
        case SDL_MOUSEMOTION:
            int x, y;
            SDL_GetMouseState(&x, &y);
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

    // Create body - Force Registration Pairs.
    for(auto& body : bodies) {

    }

    for(auto& body : bodies) {
        /*body->Update(deltaTime);*/
        body->isColliding = false; // Temporary until we have collision detection engine setup.
    }

    // Check all rigid bodies for collision
    for(int i = 0; i < bodies.size() - 1; i++)
    {
        for(int j = i+1; j < bodies.size(); j++)
        {   
            Aerolite::Contact2D contact = Aerolite::Contact2D();
            if(CollisionDetection2D::IsColliding(*bodies[i], *bodies[j], contact))
            {
                // Here we have the contact information inside the contact object.
                Graphics::DrawFillCircle(contact.start.x, contact.start.y, 3, 0xFFFF00FF);
                Graphics::DrawFillCircle(contact.end.x, contact.end.y, 3, 0xFFFF00FF);
                Graphics::DrawLine(contact.start.x, contact.start.y, contact.start.x + contact.normal.x * 15, contact.start.y + contact.normal.y * 15, 0xFFFF00FF);
                bodies[i]->isColliding = true;
                bodies[j]->isColliding = true;
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// Render function (called several times per second to draw objects)
///////////////////////////////////////////////////////////////////////////////
void Collision2DScene::Render() {
    for(auto& body : bodies) {

        uint32_t color = body->isColliding ? 0xFF0000FF : 0xFFFFFFFF;
        if(body->shape->GetType() == Circle) {
            auto circleShape = dynamic_cast<CircleShape*>(body->shape);
            Graphics::DrawCircle(body->position.x, body->position.y, circleShape->radius, body->rotation, color);
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
void Collision2DScene::Destroy() {
    Graphics::CloseWindow();
}