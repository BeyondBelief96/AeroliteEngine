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
    running = Graphics::OpenWindow();

    auto bigBall = std::make_unique<Body2D>(new CircleShape(100), Graphics::Width() / 2.0, Graphics::Height() / 2.0, 0.0);

    bodies.push_back(std::move(bigBall));
}

///////////////////////////////////////////////////////////////////////////////
// Input processing
///////////////////////////////////////////////////////////////////////////////
void CollisionProjectionResolutionScene::Input() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
        case SDL_QUIT:
            running = false;
            break;
        case SDL_KEYDOWN:
            if (event.key.keysym.sym == SDLK_ESCAPE)
                running = false;
        case SDL_MOUSEBUTTONDOWN:
            if(event.button.button == SDL_BUTTON_LEFT) {
                int x, y;
                SDL_GetMouseState(&x, &y);
                auto ball = std::make_unique<Body2D>(new CircleShape(50), x, y, 1.0);
                bodies.push_back(std::move(ball));
            }
            break;
        }
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

    // Create body - Force Registration Pairs.
    for(auto& body : bodies) {
         Vec2 weight = Vec2(0, body->mass * 9.8 * PIXELS_PER_METER);
         body->AddForce(weight);

         Vec2 wind = Vec2(2.0 * PIXELS_PER_METER, 0.0);
         body->AddForce(wind);
    }

    for(auto& body : bodies) {
        body->Update(deltaTime);
    }

    for (auto& body : bodies) {
        body->Update(deltaTime);
        body->isColliding = false; // Temporary until we have collision detection engine setup.
    }
   

    // Check all rigid bodies for collision
    for(int i = 0; i < bodies.size() - 1; i++)
    {
        for(int j = i+1; j < bodies.size(); j++)
        {   
            Aerolite::Contact2D contact;
            if(CollisionDetection2D::IsColliding(*bodies[i], *bodies[j], contact))
            {
                contact.ResolveCollision();
                bodies[i]->isColliding = true;
                bodies[j]->isColliding = true;
            }
        }
    }

    // Check the boundaries of the window applying a hardcoded bounce flip in velocity
    for (auto& body : bodies) {
        if (body->shape->GetType() == Aerolite::ShapeType::Circle) {
            auto circleShape = dynamic_cast<CircleShape*>(body->shape);
            if (body->position.x - circleShape->radius <= 0) {
                body->position.x = circleShape->radius;
                body->velocity.x *= -0.9;
            } else if (body->position.x + circleShape->radius >= Graphics::Width()) {
                body->position.x = Graphics::Width() - circleShape->radius;
                body->velocity.x *= -0.9;
            }
            if (body->position.y - circleShape->radius <= 0) {
                body->position.y = circleShape->radius;
                body->velocity.y *= -0.9;
            } else if (body->position.y + circleShape->radius >= Graphics::Height()) {
                body->position.y = Graphics::Height() - circleShape->radius;
                body->velocity.y *= -0.9;
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// Render function (called several times per second to draw objects)
///////////////////////////////////////////////////////////////////////////////
void CollisionProjectionResolutionScene::Render() {
    Graphics::ClearScreen(0xFF000000);
    for(auto& body : bodies) {

        uint32_t color = body->isColliding ? 0xFF0000FF : 0xFFFFFFFF;
        if(body->shape->GetType() == Circle) {
            auto circleShape = dynamic_cast<CircleShape*>(body->shape);
            Graphics::DrawFillCircle(body->position.x, body->position.y, circleShape->radius, color);
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