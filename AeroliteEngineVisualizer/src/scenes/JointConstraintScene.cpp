#include <random>
#include <iostream>
#include "AeroWorld2D.h"
#include "Collision2D.h"
#include "Contact2D.h"
#include "Constants.h"
#include "pfgen.h"
#include "Scene.h"

// GLOBAL VARIABLES

///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
void JointConstraintScene::Setup() {
    running = Graphics::OpenWindow();

    world = std::make_unique<Aerolite::AeroWorld2D>(-9.8);

    const int NUM_BODIES = 10;
    for (int i = 0; i < NUM_BODIES; i++) {
        real mass = (i == 0) ? 0.0f : 1.0f;
        auto body = std::make_unique<AeroBody2D>(new BoxShape(30, 30), Graphics::Width() / make_real<real>(2.0) - (i * 40), 100, mass);
        world->AddBody2D(std::move(body));
    }

    for (int i = 0; i < NUM_BODIES - 1; i++) {
        std::vector<std::unique_ptr<AeroBody2D>>& bodies = world->GetBodies();
        auto a = bodies[i].get();
        auto b = bodies[i + 1].get();
        auto joint = std::make_unique<JointConstraint>(*a, *b, a->position);
        world->AddConstraint(std::move(joint));
    }
    
}

///////////////////////////////////////////////////////////////////////////////
// Input processing
///////////////////////////////////////////////////////////////////////////////
void JointConstraintScene::Input(SDL_Event event) {
     switch (event.type) {
     case SDL_QUIT:
         running = false;
         break;
     case SDL_KEYDOWN:
         if (event.key.keysym.sym == SDLK_ESCAPE)
             running = false;
         if (event.key.keysym.sym == SDLK_d) {
             debug = !debug;
         }
         break;
     case SDL_MOUSEBUTTONDOWN:
         if (event.button.button == SDL_BUTTON_LEFT) {
             int x, y;
             SDL_GetMouseState(&x, &y);
             // Create and add a new BoxShape at the mouse location
             auto circle = std::make_unique<AeroBody2D>(new CircleShape(50), x, y, make_real<real>(2.0));
             circle->restitution = make_real<real>(1.0);
             circle->friction = make_real<real>(0.4);
             world->AddBody2D(std::move(circle));
         }
         else if (event.button.button == SDL_BUTTON_RIGHT) {
             int x, y;
             SDL_GetMouseState(&x, &y);
             // Create and add a new CircleShape at the mouse location
             auto circle = std::make_unique<AeroBody2D>(PolygonShape::CreateRegularPolygon(5, 50), x, y, make_real<real>(1.0)); // Assuming radius 25 for the circle
             circle->restitution = make_real<real>(1.0);
             circle->friction = make_real<real>(0.4);
             world->AddBody2D(std::move(circle));
         }
         break;
     }
    
}

///////////////////////////////////////////////////////////////////////////////
// Update function (called several times per second to update objects)
///////////////////////////////////////////////////////////////////////////////
void JointConstraintScene::Update() {
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
void JointConstraintScene::Render() {
    Graphics::ClearScreen(0xFF000000);

    //for (auto& joint : world->GetConstraints()) {
    //    const AeroVec2 pa = joint->a.LocalSpaceToWorldSpace(joint->aPoint);
    //    const AeroVec2 pb = joint->b.LocalSpaceToWorldSpace(joint->bPoint);
    //    Graphics::DrawLine(pa.x, pa.y, pb.x, pb.y, 0xFFFFFFFF);
    //}

    for (auto& body : world->GetBodies()) {
        if (body->shape->GetType() == Circle) {
            auto circleShape = dynamic_cast<CircleShape*>(body->shape);
            Graphics::DrawCircle(body->position.x, body->position.y, circleShape->radius, body->rotation, 0xFFFFFFFF);
        }
        else  if (body->shape->GetType() == Box) {
            auto boxShape = dynamic_cast<BoxShape*>(body->shape);
            Graphics::DrawFillPolygon(body->position.x, body->position.y, boxShape->worldVertices, 0xFFFFFFFF);
        }
        else  if (body->shape->GetType() == Polygon) {
            auto boxShape = dynamic_cast<PolygonShape*>(body->shape);
            Graphics::DrawFillPolygon(body->position.x, body->position.y, boxShape->worldVertices, 0xFFFF0000);
        }
    }

    if (debug) {
        for (auto& contact : world->GetContacts())
        {
            Graphics::DrawFillCircle(contact.start.x, contact.start.y, 4, 0xFFFF00FF);
            Graphics::DrawFillCircle(contact.end.x, contact.end.y, 4, 0xFFFF00FF);
            Graphics::DrawLine(contact.start.x, contact.start.y, contact.start.x + contact.normal.x * 15,
                contact.start.y + contact.normal.y * 15, 0xFFFF00FF);
        }
    }

    Graphics::RenderFrame();
}

///////////////////////////////////////////////////////////////////////////////
// Destroy function to delete objects and close the window
///////////////////////////////////////////////////////////////////////////////
void JointConstraintScene::Destroy() {
    Graphics::CloseWindow();
}