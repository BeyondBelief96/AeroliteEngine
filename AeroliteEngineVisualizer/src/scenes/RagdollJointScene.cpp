#include <random>
#include <iostream>
#include "AeroWorld2D.h"
#include "Collision2D.h"
#include "Constants.h"
#include "imgui.h"
#include "scene.h"

// GLOBAL VARIABLES

///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
void RagdollJointScene::Setup() {
    running = true;

    world = std::make_unique<AeroWorld2D>();

    // Add rag doll parts (rigid bodies)
    const auto bob = world->CreateBody2D(std::make_shared< CircleShape>(5), Graphics::Width() / make_real<real>(2.0), Graphics::Height() / make_real<real>(2.0) - 200, make_real<real>(0.0));
    const auto head = world->CreateBody2D(std::make_shared< CircleShape>(25), bob->position.x, bob->position.y + 70, make_real<real>(5.0));
    const auto torso = world->CreateBody2D(std::make_shared< BoxShape>(50, 100), head->position.x, head->position.y + 80, make_real<real>(3.0));
    const auto leftArm = world->CreateBody2D(std::make_shared<BoxShape>(15, 70), torso->position.x - 32, torso->position.y -10, make_real<real>(1.0));
    const auto rightArm = world->CreateBody2D(std::make_shared<BoxShape>(15, 70), torso->position.x + 32, torso->position.y - 10, make_real<real>(1.0));
    const auto leftLeg = world->CreateBody2D(std::make_shared<BoxShape>(20, 90), torso->position.x - 20, torso->position.y + 97, make_real<real>(1.0));
    const auto rightLeg = world->CreateBody2D(std::make_shared<BoxShape>(20, 90), torso->position.x + 20, torso->position.y + 97, make_real<real>(1.0));

    world->AddJointConstraint(bob, head, bob->position);
    world->AddJointConstraint(head, torso, head->position + Vec2(0, 25));
    world->AddJointConstraint(torso, leftArm, torso->position + Vec2(-28, -45));
    world->AddJointConstraint(torso, rightArm, torso->position + Vec2(+28, -45));
    world->AddJointConstraint(torso, leftLeg, torso->position + Vec2(-20, 50));
    world->AddJointConstraint(torso, rightLeg, torso->position + Vec2(20, 50));

    const auto floor = world->CreateBody2D(std::make_shared<BoxShape>(Graphics::Width() - 50, 50),
                                           Graphics::Width() / make_real<real>(2.0), Graphics::Height() - 50, make_real<real>(0.0));
    floor->restitution = make_real<real>(0.1);

    const auto leftWall = world->CreateBody2D(std::make_shared<BoxShape>(50, Graphics::Height() - 50),
                                              0, Graphics::Height() / make_real<real>(2.0), make_real<real>(0.0));
    leftWall->restitution = make_real<real>(0.1);

    const auto rightWall = world->CreateBody2D(std::make_shared<BoxShape>(50, Graphics::Height() - 50),
                                               Graphics::Width(), Graphics::Height() / make_real<real>(2.0), make_real<real>(0.0));
    rightWall->restitution = make_real<real>(0.1);


}

///////////////////////////////////////////////////////////////////////////////
// Input processing
///////////////////////////////////////////////////////////////////////////////
void RagdollJointScene::Input(const SDL_Event event) {
     switch (event.type) {
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
             // Create and add a std::make_shared< BoxShape at the mouse location
             const auto circle = world->CreateBody2D(std::make_shared<CircleShape>(50), x, y, make_real<real>(2.0));
             circle->restitution = make_real<real>(1.0);
             circle->friction = make_real<real>(0.4);
         }
         else if (event.button.button == SDL_BUTTON_RIGHT) {
             int x, y;
             SDL_GetMouseState(&x, &y);
             // Create and add a std::make_shared< CircleShape at the mouse location
             const auto circle = world->CreateBody2D(PolygonShape::CreateRegularPolygon(5, 50), x, y, make_real<real>(1.0)); // Assuming radius 25 for the circle
             circle->restitution = make_real<real>(1.0);
             circle->friction = make_real<real>(0.4);
         }
         break;
     case SDL_MOUSEMOTION:
         int x, y;
         SDL_GetMouseState(&x, &y);
         const auto mouse = Vec2(x, y);
         const std::shared_ptr<AeroBody2D> bob = world->GetBodies()[0];
         const Vec2 direction = (mouse - bob->position).UnitVector();
         const real speed = make_real<real>(5.0);
         bob->position += direction * speed;
     }
    
}

///////////////////////////////////////////////////////////////////////////////
// Update function (called several times per second to update objects)
///////////////////////////////////////////////////////////////////////////////
void RagdollJointScene::Update() {
    // Check if we are too fast, and if so, waste some milliseconds until we reach
    // MILLISECONDS_PER_FRAME.
    const int timeToWait = MILLISECS_PER_FRAME - (SDL_GetTicks() - timePreviousFrame);
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
void RagdollJointScene::Render() {
    Graphics::ClearScreen(0xFF000000);

    const AeroBody2D* bob = world->GetBodies()[0].get();
    const AeroBody2D* head = world->GetBodies()[1].get();
    Graphics::DrawLine(bob->position.x, bob->position.y, head->position.x, head->position.y, 0xFFFFFFFF);

    for (const auto& joint : world->GetConstraints()) {
        if (debug) {
            const Vec2 anchorPoint = joint->a->LocalSpaceToWorldSpace(joint->aPoint);
            Graphics::DrawFillCircle(anchorPoint.x, anchorPoint.y, 3, 0xFF0000FF);
        }
    }

    for (const auto& body : world->GetBodies()) {
        if (body->shape->GetType() == Circle) {
	        const auto circleShape =  std::dynamic_pointer_cast<CircleShape>(body->shape);
            Graphics::DrawCircle(body->position.x, body->position.y, circleShape->radius, body->rotation, 0xFFFFFFFF);
        }
        else  if (body->shape->GetType() == Box) {
	        const auto boxShape =  std::dynamic_pointer_cast<BoxShape>(body->shape);
            Graphics::DrawPolygon(body->position.x, body->position.y, boxShape->worldVertices, 0xFFFFFFFF);
        }
        else  if (body->shape->GetType() == Polygon) {
	        const auto boxShape =  std::dynamic_pointer_cast<PolygonShape>(body->shape);
            Graphics::DrawPolygon(body->position.x, body->position.y, boxShape->worldVertices, 0xFFFF0000);
        }
    }
}