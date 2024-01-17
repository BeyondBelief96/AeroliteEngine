#include <random>
#include <iostream>
#include "AeroWorld2D.h"
#include "Collision2D.h"
#include "Contact2D.h"
#include "Constants.h"
#include "pfgen.h"
#include "scene.h"

// GLOBAL VARIABLES

///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
void RagdollJointScene::Setup() {
    running = Graphics::OpenWindow();

    world = std::make_unique<Aerolite::AeroWorld2D>();

    // Add ragdoll parts (rigid bodies)
    auto bob = std::make_unique<Body2D>(new CircleShape(5), Graphics::Width() / 2.0, Graphics::Height() / 2.0 - 200, 0.0);
    auto head = std::make_unique<Body2D>(new CircleShape(25), bob->position.x, bob->position.y + 70, 5.0);
    auto torso = std::make_unique<Body2D>(new BoxShape(50, 100), head->position.x, head->position.y + 80, 3.0);
    auto leftArm = std::make_unique<Body2D>(new BoxShape(15, 70), torso->position.x - 32, torso->position.y -10, 1.0);
    auto rightArm = std::make_unique<Body2D>(new BoxShape(15, 70), torso->position.x + 32, torso->position.y - 10, 1.0);
    auto leftLeg = std::make_unique<Body2D>(new BoxShape(20, 90), torso->position.x - 20, torso->position.y + 97, 1.0);
    auto rightLeg = std::make_unique<Body2D>(new BoxShape(20, 90), torso->position.x + 20, torso->position.y + 97, 1.0);

    auto string = std::make_unique<JointConstraint>(*bob, *head, bob->position);
    auto neck = std::make_unique<JointConstraint>(*head, *torso, head->position + Vec2(0, 25));
    auto leftShoulder = std::make_unique<JointConstraint>(*torso, *leftArm, torso->position + Vec2(-28, -45));
    auto rightShoulder = std::make_unique<JointConstraint>(*torso, *rightArm, torso->position + Vec2(+28, -45));
    auto leftHip = std::make_unique<JointConstraint>(*torso, *leftLeg, torso->position + Vec2(-20, 50));
    auto rightHip = std::make_unique<JointConstraint>(*torso, *rightLeg, torso->position + Vec2(20, 50));

    world->AddBody2D(std::move(bob));
    world->AddBody2D(std::move(head));
    world->AddBody2D(std::move(torso));
    world->AddBody2D(std::move(leftArm));
    world->AddBody2D(std::move(rightArm));
    world->AddBody2D(std::move(leftLeg));
    world->AddBody2D(std::move(rightLeg));

    world->AddConstraint(std::move(string));
    world->AddConstraint(std::move(neck));
    world->AddConstraint(std::move(leftShoulder));
    world->AddConstraint(std::move(rightShoulder));
    world->AddConstraint(std::move(leftHip));
    world->AddConstraint(std::move(rightHip));

    auto floor = std::make_unique<Aerolite::Body2D>(new BoxShape(Graphics::Width() - 50, 50),
        Graphics::Width() / 2.0, Graphics::Height() - 50, 0.0);
    floor->restitution = 0.1;

    auto leftWall = std::make_unique<Aerolite::Body2D>(new BoxShape(50, Graphics::Height() - 50),
        0, Graphics::Height() / 2.0, 0.0);
    leftWall->restitution = 0.1;

    auto rightWall = std::make_unique<Aerolite::Body2D>(new BoxShape(50, Graphics::Height() - 50),
        Graphics::Width(), Graphics::Height() / 2.0, 0.0);
    rightWall->restitution = 0.1;

    world->AddBody2D(std::move(floor));
    world->AddBody2D(std::move(leftWall));
    world->AddBody2D(std::move(rightWall));


}

///////////////////////////////////////////////////////////////////////////////
// Input processing
///////////////////////////////////////////////////////////////////////////////
void RagdollJointScene::Input() {
    SDL_Event event;
    //static std::default_random_engine engine(std::random_device{}()); // Random number engine
    //static std::uniform_int_distribution<int> distribution(0, 1);    // Distribution to generate either 0 or 1
    while (SDL_PollEvent(&event)) {
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
                auto circle = std::make_unique<Body2D>(new CircleShape(50), x, y, 2.0);
                circle->restitution = 1.0;
                circle->friction = 0.4;
                world->AddBody2D(std::move(circle));
            }
            else if (event.button.button == SDL_BUTTON_RIGHT) {
                int x, y;
                SDL_GetMouseState(&x, &y);
                // Create and add a new CircleShape at the mouse location
                auto circle = std::make_unique<Body2D>(PolygonShape::CreateRegularPolygon(5, 50), x, y, 1.0); // Assuming radius 25 for the circle
                circle->restitution = 1.0;
                circle->friction = 0.4;
                world->AddBody2D(std::move(circle));
            }
            break;
        case SDL_MOUSEMOTION:
            int x, y;
            SDL_GetMouseState(&x, &y);
            Vec2 mouse = Vec2(x, y);
            Aerolite::Body2D* bob = world->GetBodies()[0].get();
            Vec2 direction = (mouse - bob->position).UnitVector();
            real speed = 5.0;
            bob->position += direction * speed;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// Update function (called several times per second to update objects)
///////////////////////////////////////////////////////////////////////////////
void RagdollJointScene::Update() {
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
void RagdollJointScene::Render() {
    Graphics::ClearScreen(0xFF000000);

    Body2D* bob = world->GetBodies()[0].get();
    Body2D* head = world->GetBodies()[1].get();
    Graphics::DrawLine(bob->position.x, bob->position.y, head->position.x, head->position.y, 0xFFFFFFFF);

    for (auto& joint : world->GetConstraints()) {
        if (debug) {
            const Vec2 anchorPoint = joint->a.LocalSpaceToWorldSpace(joint->aPoint);
            Graphics::DrawFillCircle(anchorPoint.x, anchorPoint.y, 3, 0xFF0000FF);
        }
    }

    for (auto& body : world->GetBodies()) {
        if (body->shape->GetType() == Circle) {
            auto circleShape = dynamic_cast<CircleShape*>(body->shape);
            Graphics::DrawCircle(body->position.x, body->position.y, circleShape->radius, body->rotation, 0xFFFFFFFF);
        }
        else  if (body->shape->GetType() == Box) {
            auto boxShape = dynamic_cast<BoxShape*>(body->shape);
            Graphics::DrawPolygon(body->position.x, body->position.y, boxShape->worldVertices, 0xFFFFFFFF);
        }
        else  if (body->shape->GetType() == Polygon) {
            auto boxShape = dynamic_cast<PolygonShape*>(body->shape);
            Graphics::DrawPolygon(body->position.x, body->position.y, boxShape->worldVertices, 0xFFFF0000);
        }
    }

    /*if (debug) {
        for (auto& contact : world->GetContacts())
        {
            Graphics::DrawFillCircle(contact.start.x, contact.start.y, 4, 0xFFFF00FF);
            Graphics::DrawFillCircle(contact.end.x, contact.end.y, 4, 0xFFFF00FF);
            Graphics::DrawLine(contact.start.x, contact.start.y, contact.start.x + contact.normal.x * 15,
                contact.start.y + contact.normal.y * 15, 0xFFFF00FF);
        }
    }*/

    Graphics::RenderFrame();
}

///////////////////////////////////////////////////////////////////////////////
// Destroy function to delete objects and close the window
///////////////////////////////////////////////////////////////////////////////
void RagdollJointScene::Destroy() {
    Graphics::CloseWindow();
}