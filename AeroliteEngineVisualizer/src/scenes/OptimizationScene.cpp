#include <random>
#include <iostream>
#include <filesystem>
#include <SDL2/SDL_image.h>
#include "AeroWorld2D.h"
#include "Collision2D.h"
#include "Contact2D.h"
#include "Constants.h"
#include "pfgen.h"
#include "Scene.h"

///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
void OptimizationScene::Setup() {

    running = Graphics::OpenWindow();

    world = std::make_unique<Aerolite::AeroWorld2D>(-9.8);

    // Add bird
    auto bird = std::make_unique<Body2D>(new CircleShape(45), 100, Graphics::Height() / 2.0 + 220, 3.0);
    world->AddBody2D(std::move(bird));

    // Add a floor and walls to contain objects objects
    auto floor = std::make_unique<Body2D>(new BoxShape(Graphics::Width() - 50, 50),
        Graphics::Width() / 2.0, Graphics::Height() / 2.0 + 290, 0.0);
    auto leftFence = std::make_unique<Body2D>(new BoxShape(50, Graphics::Height() - 200), 0, Graphics::Height() / 2.0 - 35, 0.0);
    auto rightFence = std::make_unique<Body2D>(new BoxShape(50, Graphics::Height() - 200), Graphics::Width(),
        Graphics::Height() / 2.0 - 35, 0.0);
    

    // Add a stack of boxes
    for (int i = 1; i <= 4; i++) {
        float mass = 10.0 / (float)i;
        auto box = std::make_unique<Body2D>(new BoxShape(50, 50), 600, floor->position.y - i * 55, mass);
        box->friction = 0.9;
        box->restitution = 0.1;
        world->AddBody2D(std::move(box));
    }

    // Add structure with blocks
    auto plank1 = std::make_unique<Body2D>(new BoxShape(50, 150), Graphics::Width() / 2.0 + 20, floor->position.y - 100, 5.0);
    auto plank2 = std::make_unique<Body2D>(new BoxShape(50, 150), Graphics::Width() / 2.0 + 180, floor->position.y - 100, 5.0);
    auto plank3 = std::make_unique<Body2D>(new BoxShape(250, 25), Graphics::Width() / 2.0 + 100.0f, floor->position.y - 200, 2.0);
   

    // Add a triangle polygon
    std::vector<Vec2> triangleVertices = { Vec2(30, 30), Vec2(-30, 30), Vec2(0, -30) };
    auto triangle = std::make_unique<Body2D> (new PolygonShape(triangleVertices), plank3->position.x, plank3->position.y - 50, 0.5);

    // Add a pyramid of boxes
    int numRows = 5;
    for (int col = 0; col < numRows; col++) {
        for (int row = 0; row < col; row++) {
            float x = (plank3->position.x + 200.0f) + col * 50.0f - (row * 25.0f);
            float y = (floor->position.y - 50.0f) - row * 52.0f;
            float mass = (5.0f / (row + 1.0f));
            auto box = std::make_unique<Body2D>(new BoxShape(50, 50), x, y, mass);
            box->friction = 0.9;
            box->restitution = 0.0;
            world->AddBody2D(std::move(box));
        }
    }

    // Add a bridge of connected steps and joints
    int numSteps = 10;
    int spacing = 33;
    auto startStep = std::make_unique<Body2D>(new BoxShape(80, 20), 200, 200, 0.0);
    Body2D* last = floor.get();
    for (int i = 1; i <= numSteps; i++) {
        float x = startStep->position.x + 30 + (i * spacing);
        float y = startStep->position.y + 20;
        float mass = (i == numSteps) ? 0.0 : 3.0;
        auto step = std::make_unique<Body2D>(new CircleShape(15), x, y, mass);
        auto joint = std::make_unique<JointConstraint>(*last, *step, step->position);
        last = step.get();
        world->AddConstraint(std::move(joint));
        world->AddBody2D(std::move(step));
    }

    auto endStep = std::make_unique<Body2D>(new BoxShape(80, 20), last->position.x + 60, last->position.y - 20, 0.0);

    auto pig1 = std::make_unique<Body2D>(new CircleShape(30), plank1->position.x + 80, floor->position.y - 50, 3.0);
    auto pig2 = std::make_unique<Body2D>(new CircleShape(30), plank2->position.x + 400, floor->position.y - 50, 3.0);
    auto pig3 = std::make_unique<Body2D>(new CircleShape(30), plank2->position.x + 460, floor->position.y - 50, 3.0);
    auto pig4 = std::make_unique<Body2D>(new CircleShape(30), 220, 130, 1.0);


    world->AddBody2D(std::move(triangle));
    world->AddBody2D(std::move(startStep));
    world->AddBody2D(std::move(endStep));
    world->AddBody2D(std::move(floor));
    world->AddBody2D(std::move(leftFence));
    world->AddBody2D(std::move(rightFence));
    world->AddBody2D(std::move(plank1));
    world->AddBody2D(std::move(plank2));
    world->AddBody2D(std::move(plank3));
    world->AddBody2D(std::move(pig1));
    world->AddBody2D(std::move(pig2));
    world->AddBody2D(std::move(pig3));
    world->AddBody2D(std::move(pig4));
}

///////////////////////////////////////////////////////////////////////////////
// Input processing
///////////////////////////////////////////////////////////////////////////////
void OptimizationScene::Input() {
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
            if (event.key.keysym.sym == SDLK_UP)
                world->GetBodies()[0]->ApplyImpulseLinear(Vec2(0.0, -600.0));
            if (event.key.keysym.sym == SDLK_LEFT)
                world->GetBodies()[0]->ApplyImpulseLinear(Vec2(-400.0, 0.0));
            if (event.key.keysym.sym == SDLK_RIGHT)
                world->GetBodies()[0]->ApplyImpulseLinear(Vec2(+400.0, 0.0));
            break;
        case SDL_MOUSEBUTTONDOWN:
            if (event.button.button == SDL_BUTTON_LEFT) {
                int x, y;
                SDL_GetMouseState(&x, &y);
                // Create and add a new BoxShape at the mouse location
                auto circle = std::make_unique<Body2D>(new CircleShape(50), x, y, 2.0);
                circle->restitution = 0;
                circle->friction = 0.4;
                world->AddBody2D(std::move(circle));
            }
            else if (event.button.button == SDL_BUTTON_RIGHT) {
                int x, y;
                SDL_GetMouseState(&x, &y);
                // Create and add a new CircleShape at the mouse location
                auto circle = std::make_unique<Body2D>(new BoxShape(50,50), x, y, 1.0); // Assuming radius 25 for the circle
                circle->restitution = 0;
                circle->friction = 0.4;
                world->AddBody2D(std::move(circle));
            }
            break;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// Update function (called several times per second to update objects)
///////////////////////////////////////////////////////////////////////////////
void OptimizationScene::Update() {
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
void OptimizationScene::Render() {
    Graphics::ClearScreen(0xFF000000);
    for (auto& body : world->GetBodies()) {
        if (body->shape->GetType() == Circle) {
            auto circleShape = dynamic_cast<CircleShape*>(body->shape);
            Graphics::DrawCircle(body->position.x, body->position.y, circleShape->radius, body->rotation, 0xFF00FF00);
        }
        else  if (body->shape->GetType() == Box) {
            auto boxShape = dynamic_cast<BoxShape*>(body->shape);
            Graphics::DrawPolygon(body->position.x, body->position.y, boxShape->worldVertices, 0xFF00FF00);
        }
        else  if (body->shape->GetType() == Polygon) {
            auto boxShape = dynamic_cast<PolygonShape*>(body->shape);
            Graphics::DrawPolygon(body->position.x, body->position.y, boxShape->worldVertices, 0xFF00FF00);
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
void OptimizationScene::Destroy() {
    Graphics::CloseWindow();
}