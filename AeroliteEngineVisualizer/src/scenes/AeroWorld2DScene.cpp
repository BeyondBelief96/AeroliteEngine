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
void AeroWorld2DScene::Setup() {
    running = Graphics::OpenWindow();

    world = std::make_unique<Aerolite::AeroWorld2D>(-9.8);

    auto bar1 = std::make_unique<Aerolite::Body2D>(new BoxShape(500, 50),
        Graphics::Width() / 2.0 - 600, Graphics::Height() - 500, 0.0);
    bar1->restitution = 0.3;
    bar1->rotation = 0.5;

    auto bar2 = std::make_unique<Aerolite::Body2D>(new BoxShape(500, 50),
        Graphics::Width() / 2.0 + 600, Graphics::Height() - 500, 0.0);
    bar2->restitution = 0.3;
    bar2->rotation = -0.5;

    auto floor = std::make_unique<Aerolite::Body2D>(new BoxShape(Graphics::Width() - 50, 50),
        Graphics::Width() / 2.0, Graphics::Height() - 50, 0.0);
    floor->restitution = 0.3;

    auto leftWall = std::make_unique<Aerolite::Body2D>(new BoxShape(50, Graphics::Height() - 50),
        0, Graphics::Height() / 2.0, 0.0);
    leftWall->restitution = 0.3;

    auto rightWall = std::make_unique<Aerolite::Body2D>(new BoxShape(50, Graphics::Height() - 50),
        Graphics::Width(), Graphics::Height() / 2.0, 0.0);
    rightWall->restitution = 0.3;

    auto bigBox = std::make_unique<Aerolite::Body2D>(new BoxShape(200, 200), Graphics::Width() / 2.0,
        Graphics::Height() / 2.0, 0.0);
    bigBox->restitution = 0.3;

    world->AddBody2D(std::move(bar1));
    world->AddBody2D(std::move(bar2));
    world->AddBody2D(std::move(floor));
    world->AddBody2D(std::move(leftWall));
    world->AddBody2D(std::move(rightWall));
    world->AddBody2D(std::move(bigBox));
}

///////////////////////////////////////////////////////////////////////////////
// Input processing
///////////////////////////////////////////////////////////////////////////////
void AeroWorld2DScene::Input() {
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
                auto polygon = std::make_unique<Body2D>(new BoxShape(100, 100), x, y, 2.0);
                polygon->restitution = 0.1;
                polygon->friction = 0.4;
                world->AddBody2D(std::move(polygon));
            }
            else if (event.button.button == SDL_BUTTON_RIGHT) {
                int x, y;
                SDL_GetMouseState(&x, &y);
                // Create and add a new CircleShape at the mouse location
                auto circle = std::make_unique<Body2D>(new CircleShape(25), x, y, 1.0); // Assuming radius 25 for the circle
                circle->restitution = 0.5;
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
void AeroWorld2DScene::Update() {
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
void AeroWorld2DScene::Render() {
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
void AeroWorld2DScene::Destroy() {
    Graphics::CloseWindow();
}