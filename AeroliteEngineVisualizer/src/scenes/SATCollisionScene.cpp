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
void SATCollisionScene::Setup() {
    running = Graphics::OpenWindow();

    auto moveableBox = std::make_unique<Aerolite::Body2D>(new BoxShape(100, 100), 0, 0, 1.0);
    moveableBox->rotation = 0.3;
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

    /*bodies.push_back(std::move(moveableBox));*/
    bodies.push_back(std::move(floor));
    bodies.push_back(std::move(leftWall));
    bodies.push_back(std::move(rightWall));
    bodies.push_back(std::move(bigBox));
    bodies.push_back(std::move(bar1));
    bodies.push_back(std::move(bar2));
}

///////////////////////////////////////////////////////////////////////////////
// Input processing
///////////////////////////////////////////////////////////////////////////////
void SATCollisionScene::Input() {
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
                auto polygon = std::make_unique<Body2D>(PolygonShape::CreateRegularPolygon(5, 50), x, y, 2.0);
                polygon->restitution = 0.1;
                polygon->friction = 0.4;
                bodies.push_back(std::move(polygon));
            }
            else if (event.button.button == SDL_BUTTON_RIGHT) {
                int x, y;
                SDL_GetMouseState(&x, &y);
                // Create and add a new CircleShape at the mouse location
                auto circle = std::make_unique<Body2D>(new CircleShape(25), x, y, 1.0); // Assuming radius 25 for the circle
                circle->restitution = 0.5;
                circle->friction = 0.4;
                bodies.push_back(std::move(circle));
            }
            break;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// Update function (called several times per second to update objects)
///////////////////////////////////////////////////////////////////////////////
void SATCollisionScene::Update() {
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

   contactList.clear();

    for (auto& body : bodies) {
        Vec2 weight = Vec2(0, body->mass * 9.8 * PIXELS_PER_METER);
        body->AddForce(weight);
    }

    for (auto& body : bodies) {
        /*body->Update(deltaTime);*/
        body->isColliding = false; // Temporary until we have collision detection engine setup.
    }

    // Collision Detection
    for (int i = 0; i < bodies.size() - 1; i++)
    {
        for (int j = i + 1; j < bodies.size(); j++)
        {
            Aerolite::Contact2D contact;
            if (CollisionDetection2D::IsColliding(*bodies[i], *bodies[j], contact))
            {
                contactList.emplace_back(contact);

                if (debug) {
                    bodies[i]->isColliding = true;
                    bodies[j]->isColliding = true;
                }
            }
        }
    }

    // Collision Resolution
    for (auto& contact : contactList)
    {
        contact.ResolveCollision();
    }
}

///////////////////////////////////////////////////////////////////////////////
// Render function (called several times per second to draw objects)
///////////////////////////////////////////////////////////////////////////////
void SATCollisionScene::Render() {
    Graphics::ClearScreen(0xFF000000);
    for (auto& body : bodies) {
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
        for (auto& contact : contactList)
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
void SATCollisionScene::Destroy() {
    Graphics::CloseWindow();
}