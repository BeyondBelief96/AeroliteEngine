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

    auto bar1 = std::make_unique<Aerolite::Body2D>(new BoxShape(500, 50),
        Graphics::Width() / 2.0 - 600, Graphics::Height() - 500, 0.0);
    bar1->restitution = 0.0;
    bar1->rotation = 0.5;

    auto bar2 = std::make_unique<Aerolite::Body2D>(new BoxShape(500, 50),
        Graphics::Width() / 2.0 + 600, Graphics::Height() - 500, 0.0);
    bar2->restitution = 0.0;
    bar2->rotation = -0.5;

    auto floor = std::make_unique<Aerolite::Body2D>(new BoxShape(Graphics::Width() - 50, 50),
        Graphics::Width() / 2.0, Graphics::Height() - 50, 0.0);
    floor->restitution = 0.0;

    auto bigBall = std::make_unique<Aerolite::Body2D>(new BoxShape(200, 200), Graphics::Width() / 2.0,
        Graphics::Height() / 2.0, 0.0);
    bigBall->restitution = 0.5;

    auto ball = std::make_unique<Aerolite::Body2D>(new CircleShape(200), 0, 0, 1.0);

    bodies.push_back(std::move(floor));
    bodies.push_back(std::move(bigBall));
    bodies.push_back(std::move(bar1));
    bodies.push_back(std::move(bar2));
    /*bodies.push_back(std::move(ball))*/;
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
            break;
        case SDL_MOUSEBUTTONDOWN:
            if (event.button.button == SDL_BUTTON_LEFT) {
                int x, y;
                SDL_GetMouseState(&x, &y);
                // Create and add a new BoxShape at the mouse location
                auto box = std::make_unique<Body2D>(new BoxShape(50, 50), x, y, 50);
                bodies.push_back(std::move(box));
            }
            else if (event.button.button == SDL_BUTTON_RIGHT) {
                int x, y;
                SDL_GetMouseState(&x, &y);
                // Create and add a new CircleShape at the mouse location
                auto circle = std::make_unique<Body2D>(new CircleShape(25), x, y, 1.0); // Assuming radius 25 for the circle
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
        body->Update(deltaTime);
        body->isColliding = false; // Temporary until we have collision detection engine setup.
    }

    // Collision Detection
    for (int i = 0; i < bodies.size() - 1; i++)
    {
        for (int j = i + 1; j < bodies.size(); j++)
        {
            Aerolite::Contact2D contact;
            if (CollisionDetection2D::IsColliding(bodies[i].get(), bodies[j].get(), contact))
            {
                contactList.emplace_back(contact);
                bodies[i]->isColliding = true;
                bodies[j]->isColliding = true;
            }
        }
    }

    // Collision Resolution
    for (auto& contact : contactList)
    {
        /*contact.ResolveCollision();*/
    }

    // Check the boundaries of the window applying a hardcoded bounce flip in velocity
    for (int i = 0; i < bodies.size(); i++) {
        auto& body = bodies.at(i);
        if (body->shape->GetType() == Aerolite::ShapeType::Circle) {
            auto circleShape = dynamic_cast<CircleShape*>(body->shape);
            if (body->position.x - circleShape->radius <= 0) {
                body->position.x = circleShape->radius;
                body->velocity.x *= -0.9;
            }
            else if (body->position.x + circleShape->radius >= Graphics::Width()) {
                body->position.x = Graphics::Width() - circleShape->radius;
                body->velocity.x *= -0.9;
            }
            if (body->position.y - circleShape->radius <= 0) {
                body->position.y = circleShape->radius;
                body->velocity.y *= -0.9;
            }
            else if (body->position.y + circleShape->radius >= Graphics::Height()) {
                body->position.y = Graphics::Height() - circleShape->radius;
                body->velocity.y *= -0.9;
            }
        }
        auto aabb = body->GetAABB();
        if (aabb.min.y >= Graphics::Height() || aabb.max.x <= 0 || aabb.min.x >= Graphics::Width())
        {
            body->~Body2D();
            bodies.erase(bodies.begin() + i);
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// Render function (called several times per second to draw objects)
///////////////////////////////////////////////////////////////////////////////
void SATCollisionScene::Render() {
    Graphics::ClearScreen(0xFF000000);
    for (auto& body : bodies) {

        uint32_t color = body->isColliding ? 0xFF0000FF : 0xFFFFFFFF;
        if (body->shape->GetType() == Circle) {
            auto circleShape = dynamic_cast<CircleShape*>(body->shape);
            Graphics::DrawCircle(body->position.x, body->position.y, circleShape->radius, body->rotation, color);
        }
        else  if (body->shape->GetType() == Box) {
            auto boxShape = dynamic_cast<BoxShape*>(body->shape);
            Graphics::DrawPolygon(body->position.x, body->position.y, boxShape->worldVertices, color);
        }
    }

    for (auto& contact : contactList)
    {
        if (contact.contactCount == 1)
            Graphics::DrawFillCircle(contact.contact1.x, contact.contact1.y, 4, 0xFFFF00FF);

        else if (contact.contactCount > 1)
        {
            Graphics::DrawFillCircle(contact.contact1.x, contact.contact1.y, 4, 0xFFFF00FF);
            Graphics::DrawFillCircle(contact.contact2.x, contact.contact2.y, 4, 0xFFFF00FF);
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