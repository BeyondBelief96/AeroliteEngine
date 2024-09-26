#include <random>
#include "AeroWorld2D.h"
#include "Collision2D.h"
#include "Contact2D.h"
#include "Constants.h"
#include "Scene.h"

namespace Aerolite {
    ///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
    void JointConstraintScene::Setup() {
        running = Graphics::OpenWindow();

        world = std::make_unique<AeroWorld2D>(-9.8);

        const int numBodies = 10;
        for (int i = 0; i < numBodies; i++) {
            const real mass = (i == 0) ? 0.0f : 1.0f;
            auto body = world->CreateBody2D(std::make_shared<BoxShape>(30, 30), Graphics::Width() / make_real<real>(2.0) - (i * 40), 100, mass);
        }

        for (int i = 0; i < numBodies - 1; i++) {
            std::vector<std::shared_ptr<AeroBody2D>>& bodies = world->GetBodies();
            const auto a = bodies[i];
            const auto b = bodies[i + 1];
            world->AddJointConstraint(a, b, a->position);
        }

    }

    ///////////////////////////////////////////////////////////////////////////////
    // Input processing
    ///////////////////////////////////////////////////////////////////////////////
    void JointConstraintScene::Input(const SDL_Event event) {
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
        }

    }

    ///////////////////////////////////////////////////////////////////////////////
    // Update function (called several times per second to update objects)
    ///////////////////////////////////////////////////////////////////////////////
    void JointConstraintScene::Update() {
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
    void JointConstraintScene::Render() {
        Graphics::ClearScreen(0xFF000000);
        for (const auto& body : world->GetBodies()) {
            if (body->shape->GetType() == Circle) {
                const auto circleShape = std::dynamic_pointer_cast<CircleShape>(body->shape);
                Graphics::DrawCircle(body->position.x, body->position.y, circleShape->radius, body->rotation, 0xFFFFFFFF);
            }
            else  if (body->shape->GetType() == Box) {
                const auto boxShape = std::dynamic_pointer_cast<BoxShape>(body->shape);
                Graphics::DrawFillPolygon(body->position.x, body->position.y, boxShape->worldVertices, 0xFFFFFFFF);
            }
            else  if (body->shape->GetType() == Polygon) {
                const auto boxShape = std::dynamic_pointer_cast<PolygonShape>(body->shape);
                Graphics::DrawFillPolygon(body->position.x, body->position.y, boxShape->worldVertices, 0xFFFF0000);
            }
        }

        if (debug) {
            for (const auto& contact : world->GetContacts())
            {
                Graphics::DrawFillCircle(contact.start.x, contact.start.y, 4, 0xFFFF00FF);
                Graphics::DrawFillCircle(contact.end.x, contact.end.y, 4, 0xFFFF00FF);
                Graphics::DrawLine(contact.start.x, contact.start.y, contact.start.x + contact.normal.x * 15,
                    contact.start.y + contact.normal.y * 15, 0xFFFF00FF);
            }
        }

        Graphics::RenderFrame();
    }
}
