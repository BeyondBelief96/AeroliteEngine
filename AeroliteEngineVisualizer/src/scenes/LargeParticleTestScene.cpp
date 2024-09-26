#include <random>
#include "Collision2D.h"
#include "Constants.h"
#include "pfgen.h"
#include "Scene.h"

#pragma warning(disable : 4244)

namespace Aerolite {
    void LargeParticleTestScene::CreateRandomCircles(const int numberOfCircles, const real velocityRange, real radius) const {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> xDist(0.0, Graphics::Width());  // Random x-coordinate within window width
        std::uniform_real_distribution<> yDist(0.0, Graphics::Height()); // Random y-coordinate within window height
        std::uniform_real_distribution<> velDist(-velocityRange, velocityRange);

        for (int i = 0; i < numberOfCircles; ++i) {
            // Random position within the window
            const AeroVec2 position(xDist(gen), yDist(gen));

            // Random linear_velocity within the given range
            const AeroVec2 linearVelocity(velDist(gen), velDist(gen));

            // Create the body with the shape and add it to the world
            const auto body = world->CreateBody2D(std::make_shared<CircleShape>(radius), position.x, position.y, /* Mass */ 1.0);
            body->linear_velocity = linearVelocity;
        }
    }

    ///////////////////////////////////////////////////////////////////////////////
    // Setup function (executed once in the beginning of the simulation)
    ///////////////////////////////////////////////////////////////////////////////
    void LargeParticleTestScene::Setup() {
        running = true;
        world = std::make_unique<AeroWorld2D>(0);
        world->SetBroadPhaseAlgorithm(BroadPhaseAlg::SHG);
        world->ShgSetBounds({ 0, 0 }, { static_cast<real>(Graphics::Width()), static_cast<real>(Graphics::Height()) });
        world->ShgSetCellWidth(10);
        world->ShgSetCellHeight(10);
        CreateRandomCircles(3000, 1500, 2);
    }

    ///////////////////////////////////////////////////////////////////////////////
    // Input processing
    ///////////////////////////////////////////////////////////////////////////////
    void LargeParticleTestScene::Input(const SDL_Event event) {
        switch (event.type) {
        case SDL_QUIT:
            running = false;
            break;
        case SDL_KEYDOWN:
            if (event.key.keysym.sym == SDLK_ESCAPE)
                running = false;
            break;
        }

    }

    ///////////////////////////////////////////////////////////////////////////////
    // Update function (called several times per second to update objects)
    ///////////////////////////////////////////////////////////////////////////////
    void LargeParticleTestScene::Update() {
        // Check if we are too fast, and if so, waste some milliseconds until we reach
        // MILLISECONDS_PER_FRAME.
        const int timeToWait = MILLISECS_PER_FRAME - (SDL_GetTicks() - timePreviousFrame);
        if (timeToWait > 0) {
            SDL_Delay(timeToWait);
        }

        // Calculate the delta time factor in seconds to be used when we update objects
        real deltaTime = (SDL_GetTicks() - timePreviousFrame) / 1000.0f;
        if (deltaTime > 0.016) {
            deltaTime = 0.016;
        }

        // Set the time of the current frame to be used in the next one.
        timePreviousFrame = SDL_GetTicks();

        world->Update(deltaTime);

        for (const auto& body : world->GetBodies())
        {
            if (body->shape->GetType() == Circle)
            {
                const auto circleShape = std::dynamic_pointer_cast<CircleShape>(body->shape);
                const real radius = circleShape->radius;

                // Check collision with left and right boundaries
                if (body->position.x - radius < 0 || body->position.x + radius > Graphics::Width()) {
                    body->linear_velocity.x = -body->linear_velocity.x;
                }

                // Check collision with top and bottom boundaries
                if (body->position.y - radius < 0 || body->position.y + radius > Graphics::Height()) {
                    body->linear_velocity.y = -body->linear_velocity.y;
                }
            }
        }
    }

    ///////////////////////////////////////////////////////////////////////////////
    // Render function (called several times per second to draw objects)
    ///////////////////////////////////////////////////////////////////////////////
    void LargeParticleTestScene::Render() {
        Graphics::ClearScreen(0xFF000000);
        const auto bodies = world->GetBodies();
        for (const auto& body : bodies) {
            if (body->shape->GetType() == Circle) {
                const auto circleShape = std::dynamic_pointer_cast<CircleShape>(body->shape);
                Graphics::DrawCircle(body->position.x, body->position.y, circleShape->radius, body->rotation, 0xFFFFFFFF);
            }
            else  if (body->shape->GetType() == Box) {
                const auto boxShape = std::dynamic_pointer_cast<BoxShape>(body->shape);
                Graphics::DrawPolygon(body->position.x, body->position.y, boxShape->worldVertices, 0xFFFFFFFF);
            }
        }

        Graphics::RenderFrame();
    }
}

