#include <random>
#include "Collision2D.h"
#include "Constants.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"
#include "pfgen.h"
#include "Scene.h"

#pragma warning(disable : 4244)

namespace Aerolite {
    ///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
    void TheGreatPyramidScene::Setup() {
        running = true;
        world = std::make_unique<AeroWorld2D>(-9.8f);
        world->ShgSetBounds({ 0, 0 }, { static_cast<real>(Graphics::Width()), static_cast<real>(Graphics::Height()) });
        world->ShgSetCellWidth(100);
        world->ShgSetCellHeight(100);
        world->SetBroadPhaseAlgorithm(BroadPhaseAlg::SHG);

        // Add a floor
        const auto floor = world->CreateBody2D(std::make_shared<BoxShape>(Graphics::Width(), 500),
            Graphics::Width() / 2.0f, Graphics::Height() + 230, 0.0f); // Floor positioned at the bottom
        floor->restitution = 0.1f;
        floor->friction = 0.5f;

        auto leftWall = world->CreateBody2D(std::make_shared<BoxShape>(50, Graphics::Height()), 0, Graphics::Height() / make_real<real>(2.0) - 35, make_real<real>(0.0));
        auto rightWall = world->CreateBody2D(std::make_shared<BoxShape>(50, Graphics::Height() * 10), Graphics::Width(),
            Graphics::Height() / make_real<real>(2.0) - 35, make_real<real>(0.0));

        // Flipping the pyramid around
        for (int row = 0; row < pyramidHeight; ++row) {
            for (int col = 0; col <= row; ++col) {
                const float baseY = 40.0f;
                const float x = (Graphics::Width() / 2.0f) + (col - row / 2.0f) * 50.0f; // Center the pyramid
                const float y = baseY + row * 50.0f; // Stack rows upwards
                const auto box = world->CreateBody2D(std::make_shared<BoxShape>(50, 50), x, y, 1.0f);
                box->friction = 0.5f;
                box->restitution = 0.1f;
            }
        }
        for (auto& body : world->GetBodies())
        {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> colorDist(0, 255);
            // Generate random colors for red, green, and blue components
            const int red = colorDist(gen);
            const int green = colorDist(gen);
            const int blue = colorDist(gen);

            // Combine the color components into a single ARGB value with full opacity (0xFF)
            // The '0xFF' is the alpha component for full opacity.
            const auto color = (0xFF << 24) | (red << 16) | (green << 8) | blue;
            m_bodyColors.push_back(color);
        }
    }

    ///////////////////////////////////////////////////////////////////////////////
    // Input processing
    ///////////////////////////////////////////////////////////////////////////////
    void TheGreatPyramidScene::Input(const SDL_Event event) {
        switch (event.type) {
        case SDL_KEYDOWN:
            if (event.key.keysym.sym == SDLK_ESCAPE)
                running = false;
            break;
        }

    }

    ///////////////////////////////////////////////////////////////////////////////
    // Update function (called several times per second to update objects)
    ///////////////////////////////////////////////////////////////////////////////
    void TheGreatPyramidScene::Update() {
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
    void TheGreatPyramidScene::Render() {
        ImGui::Begin("Scene Controls");
        if (ImGui::SliderInt("Pyramid Height", &pyramidHeight, 1, 21)) {
            Destroy(); // Clean up current scene objects
            Setup();   // Re-setup the scene with the std::make_shared< pyramid height
        }
        ImGui::End();

        Graphics::ClearScreen(0xFF000000);
        const auto bodies = world->GetBodies();
        int i = 0;
        for (const auto& body : bodies) {
            if (body->shape->GetType() == Circle) {
                const auto circleShape = std::dynamic_pointer_cast<CircleShape>(body->shape);
                Graphics::DrawFillCircle(body->position.x, body->position.y, circleShape->radius, m_bodyColors[i++]);
            }
            else  if (body->shape->GetType() == Box) {
                const auto boxShape = std::dynamic_pointer_cast<BoxShape>(body->shape);
                boxShape->UpdateVertices(body->rotation, body->position);
                Graphics::DrawFillPolygon(body->position.x, body->position.y, boxShape->worldVertices, m_bodyColors[i++]);
            }
            else if (body->shape->GetType() == Polygon)
            {
                const auto polygonShape = std::dynamic_pointer_cast<PolygonShape>(body->shape);
                polygonShape->UpdateVertices(body->rotation, body->position);
                Graphics::DrawFillPolygon(body->position.x, body->position.y, polygonShape->worldVertices, m_bodyColors[i++]);
            }
        }
    }
}

