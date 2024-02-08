#include <random>
#include "Collision2D.h"
#include "Contact2D.h"
#include "Constants.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"
#include "pfgen.h"
#include "Scene.h"

#pragma warning(disable : 4244)

///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
void ProjectileExplosionDemoScene::Setup() {
    running = true;
    world = std::make_unique<AeroWorld2D>(-3); // Assuming gravity is not needed or will be set elsewhere

    // Static floor setup
    auto floor = std::make_unique<AeroBody2D>(new BoxShape(Graphics::Width(), 50), Graphics::Width() / 2, Graphics::Height() - 25, 0.0f);
    world->AddBody2D(std::move(floor));

    // Structure setup: A tower of boxes
    const int towerHeight = 10;
    const int towerBaseWidth = 5;
    const float boxSize = 50.0f;
    for (int i = 0; i < towerHeight; ++i) {
        for (int j = 0; j < towerBaseWidth - i; ++j) {
            auto box = std::make_unique<AeroBody2D>(new BoxShape(boxSize, boxSize),
                Graphics::Width() / 2 - ((towerBaseWidth - 1) * boxSize) / 2 + j * boxSize + i * boxSize / 2,
                Graphics::Height() - 75 - i * boxSize, 1.0f);
            world->AddBody2D(std::move(box));
        }
    }

    // Anchor point setup (static point for the top of the pendulum arm)
    Vec2 anchorPoint(Graphics::Width() / 2 - 500, 200);

    // Pendulum arm setup (intermediary anchor body)
    const auto armLength = 600;
    auto pendulumArm = std::make_unique<AeroBody2D>(new BoxShape(10, armLength),
        anchorPoint.x, anchorPoint.y + armLength / 2.0, 1.0); // Making the arm dynamic with mass

    // Create a static anchor body at the anchor point
    auto ceilingAnchor = std::make_unique<AeroBody2D>(new BoxShape(50, 50),
        anchorPoint.x, anchorPoint.y, 0.0f); // This body acts as the ceiling anchor

    // Pendulum ball setup
    const float pendulumBallRadius = 40;
    auto pendulumBall = std::make_unique<AeroBody2D>(new CircleShape(pendulumBallRadius),
        anchorPoint.x, anchorPoint.y + armLength + pendulumBallRadius * 2, 100.0f); // Positioned at the bottom of the arm

    // Adjust joint positions for slight spacing
    AeroVec2 ceilingJointPoint = { anchorPoint.x, anchorPoint.y + 10 }; // Slightly below the ceiling anchor for a gap
    auto ceilingJoint = std::make_unique<JointConstraint>(*ceilingAnchor, *pendulumArm, ceilingJointPoint);

    Vec2 ballJointPoint = Vec2(anchorPoint.x, anchorPoint.y + armLength + 5); // Slightly above the ball for a gap
    auto ballJoint = std::make_unique<JointConstraint>(*pendulumArm, *pendulumBall, ballJointPoint);

    // Apply a zero impulse to the pendulum ball to initiate motion (if needed)
    pendulumBall->ApplyImpulseLinear({ 100000, 0 });

    // Add bodies and constraints to the world
    world->AddBody2D(std::move(ceilingAnchor));
    world->AddBody2D(std::move(pendulumBall));
    world->AddBody2D(std::move(pendulumArm));
    world->AddConstraint(std::move(ceilingJoint));
    world->AddConstraint(std::move(ballJoint));

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
void ProjectileExplosionDemoScene::Input(const SDL_Event event) {
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
void ProjectileExplosionDemoScene::Update() {
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
void ProjectileExplosionDemoScene::Render() {
    Graphics::ClearScreen(0xFF000000);
    const std::vector<std::unique_ptr<AeroBody2D>>& bodies = world->GetBodies();
    int i = 0;
    for (const auto& body : bodies) {
        if (body->shape->GetType() == Circle) {
            const auto circleShape = dynamic_cast<CircleShape*>(body->shape);
            Graphics::DrawFillCircle(body->position.x, body->position.y, circleShape->radius, m_bodyColors[i++]);
        }
        else  if (body->shape->GetType() == Box) {
            const auto boxShape = dynamic_cast<BoxShape*>(body->shape);
            boxShape->UpdateVertices(body->rotation, body->position);
            Graphics::DrawFillPolygon(body->position.x, body->position.y, boxShape->worldVertices, m_bodyColors[i++]);
        }
        else if (body->shape->GetType() == Polygon)
        {
            const auto polygonShape = dynamic_cast<PolygonShape*>(body->shape);
            polygonShape->UpdateVertices(body->rotation, body->position);
            Graphics::DrawFillPolygon(body->position.x, body->position.y, polygonShape->worldVertices, m_bodyColors[i++]);
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// Destroy function to delete objects and close the window
///////////////////////////////////////////////////////////////////////////////
void ProjectileExplosionDemoScene::Destroy() {
    running = false;
}