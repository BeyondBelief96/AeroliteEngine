#include <random>
#include <iostream>
#include <filesystem>
#include <SDL2/SDL_image.h>
#include "AeroWorld2D.h"
#include "Collision2D.h"
#include "Contact2D.h"
#include "Constants.h"
#include "Scene.h"

///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
void FiveDollarFlappyBirdScene::Setup() {

    running = true;
    world = std::make_unique<AeroWorld2D>(-9.8f);
    world->ShgSetBounds({ 0, 0 }, { make_real<real>(Graphics::Width()), make_real<real>(Graphics::Height())});
    world->ShgSetCellWidth(100);
    world->ShgSetCellHeight(100);
    world->SetBroadPhaseAlgorithm(BroadPhaseAlg::SHG);

    // Add bird
    auto bird = world->CreateBody2D(std::make_shared<CircleShape>(45), 100, Graphics::Height() / make_real<real>(2.0) + 220, make_real<real>(3.0));

    // Add a floor and walls to contain objects
    const auto floor = world->CreateBody2D(std::make_shared<BoxShape>(Graphics::Width() - 50, 50),
                                           Graphics::Width() / make_real<real>(2.0), Graphics::Height() / make_real<real>(2.0) + 290, make_real<real>(0.0));
    auto leftFence = world->CreateBody2D(std::make_shared<BoxShape>(50, Graphics::Height() - 200), 0, Graphics::Height() / make_real<real>(2.0) - 35, make_real<real>(0.0));
    auto rightFence = world->CreateBody2D(std::make_shared<BoxShape>(50, Graphics::Height() - 200), Graphics::Width(),
        Graphics::Height() / make_real<real>(2.0) - 35, make_real<real>(0.0));
    

    // Add a stack of boxes
    for (int i = 1; i <= 4; i++) {
	    const float mass = 10.0f / static_cast<real>(i);
	    const auto box = world->CreateBody2D(std::make_shared<BoxShape>(50, 50), 600, floor->position.y - i * 55, mass);
        box->SetFriction(0.9f);
        box->SetRestitution(0.1f);
    }

    // Add structure with blocks
    const auto plank1 = world->CreateBody2D(std::make_shared<BoxShape>(50, 150), Graphics::Width() / make_real<real>(2.0) + 20, floor->position.y - 100, make_real<real>(5.0));
    const auto plank2 = world->CreateBody2D(std::make_shared<BoxShape>(50, 150), Graphics::Width() / make_real<real>(2.0) + 180, floor->position.y - 100, make_real<real>(5.0));
    const auto plank3 = world->CreateBody2D(std::make_shared<BoxShape>(250, 25), Graphics::Width() / make_real<real>(2.0) + 100.0f, floor->position.y - 200, make_real<real>(2.0));
   

    // Add a triangle polygon
    const std::vector triangleVertices = { Vec2(30, 30), Vec2(-30, 30), Vec2(0, -30) };
    auto triangle = world->CreateBody2D(std::make_shared<PolygonShape>(triangleVertices), plank3->position.x, plank3->position.y - 50, make_real<real>(0.5));

    // Add a pyramid of boxes
    const int numRows = 5;
    for (int col = 0; col < numRows; col++) {
        for (int row = 0; row < col; row++) {
	        const float x = plank3->position.x + 200.0f + col * 50.0f - row * 25.0f;
	        const float y = floor->position.y - 50.0f - row * 52.0f;
	        const float mass = 5.0f / (row + 1.0f);
	        const auto box = world->CreateBody2D(std::make_shared<BoxShape>(50, 50), x, y, mass);
            box->SetFriction(0.9f);
            box->SetRestitution(0.0f);
        }
    }

    // Add a bridge of connected steps and joints
    const int numSteps = 10;

    // Create the initial step (floor)
    const auto startStep = world->CreateBody2D(std::make_shared<BoxShape>(80, 20), 200, 200, make_real<real>(0.0));
    auto lastStep = startStep;  // Initialize lastStep to the starting step

    // Iterate to create the steps in the bridge
    for (int i = 1; i <= numSteps; i++) {

        // Calculate the position of the new step
        const real x = lastStep->position.x + 30 + i;
        const real y = lastStep->position.y + 20;

        // Set the mass for the current step (0.0 for the last step, 3.0 for others)
        const real mass = (i == numSteps) ? 0.0f : 3.0f;

        // Create the step using a CircleShape
        auto step = world->CreateBody2D(std::make_shared<CircleShape>(15), x, y, mass);

        // Connect the current step to the previous step using a joint constraint
        world->AddJointConstraint(lastStep, step, step->position);

        // Update lastStep to be the current step for the next iteration
        lastStep = step;
    }

    auto endStep = world->CreateBody2D(std::make_shared<BoxShape>(80, 20), lastStep->position.x + 60, lastStep->position.y - 20, make_real<real>(0.0));

    auto pig1 = world->CreateBody2D(std::make_shared<CircleShape>(30), plank1->position.x + 80, floor->position.y - 50, make_real<real>(3.0));
    auto pig2 = world->CreateBody2D(std::make_shared<CircleShape>(30), plank2->position.x + 400, floor->position.y - 50, make_real<real>(3.0));
    auto pig3 = world->CreateBody2D(std::make_shared<CircleShape>(30), plank2->position.x + 460, floor->position.y - 50, make_real<real>(3.0));
    auto pig4 = world->CreateBody2D(std::make_shared<CircleShape>(30), 220, 130, make_real<real>(1.0));

    for(auto& body : world->GetBodies())
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
        const auto color = 0xFF << 24 | red << 16 | green << 8 | blue;
        m_bodyColors.push_back(color);
    }
}

///////////////////////////////////////////////////////////////////////////////
// Input processing
///////////////////////////////////////////////////////////////////////////////
void FiveDollarFlappyBirdScene::Input(const SDL_Event event) {
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
             world->GetBodies()[0]->ApplyImpulseLinear(Vec2(make_real<real>(0.0), -make_real<real>(600.0)));
         if (event.key.keysym.sym == SDLK_LEFT)
             world->GetBodies()[0]->ApplyImpulseLinear(Vec2(-make_real<real>(400.0), make_real<real>(0.0)));
         if (event.key.keysym.sym == SDLK_RIGHT)
             world->GetBodies()[0]->ApplyImpulseLinear(Vec2(+make_real<real>(400.0), make_real<real>(0.0)));
         break;
     case SDL_MOUSEBUTTONDOWN:
         if (event.button.button == SDL_BUTTON_LEFT) {
            int x, y;
            SDL_GetMouseState(&x, &y);
            // Create and add a std::make_shared< BoxShape at the mouse location
            const auto circle = world->CreateBody2D(std::make_shared<CircleShape>(50), x, y, make_real<real>(2.0));
            circle->restitution = 0;
            circle->friction = make_real<real>(0.4);
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> colorDist(0, 255);
            // Generate random colors for red, green, and blue components
            const int red = colorDist(gen);
            const int green = colorDist(gen);
            const int blue = colorDist(gen);

            // Combine the color components into a single ARGB value with full opacity (0xFF)
            // The '0xFF' is the alpha component for full opacity.
            const auto color = 0xFF << 24 | red << 16 | green << 8 | blue;
            m_bodyColors.push_back(color);
         }
         else if (event.button.button == SDL_BUTTON_RIGHT) {
            int x, y;
            SDL_GetMouseState(&x, &y);
            // Create and add a std::make_shared< CircleShape at the mouse location
            const auto circle = world->CreateBody2D(std::make_shared<BoxShape>(50,50), x, y, make_real<real>(1.0)); // Assuming radius 25 for the circle
            circle->restitution = 0;
            circle->friction = make_real<real>(0.4);
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> colorDist(0, 255);
            // Generate random colors for red, green, and blue components
            const int red = colorDist(gen);
            const int green = colorDist(gen);
            const int blue = colorDist(gen);

            // Combine the color components into a single ARGB value with full opacity (0xFF)
            // The '0xFF' is the alpha component for full opacity.
            const auto color = 0xFF << 24 | red << 16 | green << 8 | blue;
            m_bodyColors.push_back(color);
         }
         break;
     }
}

///////////////////////////////////////////////////////////////////////////////
// Update function (called several times per second to update objects)
///////////////////////////////////////////////////////////////////////////////
void FiveDollarFlappyBirdScene::Update() {
    // Check if we are too fast, and if so, waste some milliseconds until we reach
    // MILLISECONDS_PER_FRAME.
    const int time_to_wait = MILLISECS_PER_FRAME - (SDL_GetTicks() - timePreviousFrame);
    if (time_to_wait > 0) {
        SDL_Delay(time_to_wait);
    }

    // Calculate the delta time factor in seconds to be used when we update objects
    real delta_time = (SDL_GetTicks() - timePreviousFrame) / 1000.0f;
    if (delta_time > 0.016f) {
        delta_time = 0.016f;
    }

    // Set the time of the current frame to be used in the next one.
    timePreviousFrame = SDL_GetTicks();

    world->Update(delta_time);
}

///////////////////////////////////////////////////////////////////////////////
// Render function (called several times per second to draw objects)
///////////////////////////////////////////////////////////////////////////////
void FiveDollarFlappyBirdScene::Render() {
    Graphics::ClearScreen(0xFF000000);
    int i = 0;
    for (const auto& body : world->GetBodies()) {
        if (body->shape->GetType() == Circle) {
	        const auto circleShape =  std::dynamic_pointer_cast<CircleShape>(body->shape);
            Graphics::DrawCircle(body->position.x, body->position.y, circleShape->radius, body->rotation, m_bodyColors[i++]);
        }
        else  if (body->shape->GetType() == Box) {
	        const auto boxShape =  std::dynamic_pointer_cast<BoxShape>(body->shape);
            Graphics::DrawFillPolygon(body->position.x, body->position.y, boxShape->worldVertices, m_bodyColors[i++]);
        }
        else  if (body->shape->GetType() == Polygon) {
	        const auto boxShape =  std::dynamic_pointer_cast<PolygonShape>(body->shape);
            Graphics::DrawFillPolygon(body->position.x, body->position.y, boxShape->worldVertices, m_bodyColors[i++]);
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
}