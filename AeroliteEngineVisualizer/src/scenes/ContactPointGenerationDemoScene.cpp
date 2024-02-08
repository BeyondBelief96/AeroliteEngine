#include <random>
#include "Collision2D.h"
#include "Contact2D.h"
#include "Constants.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"
#include "pfgen.h"
#include "Scene.h"

#pragma warning(disable : 4244)

void ContactPointGenerationDemoScene::AddRandomBody() {
    // Generate random position and color
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> posDist(0, Graphics::Width()), disy(0, Graphics::Height());
    std::uniform_int_distribution<> colorDist(0, 255);
    std::uniform_int_distribution<> polygonSidesDist(3, 10);
    std::uniform_int_distribution<> polygonSidesLengthDist(10, 200);
    auto position = AeroVec2(posDist(gen), disy(gen));

    // Generate random colors for red, green, and blue components
    const int red = colorDist(gen);
    const int green = colorDist(gen);
    const int blue = colorDist(gen);

    // Combine the color components into a single ARGB value with full opacity (0xFF)
    // The '0xFF' is the alpha component for full opacity.
    const auto color = (0xFF << 24) | (red << 16) | (green << 8) | blue;
    m_bodyColors.push_back(color);
    auto body = std::make_unique<AeroBody2D>(PolygonShape::CreateRegularPolygon(polygonSidesDist(gen), polygonSidesLengthDist(gen)), position.x, position.y, 0.0);
    world->AddBody2D(std::move(body));
}

void ContactPointGenerationDemoScene::RemoveLastBody()  {
    if (!world->GetBodies().empty()) {
        m_bodyColors.pop_back();
        world->RemoveBody2D(world->GetBodies().size() - 1);
    }
}

///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
void ContactPointGenerationDemoScene::Setup() {
    running = true;
    world = std::make_unique<AeroWorld2D>(0);
    m_bodyColors.push_back(0xFFFFFFFF);
    auto moveablePolygon = std::make_unique<AeroBody2D>(PolygonShape::CreateRegularPolygon(4, 50), Graphics::Width() / 2, Graphics::Height() / 2, make_real<real>(0.0));
    world->AddBody2D(std::move(moveablePolygon));
}

///////////////////////////////////////////////////////////////////////////////
// Input processing
///////////////////////////////////////////////////////////////////////////////
void ContactPointGenerationDemoScene::Input(const SDL_Event event) {
     switch (event.type) {
         case SDL_KEYDOWN:
             if (event.key.keysym.sym == SDLK_ESCAPE)
                 running = false;
             break;
         case SDL_MOUSEMOTION:
             int x, y;
             SDL_GetMouseState(&x, &y);
             const std::vector<std::unique_ptr<AeroBody2D>>& bodies = world->GetBodies();
             bodies[0]->position.x = x;
             bodies[0]->position.y = y;
         break;
     }
    
}

///////////////////////////////////////////////////////////////////////////////
// Update function (called several times per second to update objects)
///////////////////////////////////////////////////////////////////////////////
void ContactPointGenerationDemoScene::Update() {
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
void ContactPointGenerationDemoScene::Render() {
    ImGui::Begin("Contact Point Generation");
    // Slider UI
    const int previousSliderValue = sliderValue;
    ImGui::SliderInt("Body Count", &sliderValue, 1, 100);

    // Logic to add or remove bodies
    if(sliderValue > previousSliderValue)
    {
	    for(int i = previousSliderValue; i < sliderValue; ++i)
	    {
            AddRandomBody();
	    }
    }
    else if(sliderValue < previousSliderValue)
    {
        for(int i = previousSliderValue; i > sliderValue; --i)
        {
            RemoveLastBody();
        }
    }

    ImGui::End();

    Graphics::ClearScreen(0xFF000000);
	const std::vector<std::unique_ptr<AeroBody2D>>& bodies = world->GetBodies();
    int i = 0;
    for(const auto& body : bodies) {
        if(body->shape->GetType() == Circle) {
	        const auto circleShape = dynamic_cast<CircleShape*>(body->shape);
            Graphics::DrawCircle(body->position.x, body->position.y, circleShape->radius, body->rotation, 0xFFFFFFFF);
        }
        else  if(body->shape->GetType() == Box) {
	        const auto boxShape = dynamic_cast<BoxShape*>(body->shape);
            boxShape->UpdateVertices(body->rotation, body->position);
            Graphics::DrawPolygon(body->position.x, body->position.y, boxShape->worldVertices, m_bodyColors[i++]);
        }
        else if(body->shape->GetType() == Polygon)
        {
            const auto polygonShape = dynamic_cast<PolygonShape*>(body->shape);
            polygonShape->UpdateVertices(body->rotation, body->position);
            Graphics::DrawPolygon(body->position.x, body->position.y, polygonShape->worldVertices, m_bodyColors[i++]);
        }

        auto contacts = world->GetContacts();

        for (const auto& contact : contacts)
        {
            Graphics::DrawCircle(contact.start.x, contact.start.y, 5, make_real<real>(0.0), 0xFF00FFFF);
            Graphics::DrawCircle(contact.end.x, contact.end.y, 2, make_real<real>(0.0), 0xFF00FFFF);
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// Destroy function to delete objects and close the window
///////////////////////////////////////////////////////////////////////////////
void ContactPointGenerationDemoScene::Destroy() {
    sliderValue = 0;
    running = false;
}