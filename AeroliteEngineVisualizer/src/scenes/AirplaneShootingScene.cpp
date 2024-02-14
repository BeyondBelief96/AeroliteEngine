#include "Collision2D.h"
#include "Contact2D.h"
#include "Constants.h"
#include "imgui_impl_sdl2.h"
#include "Scene.h"

///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
void AirplaneShootingScene::Setup() {
    running = true;
    world = std::make_unique<AeroWorld2D>(0);
    world->ShgSetBounds({ 0, 0 }, { static_cast<real>(Graphics::Width()), static_cast<real>(Graphics::Height()) });
    world->ShgSetCellWidth(200);
    world->ShgSetCellHeight(200);
}

///////////////////////////////////////////////////////////////////////////////
// Input processing
///////////////////////////////////////////////////////////////////////////////
void AirplaneShootingScene::Input(const SDL_Event event) {
    switch (event.type) {
    case SDL_KEYDOWN:
        if (event.key.keysym.sym == SDLK_ESCAPE)
            running = false;
        break;
    case SDL_MOUSEMOTION:
        int x, y;
        SDL_GetMouseState(&x, &y);
        const auto bodies = world->GetBodies();
        bodies[0]->position.x = x;
        bodies[0]->position.y = y;
        break;
    }

}

///////////////////////////////////////////////////////////////////////////////
// Update function (called several times per second to update objects)
///////////////////////////////////////////////////////////////////////////////
void AirplaneShootingScene::Update() {
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
void AirplaneShootingScene::Render() {
    Graphics::ClearScreen(0xFF000000);
    const auto bodies = world->GetBodies();
    int i = 0;
    for (const auto& body : bodies) {
        if (body->shape->GetType() == Circle) {
            const auto circleShape = std::dynamic_pointer_cast<CircleShape>(body->shape);
            Graphics::DrawCircle(body->position.x, body->position.y, circleShape->radius, body->rotation, 0xFFFFFFFF);
        }
        else  if (body->shape->GetType() == Box) {
            const auto boxShape = std::dynamic_pointer_cast<BoxShape>(body->shape);
            boxShape->UpdateVertices(body->rotation, body->position);
            Graphics::DrawPolygon(body->position.x, body->position.y, boxShape->worldVertices, m_bodyColors[i++]);
        }
        else if (body->shape->GetType() == Polygon)
        {
            const auto polygonShape = std::dynamic_pointer_cast<PolygonShape>(body->shape);
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