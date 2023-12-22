#include <Constants.h>
#include <random>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <pfgen.h>
#include "Scene.h"

// Generates a solar system layout with planets in orbit around a central sun.
void SolarSystemScene::GenerateSolarSystem(std::vector<std::shared_ptr<Particle>>& planets,
    std::vector<std::shared_ptr<ParticleGAttraction>>& gravAttractionForces,
    std::shared_ptr<Particle> sun,
    int numPlanets,
    float gravitationalConstant,
    float minOrbitRadius,
    float maxOrbitRadius,
    float sunMass) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> distAngle(0, 2 * M_PI);
    std::uniform_real_distribution<float> distRadius(minOrbitRadius, maxOrbitRadius);

    for (int i = 0; i < numPlanets; ++i) {
        // Randomly generate orbital radius and angle.
        float radius = distRadius(gen);
        float angle = distAngle(gen);

        // Calculate the planet's position.
        float x = radius * cosf(angle);
        float y = radius * sinf(angle);

        // Calculate velocity for a circular orbit. 
        // The velocity direction should be perpendicular to the radial vector (x, y).
        float velocityMagnitude = sqrtf(gravitationalConstant * sunMass / radius) * 30;
        // Velocity direction is perpendicular to radial direction
        Vec2 velocity(-y, x); // Rotate position vector by 90 degrees to get tangent direction
        velocity = velocity.UnitVector() * velocityMagnitude; // Normalize and scale by magnitude

        // Create the planet particle.
        auto planet = std::make_shared<Particle>(sun->position.x + x, sun->position.y + y, 1.0); // Assuming a unit mass for simplicity.
        planet->velocity = velocity;
        planets.push_back(planet);

        // Create gravitational attraction force generators.
        auto gravAttractionToSun = std::make_shared<ParticleGAttraction>(planet, sun, gravitationalConstant, 5, 100);
        gravAttractionForces.push_back(gravAttractionToSun);
    }
}

// GLOBAL VARIABLES
auto gravAttractionForces = std::vector<std::shared_ptr<ParticleGAttraction>>();
auto sun = std::make_shared<Particle>(0, 0, 50);
auto planetColors = std::vector<unsigned int>();

///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
void SolarSystemScene::Setup() {
    running = Graphics::OpenWindow();
    sun->radius = 30;
    sun->position.x = Graphics::Width() / 2;
    sun->position.y = Graphics::Height() / 2;
    GenerateSolarSystem(planets, gravAttractionForces, sun, 5000,
        100, 50, 300, sun->mass);

    for (auto planet : planets) {
        // Create a random device and generator for color generation
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dist(0, 255);

        // Generate random colors for red, green, and blue components
        int red = dist(gen);
        int green = dist(gen);
        int blue = dist(gen);

        // Combine the color components into a single ARGB value with full opacity (0xFF)
        // The '0xFF' is the alpha component for full opacity.
        unsigned int color = (0xFF << 24) | (red << 16) | (green << 8) | blue;
        planetColors.push_back(color);
    }
}

///////////////////////////////////////////////////////////////////////////////
// Input processing
///////////////////////////////////////////////////////////////////////////////
void SolarSystemScene::Input() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
        case SDL_QUIT:
            running = false;
            break;
        case SDL_KEYDOWN:
            if (event.key.keysym.sym == SDLK_ESCAPE)
                running = false;
            break;
        case SDL_MOUSEMOTION:
            mouseCursor.x = event.motion.x;
            mouseCursor.y = event.motion.y;
            break;
        case SDL_MOUSEBUTTONDOWN:
            if (!leftMouseButtonDown && event.button.button == SDL_BUTTON_LEFT) {
                leftMouseButtonDown = true;
                int x, y;
                SDL_GetMouseState(&x, &y);
                mouseCursor.x = x;
                mouseCursor.y = y;
            }
            break;
        case SDL_MOUSEBUTTONUP:
            if (leftMouseButtonDown && event.button.button == SDL_BUTTON_LEFT) {
                leftMouseButtonDown = false;
                Vec2 impulseDirection = (planets[0]->position - mouseCursor).UnitVector();
                float impulseMagnitude = (planets[0]->position - mouseCursor).Magnitude() * 1;
                planets[0]->velocity = impulseDirection * impulseMagnitude;
            }
            break;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// Update function (called several times per second to update objects)
///////////////////////////////////////////////////////////////////////////////
void SolarSystemScene::Update() {
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

    for (int i = 0; i < planets.size(); i++)
    {
        pfg.Add(planets[i], gravAttractionForces[i]);
    }

    // Update Forces From Particle Force Registry
    pfg.UpdateForces(deltaTime);

    // Preform integration for each particle.
    for (auto particle : planets)
    {
        // Integrate the accleration and velocity to find the new position.
        particle->Integrate(deltaTime);
    }

    // Check boundaries and keep particle inside window.
    for (auto particle : planets)
    {
        // Nasty hardcoded flip in velocity if it touches the limits of the screen
        if (particle->position.x - particle->radius <= 0) {
            particle->position.x = particle->radius;
            particle->velocity.x *= -0.95f;
        }
        else if (particle->position.x + particle->radius >= Graphics::Width()) {
            particle->position.x = Graphics::Width() - particle->radius;
            particle->velocity.x *= -0.95f;
        }
        if (particle->position.y - particle->radius <= 0) {
            particle->position.y = particle->radius;
            particle->velocity.y *= -0.95f;
        }
        else if (particle->position.y + particle->radius >= Graphics::Height()) {
            particle->position.y = Graphics::Height() - particle->radius;
            particle->velocity.y *= -0.95f;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// Render function (called several times per second to draw objects)
///////////////////////////////////////////////////////////////////////////////
void SolarSystemScene::Render() {
    Graphics::ClearScreen(0xFF000000);

    if (leftMouseButtonDown)
        Graphics::DrawLine(planets[0]->position.x, planets[0]->position.y, mouseCursor.x, mouseCursor.y, 0xFFFF1808);

    // Draw sun
    Graphics::DrawFillCircle(sun->position.x, sun->position.y, sun->radius, 0xFF6F84D1);

    int i = 0;
    // Draw planets
    for (auto planet : planets) {

        // Draw the planet with the generated random color
        Graphics::DrawFillCircle(planet->position.x, planet->position.y, planet->radius, planetColors[i++]);
    }
    Graphics::RenderFrame();
}

///////////////////////////////////////////////////////////////////////////////
// Destroy function to delete objects and close the window
///////////////////////////////////////////////////////////////////////////////
void SolarSystemScene::Destroy() {
    Graphics::CloseWindow();
}