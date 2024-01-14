#include "Constants.h"
#include <random>
#include <stdio.h>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include "pfgen.h"
#include "Scene.h"

// Generates a solar system layout with planets in orbit around a central sun.
void SolarSystemScene::GenerateSolarSystem(std::vector<std::unique_ptr<Particle2D>>& planets,
    std::shared_ptr<Particle2D>& sun,
    int numPlanets,
    real gravitationalConstant,
    real minOrbitRadius,
    real maxOrbitRadius,
    real sunMass) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> distAngle(0, 2 * M_PI);
    std::uniform_real_distribution<float> distRadius(minOrbitRadius, maxOrbitRadius);

    for (int i = 0; i < numPlanets; ++i) {
        // Randomly generate orbital radius and angle.
        real radius = distRadius(gen);
        real angle = distAngle(gen);

        // Calculate the planet's position.
        real x = radius * cos(angle);
        real y = radius * sin(angle);

        planetRadii.push_back(radius);
        // Calculate velocity for a circular orbit. 
        // The velocity direction should be perpendicular to the radial vector (x, y).
        real velocityMagnitude = sqrtf(gravitationalConstant * sunMass / radius);
        // Velocity direction is perpendicular to radial direction
        Vec2 velocity(-y, x); // Rotate position vector by 90 degrees to get tangent direction

        // Create the planet particle.
        auto planet = std::make_unique<Particle2D>(sun->position.x + x, sun->position.y + y, MASS_OF_EARTH); // Assuming a unit mass for simplicity.
        planet->velocity = velocity.UnitVector() * velocityMagnitude;
        planet->radius = 10;

        planets.push_back(std::move(planet));
    }
}

// GLOBAL VARIABLES
auto sun = std::make_shared<Particle2D>(0, 0, MASS_OF_SUN);
auto planetColors = std::vector<unsigned int>();

///////////////////////////////////////////////////////////////////////////////
// Setup function (executed once in the beginning of the simulation)
///////////////////////////////////////////////////////////////////////////////
void SolarSystemScene::Setup() {
    running = Graphics::OpenWindow();
    world = std::make_unique<Aerolite::AeroWorld2D>(0);
    sun->radius = 30;
    sun->position.x = Graphics::Width() / 2;
    sun->position.y = Graphics::Height() / 2;
    auto planets = std::vector<std::unique_ptr<Particle2D>>();
    GenerateSolarSystem(planets, sun, 10000,
        GRAV_CONSTANT, 100, 700, sun->mass);

    for (auto& planet : planets) {
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

    world->AddParticle2Ds(std::move(planets));
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
                auto planets = world->GetParticle2Ds();
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

    auto planets = world->GetParticle2Ds();
    for (int i = 0; i < planets.size(); i++)
    {
        Vec2 gravitationalForce = Particle2DForceGenerators::GenerateGravitationalAttractionForce(*planets[i],
            *sun, 5, 1000, GRAV_CONSTANT);
        planets[i]->ApplyForce(gravitationalForce);
        sun->ApplyForce(-gravitationalForce);
    }

    world->Update(deltaTime);

    // Check boundaries and keep particle inside window.
    for (auto& particle : planets)
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
    auto planets = world->GetParticle2Ds();
    if (leftMouseButtonDown)
        Graphics::DrawLine(planets[0]->position.x, planets[0]->position.y, mouseCursor.x, mouseCursor.y, 0xFFFF1808);

    // Draw sun
    Graphics::DrawFillCircle(sun->position.x, sun->position.y, sun->radius, 0xFF6F84D1);

    int i = 0;
    // Draw planets
    for (auto& planet : planets) {

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