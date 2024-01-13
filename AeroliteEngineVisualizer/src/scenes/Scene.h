#ifndef APPLICATION_H
#define APPLICATION_H

#include "Particle.h"
#include "Body2D.h"
#include "pfgen.h"
#include "Contact2D.h"
#include "AeroWorld2D.h"
#include <vector>
#include <memory>
#include "../Graphics.h"

class Scene {
protected:
    bool running = false;
    int timePreviousFrame = 0;
public:
    bool IsRunning() { return running; }
    virtual void Setup() = 0;
    virtual void Input() = 0;
    virtual void Update() = 0;
    virtual void Render() = 0;
    virtual void Destroy() = 0;
protected:
    Scene() = default;
    virtual ~Scene() = default;
};

class GravityDragScene : public Scene
{
private:
    std::unique_ptr<Aerolite::AeroWorld2D> world;
    SDL_Rect liquid;
    SDL_Rect floor;
    Vec2 pushForce;
    
public:
    void Setup() override;
    void Input() override;
    void Update() override;
    void Render() override;
    void Destroy()override;
};

class BilliardScene : public Scene
{
private:
    std::unique_ptr<Aerolite::AeroWorld2D> world;
    Vec2 pushForce;
    Vec2 mouseCursor;
    bool leftMouseButtonDown;

public:
    void Setup() override;
    void Input() override;
    void Update() override;
    void Render() override;
    void Destroy()override;

};

class SolarSystemScene : public Scene
{
private:
    std::unique_ptr<Aerolite::AeroWorld2D> world;
    std::vector<float> planetRadii;
    Vec2 mouseCursor;
    bool leftMouseButtonDown;
public:
    void GenerateSolarSystem(std::vector<std::unique_ptr<Particle>>& planets,
        std::shared_ptr<Particle>& sun,
        int numPlanets,
        float gravitationalConstant,
        float minOrbitRadius,
        float maxOrbitRadius,
        float sunMass);


    void Setup() override;
    void Input() override;
    void Update() override;
    void Render() override;
    void Destroy()override;
};

class SpringScene : public Scene
{
private:
    std::unique_ptr<Aerolite::AeroWorld2D> world;
    Vec2 pushForce;
    Vec2 mouseCursor;
    bool leftMouseButtonDown;

    Vec2 anchor = Vec2(0, 0);
    float k = 150;
    float restLength = 50;
    int NUM_PARTICLES = 5;
    float PARTICLE_MASS = 1.0;

public:
    void Setup() override;
    void Input() override;
    void Update() override;
    void Render() override;
    void Destroy()override;

};

class Collision2DScene : public Scene
{
private:
    std::vector<std::unique_ptr<Body2D>> bodies;
    Vec2 pushForce;
    Vec2 mouseCursor;
    bool leftMouseButtonDown;

public:
    void Setup() override;
    void Input() override;
    void Update() override;
    void Render() override;
    void Destroy()override;

};

class CollisionProjectionResolutionScene : public Scene
{
    private:
        std::vector<std::unique_ptr<Body2D>> bodies;
        Vec2 pushForce;
        Vec2 mouseCursor;
        bool leftMouseButtonDown;

    public:
        void Setup() override;
        void Input() override;
        void Update() override;
        void Render() override;
        void Destroy()override;
};

class SATCollisionScene : public Scene
{
private:
    std::vector<std::unique_ptr<Body2D>> bodies;
    std::vector<Aerolite::Contact2D> contactList;
    Vec2 pushForce;
    Vec2 mouseCursor;
    bool leftMouseButtonDown;
    bool debug = false;

public:
    void Setup() override;
    void Input() override;
    void Update() override;
    void Render() override;
    void Destroy()override;
};

class AeroWorld2DScene : public Scene
{
private:
    std::unique_ptr<Aerolite::AeroWorld2D> world;
    bool debug;
public:
    void Setup() override;
    void Input() override;
    void Update() override;
    void Render() override;
    void Destroy()override;
};

#endif