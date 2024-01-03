#ifndef APPLICATION_H
#define APPLICATION_H

#include "Particle.h"
#include "Body2D.h"
#include "pfgen.h"
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
    std::vector<std::shared_ptr<Particle>> particles;
    ParticleForceRegistry pfg;
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
    std::vector<std::shared_ptr<Particle>> particles;
    ParticleForceRegistry pfg;
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
    std::vector<std::shared_ptr<Particle>> planets;
    std::vector<float> planetRadii;
    ParticleForceRegistry pfg;
    Vec2 mouseCursor;
    bool leftMouseButtonDown;
public:
    void GenerateSolarSystem(std::vector<std::shared_ptr<Particle>>& planets,
        std::vector<std::shared_ptr<ParticleGAttraction>>& gravAttractionForces,
        std::shared_ptr<Particle> sun,
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
    std::vector<std::shared_ptr<Particle>> particles;
    ParticleForceRegistry pfg;
    Vec2 pushForce;
    Vec2 mouseCursor;
    bool leftMouseButtonDown;

    Vec2 anchor = Vec2(0, 0);
    float k = 150;
    float restLength = 15;
    int NUM_PARTICLES = 15;
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

#endif