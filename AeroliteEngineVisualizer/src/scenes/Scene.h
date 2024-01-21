#ifndef APPLICATION_H
#define APPLICATION_H

#include "Particle2D.h"
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
    void GenerateSolarSystem(
        std::vector<std::unique_ptr<Particle2D>>& planets,
        std::shared_ptr<Particle2D>& sun,
        int numPlanets,
        real gravitationalConstant,
        real minOrbitRadius,
        real maxOrbitRadius,
        real sunMass);


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
    float restLength = 20;
    int NUM_PARTICLES = 30;
    float PARTICLE_MASS = make_real<real>(1.0);

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
    std::unique_ptr<AeroWorld2D> world;
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
        std::unique_ptr<AeroWorld2D> world;
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
    std::unique_ptr<AeroWorld2D> world;
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

class JointConstraintScene : public Scene
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

class RagdollJointScene : public Scene
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

class OptimizationScene : public Scene
{
private:
    SDL_Texture* bgTexture;
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