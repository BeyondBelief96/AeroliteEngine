#ifndef APPLICATION_H
#define APPLICATION_H

#include <vector>
#include <memory>

#include "Particle2D.h"
#include "AeroBody2D.h"
#include "AeroWorld2D.h"

#include "Graphics.h"

class Scene {
protected:
    bool running = false;
    int timePreviousFrame = 0;
public:
    bool IsRunning() const { return running; }
    virtual void Setup() = 0;
    virtual void Input(SDL_Event event) = 0;
    virtual void Update() = 0;
    virtual void Render() = 0;
    virtual void Destroy() = 0;
    Scene() = default;
    ~Scene() = default;
};

class GravityDragScene : public Scene
{
private:
    std::unique_ptr<Aerolite::AeroWorld2D> world;
    SDL_Rect liquid;
    SDL_Rect floor;
    Vec2 pushForce;
    
public:
    virtual void Setup() override;
    virtual void Input(SDL_Event event) override;
    virtual void Update() override;
    virtual void Render() override;
    virtual void Destroy()override;
};

class BilliardScene : public Scene
{
private:
    std::unique_ptr<Aerolite::AeroWorld2D> world;
    Vec2 pushForce;
    Vec2 mouseCursor;
    bool leftMouseButtonDown;

public:
    virtual void Setup() override;
    virtual void Input(SDL_Event event) override;
    virtual void Update() override;
    virtual void Render() override;
    virtual void Destroy()override;

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


    virtual void Setup() override;
    virtual void Input(SDL_Event event) override;
    virtual void Update() override;
    virtual void Render() override;
    virtual void Destroy()override;
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
    virtual void Setup() override;
    virtual void Input(SDL_Event event) override;
    virtual void Update() override;
    virtual void Render() override;
    virtual void Destroy()override;

};

class ContactPointGenerationDemoScene : public Scene
{
private:
    std::unique_ptr<AeroWorld2D> world;
    std::vector<unsigned int> m_bodyColors;
    Vec2 mouseCursor;
    bool leftMouseButtonDown;
    int sliderValue = 0;

    void AddRandomBody();
    void RemoveLastBody();

public:
    virtual void Setup() override;
    virtual void Input(SDL_Event event) override;
    virtual void Update() override;
    virtual void Render() override;
    virtual void Destroy()override;

};

class JointConstraintScene : public Scene
{
private:
    std::unique_ptr<Aerolite::AeroWorld2D> world;
    bool debug;
public:
    virtual void Setup() override;
    virtual void Input(SDL_Event event) override;
    virtual void Update() override;
    virtual void Render() override;
    virtual void Destroy()override;
};

class RagdollJointScene : public Scene
{
private:
    std::unique_ptr<Aerolite::AeroWorld2D> world;
    bool debug;
public:
    virtual void Setup() override;
    virtual void Input(SDL_Event event) override;
    virtual void Update() override;
    virtual void Render() override;
    virtual void Destroy()override;
};

class RigidBodiesAndJointsDemoScene : public Scene
{
private:
    std::unique_ptr<Aerolite::AeroWorld2D> world;
    std::vector<unsigned int> m_bodyColors;
    bool debug;
public:
    virtual void Setup() override;
    virtual void Input(SDL_Event event) override;
    virtual void Update() override;
    virtual void Render() override;
    virtual void Destroy()override;
};

class ProjectileExplosionDemoScene : public Scene
{
private:
    std::unique_ptr<Aerolite::AeroWorld2D> world;
    std::vector<unsigned int> m_bodyColors;
    bool debug;
public:
    virtual void Setup() override;
    virtual void Input(SDL_Event event) override;
    virtual void Update() override;
    virtual void Render() override;
    virtual void Destroy()override;
};

class LargeParticleTestScene : public Scene
{
    std::unique_ptr<Aerolite::AeroWorld2D> world;
    bool debug;
	void CreateRandomCircles(int numberOfCircles, real velocityRange, real radius) const;
public:
    virtual void Setup() override;
    virtual void Input(SDL_Event event) override;
    virtual void Update() override;
    virtual void Render() override;
    virtual void Destroy()override;
};

#endif