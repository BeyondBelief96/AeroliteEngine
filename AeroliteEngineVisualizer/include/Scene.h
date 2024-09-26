#ifndef APPLICATION_H
#define APPLICATION_H

#include <vector>
#include <memory>

#include "Particle2D.h"
#include "AeroBody2D.h"
#include "AeroWorld2D.h"

#include "Graphics.h"

namespace Aerolite {
    class Scene {
    protected:
        std::unique_ptr<AeroWorld2D> world;
        bool running = false;
        int timePreviousFrame = 0;
    public:
        bool IsRunning() const { return running; }
        virtual void Setup() = 0;
        virtual void Input(SDL_Event event) = 0;
        virtual void Update() = 0;
        virtual void Render() = 0;
        void Destroy();
        Scene() = default;
        virtual ~Scene() = default;

        void UpdatePhysicsProperties(const real friction, const real restitution) const;
        AeroWorld2D* GetWorld() const { return world.get(); }
    };

    class SolarSystemScene : public Scene
    {
    private:
        std::vector<float> planetRadii;
        Vec2 mouseCursor;
        bool leftMouseButtonDown = false;
    public:
        void GenerateSolarSystem(
            std::vector<std::unique_ptr<Particle2D>>& planets,
            const std::shared_ptr<Particle2D>& sun,
            int numPlanets,
            real gravitationalConstant,
            real minOrbitRadius,
            real maxOrbitRadius,
            real sunMass);


        virtual void Setup() override;
        virtual void Input(SDL_Event event) override;
        virtual void Update() override;
        virtual void Render() override;
    };

    class ContactPointGenerationDemoScene : public Scene
    {
    private:
        std::vector<unsigned int> m_bodyColors;
        Vec2 mouseCursor;
        bool leftMouseButtonDown = false;
        int sliderValue = 0;

        void AddRandomBody();
        void RemoveLastBody();

    public:
        virtual void Setup() override;
        virtual void Input(SDL_Event event) override;
        virtual void Update() override;
        virtual void Render() override;

    };

    class JointConstraintScene : public Scene
    {
    private:
        bool debug = false;
    public:
        virtual void Setup() override;
        virtual void Input(SDL_Event event) override;
        virtual void Update() override;
        virtual void Render() override;
    };

    class RagdollJointScene : public Scene
    {
    private:
        bool debug = false;
    public:
        virtual void Setup() override;
        virtual void Input(SDL_Event event) override;
        virtual void Update() override;
        virtual void Render() override;
    };

    class FiveDollarFlappyBirdScene : public Scene
    {
    private:
        std::vector<unsigned int> m_bodyColors;
        bool debug = false;
    public:
        virtual void Setup() override;
        virtual void Input(SDL_Event event) override;
        virtual void Update() override;
        virtual void Render() override;
    };

    class TheGreatPyramidScene : public Scene
    {
    private:
        int pyramidHeight = 5;
        std::vector<unsigned int> m_bodyColors;
        bool debug = false;
    public:
        virtual void Setup() override;
        virtual void Input(SDL_Event event) override;
        virtual void Update() override;
        virtual void Render() override;
    };

    class LargeParticleTestScene : public Scene
    {
    private:
        bool debug = false;
        void CreateRandomCircles(int numberOfCircles, real velocityRange, real radius) const;
    public:
        virtual void Setup() override;
        virtual void Input(SDL_Event event) override;
        virtual void Update() override;
        virtual void Render() override;
    };

    class AirplaneShootingScene : public Scene
    {
    private:
        bool debug = false;
        std::vector<unsigned int> m_bodyColors;
    public:
        virtual void Setup() override;
        virtual void Input(SDL_Event event) override;
        virtual void Update() override;
        virtual void Render() override;
    };
}

#endif