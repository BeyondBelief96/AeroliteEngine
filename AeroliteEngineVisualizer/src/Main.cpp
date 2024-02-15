#include <iostream>
#include <map>
#include <string>

#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"
#include "Scene.h"
#include "Graphics.h"

float globalFriction = 0.5f; // Default friction value
float globalRestitution = 0.1f; // Default restitution value
float frameRate = 0.0f;

int main(int argc, char* args[]) {

    // Initializes SDL2 and the SDL Window/Renderer
    Graphics::OpenWindow();

    std::shared_ptr<Scene> currentScene = nullptr; // Direct pointer to the current scene
    std::string currentSceneKey = "$5 flappy bird.";
    std::map<std::string, std::shared_ptr<Scene>> scenes;
    scenes["Contacts...contacts everywhere."] = std::make_shared<ContactPointGenerationDemoScene>();
    scenes["$5 flappy bird."] = std::make_shared<FiveDollarFlappyBirdScene>();
    scenes["Well...Its a rag doll alright."] = std::make_shared<RagdollJointScene>();
    scenes["PARTICLES!!!!"] = std::make_shared<LargeParticleTestScene>();
    scenes["Solar System Cyclone"] = std::make_shared<SolarSystemScene>();
    scenes["The Great Pyramid"] = std::make_shared<TheGreatPyramidScene>();

    // Initialize the current scene based on currentSceneKey
    currentScene = scenes[currentSceneKey];
    currentScene->Setup();

    bool done = false;
    while (!done) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);

            // Check if ImGui wants to capture the mouse
            const bool imguiCapturesMouse = ImGui::GetIO().WantCaptureMouse;

            if (event.type == SDL_QUIT) done = true;
            else if (event.key.keysym.sym == SDLK_ESCAPE) done = true;

            // Modify your condition to check if ImGui is not using the mouse
            if (!imguiCapturesMouse && currentScene && currentScene->IsRunning()) {
                currentScene->Input(event);
            }
        }

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        frameRate = ImGui::GetIO().Framerate;
        // ImGui window for scene selection
        ImGui::Begin("Scene Selector");

        for (auto& scene : scenes) {
            if (ImGui::RadioButton(scene.first.c_str(), currentSceneKey == scene.first)) {
                if (currentSceneKey != scene.first) {
                    currentScene->Destroy();
                    currentScene = scene.second; // Move the selected scene to currentScene
                    currentSceneKey = scene.first;
                    currentScene->Setup();
                }
            }
        }

        if(ImGui::Button("Reset Scene", { 100, 30 }))
        {
            currentScene->Destroy();
            currentScene->Setup();
        }

        if (ImGui::CollapsingHeader("Broadphase Options")) {
            static const char* broadPhaseAlgNames[] = { "Brute Force", "Spatial Hash Grid" };
            static int currentAlg = 0; // default to Brute Force

            // Dropdown to select broad phase algorithm
            ImGui::Combo("Algorithm", &currentAlg, broadPhaseAlgNames, IM_ARRAYSIZE(broadPhaseAlgNames));

            // Access the world from the current scene
            const auto world = std::dynamic_pointer_cast<Scene>(currentScene)->GetWorld();

            // Update the broad phase algorithm based on selection
            world->SetBroadPhaseAlgorithm(static_cast<BroadPhaseAlg>(currentAlg));

            // Inputs to adjust cell width and height for Spatial Hash Grid
            if (currentAlg == 1) { // Assuming 1 corresponds to Spatial Hash Grid
                float cellWidth = world->GetShg().GetCellWidth();
                float cellHeight = world->GetShg().GetCellHeight();
                if (ImGui::InputFloat("Cell Width", &cellWidth)) {
                    world->ShgSetCellWidth(cellWidth);
                }
                if (ImGui::InputFloat("Cell Height", &cellHeight)) {
                    world->ShgSetCellHeight(cellHeight);
                }
            }
        }

        if(ImGui::CollapsingHeader("Global Physics Options")) {
            ImGui::SliderFloat("Friction", &globalFriction, 0.0f, 1.0f, "Friction = %.3f");
            ImGui::SliderFloat("Restitution", &globalRestitution, 0.0f, 1.0f, "Restitution = %.3f");
        }

        // Display the FPS
        ImGui::Text("FPS: %.1f", frameRate);


        ImGui::End();
        try
        {
            // Check if a scene is currently selected and running
            if (currentScene && currentScene->IsRunning()) {
                currentScene->UpdatePhysicsProperties(globalFriction, globalRestitution);
                currentScene->Update();
                currentScene->Render();
                ImGui::Render();
                ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData());
                Graphics::RenderFrame();
            }
        }
        catch(std::exception& e)
        {
            std::cout << e.what() << '\n';
        }

        ImGui::EndFrame();
    }

    if (currentScene) currentScene->Destroy();
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    Graphics::CloseWindow();
}
