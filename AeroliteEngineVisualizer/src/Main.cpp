#include <map>
#include <string>

#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"
#include "Scene.h"
#include "Graphics.h"

int main(int argc, char* args[]) {

    // Initializes SDL2 and the SDL Window/Renderer
    Graphics::OpenWindow();

    std::unique_ptr<Scene> currentScene = nullptr; // Direct pointer to the current scene
    std::string currentSceneKey = "Contact Point Generation Scene";
    std::map<std::string, std::unique_ptr<Scene>> scenes;
    scenes["Contact Point Generation Scene"] = std::make_unique<ContactPointGenerationDemoScene>();
    scenes["Rag doll Joint Constraint Scene"] = std::make_unique<RagdollJointScene>();

    // Initialize the current scene based on currentSceneKey
    currentScene = std::move(scenes[currentSceneKey]);
    currentScene->Setup();

    bool done = false;
    while (!done) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) done = true;
            if(currentScene && currentScene->IsRunning())
            {
                currentScene->Input(event);
            }
        }

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        // ImGui window for scene selection
        ImGui::Begin("Scene Selector");
        for (auto& scene : scenes) {
            if (ImGui::RadioButton(scene.first.c_str(), currentSceneKey == scene.first)) {
                if (currentSceneKey != scene.first) {
                    currentScene->Destroy();
                    currentScene = std::move(scene.second); // Move the selected scene to currentScene
                    currentSceneKey = scene.first;
                    currentScene->Setup();
                }
            }
        }
        ImGui::End();

        // Check if a scene is currently selected and running
        if (currentScene && currentScene->IsRunning()) {
            currentScene->Update();
            currentScene->Render();
        }
    }

    if (currentScene) currentScene->Destroy();
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
}
