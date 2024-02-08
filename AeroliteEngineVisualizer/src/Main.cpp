#include <map>
#include <string>

#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"
#include "Scene.h"
#include "Graphics.h"

int main(int argc, char* args[]) {

    // Initializes SDL2 and the SDL Window/Renderer
    Graphics::OpenWindow();

    std::shared_ptr<Scene> currentScene = nullptr; // Direct pointer to the current scene
    std::string currentSceneKey = "$5 flappy bird.";
    std::map<std::string, std::shared_ptr<Scene>> scenes;
    scenes["Contacts...contacts everywhere."] = std::make_shared<ContactPointGenerationDemoScene>();
    scenes["$5 flappy bird."] = std::make_shared<RigidBodiesAndJointsDemoScene>();
    scenes["Well...Its a rag doll alright."] = std::make_shared<RagdollJointScene>();
    scenes["Destroy the castle!"] = std::make_shared<ProjectileExplosionDemoScene>();

    // Initialize the current scene based on currentSceneKey
    currentScene = scenes[currentSceneKey];
    currentScene->Setup();

    bool done = false;
    while (!done) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) done = true;
            else if (event.key.keysym.sym == SDLK_ESCAPE) done = true;

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
                    currentScene = scene.second; // Move the selected scene to currentScene
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
            ImGui::Render();
            ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData());
            Graphics::RenderFrame();
        }

        ImGui::EndFrame();
    }

    if (currentScene) currentScene->Destroy();
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    Graphics::CloseWindow();
}
