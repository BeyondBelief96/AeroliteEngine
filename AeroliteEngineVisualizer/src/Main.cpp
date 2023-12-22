#include "scenes/Scene.h"


int main(int argc, char *args[]) {
    SpringScene scene;

    scene.Setup();

    while (scene.IsRunning()) {
        scene.Input();
        scene.Update();
        scene.Render();
    }

    scene.Destroy();

    return 0;
}