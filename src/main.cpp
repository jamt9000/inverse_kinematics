#include "Scene.hpp"
#include "Renderer.hpp"
#include <iostream>

int main() {
    try {
        Scene scene;
        Renderer renderer(1280, 720, "Inverse Kinematics");
        
        float lastFrame = 0.0f;
        
        while (!renderer.shouldClose()) {
            float currentFrame = static_cast<float>(glfwGetTime());
            float deltaTime = currentFrame - lastFrame;
            lastFrame = currentFrame;
            
            scene.handleInput(renderer.getWindow(), deltaTime);
            scene.update();
            renderer.render(scene);
        }
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}