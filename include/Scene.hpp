#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "IKChain.hpp"
#include "Camera.hpp"
#include <glm/glm.hpp>

class Scene {
public:
    Scene();
    ~Scene() = default;

    void update();
    void handleInput(GLFWwindow* window, float deltaTime);

    // Getters for renderer
    const ik::IKChain& getIKChain() const { return chain; }
    const Camera& getCamera() const { return camera; }
    const glm::vec3& getTargetPosition() const { return targetPosition; }
    bool shouldShowCones() const { return showCones; }
    
    // UI controls
    void renderUI() const;

private:
    ik::IKChain chain;
    Camera camera;
    mutable glm::vec3 targetPosition;  // Make mutable since we modify it in const UI function
    mutable bool showCones = false;    // Make mutable since we modify it in const UI function
    mutable int numSegments = 3;       // Make mutable since we modify it in const UI function
    mutable bool needsRebuild = false; // Make mutable since we modify it in const UI function
    
    static ik::IKChain createChain(int numSegments);  // Static function to create chain
};