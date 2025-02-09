#include "Scene.hpp"
#include <imgui.h>

Scene::Scene() : camera(glm::vec3(0.0f, 0.3f, 2.0f)) {  // Directly in front, slightly elevated
    float segmentLength = 0.3f;  // Fixed segment length
    float baseHeight = -0.6f;    // Starting height
    
    // Create a vertical robotic arm with equal segment lengths
    chain.addJoint(ik::Joint(glm::vec3(0.0f, baseHeight, 0.0f), segmentLength));                    // Base joint
    chain.addJoint(ik::Joint(glm::vec3(0.0f, baseHeight + segmentLength, 0.0f), segmentLength));    // First middle joint
    chain.addJoint(ik::Joint(glm::vec3(0.0f, baseHeight + segmentLength * 2, 0.0f), segmentLength)); // Second middle joint
    chain.addJoint(ik::Joint(glm::vec3(0.0f, baseHeight + segmentLength * 3, 0.0f), segmentLength)); // Top joint

    // Place target in a clear position
    targetPosition = glm::vec3(0.4f, -0.2f, 0.3f);  // Off to the side and down a bit
}

void Scene::update() {
    // Set the target for IK to be slightly offset from the visual target
    glm::vec3 ikTarget = targetPosition - glm::normalize(targetPosition - chain.getJointPositions().back()) * 0.15f;
    chain.setTarget(ikTarget);
    chain.solve(50, 0.0001f);  // More iterations and tighter tolerance
}

void Scene::handleInput(GLFWwindow* window, float deltaTime) {
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.processKeyboard(CameraMovement::FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.processKeyboard(CameraMovement::BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.processKeyboard(CameraMovement::LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.processKeyboard(CameraMovement::RIGHT, deltaTime);
}

void Scene::renderUI() const {
    ImGui::Begin("IK Controls");
    ImGui::Text("Target Position");
    ImGui::SliderFloat("X", &targetPosition.x, -0.8f, 0.8f);    // Left/right
    ImGui::SliderFloat("Y", &targetPosition.y, -0.6f, 0.6f);    // Up/down (adjusted for new base position)
    ImGui::SliderFloat("Z", &targetPosition.z, -0.8f, 0.8f);    // Forward/back
    ImGui::End();
}