#include "Scene.hpp"
#include <imgui.h>

Scene::Scene() : camera(glm::vec3(0.0f, 0.3f, 3.0f)) {  // Move camera back to see whole arm
    float segmentLength = 0.3f;  // Fixed segment length
    float baseHeight = -0.6f;    // Starting height
    
    // Create first segment pointing straight up
    ik::Joint baseJoint(
        glm::vec3(0.0f, baseHeight, 0.0f),  // Base position stays on ground
        segmentLength,
        glm::vec3(1.0f, 0.0f, 0.0f),        // X-axis rotation for base
        -glm::pi<float>() / 2.0f,           // minAngle: -90 degrees
        glm::pi<float>() / 2.0f             // maxAngle: +90 degrees
    );
    chain.addJoint(baseJoint);

    // Position second joint bent forward
    ik::Joint middleJoint(
        glm::vec3(0.0f, baseHeight + segmentLength, 0.0f),  // Start at end of first segment
        segmentLength,
        glm::vec3(1.0f, 0.0f, 0.0f),        // X-axis rotation for middle joint
        -glm::pi<float>() / 3.0f,           // minAngle: -60 degrees
        glm::pi<float>() / 3.0f             // maxAngle: +60 degrees
    );
    chain.addJoint(middleJoint);

    // Position third joint bent back
    ik::Joint endJoint(
        glm::vec3(0.0f, baseHeight + segmentLength * 1.7f, segmentLength * 0.7f),  // Bent position
        segmentLength,
        glm::vec3(1.0f, 0.0f, 0.0f),        // X-axis rotation for end joint
        -glm::pi<float>() / 3.0f,           // minAngle: -60 degrees
        glm::pi<float>() / 3.0f             // maxAngle: +60 degrees
    );
    chain.addJoint(endJoint);

    // Place target in an interesting position
    targetPosition = glm::vec3(0.3f, baseHeight + segmentLength * 2, 0.3f);  // Off to the side
}

void Scene::update() {
    // Keep the end effector from overlapping with the target sphere
    float targetSphereRadius = 0.08f;  // Match the radius used in Renderer.cpp
    float endEffectorRadius = 0.06f;   // Match the radius used in Renderer.cpp
    float minDistance = targetSphereRadius + endEffectorRadius;  // Prevent overlap
    
    // Calculate direction from target to last joint
    glm::vec3 lastJointPos = chain.getJointPositions().back();
    glm::vec3 toTarget = targetPosition - lastJointPos;
    glm::vec3 direction = glm::normalize(toTarget);
    
    // Set IK target to be offset from the visual target
    glm::vec3 ikTarget = targetPosition - direction * minDistance;
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
    ImGui::Separator();
    ImGui::Checkbox("Show Joint Cones", &showCones);
    ImGui::End();
}