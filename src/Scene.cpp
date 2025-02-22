#include "Scene.hpp"
#include <imgui.h>

Scene::Scene() : camera(glm::vec3(0.0f, 0.3f, 3.0f)) {  // Move camera back to see whole arm
    float segmentLength = 0.4f;  // Longer segments
    float baseHeight = -0.6f;    // Starting height
    
    // Base joint starts pointing up (+Y) and can rotate around Y axis
    glm::quat baseOrientation = glm::angleAxis(glm::half_pi<float>(), glm::vec3(0.0f, 0.0f, 1.0f));  // Rotate +X to +Y
    ik::Joint baseJoint(
        glm::vec3(0.0f, baseHeight, 0.0f),
        segmentLength,
        glm::vec3(0.0f, 1.0f, 0.0f),        // Y-axis rotation for base
        glm::pi<float>() / 3.0f             // ±60 degrees cone
    );
    baseJoint.orientation = baseOrientation;
    chain.addJoint(baseJoint);

    // Middle joint inherits base orientation and can rotate around X axis
    glm::vec3 middlePos = baseJoint.position + baseOrientation * glm::vec3(0.0f, segmentLength, 0.0f);
    ik::Joint middleJoint(
        middlePos,
        segmentLength,
        glm::vec3(1.0f, 0.0f, 0.0f),        // X-axis rotation
        glm::pi<float>() / 4.0f             // ±45 degrees cone
    );
    middleJoint.orientation = baseOrientation;  // Start aligned with base
    chain.addJoint(middleJoint);

    // End joint inherits middle orientation and can rotate around X axis
    glm::vec3 endPos = middlePos + baseOrientation * glm::vec3(0.0f, segmentLength, 0.0f);
    ik::Joint endJoint(
        endPos,
        segmentLength * 0.8f,                // Slightly shorter end segment
        glm::vec3(1.0f, 0.0f, 0.0f),        // X-axis rotation
        glm::pi<float>() / 4.0f             // ±45 degrees cone
    );
    endJoint.orientation = baseOrientation;  // Start aligned with others
    chain.addJoint(endJoint);

    // Place target in a reachable position
    targetPosition = glm::vec3(0.4f, -0.2f, 0.4f);  // Slightly up and to the side
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