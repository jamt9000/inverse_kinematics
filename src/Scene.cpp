#include "Scene.hpp"
#include <imgui.h>

Scene::Scene() : camera(glm::vec3(0.0f, 0.3f, 3.0f)) {  // Move camera back to see whole arm
    // Place target in a reachable position
    targetPosition = glm::vec3(0.4f, -0.2f, 0.4f);  // Slightly up and to the side
    
    // Initialize chain with default number of segments
    chain = createChain(numSegments);
}

void Scene::update() {
    // Rebuild chain if needed
    if (needsRebuild) {
        chain = createChain(numSegments);
        needsRebuild = false;
    }
    
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
    
    // Add segment control
    if (ImGui::SliderInt("Number of Segments", &numSegments, 2, 10)) {
        needsRebuild = true;
    }
    
    ImGui::Separator();
    
    // Target position controls
    ImGui::Text("Target Position");
    ImGui::SliderFloat("X", &targetPosition.x, -0.8f, 0.8f);    // Left/right
    ImGui::SliderFloat("Y", &targetPosition.y, -0.6f, 0.6f);    // Up/down
    ImGui::SliderFloat("Z", &targetPosition.z, -0.8f, 0.8f);    // Forward/back
    
    ImGui::Separator();
    
    // Show/hide constraint cones
    ImGui::Checkbox("Show Constraints", &showCones);
    
    ImGui::End();
}

ik::IKChain Scene::createChain(int numSegments) {
    ik::IKChain newChain;
    
    float segmentLength = 0.4f;  // Base segment length
    float baseHeight = -0.6f;    // Starting height
    
    // Base joint starts pointing up (+Y) and can rotate around Y axis
    glm::quat baseOrientation = glm::angleAxis(glm::half_pi<float>(), glm::vec3(0.0f, 0.0f, 1.0f));
    ik::Joint baseJoint(
        glm::vec3(0.0f, baseHeight, 0.0f),
        segmentLength,
        glm::vec3(0.0f, 1.0f, 0.0f),        // Y-axis rotation for base
        glm::pi<float>() / 3.0f             // ±60 degrees cone
    );
    baseJoint.orientation = baseOrientation;
    newChain.addJoint(baseJoint);
    
    // Add remaining segments
    glm::vec3 lastPos = baseJoint.position;
    glm::quat lastOrientation = baseOrientation;
    
    for (int i = 1; i < numSegments; i++) {
        // Calculate new position based on previous joint
        glm::vec3 newPos = lastPos + lastOrientation * glm::vec3(0.0f, segmentLength, 0.0f);
        
        // Create new joint
        ik::Joint joint(
            newPos,
            segmentLength * (i == numSegments - 1 ? 0.8f : 1.0f),  // Last segment is slightly shorter
            glm::vec3(1.0f, 0.0f, 0.0f),        // X-axis rotation
            glm::pi<float>() / 4.0f             // ±45 degrees cone
        );
        joint.orientation = lastOrientation;
        newChain.addJoint(joint);
        
        // Update for next iteration
        lastPos = newPos;
    }
    
    return newChain;
}