#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

enum class CameraMovement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    UP,
    DOWN
};

class Camera {
public:
    Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 3.0f),
           glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f),
           float yaw = -90.0f,
           float pitch = 0.0f);

    // Get view matrix
    glm::mat4 getViewMatrix() const;
    
    // Process keyboard input
    void processKeyboard(CameraMovement direction, float deltaTime);
    
    // Process mouse input
    void processMouseMovement(float xoffset, float yoffset, bool constrainPitch = true);
    
    // Process mouse scroll
    void processMouseScroll(float yoffset);

    // Getters
    float getFOV() const { return fov; }
    glm::vec3 getPosition() const { return position; }
    glm::vec3 getFront() const { return front; }

private:
    // Camera attributes
    glm::vec3 position;
    glm::vec3 front;
    glm::vec3 up;
    glm::vec3 right;
    glm::vec3 worldUp;
    
    // Euler angles
    float yaw;
    float pitch;
    
    // Camera options
    float movementSpeed = 2.5f;
    float mouseSensitivity = 0.1f;
    float fov = 45.0f;
    
    // Update camera vectors
    void updateCameraVectors();
};