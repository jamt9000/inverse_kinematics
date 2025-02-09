#pragma once

#include <vector>
#include <glm/glm.hpp>

namespace geometry {

struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
};

// Create mesh data for basic shapes
std::vector<Vertex> createCylinder(float radius, float height, int segments);
std::vector<Vertex> createSphere(float radius, int segments);

// Convert vertex data to raw float array for OpenGL
std::vector<float> vertexDataToFloatArray(const std::vector<Vertex>& vertices);

} // namespace geometry