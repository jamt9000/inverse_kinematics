#include "Geometry.hpp"
#include <glm/gtc/constants.hpp>

namespace geometry {

std::vector<Vertex> createCylinder(float radius, float height, int segments) {
    std::vector<Vertex> vertices;
    const int verticesPerSegment = 12;  // 2 triangles for side, 1 for top, 1 for bottom
    vertices.reserve(segments * verticesPerSegment);

    // Generate side vertices
    for (int i = 0; i < segments; ++i) {
        float angle1 = (float)i / segments * glm::two_pi<float>();
        float angle2 = (float)(i + 1) / segments * glm::two_pi<float>();

        float x1 = radius * cos(angle1);
        float z1 = radius * sin(angle1);
        float x2 = radius * cos(angle2);
        float z2 = radius * sin(angle2);

        glm::vec3 normal1(cos(angle1), 0.0f, sin(angle1));
        glm::vec3 normal2(cos(angle2), 0.0f, sin(angle2));

        // Side quad (two triangles) - start from 0 and extend up
        vertices.push_back({glm::vec3(x1, 0.0f, z1), normal1});
        vertices.push_back({glm::vec3(x2, 0.0f, z2), normal2});
        vertices.push_back({glm::vec3(x1, height, z1), normal1});

        vertices.push_back({glm::vec3(x1, height, z1), normal1});
        vertices.push_back({glm::vec3(x2, 0.0f, z2), normal2});
        vertices.push_back({glm::vec3(x2, height, z2), normal2});

        // End cap (at y = height)
        vertices.push_back({glm::vec3(0, height, 0), glm::vec3(0, 1, 0)});
        vertices.push_back({glm::vec3(x1, height, z1), glm::vec3(0, 1, 0)});
        vertices.push_back({glm::vec3(x2, height, z2), glm::vec3(0, 1, 0)});

        // Start cap (at y = 0)
        vertices.push_back({glm::vec3(0, 0, 0), glm::vec3(0, -1, 0)});
        vertices.push_back({glm::vec3(x2, 0, z2), glm::vec3(0, -1, 0)});
        vertices.push_back({glm::vec3(x1, 0, z1), glm::vec3(0, -1, 0)});
    }

    return vertices;
}

std::vector<Vertex> createSphere(float radius, int segments) {
    std::vector<Vertex> vertices;
    int stacks = segments / 2;
    vertices.reserve(segments * stacks * 6);

    for (int stack = 0; stack < stacks; ++stack) {
        float phi1 = glm::pi<float>() * (float)stack / stacks;
        float phi2 = glm::pi<float>() * (float)(stack + 1) / stacks;

        for (int slice = 0; slice < segments; ++slice) {
            float theta1 = glm::two_pi<float>() * (float)slice / segments;
            float theta2 = glm::two_pi<float>() * (float)(slice + 1) / segments;

            glm::vec3 v1(
                radius * sin(phi1) * cos(theta1),
                radius * cos(phi1),
                radius * sin(phi1) * sin(theta1)
            );
            glm::vec3 v2(
                radius * sin(phi2) * cos(theta1),
                radius * cos(phi2),
                radius * sin(phi2) * sin(theta1)
            );
            glm::vec3 v3(
                radius * sin(phi2) * cos(theta2),
                radius * cos(phi2),
                radius * sin(phi2) * sin(theta2)
            );
            glm::vec3 v4(
                radius * sin(phi1) * cos(theta2),
                radius * cos(phi1),
                radius * sin(phi1) * sin(theta2)
            );

            // Normals for a sphere are just normalized positions
            glm::vec3 n1 = glm::normalize(v1);
            glm::vec3 n2 = glm::normalize(v2);
            glm::vec3 n3 = glm::normalize(v3);
            glm::vec3 n4 = glm::normalize(v4);

            // First triangle
            vertices.push_back({v1, n1});
            vertices.push_back({v2, n2});
            vertices.push_back({v3, n3});

            // Second triangle
            vertices.push_back({v1, n1});
            vertices.push_back({v3, n3});
            vertices.push_back({v4, n4});
        }
    }

    return vertices;
}

std::vector<float> vertexDataToFloatArray(const std::vector<Vertex>& vertices) {
    std::vector<float> data;
    data.reserve(vertices.size() * 6); // 3 for position, 3 for normal

    for (const auto& vertex : vertices) {
        // Position
        data.push_back(vertex.position.x);
        data.push_back(vertex.position.y);
        data.push_back(vertex.position.z);
        // Normal
        data.push_back(vertex.normal.x);
        data.push_back(vertex.normal.y);
        data.push_back(vertex.normal.z);
    }

    return data;
}

} // namespace geometry