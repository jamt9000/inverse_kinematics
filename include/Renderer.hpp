#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "Scene.hpp"
#include "Shader.hpp"
#include "Geometry.hpp"
#include <memory>

class Renderer {
public:
    Renderer(int width, int height, const char* title);
    ~Renderer();

    bool shouldClose() const;
    void render(const Scene& scene);

    GLFWwindow* getWindow() { return window; }

private:
    void initGL();
    void initImGui();
    void setupVertexAttributes(GLuint vao, GLuint vbo, const std::vector<float>& vertices);
    void renderIKChain(const ik::IKChain& chain, const Shader& shader);
    void renderTarget(const glm::vec3& position, const Shader& shader);

    GLFWwindow* window;
    std::unique_ptr<Shader> shader;
    
    // OpenGL objects
    struct {
        GLuint jointVAO;
        GLuint jointVBO;
        GLuint segmentVAO;
        GLuint segmentVBO;
        GLuint targetVAO;
        GLuint targetVBO;
    } gl;

    int width;
    int height;
};