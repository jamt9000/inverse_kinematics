#include "Renderer.hpp"
#include "Geometry.hpp"
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <stdexcept>

Renderer::Renderer(int width, int height, const char* title) 
    : width(width), height(height) {
    
    initGL();
    window = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }
    
    glfwMakeContextCurrent(window);
    
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        throw std::runtime_error("Failed to initialize GLAD");
    }
    
    initImGui();
    
    // Create and compile shader
    shader = std::make_unique<Shader>("shaders/vertex.glsl", "shaders/fragment.glsl");
    
    // Set up geometry with robot-like proportions
    auto jointMesh = geometry::createSphere(0.06f, 32);              // Spherical joints
    auto segmentMesh = geometry::createCylinder(0.05f, 1.0f, 16);    // Keep segments as cylinders
    auto targetMesh = geometry::createSphere(0.08f, 32);             // Keep target size
    
    auto jointVertices = geometry::vertexDataToFloatArray(jointMesh);
    auto segmentVertices = geometry::vertexDataToFloatArray(segmentMesh);
    auto targetVertices = geometry::vertexDataToFloatArray(targetMesh);
    
    // Set up VAOs and VBOs
    glGenVertexArrays(1, &gl.jointVAO);
    glGenBuffers(1, &gl.jointVBO);
    setupVertexAttributes(gl.jointVAO, gl.jointVBO, jointVertices);
    
    glGenVertexArrays(1, &gl.segmentVAO);
    glGenBuffers(1, &gl.segmentVBO);
    setupVertexAttributes(gl.segmentVAO, gl.segmentVBO, segmentVertices);
    
    glGenVertexArrays(1, &gl.targetVAO);
    glGenBuffers(1, &gl.targetVBO);
    setupVertexAttributes(gl.targetVAO, gl.targetVBO, targetVertices);
    
    // Enable depth testing
    glEnable(GL_DEPTH_TEST);
}

Renderer::~Renderer() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    
    glDeleteVertexArrays(1, &gl.jointVAO);
    glDeleteBuffers(1, &gl.jointVBO);
    glDeleteVertexArrays(1, &gl.segmentVAO);
    glDeleteBuffers(1, &gl.segmentVBO);
    glDeleteVertexArrays(1, &gl.targetVAO);
    glDeleteBuffers(1, &gl.targetVBO);
    
    glfwDestroyWindow(window);
    glfwTerminate();
}

bool Renderer::shouldClose() const {
    return glfwWindowShouldClose(window);
}

void Renderer::render(const Scene& scene) {
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Start ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    
    // Render scene geometry
    shader->use();
    
    // Set up view/projection matrices
    const auto& camera = scene.getCamera();
    glm::mat4 projection = glm::perspective(glm::radians(camera.getFOV()),
        static_cast<float>(width) / height, 0.1f, 100.0f);
    glm::mat4 view = camera.getViewMatrix();
    
    shader->setMat4("projection", projection);
    shader->setMat4("view", view);
    shader->setVec3("lightPos", glm::vec3(5.0f, 5.0f, 5.0f));
    shader->setVec3("viewPos", camera.getPosition());
    shader->setVec3("lightColor", glm::vec3(1.0f));
    
    // Render IK chain
    renderIKChain(scene.getIKChain(), *shader);
    
    // Render target
    renderTarget(scene.getTargetPosition(), *shader);
    
    // Render UI
    scene.renderUI();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    
    glfwSwapBuffers(window);
    glfwPollEvents();
}

void Renderer::initGL() {
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }
    
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
}

void Renderer::initImGui() {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    ImGui::StyleColorsDark();
}

void Renderer::setupVertexAttributes(GLuint vao, GLuint vbo, const std::vector<float>& vertices) {
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
    
    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // Normal attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
}

void Renderer::renderIKChain(const ik::IKChain& chain, const Shader& shader) {
    const auto& joints = chain.getJoints();
    const auto& positions = chain.getJointPositions();
    const auto& target = chain.getTarget();  // Get target from chain

    // Draw segments first (so they appear behind joints)
    for (size_t i = 0; i < positions.size() - 1; ++i) {
        glm::vec3 direction = positions[i + 1] - positions[i];
        float length = glm::length(direction);
        direction = glm::normalize(direction);
        
        // Create segment model matrix starting from joint position
        glm::mat4 segmentModel = glm::mat4(1.0f);
        segmentModel = glm::translate(segmentModel, positions[i]);  // Start at current joint
        
        // Create rotation matrix to align segment with direction
        glm::vec3 yAxis(0.0f, 1.0f, 0.0f);
        float angle = glm::acos(glm::dot(yAxis, direction));
        glm::vec3 rotAxis = glm::cross(yAxis, direction);
        
        if (glm::length(rotAxis) > 0.0001f) {
            rotAxis = glm::normalize(rotAxis);
            segmentModel = glm::rotate(segmentModel, angle, rotAxis);
        }
        
        // Scale and position segment - extend fully between joints
        segmentModel = glm::scale(segmentModel, glm::vec3(1.0f, length, 1.0f));  // Use full length
        
        shader.setMat4("model", segmentModel);
        shader.setVec3("objectColor", glm::vec3(0.2f, 0.2f, 0.9f));  // Brighter blue for segments
        
        glBindVertexArray(gl.segmentVAO);
        glDrawArrays(GL_TRIANGLES, 0, 16 * 12);
    }
    
    // For the last segment, draw to target
    if (!positions.empty()) {
        glm::vec3 direction = target - positions.back();
        float length = glm::length(direction);
        direction = glm::normalize(direction);
        
        glm::mat4 segmentModel = glm::mat4(1.0f);
        segmentModel = glm::translate(segmentModel, positions.back());
        
        glm::vec3 yAxis(0.0f, 1.0f, 0.0f);
        float angle = glm::acos(glm::dot(yAxis, direction));
        glm::vec3 rotAxis = glm::cross(yAxis, direction);
        
        if (glm::length(rotAxis) > 0.0001f) {
            rotAxis = glm::normalize(rotAxis);
            segmentModel = glm::rotate(segmentModel, angle, rotAxis);
        }
        
        // Use full length for the segment
        segmentModel = glm::scale(segmentModel, glm::vec3(1.0f, length, 1.0f));
        
        shader.setMat4("model", segmentModel);
        shader.setVec3("objectColor", glm::vec3(0.2f, 0.2f, 0.9f));
        
        glBindVertexArray(gl.segmentVAO);
        glDrawArrays(GL_TRIANGLES, 0, 16 * 12);
    }

    // Draw joints on top
    for (size_t i = 0; i < joints.size(); ++i) {
        glm::mat4 jointModel = glm::mat4(1.0f);
        jointModel = glm::translate(jointModel, positions[i]);
        
        shader.setMat4("model", jointModel);
        shader.setVec3("objectColor", glm::vec3(0.8f, 0.2f, 0.2f));  // Red for joints
        
        glBindVertexArray(gl.jointVAO);
        glDrawArrays(GL_TRIANGLES, 0, 32 * 32 * 6);  // Use sphere vertex count
    }

    // Draw end effector joint at the end of the last segment
    if (!positions.empty()) {
        glm::mat4 jointModel = glm::mat4(1.0f);
        jointModel = glm::translate(jointModel, target);  // Place at the actual IK target
        
        shader.setMat4("model", jointModel);
        shader.setVec3("objectColor", glm::vec3(0.8f, 0.2f, 0.2f));  // Red for joints
        
        glBindVertexArray(gl.jointVAO);
        glDrawArrays(GL_TRIANGLES, 0, 32 * 32 * 6);  // Use sphere vertex count
    }
}

void Renderer::renderTarget(const glm::vec3& position, const Shader& shader) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, position);
    
    shader.setMat4("model", model);
    shader.setVec3("objectColor", glm::vec3(0.1f, 0.7f, 0.1f));  // Darker green for target
    
    glBindVertexArray(gl.targetVAO);
    glDrawArrays(GL_TRIANGLES, 0, 32 * 32 * 6);  // segments * stacks * triangles per quad
}