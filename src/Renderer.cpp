#include "Renderer.hpp"
#include "Geometry.hpp"
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <glm/gtx/quaternion.hpp>
#include <stdexcept>
#include <iostream>

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
    
    renderIKChain(scene.getIKChain(), scene.shouldShowCones());
    renderTarget(scene.getTargetPosition());
    
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

void Renderer::renderIKChain(const ik::IKChain& chain, bool showCones) {
    const auto& joints = chain.getJoints();
    auto positions = chain.getJointPositions();
    
    // Debug output
    std::cout << "\nJoint positions:" << std::endl;
    for (size_t i = 0; i < positions.size(); i++) {
        std::cout << "Joint " << i << ": (" << positions[i].x << ", " << positions[i].y << ", " << positions[i].z << ")" << std::endl;
    }
    
    // Calculate end effector position
    glm::vec3 endEffectorPos;
    if (!positions.empty()) {
        glm::vec3 lastJointPos = positions.back();
        glm::vec3 direction = glm::normalize(chain.getTarget() - lastJointPos);
        endEffectorPos = lastJointPos + direction * joints.back().length;
        std::cout << "End effector: (" << endEffectorPos.x << ", " << endEffectorPos.y << ", " << endEffectorPos.z << ")" << std::endl;
    }
    
    // Draw segments first (so they appear behind joints)
    for (size_t i = 0; i < joints.size(); i++) {
        glm::vec3 start = positions[i];
        glm::vec3 end = (i < positions.size() - 1) ? positions[i + 1] : endEffectorPos;
        
        // Draw segment
        glm::vec3 direction = glm::normalize(end - start);
        float length = glm::distance(start, end);
        
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, start);
        
        // Create rotation matrix to align segment with direction
        glm::vec3 yAxis(0.0f, 1.0f, 0.0f);
        float angle = glm::acos(glm::dot(yAxis, direction));
        glm::vec3 rotAxis = glm::cross(yAxis, direction);
        
        if (glm::length(rotAxis) > 0.0001f) {
            rotAxis = glm::normalize(rotAxis);
            model = glm::rotate(model, angle, rotAxis);
        }
        
        model = glm::scale(model, glm::vec3(1.0f, length, 1.0f));
        
        shader->setMat4("model", model);
        shader->setVec3("objectColor", glm::vec3(0.2f, 0.2f, 0.9f));  // Blue for segments
        
        glBindVertexArray(gl.segmentVAO);
        glDrawArrays(GL_TRIANGLES, 0, 16 * 12);  // Draw cylinder
    }
    
    // Draw ball joints (red spheres) and cones
    for (size_t i = 0; i < positions.size(); i++) {
        // Draw the red sphere
        renderJointSphere(positions[i]);
        
        // Draw the constraint cone if enabled
        if (showCones) {
            shader->setVec3("objectColor", glm::vec3(0.8f, 0.8f, 0.2f));  // Yellow for constraints
            
            // Get the reference direction (direction of current segment)
            glm::vec3 refDirection;
            if (i == 0) {
                refDirection = glm::vec3(0.0f, 1.0f, 0.0f);  // Base uses vertical as reference
            } else {
                // Use direction of current segment as reference
                refDirection = glm::normalize(positions[i] - positions[i-1]);
            }
            
            float maxAngle = (i == 0) ? glm::pi<float>() / 2.0f  // Base: 90° from vertical
                                    : glm::pi<float>() / 3.0f;   // Others: 60° from previous segment
            
            // For base joint (i=0), use a shorter height since it has a wider angle
            float coneHeight = (i == 0) ? joints[i].length * 0.5f  // Shorter height for base
                                      : joints[i].length;          // Full length for others
            
            renderCone(positions[i], refDirection, maxAngle, coneHeight);
        }
    }
    
    // Draw end effector
    if (!positions.empty()) {
        renderJointSphere(endEffectorPos);
    }
}

void Renderer::renderJointSphere(const glm::vec3& position) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, position);
    
    shader->setMat4("model", model);
    shader->setVec3("objectColor", glm::vec3(0.8f, 0.2f, 0.2f));  // Red for joints
    
    glBindVertexArray(gl.jointVAO);
    glDrawArrays(GL_TRIANGLES, 0, 32 * 32 * 6);  // segments * stacks * triangles per quad
}

void Renderer::renderTarget(const glm::vec3& position) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, position);
    
    shader->setMat4("model", model);
    shader->setVec3("objectColor", glm::vec3(0.1f, 0.7f, 0.1f));  // Darker green for target
    
    glBindVertexArray(gl.targetVAO);
    glDrawArrays(GL_TRIANGLES, 0, 32 * 32 * 6);  // segments * stacks * triangles per quad
}

void Renderer::renderCone(const glm::vec3& tip, const glm::vec3& direction, float angle, float height) {
    // Create model matrix to position and orient the cone
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, tip);  // Move to tip position
    
    // Create rotation from (0,1,0) to direction vector
    glm::vec3 yAxis(0.0f, 1.0f, 0.0f);
    float rotAngle = std::acos(glm::dot(yAxis, direction));
    glm::vec3 rotAxis = glm::cross(yAxis, direction);
    if (glm::length(rotAxis) > 0.0001f) {
        rotAxis = glm::normalize(rotAxis);
        model = glm::rotate(model, rotAngle, rotAxis);
    }
    
    shader->setMat4("model", model);
    
    const int segments = 32;
    const float radius = height * std::tan(angle);
    
    std::vector<glm::vec3> vertices;
    std::vector<GLuint> indices;
    
    // Tip is at origin
    vertices.push_back(glm::vec3(0.0f));
    
    // Create base circle points in XZ plane, height units up in Y
    for (int i = 0; i <= segments; i++) {
        float theta = (float)i / segments * 2.0f * glm::pi<float>();
        vertices.push_back(glm::vec3(
            radius * std::cos(theta),
            height,
            radius * std::sin(theta)
        ));
        
        if (i < segments) {
            // Base circle line
            indices.push_back(i + 1);
            indices.push_back(i + 2);
            
            // Line from tip to base point
            indices.push_back(0);
            indices.push_back(i + 1);
        }
    }
    
    // Create and bind temporary buffers
    GLuint vao, vbo, ebo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);
    
    glBindVertexArray(vao);
    
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), vertices.data(), GL_STATIC_DRAW);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(), GL_STATIC_DRAW);
    
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);
    
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDrawElements(GL_LINES, indices.size(), GL_UNSIGNED_INT, 0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &ebo);
}