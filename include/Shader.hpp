#pragma once

#include <glad/glad.h>
#include <string>
#include <glm/glm.hpp>

class Shader {
public:
    // Constructor reads and builds the shader
    Shader(const std::string& vertexPath, const std::string& fragmentPath);
    
    // Use/activate the shader
    void use();
    
    // Utility uniform functions
    void setBool(const std::string& name, bool value) const;
    void setInt(const std::string& name, int value) const;
    void setFloat(const std::string& name, float value) const;
    void setVec3(const std::string& name, const glm::vec3& value) const;
    void setVec4(const std::string& name, const glm::vec4& value) const;
    void setMat4(const std::string& name, const glm::mat4& mat) const;

    // Get program ID
    GLuint getID() const { return ID; }

private:
    GLuint ID;  // Program ID
    
    // Utility function for checking shader compilation/linking errors
    void checkCompileErrors(GLuint shader, const std::string& type);
};