#pragma once

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace ik {

struct Joint {
    glm::vec3 position;          // Joint position in local space
    glm::quat rotation;          // Joint rotation
    glm::vec3 axis;             // Rotation axis
    float length;               // Length to next joint
    float minAngle;            // Constraint: minimum angle
    float maxAngle;            // Constraint: maximum angle
    
    Joint(const glm::vec3& pos, float len, const glm::vec3& rotAxis = glm::vec3(0, 0, 1))
        : position(pos)
        , rotation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f))
        , axis(rotAxis)
        , length(len)
        , minAngle(-glm::pi<float>())
        , maxAngle(glm::pi<float>())
    {}
};

class IKChain {
public:
    IKChain() = default;
    ~IKChain() = default;

    // Add a joint to the chain
    void addJoint(const Joint& joint);
    
    // Set target position for end effector
    void setTarget(const glm::vec3& target);
    
    // Get current end effector position
    glm::vec3 getEndEffectorPosition() const;
    
    // Get current target position
    glm::vec3 getTarget() const { return target_; }
    
    // Get joints for rendering
    const std::vector<Joint>& getJoints() const { return joints_; }
    
    // Solve IK using FABRIK (Forward And Backward Reaching Inverse Kinematics)
    void solve(int maxIterations = 10, float tolerance = 0.001f);
    
    // Get joint positions for rendering
    std::vector<glm::vec3> getJointPositions() const;
    
    // Get joint rotations for rendering
    std::vector<glm::quat> getJointRotations() const;

private:
    std::vector<Joint> joints_;
    glm::vec3 target_{0.0f};
    
    // Forward pass of FABRIK
    void forwardPass();
    
    // Backward pass of FABRIK
    void backwardPass();
    
    // Apply joint constraints
    void applyConstraints();
    
    // Calculate joint rotations from positions
    void calculateRotations();
};

} // namespace ik