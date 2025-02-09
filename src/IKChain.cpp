#include "IKChain.hpp"
#include <glm/gtx/quaternion.hpp>
#include <algorithm>

namespace ik {

void IKChain::addJoint(const Joint& joint) {
    joints_.push_back(joint);
}

void IKChain::setTarget(const glm::vec3& target) {
    target_ = target;
}

glm::vec3 IKChain::getEndEffectorPosition() const {
    if (joints_.empty()) return glm::vec3(0.0f);
    
    glm::vec3 pos(0.0f);
    glm::quat totalRot(1.0f, 0.0f, 0.0f, 0.0f);
    
    for (const auto& joint : joints_) {
        totalRot = totalRot * joint.rotation;
        pos += totalRot * glm::vec3(joint.length, 0.0f, 0.0f);
    }
    
    return pos;
}

void IKChain::solve(int maxIterations, float tolerance) {
    if (joints_.empty()) return;

    std::vector<glm::vec3> positions(joints_.size() + 1);
    
    // Initialize positions
    positions[0] = glm::vec3(0.0f);  // Base ALWAYS at origin
    for (size_t i = 0; i < joints_.size(); ++i) {
        positions[i + 1] = positions[i] + joints_[i].length * glm::normalize(
            i < joints_.size() - 1 ? joints_[i + 1].position - joints_[i].position : 
            target_ - joints_[i].position
        );
    }

    for (int iter = 0; iter < maxIterations; ++iter) {
        // BACKWARD PASS (start from target)
        positions[joints_.size()] = target_;  // Set end effector to target
        
        for (int i = joints_.size() - 1; i >= 0; --i) {
            float segmentLength = joints_[i].length;
            glm::vec3 direction = glm::normalize(positions[i] - positions[i + 1]);
            positions[i] = positions[i + 1] + direction * segmentLength;
        }

        // FORWARD PASS (start from base)
        positions[0] = glm::vec3(0.0f);  // Reset base to origin
        
        for (size_t i = 0; i < joints_.size(); ++i) {
            float segmentLength = joints_[i].length;
            glm::vec3 direction = glm::normalize(positions[i + 1] - positions[i]);
            positions[i + 1] = positions[i] + direction * segmentLength;
        }

        // One final backward pass to ensure we reach the target
        positions[joints_.size()] = target_;
        for (int i = joints_.size() - 1; i >= 0; --i) {
            float segmentLength = joints_[i].length;
            glm::vec3 direction = glm::normalize(positions[i] - positions[i + 1]);
            positions[i] = positions[i + 1] + direction * segmentLength;
        }

        // Update joint positions
        for (size_t i = 0; i < joints_.size(); ++i) {
            joints_[i].position = positions[i];
        }

        // Check if we're close enough to target
        float error = glm::distance(positions[joints_.size()], target_);
        if (error < tolerance) break;
    }

    // Calculate rotations for visualization
    for (size_t i = 0; i < joints_.size(); ++i) {
        glm::vec3 currentDir;
        if (i < joints_.size() - 1) {
            currentDir = glm::normalize(joints_[i + 1].position - joints_[i].position);
        } else {
            currentDir = glm::normalize(target_ - joints_[i].position);
        }

        // Calculate rotation from X axis to current direction
        glm::vec3 xAxis(1.0f, 0.0f, 0.0f);
        float angle = glm::acos(glm::dot(xAxis, currentDir));
        glm::vec3 rotAxis = glm::cross(xAxis, currentDir);
        
        if (glm::length(rotAxis) > 0.0001f) {
            rotAxis = glm::normalize(rotAxis);
            joints_[i].rotation = glm::angleAxis(angle, rotAxis);
        } else {
            // Handle case where vectors are parallel
            if (glm::dot(xAxis, currentDir) < 0) {
                // 180-degree rotation needed
                joints_[i].rotation = glm::angleAxis(glm::pi<float>(), glm::vec3(0.0f, 1.0f, 0.0f));
            } else {
                // No rotation needed
                joints_[i].rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            }
        }
    }
}

std::vector<glm::vec3> IKChain::getJointPositions() const {
    std::vector<glm::vec3> positions;
    positions.reserve(joints_.size());
    
    for (const auto& joint : joints_) {
        positions.push_back(joint.position);
    }
    
    return positions;
}

std::vector<glm::quat> IKChain::getJointRotations() const {
    std::vector<glm::quat> rotations;
    rotations.reserve(joints_.size());
    
    for (const auto& joint : joints_) {
        rotations.push_back(joint.rotation);
    }
    
    return rotations;
}

} // namespace ik