#include "IKChain.hpp"
#include <glm/gtx/quaternion.hpp>
#include <algorithm>

/*
 * FABRIK (Forward And Backward Reaching Inverse Kinematics)
 * 
 * FABRIK is an iterative IK solver that works by:
 * 1. Backward Pass (from target to base):
 *    - Set end effector at target position
 *    - Work backwards, placing each joint at the correct distance from the next
 *    - This ensures reaching but loses the base constraint
 * 
 * 2. Forward Pass (from base to target):
 *    - Fix base joint back at origin
 *    - Work forwards, placing each joint at the correct distance from the previous
 *    - This maintains the base constraint but might lose target reaching
 * 
 * 3. Angle Constraints:
 *    - First joint: measures angle between up vector (0,1,0) and first segment
 *    - Other joints: measure angle between their two adjacent segments
 *    - All angles are measured in the plane perpendicular to each joint's rotation axis
 *    - For example: if axis is (0,1,0), we measure angle between vectors projected onto XZ plane
 *    - Base joint: ±90° around Y axis (left/right swivel from vertical)
 *    - Other joints: ±60° around Z axis (up/down bending between segments)
 *    - Angles are signed (+ or -) based on cross product with rotation axis
 * 
 * The algorithm iterates these passes until:
 * - The end effector is close enough to target (within tolerance)
 * - Maximum iterations reached
 * - Solution becomes stable (error stops improving)
 * 
 * Our implementation adds:
 * - Collision avoidance with target sphere
 * - Position interpolation for smooth movement
 * - Mechanical joint constraints with different axes per joint
 * 
 * Reference: Aristidou, A., & Lasenby, J. (2011). FABRIK: A fast, iterative solver 
 * for the Inverse Kinematics problem. Graphical Models, 73(5), 243-260.
 */

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
    float totalError = 0.0f;
    int stableCount = 0;
    
    // Store initial positions
    std::vector<glm::vec3> initialPositions;
    initialPositions.reserve(joints_.size());
    for (const auto& joint : joints_) {
        initialPositions.push_back(joint.position);
    }
    
    for (int iter = 0; iter < maxIterations; ++iter) {
        float prevError = totalError;
        
        backwardPass();
        forwardPass();
        applyConstraints();
        
        // Interpolate between previous and new positions for smoother movement
        float t = 0.3f;  // Interpolation factor (smaller = smoother but slower)
        for (size_t i = 0; i < joints_.size(); ++i) {
            joints_[i].position = glm::mix(initialPositions[i], joints_[i].position, t);
        }
        
        // Calculate total error
        totalError = glm::distance(joints_.back().position, target_);
        
        // Check for convergence
        if (totalError < tolerance) {
            break;
        }
        
        // Check for stability
        if (std::abs(totalError - prevError) < tolerance * 0.1f) {
            stableCount++;
            if (stableCount > 3) break;  // Break if solution is stable for several iterations
        } else {
            stableCount = 0;
        }
    }

    calculateRotations();
}

void IKChain::backwardPass() {
    if (joints_.empty()) return;
    
    // Calculate the end effector position (one segment length beyond the last joint)
    glm::vec3 endEffector = target_;
    
    // Work backwards from end effector to base
    for (int i = joints_.size() - 1; i >= 0; --i) {
        glm::vec3 nextPos = (i == joints_.size() - 1) ? endEffector : joints_[i + 1].position;
        glm::vec3 currentPos = joints_[i].position;
        glm::vec3 desiredDir = glm::normalize(nextPos - currentPos);
        
        // Project the desired direction onto the plane perpendicular to the rotation axis
        glm::vec3 axis = joints_[i].axis;
        glm::vec3 projectedDir = glm::normalize(desiredDir - glm::dot(desiredDir, axis) * axis);
        
        // Move the joint using the projected direction
        if (i > 0) {
            joints_[i].position = nextPos - projectedDir * joints_[i].length;
        }
    }
}

void IKChain::forwardPass() {
    if (joints_.empty()) return;
    
    // Fix base at origin
    joints_[0].position = glm::vec3(0.0f);
    
    // Work forwards from base to end
    for (size_t i = 0; i < joints_.size() - 1; ++i) {
        glm::vec3 currentPos = joints_[i].position;
        glm::vec3 nextPos = joints_[i + 1].position;
        glm::vec3 desiredDir = glm::normalize(nextPos - currentPos);
        
        // Project the desired direction onto the plane perpendicular to the rotation axis
        glm::vec3 axis = joints_[i].axis;
        glm::vec3 projectedDir = glm::normalize(desiredDir - glm::dot(desiredDir, axis) * axis);
        
        // Move the next joint using the projected direction
        joints_[i + 1].position = currentPos + projectedDir * joints_[i].length;
    }
    
    // Ensure last segment points toward target
    if (!joints_.empty()) {
        glm::vec3 lastPos = joints_.back().position;
        glm::vec3 desiredDir = glm::normalize(target_ - lastPos);
        
        // Project the final direction onto the last joint's rotation plane
        glm::vec3 axis = joints_.back().axis;
        glm::vec3 projectedDir = glm::normalize(desiredDir - glm::dot(desiredDir, axis) * axis);
        
        // The end effector position would be:
        // lastPos + projectedDir * joints_.back().length
    }
}

void IKChain::applyConstraints() {
    if (joints_.size() < 2) return;
    
    // For each joint, measure angle between its segments and apply constraints
    for (size_t i = 0; i < joints_.size(); ++i) {
        // Get the current segment's direction
        glm::vec3 nextPos = (i < joints_.size() - 1) ? joints_[i+1].position : target_;
        glm::vec3 currentDirection = glm::normalize(nextPos - joints_[i].position);
        
        // Get the previous segment's direction (or use reference direction for base)
        glm::vec3 prevDirection;
        if (i == 0) {
            // For base joint, use vertical direction as reference
            // This means the base joint measures angle from vertical in the XZ plane
            prevDirection = glm::vec3(0.0f, 1.0f, 0.0f);
        } else {
            prevDirection = glm::normalize(joints_[i].position - joints_[i-1].position);
        }
        
        // Project both directions onto plane perpendicular to rotation axis
        // For Y-axis rotation (base): Projects onto XZ plane for left/right swivel
        // For Z-axis rotation (others): Projects onto XY plane for up/down bending
        glm::vec3 axis = joints_[i].axis;
        glm::vec3 projectedPrev = glm::normalize(prevDirection - glm::dot(prevDirection, axis) * axis);
        glm::vec3 projectedCurrent = glm::normalize(currentDirection - glm::dot(currentDirection, axis) * axis);
        
        // Calculate signed angle between the projected vectors in the rotation plane
        // Sign is determined by cross product with rotation axis (+ is CCW around axis)
        float angle = glm::sign(glm::dot(glm::cross(projectedPrev, projectedCurrent), axis)) *
                     glm::acos(glm::clamp(glm::dot(projectedPrev, projectedCurrent), -1.0f, 1.0f));
        
        // Apply angle constraints
        if (angle < joints_[i].minAngle || angle > joints_[i].maxAngle) {
            float clampedAngle = glm::clamp(angle, joints_[i].minAngle, joints_[i].maxAngle);
            
            // Create rotation around the joint's axis by the clamped angle
            glm::quat rotation = glm::angleAxis(clampedAngle, axis);
            // Apply rotation to the projected previous direction
            glm::vec3 newDirection = glm::normalize(rotation * projectedPrev);
            
            // Update joint chain with constrained angle
            if (i < joints_.size() - 1) {
                joints_[i+1].position = joints_[i].position + newDirection * joints_[i].length;
                
                // Propagate the change to remaining joints to maintain chain integrity
                for (size_t j = i + 2; j < joints_.size(); ++j) {
                    glm::vec3 dir = glm::normalize(joints_[j].position - joints_[j-1].position);
                    joints_[j].position = joints_[j-1].position + dir * joints_[j-1].length;
                }
            }
        }
    }
}

void IKChain::calculateRotations() {
    if (joints_.empty()) return;
    
    for (size_t i = 0; i < joints_.size(); ++i) {
        // Get current segment direction
        glm::vec3 currentDir;
        if (i < joints_.size() - 1) {
            currentDir = glm::normalize(joints_[i + 1].position - joints_[i].position);
        } else {
            currentDir = glm::normalize(target_ - joints_[i].position);
        }
        
        // Get previous segment direction (or up vector for base)
        glm::vec3 prevDir;
        if (i == 0) {
            prevDir = glm::vec3(0.0f, 1.0f, 0.0f);  // Use up vector as reference for base
        } else {
            prevDir = glm::normalize(joints_[i].position - joints_[i-1].position);
        }
        
        // Project both directions onto plane perpendicular to joint's rotation axis
        glm::vec3 axis = joints_[i].axis;
        glm::vec3 projectedPrev = glm::normalize(prevDir - glm::dot(prevDir, axis) * axis);
        glm::vec3 projectedCurrent = glm::normalize(currentDir - glm::dot(currentDir, axis) * axis);
        
        // Calculate angle between projected vectors
        float angle = glm::sign(glm::dot(glm::cross(projectedPrev, projectedCurrent), axis)) *
                     glm::acos(glm::clamp(glm::dot(projectedPrev, projectedCurrent), -1.0f, 1.0f));
        
        // Create rotation around joint's axis
        joints_[i].rotation = glm::angleAxis(angle, axis);
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