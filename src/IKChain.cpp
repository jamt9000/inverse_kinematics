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
    
    // Start from target
    glm::vec3 endEffector = target_;
    
    // Work backwards from end effector to base
    for (int i = joints_.size() - 1; i >= 0; --i) {
        glm::vec3 nextPos = (i == joints_.size() - 1) ? endEffector : joints_[i + 1].position;
        
        if (i > 0) {  // Don't move base joint
            // Get reference direction (previous segment)
            glm::vec3 referenceDir = glm::normalize(joints_[i].position - joints_[i-1].position);
            
            // Get desired direction to next position
            glm::vec3 desiredDir = glm::normalize(nextPos - joints_[i].position);
            
            // Project onto constraint plane
            glm::vec3 axis = joints_[i].axis;
            glm::vec3 projectedRef = referenceDir - axis * glm::dot(referenceDir, axis);
            glm::vec3 projectedDes = desiredDir - axis * glm::dot(desiredDir, axis);
            
            if (glm::length(projectedRef) > 0.001f && glm::length(projectedDes) > 0.001f) {
                projectedRef = glm::normalize(projectedRef);
                projectedDes = glm::normalize(projectedDes);
                
                // Calculate and clamp angle
                float angle = std::acos(glm::clamp(glm::dot(projectedRef, projectedDes), -1.0f, 1.0f));
                if (glm::dot(glm::cross(projectedRef, projectedDes), axis) < 0) angle = -angle;
                
                float clampedAngle = glm::clamp(angle, joints_[i].minAngle, joints_[i].maxAngle);
                
                // If angle was clamped, create new direction
                if (angle != clampedAngle) {
                    glm::quat rotation = glm::angleAxis(clampedAngle, axis);
                    desiredDir = glm::normalize(rotation * referenceDir);
                }
            }
            
            // Move joint while maintaining length
            joints_[i].position = nextPos - desiredDir * joints_[i].length;
        }
    }
}

void IKChain::forwardPass() {
    if (joints_.empty()) return;
    
    // Keep base at its initial position
    glm::vec3 basePos = joints_[0].position;
    
    // Work forwards from base to end
    for (size_t i = 0; i < joints_.size() - 1; ++i) {
        glm::vec3 currentPos = (i == 0) ? basePos : joints_[i].position;
        
        // Get reference direction
        glm::vec3 referenceDir;
        if (i == 0) {
            referenceDir = glm::vec3(0.0f, 1.0f, 0.0f);  // Base uses vertical
        } else {
            referenceDir = glm::normalize(joints_[i].position - joints_[i-1].position);
        }
        
        // Get desired direction to next joint
        glm::vec3 desiredDir = glm::normalize(joints_[i + 1].position - currentPos);
        
        // Project onto constraint plane
        glm::vec3 axis = joints_[i].axis;
        glm::vec3 projectedRef = referenceDir - axis * glm::dot(referenceDir, axis);
        glm::vec3 projectedDes = desiredDir - axis * glm::dot(desiredDir, axis);
        
        if (glm::length(projectedRef) > 0.001f && glm::length(projectedDes) > 0.001f) {
            projectedRef = glm::normalize(projectedRef);
            projectedDes = glm::normalize(projectedDes);
            
            // Calculate and clamp angle
            float angle = std::acos(glm::clamp(glm::dot(projectedRef, projectedDes), -1.0f, 1.0f));
            if (glm::dot(glm::cross(projectedRef, projectedDes), axis) < 0) angle = -angle;
            
            float clampedAngle = glm::clamp(angle, joints_[i].minAngle, joints_[i].maxAngle);
            
            // If angle was clamped, create new direction
            if (angle != clampedAngle) {
                glm::quat rotation = glm::angleAxis(clampedAngle, axis);
                desiredDir = glm::normalize(rotation * referenceDir);
            }
        }
        
        // Move next joint using constrained direction
        joints_[i + 1].position = currentPos + desiredDir * joints_[i].length;
    }
}

void IKChain::applyConstraints() {
    if (joints_.size() < 2) return;
    
    // For each joint, ensure segments maintain their constraints
    for (size_t i = 0; i < joints_.size(); ++i) {
        // Get the current segment's direction
        glm::vec3 nextPos = (i < joints_.size() - 1) ? joints_[i+1].position : target_;
        glm::vec3 currentDirection = glm::normalize(nextPos - joints_[i].position);
        
        // Get reference direction (vertical for base, previous segment for others)
        glm::vec3 referenceDir;
        if (i == 0) {
            referenceDir = glm::vec3(0.0f, 1.0f, 0.0f);  // Base uses vertical
        } else {
            referenceDir = glm::normalize(joints_[i].position - joints_[i-1].position);
        }
        
        // Project vectors onto plane perpendicular to rotation axis
        glm::vec3 axis = joints_[i].axis;
        
        // Project both vectors onto the plane perpendicular to the rotation axis
        glm::vec3 projectedRef = referenceDir - axis * glm::dot(referenceDir, axis);
        glm::vec3 projectedCur = currentDirection - axis * glm::dot(currentDirection, axis);
        
        float refLength = glm::length(projectedRef);
        float curLength = glm::length(projectedCur);
        
        // Skip if either projection is too small (vectors nearly parallel to axis)
        if (refLength < 0.001f || curLength < 0.001f) continue;
        
        projectedRef = projectedRef / refLength;
        projectedCur = projectedCur / curLength;
        
        // Calculate signed angle between projected vectors
        float angle = std::acos(glm::clamp(glm::dot(projectedRef, projectedCur), -1.0f, 1.0f));
        glm::vec3 cross = glm::cross(projectedRef, projectedCur);
        if (glm::dot(cross, axis) < 0) angle = -angle;
        
        // Get constraints for this joint
        float maxAngle = joints_[i].maxAngle;
        float minAngle = joints_[i].minAngle;
        
        if (angle > maxAngle || angle < minAngle) {
            // Clamp to nearest valid angle
            float clampedAngle = glm::clamp(angle, minAngle, maxAngle);
            
            // Create rotation to bring current direction to clamped angle
            glm::quat rotation = glm::angleAxis(clampedAngle - angle, axis);
            
            // Apply rotation to get new direction
            glm::vec3 clampedDirection = glm::normalize(rotation * currentDirection);
            
            // Update current joint's next position
            if (i < joints_.size() - 1) {
                joints_[i+1].position = joints_[i].position + clampedDirection * joints_[i].length;
                
                // Propagate change through rest of chain
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
        
        // Get reference direction for this ball joint
        glm::vec3 referenceDir;
        if (i == 0) {
            referenceDir = glm::vec3(0.0f, 1.0f, 0.0f);  // Base ball joint: vertical reference
        } else {
            referenceDir = glm::normalize(joints_[i].position - joints_[i-1].position);  // Previous segment
        }
        
        // Calculate rotation from reference to current direction
        glm::vec3 rotationAxis = glm::normalize(glm::cross(referenceDir, currentDir));
        if (glm::length(rotationAxis) < 0.001f) {
            rotationAxis = joints_[i].axis;  // Fallback if vectors are parallel
        }
        
        float angle = std::acos(glm::clamp(glm::dot(referenceDir, currentDir), -1.0f, 1.0f));
        joints_[i].rotation = glm::angleAxis(angle, rotationAxis);
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