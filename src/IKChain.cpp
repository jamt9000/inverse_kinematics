#include "IKChain.hpp"
#include <glm/gtx/quaternion.hpp>
#include <algorithm>
#include <iostream>

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
        totalRot = totalRot * joint.orientation;
        pos += totalRot * glm::vec3(joint.length, 0.0f, 0.0f);
    }
    
    return pos;
}

// Helper function for swing-twist decomposition with improved numerical stability
std::pair<glm::quat, glm::quat> decomposeSwingTwist(const glm::quat& q, const glm::vec3& twistAxis) {
    // Normalize inputs for numerical stability
    glm::quat qn = glm::normalize(q);
    glm::vec3 axisN = glm::normalize(twistAxis);
    
    // Get the twist component
    float d = 2.0f * (axisN.x * qn.x + axisN.y * qn.y + axisN.z * qn.z);
    glm::quat twist(
        qn.w,
        axisN.x * d / 2.0f,
        axisN.y * d / 2.0f,
        axisN.z * d / 2.0f
    );
    
    // Normalize twist
    float len = std::sqrt(twist.w * twist.w + d * d / 4.0f);
    if (len > 0.00001f) {
        twist = twist * (1.0f / len);
    } else {
        twist = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    }
    
    // Get swing as swing = q * twist^-1
    glm::quat swing = qn * glm::inverse(twist);
    return {swing, twist};
}

// Helper to constrain a ball-socket joint
void constrainBallSocket(Joint& joint, const Joint& parent) {
    // Get the current direction of this joint in parent space
    glm::vec3 currentDir = glm::normalize(joint.position - parent.position);
    
    // Get the rest direction in parent space
    glm::vec3 restDir = glm::vec3(0.0f, 1.0f, 0.0f);  // Always +Y in parent space
    
    // Calculate angle between rest and current direction
    float angle = std::acos(glm::clamp(glm::dot(restDir, currentDir), -1.0f, 1.0f));
    
    // If angle exceeds limit, clamp to cone surface
    if (angle > joint.maxSwingAngle) {
        // Create rotation axis perpendicular to rest direction
        glm::vec3 rotAxis = glm::normalize(glm::cross(restDir, currentDir));
        if (glm::length(rotAxis) < 0.001f) {
            // If vectors are parallel, use any perpendicular axis
            rotAxis = glm::normalize(glm::cross(restDir, glm::vec3(1.0f, 0.0f, 0.0f)));
        }
        
        // Create rotation to place vector on cone surface
        glm::quat clampRotation = glm::angleAxis(joint.maxSwingAngle, rotAxis);
        glm::vec3 clampedDir = glm::normalize(clampRotation * restDir);
        
        // Update joint position to respect constraint
        joint.position = parent.position + clampedDir * joint.length;
        
        // Update orientation to match clamped direction
        glm::vec3 parentForward = parent.orientation * glm::vec3(0.0f, 1.0f, 0.0f);
        joint.orientation = glm::rotation(parentForward, clampedDir) * parent.orientation;
    }
}

// Helper to clamp a direction to within a cone around a rotation axis
glm::vec3 clampToCone(const glm::vec3& direction, const glm::vec3& referenceDir, const glm::vec3& rotationAxis, float maxAngle) {
    // First, get the plane perpendicular to the rotation axis
    glm::vec3 normalizedAxis = glm::normalize(rotationAxis);
    
    // Project both vectors onto this plane
    glm::vec3 projectedRef = referenceDir - normalizedAxis * glm::dot(referenceDir, normalizedAxis);
    glm::vec3 projectedDir = direction - normalizedAxis * glm::dot(direction, normalizedAxis);
    
    float projRefLen = glm::length(projectedRef);
    float projDirLen = glm::length(projectedDir);
    
    // If either projection is too small, the vectors are nearly parallel to the axis
    if (projRefLen < 0.0001f || projDirLen < 0.0001f) {
        return referenceDir;
    }
    
    // Normalize projections
    projectedRef /= projRefLen;
    projectedDir /= projDirLen;
    
    // Calculate signed angle in the rotation plane
    float cosAngle = glm::dot(projectedRef, projectedDir);
    float angle = std::acos(glm::clamp(cosAngle, -1.0f, 1.0f));
    
    // Get sign using cross product
    if (glm::dot(glm::cross(projectedRef, projectedDir), normalizedAxis) < 0) {
        angle = -angle;
    }
    
    // If angle exceeds limit, clamp it
    if (std::abs(angle) > maxAngle) {
        // Create rotation that puts vector on cone surface
        float clampedAngle = glm::sign(angle) * maxAngle;
        glm::quat rotation = glm::angleAxis(clampedAngle, normalizedAxis);
        return glm::normalize(rotation * referenceDir);
    }
    
    return direction;
}

// Find nearest point on cone to target
glm::vec3 findNearestPointOnCone(const glm::vec3& joint, const glm::vec3& parent, const glm::vec3& target, float maxAngle) {
    // Project target onto joint-parent line
    glm::vec3 axis = glm::normalize(joint - parent);
    glm::vec3 toTarget = target - joint;
    glm::vec3 projection = joint + axis * glm::dot(toTarget, axis);
    
    // If target is on or very close to the axis, return the point on the cone in any direction
    if (glm::distance(target, projection) < 0.0001f) {
        // Choose any perpendicular direction
        glm::vec3 perpDir = glm::normalize(glm::cross(axis, glm::vec3(0.0f, 1.0f, 0.0f)));
        if (glm::length(perpDir) < 0.0001f) {
            perpDir = glm::normalize(glm::cross(axis, glm::vec3(1.0f, 0.0f, 0.0f)));
        }
        return joint + (axis * std::cos(maxAngle) + perpDir * std::sin(maxAngle)) * glm::length(toTarget);
    }
    
    // Get vector from projection to target
    glm::vec3 toTargetPerp = target - projection;
    float perpDist = glm::length(toTargetPerp);
    float axialDist = glm::dot(toTarget, axis);
    
    // Calculate allowed radius at this axial distance
    float allowedRadius = std::abs(axialDist) * std::tan(maxAngle);
    
    // If within cone, return target
    if (perpDist <= allowedRadius) {
        return target;
    }
    
    // Otherwise, clamp to cone surface
    glm::vec3 perpNorm = toTargetPerp / perpDist;
    return projection + perpNorm * allowedRadius;
}

void IKChain::solve(int maxIterations, float tolerance) {
    if (joints_.empty()) return;
    
    // Store original positions for constraint checking
    std::vector<glm::vec3> originalPositions;
    originalPositions.reserve(joints_.size());
    for (const auto& joint : joints_) {
        originalPositions.push_back(joint.position);
    }
    
    // Check if target is reachable
    float totalLength = 0.0f;
    for (const auto& joint : joints_) {
        totalLength += joint.length;
    }
    
    float targetDistance = glm::distance(target_, joints_[0].position);
    if (targetDistance > totalLength) {
        // Target is unreachable - move joints to get as close as possible
        glm::vec3 direction = glm::normalize(target_ - joints_[0].position);
        
        // Start from base
        joints_[0].position = glm::vec3(0.0f, -0.6f, 0.0f);
        
        // Extend chain in target direction
        for (size_t i = 0; i < joints_.size() - 1; ++i) {
            joints_[i + 1].position = joints_[i].position + direction * joints_[i].length;
        }
        return;
    }
    
    float totalError = std::numeric_limits<float>::max();
    
    for (int iter = 0; iter < maxIterations; ++iter) {
        // --- Backward pass ---
        // Set end effector at target
        joints_.back().position = target_;
        
        // Work backwards from target to base
        for (int i = joints_.size() - 2; i >= 0; --i) {
            glm::vec3 direction = glm::normalize(joints_[i].position - joints_[i + 1].position);
            joints_[i].position = joints_[i + 1].position + direction * joints_[i].length;
        }
        
        // --- Forward pass ---
        // Fix base at origin
        joints_[0].position = glm::vec3(0.0f, -0.6f, 0.0f);
        
        // Work forwards from base to end effector
        for (size_t i = 0; i < joints_.size() - 1; ++i) {
            glm::vec3 direction = glm::normalize(joints_[i + 1].position - joints_[i].position);
            
            // For each joint, check if it violates its cone constraint
            if (i > 0) {  // Skip base joint
                // Get the reference direction (previous segment)
                glm::vec3 parentDir = glm::normalize(joints_[i].position - joints_[i-1].position);
                
                // Calculate angle between parent direction and current direction
                float angle = std::acos(glm::clamp(glm::dot(parentDir, direction), -1.0f, 1.0f));
                
                std::cout << "Joint " << i << " angle: " << glm::degrees(angle) 
                         << " (max: " << glm::degrees(joints_[i].maxSwingAngle) << ")" << std::endl;
                
                // If angle exceeds constraint, clamp it
                if (angle > joints_[i].maxSwingAngle) {
                    // Find rotation axis (perpendicular to both directions)
                    glm::vec3 rotationAxis = glm::normalize(glm::cross(parentDir, direction));
                    if (glm::length(rotationAxis) > 0.0001f) {
                        // Create rotation to place direction on cone surface
                        glm::quat rotation = glm::angleAxis(joints_[i].maxSwingAngle, rotationAxis);
                        direction = glm::normalize(rotation * parentDir);
                    }
                }
            }
            
            // Update position while maintaining length
            joints_[i + 1].position = joints_[i].position + direction * joints_[i].length;
        }
        
        // Check if we've reached the target within tolerance
        float newError = glm::distance(joints_.back().position, target_);
        if (newError < tolerance || std::abs(newError - totalError) < tolerance * 0.1f) {
            break;
        }
        totalError = newError;
    }
    
    // Calculate final orientations
    for (size_t i = 0; i < joints_.size(); ++i) {
        glm::vec3 currentDir;
        if (i < joints_.size() - 1) {
            currentDir = glm::normalize(joints_[i + 1].position - joints_[i].position);
        } else {
            currentDir = glm::normalize(target_ - joints_[i].position);
        }
        
        glm::vec3 referenceDir = (i == 0) ? glm::vec3(0.0f, 1.0f, 0.0f) : 
            glm::normalize(joints_[i].position - joints_[i-1].position);
            
        joints_[i].orientation = glm::rotation(referenceDir, currentDir);
    }
}

void IKChain::applyConstraints() {}
void IKChain::backwardPass() {}
void IKChain::forwardPass() {}
void IKChain::calculateRotations() {}

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
        rotations.push_back(joint.orientation);
    }
    
    return rotations;
}

} // namespace ik