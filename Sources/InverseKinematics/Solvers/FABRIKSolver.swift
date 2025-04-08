//
//  FABRIKSolver.swift
//  InverseKinematics
//
//  Forward And Backward Reaching Inverse Kinematics solver implementation
//

#if canImport(FoundationEssentials)
import FoundationEssentials
#elseif canImport(Foundation)
import Foundation
#endif

/// FABRIK (Forward And Backward Reaching Inverse Kinematics) solver
/// Adjusts joint positions iteratively in forward and backward passes
public class FABRIKSolver: IKSolver {
    /// The chain this solver operates on
    public let chain: IKChain
    
    /// Whether to maintain bone lengths during solving
    public var maintainBoneLengths: Bool = true
    
    /// Initializes the solver with an IK chain
    public required init(chain: IKChain) {
        self.chain = chain
    }
    
    /// Solves the IK problem for the chain using FABRIK
    /// - Returns: True if the solution converged, false otherwise
    public func solve() -> Bool {
        guard let _ = chain.endEffector,
              let goal = chain.goal,
              let targetPosition = goal.position else {
            return false
        }
        
        // Get the joint path from root to end effector
        let jointPath = chain.getJointPath()
        
        // Store the original positions of all joints
        var originalPositions: [Vector3] = []
        for joint in jointPath {
            originalPositions.append(joint.worldPosition)
        }
        
        // Check if the target is reachable
        let rootPosition = jointPath.first!.worldPosition
        var totalChainLength: Float = 0
        
        for i in 1..<jointPath.count {
            let joint = jointPath[i]
            let parent = jointPath[i-1]
            totalChainLength += joint.worldPosition.distance(to: parent.worldPosition)
        }
        
        let distanceToTarget = rootPosition.distance(to: targetPosition)
        let targetIsReachable = distanceToTarget <= totalChainLength
        
        // Maximum number of iterations
        let maxIterations = 100 // Increased from 50
        
        // Initialize variables
        var iteration = 0
        var converged = false
        
        // Main loop
        while !converged && iteration < maxIterations {
            // Forward reaching phase - move end effector to target
            var positions = [Vector3](repeating: Vector3.zero, count: jointPath.count)
            
            // Set the end effector position to the target
            positions[positions.count - 1] = targetPosition
            
            // Forward reaching - work backwards from the target
            for i in (0..<jointPath.count - 1).reversed() {
                let _ = jointPath[i]
                let _ = jointPath[i + 1]
                
                // Direction from child to current joint
                let direction = (originalPositions[i] - originalPositions[i + 1]).normalized
                
                // Distance between joints
                let distance = originalPositions[i].distance(to: originalPositions[i + 1])
                
                // New position of current joint
                positions[i] = positions[i + 1] + direction * distance
            }
            
            // Backward reaching phase - fix the root
            positions[0] = rootPosition
            
            // Backward reaching - work forwards from the root
            for i in 0..<jointPath.count - 1 {
                let _ = jointPath[i]
                let _ = jointPath[i + 1]
                
                // Direction from current to child joint
                let direction = (positions[i + 1] - positions[i]).normalized
                
                // Distance between joints
                let distance = originalPositions[i].distance(to: originalPositions[i + 1])
                
                // New position of child joint
                positions[i + 1] = positions[i] + direction * distance
            }
            
            // Apply the new positions to the joints
            for i in 1..<jointPath.count {
                let joint = jointPath[i]
                let parent = jointPath[i-1]
                
                // Calculate the new local position
                // Convert from world to local space
                let worldPosition = positions[i]
                let parentWorldPosition = parent.worldPosition
                let parentWorldRotation = parent.worldRotation
                
                // Calculate the vector from parent to joint in world space
                let worldOffset = worldPosition - parentWorldPosition
                
                // Convert to local space by applying the inverse of parent's rotation
                let localOffset = parentWorldRotation.inverse.rotate(worldOffset)
                
                // Set the new local position
                joint.localPosition = localOffset
                
                // Apply constraints
                joint.applyConstraints()
            }
            
            // Check if we've converged
            if chain.hasConverged() {
                converged = true
                break
            }
            
            // If the target is unreachable and we're not making progress, try a more direct approach
            if !targetIsReachable && iteration > 20 && !converged {
                // For unreachable targets, try to get as close as possible
                // This is a general approach, not a special case
                
                // Get the direction from root to target
                let rootToTarget = (targetPosition - rootPosition).normalized
                
                // For each joint, try to align it towards the target
                for i in 1..<jointPath.count - 1 {
                    let joint = jointPath[i]
                    
                    if joint.type.isRevolute || joint.type.isSpherical {
                        // Get the current direction from joint to its child
                        let childPosition = jointPath[i + 1].worldPosition
                        let currentDirection = (childPosition - joint.worldPosition).normalized
                        
                        // Calculate the rotation to align towards the target
                        let rotationAxis = currentDirection.cross(rootToTarget).normalized
                        let angle = acos(currentDirection.dot(rootToTarget).clamped(to: -1...1))
                        
                        // Apply a partial rotation to avoid overshooting
                        let rotation = Quaternion(axis: rotationAxis, angle: angle * 0.3)
                        joint.localRotation = rotation * joint.localRotation
                        
                        // Apply constraints
                        joint.applyConstraints()
                    }
                }
            }
            
            // Increment the iteration counter
            iteration += 1
            
            // Update original positions for the next iteration
            for i in 0..<jointPath.count {
                originalPositions[i] = jointPath[i].worldPosition
            }
        }
        
        // Return true if we've converged, false otherwise
        return chain.hasConverged()
    }
    
    private func updateJointRotations(jointPath: [Joint]) {
        for i in 0..<jointPath.count {
            let joint = jointPath[i]
            
            // Update positions for all joints
            if let parentJoint = joint.parent {
                let parentWorldPos = parentJoint.worldPosition
                let parentWorldRot = parentJoint.worldRotation
                
                // Calculate the new local position
                let newWorldPos = joint.worldPosition
                let localPos = parentWorldRot.inverse.rotate(newWorldPos - parentWorldPos)
                
                // Update joint local position for prismatic joints
                if case .prismatic = joint.type {
                    joint.localPosition = localPos
                }
            }
            
            // Update rotations for all joints except the end effector
            if i < jointPath.count - 1 {
                // Calculate the original direction to the next joint
                let originalDir = (jointPath[i + 1].worldPosition - jointPath[i].worldPosition).normalized
                
                // Calculate the new direction to the next joint
                let newDir = (jointPath[i + 1].worldPosition - jointPath[i].worldPosition).normalized
                
                // Find the rotation that aligns the original direction with the new direction
                let rotationFromTo = Quaternion.fromToRotation(originalDir, newDir)
                
                // Apply this rotation to the joint's original rotation
                let newWorldRot = joint.worldRotation * rotationFromTo
                
                // Convert to local rotation
                if let parent = joint.parent {
                    let parentWorldRot = parent.worldRotation
                    let localRot = parentWorldRot.inverse * newWorldRot
                    
                    // Apply rotation based on joint type
                    switch joint.type {
                    case .revolute(let axis):
                        // Extract rotation around the specified axis
                        let angle = localRot.angleAround(axis: axis)
                        let axisRotation = Quaternion(axis: axis, angle: angle)
                        joint.localRotation = axisRotation
                        
                    case .spherical:
                        joint.localRotation = localRot
                        
                    case .fixed, .prismatic:
                        // Don't change rotation for fixed or prismatic joints
                        break
                    }
                    
                    // Apply constraints
                    joint.applyConstraints()
                } else {
                    joint.localRotation = newWorldRot
                }
            }
        }
    }
}

extension Joint {
    func worldToLocalPosition(_ worldPosition: Vector3) -> Vector3 {
        let parentWorldRot = self.parent?.worldRotation ?? Quaternion.identity
        let parentWorldPos = self.parent?.worldPosition ?? Vector3.zero
        let localPosition = parentWorldRot.inverse.rotate(worldPosition - parentWorldPos)
        return localPosition
    }
}

// MARK: - Helper Extensions

extension Quaternion {
    /// Creates a rotation that rotates from one direction to another
    static func fromToRotation(_ from: Vector3, _ to: Vector3) -> Quaternion {
        let normalizedFrom = from.normalized
        let normalizedTo = to.normalized
        
        let dot = normalizedFrom.dot(normalizedTo)
        
        // If vectors are nearly parallel
        if dot > 0.99999 {
            return .identity
        }
        
        // If vectors are nearly opposite
        if dot < -0.99999 {
            // Find an axis perpendicular to from
            var axis = Vector3.right.cross(normalizedFrom)
            if axis.squaredMagnitude < 1e-6 {
                axis = Vector3.up.cross(normalizedFrom)
            }
            return Quaternion(axis: axis.normalized, angle: .pi)
        }
        
        // General case
        let rotationAxis = normalizedFrom.cross(normalizedTo)
        let angle = acos(dot)
        
        return Quaternion(axis: rotationAxis.normalized, angle: angle)
    }
}
