//
//  IKChain.swift
//  InverseKinematics
//
//  Chain management for inverse kinematics
//

#if canImport(FoundationEssentials)
import FoundationEssentials
#elseif canImport(Foundation)
import Foundation
#endif

/// Represents a goal for an IK chain's end effector
public struct IKGoal: Equatable, Hashable, Codable {
    /// The target position for the end effector
    public var position: Vector3?
    
    /// The target orientation for the end effector
    public var orientation: Quaternion?
    
    /// The weight of the position goal (0-1)
    public var positionWeight: Float
    
    /// The weight of the orientation goal (0-1)
    public var orientationWeight: Float
    
    /// Creates a new IK goal
    public init(
        position: Vector3? = nil,
        orientation: Quaternion? = nil,
        positionWeight: Float = 1.0,
        orientationWeight: Float = 1.0
    ) {
        self.position = position
        self.orientation = orientation
        self.positionWeight = positionWeight.clamped(to: 0...1)
        self.orientationWeight = orientationWeight.clamped(to: 0...1)
    }
    
    /// Creates a position-only goal
    public static func position(_ position: Vector3, weight: Float = 1.0) -> IKGoal {
        return IKGoal(position: position, orientation: nil, positionWeight: weight, orientationWeight: 0)
    }
    
    /// Creates an orientation-only goal
    public static func orientation(_ orientation: Quaternion, weight: Float = 1.0) -> IKGoal {
        return IKGoal(position: nil, orientation: orientation, positionWeight: 0, orientationWeight: weight)
    }
}

/// Represents a chain of joints for inverse kinematics
public class IKChain: Identifiable, Codable {
    /// Unique identifier for the chain
    public let id: UUID
    
    /// The root joint of the chain
    public private(set) var rootJoint: Joint?
    
    /// The end effector joint of the chain
    public private(set) var endEffector: Joint?
    
    /// The current goal for the end effector
    public var goal: IKGoal?
    
    /// The maximum number of iterations for solving
    public var maxIterations: Int
    
    /// The tolerance for position convergence
    public var positionTolerance: Float
    
    /// The tolerance for orientation convergence
    public var orientationTolerance: Float
    
    /// All joints in the chain (cached for performance)
    public private(set) var joints: [Joint]
    
    /// Creates a new IK chain
    public init(
        rootJoint: Joint? = nil,
        endEffector: Joint? = nil,
        maxIterations: Int = 20,
        positionTolerance: Float = 0.01,
        orientationTolerance: Float = 0.01
    ) {
        self.id = UUID()
        self.rootJoint = rootJoint
        self.endEffector = endEffector
        self.maxIterations = maxIterations
        self.positionTolerance = positionTolerance
        self.orientationTolerance = orientationTolerance
        self.joints = []
        
        if let root = rootJoint {
            updateJointCache(from: root)
        }
    }
    
    /// Sets the root joint of the chain
    public func setRootJoint(_ joint: Joint) {
        rootJoint = joint
        updateJointCache(from: joint)
    }
    
    /// Sets the end effector joint of the chain
    public func setEndEffector(_ joint: Joint) {
        // Ensure the joint is part of the chain
        if let root = rootJoint {
            let allJoints = collectJoints(from: root)
            if allJoints.contains(where: { $0 === joint }) {
                endEffector = joint
            } else {
                print("Warning: End effector must be part of the chain")
            }
        } else {
            endEffector = joint
        }
    }
    
    /// Sets the goal for the end effector
    public func setGoal(position: Vector3? = nil, orientation: Quaternion? = nil) {
        goal = IKGoal(position: position, orientation: orientation)
    }
    
    /// Updates the cached list of joints in the chain
    private func updateJointCache(from root: Joint) {
        joints = collectJoints(from: root)
    }
    
    /// Collects all joints in the hierarchy starting from a root joint
    private func collectJoints(from root: Joint) -> [Joint] {
        var result = [root]
        for child in root.children {
            result.append(contentsOf: collectJoints(from: child))
        }
        return result
    }
    
    /// Gets the path of joints from the root to the end effector
    public func getJointPath() -> [Joint] {
        guard let endEffector = endEffector else { return [] }
        
        var path: [Joint] = [endEffector]
        var current: Joint? = endEffector
        
        while let joint = current?.parent {
            path.insert(joint, at: 0)
            current = joint
            
            // Stop if we've reached the root joint
            if joint === rootJoint {
                break
            }
        }
        
        return path
    }
    
    /// Applies all constraints to all joints in the chain
    public func applyConstraints() {
        for joint in joints {
            joint.applyConstraints()
        }
    }
    
    /// Calculates the current error between the end effector and the goal
    public func calculateError() -> (positionError: Float, orientationError: Float) {
        guard let endEffector = endEffector, let goal = goal else {
            return (Float.infinity, Float.infinity)
        }
        
        var positionError: Float = 0
        var orientationError: Float = 0
        
        if let targetPosition = goal.position {
            positionError = endEffector.worldPosition.distance(to: targetPosition)
        }
        
        if let targetOrientation = goal.orientation {
            // Calculate the angular difference between current and target orientation
            let currentOrientation = endEffector.worldRotation
            let diff = currentOrientation.inverse * targetOrientation
            orientationError = 2 * acos(abs(diff.w).clamped(to: 0...1))
        }
        
        return (positionError, orientationError)
    }
    
    /// Checks if the current solution has converged to the goal within tolerance
    public func hasConverged() -> Bool {
        guard let endEffector = endEffector, let goal = goal else {
            return false
        }
        
        // Check position convergence if a position goal is set
        if let targetPosition = goal.position {
            let currentPosition = endEffector.worldPosition
            let positionError = currentPosition.distance(to: targetPosition)
            
            if positionError > positionTolerance {
                return false
            }
        }
        
        // Check orientation convergence if an orientation goal is set
        if let targetOrientation = goal.orientation {
            let currentOrientation = endEffector.worldRotation
            let diff = currentOrientation.inverse * targetOrientation
            let angle = 2 * acos(abs(diff.w).clamped(to: 0...1))
            
            if angle > orientationTolerance {
                return false
            }
        }
        
        // If we have no goals, we can't converge
        if goal.position == nil && goal.orientation == nil {
            return false
        }
        
        // If we've passed all checks, we've converged
        return true
    }
    
    // MARK: - Codable
    
    private enum CodingKeys: String, CodingKey {
        case id, rootJoint, endEffectorId, goal, maxIterations, positionTolerance, orientationTolerance
    }
    
    public required init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        id = try container.decode(UUID.self, forKey: .id)
        rootJoint = try container.decodeIfPresent(Joint.self, forKey: .rootJoint)
        goal = try container.decodeIfPresent(IKGoal.self, forKey: .goal)
        maxIterations = try container.decode(Int.self, forKey: .maxIterations)
        positionTolerance = try container.decode(Float.self, forKey: .positionTolerance)
        orientationTolerance = try container.decode(Float.self, forKey: .orientationTolerance)
        
        // Rebuild the joint cache
        joints = []
        if let root = rootJoint {
            updateJointCache(from: root)
            
            // Find the end effector by ID
            if let endEffectorId = try container.decodeIfPresent(UUID.self, forKey: .endEffectorId) {
                endEffector = joints.first { $0.id == endEffectorId }
            }
        }
    }
    
    public func encode(to encoder: Encoder) throws {
        var container = encoder.container(keyedBy: CodingKeys.self)
        try container.encode(id, forKey: .id)
        try container.encode(rootJoint, forKey: .rootJoint)
        try container.encode(endEffector?.id, forKey: .endEffectorId)
        try container.encode(goal, forKey: .goal)
        try container.encode(maxIterations, forKey: .maxIterations)
        try container.encode(positionTolerance, forKey: .positionTolerance)
        try container.encode(orientationTolerance, forKey: .orientationTolerance)
    }
}
