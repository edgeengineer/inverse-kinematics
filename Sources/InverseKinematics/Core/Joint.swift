//
//  Joint.swift
//  InverseKinematics
//
//  Core joint implementation for IK chains
//

#if canImport(FoundationEssentials)
import FoundationEssentials
#elseif canImport(Foundation)
import Foundation
#endif

/// Defines the type of joint in an IK chain
public enum JointType: Equatable, Hashable, Codable {
    /// Revolute joint that rotates around a single axis
    case revolute(axis: Vector3)
    
    /// Prismatic joint that slides along a single axis
    case prismatic(axis: Vector3)
    
    /// Spherical joint that can rotate freely in multiple axes
    case spherical
    
    /// Fixed joint that doesn't move (used for end effectors or base joints)
    case fixed
    
    /// Returns true if this is a spherical joint
    public var isSpherical: Bool {
        if case .spherical = self {
            return true
        }
        return false
    }
    
    /// Returns true if this is a revolute joint
    public var isRevolute: Bool {
        if case .revolute = self {
            return true
        }
        return false
    }
    
    /// Returns true if this is a prismatic joint
    public var isPrismatic: Bool {
        if case .prismatic = self {
            return true
        }
        return false
    }
    
    /// Returns true if this is a fixed joint
    public var isFixed: Bool {
        if case .fixed = self {
            return true
        }
        return false
    }
}

/// Defines constraints for a joint
public struct JointConstraint: Equatable, Hashable, Codable {
    /// Constraint type
    public enum ConstraintType: Equatable, Hashable, Codable {
        /// Angle limit for revolute joints (min and max in radians)
        case angleLimit(min: Float, max: Float)
        
        /// Distance limit for prismatic joints (min and max in units)
        case distanceLimit(min: Float, max: Float)
        
        /// Cone constraint for spherical joints (axis and angle in radians)
        case coneLimit(axis: Vector3, angle: Float)
    }
    
    /// The type of constraint
    public let type: ConstraintType
    
    /// Creates a new joint constraint
    public init(type: ConstraintType) {
        self.type = type
    }
    
    /// Applies the constraint to a joint value (angle or distance)
    public func apply(to value: Float, jointType: JointType) -> Float {
        switch (type, jointType) {
        case (.angleLimit(let min, let max), .revolute):
            return Swift.max(min, Swift.min(max, value))
            
        case (.distanceLimit(let min, let max), .prismatic):
            return Swift.max(min, Swift.min(max, value))
            
        default:
            return value
        }
    }
    
    /// Applies the constraint to a rotation quaternion for spherical joints
    public func apply(to rotation: Quaternion, originalRotation: Quaternion) -> Quaternion {
        switch type {
        case .coneLimit(let axis, let angle):
            // Get the rotation axis and angle
            let currentAxis = rotation.rotate(Vector3.forward)
            
            // Calculate the angle between the current axis and the constraint axis
            let currentAngle = acos(currentAxis.dot(axis).clamped(to: -1...1))
            
            // If within the cone, return the original rotation
            if currentAngle <= angle {
                return rotation
            }
            
            // Otherwise, clamp to the cone
            // Create a rotation that aligns the forward vector with the constraint axis
            let alignmentAxis = currentAxis.cross(axis).normalized
            if alignmentAxis.squaredMagnitude < 1e-6 {
                // If vectors are parallel or anti-parallel, use a perpendicular axis
                let perpendicularAxis = axis.perpendicular
                return Quaternion(axis: perpendicularAxis, angle: angle)
            }
            
            // Rotate by the maximum allowed angle
            return Quaternion(axis: alignmentAxis, angle: angle)
            
        default:
            return rotation
        }
    }
}

/// Represents a joint in an IK chain
public class Joint: Identifiable, Equatable, Hashable, Codable {
    /// Unique identifier for the joint
    public let id: UUID
    
    /// The type of joint
    public let type: JointType
    
    /// The local position of the joint relative to its parent
    public var localPosition: Vector3
    
    /// The local rotation of the joint relative to its parent
    public var localRotation: Quaternion
    
    /// The constraints applied to this joint
    public var constraints: [JointConstraint]
    
    /// The length of the joint (distance to the next joint)
    public var length: Float
    
    /// The parent joint in the chain (nil for root)
    public weak var parent: Joint?
    
    /// The child joints in the chain
    public var children: [Joint]
    
    /// The world position of the joint
    public var worldPosition: Vector3 {
        if let parent = parent {
            return parent.worldPosition + parent.worldRotation.rotate(localPosition)
        }
        return localPosition
    }
    
    /// The world rotation of the joint
    public var worldRotation: Quaternion {
        if let parent = parent {
            return parent.worldRotation * localRotation
        }
        return localRotation
    }
    
    /// The forward direction of the joint in world space
    public var forwardDirection: Vector3 {
        return worldRotation.rotate(Vector3.forward)
    }
    
    /// The up direction of the joint in world space
    public var upDirection: Vector3 {
        return worldRotation.rotate(Vector3.up)
    }
    
    /// The right direction of the joint in world space
    public var rightDirection: Vector3 {
        return worldRotation.rotate(Vector3.right)
    }
    
    /// The number of degrees of freedom for this joint
    public var degreesOfFreedom: Int {
        switch type {
        case .revolute, .prismatic:
            return 1
        case .spherical:
            return 3
        case .fixed:
            return 0
        }
    }
    
    /// Creates a new joint
    public init(
        type: JointType,
        localPosition: Vector3 = .zero,
        localRotation: Quaternion = .identity,
        constraints: [JointConstraint] = [],
        length: Float = 0
    ) {
        self.id = UUID()
        self.type = type
        self.localPosition = localPosition
        self.localRotation = localRotation
        self.constraints = constraints
        self.length = length
        self.parent = nil
        self.children = []
    }
    
    /// Adds a child joint to this joint
    @discardableResult
    public func addChild(_ joint: Joint) -> Joint {
        joint.parent = self
        children.append(joint)
        return joint
    }
    
    /// Removes a child joint from this joint
    public func removeChild(_ joint: Joint) {
        children.removeAll { $0 === joint }
        joint.parent = nil
    }
    
    /// Applies all constraints to the joint
    public func applyConstraints() {
        switch type {
        case .revolute(let axis):
            // For revolute joints, we need to extract the rotation around the axis
            // and apply angle limits
            let angle = localRotation.angle(around: axis)
            
            for constraint in constraints {
                if case .angleLimit(let min, let max) = constraint.type {
                    let constrainedAngle = Swift.max(min, Swift.min(max, angle))
                    if abs(constrainedAngle - angle) > 1e-6 {
                        // Reconstruct the rotation with the constrained angle
                        localRotation = Quaternion(axis: axis, angle: constrainedAngle)
                    }
                }
            }
            
        case .prismatic:
            // For prismatic joints, we apply distance limits to the local position
            for constraint in constraints {
                if case .distanceLimit(let min, let max) = constraint.type {
                    // Assuming the local position magnitude represents the distance
                    let currentDistance = localPosition.magnitude
                    let constrainedDistance = Swift.max(min, Swift.min(max, currentDistance))
                    
                    if abs(constrainedDistance - currentDistance) > 1e-6 {
                        // Scale the position to the constrained distance
                        localPosition = localPosition.normalized * constrainedDistance
                    }
                }
            }
            
        case .spherical:
            // For spherical joints, we apply cone constraints
            for constraint in constraints {
                if case .coneLimit(_, _) = constraint.type {
                    localRotation = constraint.apply(to: localRotation, originalRotation: localRotation)
                }
            }
            
        case .fixed:
            // Fixed joints don't move, so no constraints needed
            break
        }
    }
    
    // MARK: - Equatable & Hashable
    
    public static func == (lhs: Joint, rhs: Joint) -> Bool {
        return lhs.id == rhs.id
    }
    
    public func hash(into hasher: inout Hasher) {
        hasher.combine(id)
    }
    
    // MARK: - Codable
    
    private enum CodingKeys: String, CodingKey {
        case id, type, localPosition, localRotation, constraints, length, children
    }
    
    public required init(from decoder: Decoder) throws {
        let container = try decoder.container(keyedBy: CodingKeys.self)
        id = try container.decode(UUID.self, forKey: .id)
        type = try container.decode(JointType.self, forKey: .type)
        localPosition = try container.decode(Vector3.self, forKey: .localPosition)
        localRotation = try container.decode(Quaternion.self, forKey: .localRotation)
        constraints = try container.decode([JointConstraint].self, forKey: .constraints)
        length = try container.decode(Float.self, forKey: .length)
        children = try container.decode([Joint].self, forKey: .children)
        
        // Set parent references for children
        for child in children {
            child.parent = self
        }
    }
    
    public func encode(to encoder: Encoder) throws {
        var container = encoder.container(keyedBy: CodingKeys.self)
        try container.encode(id, forKey: .id)
        try container.encode(type, forKey: .type)
        try container.encode(localPosition, forKey: .localPosition)
        try container.encode(localRotation, forKey: .localRotation)
        try container.encode(constraints, forKey: .constraints)
        try container.encode(length, forKey: .length)
        try container.encode(children, forKey: .children)
    }
}

// MARK: - Helper Extensions

extension Quaternion {
    /// Extracts the rotation angle around a specific axis
    func angle(around axis: Vector3) -> Float {
        let normalizedAxis = axis.normalized
        
        // Convert quaternion to axis-angle representation
        let qAxis = Vector3(x: self.x, y: self.y, z: self.z)
        let qAngle = 2 * acos(self.w)
        
        // Project the quaternion axis onto the specified axis
        let projection = qAxis.dot(normalizedAxis)
        
        // The sign of the projection determines the direction
        let sign: Float = projection >= 0 ? 1.0 : -1.0
        
        return sign * qAngle
    }
}
