#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif

/// The type of motion a joint can perform in a robotic system.
///
/// Joint types define the fundamental motion capabilities of robotic joints,
/// determining how they contribute to the overall kinematic chain. Each type
/// represents a different constraint on the relative motion between connected links.
///
/// ## Overview
///
/// Understanding joint types is crucial for:
/// - Proper kinematic modeling
/// - Workspace analysis  
/// - Inverse kinematics solver selection
/// - Control system design
///
/// ## Example Usage
///
/// ```swift
/// // Common industrial robot joints
/// let baseJoint = Joint(id: "base", type: .revolute, axis: Vector3D.unitZ)
/// let shoulderJoint = Joint(id: "shoulder", type: .revolute, axis: Vector3D.unitY)
/// let prismaticActuator = Joint(id: "actuator", type: .prismatic, axis: Vector3D.unitX)
/// ```
public enum JointType: Sendable, Codable {
    /// A revolute joint that rotates around a fixed axis.
    ///
    /// Revolute joints are the most common type in robotic systems, providing
    /// rotational motion around a specified axis. They're used in robot arms,
    /// rotary actuators, and most articulated mechanisms.
    ///
    /// **Motion**: Single rotational degree of freedom
    /// **Parameter**: Angle in radians
    /// **Common Uses**: Robot arm joints, servo motors, rotary actuators
    case revolute
    
    /// A prismatic joint that translates along a fixed axis.
    ///
    /// Prismatic joints provide linear motion along a specified direction.
    /// They're commonly used in linear actuators, telescoping mechanisms,
    /// and Cartesian coordinate robots.
    ///
    /// **Motion**: Single translational degree of freedom
    /// **Parameter**: Distance in meters
    /// **Common Uses**: Linear actuators, Cartesian robots, telescoping arms
    case prismatic
    
    /// A spherical joint allowing rotation around any axis through the joint center.
    ///
    /// Spherical joints provide three rotational degrees of freedom, allowing
    /// unconstrained rotational motion. They're used for wrist mechanisms
    /// and universal joints in robotic systems.
    ///
    /// **Motion**: Three rotational degrees of freedom
    /// **Parameter**: Currently returns identity (future: Euler angles or quaternion)
    /// **Common Uses**: Robot wrists, universal joints, gimbal mechanisms
    case spherical
    
    /// A planar joint allowing motion within a fixed plane.
    ///
    /// Planar joints constrain motion to a single plane, providing two
    /// translational and one rotational degree of freedom within that plane.
    /// Used in specialized 2.5D robotic applications.
    ///
    /// **Motion**: Two translation + one rotation in plane
    /// **Parameter**: Currently returns identity (future: planar coordinates)
    /// **Common Uses**: 2.5D manipulation, specialized industrial applications
    case planar
    
    /// A cylindrical joint combining rotation and translation along the same axis.
    ///
    /// Cylindrical joints provide both rotational and translational motion
    /// along the same axis, like a screw mechanism. Currently implemented
    /// as rotation only.
    ///
    /// **Motion**: Rotation + translation along same axis
    /// **Parameter**: Currently rotation angle (future: coupled motion)
    /// **Common Uses**: Screw mechanisms, combined linear-rotary actuators
    case cylindrical
    
    /// A fixed joint that provides no motion.
    ///
    /// Fixed joints create rigid connections between links, effectively
    /// making them a single rigid body. Used for structural connections
    /// and when joints need to be temporarily locked.
    ///
    /// **Motion**: No degrees of freedom
    /// **Parameter**: Ignored (always returns identity transform)
    /// **Common Uses**: Rigid connections, structural elements, locked joints
    case fixed
}

/// Defines the motion limits for a robotic joint.
///
/// Joint limits are essential for safe robot operation, preventing mechanical
/// damage and ensuring the robot operates within its designed workspace.
/// They constrain joint values to physically realizable ranges.
///
/// ## Overview
///
/// Joint limits serve multiple purposes:
/// - **Safety**: Prevent mechanical damage from over-extension
/// - **Workspace Definition**: Define the reachable space for inverse kinematics
/// - **Control Constraints**: Provide bounds for optimization algorithms
/// - **Physical Modeling**: Represent real hardware limitations
///
/// ## Example Usage
///
/// ```swift
/// // Standard revolute joint limits (±180 degrees)
/// let standardLimits = JointLimits(min: -Double.pi, max: Double.pi)
///
/// // Limited range joint (±90 degrees)
/// let limitedRange = JointLimits(min: -Double.pi/2, max: Double.pi/2)
///
/// // Prismatic joint limits (0 to 50cm extension)
/// let linearLimits = JointLimits(min: 0.0, max: 0.5)
///
/// // Check if value is within limits
/// if limits.contains(jointAngle) {
///     // Safe to move to this position
/// }
/// ```
public struct JointLimits: Sendable, Codable {
    /// The minimum allowed joint value.
    ///
    /// For revolute joints, this is the minimum angle in radians.
    /// For prismatic joints, this is the minimum extension distance in meters.
    /// Must be less than or equal to the maximum value.
    public let min: Double
    
    /// The maximum allowed joint value.
    ///
    /// For revolute joints, this is the maximum angle in radians.
    /// For prismatic joints, this is the maximum extension distance in meters.
    /// Must be greater than or equal to the minimum value.
    public let max: Double
    
    /// Creates joint limits with the specified minimum and maximum values.
    ///
    /// - Parameters:
    ///   - min: The minimum allowed joint value
    ///   - max: The maximum allowed joint value
    ///
    /// - Precondition: `min` must be less than or equal to `max`
    ///
    /// ## Example
    ///
    /// ```swift
    /// // Revolute joint with ±180° range
    /// let revoluteLimits = JointLimits(min: -Double.pi, max: Double.pi)
    ///
    /// // Prismatic joint with 0-30cm range
    /// let prismaticLimits = JointLimits(min: 0.0, max: 0.3)
    /// ```
    public init(min: Double, max: Double) {
        precondition(min <= max, "Joint limit minimum must be less than or equal to maximum")
        self.min = min
        self.max = max
    }
    
    /// Unlimited joint limits allowing any value.
    ///
    /// Represents joints with no physical constraints, useful for:
    /// - Continuous rotation joints (like wheels)
    /// - Virtual joints in kinematic modeling
    /// - Theoretical analysis without physical constraints
    ///
    /// ## Example
    ///
    /// ```swift
    /// let wheelJoint = Joint(
    ///     id: "wheel", 
    ///     type: .revolute, 
    ///     limits: .unlimited
    /// )
    /// ```
    public static let unlimited = JointLimits(min: -.infinity, max: .infinity)
    
    /// Clamps a value to be within the joint limits.
    ///
    /// If the value is below the minimum, returns the minimum.
    /// If the value is above the maximum, returns the maximum.
    /// Otherwise, returns the value unchanged.
    ///
    /// - Parameter value: The value to clamp
    /// - Returns: The clamped value within [min, max]
    ///
    /// ## Example
    ///
    /// ```swift
    /// let limits = JointLimits(min: -1.0, max: 1.0)
    /// let clamped1 = limits.clamp(-2.0) // Returns -1.0
    /// let clamped2 = limits.clamp(0.5)  // Returns 0.5
    /// let clamped3 = limits.clamp(2.0)  // Returns 1.0
    /// ```
    public func clamp(_ value: Double) -> Double {
        Swift.max(min, Swift.min(max, value))
    }
    
    /// Checks if a value is within the joint limits.
    ///
    /// - Parameter value: The value to check
    /// - Returns: `true` if the value is within [min, max], `false` otherwise
    ///
    /// ## Example
    ///
    /// ```swift
    /// let limits = JointLimits(min: -1.0, max: 1.0)
    /// limits.contains(0.5)   // true
    /// limits.contains(-1.5)  // false
    /// limits.contains(1.0)   // true (boundaries are inclusive)
    /// ```
    public func contains(_ value: Double) -> Bool {
        value >= min && value <= max
    }
}

/// Represents a robotic joint in a kinematic chain.
///
/// A joint defines the connection between two links in a robotic system,
/// specifying the type of motion, motion limits, and current configuration.
/// Joints are the fundamental building blocks of kinematic chains used in
/// inverse kinematics calculations.
///
/// ## Overview
///
/// Joints encapsulate several key aspects of robotic motion:
/// - **Motion Type**: Defines how the joint can move (revolute, prismatic, etc.)
/// - **Motion Axis**: Specifies the axis of rotation or translation
/// - **Limits**: Constrains the joint to safe operating ranges
/// - **Current State**: Tracks the current joint position/angle
/// - **Coordinate Frame**: Defines the joint's position in the kinematic chain
///
/// ## Example Usage
///
/// ```swift
/// // Create a typical robot arm base joint
/// let baseJoint = Joint(
///     id: "base_rotation",
///     type: .revolute,
///     axis: Vector3D.unitZ,
///     limits: JointLimits(min: -Double.pi, max: Double.pi),
///     value: 0.0
/// )
///
/// // Create a shoulder joint with parent transform
/// let shoulderJoint = Joint(
///     id: "shoulder",
///     type: .revolute,
///     axis: Vector3D.unitY,
///     limits: JointLimits(min: -Double.pi/2, max: Double.pi/2),
///     parentTransform: Transform(position: Vector3D(x: 0.0, y: 0.0, z: 0.1))
/// )
///
/// // Create a linear actuator
/// let actuator = Joint(
///     id: "linear_actuator",
///     type: .prismatic,
///     axis: Vector3D.unitX,
///     limits: JointLimits(min: 0.0, max: 0.5)
/// )
/// ```
///
/// ## Coordinate Frames
///
/// Each joint defines a local coordinate frame transformation that contributes
/// to the overall kinematic chain. The `parentTransform` positions the joint
/// relative to the previous link, while the joint's `transform` property
/// provides the motion-dependent transformation.
///
/// ## Thread Safety
///
/// Joint conforms to `Sendable` and can be safely used across concurrent contexts.
/// However, be careful when mutating the `value` property in concurrent code.
public struct Joint: Sendable, Codable {
    /// Unique identifier for the joint within a kinematic chain.
    ///
    /// Used for referencing specific joints in inverse kinematics solutions,
    /// debugging, and kinematic chain management. Should be unique within
    /// a single kinematic chain.
    ///
    /// ## Example
    ///
    /// ```swift
    /// let joint = Joint(id: "shoulder_pitch", type: .revolute)
    /// print("Joint ID: \(joint.id)") // "shoulder_pitch"
    /// ```
    public let id: String
    
    /// The type of motion this joint provides.
    ///
    /// Determines how the joint contributes to the kinematic chain and
    /// what type of motion parameter (angle or distance) it accepts.
    /// This affects forward kinematics calculations and solver behavior.
    ///
    /// See ``JointType`` for detailed descriptions of each type.
    public let type: JointType
    
    /// The axis of motion for the joint.
    ///
    /// For revolute joints, this is the axis of rotation.
    /// For prismatic joints, this is the direction of translation.
    /// The axis is automatically normalized during initialization.
    ///
    /// ## Example
    ///
    /// ```swift
    /// // Vertical rotation axis (typical base joint)
    /// let baseJoint = Joint(id: "base", type: .revolute, axis: Vector3D.unitZ)
    ///
    /// // Horizontal motion axis (linear actuator)
    /// let actuator = Joint(id: "actuator", type: .prismatic, axis: Vector3D.unitX)
    /// ```
    public let axis: Vector3D
    
    /// The motion limits for this joint.
    ///
    /// Constrains the joint value to safe operating ranges, preventing
    /// mechanical damage and defining the workspace boundaries for
    /// inverse kinematics calculations.
    ///
    /// See ``JointLimits`` for detailed usage.
    public let limits: JointLimits
    
    /// The current joint value (angle for revolute, distance for prismatic).
    ///
    /// This value is automatically clamped to the joint limits when set.
    /// For revolute joints, the value is in radians. For prismatic joints,
    /// the value is in meters.
    ///
    /// ## Example
    ///
    /// ```swift
    /// var joint = Joint(id: "elbow", type: .revolute, limits: JointLimits(min: 0, max: Double.pi))
    /// joint.setValue(Double.pi / 2) // 90 degrees
    /// print(joint.value) // π/2
    /// ```
    public var value: Double
    
    /// The transform from the parent link to this joint's coordinate frame.
    ///
    /// This transform positions the joint relative to the previous link in
    /// the kinematic chain. It represents the fixed geometric relationship
    /// that doesn't change with joint motion.
    ///
    /// ## Example
    ///
    /// ```swift
    /// // Joint positioned 10cm forward from parent link
    /// let joint = Joint(
    ///     id: "offset_joint",
    ///     type: .revolute,
    ///     parentTransform: Transform(position: Vector3D(x: 0.1, y: 0.0, z: 0.0))
    /// )
    /// ```
    public let parentTransform: Transform
    
    /// Creates a new joint with the specified parameters.
    ///
    /// - Parameters:
    ///   - id: Unique identifier for the joint
    ///   - type: The type of motion the joint provides
    ///   - axis: The axis of motion (default: Vector3D.unitZ)
    ///   - limits: Motion limits (default: unlimited)
    ///   - value: Initial joint value (default: 0.0)
    ///   - parentTransform: Transform from parent link (default: identity)
    ///
    /// The axis is automatically normalized, and the initial value is clamped
    /// to the specified limits.
    ///
    /// ## Example
    ///
    /// ```swift
    /// // Minimal joint creation
    /// let simpleJoint = Joint(id: "joint1", type: .revolute)
    ///
    /// // Fully specified joint
    /// let complexJoint = Joint(
    ///     id: "shoulder_roll",
    ///     type: .revolute,
    ///     axis: Vector3D.unitX,
    ///     limits: JointLimits(min: -Double.pi/2, max: Double.pi/2),
    ///     value: 0.0,
    ///     parentTransform: Transform(position: Vector3D(x: 0.0, y: 0.0, z: 0.3))
    /// )
    /// ```
    public init(
        id: String,
        type: JointType,
        axis: Vector3D = .unitZ,
        limits: JointLimits = .unlimited,
        value: Double = 0.0,
        parentTransform: Transform = .identity
    ) {
        self.id = id
        self.type = type
        self.axis = axis.normalized
        self.limits = limits
        self.value = limits.clamp(value)
        self.parentTransform = parentTransform
    }
    
    /// The transformation produced by this joint's current motion.
    ///
    /// Calculates the transform representing the joint's contribution to the
    /// kinematic chain based on its current value. This is the motion-dependent
    /// transformation that changes as the joint moves.
    ///
    /// ## Transform Types by Joint Type
    ///
    /// - **Revolute**: Rotation around the joint axis by the current angle
    /// - **Prismatic**: Translation along the joint axis by the current distance
    /// - **Fixed**: Identity transform (no motion)
    /// - **Other types**: Currently return identity (future implementation)
    ///
    /// ## Example
    ///
    /// ```swift
    /// let joint = Joint(id: "elbow", type: .revolute, axis: Vector3D.unitZ, value: Double.pi/2)
    /// let transform = joint.transform
    /// // Contains 90-degree rotation around Z-axis
    /// ```
    ///
    /// - Returns: The transform representing the joint's current motion
    public var transform: Transform {
        switch type {
        case .revolute:
            let rotation = Quaternion(axis: axis, angle: value)
            return Transform(position: Vector3D.zero, rotation: rotation)
            
        case .prismatic:
            let translation = axis * value
            return Transform(position: translation, rotation: Quaternion.identity)
            
        case .spherical:
            return Transform.identity
            
        case .planar:
            return Transform.identity
            
        case .cylindrical:
            let rotation = Quaternion(axis: axis, angle: value)
            return Transform(position: Vector3D.zero, rotation: rotation)
            
        case .fixed:
            return Transform.identity
        }
    }
    
    /// Sets the joint value, automatically clamping to limits.
    ///
    /// The new value is automatically constrained to be within the joint's
    /// motion limits. This ensures the joint never exceeds safe operating ranges.
    ///
    /// - Parameter newValue: The desired joint value
    ///
    /// ## Example
    ///
    /// ```swift
    /// var joint = Joint(id: "limited", type: .revolute, limits: JointLimits(min: -1.0, max: 1.0))
    /// joint.setValue(2.0)
    /// print(joint.value) // 1.0 (clamped to maximum)
    /// ```
    public mutating func setValue(_ newValue: Double) {
        value = limits.clamp(newValue)
    }
    
    /// Returns a copy of the joint with a new value.
    ///
    /// Creates a new joint instance with the specified value, automatically
    /// clamped to the joint's limits. This is useful for functional programming
    /// patterns and when you need to preserve the original joint.
    ///
    /// - Parameter newValue: The desired joint value
    /// - Returns: A new joint with the updated value
    ///
    /// ## Example
    ///
    /// ```swift
    /// let originalJoint = Joint(id: "elbow", type: .revolute)
    /// let movedJoint = originalJoint.withValue(Double.pi / 4)
    /// // originalJoint.value is still 0.0
    /// // movedJoint.value is π/4
    /// ```
    public func withValue(_ newValue: Double) -> Joint {
        var joint = self
        joint.setValue(newValue)
        return joint
    }
}

extension Joint: Equatable {
    public static func == (lhs: Joint, rhs: Joint) -> Bool {
        lhs.id == rhs.id &&
        lhs.type == rhs.type &&
        lhs.axis == rhs.axis &&
        lhs.limits.min == rhs.limits.min &&
        lhs.limits.max == rhs.limits.max &&
        abs(lhs.value - rhs.value) < Double.ulpOfOne &&
        lhs.parentTransform == rhs.parentTransform
    }
}

extension Joint: CustomStringConvertible {
    public var description: String {
        "Joint(id: \"\(id)\", type: \(type), value: \(value), limits: [\(limits.min), \(limits.max)])"
    }
}

extension Joint: Identifiable {}