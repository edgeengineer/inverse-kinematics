#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif

/// A three-dimensional vector representing position, direction, or displacement in 3D space.
///
/// `Vector3D` is a fundamental mathematical type used throughout the inverse kinematics library
/// for representing positions, directions, joint axes, and geometric calculations. It provides
/// comprehensive vector operations optimized for robotics applications.
///
/// ## Overview
///
/// Vector3D supports all standard vector operations including addition, subtraction, scalar
/// multiplication, dot product, cross product, and normalization. The type is designed for
/// high-performance calculations while maintaining numerical stability.
///
/// ## Example Usage
///
/// ```swift
/// // Create position vectors
/// let origin = Vector3D.zero
/// let endEffectorPosition = Vector3D(x: 1.5, y: 2.0, z: 0.5)
///
/// // Calculate distance between points
/// let distance = origin.distance(to: endEffectorPosition)
///
/// // Create unit direction vectors
/// let forward = Vector3D.unitX
/// let up = Vector3D.unitZ
///
/// // Perform vector operations
/// let displacement = endEffectorPosition - origin
/// let normalizedDirection = displacement.normalized
/// let dotProduct = forward.dot(normalizedDirection)
/// ```
///
/// ## Performance Considerations
///
/// Vector3D operations are optimized for robotics calculations. For large-scale operations,
/// consider using the SIMD-optimized variants available through the `simd` property.
///
/// - Note: Vector3D uses Double precision for maximum accuracy in robotics calculations.
/// - Important: The type implements value semantics and is thread-safe (`Sendable`).
public struct Vector3D: Sendable {
    /// The x-coordinate component of the vector.
    ///
    /// In robotics applications, this typically represents:
    /// - The forward/backward position along the primary axis
    /// - The roll component in orientation vectors
    /// - The first joint axis component
    public var x: Double
    
    /// The y-coordinate component of the vector.
    ///
    /// In robotics applications, this typically represents:
    /// - The left/right position along the secondary axis
    /// - The pitch component in orientation vectors
    /// - The second joint axis component
    public var y: Double
    
    /// The z-coordinate component of the vector.
    ///
    /// In robotics applications, this typically represents:
    /// - The up/down position along the vertical axis
    /// - The yaw component in orientation vectors
    /// - The third joint axis component
    public var z: Double
    
    /// Creates a new 3D vector with the specified components.
    ///
    /// - Parameters:
    ///   - x: The x-coordinate component (default: 0.0)
    ///   - y: The y-coordinate component (default: 0.0)
    ///   - z: The z-coordinate component (default: 0.0)
    ///
    /// ## Example
    ///
    /// ```swift
    /// // Create specific position
    /// let position = Vector3D(x: 1.0, y: 2.0, z: 3.0)
    ///
    /// // Create with default values
    /// let origin = Vector3D() // (0, 0, 0)
    ///
    /// // Create 2D position (z = 0)
    /// let planarPosition = Vector3D(x: 1.0, y: 1.0)
    /// ```
    public init(x: Double = 0.0, y: Double = 0.0, z: Double = 0.0) {
        self.x = x
        self.y = y
        self.z = z
    }
    
    /// The zero vector (0, 0, 0).
    ///
    /// Commonly used to represent:
    /// - The origin point in coordinate systems
    /// - No displacement or motion
    /// - Default positions in kinematic chains
    ///
    /// ## Example
    ///
    /// ```swift
    /// let origin = Vector3D.zero
    /// let displacement = targetPosition - Vector3D.zero
    /// ```
    public static let zero = Vector3D(x: 0, y: 0, z: 0)
    
    /// The unit vector in the positive x-direction (1, 0, 0).
    ///
    /// In robotics coordinate systems, this typically represents:
    /// - The forward direction of a robot base
    /// - The primary axis for revolute joints
    /// - The reference direction for end-effector orientation
    ///
    /// ## Example
    ///
    /// ```swift
    /// let forwardDirection = Vector3D.unitX
    /// let joint = Joint(axis: Vector3D.unitX, type: .revolute)
    /// ```
    public static let unitX = Vector3D(x: 1, y: 0, z: 0)
    
    /// The unit vector in the positive y-direction (0, 1, 0).
    ///
    /// In robotics coordinate systems, this typically represents:
    /// - The left/right direction relative to robot base
    /// - The secondary axis for joint configurations
    /// - Lateral movement directions
    public static let unitY = Vector3D(x: 0, y: 1, z: 0)
    
    /// The unit vector in the positive z-direction (0, 0, 1).
    ///
    /// In robotics coordinate systems, this typically represents:
    /// - The upward direction (gravity-opposite)
    /// - The vertical axis for base rotation joints
    /// - The "up" reference for end-effector orientation
    ///
    /// ## Example
    ///
    /// ```swift
    /// let upDirection = Vector3D.unitZ
    /// let baseJoint = Joint(axis: Vector3D.unitZ, type: .revolute)
    /// ```
    public static let unitZ = Vector3D(x: 0, y: 0, z: 1)
}

extension Vector3D {
    /// The length (magnitude) of the vector.
    ///
    /// Calculates the Euclidean norm of the vector: √(x² + y² + z²).
    /// This is essential for determining distances, normalizing vectors,
    /// and calculating workspace boundaries in robotics applications.
    ///
    /// ## Example
    ///
    /// ```swift
    /// let position = Vector3D(x: 3.0, y: 4.0, z: 0.0)
    /// let distance = position.magnitude // 5.0
    ///
    /// // Check if end-effector is within reach
    /// let maxReach = 2.5
    /// if position.magnitude <= maxReach {
    ///     // Target is reachable
    /// }
    /// ```
    ///
    /// - Returns: The magnitude of the vector as a Double.
    /// - Complexity: O(1)
    public var magnitude: Double {
        sqrt(x * x + y * y + z * z)
    }
    
    /// The squared magnitude of the vector.
    ///
    /// Returns x² + y² + z² without computing the square root.
    /// This is more efficient when you only need to compare distances
    /// or when the actual magnitude value isn't required.
    ///
    /// ## Example
    ///
    /// ```swift
    /// let position1 = Vector3D(x: 1.0, y: 2.0, z: 3.0)
    /// let position2 = Vector3D(x: 4.0, y: 5.0, z: 6.0)
    ///
    /// // Efficient distance comparison
    /// let distanceSquared1 = position1.magnitudeSquared
    /// let distanceSquared2 = position2.magnitudeSquared
    /// let closer = distanceSquared1 < distanceSquared2 ? position1 : position2
    /// ```
    ///
    /// - Returns: The squared magnitude of the vector.
    /// - Complexity: O(1)
    public var magnitudeSquared: Double {
        x * x + y * y + z * z
    }
    
    /// A unit vector in the same direction as this vector.
    ///
    /// Returns a vector with magnitude 1.0 pointing in the same direction.
    /// If the vector has zero magnitude, returns the zero vector.
    /// Essential for creating joint axes and direction vectors in robotics.
    ///
    /// ## Example
    ///
    /// ```swift
    /// let direction = Vector3D(x: 3.0, y: 4.0, z: 0.0)
    /// let unitDirection = direction.normalized // (0.6, 0.8, 0.0)
    ///
    /// // Create joint with normalized axis
    /// let joint = Joint(axis: direction.normalized, type: .revolute)
    /// ```
    ///
    /// - Returns: A normalized copy of the vector, or zero vector if magnitude is zero.
    /// - Complexity: O(1)
    public var normalized: Vector3D {
        let mag = magnitude
        guard mag > 0 else { return Vector3D.zero }
        return Vector3D(x: x / mag, y: y / mag, z: z / mag)
    }
    
    /// Normalizes this vector in place.
    ///
    /// Modifies the vector to have unit magnitude while preserving direction.
    /// If the vector has zero magnitude, it remains unchanged.
    ///
    /// ## Example
    ///
    /// ```swift
    /// var axis = Vector3D(x: 2.0, y: 0.0, z: 0.0)
    /// axis.normalize() // Now (1.0, 0.0, 0.0)
    /// ```
    ///
    /// - Complexity: O(1)
    public mutating func normalize() {
        self = normalized
    }
    
    /// Calculates the Euclidean distance to another vector.
    ///
    /// Computes the straight-line distance between two points in 3D space.
    /// Commonly used for measuring end-effector positioning accuracy
    /// and workspace analysis in robotics applications.
    ///
    /// ## Example
    ///
    /// ```swift
    /// let currentPosition = Vector3D(x: 1.0, y: 1.0, z: 0.0)
    /// let targetPosition = Vector3D(x: 4.0, y: 5.0, z: 0.0)
    /// let error = currentPosition.distance(to: targetPosition) // 5.0
    ///
    /// // Check if close enough to target
    /// if error < 0.01 {
    ///     // Position achieved
    /// }
    /// ```
    ///
    /// - Parameter other: The target vector to measure distance to.
    /// - Returns: The distance between the two vectors.
    /// - Complexity: O(1)
    public func distance(to other: Vector3D) -> Double {
        (self - other).magnitude
    }
    
    /// Computes the dot product with another vector.
    ///
    /// Returns the scalar dot product: (x₁ × x₂) + (y₁ × y₂) + (z₁ × z₂).
    /// The dot product is used for angle calculations, projection operations,
    /// and determining perpendicularity in robotics applications.
    ///
    /// ## Example
    ///
    /// ```swift
    /// let forward = Vector3D.unitX
    /// let direction = Vector3D(x: 0.707, y: 0.707, z: 0.0)
    /// let dotProduct = forward.dot(direction) // 0.707
    ///
    /// // Calculate angle between vectors
    /// let angle = acos(dotProduct) // 45 degrees in radians
    /// ```
    ///
    /// - Parameter other: The vector to compute dot product with.
    /// - Returns: The scalar dot product.
    /// - Complexity: O(1)
    public func dot(_ other: Vector3D) -> Double {
        x * other.x + y * other.y + z * other.z
    }
    
    /// Computes the cross product with another vector.
    ///
    /// Returns a vector perpendicular to both input vectors, following
    /// the right-hand rule. The magnitude equals the area of the parallelogram
    /// formed by the two vectors. Essential for computing joint axes
    /// and rotation operations in robotics.
    ///
    /// ## Example
    ///
    /// ```swift
    /// let forward = Vector3D.unitX
    /// let right = Vector3D.unitY
    /// let up = forward.cross(right) // Vector3D.unitZ
    ///
    /// // Create perpendicular joint axis
    /// let link1Direction = Vector3D(x: 1.0, y: 1.0, z: 0.0)
    /// let link2Direction = Vector3D(x: -1.0, y: 1.0, z: 0.0)
    /// let jointAxis = link1Direction.cross(link2Direction).normalized
    /// ```
    ///
    /// - Parameter other: The vector to compute cross product with.
    /// - Returns: A vector perpendicular to both input vectors.
    /// - Complexity: O(1)
    public func cross(_ other: Vector3D) -> Vector3D {
        Vector3D(
            x: y * other.z - z * other.y,
            y: z * other.x - x * other.z,
            z: x * other.y - y * other.x
        )
    }
    
    /// Linearly interpolates between this vector and another.
    ///
    /// Performs linear interpolation: `self + t × (other - self)`.
    /// When t = 0, returns this vector; when t = 1, returns the other vector.
    /// Useful for smooth trajectory planning and animation in robotics.
    ///
    /// ## Example
    ///
    /// ```swift
    /// let start = Vector3D(x: 0.0, y: 0.0, z: 0.0)
    /// let end = Vector3D(x: 10.0, y: 0.0, z: 0.0)
    ///
    /// let quarter = start.lerp(to: end, t: 0.25) // (2.5, 0.0, 0.0)
    /// let halfway = start.lerp(to: end, t: 0.5)  // (5.0, 0.0, 0.0)
    /// let threeFourths = start.lerp(to: end, t: 0.75) // (7.5, 0.0, 0.0)
    /// ```
    ///
    /// - Parameters:
    ///   - other: The target vector to interpolate towards.
    ///   - t: The interpolation factor (0.0 to 1.0).
    /// - Returns: The interpolated vector.
    /// - Complexity: O(1)
    public func lerp(to other: Vector3D, t: Double) -> Vector3D {
        self + (other - self) * t
    }
}

extension Vector3D {
    public static func + (lhs: Vector3D, rhs: Vector3D) -> Vector3D {
        Vector3D(x: lhs.x + rhs.x, y: lhs.y + rhs.y, z: lhs.z + rhs.z)
    }
    
    public static func - (lhs: Vector3D, rhs: Vector3D) -> Vector3D {
        Vector3D(x: lhs.x - rhs.x, y: lhs.y - rhs.y, z: lhs.z - rhs.z)
    }
    
    public static prefix func - (vector: Vector3D) -> Vector3D {
        Vector3D(x: -vector.x, y: -vector.y, z: -vector.z)
    }
    
    public static func * (lhs: Vector3D, rhs: Double) -> Vector3D {
        Vector3D(x: lhs.x * rhs, y: lhs.y * rhs, z: lhs.z * rhs)
    }
    
    public static func * (lhs: Double, rhs: Vector3D) -> Vector3D {
        rhs * lhs
    }
    
    public static func / (lhs: Vector3D, rhs: Double) -> Vector3D {
        Vector3D(x: lhs.x / rhs, y: lhs.y / rhs, z: lhs.z / rhs)
    }
    
    public static func += (lhs: inout Vector3D, rhs: Vector3D) {
        lhs = lhs + rhs
    }
    
    public static func -= (lhs: inout Vector3D, rhs: Vector3D) {
        lhs = lhs - rhs
    }
    
    public static func *= (lhs: inout Vector3D, rhs: Double) {
        lhs = lhs * rhs
    }
    
    public static func /= (lhs: inout Vector3D, rhs: Double) {
        lhs = lhs / rhs
    }
}

extension Vector3D: Equatable {
    public static func == (lhs: Vector3D, rhs: Vector3D) -> Bool {
        abs(lhs.x - rhs.x) < Double.ulpOfOne &&
        abs(lhs.y - rhs.y) < Double.ulpOfOne &&
        abs(lhs.z - rhs.z) < Double.ulpOfOne
    }
}

extension Vector3D: CustomStringConvertible {
    public var description: String {
        "Vector3D(x: \(x), y: \(y), z: \(z))"
    }
}

extension Vector3D: Codable {}