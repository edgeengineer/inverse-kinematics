//
//  Vector3.swift
//  InverseKinematics
//
//  Cross-platform 3D vector implementation
//

#if canImport(FoundationEssentials)
import FoundationEssentials
#elseif canImport(Foundation)
import Foundation
#endif

/// A 3D vector structure for representing positions, directions, and rotations
public struct Vector3: Equatable, Hashable, Codable, Sendable {
    public var x: Float
    public var y: Float
    public var z: Float
    
    public init(x: Float = 0, y: Float = 0, z: Float = 0) {
        self.x = x
        self.y = y
        self.z = z
    }
    
    // MARK: - Static Vectors
    
    public static let zero = Vector3(x: 0, y: 0, z: 0)
    public static let one = Vector3(x: 1, y: 1, z: 1)
    public static let up = Vector3(x: 0, y: 1, z: 0)
    public static let down = Vector3(x: 0, y: -1, z: 0)
    public static let right = Vector3(x: 1, y: 0, z: 0)
    public static let left = Vector3(x: -1, y: 0, z: 0)
    public static let forward = Vector3(x: 0, y: 0, z: 1)
    public static let backward = Vector3(x: 0, y: 0, z: -1)
    
    // MARK: - Vector Operations
    
    /// Returns the length (magnitude) of the vector
    public var magnitude: Float {
        return sqrt(x * x + y * y + z * z)
    }
    
    /// Returns the squared length of the vector (more efficient than magnitude)
    public var squaredMagnitude: Float {
        return x * x + y * y + z * z
    }
    
    /// Returns a normalized copy of this vector (magnitude of 1)
    public var normalized: Vector3 {
        let mag = magnitude
        if mag > 0 {
            return Vector3(x: x / mag, y: y / mag, z: z / mag)
        }
        return .zero
    }
    
    /// Normalizes this vector in-place
    public mutating func normalize() {
        let mag = magnitude
        if mag > 0 {
            x /= mag
            y /= mag
            z /= mag
        }
    }
    
    /// Returns the dot product of this vector and another vector
    public func dot(_ other: Vector3) -> Float {
        return x * other.x + y * other.y + z * other.z
    }
    
    /// Returns the cross product of this vector and another vector
    public func cross(_ other: Vector3) -> Vector3 {
        return Vector3(
            x: y * other.z - z * other.y,
            y: z * other.x - x * other.z,
            z: x * other.y - y * other.x
        )
    }
    
    /// Returns the distance between this vector and another vector
    public func distance(to other: Vector3) -> Float {
        return (self - other).magnitude
    }
    
    /// Returns the squared distance between this vector and another vector
    public func squaredDistance(to other: Vector3) -> Float {
        return (self - other).squaredMagnitude
    }
    
    /// Linearly interpolates between this vector and another vector
    public func lerp(to target: Vector3, t: Float) -> Vector3 {
        let clampedT = max(0, min(1, t))
        return Vector3(
            x: x + (target.x - x) * clampedT,
            y: y + (target.y - y) * clampedT,
            z: z + (target.z - z) * clampedT
        )
    }
    
    /// Checks if this vector is approximately equal to another vector
    /// - Parameters:
    ///   - other: The other vector to compare with
    ///   - epsilon: The maximum difference allowed between components
    /// - Returns: True if the vectors are approximately equal
    public func isApproximately(_ other: Vector3, epsilon: Float = 1e-5) -> Bool {
        return abs(x - other.x) <= epsilon &&
               abs(y - other.y) <= epsilon &&
               abs(z - other.z) <= epsilon
    }
    
    // MARK: - Operators
    
    public static func + (lhs: Vector3, rhs: Vector3) -> Vector3 {
        return Vector3(x: lhs.x + rhs.x, y: lhs.y + rhs.y, z: lhs.z + rhs.z)
    }
    
    public static func - (lhs: Vector3, rhs: Vector3) -> Vector3 {
        return Vector3(x: lhs.x - rhs.x, y: lhs.y - rhs.y, z: lhs.z - rhs.z)
    }
    
    public static func * (lhs: Vector3, rhs: Float) -> Vector3 {
        return Vector3(x: lhs.x * rhs, y: lhs.y * rhs, z: lhs.z * rhs)
    }
    
    public static func * (lhs: Float, rhs: Vector3) -> Vector3 {
        return Vector3(x: lhs * rhs.x, y: lhs * rhs.y, z: lhs * rhs.z)
    }
    
    public static func / (lhs: Vector3, rhs: Float) -> Vector3 {
        return Vector3(x: lhs.x / rhs, y: lhs.y / rhs, z: lhs.z / rhs)
    }
    
    public static func += (lhs: inout Vector3, rhs: Vector3) {
        lhs.x += rhs.x
        lhs.y += rhs.y
        lhs.z += rhs.z
    }
    
    public static func -= (lhs: inout Vector3, rhs: Vector3) {
        lhs.x -= rhs.x
        lhs.y -= rhs.y
        lhs.z -= rhs.z
    }
    
    public static func *= (lhs: inout Vector3, rhs: Float) {
        lhs.x *= rhs
        lhs.y *= rhs
        lhs.z *= rhs
    }
    
    public static func /= (lhs: inout Vector3, rhs: Float) {
        lhs.x /= rhs
        lhs.y /= rhs
        lhs.z /= rhs
    }
    
    public static prefix func - (vector: Vector3) -> Vector3 {
        return Vector3(x: -vector.x, y: -vector.y, z: -vector.z)
    }
}

// MARK: - CustomStringConvertible
extension Vector3: CustomStringConvertible {
    public var description: String {
        return "Vector3(x: \(x), y: \(y), z: \(z))"
    }
}
