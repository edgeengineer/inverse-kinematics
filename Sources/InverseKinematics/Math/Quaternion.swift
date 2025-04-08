//
//  Quaternion.swift
//  InverseKinematics
//
//  Cross-platform quaternion implementation for rotations
//

#if canImport(FoundationEssentials)
import FoundationEssentials
#elseif canImport(Foundation)
import Foundation
#endif

/// A quaternion structure for representing rotations in 3D space
public struct Quaternion: Equatable, Hashable, Codable, Sendable {
    public var x: Float
    public var y: Float
    public var z: Float
    public var w: Float
    
    public init(x: Float = 0, y: Float = 0, z: Float = 0, w: Float = 1) {
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    }
    
    /// Creates a quaternion from an axis-angle representation
    public init(axis: Vector3, angle: Float) {
        let halfAngle = angle * 0.5
        let s = sin(halfAngle)
        
        x = axis.x * s
        y = axis.y * s
        z = axis.z * s
        w = cos(halfAngle)
    }
    
    /// Creates a quaternion from Euler angles (in radians)
    public init(eulerAngles: Vector3) {
        let cy = cos(eulerAngles.z * 0.5)
        let sy = sin(eulerAngles.z * 0.5)
        let cp = cos(eulerAngles.y * 0.5)
        let sp = sin(eulerAngles.y * 0.5)
        let cr = cos(eulerAngles.x * 0.5)
        let sr = sin(eulerAngles.x * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
    }
    
    // MARK: - Static Quaternions
    
    public static let identity = Quaternion(x: 0, y: 0, z: 0, w: 1)
    
    // MARK: - Quaternion Operations
    
    /// Returns the magnitude (length) of the quaternion
    public var magnitude: Float {
        return sqrt(x * x + y * y + z * z + w * w)
    }
    
    /// Returns a normalized copy of this quaternion
    public var normalized: Quaternion {
        let mag = magnitude
        if mag > 0 {
            return Quaternion(x: x / mag, y: y / mag, z: z / mag, w: w / mag)
        }
        return .identity
    }
    
    /// Normalizes this quaternion in-place
    public mutating func normalize() {
        let mag = magnitude
        if mag > 0 {
            x /= mag
            y /= mag
            z /= mag
            w /= mag
        }
    }
    
    /// Returns the conjugate of this quaternion
    public var conjugate: Quaternion {
        return Quaternion(x: -x, y: -y, z: -z, w: w)
    }
    
    /// Returns the inverse of this quaternion
    public var inverse: Quaternion {
        let magSquared = x * x + y * y + z * z + w * w
        if magSquared > 0 {
            let invMagSquared = 1.0 / magSquared
            return Quaternion(
                x: -x * invMagSquared,
                y: -y * invMagSquared,
                z: -z * invMagSquared,
                w: w * invMagSquared
            )
        }
        return .identity
    }
    
    /// Converts this quaternion to Euler angles (in radians)
    public var eulerAngles: Vector3 {
        // Roll (x-axis rotation)
        let sinr_cosp = 2 * (w * x + y * z)
        let cosr_cosp = 1 - 2 * (x * x + y * y)
        let roll = atan2(sinr_cosp, cosr_cosp)
        
        // Pitch (y-axis rotation)
        let sinp = 2 * (w * y - z * x)
        let pitch: Float
        if abs(sinp) >= 1 {
            pitch = copysign(Float.pi / 2, sinp) // Use 90 degrees if out of range
        } else {
            pitch = asin(sinp)
        }
        
        // Yaw (z-axis rotation)
        let siny_cosp = 2 * (w * z + x * y)
        let cosy_cosp = 1 - 2 * (y * y + z * z)
        let yaw = atan2(siny_cosp, cosy_cosp)
        
        return Vector3(x: roll, y: pitch, z: yaw)
    }
    
    /// Returns the dot product of this quaternion and another quaternion
    public func dot(_ other: Quaternion) -> Float {
        return x * other.x + y * other.y + z * other.z + w * other.w
    }
    
    /// Spherically interpolates between two quaternions
    public func slerp(to end: Quaternion, t: Float) -> Quaternion {
        let clampedT = max(0, min(1, t))
        
        var cosHalfTheta = dot(end)
        var endQ = end
        
        // If the dot product is negative, slerp won't take the shorter path
        // Fix by reversing one quaternion
        if cosHalfTheta < 0 {
            endQ = Quaternion(x: -end.x, y: -end.y, z: -end.z, w: -end.w)
            cosHalfTheta = -cosHalfTheta
        }
        
        // If the quaternions are very close, just use linear interpolation
        if cosHalfTheta > 0.9999 {
            return Quaternion(
                x: x + (endQ.x - x) * clampedT,
                y: y + (endQ.y - y) * clampedT,
                z: z + (endQ.z - z) * clampedT,
                w: w + (endQ.w - w) * clampedT
            ).normalized
        }
        
        // Calculate coefficients
        let halfTheta = acos(cosHalfTheta)
        let sinHalfTheta = sqrt(1.0 - cosHalfTheta * cosHalfTheta)
        
        // If theta = 180 degrees, result is not fully defined
        // We could rotate around any axis normal to start/end
        if abs(sinHalfTheta) < 0.001 {
            return Quaternion(
                x: (x + endQ.x) * 0.5,
                y: (y + endQ.y) * 0.5,
                z: (z + endQ.z) * 0.5,
                w: (w + endQ.w) * 0.5
            ).normalized
        }
        
        // Calculate final values
        let ratioA = sin((1 - clampedT) * halfTheta) / sinHalfTheta
        let ratioB = sin(clampedT * halfTheta) / sinHalfTheta
        
        return Quaternion(
            x: x * ratioA + endQ.x * ratioB,
            y: y * ratioA + endQ.y * ratioB,
            z: z * ratioA + endQ.z * ratioB,
            w: w * ratioA + endQ.w * ratioB
        )
    }
    
    /// Returns the angle between this quaternion and another quaternion in radians
    public func angle(to other: Quaternion) -> Float {
        // Calculate the dot product
        let dot = self.dot(other).clamped(to: -1...1)
        
        // Return the angle
        return acos(abs(dot)) * 2
    }
    
    /// Checks if this quaternion is approximately equal to another quaternion
    /// - Parameters:
    ///   - other: The other quaternion to compare with
    ///   - epsilon: The maximum difference allowed between components
    /// - Returns: True if the quaternions are approximately equal
    public func isApproximately(_ other: Quaternion, epsilon: Float = 1e-5) -> Bool {
        // Either the components are all close, or the negated components are all close
        // (because q and -q represent the same rotation)
        return (abs(x - other.x) <= epsilon &&
                abs(y - other.y) <= epsilon &&
                abs(z - other.z) <= epsilon &&
                abs(w - other.w) <= epsilon) ||
               (abs(x + other.x) <= epsilon &&
                abs(y + other.y) <= epsilon &&
                abs(z + other.z) <= epsilon &&
                abs(w + other.w) <= epsilon)
    }
    
    /// Returns the rotation angle around a specific axis
    public func angleAround(axis: Vector3) -> Float {
        // Project the quaternion onto the given axis
        let normalizedAxis = axis.normalized
        let projection = x * normalizedAxis.x + y * normalizedAxis.y + z * normalizedAxis.z
        
        // Calculate the angle
        let angle = 2 * atan2(projection, w)
        return angle
    }
    
    // MARK: - Operators
    
    /// Multiplies two quaternions (composition of rotations)
    public static func * (lhs: Quaternion, rhs: Quaternion) -> Quaternion {
        return Quaternion(
            x: lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
            y: lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
            z: lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
            w: lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z
        )
    }
    
    /// Rotates a vector by this quaternion
    public func rotate(_ vector: Vector3) -> Vector3 {
        let q = self.normalized
        
        // Convert vector to quaternion with w=0
        let vectorQ = Quaternion(x: vector.x, y: vector.y, z: vector.z, w: 0)
        
        // Apply rotation: q * v * q^-1
        let resultQ = q * vectorQ * q.conjugate
        
        return Vector3(x: resultQ.x, y: resultQ.y, z: resultQ.z)
    }
}

// MARK: - CustomStringConvertible
extension Quaternion: CustomStringConvertible {
    public var description: String {
        return "Quaternion(x: \(x), y: \(y), z: \(z), w: \(w))"
    }
}
