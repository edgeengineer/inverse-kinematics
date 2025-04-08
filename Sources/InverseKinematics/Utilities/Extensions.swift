//
//  Extensions.swift
//  InverseKinematics
//
//  Common extensions used throughout the library
//

#if canImport(FoundationEssentials)
import FoundationEssentials
#elseif canImport(Foundation)
import Foundation
#endif

// MARK: - Float Extensions

extension Float {
    /// Clamps a value between a minimum and maximum value
    public func clamped(to range: ClosedRange<Float>) -> Float {
        return max(range.lowerBound, min(range.upperBound, self))
    }
}

// MARK: - Vector3 Extensions

extension Vector3 {
    /// Returns the vector with each component clamped to the given range
    public func clamped(to range: ClosedRange<Float>) -> Vector3 {
        return Vector3(
            x: x.clamped(to: range),
            y: y.clamped(to: range),
            z: z.clamped(to: range)
        )
    }
    
    /// Returns a vector perpendicular to this vector
    public var perpendicular: Vector3 {
        // Find a non-parallel vector to create a perpendicular vector
        if abs(x) < abs(y) && abs(x) < abs(z) {
            return Vector3(x: 0, y: z, z: -y).normalized
        } else if abs(y) < abs(z) {
            return Vector3(x: z, y: 0, z: -x).normalized
        } else {
            return Vector3(x: y, y: -x, z: 0).normalized
        }
    }
}

// MARK: - Quaternion Extensions

extension Quaternion {
    /// Returns the shortest arc quaternion between two vectors
    public static func fromToRotation(from: Vector3, to: Vector3) -> Quaternion {
        let fromNormalized = from.normalized
        let toNormalized = to.normalized
        
        let dot = fromNormalized.dot(toNormalized)
        
        // If vectors are nearly parallel, return identity
        if dot > 0.999999 {
            return .identity
        }
        
        // If vectors are nearly opposite, rotate 180 degrees around any perpendicular axis
        if dot < -0.999999 {
            // Find a perpendicular vector to 'from'
            var axis = Vector3.right.cross(fromNormalized)
            if axis.magnitude < 0.000001 {
                axis = Vector3.up.cross(fromNormalized)
            }
            return Quaternion(axis: axis.normalized, angle: Float.pi)
        }
        
        // Otherwise, create the rotation quaternion
        let axis = fromNormalized.cross(toNormalized).normalized
        let angle = acos(dot)
        
        return Quaternion(axis: axis, angle: angle)
    }
}
