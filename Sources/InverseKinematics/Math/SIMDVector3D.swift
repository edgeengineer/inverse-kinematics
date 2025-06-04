#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif
import simd

/// SIMD-optimized version of Vector3D for performance-critical operations
public struct SIMDVector3D: Sendable {
    public var vector: SIMD3<Double>
    
    public init(x: Double = 0.0, y: Double = 0.0, z: Double = 0.0) {
        self.vector = SIMD3<Double>(x, y, z)
    }
    
    public init(_ simd: SIMD3<Double>) {
        self.vector = simd
    }
    
    // Component accessors
    public var x: Double {
        get { vector.x }
        set { vector.x = newValue }
    }
    
    public var y: Double {
        get { vector.y }
        set { vector.y = newValue }
    }
    
    public var z: Double {
        get { vector.z }
        set { vector.z = newValue }
    }
    
    // Static constants
    public static let zero = SIMDVector3D(x: 0, y: 0, z: 0)
    public static let unitX = SIMDVector3D(x: 1, y: 0, z: 0)
    public static let unitY = SIMDVector3D(x: 0, y: 1, z: 0)
    public static let unitZ = SIMDVector3D(x: 0, y: 0, z: 1)
}

extension SIMDVector3D {
    /// SIMD-optimized magnitude calculation
    public var magnitude: Double {
        simd_length(vector)
    }
    
    /// SIMD-optimized magnitude squared
    public var magnitudeSquared: Double {
        simd_length_squared(vector)
    }
    
    /// SIMD-optimized normalization
    public var normalized: SIMDVector3D {
        guard magnitudeSquared > 0 else { return SIMDVector3D.zero }
        return SIMDVector3D(simd_normalize(vector))
    }
    
    public mutating func normalize() {
        self = normalized
    }
    
    /// SIMD-optimized distance calculation
    public func distance(to other: SIMDVector3D) -> Double {
        simd_distance(vector, other.vector)
    }
    
    /// SIMD-optimized dot product
    public func dot(_ other: SIMDVector3D) -> Double {
        simd_dot(vector, other.vector)
    }
    
    /// SIMD-optimized cross product
    public func cross(_ other: SIMDVector3D) -> SIMDVector3D {
        SIMDVector3D(simd_cross(vector, other.vector))
    }
    
    /// SIMD-optimized linear interpolation
    public func lerp(to other: SIMDVector3D, t: Double) -> SIMDVector3D {
        SIMDVector3D((1.0 - t) * vector + t * other.vector)
    }
}

// MARK: - Arithmetic Operations
extension SIMDVector3D {
    public static func + (lhs: SIMDVector3D, rhs: SIMDVector3D) -> SIMDVector3D {
        SIMDVector3D(lhs.vector + rhs.vector)
    }
    
    public static func - (lhs: SIMDVector3D, rhs: SIMDVector3D) -> SIMDVector3D {
        SIMDVector3D(lhs.vector - rhs.vector)
    }
    
    public static prefix func - (vector: SIMDVector3D) -> SIMDVector3D {
        SIMDVector3D(-vector.vector)
    }
    
    public static func * (lhs: SIMDVector3D, rhs: Double) -> SIMDVector3D {
        SIMDVector3D(lhs.vector * rhs)
    }
    
    public static func * (lhs: Double, rhs: SIMDVector3D) -> SIMDVector3D {
        SIMDVector3D(lhs * rhs.vector)
    }
    
    public static func / (lhs: SIMDVector3D, rhs: Double) -> SIMDVector3D {
        SIMDVector3D(lhs.vector / rhs)
    }
    
    public static func += (lhs: inout SIMDVector3D, rhs: SIMDVector3D) {
        lhs.vector += rhs.vector
    }
    
    public static func -= (lhs: inout SIMDVector3D, rhs: SIMDVector3D) {
        lhs.vector -= rhs.vector
    }
    
    public static func *= (lhs: inout SIMDVector3D, rhs: Double) {
        lhs.vector *= rhs
    }
    
    public static func /= (lhs: inout SIMDVector3D, rhs: Double) {
        lhs.vector /= rhs
    }
}

// MARK: - Conformances
extension SIMDVector3D: Equatable {
    public static func == (lhs: SIMDVector3D, rhs: SIMDVector3D) -> Bool {
        simd_equal(lhs.vector, rhs.vector)
    }
}

extension SIMDVector3D: CustomStringConvertible {
    public var description: String {
        "SIMDVector3D(x: \(x), y: \(y), z: \(z))"
    }
}

extension SIMDVector3D: Codable {
    public init(from decoder: Decoder) throws {
        var container = try decoder.unkeyedContainer()
        let x = try container.decode(Double.self)
        let y = try container.decode(Double.self)
        let z = try container.decode(Double.self)
        self.init(x: x, y: y, z: z)
    }
    
    public func encode(to encoder: Encoder) throws {
        var container = encoder.unkeyedContainer()
        try container.encode(x)
        try container.encode(y)
        try container.encode(z)
    }
}

// MARK: - Conversion between Vector3D and SIMDVector3D
extension Vector3D {
    /// Convert to SIMD-optimized version
    public var simd: SIMDVector3D {
        SIMDVector3D(x: x, y: y, z: z)
    }
}

extension SIMDVector3D {
    /// Convert to standard Vector3D
    public var standard: Vector3D {
        Vector3D(x: x, y: y, z: z)
    }
}