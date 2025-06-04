#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif
import simd

/// SIMD-optimized quaternion for performance-critical operations
public struct SIMDQuaternion: Sendable {
    public var vector: simd_quatd
    
    public init(x: Double = 0.0, y: Double = 0.0, z: Double = 0.0, w: Double = 1.0) {
        self.vector = simd_quatd(ix: x, iy: y, iz: z, r: w)
    }
    
    public init(axis: SIMDVector3D, angle: Double) {
        self.vector = simd_quatd(angle: angle, axis: axis.vector)
    }
    
    public init(eulerAngles: SIMDVector3D) {
        // Convert Euler angles to quaternion using SIMD
        let cx = cos(eulerAngles.x * 0.5)
        let sx = sin(eulerAngles.x * 0.5)
        let cy = cos(eulerAngles.y * 0.5)
        let sy = sin(eulerAngles.y * 0.5)
        let cz = cos(eulerAngles.z * 0.5)
        let sz = sin(eulerAngles.z * 0.5)
        
        let w = cx * cy * cz + sx * sy * sz
        let x = sx * cy * cz - cx * sy * sz
        let y = cx * sy * cz + sx * cy * sz
        let z = cx * cy * sz - sx * sy * cz
        
        self.vector = simd_quatd(ix: x, iy: y, iz: z, r: w)
    }
    
    public init(_ simd: simd_quatd) {
        self.vector = simd
    }
    
    // Component accessors
    public var x: Double {
        get { vector.imag.x }
        set { vector.imag.x = newValue }
    }
    
    public var y: Double {
        get { vector.imag.y }
        set { vector.imag.y = newValue }
    }
    
    public var z: Double {
        get { vector.imag.z }
        set { vector.imag.z = newValue }
    }
    
    public var w: Double {
        get { vector.real }
        set { vector.real = newValue }
    }
    
    public static let identity = SIMDQuaternion(x: 0, y: 0, z: 0, w: 1)
}

extension SIMDQuaternion {
    /// SIMD-optimized magnitude calculation
    public var magnitude: Double {
        simd_length(vector)
    }
    
    /// SIMD-optimized magnitude squared
    public var magnitudeSquared: Double {
        simd_dot(vector, vector)
    }
    
    /// SIMD-optimized normalization
    public var normalized: SIMDQuaternion {
        guard magnitudeSquared > 0 else { return SIMDQuaternion.identity }
        return SIMDQuaternion(simd_normalize(vector))
    }
    
    public mutating func normalize() {
        self = normalized
    }
    
    /// SIMD-optimized conjugate
    public var conjugate: SIMDQuaternion {
        SIMDQuaternion(simd_conjugate(vector))
    }
    
    /// SIMD-optimized inverse
    public var inverse: SIMDQuaternion {
        SIMDQuaternion(simd_inverse(vector))
    }
    
    /// Convert to Euler angles
    public var eulerAngles: SIMDVector3D {
        let test = x * y + z * w
        
        if test > 0.499 {
            return SIMDVector3D(
                x: 2 * atan2(x, w),
                y: .pi / 2,
                z: 0
            )
        }
        
        if test < -0.499 {
            return SIMDVector3D(
                x: -2 * atan2(x, w),
                y: -.pi / 2,
                z: 0
            )
        }
        
        let sqx = x * x
        let sqy = y * y
        let sqz = z * z
        
        return SIMDVector3D(
            x: atan2(2 * y * w - 2 * x * z, 1 - 2 * sqy - 2 * sqz),
            y: asin(2 * test),
            z: atan2(2 * x * w - 2 * y * z, 1 - 2 * sqx - 2 * sqz)
        )
    }
    
    /// SIMD-optimized vector rotation
    public func rotate(_ vector: SIMDVector3D) -> SIMDVector3D {
        SIMDVector3D(simd_act(self.vector, vector.vector))
    }
    
    /// SIMD-optimized spherical linear interpolation
    public func slerp(to other: SIMDQuaternion, t: Double) -> SIMDQuaternion {
        SIMDQuaternion(simd_slerp(vector, other.vector, t))
    }
    
    /// Linear interpolation fallback
    public func lerp(to other: SIMDQuaternion, t: Double) -> SIMDQuaternion {
        let result = (1.0 - t) * vector + t * other.vector
        return SIMDQuaternion(simd_normalize(result))
    }
    
    /// SIMD-optimized dot product
    public func dot(_ other: SIMDQuaternion) -> Double {
        simd_dot(vector, other.vector)
    }
}

// MARK: - Arithmetic Operations
extension SIMDQuaternion {
    public static func * (lhs: SIMDQuaternion, rhs: SIMDQuaternion) -> SIMDQuaternion {
        SIMDQuaternion(simd_mul(lhs.vector, rhs.vector))
    }
    
    public static func * (lhs: SIMDQuaternion, rhs: Double) -> SIMDQuaternion {
        SIMDQuaternion(lhs.vector * rhs)
    }
    
    public static func * (lhs: Double, rhs: SIMDQuaternion) -> SIMDQuaternion {
        SIMDQuaternion(lhs * rhs.vector)
    }
    
    public static func + (lhs: SIMDQuaternion, rhs: SIMDQuaternion) -> SIMDQuaternion {
        SIMDQuaternion(lhs.vector + rhs.vector)
    }
    
    public static func - (lhs: SIMDQuaternion, rhs: SIMDQuaternion) -> SIMDQuaternion {
        SIMDQuaternion(lhs.vector - rhs.vector)
    }
}

// MARK: - Conformances
extension SIMDQuaternion: Equatable {
    public static func == (lhs: SIMDQuaternion, rhs: SIMDQuaternion) -> Bool {
        abs(lhs.x - rhs.x) < Double.ulpOfOne &&
        abs(lhs.y - rhs.y) < Double.ulpOfOne &&
        abs(lhs.z - rhs.z) < Double.ulpOfOne &&
        abs(lhs.w - rhs.w) < Double.ulpOfOne
    }
}

extension SIMDQuaternion: CustomStringConvertible {
    public var description: String {
        "SIMDQuaternion(x: \(x), y: \(y), z: \(z), w: \(w))"
    }
}

extension SIMDQuaternion: Codable {
    public init(from decoder: Decoder) throws {
        var container = try decoder.unkeyedContainer()
        let x = try container.decode(Double.self)
        let y = try container.decode(Double.self)
        let z = try container.decode(Double.self)
        let w = try container.decode(Double.self)
        self.init(x: x, y: y, z: z, w: w)
    }
    
    public func encode(to encoder: Encoder) throws {
        var container = encoder.unkeyedContainer()
        try container.encode(x)
        try container.encode(y)
        try container.encode(z)
        try container.encode(w)
    }
}

// MARK: - Conversion between Quaternion and SIMDQuaternion
extension Quaternion {
    /// Convert to SIMD-optimized version
    public var simd: SIMDQuaternion {
        SIMDQuaternion(x: x, y: y, z: z, w: w)
    }
}

extension SIMDQuaternion {
    /// Convert to standard Quaternion
    public var standard: Quaternion {
        Quaternion(x: x, y: y, z: z, w: w)
    }
}