#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif

public struct Vector3D: Sendable {
    public var x: Double
    public var y: Double
    public var z: Double
    
    public init(x: Double = 0.0, y: Double = 0.0, z: Double = 0.0) {
        self.x = x
        self.y = y
        self.z = z
    }
    
    public static let zero = Vector3D(x: 0, y: 0, z: 0)
    public static let unitX = Vector3D(x: 1, y: 0, z: 0)
    public static let unitY = Vector3D(x: 0, y: 1, z: 0)
    public static let unitZ = Vector3D(x: 0, y: 0, z: 1)
}

extension Vector3D {
    public var magnitude: Double {
        sqrt(x * x + y * y + z * z)
    }
    
    public var magnitudeSquared: Double {
        x * x + y * y + z * z
    }
    
    public var normalized: Vector3D {
        let mag = magnitude
        guard mag > 0 else { return Vector3D.zero }
        return Vector3D(x: x / mag, y: y / mag, z: z / mag)
    }
    
    public mutating func normalize() {
        self = normalized
    }
    
    public func distance(to other: Vector3D) -> Double {
        (self - other).magnitude
    }
    
    public func dot(_ other: Vector3D) -> Double {
        x * other.x + y * other.y + z * other.z
    }
    
    public func cross(_ other: Vector3D) -> Vector3D {
        Vector3D(
            x: y * other.z - z * other.y,
            y: z * other.x - x * other.z,
            z: x * other.y - y * other.x
        )
    }
    
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