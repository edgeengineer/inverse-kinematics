import Foundation

public struct Quaternion: Sendable {
    public var x: Double
    public var y: Double
    public var z: Double
    public var w: Double
    
    public init(x: Double = 0.0, y: Double = 0.0, z: Double = 0.0, w: Double = 1.0) {
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    }
    
    public init(axis: Vector3D, angle: Double) {
        let halfAngle = angle * 0.5
        let sinHalfAngle = sin(halfAngle)
        let normalizedAxis = axis.normalized
        
        self.x = normalizedAxis.x * sinHalfAngle
        self.y = normalizedAxis.y * sinHalfAngle
        self.z = normalizedAxis.z * sinHalfAngle
        self.w = cos(halfAngle)
    }
    
    public init(eulerAngles: Vector3D) {
        let cx = cos(eulerAngles.x * 0.5)
        let sx = sin(eulerAngles.x * 0.5)
        let cy = cos(eulerAngles.y * 0.5)
        let sy = sin(eulerAngles.y * 0.5)
        let cz = cos(eulerAngles.z * 0.5)
        let sz = sin(eulerAngles.z * 0.5)
        
        self.w = cx * cy * cz + sx * sy * sz
        self.x = sx * cy * cz - cx * sy * sz
        self.y = cx * sy * cz + sx * cy * sz
        self.z = cx * cy * sz - sx * sy * cz
    }
    
    public static let identity = Quaternion(x: 0, y: 0, z: 0, w: 1)
}

extension Quaternion {
    public var magnitude: Double {
        sqrt(x * x + y * y + z * z + w * w)
    }
    
    public var magnitudeSquared: Double {
        x * x + y * y + z * z + w * w
    }
    
    public var normalized: Quaternion {
        let mag = magnitude
        guard mag > 0 else { return Quaternion.identity }
        return Quaternion(x: x / mag, y: y / mag, z: z / mag, w: w / mag)
    }
    
    public mutating func normalize() {
        self = normalized
    }
    
    public var conjugate: Quaternion {
        Quaternion(x: -x, y: -y, z: -z, w: w)
    }
    
    public var inverse: Quaternion {
        let magSq = magnitudeSquared
        guard magSq > 0 else { return Quaternion.identity }
        let conj = conjugate
        return Quaternion(x: conj.x / magSq, y: conj.y / magSq, z: conj.z / magSq, w: conj.w / magSq)
    }
    
    public var eulerAngles: Vector3D {
        let test = x * y + z * w
        
        if test > 0.499 {
            return Vector3D(
                x: 2 * atan2(x, w),
                y: .pi / 2,
                z: 0
            )
        }
        
        if test < -0.499 {
            return Vector3D(
                x: -2 * atan2(x, w),
                y: -.pi / 2,
                z: 0
            )
        }
        
        let sqx = x * x
        let sqy = y * y
        let sqz = z * z
        
        return Vector3D(
            x: atan2(2 * y * w - 2 * x * z, 1 - 2 * sqy - 2 * sqz),
            y: asin(2 * test),
            z: atan2(2 * x * w - 2 * y * z, 1 - 2 * sqx - 2 * sqz)
        )
    }
    
    public func rotate(_ vector: Vector3D) -> Vector3D {
        let qv = Vector3D(x: x, y: y, z: z)
        let uv = qv.cross(vector)
        let uuv = qv.cross(uv)
        
        return vector + (uv * (2 * w)) + (uuv * 2)
    }
    
    public func slerp(to other: Quaternion, t: Double) -> Quaternion {
        let dot = self.dot(other)
        let threshold: Double = 0.9995
        
        if abs(dot) > threshold {
            return lerp(to: other, t: t).normalized
        }
        
        let theta0 = acos(abs(dot))
        let theta = theta0 * t
        
        let q2 = (other - self * dot).normalized
        
        return self * cos(theta) + q2 * sin(theta)
    }
    
    public func lerp(to other: Quaternion, t: Double) -> Quaternion {
        Quaternion(
            x: x + (other.x - x) * t,
            y: y + (other.y - y) * t,
            z: z + (other.z - z) * t,
            w: w + (other.w - w) * t
        )
    }
    
    public func dot(_ other: Quaternion) -> Double {
        x * other.x + y * other.y + z * other.z + w * other.w
    }
}

extension Quaternion {
    public static func * (lhs: Quaternion, rhs: Quaternion) -> Quaternion {
        Quaternion(
            x: lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
            y: lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
            z: lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
            w: lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z
        )
    }
    
    public static func * (lhs: Quaternion, rhs: Double) -> Quaternion {
        Quaternion(x: lhs.x * rhs, y: lhs.y * rhs, z: lhs.z * rhs, w: lhs.w * rhs)
    }
    
    public static func * (lhs: Double, rhs: Quaternion) -> Quaternion {
        rhs * lhs
    }
    
    public static func + (lhs: Quaternion, rhs: Quaternion) -> Quaternion {
        Quaternion(x: lhs.x + rhs.x, y: lhs.y + rhs.y, z: lhs.z + rhs.z, w: lhs.w + rhs.w)
    }
    
    public static func - (lhs: Quaternion, rhs: Quaternion) -> Quaternion {
        Quaternion(x: lhs.x - rhs.x, y: lhs.y - rhs.y, z: lhs.z - rhs.z, w: lhs.w - rhs.w)
    }
}

extension Quaternion: Equatable {
    public static func == (lhs: Quaternion, rhs: Quaternion) -> Bool {
        abs(lhs.x - rhs.x) < Double.ulpOfOne &&
        abs(lhs.y - rhs.y) < Double.ulpOfOne &&
        abs(lhs.z - rhs.z) < Double.ulpOfOne &&
        abs(lhs.w - rhs.w) < Double.ulpOfOne
    }
}

extension Quaternion: CustomStringConvertible {
    public var description: String {
        "Quaternion(x: \(x), y: \(y), z: \(z), w: \(w))"
    }
}

extension Quaternion: Codable {}