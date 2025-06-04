#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif

public struct Transform: Sendable {
    public var position: Vector3D
    public var rotation: Quaternion
    
    public init(position: Vector3D = .zero, rotation: Quaternion = .identity) {
        self.position = position
        self.rotation = rotation
    }
    
    public init(position: Vector3D, eulerAngles: Vector3D) {
        self.position = position
        self.rotation = Quaternion(eulerAngles: eulerAngles)
    }
    
    public static let identity = Transform(position: .zero, rotation: .identity)
}

extension Transform {
    public var matrix: Matrix4x4 {
        Matrix4x4(translation: position, rotation: rotation)
    }
    
    public var inverse: Transform {
        let invRotation = rotation.inverse
        let invPosition = invRotation.rotate(-position)
        return Transform(position: invPosition, rotation: invRotation)
    }
    
    public func transformPoint(_ point: Vector3D) -> Vector3D {
        rotation.rotate(point) + position
    }
    
    public func transformDirection(_ direction: Vector3D) -> Vector3D {
        rotation.rotate(direction)
    }
    
    public func inverseTransformPoint(_ point: Vector3D) -> Vector3D {
        rotation.inverse.rotate(point - position)
    }
    
    public func inverseTransformDirection(_ direction: Vector3D) -> Vector3D {
        rotation.inverse.rotate(direction)
    }
    
    public func relativeTo(_ other: Transform) -> Transform {
        other.inverse * self
    }
    
    public func lerp(to other: Transform, t: Double) -> Transform {
        Transform(
            position: position.lerp(to: other.position, t: t),
            rotation: rotation.slerp(to: other.rotation, t: t)
        )
    }
}

extension Transform {
    public static func * (lhs: Transform, rhs: Transform) -> Transform {
        Transform(
            position: lhs.transformPoint(rhs.position),
            rotation: lhs.rotation * rhs.rotation
        )
    }
}

extension Transform: Equatable {
    public static func == (lhs: Transform, rhs: Transform) -> Bool {
        lhs.position == rhs.position && lhs.rotation == rhs.rotation
    }
}

extension Transform: CustomStringConvertible {
    public var description: String {
        "Transform(position: \(position), rotation: \(rotation))"
    }
}

extension Transform: Codable {}

public struct Matrix4x4: Sendable {
    public var elements: [Double]
    
    public init(elements: [Double] = Array(repeating: 0.0, count: 16)) {
        precondition(elements.count == 16, "Matrix4x4 requires exactly 16 elements")
        self.elements = elements
    }
    
    public init(translation: Vector3D = .zero, rotation: Quaternion = .identity, scale: Vector3D = Vector3D(x: 1, y: 1, z: 1)) {
        let qx = rotation.x
        let qy = rotation.y
        let qz = rotation.z
        let qw = rotation.w
        
        let x2 = qx + qx
        let y2 = qy + qy
        let z2 = qz + qz
        
        let xx = qx * x2
        let xy = qx * y2
        let xz = qx * z2
        let yy = qy * y2
        let yz = qy * z2
        let zz = qz * z2
        let wx = qw * x2
        let wy = qw * y2
        let wz = qw * z2
        
        self.elements = [
            (1 - (yy + zz)) * scale.x, (xy + wz) * scale.y, (xz - wy) * scale.z, translation.x,
            (xy - wz) * scale.x, (1 - (xx + zz)) * scale.y, (yz + wx) * scale.z, translation.y,
            (xz + wy) * scale.x, (yz - wx) * scale.y, (1 - (xx + yy)) * scale.z, translation.z,
            0, 0, 0, 1
        ]
    }
    
    public static let identity = Matrix4x4(elements: [
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    ])
    
    public subscript(row: Int, col: Int) -> Double {
        get {
            precondition(row >= 0 && row < 4 && col >= 0 && col < 4, "Matrix indices out of bounds")
            return elements[row * 4 + col]
        }
        set {
            precondition(row >= 0 && row < 4 && col >= 0 && col < 4, "Matrix indices out of bounds")
            elements[row * 4 + col] = newValue
        }
    }
    
    public var translation: Vector3D {
        Vector3D(x: elements[3], y: elements[7], z: elements[11])
    }
    
    public var rotation: Quaternion {
        let trace = elements[0] + elements[5] + elements[10]
        
        if trace > 0 {
            let s = sqrt(trace + 1.0) * 2
            return Quaternion(
                x: (elements[9] - elements[6]) / s,
                y: (elements[2] - elements[8]) / s,
                z: (elements[4] - elements[1]) / s,
                w: 0.25 * s
            )
        } else if elements[0] > elements[5] && elements[0] > elements[10] {
            let s = sqrt(1.0 + elements[0] - elements[5] - elements[10]) * 2
            return Quaternion(
                x: 0.25 * s,
                y: (elements[1] + elements[4]) / s,
                z: (elements[2] + elements[8]) / s,
                w: (elements[9] - elements[6]) / s
            )
        } else if elements[5] > elements[10] {
            let s = sqrt(1.0 + elements[5] - elements[0] - elements[10]) * 2
            return Quaternion(
                x: (elements[1] + elements[4]) / s,
                y: 0.25 * s,
                z: (elements[6] + elements[9]) / s,
                w: (elements[2] - elements[8]) / s
            )
        } else {
            let s = sqrt(1.0 + elements[10] - elements[0] - elements[5]) * 2
            return Quaternion(
                x: (elements[2] + elements[8]) / s,
                y: (elements[6] + elements[9]) / s,
                z: 0.25 * s,
                w: (elements[4] - elements[1]) / s
            )
        }
    }
    
    public func transformPoint(_ point: Vector3D) -> Vector3D {
        Vector3D(
            x: elements[0] * point.x + elements[1] * point.y + elements[2] * point.z + elements[3],
            y: elements[4] * point.x + elements[5] * point.y + elements[6] * point.z + elements[7],
            z: elements[8] * point.x + elements[9] * point.y + elements[10] * point.z + elements[11]
        )
    }
    
    public func transformDirection(_ direction: Vector3D) -> Vector3D {
        Vector3D(
            x: elements[0] * direction.x + elements[1] * direction.y + elements[2] * direction.z,
            y: elements[4] * direction.x + elements[5] * direction.y + elements[6] * direction.z,
            z: elements[8] * direction.x + elements[9] * direction.y + elements[10] * direction.z
        )
    }
}

extension Matrix4x4 {
    public static func * (lhs: Matrix4x4, rhs: Matrix4x4) -> Matrix4x4 {
        var result = Matrix4x4()
        
        for row in 0..<4 {
            for col in 0..<4 {
                result[row, col] = 
                    lhs[row, 0] * rhs[0, col] +
                    lhs[row, 1] * rhs[1, col] +
                    lhs[row, 2] * rhs[2, col] +
                    lhs[row, 3] * rhs[3, col]
            }
        }
        
        return result
    }
}

extension Matrix4x4: Equatable {
    public static func == (lhs: Matrix4x4, rhs: Matrix4x4) -> Bool {
        for i in 0..<16 {
            if abs(lhs.elements[i] - rhs.elements[i]) > Double.ulpOfOne {
                return false
            }
        }
        return true
    }
}

extension Matrix4x4: CustomStringConvertible {
    public var description: String {
        var result = "Matrix4x4(\n"
        for row in 0..<4 {
            result += "  ["
            for col in 0..<4 {
                result += String(format: "%8.3f", self[row, col])
                if col < 3 { result += ", " }
            }
            result += "]\n"
        }
        result += ")"
        return result
    }
}

extension Matrix4x4: Codable {}