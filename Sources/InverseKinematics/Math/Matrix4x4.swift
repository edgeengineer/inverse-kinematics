//
//  Matrix4x4.swift
//  InverseKinematics
//
//  Cross-platform 4x4 matrix implementation for transformations
//

#if canImport(FoundationEssentials)
import FoundationEssentials
#elseif canImport(Foundation)
import Foundation
#endif

/// A 4x4 matrix structure for representing transformations in 3D space
public struct Matrix4x4: Equatable, Hashable, Codable, Sendable {
    // Matrix is stored in column-major order (OpenGL style)
    // m[column][row]
    public var m: [[Float]]
    
    public init() {
        m = Array(repeating: Array(repeating: 0, count: 4), count: 4)
        m[0][0] = 1
        m[1][1] = 1
        m[2][2] = 1
        m[3][3] = 1
    }
    
    public init(m00: Float, m01: Float, m02: Float, m03: Float,
                m10: Float, m11: Float, m12: Float, m13: Float,
                m20: Float, m21: Float, m22: Float, m23: Float,
                m30: Float, m31: Float, m32: Float, m33: Float) {
        m = [
            [m00, m01, m02, m03],
            [m10, m11, m12, m13],
            [m20, m21, m22, m23],
            [m30, m31, m32, m33]
        ]
    }
    
    // MARK: - Static Matrices
    
    /// Returns an identity matrix
    public static var identity: Matrix4x4 {
        return Matrix4x4()
    }
    
    /// Creates a translation matrix
    public static func translation(_ vector: Vector3) -> Matrix4x4 {
        var matrix = Matrix4x4()
        matrix.m[3][0] = vector.x
        matrix.m[3][1] = vector.y
        matrix.m[3][2] = vector.z
        return matrix
    }
    
    /// Creates a scaling matrix
    public static func scaling(_ vector: Vector3) -> Matrix4x4 {
        var matrix = Matrix4x4()
        matrix.m[0][0] = vector.x
        matrix.m[1][1] = vector.y
        matrix.m[2][2] = vector.z
        return matrix
    }
    
    /// Creates a rotation matrix from a quaternion
    public static func rotation(_ quaternion: Quaternion) -> Matrix4x4 {
        let q = quaternion.normalized
        let xx = q.x * q.x
        let xy = q.x * q.y
        let xz = q.x * q.z
        let xw = q.x * q.w
        let yy = q.y * q.y
        let yz = q.y * q.z
        let yw = q.y * q.w
        let zz = q.z * q.z
        let zw = q.z * q.w
        
        return Matrix4x4(
            m00: 1 - 2 * (yy + zz), m01: 2 * (xy - zw),     m02: 2 * (xz + yw),     m03: 0,
            m10: 2 * (xy + zw),     m11: 1 - 2 * (xx + zz), m12: 2 * (yz - xw),     m13: 0,
            m20: 2 * (xz - yw),     m21: 2 * (yz + xw),     m22: 1 - 2 * (xx + yy), m23: 0,
            m30: 0,                 m31: 0,                 m32: 0,                 m33: 1
        )
    }
    
    /// Creates a rotation matrix around the X axis
    public static func rotationX(_ angle: Float) -> Matrix4x4 {
        let c = cos(angle)
        let s = sin(angle)
        
        var matrix = Matrix4x4()
        matrix.m[1][1] = c
        matrix.m[1][2] = s
        matrix.m[2][1] = -s
        matrix.m[2][2] = c
        return matrix
    }
    
    /// Creates a rotation matrix around the Y axis
    public static func rotationY(_ angle: Float) -> Matrix4x4 {
        let c = cos(angle)
        let s = sin(angle)
        
        var matrix = Matrix4x4()
        matrix.m[0][0] = c
        matrix.m[0][2] = -s
        matrix.m[2][0] = s
        matrix.m[2][2] = c
        return matrix
    }
    
    /// Creates a rotation matrix around the Z axis
    public static func rotationZ(_ angle: Float) -> Matrix4x4 {
        let c = cos(angle)
        let s = sin(angle)
        
        var matrix = Matrix4x4()
        matrix.m[0][0] = c
        matrix.m[0][1] = s
        matrix.m[1][0] = -s
        matrix.m[1][1] = c
        return matrix
    }
    
    // MARK: - Matrix Operations
    
    /// Returns the determinant of this matrix
    public var determinant: Float {
        let a = m[0][0] * (
            m[1][1] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) -
            m[1][2] * (m[2][1] * m[3][3] - m[2][3] * m[3][1]) +
            m[1][3] * (m[2][1] * m[3][2] - m[2][2] * m[3][1])
        )
        
        let b = m[0][1] * (
            m[1][0] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) -
            m[1][2] * (m[2][0] * m[3][3] - m[2][3] * m[3][0]) +
            m[1][3] * (m[2][0] * m[3][2] - m[2][2] * m[3][0])
        )
        
        let c = m[0][2] * (
            m[1][0] * (m[2][1] * m[3][3] - m[2][3] * m[3][1]) -
            m[1][1] * (m[2][0] * m[3][3] - m[2][3] * m[3][0]) +
            m[1][3] * (m[2][0] * m[3][1] - m[2][1] * m[3][0])
        )
        
        let d = m[0][3] * (
            m[1][0] * (m[2][1] * m[3][2] - m[2][2] * m[3][1]) -
            m[1][1] * (m[2][0] * m[3][2] - m[2][2] * m[3][0]) +
            m[1][2] * (m[2][0] * m[3][1] - m[2][1] * m[3][0])
        )
        
        return a - b + c - d
    }
    
    /// Returns the transpose of this matrix
    public var transpose: Matrix4x4 {
        return Matrix4x4(
            m00: m[0][0], m01: m[1][0], m02: m[2][0], m03: m[3][0],
            m10: m[0][1], m11: m[1][1], m12: m[2][1], m13: m[3][1],
            m20: m[0][2], m21: m[1][2], m22: m[2][2], m23: m[3][2],
            m30: m[0][3], m31: m[1][3], m32: m[2][3], m33: m[3][3]
        )
    }
    
    /// Returns the inverse of this matrix
    public var inverse: Matrix4x4 {
        // Special case for translation matrices (which are very common in IK)
        // A translation matrix has the form:
        // 1 0 0 0
        // 0 1 0 0
        // 0 0 1 0
        // tx ty tz 1
        if m[0][0] == 1 && m[1][1] == 1 && m[2][2] == 1 && m[3][3] == 1 &&
           m[0][1] == 0 && m[0][2] == 0 && m[0][3] == 0 &&
           m[1][0] == 0 && m[1][2] == 0 && m[1][3] == 0 &&
           m[2][0] == 0 && m[2][1] == 0 && m[2][3] == 0 &&
           m[3][0] != 0 && m[3][1] != 0 && m[3][2] != 0 {
            // For a pure translation matrix, the inverse is just negating the translation components
            var result = Matrix4x4.identity
            result.m[3][0] = -m[3][0]
            result.m[3][1] = -m[3][1]
            result.m[3][2] = -m[3][2]
            return result
        }
        
        let det = determinant
        
        // If determinant is zero, return identity
        if abs(det) < 1e-6 {
            return Matrix4x4.identity
        }
        
        let invDet = 1.0 / det
        
        // Calculate cofactors
        func cofactor(_ row: Int, _ col: Int) -> Float {
            var submatrix = [[Float]](repeating: [Float](repeating: 0, count: 3), count: 3)
            
            var r = 0
            for i in 0..<4 {
                if i == row { continue }
                var c = 0
                for j in 0..<4 {
                    if j == col { continue }
                    submatrix[r][c] = m[j][i]  // Note: transposed indexing
                    c += 1
                }
                r += 1
            }
            
            let subDet = submatrix[0][0] * (submatrix[1][1] * submatrix[2][2] - submatrix[1][2] * submatrix[2][1]) -
                         submatrix[0][1] * (submatrix[1][0] * submatrix[2][2] - submatrix[1][2] * submatrix[2][0]) +
                         submatrix[0][2] * (submatrix[1][0] * submatrix[2][1] - submatrix[1][1] * submatrix[2][0])
            
            return ((row + col) % 2 == 0 ? 1 : -1) * subDet
        }
        
        var result = Matrix4x4()
        for i in 0..<4 {
            for j in 0..<4 {
                result.m[j][i] = cofactor(i, j) * invDet
            }
        }
        
        return result
    }
    
    /// Transforms a point by this matrix
    public func transformPoint(_ point: Vector3) -> Vector3 {
        let x = m[0][0] * point.x + m[1][0] * point.y + m[2][0] * point.z + m[3][0]
        let y = m[0][1] * point.x + m[1][1] * point.y + m[2][1] * point.z + m[3][1]
        let z = m[0][2] * point.x + m[1][2] * point.y + m[2][2] * point.z + m[3][2]
        let w = m[0][3] * point.x + m[1][3] * point.y + m[2][3] * point.z + m[3][3]
        
        if abs(w) > 1e-6 {
            return Vector3(x: x / w, y: y / w, z: z / w)
        }
        return Vector3(x: x, y: y, z: z)
    }
    
    /// Transforms a direction by this matrix (ignores translation)
    public func transformDirection(_ direction: Vector3) -> Vector3 {
        // For a direction vector, we only apply the rotation part of the matrix
        // and we need to ensure the result is normalized
        let x = m[0][0] * direction.x + m[0][1] * direction.y + m[0][2] * direction.z
        let y = m[1][0] * direction.x + m[1][1] * direction.y + m[1][2] * direction.z
        let z = m[2][0] * direction.x + m[2][1] * direction.y + m[2][2] * direction.z
        
        return Vector3(x: x, y: y, z: z)
    }
    
    /// Extracts the translation component from this matrix
    public var translation: Vector3 {
        return Vector3(x: m[3][0], y: m[3][1], z: m[3][2])
    }
    
    /// Extracts the rotation component from this matrix as a quaternion
    public var rotation: Quaternion {
        // Algorithm from http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
        let trace = m[0][0] + m[1][1] + m[2][2]
        
        if trace > 0 {
            let s = 0.5 / sqrt(trace + 1.0)
            return Quaternion(
                x: (m[1][2] - m[2][1]) * s,
                y: (m[2][0] - m[0][2]) * s,
                z: (m[0][1] - m[1][0]) * s,
                w: 0.25 / s
            )
        } else if m[0][0] > m[1][1] && m[0][0] > m[2][2] {
            let s = 2.0 * sqrt(1.0 + m[0][0] - m[1][1] - m[2][2])
            return Quaternion(
                x: 0.25 * s,
                y: (m[0][1] + m[1][0]) / s,
                z: (m[2][0] + m[0][2]) / s,
                w: (m[1][2] - m[2][1]) / s
            )
        } else if m[1][1] > m[2][2] {
            let s = 2.0 * sqrt(1.0 + m[1][1] - m[0][0] - m[2][2])
            return Quaternion(
                x: (m[0][1] + m[1][0]) / s,
                y: 0.25 * s,
                z: (m[1][2] + m[2][1]) / s,
                w: (m[2][0] - m[0][2]) / s
            )
        } else {
            let s = 2.0 * sqrt(1.0 + m[2][2] - m[0][0] - m[1][1])
            return Quaternion(
                x: (m[2][0] + m[0][2]) / s,
                y: (m[1][2] + m[2][1]) / s,
                z: 0.25 * s,
                w: (m[0][1] - m[1][0]) / s
            )
        }
    }
    
    /// Extracts the scale component from this matrix
    public var scale: Vector3 {
        let scaleX = Vector3(x: m[0][0], y: m[0][1], z: m[0][2]).magnitude
        let scaleY = Vector3(x: m[1][0], y: m[1][1], z: m[1][2]).magnitude
        let scaleZ = Vector3(x: m[2][0], y: m[2][1], z: m[2][2]).magnitude
        
        return Vector3(x: scaleX, y: scaleY, z: scaleZ)
    }
    
    // MARK: - Operators
    
    public static func * (lhs: Matrix4x4, rhs: Matrix4x4) -> Matrix4x4 {
        var result = Matrix4x4()
        
        for i in 0..<4 {
            for j in 0..<4 {
                result.m[i][j] = 0
                for k in 0..<4 {
                    result.m[i][j] += lhs.m[k][j] * rhs.m[i][k]
                }
            }
        }
        
        return result
    }
}

// MARK: - CustomStringConvertible
extension Matrix4x4: CustomStringConvertible {
    public var description: String {
        var result = "Matrix4x4(\n"
        for i in 0..<4 {
            result += "  "
            for j in 0..<4 {
                result += String(format: "%.3f", m[j][i])
                if j < 3 {
                    result += ", "
                }
            }
            result += "\n"
        }
        result += ")"
        return result
    }
}
