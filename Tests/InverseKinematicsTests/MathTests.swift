import Testing
import Foundation
@testable import InverseKinematics

@Suite("Math Types Tests")
struct MathTests {
    
    @Suite("Vector3D Tests")
    struct Vector3DTests {
        
        @Test("Vector3D initialization")
        func testVector3DInit() {
            let v1 = Vector3D()
            #expect(v1.x == 0.0)
            #expect(v1.y == 0.0)
            #expect(v1.z == 0.0)
            
            let v2 = Vector3D(x: 1.0, y: 2.0, z: 3.0)
            #expect(v2.x == 1.0)
            #expect(v2.y == 2.0)
            #expect(v2.z == 3.0)
        }
        
        @Test("Vector3D static constants")
        func testVector3DConstants() {
            #expect(Vector3D.zero == Vector3D(x: 0, y: 0, z: 0))
            #expect(Vector3D.unitX == Vector3D(x: 1, y: 0, z: 0))
            #expect(Vector3D.unitY == Vector3D(x: 0, y: 1, z: 0))
            #expect(Vector3D.unitZ == Vector3D(x: 0, y: 0, z: 1))
        }
        
        @Test("Vector3D magnitude")
        func testVector3DMagnitude() {
            let v = Vector3D(x: 3.0, y: 4.0, z: 0.0)
            #expect(v.magnitude == 5.0)
            #expect(v.magnitudeSquared == 25.0)
        }
        
        @Test("Vector3D normalization")
        func testVector3DNormalization() {
            let v = Vector3D(x: 3.0, y: 4.0, z: 0.0)
            let normalized = v.normalized
            #expect(abs(normalized.magnitude - 1.0) < 1e-10)
        }
        
        @Test("Vector3D arithmetic operations")
        func testVector3DArithmetic() {
            let v1 = Vector3D(x: 1.0, y: 2.0, z: 3.0)
            let v2 = Vector3D(x: 4.0, y: 5.0, z: 6.0)
            
            let sum = v1 + v2
            #expect(sum == Vector3D(x: 5.0, y: 7.0, z: 9.0))
            
            let diff = v2 - v1
            #expect(diff == Vector3D(x: 3.0, y: 3.0, z: 3.0))
            
            let scaled = v1 * 2.0
            #expect(scaled == Vector3D(x: 2.0, y: 4.0, z: 6.0))
        }
        
        @Test("Vector3D dot product")
        func testVector3DDotProduct() {
            let v1 = Vector3D(x: 1.0, y: 2.0, z: 3.0)
            let v2 = Vector3D(x: 4.0, y: 5.0, z: 6.0)
            let dot = v1.dot(v2)
            #expect(dot == 32.0)
        }
        
        @Test("Vector3D cross product")
        func testVector3DCrossProduct() {
            let v1 = Vector3D(x: 1.0, y: 0.0, z: 0.0)
            let v2 = Vector3D(x: 0.0, y: 1.0, z: 0.0)
            let cross = v1.cross(v2)
            #expect(cross == Vector3D(x: 0.0, y: 0.0, z: 1.0))
        }
    }
    
    @Suite("Quaternion Tests")
    struct QuaternionTests {
        
        @Test("Quaternion initialization")
        func testQuaternionInit() {
            let q1 = Quaternion()
            #expect(q1.x == 0.0)
            #expect(q1.y == 0.0)
            #expect(q1.z == 0.0)
            #expect(q1.w == 1.0)
            
            let q2 = Quaternion(x: 1.0, y: 2.0, z: 3.0, w: 4.0)
            #expect(q2.x == 1.0)
            #expect(q2.y == 2.0)
            #expect(q2.z == 3.0)
            #expect(q2.w == 4.0)
        }
        
        @Test("Quaternion from axis-angle")
        func testQuaternionFromAxisAngle() {
            let axis = Vector3D.unitZ
            let angle = Double.pi / 2
            let q = Quaternion(axis: axis, angle: angle)
            
            #expect(abs(q.w - cos(angle / 2)) < 1e-10)
            #expect(abs(q.z - sin(angle / 2)) < 1e-10)
        }
        
        @Test("Quaternion normalization")
        func testQuaternionNormalization() {
            let q = Quaternion(x: 1.0, y: 2.0, z: 3.0, w: 4.0)
            let normalized = q.normalized
            #expect(abs(normalized.magnitude - 1.0) < 1e-10)
        }
        
        @Test("Quaternion conjugate")
        func testQuaternionConjugate() {
            let q = Quaternion(x: 1.0, y: 2.0, z: 3.0, w: 4.0)
            let conj = q.conjugate
            #expect(conj.x == -1.0)
            #expect(conj.y == -2.0)
            #expect(conj.z == -3.0)
            #expect(conj.w == 4.0)
        }
        
        @Test("Quaternion multiplication")
        func testQuaternionMultiplication() {
            let q1 = Quaternion.identity
            let q2 = Quaternion(axis: Vector3D.unitZ, angle: .pi / 2)
            let result = q1 * q2
            #expect(result == q2)
        }
        
        @Test("Quaternion vector rotation")
        func testQuaternionVectorRotation() {
            let q = Quaternion(axis: Vector3D.unitZ, angle: .pi / 2)
            let v = Vector3D.unitX
            let rotated = q.rotate(v)
            
            #expect(abs(rotated.x - 0.0) < 1e-10)
            #expect(abs(rotated.y - 1.0) < 1e-10)
            #expect(abs(rotated.z - 0.0) < 1e-10)
        }
    }
    
    @Suite("Transform Tests")
    struct TransformTests {
        
        @Test("Transform initialization")
        func testTransformInit() {
            let t1 = Transform()
            #expect(t1.position == Vector3D.zero)
            #expect(t1.rotation == Quaternion.identity)
            
            let pos = Vector3D(x: 1.0, y: 2.0, z: 3.0)
            let rot = Quaternion(axis: Vector3D.unitZ, angle: .pi / 4)
            let t2 = Transform(position: pos, rotation: rot)
            #expect(t2.position == pos)
            #expect(t2.rotation == rot)
        }
        
        @Test("Transform point transformation")
        func testTransformPointTransformation() {
            let pos = Vector3D(x: 1.0, y: 0.0, z: 0.0)
            let rot = Quaternion(axis: Vector3D.unitZ, angle: .pi / 2)
            let transform = Transform(position: pos, rotation: rot)
            
            let point = Vector3D(x: 1.0, y: 0.0, z: 0.0)
            let transformed = transform.transformPoint(point)
            
            #expect(abs(transformed.x - 1.0) < 1e-10)
            #expect(abs(transformed.y - 1.0) < 1e-10)
            #expect(abs(transformed.z - 0.0) < 1e-10)
        }
        
        @Test("Transform multiplication")
        func testTransformMultiplication() {
            let t1 = Transform(position: Vector3D(x: 1.0, y: 0.0, z: 0.0), rotation: Quaternion.identity)
            let t2 = Transform(position: Vector3D(x: 0.0, y: 1.0, z: 0.0), rotation: Quaternion.identity)
            
            let result = t1 * t2
            #expect(result.position == Vector3D(x: 1.0, y: 1.0, z: 0.0))
        }
        
        @Test("Transform inverse")
        func testTransformInverse() {
            let pos = Vector3D(x: 1.0, y: 2.0, z: 3.0)
            let rot = Quaternion(axis: Vector3D.unitZ, angle: .pi / 4)
            let transform = Transform(position: pos, rotation: rot)
            
            let inverse = transform.inverse
            let identity = transform * inverse
            
            #expect(abs(identity.position.magnitude) < 1e-10)
            #expect(abs(identity.rotation.x) < 1e-10)
            #expect(abs(identity.rotation.y) < 1e-10)
            #expect(abs(identity.rotation.z) < 1e-10)
            #expect(abs(identity.rotation.w - 1.0) < 1e-10)
        }
    }
    
    @Suite("Matrix4x4 Tests")
    struct Matrix4x4Tests {
        
        @Test("Matrix4x4 initialization")
        func testMatrix4x4Init() {
            let m1 = Matrix4x4()
            #expect(m1.elements.count == 16)
            #expect(m1.elements.allSatisfy { $0 == 0.0 })
            
            let identity = Matrix4x4.identity
            #expect(identity[0, 0] == 1.0)
            #expect(identity[1, 1] == 1.0)
            #expect(identity[2, 2] == 1.0)
            #expect(identity[3, 3] == 1.0)
        }
        
        @Test("Matrix4x4 from transform")
        func testMatrix4x4FromTransform() {
            let pos = Vector3D(x: 1.0, y: 2.0, z: 3.0)
            let rot = Quaternion.identity
            let matrix = Matrix4x4(translation: pos, rotation: rot)
            
            #expect(matrix.translation == pos)
            #expect(matrix[0, 3] == 1.0)
            #expect(matrix[1, 3] == 2.0)
            #expect(matrix[2, 3] == 3.0)
        }
        
        @Test("Matrix4x4 point transformation")
        func testMatrix4x4PointTransformation() {
            let pos = Vector3D(x: 1.0, y: 0.0, z: 0.0)
            let matrix = Matrix4x4(translation: pos)
            
            let point = Vector3D(x: 1.0, y: 0.0, z: 0.0)
            let transformed = matrix.transformPoint(point)
            
            #expect(transformed == Vector3D(x: 2.0, y: 0.0, z: 0.0))
        }
        
        @Test("Matrix4x4 multiplication")
        func testMatrix4x4Multiplication() {
            let m1 = Matrix4x4(translation: Vector3D(x: 1.0, y: 0.0, z: 0.0))
            let m2 = Matrix4x4(translation: Vector3D(x: 0.0, y: 1.0, z: 0.0))
            
            let result = m1 * m2
            let expectedTranslation = Vector3D(x: 1.0, y: 1.0, z: 0.0)
            
            #expect(abs(result.translation.x - expectedTranslation.x) < 1e-10)
            #expect(abs(result.translation.y - expectedTranslation.y) < 1e-10)
            #expect(abs(result.translation.z - expectedTranslation.z) < 1e-10)
        }
    }
}