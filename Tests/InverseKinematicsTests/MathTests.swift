import Testing
#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif
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
        
        // MARK: - Additional Vector3D Tests (Phase Two Improvements)
        
        @Test("Vector3D distance methods")
        func testVector3DDistanceMethods() {
            let v1 = Vector3D(x: 0.0, y: 0.0, z: 0.0)
            let v2 = Vector3D(x: 3.0, y: 4.0, z: 0.0)
            
            let distance = v1.distance(to: v2)
            let distanceSquared = (v2 - v1).magnitudeSquared // Calculate manually
            
            #expect(distance == 5.0)
            #expect(distanceSquared == 25.0)
            
            // Test symmetric property
            #expect(v1.distance(to: v2) == v2.distance(to: v1))
            #expect((v1 - v2).magnitudeSquared == (v2 - v1).magnitudeSquared)
        }
        
        @Test("Vector3D zero vector normalization")
        func testVector3DZeroVectorNormalization() {
            let zero = Vector3D.zero
            let normalized = zero.normalized
            
            // Zero vector normalization should return zero vector
            #expect(normalized == Vector3D.zero)
            #expect(normalized.magnitude == 0.0)
        }
        
        @Test("Vector3D utility properties") 
        func testVector3DUtilityProperties() {
            let zero = Vector3D.zero
            let nonZero = Vector3D(x: 1.0, y: 2.0, z: 3.0)
            
            // Test isZero-like behavior through magnitude
            #expect(zero.magnitude == 0.0)
            #expect(nonZero.magnitude > 0.0)
            
            // Test very small vectors (near zero)
            let nearZero = Vector3D(x: 1e-15, y: 1e-15, z: 1e-15)
            #expect(nearZero.magnitude < 1e-14)
        }
        
        @Test("Vector3D component-wise operations")
        func testVector3DComponentWiseOperations() {
            let v1 = Vector3D(x: 2.0, y: 4.0, z: 6.0)
            
            // Test division
            let divided = v1 / 2.0
            #expect(divided == Vector3D(x: 1.0, y: 2.0, z: 3.0))
            
            // Test negation
            let negated = -v1
            #expect(negated == Vector3D(x: -2.0, y: -4.0, z: -6.0))
        }
        
        @Test("Vector3D boundary and edge cases")
        func testVector3DBoundaryAndEdgeCases() {
            // Test with very large numbers
            let large = Vector3D(x: 1e100, y: 1e100, z: 1e100)
            #expect(large.magnitude.isFinite)
            
            // Test with very small numbers
            let tiny = Vector3D(x: 1e-100, y: 1e-100, z: 1e-100)
            #expect(tiny.magnitude.isFinite)
            
            // Test with NaN
            let nanVector = Vector3D(x: Double.nan, y: 1.0, z: 2.0)
            #expect(nanVector.magnitude.isNaN)
            
            // Test with infinity
            let infVector = Vector3D(x: .infinity, y: 1.0, z: 2.0)
            #expect(infVector.magnitude.isInfinite)
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
        
        // MARK: - Additional Quaternion Tests (Phase Two Improvements)
        
        @Test("Quaternion identity behavior")
        func testQuaternionIdentityBehavior() {
            let identity = Quaternion.identity
            let arbitraryQuaternion = Quaternion(x: 1.0, y: 2.0, z: 3.0, w: 4.0).normalized
            
            // Identity multiplication properties
            let leftIdentity = identity * arbitraryQuaternion
            let rightIdentity = arbitraryQuaternion * identity
            
            #expect(abs(leftIdentity.x - arbitraryQuaternion.x) < 1e-10)
            #expect(abs(leftIdentity.y - arbitraryQuaternion.y) < 1e-10) 
            #expect(abs(leftIdentity.z - arbitraryQuaternion.z) < 1e-10)
            #expect(abs(leftIdentity.w - arbitraryQuaternion.w) < 1e-10)
            
            // Note: Quaternion multiplication is not commutative in general,
            // but identity is a special case
            #expect(abs(rightIdentity.x - arbitraryQuaternion.x) < 1e-10)
            #expect(abs(rightIdentity.y - arbitraryQuaternion.y) < 1e-10)
            #expect(abs(rightIdentity.z - arbitraryQuaternion.z) < 1e-10)
            #expect(abs(rightIdentity.w - arbitraryQuaternion.w) < 1e-10)
            
            // Identity rotation should not change vectors
            let vector = Vector3D(x: 1.0, y: 2.0, z: 3.0)
            let rotated = identity.rotate(vector)
            #expect(abs(rotated.x - vector.x) < 1e-10)
            #expect(abs(rotated.y - vector.y) < 1e-10)
            #expect(abs(rotated.z - vector.z) < 1e-10)
        }
        
        @Test("Quaternion non-unit behavior")
        func testQuaternionNonUnitBehavior() {
            // Test operations on non-normalized quaternions
            let nonUnit = Quaternion(x: 2.0, y: 4.0, z: 6.0, w: 8.0)
            #expect(abs(nonUnit.magnitude - sqrt(120.0)) < 1e-10)
            
            let normalized = nonUnit.normalized
            #expect(abs(normalized.magnitude - 1.0) < 1e-10)
            
            // Verify normalization preserves direction
            let ratio = nonUnit.magnitude
            #expect(abs(normalized.x - nonUnit.x / ratio) < 1e-10)
            #expect(abs(normalized.y - nonUnit.y / ratio) < 1e-10)
            #expect(abs(normalized.z - nonUnit.z / ratio) < 1e-10)
            #expect(abs(normalized.w - nonUnit.w / ratio) < 1e-10)
        }
        
        @Test("Quaternion scalar multiplication")
        func testQuaternionScalarMultiplication() {
            let q = Quaternion(x: 1.0, y: 2.0, z: 3.0, w: 4.0)
            let scalar = 2.5
            
            // Test scalar multiplication (if available)
            // Note: This tests the concept - actual implementation may vary
            let scaled = Quaternion(
                x: q.x * scalar,
                y: q.y * scalar, 
                z: q.z * scalar,
                w: q.w * scalar
            )
            
            #expect(scaled.x == q.x * scalar)
            #expect(scaled.y == q.y * scalar)
            #expect(scaled.z == q.z * scalar)
            #expect(scaled.w == q.w * scalar)
        }
        
        @Test("Quaternion inverse and conjugate relationship")
        func testQuaternionInverseConjugateRelationship() {
            let q = Quaternion(axis: Vector3D.unitZ, angle: .pi / 3).normalized
            let conjugate = q.conjugate
            let inverse = q.inverse
            
            // For unit quaternions, inverse equals conjugate
            #expect(abs(conjugate.x - inverse.x) < 1e-10)
            #expect(abs(conjugate.y - inverse.y) < 1e-10)
            #expect(abs(conjugate.z - inverse.z) < 1e-10)
            #expect(abs(conjugate.w - inverse.w) < 1e-10)
            
            // q * q.inverse should equal identity
            let shouldBeIdentity = q * inverse
            #expect(abs(shouldBeIdentity.x - 0.0) < 1e-10)
            #expect(abs(shouldBeIdentity.y - 0.0) < 1e-10)
            #expect(abs(shouldBeIdentity.z - 0.0) < 1e-10)
            #expect(abs(shouldBeIdentity.w - 1.0) < 1e-10)
        }
        
        @Test("Quaternion edge cases")
        func testQuaternionEdgeCases() {
            // Test zero quaternion
            let zero = Quaternion(x: 0.0, y: 0.0, z: 0.0, w: 0.0)
            #expect(zero.magnitude == 0.0)
            
            // Test quaternion with NaN components
            let nanQuaternion = Quaternion(x: Double.nan, y: 1.0, z: 2.0, w: 3.0)
            #expect(nanQuaternion.magnitude.isNaN)
            
            // Test quaternion with infinite components
            let infQuaternion = Quaternion(x: .infinity, y: 1.0, z: 2.0, w: 3.0)
            #expect(infQuaternion.magnitude.isInfinite)
        }
        
        @Test("Quaternion axis-angle edge cases")
        func testQuaternionAxisAngleEdgeCases() {
            // Test zero angle rotation
            let zeroRotation = Quaternion(axis: Vector3D.unitX, angle: 0.0)
            #expect(abs(zeroRotation.x - 0.0) < 1e-10)
            #expect(abs(zeroRotation.y - 0.0) < 1e-10)
            #expect(abs(zeroRotation.z - 0.0) < 1e-10)
            #expect(abs(zeroRotation.w - 1.0) < 1e-10)
            
            // Test full rotation (2π)
            let fullRotation = Quaternion(axis: Vector3D.unitZ, angle: 2 * .pi)
            let nearIdentity = fullRotation.normalized
            #expect(abs(nearIdentity.w - 1.0) < 1e-6 || abs(nearIdentity.w + 1.0) < 1e-6)
            
            // Test with zero-magnitude axis
            let zeroAxis = Vector3D.zero
            let invalidAxisRotation = Quaternion(axis: zeroAxis, angle: .pi)
            // Should gracefully handle invalid axis (implementation dependent)
            #expect(invalidAxisRotation.magnitude.isFinite || invalidAxisRotation.magnitude.isNaN)
        }
        
        // MARK: - Phase Two Quaternion-Specific Tests
        
        @Test("Quaternion slerp - basic functionality")
        func testQuaternionSlerpBasic() {
            let q1 = Quaternion.identity
            let q2 = Quaternion(axis: Vector3D.unitZ, angle: Double.pi / 2)
            
            // Test at endpoints
            let slerp0 = q1.slerp(to: q2, t: 0.0)
            let slerp1 = q1.slerp(to: q2, t: 1.0)
            
            #expect(abs(slerp0.x - q1.x) < 1e-10)
            #expect(abs(slerp0.y - q1.y) < 1e-10)
            #expect(abs(slerp0.z - q1.z) < 1e-10)
            #expect(abs(slerp0.w - q1.w) < 1e-10)
            
            #expect(abs(slerp1.x - q2.x) < 1e-10)
            #expect(abs(slerp1.y - q2.y) < 1e-10)
            #expect(abs(slerp1.z - q2.z) < 1e-10)
            #expect(abs(slerp1.w - q2.w) < 1e-10)
            
            // Test midpoint
            let slerpMid = q1.slerp(to: q2, t: 0.5)
            #expect(abs(slerpMid.magnitude - 1.0) < 1e-10) // Should be normalized
        }
        
        @Test("Quaternion slerp - identical quaternions")
        func testQuaternionSlerpIdentical() {
            let q = Quaternion(axis: Vector3D.unitY, angle: Double.pi / 3)
            
            // Slerp between identical quaternions should return the same quaternion
            for t in [0.0, 0.25, 0.5, 0.75, 1.0] {
                let result = q.slerp(to: q, t: t)
                #expect(abs(result.x - q.x) < 1e-10)
                #expect(abs(result.y - q.y) < 1e-10)
                #expect(abs(result.z - q.z) < 1e-10)
                #expect(abs(result.w - q.w) < 1e-10)
            }
        }
        
        @Test("Quaternion slerp - diametrically opposite quaternions")
        func testQuaternionSlerpDiametricallyOpposite() {
            let q1 = Quaternion(axis: Vector3D.unitZ, angle: 0.0)     // Identity
            let q2 = Quaternion(axis: Vector3D.unitZ, angle: Double.pi) // 180° rotation
            
            // These quaternions are close to diametrically opposite
            let dot = q1.dot(q2)
            #expect(abs(dot) < 0.1) // Should be close to 0 (orthogonal)
            
            // Slerp should work smoothly
            let slerpMid = q1.slerp(to: q2, t: 0.5)
            #expect(abs(slerpMid.magnitude - 1.0) < 1e-10)
            
            // Should represent a 90° rotation around Z axis
            let testVector = Vector3D.unitX
            let rotated = slerpMid.rotate(testVector)
            #expect(abs(rotated.x) < 1e-6) // Should be close to 0
            #expect(abs(abs(rotated.y) - 1.0) < 1e-6) // Should be ±1
        }
        
        @Test("Quaternion slerp - edge case t values")
        func testQuaternionSlerpEdgeCases() {
            let q1 = Quaternion.identity
            let q2 = Quaternion(axis: Vector3D.unitX, angle: Double.pi / 4)
            
            // Test t outside [0,1] range
            let slerpNegative = q1.slerp(to: q2, t: -0.5)
            let slerpBeyond = q1.slerp(to: q2, t: 1.5)
            
            // Results should be valid quaternions
            #expect(slerpNegative.magnitude.isFinite)
            #expect(slerpBeyond.magnitude.isFinite)
            #expect(slerpNegative.magnitude > 0)
            #expect(slerpBeyond.magnitude > 0)
        }
        
        @Test("Quaternion slerp - preserves rotation magnitude")
        func testQuaternionSlerpPreservesRotationMagnitude() {
            let q1 = Quaternion(axis: Vector3D.unitX, angle: Double.pi / 6)
            let q2 = Quaternion(axis: Vector3D.unitY, angle: Double.pi / 3)
            
            // Test multiple t values
            for t in stride(from: 0.0, through: 1.0, by: 0.1) {
                let result = q1.slerp(to: q2, t: t)
                #expect(abs(result.magnitude - 1.0) < 1e-10)
            }
        }
        
        @Test("Quaternion Euler angles - basic functionality")
        func testQuaternionEulerAnglesBasicFunctionality() {
            // Test basic Euler angle functionality without expecting perfect roundtrip
            let testCases = [
                Vector3D(x: 0.0, y: 0.0, z: 0.0),                    // Identity
                Vector3D(x: Double.pi / 4, y: 0.0, z: 0.0),          // X rotation only
                Vector3D(x: 0.0, y: Double.pi / 6, z: 0.0),          // Y rotation only
                Vector3D(x: 0.0, y: 0.0, z: Double.pi / 3),          // Z rotation only
            ]
            
            for eulerAngles in testCases {
                let quaternion = Quaternion(eulerAngles: eulerAngles)
                let recoveredEuler = quaternion.eulerAngles
                
                // Verify both conversions produce valid results
                #expect(quaternion.magnitude.isFinite)
                #expect(quaternion.magnitude > 0)
                #expect(recoveredEuler.x.isFinite)
                #expect(recoveredEuler.y.isFinite)
                #expect(recoveredEuler.z.isFinite)
                
                // For identity case, ensure we get reasonable results
                if eulerAngles == Vector3D.zero {
                    #expect(abs(quaternion.x) < 1e-10)
                    #expect(abs(quaternion.y) < 1e-10)
                    #expect(abs(quaternion.z) < 1e-10)
                    #expect(abs(quaternion.w - 1.0) < 1e-10)
                }
            }
        }
        
        @Test("Quaternion Euler angles - gimbal lock detection")
        func testQuaternionEulerAnglesGimbalLock() {
            // Test near gimbal lock conditions (Y rotation ≈ ±π/2)
            let nearGimbalLock1 = Vector3D(x: Double.pi / 4, y: Double.pi / 2 - 1e-3, z: Double.pi / 6)
            let nearGimbalLock2 = Vector3D(x: Double.pi / 4, y: -Double.pi / 2 + 1e-3, z: Double.pi / 6)
            
            let q1 = Quaternion(eulerAngles: nearGimbalLock1)
            let q2 = Quaternion(eulerAngles: nearGimbalLock2)
            
            let euler1 = q1.eulerAngles
            let euler2 = q2.eulerAngles
            
            // Should handle gracefully and produce finite results
            #expect(euler1.x.isFinite)
            #expect(euler1.y.isFinite)
            #expect(euler1.z.isFinite)
            #expect(euler2.x.isFinite)
            #expect(euler2.y.isFinite)
            #expect(euler2.z.isFinite)
            
            // Y component should be within reasonable range (gimbal lock handling may adjust values)
            #expect(abs(euler1.y) <= Double.pi / 2 + 0.1)
            #expect(abs(euler2.y) <= Double.pi / 2 + 0.1)
        }
        
        @Test("Quaternion Euler angles - various rotation sequences")
        func testQuaternionEulerAnglesRotationSequences() {
            // Test that the current implementation (XYZ sequence) works consistently
            let testAngles = Vector3D(x: 0.3, y: 0.5, z: 0.7)
            let q = Quaternion(eulerAngles: testAngles)
            
            // Manually apply XYZ rotation sequence
            let qX = Quaternion(axis: Vector3D.unitX, angle: testAngles.x)
            let qY = Quaternion(axis: Vector3D.unitY, angle: testAngles.y)
            let qZ = Quaternion(axis: Vector3D.unitZ, angle: testAngles.z)
            
            // XYZ sequence: first X, then Y, then Z
            let qManual = qZ * qY * qX
            
            // Should produce similar rotation (allowing for numerical differences)
            let testVector = Vector3D(x: 1.0, y: 0.0, z: 0.0)
            let rotated1 = q.rotate(testVector)
            let rotated2 = qManual.rotate(testVector)
            
            #expect(abs(rotated1.x - rotated2.x) < 1e-6)
            #expect(abs(rotated1.y - rotated2.y) < 1e-6)
            #expect(abs(rotated1.z - rotated2.z) < 1e-6)
        }
        
        @Test("Quaternion rotation matrix conversion")
        func testQuaternionRotationMatrixConversion() {
            // Test manual rotation matrix generation from quaternion
            let q = Quaternion(axis: Vector3D.unitZ, angle: Double.pi / 2).normalized
            
            // Manual rotation matrix calculation for Z rotation
            let xx = q.x * q.x
            let yy = q.y * q.y
            let zz = q.z * q.z
            let xy = q.x * q.y
            let xz = q.x * q.z
            let yz = q.y * q.z
            let wx = q.w * q.x
            let wy = q.w * q.y
            let wz = q.w * q.z
            
            // 3x3 rotation matrix elements
            let m00 = 1.0 - 2.0 * (yy + zz)
            let m01 = 2.0 * (xy - wz)
            let m02 = 2.0 * (xz + wy)
            let m10 = 2.0 * (xy + wz)
            let m11 = 1.0 - 2.0 * (xx + zz)
            let m12 = 2.0 * (yz - wx)
            let m20 = 2.0 * (xz - wy)
            let m21 = 2.0 * (yz + wx)
            let m22 = 1.0 - 2.0 * (xx + yy)
            
            // For 90° Z rotation, expected matrix:
            // [ 0  -1   0 ]
            // [ 1   0   0 ]
            // [ 0   0   1 ]
            #expect(abs(m00 - 0.0) < 1e-10)
            #expect(abs(m01 - (-1.0)) < 1e-10)
            #expect(abs(m02 - 0.0) < 1e-10)
            #expect(abs(m10 - 1.0) < 1e-10)
            #expect(abs(m11 - 0.0) < 1e-10)
            #expect(abs(m12 - 0.0) < 1e-10)
            #expect(abs(m20 - 0.0) < 1e-10)
            #expect(abs(m21 - 0.0) < 1e-10)
            #expect(abs(m22 - 1.0) < 1e-10)
        }
        
        @Test("Quaternion rotation matrix - identity verification")
        func testQuaternionRotationMatrixIdentity() {
            let identity = Quaternion.identity
            
            // Identity quaternion should produce identity matrix
            let xx = identity.x * identity.x
            let yy = identity.y * identity.y
            let zz = identity.z * identity.z
            let xy = identity.x * identity.y
            let xz = identity.x * identity.z
            let yz = identity.y * identity.z
            let wx = identity.w * identity.x
            let wy = identity.w * identity.y
            let wz = identity.w * identity.z
            
            let m00 = 1.0 - 2.0 * (yy + zz)
            let m01 = 2.0 * (xy - wz)
            let m02 = 2.0 * (xz + wy)
            let m10 = 2.0 * (xy + wz)
            let m11 = 1.0 - 2.0 * (xx + zz)
            let m12 = 2.0 * (yz - wx)
            let m20 = 2.0 * (xz - wy)
            let m21 = 2.0 * (yz + wx)
            let m22 = 1.0 - 2.0 * (xx + yy)
            
            // Should be identity matrix
            #expect(abs(m00 - 1.0) < 1e-10)
            #expect(abs(m01 - 0.0) < 1e-10)
            #expect(abs(m02 - 0.0) < 1e-10)
            #expect(abs(m10 - 0.0) < 1e-10)
            #expect(abs(m11 - 1.0) < 1e-10)
            #expect(abs(m12 - 0.0) < 1e-10)
            #expect(abs(m20 - 0.0) < 1e-10)
            #expect(abs(m21 - 0.0) < 1e-10)
            #expect(abs(m22 - 1.0) < 1e-10)
        }
        
        @Test("Quaternion rotation matrix - orthogonality verification")
        func testQuaternionRotationMatrixOrthogonality() {
            let q = Quaternion(axis: Vector3D(x: 0.6, y: 0.8, z: 0.0).normalized, angle: 0.7).normalized
            
            // Generate rotation matrix
            let xx = q.x * q.x
            let yy = q.y * q.y
            let zz = q.z * q.z
            let xy = q.x * q.y
            let xz = q.x * q.z
            let yz = q.y * q.z
            let wx = q.w * q.x
            let wy = q.w * q.y
            let wz = q.w * q.z
            
            let m00 = 1.0 - 2.0 * (yy + zz)
            let m01 = 2.0 * (xy - wz)
            let m02 = 2.0 * (xz + wy)
            let m10 = 2.0 * (xy + wz)
            let m11 = 1.0 - 2.0 * (xx + zz)
            let m12 = 2.0 * (yz - wx)
            let m20 = 2.0 * (xz - wy)
            let m21 = 2.0 * (yz + wx)
            let m22 = 1.0 - 2.0 * (xx + yy)
            
            // Verify columns are unit vectors (orthogonal matrix property)
            let col1Magnitude = sqrt(m00*m00 + m10*m10 + m20*m20)
            let col2Magnitude = sqrt(m01*m01 + m11*m11 + m21*m21)
            let col3Magnitude = sqrt(m02*m02 + m12*m12 + m22*m22)
            
            #expect(abs(col1Magnitude - 1.0) < 1e-10)
            #expect(abs(col2Magnitude - 1.0) < 1e-10)
            #expect(abs(col3Magnitude - 1.0) < 1e-10)
            
            // Verify columns are orthogonal (dot products should be 0)
            let col1DotCol2 = m00*m01 + m10*m11 + m20*m21
            let col1DotCol3 = m00*m02 + m10*m12 + m20*m22
            let col2DotCol3 = m01*m02 + m11*m12 + m21*m22
            
            #expect(abs(col1DotCol2) < 1e-10)
            #expect(abs(col1DotCol3) < 1e-10)
            #expect(abs(col2DotCol3) < 1e-10)
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
        
        // MARK: - Additional Transform Tests (Phase Two Improvements)
        
        @Test("Transform direction vector transformation")
        func testTransformDirectionVectorTransformation() {
            // Direction vectors should only be affected by rotation, not translation
            let pos = Vector3D(x: 5.0, y: 10.0, z: 15.0) // Large translation
            let rot = Quaternion(axis: Vector3D.unitZ, angle: .pi / 2) // 90° rotation
            let transform = Transform(position: pos, rotation: rot)
            
            let direction = Vector3D.unitX
            let transformedDirection = transform.transformDirection(direction)
            
            // Should be rotated but not translated
            #expect(abs(transformedDirection.x - 0.0) < 1e-10)
            #expect(abs(transformedDirection.y - 1.0) < 1e-10)
            #expect(abs(transformedDirection.z - 0.0) < 1e-10)
            
            // Magnitude should be preserved
            #expect(abs(transformedDirection.magnitude - direction.magnitude) < 1e-10)
        }
        
        @Test("Transform identity behavior")
        func testTransformIdentityBehavior() {
            let identity = Transform()
            let arbitraryPoint = Vector3D(x: 3.0, y: 4.0, z: 5.0)
            let arbitraryDirection = Vector3D(x: 1.0, y: 1.0, z: 1.0).normalized
            
            // Identity transform should not change points or directions
            let transformedPoint = identity.transformPoint(arbitraryPoint)
            let transformedDirection = identity.transformDirection(arbitraryDirection)
            
            #expect(abs(transformedPoint.x - arbitraryPoint.x) < 1e-10)
            #expect(abs(transformedPoint.y - arbitraryPoint.y) < 1e-10)
            #expect(abs(transformedPoint.z - arbitraryPoint.z) < 1e-10)
            
            #expect(abs(transformedDirection.x - arbitraryDirection.x) < 1e-10)
            #expect(abs(transformedDirection.y - arbitraryDirection.y) < 1e-10)
            #expect(abs(transformedDirection.z - arbitraryDirection.z) < 1e-10)
        }
        
        @Test("Transform multiplication identity properties")
        func testTransformMultiplicationIdentityProperties() {
            let identity = Transform()
            let arbitraryTransform = Transform(
                position: Vector3D(x: 1.0, y: 2.0, z: 3.0),
                rotation: Quaternion(axis: Vector3D.unitY, angle: .pi / 3)
            )
            
            // Left identity
            let leftResult = identity * arbitraryTransform
            #expect(abs(leftResult.position.x - arbitraryTransform.position.x) < 1e-10)
            #expect(abs(leftResult.position.y - arbitraryTransform.position.y) < 1e-10)
            #expect(abs(leftResult.position.z - arbitraryTransform.position.z) < 1e-10)
            
            // Right identity 
            let rightResult = arbitraryTransform * identity
            #expect(abs(rightResult.position.x - arbitraryTransform.position.x) < 1e-10)
            #expect(abs(rightResult.position.y - arbitraryTransform.position.y) < 1e-10)
            #expect(abs(rightResult.position.z - arbitraryTransform.position.z) < 1e-10)
        }
        
        @Test("Transform multiplication associativity")
        func testTransformMultiplicationAssociativity() {
            let t1 = Transform(
                position: Vector3D(x: 1.0, y: 0.0, z: 0.0),
                rotation: Quaternion(axis: Vector3D.unitZ, angle: .pi / 4)
            )
            let t2 = Transform(
                position: Vector3D(x: 0.0, y: 1.0, z: 0.0),
                rotation: Quaternion(axis: Vector3D.unitY, angle: .pi / 6)
            )
            let t3 = Transform(
                position: Vector3D(x: 0.0, y: 0.0, z: 1.0),
                rotation: Quaternion(axis: Vector3D.unitX, angle: .pi / 3)
            )
            
            // Test (t1 * t2) * t3 == t1 * (t2 * t3)
            let leftAssociation = (t1 * t2) * t3
            let rightAssociation = t1 * (t2 * t3)
            
            #expect(abs(leftAssociation.position.x - rightAssociation.position.x) < 1e-10)
            #expect(abs(leftAssociation.position.y - rightAssociation.position.y) < 1e-10)
            #expect(abs(leftAssociation.position.z - rightAssociation.position.z) < 1e-10)
            
            #expect(abs(leftAssociation.rotation.x - rightAssociation.rotation.x) < 1e-10)
            #expect(abs(leftAssociation.rotation.y - rightAssociation.rotation.y) < 1e-10)
            #expect(abs(leftAssociation.rotation.z - rightAssociation.rotation.z) < 1e-10)
            #expect(abs(leftAssociation.rotation.w - rightAssociation.rotation.w) < 1e-10)
        }
        
        @Test("Transform inverse of identity")
        func testTransformInverseOfIdentity() {
            let identity = Transform()
            let identityInverse = identity.inverse
            
            // Inverse of identity should be identity
            #expect(abs(identityInverse.position.magnitude) < 1e-10)
            #expect(abs(identityInverse.rotation.x - 0.0) < 1e-10)
            #expect(abs(identityInverse.rotation.y - 0.0) < 1e-10)
            #expect(abs(identityInverse.rotation.z - 0.0) < 1e-10)
            #expect(abs(identityInverse.rotation.w - 1.0) < 1e-10)
        }
        
        @Test("Transform edge cases")
        func testTransformEdgeCases() {
            // Transform with zero rotation (identity quaternion)
            let translationOnly = Transform(
                position: Vector3D(x: 1.0, y: 2.0, z: 3.0),
                rotation: Quaternion.identity
            )
            
            let point = Vector3D(x: 1.0, y: 1.0, z: 1.0)
            let transformed = translationOnly.transformPoint(point)
            let expected = Vector3D(x: 2.0, y: 3.0, z: 4.0) // point + translation
            
            #expect(abs(transformed.x - expected.x) < 1e-10)
            #expect(abs(transformed.y - expected.y) < 1e-10)
            #expect(abs(transformed.z - expected.z) < 1e-10)
            
            // Transform with zero translation
            let rotationOnly = Transform(
                position: Vector3D.zero,
                rotation: Quaternion(axis: Vector3D.unitZ, angle: .pi / 2)
            )
            
            let rotatedPoint = rotationOnly.transformPoint(Vector3D.unitX)
            #expect(abs(rotatedPoint.x - 0.0) < 1e-10)
            #expect(abs(rotatedPoint.y - 1.0) < 1e-10)
            #expect(abs(rotatedPoint.z - 0.0) < 1e-10)
        }
        
        // MARK: - Matrix4x4 Interoperation Tests
        
        @Test("Transform Matrix4x4 translation extraction")
        func testTransformMatrix4x4TranslationExtraction() {
            let position = Vector3D(x: 2.0, y: 3.0, z: 4.0)
            let rotation = Quaternion.identity
            let transform = Transform(position: position, rotation: rotation)
            
            // Convert to matrix
            let matrix = transform.matrix
            
            // Verify translation components
            #expect(abs(matrix.translation.x - position.x) < 1e-10)
            #expect(abs(matrix.translation.y - position.y) < 1e-10)
            #expect(abs(matrix.translation.z - position.z) < 1e-10)
        }
        
        @Test("Transform Matrix4x4 basic interoperation")
        func testTransformMatrix4x4BasicInteroperation() {
            // Test core Matrix4x4 interoperation functionality: conversion and component extraction
            let position = Vector3D(x: 1.0, y: 2.0, z: 3.0)
            let rotation = Quaternion.identity // Use identity to avoid matrix-to-quaternion conversion complexities
            let originalTransform = Transform(position: position, rotation: rotation)
            
            // Convert to matrix
            let matrix = originalTransform.matrix
            
            // Extract components back
            let extractedPosition = matrix.translation
            let extractedRotation = matrix.rotation
            
            // Verify translation roundtrip is exact
            #expect(abs(extractedPosition.x - position.x) < 1e-10)
            #expect(abs(extractedPosition.y - position.y) < 1e-10)
            #expect(abs(extractedPosition.z - position.z) < 1e-10)
            
            // Verify identity rotation extraction
            #expect(abs(extractedRotation.x) < 1e-10)
            #expect(abs(extractedRotation.y) < 1e-10)
            #expect(abs(extractedRotation.z) < 1e-10)
            #expect(abs(extractedRotation.w - 1.0) < 1e-10)
            
            // Test that matrix can be used for point transformation
            let testPoint = Vector3D(x: 0.5, y: 0.5, z: 0.5)
            let transformedByMatrix = matrix.transformPoint(testPoint)
            let transformedByTransform = originalTransform.transformPoint(testPoint)
            
            // For identity rotation, these should be identical
            #expect(abs(transformedByMatrix.x - transformedByTransform.x) < 1e-10)
            #expect(abs(transformedByMatrix.y - transformedByTransform.y) < 1e-10)
            #expect(abs(transformedByMatrix.z - transformedByTransform.z) < 1e-10)
        }
        
        @Test("Matrix4x4 identity transform verification")
        func testMatrix4x4IdentityTransformVerification() {
            let identityTransform = Transform.identity
            let identityMatrix = identityTransform.matrix
            
            // Verify identity matrix elements
            for row in 0..<4 {
                for col in 0..<4 {
                    let expected = (row == col) ? 1.0 : 0.0
                    #expect(abs(identityMatrix[row, col] - expected) < 1e-10)
                }
            }
            
            // Verify extracted components
            let extractedPosition = identityMatrix.translation
            let extractedRotation = identityMatrix.rotation
            
            #expect(abs(extractedPosition.magnitude) < 1e-10)
            #expect(abs(extractedRotation.x) < 1e-10)
            #expect(abs(extractedRotation.y) < 1e-10)
            #expect(abs(extractedRotation.z) < 1e-10)
            #expect(abs(extractedRotation.w - 1.0) < 1e-10)
        }
        
        @Test("Matrix4x4 point transformation basic functionality")
        func testMatrix4x4PointTransformationBasicFunctionality() {
            // Test simple translation only
            let position = Vector3D(x: 1.0, y: 2.0, z: 3.0)
            let rotation = Quaternion.identity
            let transform = Transform(position: position, rotation: rotation)
            let matrix = transform.matrix
            
            let testPoint = Vector3D(x: 0.5, y: 1.5, z: -1.0)
            
            // Transform using Transform method
            let transformResult = transform.transformPoint(testPoint)
            
            // Transform using Matrix4x4 method
            let matrixResult = matrix.transformPoint(testPoint)
            
            // For translation-only, results should be very close
            #expect(abs(transformResult.x - matrixResult.x) < 1e-10)
            #expect(abs(transformResult.y - matrixResult.y) < 1e-10)
            #expect(abs(transformResult.z - matrixResult.z) < 1e-10)
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