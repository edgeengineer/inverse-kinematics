import Testing
import Foundation
@testable import InverseKinematics

struct QuaternionTests {
    // Test quaternion initialization
    @Test func testQuaternionInitialization() {
        let q = Quaternion(x: 0, y: 0, z: 0, w: 1)
        #expect(q.x == 0)
        #expect(q.y == 0)
        #expect(q.z == 0)
        #expect(q.w == 1)
    }
    
    // Test quaternion identity
    @Test func testQuaternionIdentity() {
        let identity = Quaternion.identity
        #expect(identity.x == 0)
        #expect(identity.y == 0)
        #expect(identity.z == 0)
        #expect(identity.w == 1)
    }
    
    // Test quaternion from axis-angle
    @Test func testQuaternionFromAxisAngle() {
        // 90-degree rotation around Y axis
        let q = Quaternion(axis: Vector3(x: 0, y: 1, z: 0), angle: Float.pi / 2)
        #expect(abs(q.x) < 1e-6)
        #expect(abs(q.y - 0.7071) < 1e-4) // sin(π/4) ≈ 0.7071
        #expect(abs(q.z) < 1e-6)
        #expect(abs(q.w - 0.7071) < 1e-4) // cos(π/4) ≈ 0.7071
    }
    
    // Test quaternion multiplication
    @Test func testQuaternionMultiplication() {
        // Two 90-degree rotations around Y should equal one 180-degree rotation
        let q1 = Quaternion(axis: Vector3(x: 0, y: 1, z: 0), angle: Float.pi / 2)
        let q2 = Quaternion(axis: Vector3(x: 0, y: 1, z: 0), angle: Float.pi / 2)
        let result = q1 * q2
        
        // Expected result is a 180-degree rotation around Y
        let expected = Quaternion(axis: Vector3(x: 0, y: 1, z: 0), angle: Float.pi)
        
        #expect(abs(result.x - expected.x) < 1e-6)
        #expect(abs(result.y - expected.y) < 1e-6)
        #expect(abs(result.z - expected.z) < 1e-6)
        #expect(abs(result.w - expected.w) < 1e-6)
    }
    
    // Test quaternion rotation of a vector
    @Test func testQuaternionRotateVector() {
        // 90-degree rotation around Y axis
        let q = Quaternion(axis: Vector3(x: 0, y: 1, z: 0), angle: Float.pi / 2)
        
        // Rotating (0, 0, 1) 90 degrees around Y should give (1, 0, 0)
        let v = Vector3(x: 0, y: 0, z: 1)
        let rotated = q.rotate(v)
        
        #expect(abs(rotated.x - 1) < 1e-6)
        #expect(abs(rotated.y) < 1e-6)
        #expect(abs(rotated.z) < 1e-6)
    }
    
    // Test quaternion slerp
    @Test func testQuaternionSlerp() {
        // Identity quaternion
        let q1 = Quaternion.identity
        
        // 180-degree rotation around Y
        let q2 = Quaternion(axis: Vector3(x: 0, y: 1, z: 0), angle: Float.pi)
        
        // Slerp halfway should give a 90-degree rotation
        let slerp = q1.slerp(to: q2, t: 0.5)
        let expected = Quaternion(axis: Vector3(x: 0, y: 1, z: 0), angle: Float.pi / 2)
        
        #expect(abs(slerp.x - expected.x) < 1e-6)
        #expect(abs(slerp.y - expected.y) < 1e-6)
        #expect(abs(slerp.z - expected.z) < 1e-6)
        #expect(abs(slerp.w - expected.w) < 1e-6)
    }
}
