import Testing
import Foundation
@testable import InverseKinematics

struct Vector3Tests {
    // Test vector initialization
    @Test func testVectorInitialization() {
        let v = Vector3(x: 1, y: 2, z: 3)
        #expect(v.x == 1)
        #expect(v.y == 2)
        #expect(v.z == 3)
    }
    
    // Test vector addition
    @Test func testVectorAddition() {
        let v1 = Vector3(x: 1, y: 2, z: 3)
        let v2 = Vector3(x: 4, y: 5, z: 6)
        let result = v1 + v2
        #expect(result.x == 5)
        #expect(result.y == 7)
        #expect(result.z == 9)
    }
    
    // Test vector subtraction
    @Test func testVectorSubtraction() {
        let v1 = Vector3(x: 4, y: 5, z: 6)
        let v2 = Vector3(x: 1, y: 2, z: 3)
        let result = v1 - v2
        #expect(result.x == 3)
        #expect(result.y == 3)
        #expect(result.z == 3)
    }
    
    // Test vector scalar multiplication
    @Test func testVectorScalarMultiplication() {
        let v = Vector3(x: 1, y: 2, z: 3)
        let result = v * 2
        #expect(result.x == 2)
        #expect(result.y == 4)
        #expect(result.z == 6)
    }
    
    // Test vector magnitude
    @Test func testVectorMagnitude() {
        let v = Vector3(x: 3, y: 4, z: 0)
        #expect(v.magnitude == 5)
    }
    
    // Test vector normalization
    @Test func testVectorNormalization() {
        let v = Vector3(x: 3, y: 4, z: 0)
        let normalized = v.normalized
        // The magnitude of a normalized vector should be 1 (or very close to it)
        #expect(abs(normalized.magnitude - 1) < 1e-6)
    }
    
    // Test vector dot product
    @Test func testVectorDotProduct() {
        let v1 = Vector3(x: 1, y: 2, z: 3)
        let v2 = Vector3(x: 4, y: 5, z: 6)
        let dot = v1.dot(v2)
        #expect(dot == 32) // 1*4 + 2*5 + 3*6 = 32
    }
    
    // Test vector cross product
    @Test func testVectorCrossProduct() {
        let v1 = Vector3(x: 1, y: 0, z: 0)
        let v2 = Vector3(x: 0, y: 1, z: 0)
        let cross = v1.cross(v2)
        #expect(cross.x == 0)
        #expect(cross.y == 0)
        #expect(cross.z == 1)
    }
    
    // Test vector distance
    @Test func testVectorDistance() {
        let v1 = Vector3(x: 0, y: 0, z: 0)
        let v2 = Vector3(x: 3, y: 4, z: 0)
        let distance = v1.distance(to: v2)
        #expect(distance == 5)
    }
    
    // Test vector lerp
    @Test func testVectorLerp() {
        let v1 = Vector3(x: 0, y: 0, z: 0)
        let v2 = Vector3(x: 10, y: 10, z: 10)
        let lerp = v1.lerp(to: v2, t: 0.5)
        #expect(lerp.x == 5)
        #expect(lerp.y == 5)
        #expect(lerp.z == 5)
    }
}