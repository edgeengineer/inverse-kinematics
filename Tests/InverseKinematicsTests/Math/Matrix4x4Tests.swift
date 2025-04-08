import Testing
import Foundation
@testable import InverseKinematics

struct Matrix4x4Tests {
    // Test matrix initialization
    @Test func testMatrixInitialization() {
        let m = Matrix4x4()
        #expect(m.m[0][0] == 1)
        #expect(m.m[1][1] == 1)
        #expect(m.m[2][2] == 1)
        #expect(m.m[3][3] == 1)
    }
    
    // Test matrix identity
    @Test func testMatrixIdentity() {
        let identity = Matrix4x4.identity
        #expect(identity.m[0][0] == 1)
        #expect(identity.m[1][1] == 1)
        #expect(identity.m[2][2] == 1)
        #expect(identity.m[3][3] == 1)
    }
    
    // Test matrix translation
    @Test func testMatrixTranslation() {
        let translation = Matrix4x4.translation(Vector3(x: 1, y: 2, z: 3))
        #expect(translation.m[3][0] == 1)
        #expect(translation.m[3][1] == 2)
        #expect(translation.m[3][2] == 3)
    }
    
    // Test matrix scaling
    @Test func testMatrixScaling() {
        let scaling = Matrix4x4.scaling(Vector3(x: 2, y: 3, z: 4))
        #expect(scaling.m[0][0] == 2)
        #expect(scaling.m[1][1] == 3)
        #expect(scaling.m[2][2] == 4)
    }
    
    // Test matrix rotation
    @Test func testMatrixRotation() {
        // 90-degree rotation around Y axis
        let q = Quaternion(axis: Vector3(x: 0, y: 1, z: 0), angle: Float.pi / 2)
        let rotation = Matrix4x4.rotation(q)
        
        // Rotating (0, 0, 1) 90 degrees around Y should give (1, 0, 0)
        let v = Vector3(x: 0, y: 0, z: 1)
        let rotated = rotation.transformDirection(v)
        
        #expect(abs(rotated.x - 1) < 1e-6)
        #expect(abs(rotated.y) < 1e-6)
        #expect(abs(rotated.z) < 1e-6)
    }
    
    // Test matrix multiplication
    @Test func testMatrixMultiplication() {
        let translation = Matrix4x4.translation(Vector3(x: 1, y: 2, z: 3))
        let scaling = Matrix4x4.scaling(Vector3(x: 2, y: 2, z: 2))
        
        // Scale then translate
        let combined = translation * scaling
        
        // Transform a point (1, 1, 1)
        let point = Vector3(x: 1, y: 1, z: 1)
        let transformed = combined.transformPoint(point)
        
        // Expected: (1, 1, 1) scaled by 2 then translated: (2, 2, 2) + (1, 2, 3) = (3, 4, 5)
        #expect(abs(transformed.x - 3) < 1e-6)
        #expect(abs(transformed.y - 4) < 1e-6)
        #expect(abs(transformed.z - 5) < 1e-6)
    }
    
    // Test matrix inverse
    @Test func testMatrixInverse() {
        let translation = Matrix4x4.translation(Vector3(x: 1, y: 2, z: 3))
        let inverse = translation.inverse
        
        // The inverse of a translation matrix should have negated translation components
        #expect(abs(inverse.m[3][0] + 1) < 1e-6)
        #expect(abs(inverse.m[3][1] + 2) < 1e-6)
        #expect(abs(inverse.m[3][2] + 3) < 1e-6)
    }
}
