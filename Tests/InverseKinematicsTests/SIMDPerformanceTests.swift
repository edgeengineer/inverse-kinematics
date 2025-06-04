import Testing
#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif
@testable import InverseKinematics

@Suite("SIMD Performance Tests")
struct SIMDPerformanceTests {
    
    let testVectors: [Vector3D] = (0..<1000).map { i in
        Vector3D(x: Double(i), y: Double(i * 2), z: Double(i * 3))
    }
    
    let testQuaternions: [Quaternion] = (0..<1000).map { i in
        Quaternion(axis: Vector3D.unitZ, angle: Double(i) * 0.01)
    }
    
    @Test("SIMD Vector3D operations work correctly")
    func testSIMDVector3DCorrectness() {
        let v1 = Vector3D(x: 1.0, y: 2.0, z: 3.0)
        let v2 = Vector3D(x: 4.0, y: 5.0, z: 6.0)
        
        let simdV1 = v1.simd
        let simdV2 = v2.simd
        
        // Test addition
        let standardAdd = v1 + v2
        let simdAdd = (simdV1 + simdV2).standard
        
        #expect(abs(standardAdd.x - simdAdd.x) < 1e-10)
        #expect(abs(standardAdd.y - simdAdd.y) < 1e-10)
        #expect(abs(standardAdd.z - simdAdd.z) < 1e-10)
        
        // Test dot product
        let standardDot = v1.dot(v2)
        let simdDot = simdV1.dot(simdV2)
        
        #expect(abs(standardDot - simdDot) < 1e-10)
        
        // Test cross product
        let standardCross = v1.cross(v2)
        let simdCross = simdV1.cross(simdV2).standard
        
        #expect(abs(standardCross.x - simdCross.x) < 1e-10)
        #expect(abs(standardCross.y - simdCross.y) < 1e-10)
        #expect(abs(standardCross.z - simdCross.z) < 1e-10)
        
        // Test magnitude
        let standardMag = v1.magnitude
        let simdMag = simdV1.magnitude
        
        #expect(abs(standardMag - simdMag) < 1e-10)
        
        // Test normalization
        let standardNorm = v1.normalized
        let simdNorm = simdV1.normalized.standard
        
        #expect(abs(standardNorm.x - simdNorm.x) < 1e-10)
        #expect(abs(standardNorm.y - simdNorm.y) < 1e-10)
        #expect(abs(standardNorm.z - simdNorm.z) < 1e-10)
    }
    
    @Test("SIMD Quaternion operations work correctly")
    func testSIMDQuaternionCorrectness() {
        let q1 = Quaternion(axis: Vector3D.unitZ, angle: Double.pi / 4)
        let q2 = Quaternion(axis: Vector3D.unitY, angle: Double.pi / 6)
        
        let simdQ1 = q1.simd
        let simdQ2 = q2.simd
        
        // Test multiplication
        let standardMul = q1 * q2
        let simdMul = (simdQ1 * simdQ2).standard
        
        #expect(abs(standardMul.x - simdMul.x) < 1e-10)
        #expect(abs(standardMul.y - simdMul.y) < 1e-10)
        #expect(abs(standardMul.z - simdMul.z) < 1e-10)
        #expect(abs(standardMul.w - simdMul.w) < 1e-10)
        
        // Test conjugate
        let standardConj = q1.conjugate
        let simdConj = simdQ1.conjugate.standard
        
        #expect(abs(standardConj.x - simdConj.x) < 1e-10)
        #expect(abs(standardConj.y - simdConj.y) < 1e-10)
        #expect(abs(standardConj.z - simdConj.z) < 1e-10)
        #expect(abs(standardConj.w - simdConj.w) < 1e-10)
        
        // Test vector rotation
        let vector = Vector3D(x: 1.0, y: 0.0, z: 0.0)
        let standardRotated = q1.rotate(vector)
        let simdRotated = simdQ1.rotate(vector.simd).standard
        
        #expect(abs(standardRotated.x - simdRotated.x) < 1e-10)
        #expect(abs(standardRotated.y - simdRotated.y) < 1e-10)
        #expect(abs(standardRotated.z - simdRotated.z) < 1e-10)
    }
    
    @Test("SIMD Vector performance comparison", .timeLimit(.minutes(1)))
    func testSIMDVectorPerformance() {
        // Convert to SIMD vectors
        let simdVectors = testVectors.map { $0.simd }
        
        // Test vector addition performance
        let startTimeStandard = CFAbsoluteTimeGetCurrent()
        for i in 0..<testVectors.count - 1 {
            _ = testVectors[i] + testVectors[i + 1]
        }
        let standardTime = CFAbsoluteTimeGetCurrent() - startTimeStandard
        
        let startTimeSIMD = CFAbsoluteTimeGetCurrent()
        for i in 0..<simdVectors.count - 1 {
            _ = simdVectors[i] + simdVectors[i + 1]
        }
        let simdTime = CFAbsoluteTimeGetCurrent() - startTimeSIMD
        
        print("Standard Vector3D addition time: \(standardTime)s")
        print("SIMD Vector3D addition time: \(simdTime)s")
        print("SIMD speedup: \(standardTime / simdTime)x")
        
        // Test dot product performance
        let startTimeStandardDot = CFAbsoluteTimeGetCurrent()
        for i in 0..<testVectors.count - 1 {
            _ = testVectors[i].dot(testVectors[i + 1])
        }
        let standardDotTime = CFAbsoluteTimeGetCurrent() - startTimeStandardDot
        
        let startTimeSIMDDot = CFAbsoluteTimeGetCurrent()
        for i in 0..<simdVectors.count - 1 {
            _ = simdVectors[i].dot(simdVectors[i + 1])
        }
        let simdDotTime = CFAbsoluteTimeGetCurrent() - startTimeSIMDDot
        
        print("Standard Vector3D dot product time: \(standardDotTime)s")
        print("SIMD Vector3D dot product time: \(simdDotTime)s")
        print("SIMD dot product speedup: \(standardDotTime / simdDotTime)x")
        
        // Test cross product performance
        let startTimeStandardCross = CFAbsoluteTimeGetCurrent()
        for i in 0..<testVectors.count - 1 {
            _ = testVectors[i].cross(testVectors[i + 1])
        }
        let standardCrossTime = CFAbsoluteTimeGetCurrent() - startTimeStandardCross
        
        let startTimeSIMDCross = CFAbsoluteTimeGetCurrent()
        for i in 0..<simdVectors.count - 1 {
            _ = simdVectors[i].cross(simdVectors[i + 1])
        }
        let simdCrossTime = CFAbsoluteTimeGetCurrent() - startTimeSIMDCross
        
        print("Standard Vector3D cross product time: \(standardCrossTime)s")
        print("SIMD Vector3D cross product time: \(simdCrossTime)s")
        print("SIMD cross product speedup: \(standardCrossTime / simdCrossTime)x")
    }
    
    @Test("SIMD Quaternion performance comparison", .timeLimit(.minutes(1)))
    func testSIMDQuaternionPerformance() {
        // Convert to SIMD quaternions
        let simdQuaternions = testQuaternions.map { $0.simd }
        
        // Test quaternion multiplication performance
        let startTimeStandard = CFAbsoluteTimeGetCurrent()
        for i in 0..<testQuaternions.count - 1 {
            _ = testQuaternions[i] * testQuaternions[i + 1]
        }
        let standardTime = CFAbsoluteTimeGetCurrent() - startTimeStandard
        
        let startTimeSIMD = CFAbsoluteTimeGetCurrent()
        for i in 0..<simdQuaternions.count - 1 {
            _ = simdQuaternions[i] * simdQuaternions[i + 1]
        }
        let simdTime = CFAbsoluteTimeGetCurrent() - startTimeSIMD
        
        print("Standard Quaternion multiplication time: \(standardTime)s")
        print("SIMD Quaternion multiplication time: \(simdTime)s")
        print("SIMD quaternion speedup: \(standardTime / simdTime)x")
        
        // Test vector rotation performance
        let testVector = Vector3D(x: 1.0, y: 1.0, z: 1.0)
        let testSIMDVector = testVector.simd
        
        let startTimeStandardRotate = CFAbsoluteTimeGetCurrent()
        for quaternion in testQuaternions {
            _ = quaternion.rotate(testVector)
        }
        let standardRotateTime = CFAbsoluteTimeGetCurrent() - startTimeStandardRotate
        
        let startTimeSIMDRotate = CFAbsoluteTimeGetCurrent()
        for quaternion in simdQuaternions {
            _ = quaternion.rotate(testSIMDVector)
        }
        let simdRotateTime = CFAbsoluteTimeGetCurrent() - startTimeSIMDRotate
        
        print("Standard Quaternion vector rotation time: \(standardRotateTime)s")
        print("SIMD Quaternion vector rotation time: \(simdRotateTime)s")
        print("SIMD rotation speedup: \(standardRotateTime / simdRotateTime)x")
    }
}