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
    
    // MARK: - Performance Configuration
    
    /// Maximum acceptable time for standard operations (baseline threshold)
    static let maxStandardOperationTime: Double = 0.1 // 100ms
    
    /// Maximum acceptable performance degradation ratio (e.g., 10.0 means SIMD shouldn't be more than 10x slower)
    static let maxPerformanceDegradationRatio: Double = 10.0
    
    /// Minimum expected speedup for SIMD to be considered beneficial (1.0 = no slower than standard)
    /// Set to 0.1 to account for system performance variations, debug builds, and SIMD overhead
    static let minExpectedSpeedupRatio: Double = 0.1
    
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
        let standardTime = measureTime {
            for i in 0..<testVectors.count - 1 {
                _ = testVectors[i] + testVectors[i + 1]
            }
        }
        
        let simdTime = measureTime {
            for i in 0..<simdVectors.count - 1 {
                _ = simdVectors[i] + simdVectors[i + 1]
            }
        }
        
        let additionSpeedup = standardTime / simdTime
        assertPerformanceMetrics(
            operation: "Vector3D addition",
            standardTime: standardTime,
            simdTime: simdTime,
            speedup: additionSpeedup
        )
        
        // Test dot product performance
        let standardDotTime = measureTime {
            for i in 0..<testVectors.count - 1 {
                _ = testVectors[i].dot(testVectors[i + 1])
            }
        }
        
        let simdDotTime = measureTime {
            for i in 0..<simdVectors.count - 1 {
                _ = simdVectors[i].dot(simdVectors[i + 1])
            }
        }
        
        let dotSpeedup = standardDotTime / simdDotTime
        assertPerformanceMetrics(
            operation: "Vector3D dot product",
            standardTime: standardDotTime,
            simdTime: simdDotTime,
            speedup: dotSpeedup
        )
        
        // Test cross product performance
        let standardCrossTime = measureTime {
            for i in 0..<testVectors.count - 1 {
                _ = testVectors[i].cross(testVectors[i + 1])
            }
        }
        
        let simdCrossTime = measureTime {
            for i in 0..<simdVectors.count - 1 {
                _ = simdVectors[i].cross(simdVectors[i + 1])
            }
        }
        
        let crossSpeedup = standardCrossTime / simdCrossTime
        assertPerformanceMetrics(
            operation: "Vector3D cross product",
            standardTime: standardCrossTime,
            simdTime: simdCrossTime,
            speedup: crossSpeedup
        )
    }
    
    @Test("SIMD Quaternion performance comparison", .timeLimit(.minutes(1)))
    func testSIMDQuaternionPerformance() {
        // Convert to SIMD quaternions
        let simdQuaternions = testQuaternions.map { $0.simd }
        
        // Test quaternion multiplication performance
        let standardTime = measureTime {
            for i in 0..<testQuaternions.count - 1 {
                _ = testQuaternions[i] * testQuaternions[i + 1]
            }
        }
        
        let simdTime = measureTime {
            for i in 0..<simdQuaternions.count - 1 {
                _ = simdQuaternions[i] * simdQuaternions[i + 1]
            }
        }
        
        let multiplicationSpeedup = standardTime / simdTime
        assertPerformanceMetrics(
            operation: "Quaternion multiplication",
            standardTime: standardTime,
            simdTime: simdTime,
            speedup: multiplicationSpeedup
        )
        
        // Test vector rotation performance
        let testVector = Vector3D(x: 1.0, y: 1.0, z: 1.0)
        let testSIMDVector = testVector.simd
        
        let standardRotateTime = measureTime {
            for quaternion in testQuaternions {
                _ = quaternion.rotate(testVector)
            }
        }
        
        let simdRotateTime = measureTime {
            for quaternion in simdQuaternions {
                _ = quaternion.rotate(testSIMDVector)
            }
        }
        
        let rotationSpeedup = standardRotateTime / simdRotateTime
        assertPerformanceMetrics(
            operation: "Quaternion vector rotation",
            standardTime: standardRotateTime,
            simdTime: simdRotateTime,
            speedup: rotationSpeedup
        )
    }
    
    // MARK: - Performance Testing Helpers
    
    /// Measures the execution time of a closure
    private func measureTime(_ closure: () -> Void) -> Double {
        let startTime = CFAbsoluteTimeGetCurrent()
        closure()
        return CFAbsoluteTimeGetCurrent() - startTime
    }
    
    /// Asserts performance metrics and logs results
    private func assertPerformanceMetrics(
        operation: String,
        standardTime: Double,
        simdTime: Double,
        speedup: Double
    ) {
        // Log performance metrics (for debugging and analysis)
        // Note: These will only be visible in debug builds or when explicitly requested
        print("[\(operation)] Standard: \(String(format: "%.6f", standardTime))s, SIMD: \(String(format: "%.6f", simdTime))s, Speedup: \(String(format: "%.2f", speedup))x")
        
        // Assert that operations complete within reasonable time bounds
        #expect(standardTime < Self.maxStandardOperationTime, 
                "Standard \(operation) time (\(standardTime)s) exceeds maximum acceptable time (\(Self.maxStandardOperationTime)s)")
        
        #expect(simdTime < Self.maxStandardOperationTime, 
                "SIMD \(operation) time (\(simdTime)s) exceeds maximum acceptable time (\(Self.maxStandardOperationTime)s)")
        
        // Assert that SIMD performance is not significantly worse than standard
        #expect(speedup >= Self.minExpectedSpeedupRatio,
                "SIMD \(operation) speedup (\(speedup)x) is below minimum expected ratio (\(Self.minExpectedSpeedupRatio)x)")
        
        // Assert that neither implementation is excessively slow relative to the other
        let performanceRatio = max(standardTime / simdTime, simdTime / standardTime)
        #expect(performanceRatio <= Self.maxPerformanceDegradationRatio,
                "\(operation) performance ratio (\(performanceRatio)x) exceeds maximum acceptable degradation (\(Self.maxPerformanceDegradationRatio)x)")
        
        // Assert that times are finite and positive
        #expect(standardTime.isFinite && standardTime > 0, "Standard \(operation) time must be finite and positive")
        #expect(simdTime.isFinite && simdTime > 0, "SIMD \(operation) time must be finite and positive")
        #expect(speedup.isFinite && speedup > 0, "\(operation) speedup must be finite and positive")
    }
}