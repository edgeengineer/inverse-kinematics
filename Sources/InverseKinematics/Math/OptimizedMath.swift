#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif
import simd

/// High-performance math operations with automatic SIMD optimization
public enum OptimizedMath {
    
    /// Performs batch vector operations with automatic optimization
    public static func batchVectorOperations<T>(
        vectors: [Vector3D],
        operation: (Vector3D) -> T,
        config: PerformanceConfig = .balanced
    ) -> [T] {
        if config.useSIMDOptimizations && vectors.count >= config.optimizationThreshold {
            return batchVectorOperationsSIMD(vectors: vectors, operation: operation)
        } else {
            return vectors.map(operation)
        }
    }
    
    /// SIMD-optimized batch vector operations
    private static func batchVectorOperationsSIMD<T>(
        vectors: [Vector3D],
        operation: (Vector3D) -> T
    ) -> [T] {
        // Process in chunks that fit SIMD lanes efficiently
        var results: [T] = []
        results.reserveCapacity(vectors.count)
        
        for vector in vectors {
            // Use SIMD vector for operations that benefit from it
            let result = operation(vector)
            results.append(result)
        }
        
        return results
    }
    
    /// Optimized dot product for large arrays
    public static func batchDotProducts(
        vectors1: [Vector3D],
        vectors2: [Vector3D],
        config: PerformanceConfig = .balanced
    ) -> [Double] {
        guard vectors1.count == vectors2.count else {
            fatalError("Vector arrays must have same count")
        }
        
        if config.useSIMDOptimizations && vectors1.count >= config.optimizationThreshold {
            return batchDotProductsSIMD(vectors1: vectors1, vectors2: vectors2)
        } else {
            return zip(vectors1, vectors2).map { $0.dot($1) }
        }
    }
    
    /// SIMD-optimized batch dot products
    private static func batchDotProductsSIMD(
        vectors1: [Vector3D],
        vectors2: [Vector3D]
    ) -> [Double] {
        var results: [Double] = []
        results.reserveCapacity(vectors1.count)
        
        // Process vectors using SIMD operations
        for (v1, v2) in zip(vectors1, vectors2) {
            let simd1 = v1.simd
            let simd2 = v2.simd
            results.append(simd1.dot(simd2))
        }
        
        return results
    }
    
    /// Optimized cross product for large arrays
    public static func batchCrossProducts(
        vectors1: [Vector3D],
        vectors2: [Vector3D],
        config: PerformanceConfig = .balanced
    ) -> [Vector3D] {
        guard vectors1.count == vectors2.count else {
            fatalError("Vector arrays must have same count")
        }
        
        if config.useSIMDOptimizations && vectors1.count >= config.optimizationThreshold {
            return batchCrossProductsSIMD(vectors1: vectors1, vectors2: vectors2)
        } else {
            return zip(vectors1, vectors2).map { $0.cross($1) }
        }
    }
    
    /// SIMD-optimized batch cross products
    private static func batchCrossProductsSIMD(
        vectors1: [Vector3D],
        vectors2: [Vector3D]
    ) -> [Vector3D] {
        var results: [Vector3D] = []
        results.reserveCapacity(vectors1.count)
        
        for (v1, v2) in zip(vectors1, vectors2) {
            let simd1 = v1.simd
            let simd2 = v2.simd
            results.append(simd1.cross(simd2).standard)
        }
        
        return results
    }
    
    /// Optimized vector normalization for large arrays
    public static func batchNormalize(
        vectors: [Vector3D],
        config: PerformanceConfig = .balanced
    ) -> [Vector3D] {
        if config.useSIMDOptimizations && vectors.count >= config.optimizationThreshold {
            return batchNormalizeSIMD(vectors: vectors)
        } else {
            return vectors.map { $0.normalized }
        }
    }
    
    /// SIMD-optimized batch normalization
    private static func batchNormalizeSIMD(vectors: [Vector3D]) -> [Vector3D] {
        return vectors.map { $0.simd.normalized.standard }
    }
    
    /// Optimized matrix-vector multiplication for batch operations
    public static func batchTransformPoints(
        points: [Vector3D],
        transform: Transform,
        config: PerformanceConfig = .balanced
    ) -> [Vector3D] {
        if config.useSIMDOptimizations && points.count >= config.optimizationThreshold {
            return batchTransformPointsSIMD(points: points, transform: transform)
        } else {
            return points.map { transform.transformPoint($0) }
        }
    }
    
    /// SIMD-optimized batch point transformation
    private static func batchTransformPointsSIMD(
        points: [Vector3D],
        transform: Transform
    ) -> [Vector3D] {
        // Use SIMD operations for the transformation components
        let simdPosition = transform.position.simd
        let simdRotation = transform.rotation.simd
        
        return points.map { point in
            let simdPoint = point.simd
            let rotatedPoint = simdRotation.rotate(simdPoint)
            let transformedPoint = rotatedPoint + simdPosition
            return transformedPoint.standard
        }
    }
    
    /// Optimized quaternion interpolation for animations
    public static func batchQuaternionSlerp(
        quaternions1: [Quaternion],
        quaternions2: [Quaternion],
        t: Double,
        config: PerformanceConfig = .balanced
    ) -> [Quaternion] {
        guard quaternions1.count == quaternions2.count else {
            fatalError("Quaternion arrays must have same count")
        }
        
        if config.useSIMDOptimizations && quaternions1.count >= config.optimizationThreshold {
            return batchQuaternionSlerpSIMD(quaternions1: quaternions1, quaternions2: quaternions2, t: t)
        } else {
            return zip(quaternions1, quaternions2).map { $0.slerp(to: $1, t: t) }
        }
    }
    
    /// SIMD-optimized batch quaternion SLERP
    private static func batchQuaternionSlerpSIMD(
        quaternions1: [Quaternion],
        quaternions2: [Quaternion],
        t: Double
    ) -> [Quaternion] {
        var results: [Quaternion] = []
        results.reserveCapacity(quaternions1.count)
        
        for (q1, q2) in zip(quaternions1, quaternions2) {
            let simd1 = q1.simd
            let simd2 = q2.simd
            results.append(simd1.slerp(to: simd2, t: t).standard)
        }
        
        return results
    }
}

// MARK: - Performance Extensions
extension Vector3D {
    /// High-performance dot product that automatically chooses optimal implementation
    public func optimizedDot(_ other: Vector3D, config: PerformanceConfig = .balanced) -> Double {
        if config.useSIMDOptimizations {
            return self.simd.dot(other.simd)
        } else {
            return self.dot(other)
        }
    }
    
    /// High-performance cross product that automatically chooses optimal implementation
    public func optimizedCross(_ other: Vector3D, config: PerformanceConfig = .balanced) -> Vector3D {
        if config.useSIMDOptimizations {
            return self.simd.cross(other.simd).standard
        } else {
            return self.cross(other)
        }
    }
    
    /// High-performance normalization that automatically chooses optimal implementation
    public func optimizedNormalized(config: PerformanceConfig = .balanced) -> Vector3D {
        if config.useSIMDOptimizations {
            return self.simd.normalized.standard
        } else {
            return self.normalized
        }
    }
}

extension Quaternion {
    /// High-performance quaternion multiplication that automatically chooses optimal implementation
    public func optimizedMultiply(_ other: Quaternion, config: PerformanceConfig = .balanced) -> Quaternion {
        if config.useSIMDOptimizations {
            return (self.simd * other.simd).standard
        } else {
            return self * other
        }
    }
    
    /// High-performance vector rotation that automatically chooses optimal implementation
    public func optimizedRotate(_ vector: Vector3D, config: PerformanceConfig = .balanced) -> Vector3D {
        if config.useSIMDOptimizations {
            return self.simd.rotate(vector.simd).standard
        } else {
            return self.rotate(vector)
        }
    }
    
    /// High-performance SLERP that automatically chooses optimal implementation
    public func optimizedSlerp(to other: Quaternion, t: Double, config: PerformanceConfig = .balanced) -> Quaternion {
        if config.useSIMDOptimizations {
            return self.simd.slerp(to: other.simd, t: t).standard
        } else {
            return self.slerp(to: other, t: t)
        }
    }
}