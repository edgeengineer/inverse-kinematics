#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif

/// Performance configuration options for the inverse kinematics library
public struct PerformanceConfig: Sendable {
    /// Whether to use SIMD optimizations where beneficial
    public let useSIMDOptimizations: Bool
    
    /// Whether to enable parallel processing for batch operations
    public let enableParallelProcessing: Bool
    
    /// Numerical tolerance for optimization decisions
    public let optimizationTolerance: Double
    
    /// Threshold for switching to optimized algorithms based on problem size
    public let optimizationThreshold: Int
    
    public init(
        useSIMDOptimizations: Bool = true,
        enableParallelProcessing: Bool = true,
        optimizationTolerance: Double = 1e-10,
        optimizationThreshold: Int = 100
    ) {
        self.useSIMDOptimizations = useSIMDOptimizations
        self.enableParallelProcessing = enableParallelProcessing
        self.optimizationTolerance = optimizationTolerance
        self.optimizationThreshold = optimizationThreshold
    }
    
    /// Default high-performance configuration
    public static let highPerformance = PerformanceConfig(
        useSIMDOptimizations: true,
        enableParallelProcessing: true,
        optimizationTolerance: 1e-8,
        optimizationThreshold: 50
    )
    
    /// Default balanced configuration
    public static let balanced = PerformanceConfig(
        useSIMDOptimizations: true,
        enableParallelProcessing: false,
        optimizationTolerance: 1e-10,
        optimizationThreshold: 100
    )
    
    /// Default precision-focused configuration
    public static let precision = PerformanceConfig(
        useSIMDOptimizations: false,
        enableParallelProcessing: false,
        optimizationTolerance: 1e-15,
        optimizationThreshold: 1000
    )
}