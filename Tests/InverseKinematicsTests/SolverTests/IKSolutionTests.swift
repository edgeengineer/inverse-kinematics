import Testing
#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif
@testable import InverseKinematics

@Suite("IK Solution Tests")
struct IKSolutionTests {
    
    // MARK: - Basic Initialization Tests
    
    @Test("IK solution initialization")
    func testIKSolutionInit() {
        let solution = IKSolution(
            jointValues: [1.0, 2.0, 3.0],
            success: true,
            error: 1e-6,
            iterations: 15,
            algorithm: .dampedLeastSquares,
            convergenceHistory: [1.0, 0.5, 1e-6]
        )
        
        #expect(solution.jointValues == [1.0, 2.0, 3.0])
        #expect(solution.success == true)
        #expect(solution.error == 1e-6)
        #expect(solution.iterations == 15)
        #expect(solution.algorithm == .dampedLeastSquares)
        #expect(solution.convergenceHistory?.count == 3)
    }
    
    @Test("IK solution failed static instance")
    func testIKSolutionFailed() {
        let failed = IKSolution.failed
        #expect(failed.success == false)
        #expect(failed.error == .infinity)
        #expect(failed.iterations == 0)
        #expect(failed.jointValues.isEmpty)
        #expect(failed.algorithm == .analytical) // Default algorithm for failed
        #expect(failed.convergenceHistory == nil)
    }
    
    // MARK: - Convergence History Tests
    
    @Test("IK solution with empty convergence history")
    func testIKSolutionEmptyConvergenceHistory() {
        let solution = IKSolution(
            jointValues: [0.5, 1.0],
            success: true,
            error: 1e-4,
            iterations: 5,
            algorithm: .jacobianTranspose,
            convergenceHistory: []
        )
        
        #expect(solution.convergenceHistory?.isEmpty == true)
    }
    
    @Test("IK solution with nil convergence history")
    func testIKSolutionNilConvergenceHistory() {
        let solution = IKSolution(
            jointValues: [0.0, 0.0],
            success: false,
            error: 1e-2,
            iterations: 100,
            algorithm: .fabrik,
            convergenceHistory: nil
        )
        
        #expect(solution.convergenceHistory == nil)
    }
    
    @Test("IK solution convergence history format and detail")
    func testIKSolutionConvergenceHistoryDetail() {
        // Simulate a typical convergence pattern
        let convergenceData = [1.0, 0.5, 0.25, 0.1, 0.05, 1e-3, 1e-5]
        
        let solution = IKSolution(
            jointValues: [Double.pi/4, Double.pi/6],
            success: true,
            error: 1e-5,
            iterations: 7,
            algorithm: .dampedLeastSquares,
            convergenceHistory: convergenceData
        )
        
        guard let history = solution.convergenceHistory else {
            #expect(Bool(false), "Convergence history should not be nil")
            return
        }
        
        #expect(history.count == 7)
        #expect(history.first == 1.0)
        #expect(history.last == 1e-5)
        
        // Verify generally decreasing trend (with some tolerance for oscillation)
        let majorDecreases = zip(history.dropLast(), history.dropFirst()).filter { $0.0 > $0.1 }.count
        #expect(majorDecreases >= history.count / 2) // At least half should show decrease
    }
    
    // MARK: - Algorithm Type Tests
    
    @Test("IK solution with different algorithm types")
    func testIKSolutionDifferentAlgorithms() {
        let algorithms: [IKAlgorithmType] = [
            .analytical,
            .jacobianTranspose,
            .dampedLeastSquares,
            .selectivelyDampedLeastSquares,
            .cyclicCoordinateDescent,
            .fabrik,
            .newtonRaphson
        ]
        
        for algorithm in algorithms {
            let solution = IKSolution(
                jointValues: [1.0],
                success: true,
                error: 1e-6,
                iterations: 10,
                algorithm: algorithm
            )
            
            #expect(solution.algorithm == algorithm)
        }
    }
    
    // MARK: - Error Validation Tests
    
    @Test("IK solution error validation - negative errors become zero")
    func testIKSolutionNegativeErrorValidation() {
        let solution = IKSolution(
            jointValues: [0.0],
            success: false,
            error: -1.5, // Negative error should be clamped
            iterations: 0,
            algorithm: .analytical
        )
        
        #expect(solution.error == 0.0) // Should be clamped to zero
    }
    
    @Test("IK solution iterations validation - negative iterations become zero")
    func testIKSolutionNegativeIterationsValidation() {
        let solution = IKSolution(
            jointValues: [0.0],
            success: false,
            error: 1.0,
            iterations: -5, // Negative iterations should be clamped
            algorithm: .analytical
        )
        
        #expect(solution.iterations == 0) // Should be clamped to zero
    }
    
    // MARK: - Boundary Value Tests
    
    @Test("IK solution with zero iterations")
    func testIKSolutionZeroIterations() {
        let solution = IKSolution(
            jointValues: [1.0, 2.0],
            success: true,
            error: 0.0,
            iterations: 0, // Analytical solver might solve in zero iterations
            algorithm: .analytical
        )
        
        #expect(solution.iterations == 0)
        #expect(solution.success == true)
    }
    
    @Test("IK solution with maximum reasonable iterations")
    func testIKSolutionMaxIterations() {
        let solution = IKSolution(
            jointValues: [0.1, 0.2, 0.3],
            success: false,
            error: 1e-3,
            iterations: 1000, // High iteration count
            algorithm: .dampedLeastSquares
        )
        
        #expect(solution.iterations == 1000)
        #expect(solution.success == false) // Didn't converge despite many iterations
    }
    
    @Test("IK solution with very small error")
    func testIKSolutionVerySmallError() {
        let solution = IKSolution(
            jointValues: [0.0, 0.0],
            success: true,
            error: 1e-15, // Very small error
            iterations: 5,
            algorithm: .analytical
        )
        
        #expect(solution.error == 1e-15)
        #expect(solution.success == true)
    }
    
    // MARK: - Joint Values Tests
    
    @Test("IK solution with empty joint values")
    func testIKSolutionEmptyJointValues() {
        let solution = IKSolution(
            jointValues: [],
            success: false,
            error: .infinity,
            iterations: 0,
            algorithm: .analytical
        )
        
        #expect(solution.jointValues.isEmpty)
        #expect(solution.success == false)
    }
    
    @Test("IK solution with single joint value")
    func testIKSolutionSingleJoint() {
        let solution = IKSolution(
            jointValues: [Double.pi/2],
            success: true,
            error: 1e-8,
            iterations: 3,
            algorithm: .analytical
        )
        
        #expect(solution.jointValues.count == 1)
        #expect(solution.jointValues[0] == Double.pi/2)
    }
    
    @Test("IK solution with many joint values")
    func testIKSolutionManyJoints() {
        let jointValues = Array(0..<10).map { Double($0) * 0.1 }
        
        let solution = IKSolution(
            jointValues: jointValues,
            success: true,
            error: 1e-4,
            iterations: 25,
            algorithm: .jacobianTranspose
        )
        
        #expect(solution.jointValues.count == 10)
        #expect(solution.jointValues == jointValues)
    }
    
    // MARK: - Success/Failure Consistency Tests
    
    @Test("IK solution success with low error")
    func testIKSolutionSuccessConsistency() {
        let solution = IKSolution(
            jointValues: [1.0, 2.0],
            success: true,
            error: 1e-8, // Very low error
            iterations: 12,
            algorithm: .dampedLeastSquares
        )
        
        #expect(solution.success == true)
        #expect(solution.error < 1e-6) // Should be very accurate for successful solution
    }
    
    @Test("IK solution failure with high error")
    func testIKSolutionFailureConsistency() {
        let solution = IKSolution(
            jointValues: [0.0, 0.0],
            success: false,
            error: 5.0, // High error indicating failure
            iterations: 100,
            algorithm: .cyclicCoordinateDescent
        )
        
        #expect(solution.success == false)
        #expect(solution.error > 1e-3) // Should have significant error for failed solution
    }
    
    // MARK: - Special Values Tests
    
    @Test("IK solution with infinite error")
    func testIKSolutionInfiniteError() {
        let solution = IKSolution(
            jointValues: [],
            success: false,
            error: .infinity,
            iterations: 0,
            algorithm: .fabrik
        )
        
        #expect(solution.error == .infinity)
        #expect(solution.success == false)
    }
    
    @Test("IK solution with NaN joint values") 
    func testIKSolutionNaNJointValues() {
        let solution = IKSolution(
            jointValues: [Double.nan, 1.0, Double.nan],
            success: false,
            error: .infinity,
            iterations: 0,
            algorithm: .analytical
        )
        
        #expect(solution.jointValues.count == 3)
        #expect(solution.jointValues[0].isNaN)
        #expect(solution.jointValues[1] == 1.0)
        #expect(solution.jointValues[2].isNaN)
        #expect(solution.success == false)
    }
    
    // MARK: - Convergence Tests
    
    @Test("IK solution convergence with custom tolerance")
    func testIKSolutionConvergenceCustomTolerance() {
        // Test solution that converged within tolerance
        let convergedSolution = IKSolution(
            jointValues: [1.0, 2.0, 3.0],
            success: true,
            error: 1e-8,
            iterations: 15,
            algorithm: .dampedLeastSquares
        )
        
        #expect(convergedSolution.isConverged(tolerance: 1e-6))
        #expect(convergedSolution.isConverged(tolerance: 1e-7))  // 1e-8 <= 1e-7 should pass
        #expect(!convergedSolution.isConverged(tolerance: 1e-10)) // 1e-8 > 1e-10 should fail
        
        // Test solution that did not converge within tolerance
        let unconvergedSolution = IKSolution(
            jointValues: [0.5, 1.5],
            success: false,
            error: 1e-3,
            iterations: 100,
            algorithm: .jacobianTranspose
        )
        
        #expect(unconvergedSolution.isConverged(tolerance: 1e-2))
        #expect(!unconvergedSolution.isConverged(tolerance: 1e-4))
        #expect(!unconvergedSolution.isConverged(tolerance: 1e-6))
    }
    
    @Test("IK solution convergence with default tolerance")
    func testIKSolutionConvergenceDefaultTolerance() {
        // Test with default IKParameters tolerance (1e-6)
        let goodSolution = IKSolution(
            jointValues: [1.0, 2.0],
            success: true,
            error: 1e-8,
            iterations: 10,
            algorithm: .analytical
        )
        
        #expect(goodSolution.isConverged()) // Should use default tolerance
        
        let poorSolution = IKSolution(
            jointValues: [1.0, 2.0],
            success: false,
            error: 1e-4,
            iterations: 50,
            algorithm: .cyclicCoordinateDescent
        )
        
        #expect(!poorSolution.isConverged()) // Should not converge with default tolerance
    }
    
    @Test("IK solution convergence with invalid cases")
    func testIKSolutionConvergenceInvalidCases() {
        // Test solution with empty joint values
        let emptySolution = IKSolution(
            jointValues: [],
            success: false,
            error: 1e-8,
            iterations: 0,
            algorithm: .analytical
        )
        
        #expect(!emptySolution.isConverged(tolerance: 1e-6))
        #expect(!emptySolution.isConverged())
        
        // Test solution with infinite error
        let infiniteErrorSolution = IKSolution(
            jointValues: [1.0, 2.0],
            success: false,
            error: .infinity,
            iterations: 0,
            algorithm: .fabrik
        )
        
        #expect(!infiniteErrorSolution.isConverged(tolerance: 1e-6))
        #expect(!infiniteErrorSolution.isConverged())
        
        // Test solution with NaN error
        let nanErrorSolution = IKSolution(
            jointValues: [1.0, 2.0],
            success: false,
            error: .nan,
            iterations: 0,
            algorithm: .newtonRaphson
        )
        
        // Note: max(0, .nan) returns 0.0, so error is clamped to 0
        #expect(nanErrorSolution.error == 0.0)
        #expect(nanErrorSolution.isConverged(tolerance: 1e-6)) // 0.0 <= 1e-6 is true
        #expect(nanErrorSolution.isConverged()) // 0.0 <= default tolerance is true
    }
    
    @Test("IK solution convergence analysis with good convergence")
    func testIKSolutionConvergenceAnalysisGoodConvergence() {
        // Test with typical good convergence pattern
        let convergenceHistory = [1.0, 0.5, 0.25, 0.1, 0.05, 0.01, 1e-6]
        
        let solution = IKSolution(
            jointValues: [1.0, 2.0, 3.0],
            success: true,
            error: 1e-6,
            iterations: 7,
            algorithm: .dampedLeastSquares,
            convergenceHistory: convergenceHistory
        )
        
        let analysis = solution.convergenceAnalysis()
        
        #expect(analysis.hasImproved == true)
        #expect(analysis.finalImprovement != nil)
        #expect(analysis.averageImprovement != nil)
        
        if let finalImprovement = analysis.finalImprovement {
            #expect(SolverTestUtils.isApproximatelyEqual(finalImprovement, 1.0 - 1e-6, tolerance: 1e-5))
        }
        
        if let avgImprovement = analysis.averageImprovement {
            #expect(avgImprovement > 0) // Should have positive average improvement
        }
    }
    
    @Test("IK solution convergence analysis with poor convergence")
    func testIKSolutionConvergenceAnalysisPoorConvergence() {
        // Test with convergence that got worse
        let convergenceHistory = [0.01, 0.02, 0.05, 0.1, 0.2, 0.5]
        
        let solution = IKSolution(
            jointValues: [1.0, 2.0],
            success: false,
            error: 0.5,
            iterations: 6,
            algorithm: .cyclicCoordinateDescent,
            convergenceHistory: convergenceHistory
        )
        
        let analysis = solution.convergenceAnalysis()
        
        #expect(analysis.hasImproved == false) // Final error is worse than initial
        #expect(analysis.finalImprovement != nil)
        #expect(analysis.averageImprovement == nil) // No positive improvements
        
        if let finalImprovement = analysis.finalImprovement {
            #expect(finalImprovement < 0) // Should be negative (got worse)
        }
    }
    
    @Test("IK solution convergence analysis with oscillating convergence")
    func testIKSolutionConvergenceAnalysisOscillating() {
        // Test with oscillating convergence pattern
        let convergenceHistory = [1.0, 0.8, 1.2, 0.6, 0.9, 0.4, 0.7, 0.2]
        
        let solution = IKSolution(
            jointValues: [0.5, 1.5, 2.5],
            success: true,
            error: 0.2,
            iterations: 8,
            algorithm: .jacobianTranspose,
            convergenceHistory: convergenceHistory
        )
        
        let analysis = solution.convergenceAnalysis()
        
        #expect(analysis.hasImproved == true) // Final < initial
        #expect(analysis.finalImprovement != nil)
        #expect(analysis.averageImprovement != nil)
        
        if let finalImprovement = analysis.finalImprovement {
            #expect(SolverTestUtils.isApproximatelyEqual(finalImprovement, 1.0 - 0.2, tolerance: 1e-10))
        }
        
        if let avgImprovement = analysis.averageImprovement {
            #expect(avgImprovement > 0) // Should still have positive average improvement
        }
    }
    
    @Test("IK solution convergence analysis with no history")
    func testIKSolutionConvergenceAnalysisNoHistory() {
        // Test solution with nil convergence history
        let solutionNoHistory = IKSolution(
            jointValues: [1.0, 2.0],
            success: true,
            error: 1e-6,
            iterations: 10,
            algorithm: .analytical,
            convergenceHistory: nil
        )
        
        let analysisNoHistory = solutionNoHistory.convergenceAnalysis()
        
        #expect(analysisNoHistory.hasImproved == false)
        #expect(analysisNoHistory.finalImprovement == nil)
        #expect(analysisNoHistory.averageImprovement == nil)
        
        // Test solution with empty convergence history
        let solutionEmptyHistory = IKSolution(
            jointValues: [1.0, 2.0],
            success: true,
            error: 1e-6,
            iterations: 10,
            algorithm: .analytical,
            convergenceHistory: []
        )
        
        let analysisEmptyHistory = solutionEmptyHistory.convergenceAnalysis()
        
        #expect(analysisEmptyHistory.hasImproved == false)
        #expect(analysisEmptyHistory.finalImprovement == nil)
        #expect(analysisEmptyHistory.averageImprovement == nil)
    }
    
    @Test("IK solution convergence analysis with single history point")
    func testIKSolutionConvergenceAnalysisSinglePoint() {
        // Test with single convergence history point
        let solutionSinglePoint = IKSolution(
            jointValues: [1.0, 2.0],
            success: true,
            error: 1e-6,
            iterations: 1,
            algorithm: .analytical,
            convergenceHistory: [1e-6]
        )
        
        let analysisSinglePoint = solutionSinglePoint.convergenceAnalysis()
        
        #expect(analysisSinglePoint.hasImproved == false)
        #expect(analysisSinglePoint.finalImprovement == nil)
        #expect(analysisSinglePoint.averageImprovement == nil)
    }
    
    @Test("IK solution convergence analysis with flat convergence")
    func testIKSolutionConvergenceAnalysisFlatConvergence() {
        // Test with flat convergence (no improvement)
        let convergenceHistory = [0.1, 0.1, 0.1, 0.1, 0.1]
        
        let solution = IKSolution(
            jointValues: [1.0, 2.0],
            success: false,
            error: 0.1,
            iterations: 5,
            algorithm: .fabrik,
            convergenceHistory: convergenceHistory
        )
        
        let analysis = solution.convergenceAnalysis()
        
        #expect(analysis.hasImproved == false) // No change
        #expect(analysis.finalImprovement != nil)
        #expect(analysis.averageImprovement == nil) // No positive improvements
        
        if let finalImprovement = analysis.finalImprovement {
            #expect(SolverTestUtils.isApproximatelyEqual(finalImprovement, 0.0, tolerance: 1e-10))
        }
    }
    
    @Test("IK solution convergence analysis edge cases")
    func testIKSolutionConvergenceAnalysisEdgeCases() {
        // Test with very small improvements
        let tinyImprovements = [1e-10, 1e-11, 1e-12, 1e-13]
        
        let solutionTiny = IKSolution(
            jointValues: [1.0],
            success: true,
            error: 1e-13,
            iterations: 4,
            algorithm: .selectivelyDampedLeastSquares,
            convergenceHistory: tinyImprovements
        )
        
        let analysisTiny = solutionTiny.convergenceAnalysis()
        
        #expect(analysisTiny.hasImproved == true)
        #expect(analysisTiny.finalImprovement != nil)
        #expect(analysisTiny.averageImprovement != nil)
        
        // Test with large errors
        let largeErrors = [1e10, 1e8, 1e6, 1e4]
        
        let solutionLarge = IKSolution(
            jointValues: [1.0],
            success: false,
            error: 1e4,
            iterations: 4,
            algorithm: .newtonRaphson,
            convergenceHistory: largeErrors
        )
        
        let analysisLarge = solutionLarge.convergenceAnalysis()
        
        #expect(analysisLarge.hasImproved == true)
        #expect(analysisLarge.finalImprovement != nil)
        #expect(analysisLarge.averageImprovement != nil)
        
        if let finalImprovement = analysisLarge.finalImprovement {
            #expect(finalImprovement > 0)
            #expect(finalImprovement.isFinite)
        }
    }
    
    @Test("IK solution convergence tolerance boundary conditions")
    func testIKSolutionConvergenceToleranceBoundaries() {
        let solution = IKSolution(
            jointValues: [1.0, 2.0],
            success: true,
            error: 1e-6,
            iterations: 10,
            algorithm: .dampedLeastSquares
        )
        
        // Test exact tolerance boundary
        #expect(solution.isConverged(tolerance: 1e-6))    // Equal to error (should pass)
        #expect(!solution.isConverged(tolerance: 1e-7))   // Stricter than error (should fail)
        
        // Test very small tolerances
        #expect(!solution.isConverged(tolerance: 1e-15))
        #expect(!solution.isConverged(tolerance: Double.ulpOfOne))
        
        // Test very large tolerances
        #expect(solution.isConverged(tolerance: 1.0))
        #expect(solution.isConverged(tolerance: 1e10))
        
        // Test zero tolerance (should fail for any non-zero error)
        #expect(!solution.isConverged(tolerance: 0.0))
        
        // Test negative tolerance (implementation clamps to 0 in IKParameters)
        #expect(!solution.isConverged(tolerance: -1e-6))
    }
}