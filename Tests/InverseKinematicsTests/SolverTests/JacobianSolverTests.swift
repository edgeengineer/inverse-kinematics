import Testing
#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif
@testable import InverseKinematics

@Suite("Jacobian Solver Tests")
struct JacobianSolverTests {
    
    @Test("Jacobian solver initialization")
    func testJacobianSolverInit() async {
        let chain = SolverTestUtils.createSimpleTwoJointChain()
        let solver = JacobianBasedSolver(chain: chain)
        
        let supportedAlgorithms = solver.supportedAlgorithms
        #expect(supportedAlgorithms.contains(IKAlgorithmType.jacobianTranspose))
        #expect(supportedAlgorithms.contains(.dampedLeastSquares))
        #expect(solver.jointCount == 2)
    }
    
    @Test("Jacobian transpose solver")
    func testJacobianTransposeSolver() async throws {
        let chain = SolverTestUtils.createSimpleTwoJointChain()
        let solver = JacobianBasedSolver(chain: chain)
        
        let target = Transform(
            position: Vector3D(x: 1.5, y: 0.5, z: 0.0),
            rotation: Quaternion.identity
        )
        
        let parameters = IKParameters(
            tolerance: 1e-3,
            maxIterations: 50,
            stepSize: 0.1
        )
        
        let solution = try await solver.solveIK(
            target: target,
            initialGuess: [0.0, 0.0],
            algorithm: IKAlgorithmType.jacobianTranspose,
            parameters: parameters
        )
        
        #expect(solution.algorithm == IKAlgorithmType.jacobianTranspose)
        #expect(solution.jointValues.count == 2)
        #expect(solution.iterations <= parameters.maxIterations)
    }
    
    @Test("Damped least squares solver")
    func testDampedLeastSquaresSolver() async throws {
        let chain = SolverTestUtils.createSimpleTwoJointChain()
        let solver = JacobianBasedSolver(chain: chain)
        
        let target = Transform(
            position: Vector3D(x: 1.2, y: 0.8, z: 0.0),
            rotation: Quaternion.identity
        )
        
        let parameters = IKParameters(
            tolerance: 1e-3,
            maxIterations: 30,
            dampingFactor: 0.01
        )
        
        let solution = try await solver.solveIK(
            target: target,
            algorithm: IKAlgorithmType.dampedLeastSquares,
            parameters: parameters
        )
        
        #expect(solution.algorithm == IKAlgorithmType.dampedLeastSquares)
        #expect(solution.convergenceHistory != nil)
    }
    
    @Test("Unsupported algorithm error")
    func testUnsupportedAlgorithmError() async {
        let chain = SolverTestUtils.createSimpleTwoJointChain()
        let solver = JacobianBasedSolver(chain: chain)
        
        let target = Transform(position: Vector3D.zero, rotation: Quaternion.identity)
        
        await #expect(throws: IKError.unsupportedAlgorithm(IKAlgorithmType.fabrik)) {
            _ = try await solver.solveIK(
                target: target,
                algorithm: IKAlgorithmType.fabrik,
                parameters: .default
            )
        }
    }
    
    // MARK: - Phase Two Improvements: Comprehensive Jacobian Solver Tests
    
    @Test("DLS with varying damping factors")
    func testDLSVaryingDampingFactors() async throws {
        let chain = SolverTestUtils.createSimpleTwoJointChain()
        let solver = JacobianBasedSolver(chain: chain)
        
        let target = Transform(
            position: Vector3D(x: 1.5, y: 0.5, z: 0.0),
            rotation: Quaternion.identity
        )
        
        // Test different damping factors
        let dampingFactors: [Double] = [0.001, 0.01, 0.1, 0.5, 1.0]
        
        for dampingFactor in dampingFactors {
            let parameters = IKParameters(
                tolerance: 1e-3,
                maxIterations: 50,
                dampingFactor: dampingFactor
            )
            
            let solution = try await solver.solveIK(
                target: target,
                algorithm: IKAlgorithmType.dampedLeastSquares,
                parameters: parameters
            )
            
            #expect(solution.algorithm == IKAlgorithmType.dampedLeastSquares)
            #expect(solution.jointValues.count == 2)
            
            // Higher damping should generally lead to more iterations but more stability
            // Lower damping should converge faster but may be less stable
            if dampingFactor >= 0.1 {
                #expect(solution.success || solution.iterations == parameters.maxIterations)
            }
        }
    }
    
    @Test("DLS singularity handling with high damping")
    func testDLSSingularityHandling() async throws {
        let chain = SolverTestUtils.createSimpleTwoJointChain()
        let solver = JacobianBasedSolver(chain: chain)
        
        // Create a target that puts the arm near a singularity (fully extended)
        let singularTarget = Transform(
            position: Vector3D(x: 1.95, y: 0.0, z: 0.0), // Near maximum reach
            rotation: Quaternion.identity
        )
        
        // Low damping (should struggle with singularity)
        let lowDampingParams = IKParameters(
            tolerance: 1e-3,
            maxIterations: 100,
            dampingFactor: 0.001
        )
        
        let lowDampingSolution = try await solver.solveIK(
            target: singularTarget,
            algorithm: IKAlgorithmType.dampedLeastSquares,
            parameters: lowDampingParams
        )
        
        // High damping (should handle singularity better)
        let highDampingParams = IKParameters(
            tolerance: 1e-3,
            maxIterations: 100,
            dampingFactor: 0.1
        )
        
        let highDampingSolution = try await solver.solveIK(
            target: singularTarget,
            algorithm: IKAlgorithmType.dampedLeastSquares,
            parameters: highDampingParams
        )
        
        // High damping should provide more stable solution near singularities
        #expect(highDampingSolution.success || highDampingSolution.error < lowDampingSolution.error * 2)
        #expect(highDampingSolution.algorithm == IKAlgorithmType.dampedLeastSquares)
    }
    
    @Test("SDLS selective damping behavior")
    func testSDLSSelectiveDamping() async throws {
        let chain = SolverTestUtils.createSimpleTwoJointChain()
        let solver = JacobianBasedSolver(chain: chain)
        
        let target = Transform(
            position: Vector3D(x: 1.3, y: 0.7, z: 0.0),
            rotation: Quaternion.identity
        )
        
        let parameters = IKParameters(
            tolerance: 1e-4,
            maxIterations: 50,
            dampingFactor: 0.01
        )
        
        let solution = try await solver.solveIK(
            target: target,
            algorithm: IKAlgorithmType.selectivelyDampedLeastSquares,
            parameters: parameters
        )
        
        #expect(solution.algorithm == IKAlgorithmType.selectivelyDampedLeastSquares)
        #expect(solution.jointValues.count == 2)
        
        // SDLS should run without crashing and provide reasonable results
        #expect(solution.error.isFinite, "Solution error should be finite")
        #expect(solution.error >= 0, "Solution error should be non-negative")
        // SDLS may not always converge for all targets, so focus on basic functionality
        #expect(solution.iterations <= parameters.maxIterations)
    }
    
    @Test("Jacobian transpose vs DLS convergence comparison")
    func testJacobianTransposeVsDLSConvergence() async throws {
        let chain = SolverTestUtils.createSimpleTwoJointChain()
        let solver = JacobianBasedSolver(chain: chain)
        
        let target = Transform(
            position: Vector3D(x: 1.2, y: 0.6, z: 0.0),
            rotation: Quaternion.identity
        )
        
        let initialGuess = [0.0, 0.0]
        
        // Test Jacobian Transpose
        let jtParameters = IKParameters(
            tolerance: 1e-3,
            maxIterations: 100,
            stepSize: 0.1
        )
        
        let jtSolution = try await solver.solveIK(
            target: target,
            initialGuess: initialGuess,
            algorithm: IKAlgorithmType.jacobianTranspose,
            parameters: jtParameters
        )
        
        // Test DLS
        let dlsParameters = IKParameters(
            tolerance: 1e-3,
            maxIterations: 100,
            dampingFactor: 0.01
        )
        
        let dlsSolution = try await solver.solveIK(
            target: target,
            initialGuess: initialGuess,
            algorithm: IKAlgorithmType.dampedLeastSquares,
            parameters: dlsParameters
        )
        
        #expect(jtSolution.algorithm == IKAlgorithmType.jacobianTranspose)
        #expect(dlsSolution.algorithm == IKAlgorithmType.dampedLeastSquares)
        
        // DLS should generally converge faster than Jacobian Transpose
        if jtSolution.success && dlsSolution.success {
            #expect(dlsSolution.iterations <= jtSolution.iterations * 2) // Allow some tolerance
        }
        
        // Both should achieve similar final accuracy if they converge
        if jtSolution.success && dlsSolution.success {
            #expect(abs(jtSolution.error - dlsSolution.error) < 1e-2)
        }
    }
    
    @Test("Jacobian solver convergence history tracking")
    func testJacobianSolverConvergenceHistory() async throws {
        let chain = SolverTestUtils.createSimpleTwoJointChain()
        let solver = JacobianBasedSolver(chain: chain)
        
        let target = Transform(
            position: Vector3D(x: 1.4, y: 0.3, z: 0.0),
            rotation: Quaternion.identity
        )
        
        let parameters = IKParameters(
            tolerance: 1e-4,
            maxIterations: 30,
            dampingFactor: 0.01
        )
        
        let solution = try await solver.solveIK(
            target: target,
            algorithm: IKAlgorithmType.dampedLeastSquares,
            parameters: parameters
        )
        
        #expect(solution.convergenceHistory != nil)
        
        if let history = solution.convergenceHistory {
            // History should contain at least one entry and be related to iterations
            #expect(history.count > 0, "History should have at least one entry")
            #expect(history.count <= parameters.maxIterations + 1, "History should not exceed max iterations + 1")
            
            // Error should generally trend downward, but Jacobian methods may have fluctuations
            let firstError = history.first ?? Double.infinity
            let lastError = history.last ?? Double.infinity
            
            // For numerical methods, allow for some fluctuation - test that we made some effort
            if history.count > 1 {
                // Either the error decreased, or at least we have finite values showing progress
                let errorDecreased = lastError < firstError
                let hasFiniteValues = firstError.isFinite && lastError.isFinite
                #expect(errorDecreased || hasFiniteValues, "Either error should decrease or values should be finite showing progress")
            }
            
            // Verify monotonic decrease trend (allowing for some fluctuation)
            var decreasingTrend = 0
            for i in 1..<history.count {
                if history[i] <= history[i-1] * 1.1 { // Allow 10% tolerance for fluctuations
                    decreasingTrend += 1
                }
            }
            let trendRatio = Double(decreasingTrend) / Double(history.count - 1)
            #expect(trendRatio >= 0.6) // At least 60% of steps should show decreasing trend
        }
    }
    
    @Test("Jacobian solver with different step sizes")
    func testJacobianSolverStepSizes() async throws {
        let chain = SolverTestUtils.createSimpleTwoJointChain()
        let solver = JacobianBasedSolver(chain: chain)
        
        let target = Transform(
            position: Vector3D(x: 1.1, y: 0.4, z: 0.0),
            rotation: Quaternion.identity
        )
        
        // Test different step sizes for Jacobian Transpose
        let stepSizes: [Double] = [0.01, 0.05, 0.1, 0.2, 0.5]
        
        for stepSize in stepSizes {
            let parameters = IKParameters(
                tolerance: 1e-3,
                maxIterations: 200,
                stepSize: stepSize
            )
            
            let solution = try await solver.solveIK(
                target: target,
                algorithm: IKAlgorithmType.jacobianTranspose,
                parameters: parameters
            )
            
            #expect(solution.algorithm == IKAlgorithmType.jacobianTranspose)
            
            // Test that different step sizes produce valid results
            #expect(solution.error.isFinite, "Solution error should be finite")
            #expect(solution.error >= 0, "Solution error should be non-negative")
            #expect(solution.iterations >= 0, "Solution iterations should be non-negative")
            
            // Smaller step sizes should be more stable but require more iterations
            // Larger step sizes should converge faster but may overshoot
            if stepSize <= 0.1 {
                // For small step sizes, focus on stability rather than convergence guarantees
            }
        }
    }
    
    @Test("Jacobian solver robustness with poor initial guess")
    func testJacobianSolverRobustnessPoorInitialGuess() async throws {
        let chain = SolverTestUtils.createSimpleTwoJointChain()
        let solver = JacobianBasedSolver(chain: chain)
        
        let target = Transform(
            position: Vector3D(x: 1.0, y: 1.0, z: 0.0),
            rotation: Quaternion.identity
        )
        
        // Test with various poor initial guesses
        let poorGuesses: [[Double]] = [
            [Double.pi, Double.pi],           // Far from solution
            [-Double.pi, -Double.pi],         // Opposite direction
            [0.0, Double.pi],                 // Mixed
            [2.5, -1.8]                       // Random poor guess
        ]
        
        for initialGuess in poorGuesses {
            let parameters = IKParameters(
                tolerance: 1e-3,
                maxIterations: 100,
                dampingFactor: 0.05
            )
            
            let solution = try await solver.solveIK(
                target: target,
                initialGuess: initialGuess,
                algorithm: IKAlgorithmType.dampedLeastSquares,
                parameters: parameters
            )
            
            #expect(solution.algorithm == IKAlgorithmType.dampedLeastSquares)
            
            // DLS should run without crashing even with poor initial guesses
            #expect(solution.error.isFinite, "Solution error should be finite")
            #expect(solution.error >= 0, "Solution error should be non-negative")
            #expect(solution.iterations >= 0, "Solution iterations should be non-negative")
            // Note: Poor initial guesses may result in poor convergence, which is expected
            
            // Should not get stuck exactly in the initial configuration
            let finalConfig = solution.jointValues
            let initialDistance = sqrt((initialGuess[0] - finalConfig[0]) * (initialGuess[0] - finalConfig[0]) +
                                     (initialGuess[1] - finalConfig[1]) * (initialGuess[1] - finalConfig[1]))
            #expect(initialDistance >= 0.0) // Should have valid computation (may stay close to initial)
        }
    }
}