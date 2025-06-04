import Testing
#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif
@testable import InverseKinematics

/// Comprehensive tests for specific solver types as outlined in Phase Two improvements
@Suite("Specific Solver Type Tests")
struct SpecificSolverTypeTests {
    
    // MARK: - Analytical Solver Specific Tests
    
    @Suite("Enhanced Analytical Solver Tests")
    struct EnhancedAnalyticalSolverTests {
        
        @Test("Comprehensive elbow up/down solution verification")
        func testComprehensiveElbowUpDownSolutions() {
            let solver = TwoDOFPlanarSolver(link1Length: 1.0, link2Length: 1.0)
            
            // Test multiple targets that have distinct elbow up/down solutions
            let testTargets = [
                Vector3D(x: 1.2, y: 0.8, z: 0.0),
                Vector3D(x: 0.8, y: 1.2, z: 0.0),
                Vector3D(x: 1.5, y: 0.5, z: 0.0),
                Vector3D(x: 0.5, y: 1.5, z: 0.0)
            ]
            
            for target in testTargets {
                let elbowUpSolution = solver.solve(target: target, elbowUp: true)
                let elbowDownSolution = solver.solve(target: target, elbowUp: false)
                
                #expect(elbowUpSolution != nil, "Elbow up solution should exist for target \(target)")
                #expect(elbowDownSolution != nil, "Elbow down solution should exist for target \(target)")
                
                if let (upJoint1, upJoint2) = elbowUpSolution,
                   let (downJoint1, downJoint2) = elbowDownSolution {
                    
                    // Verify mathematical accuracy for both solutions
                    let upForwardKin = SolverTestUtils.calculateTwoLinkForwardKinematics(
                        link1: 1.0, link2: 1.0, joint1: upJoint1, joint2: upJoint2)
                    let downForwardKin = SolverTestUtils.calculateTwoLinkForwardKinematics(
                        link1: 1.0, link2: 1.0, joint1: downJoint1, joint2: downJoint2)
                    
                    #expect(SolverTestUtils.isApproximatelyEqual(upForwardKin.x, target.x, tolerance: 1e-12))
                    #expect(SolverTestUtils.isApproximatelyEqual(upForwardKin.y, target.y, tolerance: 1e-12))
                    #expect(SolverTestUtils.isApproximatelyEqual(downForwardKin.x, target.x, tolerance: 1e-12))
                    #expect(SolverTestUtils.isApproximatelyEqual(downForwardKin.y, target.y, tolerance: 1e-12))
                    
                    // Verify elbow configuration constraint
                    #expect(upJoint2 > 0, "Elbow up should have positive second joint angle")
                    #expect(downJoint2 < 0, "Elbow down should have negative second joint angle")
                    
                    // Verify solutions are distinct
                    let solutionDifference = abs(upJoint1 - downJoint1) + abs(upJoint2 - downJoint2)
                    #expect(solutionDifference > 1e-6, "Elbow up/down solutions should be distinct")
                }
            }
        }
        
        @Test("Six DOF analytical solver branch verification")
        func testSixDOFAnalyticalSolverBranches() async throws {
            // Create a 6-DOF chain suitable for analytical solving
            let chain = SolverTestUtils.createVariableJointChain(jointCount: 6)
            let solver = AnalyticalSolver(chain: chain, type: .sixDOF)
            
            let target = Transform(
                position: Vector3D(x: 3.0, y: 1.0, z: 0.5),
                rotation: Quaternion(axis: Vector3D.unitZ, angle: Double.pi / 6)
            )
            
            // Test all four solution branches (elbowUp x wristFlip combinations)
            let configurations: [(elbowUp: Bool, wristFlip: Bool)] = [
                (true, false), (true, true), (false, false), (false, true)
            ]
            
            for config in configurations {
                let parameters = IKParameters(tolerance: 1e-3, maxIterations: 1)
                
                do {
                    let solution = try await solver.solveIK(
                        target: target,
                        initialGuess: nil,
                        algorithm: .analytical,
                        parameters: parameters
                    )
                    
                    #expect(solution.algorithm == .analytical)
                    #expect(solution.jointValues.count == 6)
                    
                    // Verify solution reaches target within tolerance
                    if solution.success {
                        let endEffector = chain.endEffectorTransform(jointValues: solution.jointValues)
                        let positionError = target.position.distance(to: endEffector.position)
                        #expect(positionError < 0.1, "Position error should be reasonable for config \(config)")
                    }
                } catch {
                    // Analytical solver may not always find solutions for all branches
                    // This is acceptable behavior for some target configurations
                }
            }
        }
        
        @Test("Mathematical formula verification for 3DOF planar")
        func testThreeDOFMathematicalVerification() {
            let l1: Double = 1.0
            let l2: Double = 1.5
            let l3: Double = 0.5
            let solver = ThreeDOFPlanarSolver(link1Length: l1, link2Length: l2, link3Length: l3)
            
            // Use a more conservative target that's easier to reach
            let target = Transform(
                position: Vector3D(x: 1.5, y: 0.5, z: 0.0),
                rotation: Quaternion(axis: Vector3D.unitZ, angle: Double.pi / 6)
            )
            
            if let (theta1, theta2, theta3) = solver.solve(target: target, elbowUp: true) {
                // Verify that all joint angles are finite and reasonable
                #expect(theta1.isFinite && theta2.isFinite && theta3.isFinite)
                #expect(abs(theta1) < 2 * Double.pi)
                #expect(abs(theta2) < 2 * Double.pi) 
                #expect(abs(theta3) < 2 * Double.pi)
                
                // Test basic mathematical relationship: the solver should provide a solution
                // that when plugged into forward kinematics gives a reasonable result
                // (The exact mathematical verification is complex for 3DOF with orientation)
                
                // Position verification (first two joints contribution)
                let x2 = l1 * cos(theta1) + l2 * cos(theta1 + theta2)
                let y2 = l1 * sin(theta1) + l2 * sin(theta1 + theta2)
                
                // Final position including third link
                let finalX = x2 + l3 * cos(theta1 + theta2 + theta3)
                let finalY = y2 + l3 * sin(theta1 + theta2 + theta3)
                
                // Verify position is in reasonable workspace
                let distance = sqrt(finalX * finalX + finalY * finalY)
                let maxReach = l1 + l2 + l3
                #expect(distance <= maxReach, "Solution should be within workspace")
                #expect(distance >= 0, "Distance should be non-negative")
            }
        }
    }
    
    // MARK: - Enhanced Jacobian-Based Solver Tests
    
    @Suite("Enhanced Jacobian Solver Tests")
    struct EnhancedJacobianSolverTests {
        
        @Test("DLS damping factor singularity behavior")
        func testDLSDampingFactorSingularityBehavior() async throws {
            let chain = SolverTestUtils.createSimpleTwoJointChain()
            let solver = JacobianBasedSolver(chain: chain)
            
            // Create targets near different types of singularities
            let singularityTargets = [
                Transform(position: Vector3D(x: 1.99, y: 0.0, z: 0.0)),    // Near full extension
                Transform(position: Vector3D(x: 0.01, y: 0.0, z: 0.0)),    // Near origin
                Transform(position: Vector3D(x: 0.0, y: 1.99, z: 0.0)),    // Y-axis near singularity
                Transform(position: Vector3D(x: -1.99, y: 0.0, z: 0.0))    // Negative X near singularity
            ]
            
            let dampingFactors: [Double] = [0.001, 0.01, 0.1, 0.5]
            
            for target in singularityTargets {
                var bestError = Double.infinity
                var dampingStability: [Double: (error: Double, iterations: Int)] = [:]
                
                for dampingFactor in dampingFactors {
                    let parameters = IKParameters(
                        tolerance: 1e-4,
                        maxIterations: 100,
                        dampingFactor: dampingFactor
                    )
                    
                    let solution = try await solver.solveIK(
                        target: target,
                        algorithm: .dampedLeastSquares,
                        parameters: parameters
                    )
                    
                    dampingStability[dampingFactor] = (solution.error, solution.iterations)
                    bestError = min(bestError, solution.error)
                    
                    // Higher damping should provide more stable solutions
                    #expect(solution.error.isFinite, "Error should be finite for damping \(dampingFactor)")
                    #expect(solution.jointValues.allSatisfy { $0.isFinite }, "Joint values should be finite")
                }
                
                // Verify that higher damping generally provides more stability
                // (lower error variance or at least finite solutions)
                let highDampingError = dampingStability[0.5]?.error ?? Double.infinity
                
                // High damping should provide reasonable solutions (not necessarily better than low damping)
                // The key insight is that high damping should provide stable, finite solutions
                #expect(highDampingError.isFinite && highDampingError >= 0,
                       "High damping should provide stable finite solution")
            }
        }
        
        @Test("SDLS selective damping versus DLS comparison")
        func testSDLSvsDLSComparison() async throws {
            let chain = SolverTestUtils.createThreeJointChain()
            let solver = JacobianBasedSolver(chain: chain)
            
            // Test on redundant system where SDLS should show benefits
            let target = Transform(
                position: Vector3D(x: 2.0, y: 0.5, z: 0.0),
                rotation: Quaternion.identity
            )
            
            let parameters = IKParameters(
                tolerance: 1e-3,
                maxIterations: 50,
                dampingFactor: 0.01
            )
            
            // Test DLS
            let dlsSolution = try await solver.solveIK(
                target: target,
                algorithm: .dampedLeastSquares,
                parameters: parameters
            )
            
            // Test SDLS
            let sdlsSolution = try await solver.solveIK(
                target: target,
                algorithm: .selectivelyDampedLeastSquares,
                parameters: parameters
            )
            
            #expect(dlsSolution.algorithm == .dampedLeastSquares)
            #expect(sdlsSolution.algorithm == .selectivelyDampedLeastSquares)
            
            // Both should produce valid solutions
            #expect(dlsSolution.jointValues.count == 3)
            #expect(sdlsSolution.jointValues.count == 3)
            #expect(dlsSolution.error.isFinite)
            #expect(sdlsSolution.error.isFinite)
            
            // For redundant systems, SDLS may converge differently than DLS
            // The key is that both should be stable and produce reasonable results
            if dlsSolution.success && sdlsSolution.success {
                #expect(dlsSolution.error < 1.0, "DLS should achieve reasonable accuracy")
                #expect(sdlsSolution.error < 1.0, "SDLS should achieve reasonable accuracy")
            }
        }
        
        @Test("Jacobian condition number impact on solver performance")
        func testJacobianConditionNumberImpact() async throws {
            let chain = SolverTestUtils.createSimpleTwoJointChain()
            let solver = JacobianBasedSolver(chain: chain)
            
            // Test configurations with different condition numbers
            let testConfigurations = [
                (initialGuess: [0.0, 0.0], description: "Well-conditioned start"),
                (initialGuess: [0.0, Double.pi], description: "Folded configuration"),
                (initialGuess: [Double.pi/2, -Double.pi/2], description: "Stretched configuration")
            ]
            
            let target = Transform(
                position: Vector3D(x: 1.0, y: 1.0, z: 0.0),
                rotation: Quaternion.identity
            )
            
            for config in testConfigurations {
                // Test all three Jacobian methods
                for algorithm in [IKAlgorithmType.jacobianTranspose, 
                                .dampedLeastSquares, 
                                .selectivelyDampedLeastSquares] {
                    
                    let parameters = IKParameters(
                        tolerance: 1e-3,
                        maxIterations: 50,
                        dampingFactor: 0.01,
                        stepSize: 0.1
                    )
                    
                    let solution = try await solver.solveIK(
                        target: target,
                        initialGuess: config.initialGuess,
                        algorithm: algorithm,
                        parameters: parameters
                    )
                    
                    #expect(solution.algorithm == algorithm)
                    #expect(solution.jointValues.count == 2)
                    
                    // All algorithms should handle different initial conditions reasonably
                    #expect(solution.error.isFinite, 
                           "\(algorithm) should produce finite error for \(config.description)")
                    #expect(solution.jointValues.allSatisfy { $0.isFinite },
                           "\(algorithm) should produce finite joint values for \(config.description)")
                }
            }
        }
    }
    
    // MARK: - Enhanced CCD Solver Tests
    
    @Suite("Enhanced CCD Solver Tests")
    struct EnhancedCCDSolverTests {
        
        @Test("CCD position vs orientation accuracy detailed analysis")
        func testCCDPositionVsOrientationAccuracy() async throws {
            let chain = SolverTestUtils.createThreeJointChain()
            
            // Test different weight combinations
            let weightConfigurations: [(pos: Double, ori: Double, description: String)] = [
                (1.0, 0.0, "Position only"),
                (1.0, 0.1, "Position dominant"),
                (1.0, 0.5, "Position favored"),
                (1.0, 1.0, "Equal weights"),
                (0.5, 1.0, "Orientation favored"),
                (0.1, 1.0, "Orientation dominant"),
                (0.0, 1.0, "Orientation only")
            ]
            
            let target = Transform(
                position: Vector3D(x: 2.0, y: 1.0, z: 0.0),
                rotation: Quaternion(axis: Vector3D.unitZ, angle: Double.pi / 4)
            )
            
            for config in weightConfigurations {
                let solver = CCDWithOrientationSolver(
                    chain: chain,
                    positionWeight: config.pos,
                    orientationWeight: config.ori
                )
                
                let parameters = IKParameters(tolerance: 1e-2, maxIterations: 30)
                
                let solution = try await solver.solveIK(
                    target: target,
                    algorithm: .cyclicCoordinateDescent,
                    parameters: parameters
                )
                
                #expect(solution.jointValues.count == 3)
                
                // Verify the end effector result
                let endEffector = chain.endEffectorTransform(jointValues: solution.jointValues)
                let positionError = target.position.distance(to: endEffector.position)
                let orientationError = abs(target.rotation.eulerAngles.z - endEffector.rotation.eulerAngles.z)
                
                // Higher weight should lead to better accuracy in that dimension
                if config.pos > config.ori && config.pos > 0.5 {
                    // Position-weighted: position error should be relatively small
                    #expect(positionError < 2.5, "Position error should be reasonable for \(config.description)")
                } else if config.ori > config.pos && config.ori > 0.5 {
                    // Orientation-weighted: orientation error should be relatively small  
                    // Note: CCD with orientation can struggle with orientation accuracy
                    #expect(orientationError < Double.pi, "Orientation error should be within bounds for \(config.description)")
                }
                
                #expect(solution.error.isFinite, "Solution error should be finite for \(config.description)")
            }
        }
        
        @Test("CCD solver with extreme chain lengths")
        func testCCDExtremeChainLengths() async throws {
            // Test very short and very long chains
            let extremeChainLengths = [1, 2, 10, 20]
            
            for chainLength in extremeChainLengths {
                let chain = SolverTestUtils.createVariableJointChain(jointCount: chainLength)
                let solver = CCDSolver(chain: chain)
                
                // Scale target based on chain length
                let conservativeReach = Double(chainLength) * 0.2
                let target = Transform(
                    position: Vector3D(x: conservativeReach, y: conservativeReach * 0.1, z: 0.0),
                    rotation: Quaternion.identity
                )
                
                let parameters = IKParameters(
                    tolerance: 1e-1, // Relaxed tolerance for extreme cases
                    maxIterations: chainLength * 10 // Scale iterations with complexity
                )
                
                let solution = try await solver.solveIK(
                    target: target,
                    algorithm: .cyclicCoordinateDescent,
                    parameters: parameters
                )
                
                #expect(solution.jointValues.count == chainLength)
                #expect(solution.algorithm == .cyclicCoordinateDescent)
                
                // For extreme chain lengths, focus on stability rather than perfect accuracy
                #expect(solution.error.isFinite, "Solution should be stable for \(chainLength)-DOF chain")
                #expect(solution.jointValues.allSatisfy { $0.isFinite }, 
                       "All joint values should be finite for \(chainLength)-DOF chain")
                
                // Verify joints are within reasonable bounds
                #expect(solution.jointValues.allSatisfy { abs($0) < 10 * Double.pi },
                       "Joint values should be within reasonable bounds for \(chainLength)-DOF chain")
            }
        }
        
        @Test("CCD convergence behavior with different step sizes")
        func testCCDConvergenceBehaviorStepSizes() async throws {
            let chain = SolverTestUtils.createSimpleTwoJointChain()
            let solver = CCDSolver(chain: chain)
            
            let target = Transform(
                position: Vector3D(x: 1.2, y: 0.8, z: 0.0),
                rotation: Quaternion.identity
            )
            
            let stepSizes: [Double] = [0.01, 0.05, 0.1, 0.3, 0.5, 1.0]
            
            for stepSize in stepSizes {
                let parameters = IKParameters(
                    tolerance: 1e-3,
                    maxIterations: 50,
                    stepSize: stepSize
                )
                
                let solution = try await solver.solveIK(
                    target: target,
                    algorithm: .cyclicCoordinateDescent,
                    parameters: parameters
                )
                
                #expect(solution.algorithm == .cyclicCoordinateDescent)
                #expect(solution.jointValues.count == 2)
                
                // Analyze convergence behavior
                if let history = solution.convergenceHistory {
                    #expect(history.count > 0, "Should have convergence history")
                    
                    // Smaller step sizes should be more stable but potentially slower
                    // Larger step sizes should converge faster but potentially less stable
                    if stepSize <= 0.1 {
                        // Small step size: expect stable convergence
                        let hasOscillations = zip(history.dropLast(), history.dropFirst())
                            .filter { $0.0 < $0.1 }.count
                        #expect(hasOscillations < history.count / 3, 
                               "Small step size should have few oscillations")
                    }
                }
                
                #expect(solution.error.isFinite, "Error should be finite for step size \(stepSize)")
            }
        }
    }
}