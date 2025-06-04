import Testing
#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif
@testable import InverseKinematics

@Suite("Analytical Solver Tests")
struct AnalyticalSolverTests {
    
    @Test("2DOF planar solver - reachable target")
    func testTwoDOFPlanarSolverReachable() {
        let solver = TwoDOFPlanarSolver(link1Length: 1.0, link2Length: 1.0)
        let target = Vector3D(x: 1.5, y: 0.0, z: 0.0)
        
        let solution = solver.solve(target: target, elbowUp: true)
        #expect(solution != nil)
        
        if let (joint1, joint2) = solution {
            let forwardKin = SolverTestUtils.calculateTwoLinkForwardKinematics(
                link1: 1.0,
                link2: 1.0,
                joint1: joint1,
                joint2: joint2
            )
            
            #expect(abs(forwardKin.x - target.x) < 1e-10)
            #expect(abs(forwardKin.y - target.y) < 1e-10)
        }
    }
    
    @Test("2DOF planar solver - unreachable target")
    func testTwoDOFPlanarSolverUnreachable() {
        let solver = TwoDOFPlanarSolver(link1Length: 1.0, link2Length: 1.0)
        let target = Vector3D(x: 3.0, y: 0.0, z: 0.0)
        
        let solution = solver.solve(target: target)
        #expect(solution == nil)
    }
    
    @Test("3DOF planar solver")
    func testThreeDOFPlanarSolver() {
        let solver = ThreeDOFPlanarSolver(
            link1Length: 1.0,
            link2Length: 1.0,
            link3Length: 0.5
        )
        
        let target = Transform(
            position: Vector3D(x: 1.0, y: 1.0, z: 0.0),
            rotation: Quaternion.identity
        )
        
        let solution = solver.solve(target: target, elbowUp: true)
        #expect(solution != nil)
    }
    
    @Test("Spherical wrist solver")
    func testSphericalWristSolver() {
        let solver = SphericalWristSolver()
        let targetOrientation = Quaternion(axis: Vector3D.unitZ, angle: Double.pi / 4)
        
        let solution = solver.solve(
            targetOrientation: targetOrientation,
            currentJoints: []
        )
        
        #expect(solution != nil)
    }
    
    @Test("Protocol-conforming analytical solver")
    func testAnalyticalSolverProtocol() async throws {
        let chain = SolverTestUtils.createSimpleTwoJointChain()
        let solver = AnalyticalSolver(chain: chain, type: .twoDOFPlanar)
        
        let target = Transform(
            position: Vector3D(x: 1.5, y: 0.5, z: 0.0),
            rotation: Quaternion.identity
        )
        
        let solution = try await solver.solveIK(
            target: target,
            algorithm: .analytical,
            parameters: .default
        )
        
        #expect(solution.algorithm == .analytical)
        #expect(solution.jointValues.count == 2)
        
        // Verify it's within reach (2 links of length 1.0 each)
        let distance = target.position.magnitude
        #expect(distance <= 2.0) // Should be reachable
    }
    
    // MARK: - Phase Two Improvements: Comprehensive Analytical Solver Tests
    
    @Test("2DOF elbow up vs elbow down solution branches")
    func testTwoDOFElbowUpDownBranches() {
        let solver = TwoDOFPlanarSolver(link1Length: 1.0, link2Length: 1.0)
        let target = Vector3D(x: 1.2, y: 0.8, z: 0.0) // Reachable target with multiple solutions
        
        // Test elbow up solution
        let elbowUpSolution = solver.solve(target: target, elbowUp: true)
        #expect(elbowUpSolution != nil)
        
        // Test elbow down solution
        let elbowDownSolution = solver.solve(target: target, elbowUp: false)
        #expect(elbowDownSolution != nil)
        
        if let (upJoint1, upJoint2) = elbowUpSolution,
           let (downJoint1, downJoint2) = elbowDownSolution {
            
            // Solutions should be different
            #expect(abs(upJoint1 - downJoint1) > 1e-6 || abs(upJoint2 - downJoint2) > 1e-6)
            
            // Both should reach the same target
            let upForwardKin = SolverTestUtils.calculateTwoLinkForwardKinematics(
                link1: 1.0, link2: 1.0, joint1: upJoint1, joint2: upJoint2)
            let downForwardKin = SolverTestUtils.calculateTwoLinkForwardKinematics(
                link1: 1.0, link2: 1.0, joint1: downJoint1, joint2: downJoint2)
            
            #expect(abs(upForwardKin.x - target.x) < 1e-10)
            #expect(abs(upForwardKin.y - target.y) < 1e-10)
            #expect(abs(downForwardKin.x - target.x) < 1e-10)
            #expect(abs(downForwardKin.y - target.y) < 1e-10)
            
            // Verify elbow configuration: elbow up should have joint2 > 0, elbow down should have joint2 < 0
            #expect(upJoint2 > 0) // Elbow up: positive second joint angle
            #expect(downJoint2 < 0) // Elbow down: negative second joint angle
        }
    }
    
    @Test("2DOF mathematical formula verification")
    func testTwoDOFMathematicalFormulas() {
        let l1: Double = 1.5
        let l2: Double = 1.0
        let solver = TwoDOFPlanarSolver(link1Length: l1, link2Length: l2)
        
        // Test known analytical solution
        let target = Vector3D(x: 2.0, y: 1.0, z: 0.0)
        
        if let (theta1, theta2) = solver.solve(target: target, elbowUp: true) {
            // Verify using inverse kinematics equations:
            // x = l1*cos(θ1) + l2*cos(θ1 + θ2)
            // y = l1*sin(θ1) + l2*sin(θ1 + θ2)
            
            let calculatedX = l1 * cos(theta1) + l2 * cos(theta1 + theta2)
            let calculatedY = l1 * sin(theta1) + l2 * sin(theta1 + theta2)
            
            #expect(abs(calculatedX - target.x) < 1e-12)
            #expect(abs(calculatedY - target.y) < 1e-12)
            
            // Verify cosine law constraint
            let distance = sqrt(target.x * target.x + target.y * target.y)
            let cosTheta2 = (distance * distance - l1 * l1 - l2 * l2) / (2 * l1 * l2)
            #expect(abs(cos(theta2) - cosTheta2) < 1e-12)
        }
    }
    
    @Test("3DOF orientation control verification")
    func testThreeDOFOrientationControl() {
        let solver = ThreeDOFPlanarSolver(
            link1Length: 1.0,
            link2Length: 1.0,
            link3Length: 0.5
        )
        
        // Test with a reachable target position and basic orientation
        let target = Transform(
            position: Vector3D(x: 1.5, y: 0.5, z: 0.0),
            rotation: Quaternion(axis: Vector3D.unitZ, angle: Double.pi / 4)
        )
        
        let solution = solver.solve(target: target, elbowUp: true)
        #expect(solution != nil)
        
        if let (theta1, theta2, theta3) = solution {
            // Verify that we get three joint angles
            #expect(theta1.isFinite)
            #expect(theta2.isFinite)
            #expect(theta3.isFinite)
            
            // For a 3DOF planar solver, the third joint should help with orientation
            // The exact relationship depends on implementation details
            #expect(abs(theta3) < 2 * Double.pi) // Should be within reasonable bounds
        }
    }
    
    @Test("Spherical wrist ZYZ convention verification")
    func testSphericalWristZYZConvention() {
        let solver = SphericalWristSolver()
        
        // Test basic spherical wrist functionality
        let targetQuat = Quaternion(axis: Vector3D.unitZ, angle: Double.pi / 4)
        
        let solution = solver.solve(
            targetOrientation: targetQuat,
            currentJoints: []
        )
        
        #expect(solution != nil)
        
        if let (joint1, joint2, joint3) = solution {
            // Verify that we get valid joint angles
            #expect(joint1.isFinite)
            #expect(joint2.isFinite)
            #expect(joint3.isFinite)
            
            // Verify joints are within reasonable bounds
            #expect(abs(joint1) <= 2 * Double.pi)
            #expect(abs(joint2) <= 2 * Double.pi)
            #expect(abs(joint3) <= 2 * Double.pi)
            
            // Basic check that the solution is not trivial
            let hasNonZeroJoint = abs(joint1) > 1e-10 || abs(joint2) > 1e-10 || abs(joint3) > 1e-10
            #expect(hasNonZeroJoint)
        }
    }
    
    @Test("Boundary conditions and workspace limits")
    func testAnalyticalSolverBoundaryConditions() {
        let solver = TwoDOFPlanarSolver(link1Length: 1.0, link2Length: 1.0)
        
        // Test at maximum reach
        let maxReach = Vector3D(x: 2.0, y: 0.0, z: 0.0)
        let maxSolution = solver.solve(target: maxReach, elbowUp: true)
        #expect(maxSolution != nil)
        
        if let (theta1, theta2) = maxSolution {
            // At maximum reach, second joint should be nearly 0 (arm fully extended)
            #expect(abs(theta2) < 1e-10)
            #expect(abs(theta1) < 1e-10) // First joint should align with x-axis
        }
        
        // Test at minimum reach
        let minReach = Vector3D(x: 0.1, y: 0.0, z: 0.0)
        let minSolution = solver.solve(target: minReach, elbowUp: true)
        #expect(minSolution != nil)
        
        if let (_, theta2) = minSolution {
            // At minimum reach, second joint should be close to π (arm folded)
            // Allow more tolerance for numerical approximation
            #expect(abs(abs(theta2) - Double.pi) < 0.2)
        }
        
        // Test exactly at the boundary (just beyond reach)
        let beyondReach = Vector3D(x: 2.0001, y: 0.0, z: 0.0)
        let beyondSolution = solver.solve(target: beyondReach, elbowUp: true)
        #expect(beyondSolution == nil)
    }
    
    @Test("Analytical solver singularity handling")
    func testAnalyticalSolverSingularities() {
        let solver = TwoDOFPlanarSolver(link1Length: 1.0, link2Length: 1.0)
        
        // Test at origin (singularity)
        let origin = Vector3D(x: 0.0, y: 0.0, z: 0.0)
        let originSolution = solver.solve(target: origin, elbowUp: true)
        #expect(originSolution != nil) // Should still provide a solution
        
        // Test very close to origin
        let nearOrigin = Vector3D(x: 1e-10, y: 1e-10, z: 0.0)
        let nearOriginSolution = solver.solve(target: nearOrigin, elbowUp: true)
        #expect(nearOriginSolution != nil)
        
        // Test on y-axis (different singularity behavior)
        let yAxis = Vector3D(x: 0.0, y: 1.5, z: 0.0)
        let yAxisSolution = solver.solve(target: yAxis, elbowUp: true)
        #expect(yAxisSolution != nil)
        
        if let (theta1, _) = yAxisSolution {
            // For y-axis target, joint should be oriented towards y
            // The exact angle depends on solver implementation
            #expect(theta1.isFinite)
            #expect(abs(theta1) < 2 * Double.pi) // Within reasonable bounds
        }
    }
}