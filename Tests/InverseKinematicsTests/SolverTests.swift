import Testing
import Foundation
@testable import InverseKinematics

@Suite("IK Solver Tests")
struct SolverTests {
    
    @Suite("Analytical Solver Tests")
    struct AnalyticalSolverTests {
        
        @Test("2DOF planar solver - reachable target")
        func testTwoDOFPlanarSolverReachable() {
            let solver = TwoDOFPlanarSolver(link1Length: 1.0, link2Length: 1.0)
            let target = Vector3D(x: 1.5, y: 0.0, z: 0.0)
            
            let solution = solver.solve(target: target, elbowUp: true)
            #expect(solution != nil)
            
            if let (joint1, joint2) = solution {
                let forwardKin = calculateTwoLinkForwardKinematics(
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
        
        private func calculateTwoLinkForwardKinematics(
            link1: Double,
            link2: Double,
            joint1: Double,
            joint2: Double
        ) -> Vector3D {
            let x = link1 * cos(joint1) + link2 * cos(joint1 + joint2)
            let y = link1 * sin(joint1) + link2 * sin(joint1 + joint2)
            return Vector3D(x: x, y: y, z: 0.0)
        }
    }
    
    @Suite("Jacobian Solver Tests")
    struct JacobianSolverTests {
        
        @Test("Jacobian solver initialization")
        func testJacobianSolverInit() async {
            let chain = SolverTests.createSimpleTwoJointChain()
            let solver = JacobianBasedSolver(chain: chain)
            
            let supportedAlgorithms = solver.supportedAlgorithms
            #expect(supportedAlgorithms.contains(.jacobianTranspose))
            #expect(supportedAlgorithms.contains(.dampedLeastSquares))
            #expect(solver.jointCount == 2)
        }
        
        @Test("Jacobian transpose solver")
        func testJacobianTransposeSolver() async throws {
            let chain = SolverTests.createSimpleTwoJointChain()
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
                algorithm: .jacobianTranspose,
                parameters: parameters
            )
            
            #expect(solution.algorithm == .jacobianTranspose)
            #expect(solution.jointValues.count == 2)
            #expect(solution.iterations <= parameters.maxIterations)
        }
        
        @Test("Damped least squares solver")
        func testDampedLeastSquaresSolver() async throws {
            let chain = SolverTests.createSimpleTwoJointChain()
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
                algorithm: .dampedLeastSquares,
                parameters: parameters
            )
            
            #expect(solution.algorithm == .dampedLeastSquares)
            #expect(solution.convergenceHistory != nil)
        }
        
        @Test("Unsupported algorithm error")
        func testUnsupportedAlgorithmError() async {
            let chain = SolverTests.createSimpleTwoJointChain()
            let solver = JacobianBasedSolver(chain: chain)
            
            let target = Transform(position: Vector3D.zero, rotation: Quaternion.identity)
            
            await #expect(throws: IKError.unsupportedAlgorithm(.fabrik)) {
                _ = try await solver.solveIK(
                    target: target,
                    algorithm: .fabrik,
                    parameters: .default
                )
            }
        }
    }
    
    @Suite("CCD Solver Tests")
    struct CCDSolverTests {
        
        @Test("CCD solver initialization")
        func testCCDSolverInit() async {
            let chain = SolverTests.createSimpleTwoJointChain()
            let solver = CCDSolver(chain: chain)
            
            let supportedAlgorithms = solver.supportedAlgorithms
            #expect(supportedAlgorithms.contains(.cyclicCoordinateDescent))
            #expect(solver.jointCount == 2)
        }
        
        @Test("CCD solver execution")
        func testCCDSolverExecution() async throws {
            let chain = SolverTests.createSimpleTwoJointChain()
            let solver = CCDSolver(chain: chain)
            
            let target = Transform(
                position: Vector3D(x: 1.0, y: 1.0, z: 0.0),
                rotation: Quaternion.identity
            )
            
            let parameters = IKParameters(
                tolerance: 1e-2,
                maxIterations: 20,
                stepSize: 0.1
            )
            
            let solution = try await solver.solveIK(
                target: target,
                algorithm: .cyclicCoordinateDescent,
                parameters: parameters
            )
            
            #expect(solution.algorithm == .cyclicCoordinateDescent)
            #expect(solution.jointValues.count == 2)
        }
        
        @Test("CCD with orientation solver")
        func testCCDWithOrientationSolver() async throws {
            let chain = SolverTests.createThreeJointChain()
            let solver = CCDWithOrientationSolver(chain: chain, positionWeight: 1.0, orientationWeight: 0.5)
            
            let target = Transform(
                position: Vector3D(x: 2.0, y: 1.0, z: 0.0),
                rotation: Quaternion(axis: Vector3D.unitZ, angle: Double.pi / 4)
            )
            
            let parameters = IKParameters(tolerance: 1e-2, maxIterations: 30)
            
            let solution = try await solver.solveIK(
                target: target,
                algorithm: .cyclicCoordinateDescent,
                parameters: parameters
            )
            
            #expect(solution.jointValues.count == 3)
        }
    }
    
    @Suite("FABRIK Solver Tests")
    struct FABRIKSolverTests {
        
        @Test("FABRIK solver initialization")
        func testFABRIKSolverInit() async {
            let chain = SolverTests.createSimpleTwoJointChain()
            let solver = FABRIKSolver(chain: chain)
            
            let supportedAlgorithms = solver.supportedAlgorithms
            #expect(supportedAlgorithms.contains(.fabrik))
            #expect(solver.jointCount == 2)
        }
        
        @Test("FABRIK solver execution")
        func testFABRIKSolverExecution() async throws {
            let chain = SolverTests.createSimpleTwoJointChain()
            let solver = FABRIKSolver(chain: chain)
            
            let target = Transform(
                position: Vector3D(x: 1.5, y: 0.5, z: 0.0),
                rotation: Quaternion.identity
            )
            
            let parameters = IKParameters(
                tolerance: 1e-3,
                maxIterations: 20
            )
            
            let solution = try await solver.solveIK(
                target: target,
                algorithm: .fabrik,
                parameters: parameters
            )
            
            #expect(solution.algorithm == .fabrik)
            #expect(solution.jointValues.count == 2)
        }
        
        @Test("FABRIK solver unreachable target")
        func testFABRIKSolverUnreachableTarget() async throws {
            let chain = SolverTests.createSimpleTwoJointChain()
            let solver = FABRIKSolver(chain: chain)
            
            let target = Transform(
                position: Vector3D(x: 5.0, y: 0.0, z: 0.0),
                rotation: Quaternion.identity
            )
            
            let parameters = IKParameters(tolerance: 1e-3, maxIterations: 10)
            
            let solution = try await solver.solveIK(
                target: target,
                algorithm: .fabrik,
                parameters: parameters
            )
            
            #expect(solution.jointValues.count == 2)
        }
        
        @Test("Advanced FABRIK solver")
        func testAdvancedFABRIKSolver() async throws {
            let chain = SolverTests.createThreeJointChain()
            let solver = AdvancedFABRIKSolver(chain: chain, subBaseIndices: [1])
            
            let target = Transform(
                position: Vector3D(x: 2.5, y: 0.5, z: 0.0),
                rotation: Quaternion.identity
            )
            
            let parameters = IKParameters(tolerance: 1e-2, maxIterations: 25)
            
            let solution = try await solver.solveIK(
                target: target,
                algorithm: .fabrik,
                parameters: parameters
            )
            
            #expect(solution.jointValues.count == 3)
        }
    }
    
    @Suite("IK Parameters Tests")
    struct IKParametersTests {
        
        @Test("IK parameters initialization")
        func testIKParametersInit() {
            let params = IKParameters()
            #expect(params.tolerance == 1e-6)
            #expect(params.maxIterations == 100)
            #expect(params.dampingFactor == 0.01)
            #expect(params.stepSize == 0.1)
            #expect(params.positionWeight == 1.0)
            #expect(params.orientationWeight == 1.0)
        }
        
        @Test("IK parameters custom values")
        func testIKParametersCustom() {
            let params = IKParameters(
                tolerance: 1e-4,
                maxIterations: 50,
                dampingFactor: 0.1,
                stepSize: 0.2,
                positionWeight: 2.0,
                orientationWeight: 0.5
            )
            
            #expect(params.tolerance == 1e-4)
            #expect(params.maxIterations == 50)
            #expect(params.dampingFactor == 0.1)
            #expect(params.stepSize == 0.2)
            #expect(params.positionWeight == 2.0)
            #expect(params.orientationWeight == 0.5)
        }
        
        @Test("IK parameters validation")
        func testIKParametersValidation() {
            let params = IKParameters(
                tolerance: -1.0,
                maxIterations: 0,
                dampingFactor: -0.1,
                stepSize: -0.2,
                positionWeight: -1.0,
                orientationWeight: -0.5
            )
            
            #expect(params.tolerance == 0.0)
            #expect(params.maxIterations == 1)
            #expect(params.dampingFactor == 0.0)
            #expect(params.stepSize == 0.0)
            #expect(params.positionWeight == 0.0)
            #expect(params.orientationWeight == 0.0)
        }
    }
    
    @Suite("IK Solution Tests")
    struct IKSolutionTests {
        
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
        
        @Test("IK solution failed")
        func testIKSolutionFailed() {
            let failed = IKSolution.failed
            #expect(failed.success == false)
            #expect(failed.error == .infinity)
            #expect(failed.iterations == 0)
            #expect(failed.jointValues.isEmpty)
        }
    }
    
    private static func createSimpleTwoJointChain() -> KinematicChain {
        var chain = KinematicChain(id: "two_joint")
        
        let joint1 = Joint(
            id: "j1",
            type: .revolute,
            axis: Vector3D.unitZ,
            limits: JointLimits(min: -Double.pi, max: Double.pi)
        )
        
        let joint2 = Joint(
            id: "j2",
            type: .revolute,
            axis: Vector3D.unitZ,
            limits: JointLimits(min: -Double.pi, max: Double.pi),
            parentTransform: Transform(position: Vector3D(x: 1.0, y: 0.0, z: 0.0))
        )
        
        let link1 = Link(id: "l1", length: 1.0)
        let link2 = Link(id: "l2", length: 1.0)
        
        chain.addJoint(joint1)
        chain.addJoint(joint2)
        chain.addLink(link1)
        chain.addLink(link2)
        
        return chain
    }
    
    private static func createThreeJointChain() -> KinematicChain {
        var chain = KinematicChain(id: "three_joint")
        
        let joint1 = Joint(
            id: "j1",
            type: .revolute,
            axis: Vector3D.unitZ,
            limits: JointLimits(min: -Double.pi, max: Double.pi)
        )
        
        let joint2 = Joint(
            id: "j2",
            type: .revolute,
            axis: Vector3D.unitZ,
            limits: JointLimits(min: -Double.pi, max: Double.pi),
            parentTransform: Transform(position: Vector3D(x: 1.0, y: 0.0, z: 0.0))
        )
        
        let joint3 = Joint(
            id: "j3",
            type: .revolute,
            axis: Vector3D.unitZ,
            limits: JointLimits(min: -Double.pi, max: Double.pi),
            parentTransform: Transform(position: Vector3D(x: 1.0, y: 0.0, z: 0.0))
        )
        
        let link1 = Link(id: "l1", length: 1.0)
        let link2 = Link(id: "l2", length: 1.0)
        let link3 = Link(id: "l3", length: 0.5)
        
        chain.addJoint(joint1)
        chain.addJoint(joint2)
        chain.addJoint(joint3)
        chain.addLink(link1)
        chain.addLink(link2)
        chain.addLink(link3)
        
        return chain
    }
}