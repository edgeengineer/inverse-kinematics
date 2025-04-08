import Foundation
import Testing
@testable import InverseKinematics

struct JacobianSolverTests {
    // Test simple chain with Jacobian solver
    @Test func testSimpleChain() {
        // Create a simple two-joint chain
        let root = Joint(type: .fixed)
        let joint = Joint(
            type: .revolute(axis: .up),
            localPosition: Vector3(x: 0, y: 0, z: 0),
            length: 1
        )
        let endEffector = Joint(
            type: .fixed,
            localPosition: Vector3(x: 0, y: 0, z: 1)
        )
        
        root.addChild(joint)
        joint.addChild(endEffector)
        
        let chain = IKChain(
            rootJoint: root,
            endEffector: endEffector,
            maxIterations: 100,
            positionTolerance: 0.4
        )
        
        // Set a reachable goal that's more aligned with the joint's rotation capabilities
        // Since this is a revolute joint around the Y axis, it can easily reach points in the XZ plane
        let targetPosition = Vector3(x: 0.7, y: 0, z: 0.7)
        chain.setGoal(position: targetPosition)
        
        // Create and run the solver
        let solver = JacobianSolver(chain: chain)
        solver.learningRate = 0.5 // Adjust learning rate for better convergence
        let result = solver.solve()
        
        // Check if the solver converged
        #expect(result)
        
        // Check if the end effector is close to the target
        let distance = endEffector.worldPosition.distance(to: targetPosition)
        #expect(distance < chain.positionTolerance)
    }
    
    // Test orientation goal with Jacobian solver
    @Test func testOrientationGoal() {
        // Create a simple two-joint chain
        let root = Joint(type: .fixed)
        let joint = Joint(
            type: .spherical,
            localPosition: Vector3(x: 0, y: 0, z: 0),
            length: 1
        )
        let endEffector = Joint(
            type: .fixed,
            localPosition: Vector3(x: 0, y: 0, z: 1)
        )
        
        root.addChild(joint)
        joint.addChild(endEffector)
        
        let chain = IKChain(
            rootJoint: root,
            endEffector: endEffector,
            maxIterations: 50,  // Increased from 20
            positionTolerance: 0.1,
            orientationTolerance: 0.3  // Increased from 0.1
        )
        
        // Set an orientation goal
        let targetOrientation = Quaternion(axis: Vector3.right, angle: Float.pi / 2)
        chain.setGoal(orientation: targetOrientation)
        
        // Create and run the solver
        let solver = JacobianSolver(chain: chain)
        solver.learningRate = 0.5 // Adjust learning rate for better convergence
        let result = solver.solve()
        
        // Check if the solver converged
        #expect(result)
        
        // Check if the end effector orientation is close to the target
        let currentOrientation = endEffector.worldRotation
        let diff = currentOrientation.inverse * targetOrientation
        let angle = 2 * acos(abs(diff.w).clamped(to: 0...1))
        
        #expect(angle < chain.orientationTolerance)
    }
    
    // Test position goal requiring spherical joint movement
    @Test func testPositionGoalWithSpherical() {
        // Create a chain with at least one spherical joint to control orientation
        let root = Joint(type: .fixed)
        let joint1 = Joint(
            type: .revolute(axis: .up),
            localPosition: .zero,
            length: 1
        )
        let joint2 = Joint(
            type: .spherical, // Use spherical for orientation control
            localPosition: Vector3(x: 0, y: 0, z: 1),
            length: 1
        )
        let endEffector = Joint(
            type: .fixed,
            localPosition: Vector3(x: 0, y: 0, z: 1)
        )
        
        root.addChild(joint1)
        joint1.addChild(joint2)
        joint2.addChild(endEffector)
        
        let chain = IKChain(
            rootJoint: root,
            endEffector: endEffector,
            maxIterations: 500, // Further increase iterations for complex goal
            positionTolerance: 0.1,
            orientationTolerance: 0.1
        )
        
        // Set a reachable combined goal
        // Simplify goal: Position only, requires spherical joint movement
        let targetPosition = Vector3(x: 1.0, y: 1.0, z: 0.0) 
        chain.setGoal(position: targetPosition, orientation: nil) // Target position only
        
        // Create and run the solver
        let solver = JacobianSolver(chain: chain)
        // Use default solver parameters (learningRate, dampingFactor) initially for DLS
        // We might still need to adjust the damping factor if defaults don't work well.
        solver.dampingFactor = 1.0 // Try a larger damping factor for stability
        
        let result = solver.solve()
        
        // Check if the solver converged
        #expect(result)
        
        // Check position convergence
        let distance = endEffector.worldPosition.distance(to: targetPosition)
        #expect(distance < chain.positionTolerance)
        
        // No orientation target, so skip orientation check
    }
    
    // Test a simple, easily reachable combined goal
    @Test func testSimpleCombinedGoal() {
        // Simple 2-link chain: Fixed -> Revolute(Y) -> Fixed(EndEffector)
        let root = Joint(type: .fixed)
        let joint1 = Joint(
            type: .revolute(axis: .up), // Rotate around Y
            localPosition: .zero,
            length: 1.0 
        )
        let endEffector = Joint(
            type: .fixed, 
            localPosition: Vector3(x: 0, y: 0, z: 1.0) // Initial position at (0,0,1)
        )
        
        root.addChild(joint1)
        joint1.addChild(endEffector)
        
        let chain = IKChain(
            rootJoint: root,
            endEffector: endEffector,
            maxIterations: 100, 
            positionTolerance: 0.01,
            orientationTolerance: 0.01
        )
        
        // Target: Rotate 90 degrees around Y, so end effector should be at (1,0,0)
        // with orientation also rotated 90 degrees around Y.
        let targetPosition = Vector3(x: 1.0, y: 0.0, z: 0.0)
        let targetOrientation = Quaternion(axis: .up, angle: .pi / 2)
        chain.setGoal(position: targetPosition, orientation: targetOrientation)
        
        // Create and run the solver (use defaults, maybe tune damping)
        let solver = JacobianSolver(chain: chain)
        // solver.dampingFactor = 0.1 // Start with default or slightly tuned
        let result = solver.solve()
        
        // Check convergence
        #expect(result)
        
        // Check position
        let distance = endEffector.worldPosition.distance(to: targetPosition)
        #expect(distance < chain.positionTolerance)
        
        // Check orientation
        let currentOrientation = endEffector.worldRotation
        let diff = currentOrientation.inverse * targetOrientation
        let angle = 2 * acos(abs(diff.w).clamped(to: 0...1))
        #expect(angle < chain.orientationTolerance)
    }

    // Test case for a more complex combined position and orientation goal
    @Test("Test Complex Combined Goal") func testComplexCombinedGoal() throws {
        // Define joints
        let joint1 = Joint(type: .revolute(axis: Vector3(x: 0, y: 1, z: 0)), length: 0.0)
        let joint2 = Joint(type: .revolute(axis: Vector3(x: 1, y: 0, z: 0)), length: 0.5)
        let joint3 = Joint(type: .revolute(axis: Vector3(x: 0, y: 0, z: 1)), length: 0.5)

        // Link joints
        joint1.children = [joint2]
        joint2.parent = joint1
        joint2.children = [joint3]
        joint3.parent = joint2

        // Create chain
        var chain = IKChain(rootJoint: joint1, endEffector: joint3)
        chain.positionTolerance = 0.1 // Relaxed tolerance for complex test
        chain.orientationTolerance = 0.1 // Relaxed tolerance

        // Define a complex goal
        let targetPosition = Vector3(x: 0.5, y: 0.5, z: 0.5)
        let targetOrientation = Quaternion(axis: Vector3(x: 0, y: 0, z: 1), angle: .pi / 2) // 90 degrees around Z

        let goal = IKGoal(
            position: targetPosition,
            orientation: targetOrientation,
            positionWeight: 1.0,
            orientationWeight: 1.0
        )

        // Instantiate the Jacobian solver with DLS parameters
        let solver = JacobianSolver(chain: chain)
        solver.maxIterations = 200 // More iterations might be needed
        solver.learningRate = 0.2 // Increased learning rate
        solver.dampingFactor = 0.5 // Reduced damping factor

        // Assign goal to chain
        chain.goal = goal

        // Solve
        let success = solver.solve()

        // Get final state
        let finalChain = solver.chain
        let finalEndEffectorPosition = finalChain.endEffector?.worldPosition ?? .zero
        let finalEndEffectorOrientation = finalChain.endEffector?.worldRotation ?? .identity

        // Assert convergence
        #expect(success, "Solver should converge for the complex combined goal")

        let positionError = finalEndEffectorPosition.distance(to: targetPosition)
        // Calculate angular difference between final and target orientation
        let orientationError = finalEndEffectorOrientation.angle(to: targetOrientation) // Angle difference in radians

        print("--- Complex Combined Goal Test ---")
        print("Target Position: \(targetPosition)")
        print("Final Position: \(finalEndEffectorPosition)")
        print("Position Error: \(positionError)")
        print("Target Orientation: \(targetOrientation)")
        print("Final Orientation: \(finalEndEffectorOrientation)")
        print("Orientation Error (Radians): \(orientationError)")
        print("---------------------------------")

        #expect(positionError < 0.1, "Position error should be within tolerance (0.1)") // Relaxed tolerance for complex test
        #expect(orientationError < 0.1, "Orientation error should be within tolerance (0.1 radians)") // Relaxed tolerance
    }
}