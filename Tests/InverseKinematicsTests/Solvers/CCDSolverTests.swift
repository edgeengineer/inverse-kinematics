import Testing
import Foundation
@testable import InverseKinematics

struct CCDSolverTests {
    // Test simple two-joint chain with CCD solver
    @Test func testTwoJointChain() {
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
            maxIterations: 10,
            positionTolerance: 0.1
        )
        
        // Set a reachable goal
        let targetPosition = Vector3(x: 0, y: 1, z: 0)
        chain.setGoal(position: targetPosition)
        
        // Debug prints
        print("===== TEST: Two-joint chain =====")
        print("Initial joint position: \(joint.worldPosition)")
        print("Initial end effector position: \(endEffector.worldPosition)")
        print("Target position: \(targetPosition)")
        
        // Test direct rotation - this is what we want the solver to do
        print("\nTesting direct rotation:")
        joint.localRotation = Quaternion(axis: Vector3.up, angle: Float.pi / 2)
        print("End effector position after direct rotation: \(endEffector.worldPosition)")
        print("Distance to target after direct rotation: \(endEffector.worldPosition.distance(to: targetPosition))")
        
        // Reset rotation for the solver test
        joint.localRotation = Quaternion.identity
        
        // Create and run the solver
        let solver = CCDSolver(chain: chain)
        let result = solver.solve()
        
        // More debug prints
        print("\nAfter solve:")
        print("Joint position: \(joint.worldPosition)")
        print("End effector position: \(endEffector.worldPosition)")
        print("Joint local rotation: \(joint.localRotation)")
        print("Distance to target: \(endEffector.worldPosition.distance(to: targetPosition))")
        print("Position tolerance: \(chain.positionTolerance)")
        print("Solver result: \(result)")
        
        // Check if the solver converged
        #expect(result)
        
        // Check if the end effector is close to the target
        let distance = endEffector.worldPosition.distance(to: targetPosition)
        #expect(distance < chain.positionTolerance)
    }
    
    // Test unreachable target
    @Test func testUnreachableTarget() {
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
            maxIterations: 10,
            positionTolerance: 0.1
        )
        
        // Set an unreachable goal (too far away)
        let targetPosition = Vector3(x: 0, y: 0, z: 10)
        chain.setGoal(position: targetPosition)
        
        // Create and run the solver
        let solver = CCDSolver(chain: chain)
        let result = solver.solve()
        
        // The solver should not converge
        #expect(!result)
        
        // The end effector should be pointing towards the target
        let direction = (targetPosition - root.worldPosition).normalized
        let currentDirection = (endEffector.worldPosition - root.worldPosition).normalized
        let dot = direction.dot(currentDirection)
        
        // Vectors should be nearly parallel
        #expect(dot > 0.9)
    }
}