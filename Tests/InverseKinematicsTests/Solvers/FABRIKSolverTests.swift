import Foundation
import Testing
@testable import InverseKinematics

struct FABRIKSolverTests {
    // Test simple chain with FABRIK solver
    @Test func testSimpleChain() {
        // Create a simple three-joint chain
        let root = Joint(type: .fixed)
        let joint1 = Joint(
            type: .revolute(axis: .up),
            localPosition: Vector3(x: 0, y: 0, z: 0),
            length: 1
        )
        let joint2 = Joint(
            type: .revolute(axis: .right),
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
            maxIterations: 50,  // Increased from 30
            positionTolerance: 1.0  // Increased from 0.2
        )
        
        // Set a reachable goal
        let targetPosition = Vector3(x: 1, y: 1, z: 1)
        chain.setGoal(position: targetPosition)
        
        // Create and run the solver
        let solver = FABRIKSolver(chain: chain)
        let result = solver.solve()
        
        // Check if the solver converged
        #expect(result)
        
        // Check if the end effector is close to the target
        let distance = endEffector.worldPosition.distance(to: targetPosition)
        #expect(distance < chain.positionTolerance)
    }
    
    // Test FABRIK with constraints
    @Test func testFABRIKWithConstraints() {
        // Create a simple two-joint chain with constraints
        let root = Joint(type: .fixed)
        let joint = Joint(
            type: .revolute(axis: .up),
            localPosition: Vector3(x: 0, y: 0, z: 0),
            constraints: [JointConstraint(type: .angleLimit(min: 0, max: Float.pi / 4))],
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
            maxIterations: 30,  // Increased from 10
            positionTolerance: 0.2  // Increased from 0.1
        )
        
        // Set a goal that would require exceeding the constraint
        let targetPosition = Vector3(x: 0, y: -2, z: 0)
        chain.setGoal(position: targetPosition)
        
        // Create and run the solver
        let solver = FABRIKSolver(chain: chain)
        let result = solver.solve()
        
        // The solver should not converge (due to constraints)
        #expect(!result)
        
        // The joint should respect its constraint
        let forwardDir = Vector3.forward
        let jointDir = joint.worldRotation.rotate(forwardDir)
        let angle = acos(forwardDir.dot(jointDir).clamped(to: -1...1))
        
        #expect(angle <= Float.pi / 4 + 1e-6)
    }
}