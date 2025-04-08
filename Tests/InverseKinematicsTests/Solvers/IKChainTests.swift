//
//  IKTests.swift
//  InverseKinematicsTests
//
//  Tests for the Inverse Kinematics system
//

import Testing
import Foundation
@testable import InverseKinematics

struct IKChainTests {
    // Test chain creation and joint path
    @Test func testChainJointPath() {
        let root = Joint(type: .fixed)
        let joint1 = Joint(type: .revolute(axis: .up))
        let joint2 = Joint(type: .revolute(axis: .right))
        let endEffector = Joint(type: .fixed)
        
        root.addChild(joint1)
        joint1.addChild(joint2)
        joint2.addChild(endEffector)
        
        let chain = IKChain(rootJoint: root, endEffector: endEffector)
        let path = chain.getJointPath()
        
        #expect(path.count == 4)
        #expect(path[0] === root)
        #expect(path[1] === joint1)
        #expect(path[2] === joint2)
        #expect(path[3] === endEffector)
    }
    
    // Test error calculation
    @Test func testErrorCalculation() {
        let root = Joint(type: .fixed)
        let joint = Joint(type: .revolute(axis: .up), localPosition: Vector3(x: 0, y: 0, z: 1))
        let endEffector = Joint(type: .fixed, localPosition: Vector3(x: 0, y: 0, z: 1))
        
        root.addChild(joint)
        joint.addChild(endEffector)
        
        let chain = IKChain(rootJoint: root, endEffector: endEffector)
        
        // Set a goal position
        let goal = IKGoal(position: Vector3(x: 1, y: 0, z: 2))
        chain.goal = goal
        
        // Calculate error
        let (positionError, _) = chain.calculateError()
        
        // End effector is at (0, 0, 2), goal is at (1, 0, 2)
        // Distance should be 1
        #expect(abs(positionError - 1.0) < 1e-6)
    }
    
    // Test convergence check
    @Test func testConvergence() {
        let root = Joint(type: .fixed)
        let joint = Joint(type: .revolute(axis: .up), localPosition: Vector3(x: 0, y: 0, z: 1))
        let endEffector = Joint(type: .fixed, localPosition: Vector3(x: 0, y: 0, z: 1))
        
        root.addChild(joint)
        joint.addChild(endEffector)
        
        let chain = IKChain(
            rootJoint: root,
            endEffector: endEffector,
            positionTolerance: 0.01  // Small tolerance to start
        )
        
        // Set a goal position that is outside the tolerance
        // End effector is at (0, 0, 2), goal is at (0, 0, 2.05)
        // Distance is 0.05 which is > 0.01 tolerance
        chain.goal = IKGoal(position: Vector3(x: 0, y: 0, z: 2.05))
        
        // Should not have converged yet (error > tolerance)
        #expect(!chain.hasConverged())
        
        // Now increase the tolerance to be larger than the error
        chain.positionTolerance = 0.1  // Now 0.05 < 0.1
        
        // Should have converged (error < tolerance)
        #expect(chain.hasConverged())
    }
}


// Helper extension
extension Float {
    func clamped(to range: ClosedRange<Float>) -> Float {
        return max(range.lowerBound, min(range.upperBound, self))
    }
}
