//
//  InverseKinematics.swift
//  InverseKinematics
//
//  Main entry point for the Inverse Kinematics library
//

#if canImport(FoundationEssentials)
import FoundationEssentials
#elseif canImport(Foundation)
import Foundation
#endif

/// Main class for the Inverse Kinematics library
public struct InverseKinematics {
    /// The version of the library
    public static let version = "1.0.0"
    
    /// Creates a new IK chain
    /// - Parameters:
    ///   - rootJoint: The root joint of the chain
    ///   - endEffector: The end effector joint of the chain
    ///   - maxIterations: Maximum number of iterations for solving
    ///   - positionTolerance: Tolerance for position convergence
    ///   - orientationTolerance: Tolerance for orientation convergence
    /// - Returns: A new IK chain
    public static func createChain(
        rootJoint: Joint? = nil,
        endEffector: Joint? = nil,
        maxIterations: Int = 20,
        positionTolerance: Float = 0.01,
        orientationTolerance: Float = 0.01
    ) -> IKChain {
        return IKChain(
            rootJoint: rootJoint,
            endEffector: endEffector,
            maxIterations: maxIterations,
            positionTolerance: positionTolerance,
            orientationTolerance: orientationTolerance
        )
    }
    
    /// Creates a new IK solver of the specified type
    /// - Parameters:
    ///   - type: The type of solver to create
    ///   - chain: The IK chain to solve
    /// - Returns: An IK solver instance
    public static func createSolver(type: IKSolverType, chain: IKChain) -> IKSolver {
        return IKSolverFactory.createSolver(type: type, chain: chain)
    }
    
    /// Creates a revolute joint
    /// - Parameters:
    ///   - axis: The axis of rotation
    ///   - position: The local position of the joint
    ///   - rotation: The local rotation of the joint
    ///   - minAngle: The minimum angle limit (in radians)
    ///   - maxAngle: The maximum angle limit (in radians)
    ///   - length: The length of the joint (distance to the next joint)
    /// - Returns: A new revolute joint
    public static func createRevoluteJoint(
        axis: Vector3,
        position: Vector3 = .zero,
        rotation: Quaternion = .identity,
        minAngle: Float? = nil,
        maxAngle: Float? = nil,
        length: Float = 0
    ) -> Joint {
        var constraints: [JointConstraint] = []
        
        if let min = minAngle, let max = maxAngle {
            constraints.append(JointConstraint(type: .angleLimit(min: min, max: max)))
        }
        
        return Joint(
            type: .revolute(axis: axis.normalized),
            localPosition: position,
            localRotation: rotation,
            constraints: constraints,
            length: length
        )
    }
    
    /// Creates a prismatic joint
    /// - Parameters:
    ///   - axis: The axis of translation
    ///   - position: The local position of the joint
    ///   - rotation: The local rotation of the joint
    ///   - minDistance: The minimum distance limit
    ///   - maxDistance: The maximum distance limit
    ///   - length: The length of the joint (distance to the next joint)
    /// - Returns: A new prismatic joint
    public static func createPrismaticJoint(
        axis: Vector3,
        position: Vector3 = .zero,
        rotation: Quaternion = .identity,
        minDistance: Float? = nil,
        maxDistance: Float? = nil,
        length: Float = 0
    ) -> Joint {
        var constraints: [JointConstraint] = []
        
        if let min = minDistance, let max = maxDistance {
            constraints.append(JointConstraint(type: .distanceLimit(min: min, max: max)))
        }
        
        return Joint(
            type: .prismatic(axis: axis.normalized),
            localPosition: position,
            localRotation: rotation,
            constraints: constraints,
            length: length
        )
    }
    
    /// Creates a spherical joint
    /// - Parameters:
    ///   - position: The local position of the joint
    ///   - rotation: The local rotation of the joint
    ///   - coneAxis: The axis of the cone constraint
    ///   - coneAngle: The angle of the cone constraint (in radians)
    ///   - length: The length of the joint (distance to the next joint)
    /// - Returns: A new spherical joint
    public static func createSphericalJoint(
        position: Vector3 = .zero,
        rotation: Quaternion = .identity,
        coneAxis: Vector3? = nil,
        coneAngle: Float? = nil,
        length: Float = 0
    ) -> Joint {
        var constraints: [JointConstraint] = []
        
        if let axis = coneAxis, let angle = coneAngle {
            constraints.append(JointConstraint(type: .coneLimit(axis: axis.normalized, angle: angle)))
        }
        
        return Joint(
            type: .spherical,
            localPosition: position,
            localRotation: rotation,
            constraints: constraints,
            length: length
        )
    }
    
    /// Creates a fixed joint
    /// - Parameters:
    ///   - position: The local position of the joint
    ///   - rotation: The local rotation of the joint
    ///   - length: The length of the joint (distance to the next joint)
    /// - Returns: A new fixed joint
    public static func createFixedJoint(
        position: Vector3 = .zero,
        rotation: Quaternion = .identity,
        length: Float = 0
    ) -> Joint {
        return Joint(
            type: .fixed,
            localPosition: position,
            localRotation: rotation,
            constraints: [],
            length: length
        )
    }
    
    /// Creates a simple arm chain with the specified number of joints
    /// - Parameters:
    ///   - numJoints: The number of joints in the chain
    ///   - jointLength: The length of each joint
    ///   - solverType: The type of solver to use
    /// - Returns: A tuple containing the chain and solver
    public static func createSimpleArm(
        numJoints: Int,
        jointLength: Float = 1.0,
        solverType: IKSolverType = .ccd
    ) -> (chain: IKChain, solver: IKSolver) {
        // Create the root joint
        let root = createFixedJoint(position: .zero, rotation: .identity, length: jointLength)
        
        // Create the chain
        let chain = createChain(rootJoint: root)
        
        // Create the joints
        var lastJoint = root
        
        for i in 1..<numJoints {
            let isEndEffector = i == numJoints - 1
            
            let joint: Joint
            if i % 3 == 1 {
                // X-axis revolute joint
                joint = createRevoluteJoint(
                    axis: .right,
                    position: Vector3(x: 0, y: 0, z: jointLength),
                    minAngle: -.pi / 2,
                    maxAngle: .pi / 2,
                    length: isEndEffector ? 0 : jointLength
                )
            } else if i % 3 == 2 {
                // Y-axis revolute joint
                joint = createRevoluteJoint(
                    axis: .up,
                    position: Vector3(x: 0, y: 0, z: jointLength),
                    minAngle: -.pi / 2,
                    maxAngle: .pi / 2,
                    length: isEndEffector ? 0 : jointLength
                )
            } else {
                // Z-axis revolute joint
                joint = createRevoluteJoint(
                    axis: .forward,
                    position: Vector3(x: 0, y: 0, z: jointLength),
                    minAngle: -.pi / 2,
                    maxAngle: .pi / 2,
                    length: isEndEffector ? 0 : jointLength
                )
            }
            
            lastJoint.addChild(joint)
            lastJoint = joint
            
            if isEndEffector {
                chain.setEndEffector(joint)
            }
        }
        
        // Create the solver
        let solver = createSolver(type: solverType, chain: chain)
        
        return (chain, solver)
    }
    
    /// Creates a robotic arm with specific joint types
    /// - Parameters:
    ///   - solverType: The type of solver to use
    /// - Returns: A tuple containing the chain and solver
    public static func createRoboticArm(
        solverType: IKSolverType = .jacobian
    ) -> (chain: IKChain, solver: IKSolver) {
        // Create the base (fixed)
        let base = createFixedJoint(position: .zero, rotation: .identity, length: 0.5)
        
        // Create the chain
        let chain = createChain(rootJoint: base)
        
        // Create a shoulder joint (revolute around Y)
        let shoulder = createRevoluteJoint(
            axis: .up,
            position: Vector3(x: 0, y: 0.5, z: 0),
            minAngle: -.pi,
            maxAngle: .pi,
            length: 1.0
        )
        base.addChild(shoulder)
        
        // Create an elbow joint (revolute around X)
        let elbow = createRevoluteJoint(
            axis: .right,
            position: Vector3(x: 0, y: 0, z: 1.0),
            minAngle: 0,
            maxAngle: .pi * 0.75,
            length: 1.0
        )
        shoulder.addChild(elbow)
        
        // Create a wrist joint (spherical with cone constraint)
        let wrist = createSphericalJoint(
            position: Vector3(x: 0, y: 0, z: 1.0),
            coneAxis: .forward,
            coneAngle: .pi / 2,
            length: 0.5
        )
        elbow.addChild(wrist)
        
        // Create an end effector (fixed)
        let endEffector = createFixedJoint(
            position: Vector3(x: 0, y: 0, z: 0.5)
        )
        wrist.addChild(endEffector)
        
        // Set the end effector
        chain.setEndEffector(endEffector)
        
        // Create the solver
        let solver = createSolver(type: solverType, chain: chain)
        
        return (chain, solver)
    }
}
