#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif

public struct TwoDOFPlanarSolver: Sendable {
    public let link1Length: Double
    public let link2Length: Double
    
    public init(link1Length: Double, link2Length: Double) {
        precondition(link1Length > 0, "Link 1 length must be positive")
        precondition(link2Length > 0, "Link 2 length must be positive")
        self.link1Length = link1Length
        self.link2Length = link2Length
    }
    
    public func solve(target: Vector3D, elbowUp: Bool = true) -> (joint1: Double, joint2: Double)? {
        let x = target.x
        let y = target.y
        let distance = sqrt(x * x + y * y)
        
        let maxReach = link1Length + link2Length
        let minReach = abs(link1Length - link2Length)
        
        guard distance >= minReach && distance <= maxReach else {
            return nil
        }
        
        let cosTheta2 = (distance * distance - link1Length * link1Length - link2Length * link2Length) / (2 * link1Length * link2Length)
        
        guard abs(cosTheta2) <= 1 else {
            return nil
        }
        
        let theta2 = elbowUp ? acos(cosTheta2) : -acos(cosTheta2)
        
        let k1 = link1Length + link2Length * cos(theta2)
        let k2 = link2Length * sin(theta2)
        
        let theta1 = atan2(y, x) - atan2(k2, k1)
        
        return (joint1: theta1, joint2: theta2)
    }
}

public struct ThreeDOFPlanarSolver: Sendable {
    public let link1Length: Double
    public let link2Length: Double
    public let link3Length: Double
    
    public init(link1Length: Double, link2Length: Double, link3Length: Double) {
        precondition(link1Length > 0, "Link 1 length must be positive")
        precondition(link2Length > 0, "Link 2 length must be positive")
        precondition(link3Length > 0, "Link 3 length must be positive")
        self.link1Length = link1Length
        self.link2Length = link2Length
        self.link3Length = link3Length
    }
    
    public func solve(target: Transform, elbowUp: Bool = true) -> (joint1: Double, joint2: Double, joint3: Double)? {
        let wrist = target.position - target.rotation.rotate(Vector3D(x: 0, y: 0, z: link3Length))
        
        guard let twoDOFSolution = TwoDOFPlanarSolver(link1Length: link1Length, link2Length: link2Length)
            .solve(target: wrist, elbowUp: elbowUp) else {
            return nil
        }
        
        let forwardKin = calculateForwardKinematics(
            joint1: twoDOFSolution.joint1,
            joint2: twoDOFSolution.joint2,
            joint3: 0
        )
        
        let wristOrientation = forwardKin.rotation.inverse * target.rotation
        let joint3 = wristOrientation.eulerAngles.z
        
        return (joint1: twoDOFSolution.joint1, joint2: twoDOFSolution.joint2, joint3: joint3)
    }
    
    private func calculateForwardKinematics(joint1: Double, joint2: Double, joint3: Double) -> Transform {
        let t1 = Transform(position: Vector3D(x: 0, y: 0, z: 0), rotation: Quaternion(axis: .unitZ, angle: joint1))
        let t2 = Transform(position: Vector3D(x: link1Length, y: 0, z: 0), rotation: Quaternion(axis: .unitY, angle: joint2))
        let t3 = Transform(position: Vector3D(x: link2Length, y: 0, z: 0), rotation: Quaternion(axis: .unitZ, angle: joint3))
        let endEffector = Transform(position: Vector3D(x: link3Length, y: 0, z: 0), rotation: .identity)
        
        return t1 * t2 * t3 * endEffector
    }
}

public struct SphericalWristSolver: Sendable {
    public init() {}
    
    public func solve(targetOrientation: Quaternion, currentJoints: [Double]) -> (joint4: Double, joint5: Double, joint6: Double)? {
        let eulerAngles = targetOrientation.eulerAngles
        
        let joint4 = eulerAngles.z
        let joint5 = eulerAngles.y
        let joint6 = eulerAngles.x
        
        return (joint4: joint4, joint5: joint5, joint6: joint6)
    }
    
    public func solveZYZ(targetOrientation: Quaternion) -> (alpha: Double, beta: Double, gamma: Double)? {
        let matrix = targetOrientation
        let eulerAngles = matrix.eulerAngles
        
        return (alpha: eulerAngles.z, beta: eulerAngles.y, gamma: eulerAngles.x)
    }
    
    public func solveZYX(targetOrientation: Quaternion) -> (roll: Double, pitch: Double, yaw: Double)? {
        let eulerAngles = targetOrientation.eulerAngles
        
        return (roll: eulerAngles.x, pitch: eulerAngles.y, yaw: eulerAngles.z)
    }
}

public struct SixDOFAnalyticalSolver: Sendable {
    public let link1Length: Double
    public let link2Length: Double
    public let link3Length: Double
    public let wristOffset: Double
    
    public init(link1Length: Double, link2Length: Double, link3Length: Double, wristOffset: Double = 0) {
        self.link1Length = link1Length
        self.link2Length = link2Length
        self.link3Length = link3Length
        self.wristOffset = wristOffset
    }
    
    public func solve(target: Transform, elbowUp: Bool = true, wristFlip: Bool = false) -> [Double]? {
        let wristCenter = target.position - target.rotation.rotate(Vector3D(x: 0, y: 0, z: wristOffset))
        
        guard let armSolution = ThreeDOFPlanarSolver(
            link1Length: link1Length,
            link2Length: link2Length,
            link3Length: link3Length
        ).solve(target: Transform(position: wristCenter, rotation: target.rotation), elbowUp: elbowUp) else {
            return nil
        }
        
        let armForwardKin = calculateArmForwardKinematics(
            joint1: armSolution.joint1,
            joint2: armSolution.joint2,
            joint3: armSolution.joint3
        )
        
        let relativeOrientation = armForwardKin.rotation.inverse * target.rotation
        
        guard let wristSolution = SphericalWristSolver().solve(
            targetOrientation: relativeOrientation,
            currentJoints: []
        ) else {
            return nil
        }
        
        return [
            armSolution.joint1,
            armSolution.joint2,
            armSolution.joint3,
            wristSolution.joint4,
            wristSolution.joint5,
            wristSolution.joint6
        ]
    }
    
    private func calculateArmForwardKinematics(joint1: Double, joint2: Double, joint3: Double) -> Transform {
        let t1 = Transform(position: Vector3D(x: 0, y: 0, z: 0), rotation: Quaternion(axis: .unitZ, angle: joint1))
        let t2 = Transform(position: Vector3D(x: link1Length, y: 0, z: 0), rotation: Quaternion(axis: .unitY, angle: joint2))
        let t3 = Transform(position: Vector3D(x: link2Length, y: 0, z: 0), rotation: Quaternion(axis: .unitY, angle: joint3))
        
        return t1 * t2 * t3
    }
}

// MARK: - Protocol-Conforming Analytical Solver

public actor AnalyticalSolver: InverseKinematicsSolvable {
    private let chain: KinematicChain
    private let solverType: AnalyticalSolverType
    
    public enum AnalyticalSolverType: Sendable {
        case twoDOFPlanar
        case threeDOFPlanar
        case sixDOF
        case detectFromChain
    }
    
    public init(chain: KinematicChain, type: AnalyticalSolverType = .detectFromChain) {
        self.chain = chain
        self.solverType = type == .detectFromChain ? Self.detectSolverType(from: chain) : type
    }
    
    public nonisolated var supportedAlgorithms: [IKAlgorithmType] {
        [.analytical]
    }
    
    public nonisolated var jointCount: Int {
        chain.jointCount
    }
    
    public nonisolated var jointLimits: [JointLimits] {
        chain.joints.map { $0.limits }
    }
    
    public nonisolated func calculateEndEffector(jointValues: [Double]) -> Transform {
        chain.endEffectorTransform(jointValues: jointValues)
    }
    
    public nonisolated func calculateJointTransforms(jointValues: [Double]) -> [Transform] {
        chain.forwardKinematics(jointValues: jointValues)
    }
    
    public nonisolated func calculateJacobian(jointValues: [Double], epsilon: Double = 1e-6) -> [[Double]] {
        chain.jacobian(jointValues: jointValues, epsilon: epsilon)
    }
    
    public func solveIK(
        target: Transform,
        initialGuess: [Double]?,
        algorithm: IKAlgorithmType,
        parameters: IKParameters
    ) async throws -> IKSolution {
        guard algorithm == .analytical else {
            throw IKError.unsupportedAlgorithm(algorithm)
        }
        
        let solution: [Double]?
        
        switch solverType {
        case .twoDOFPlanar, .detectFromChain:
            guard jointCount == 2 else {
                throw IKError.invalidJointCount(expected: 2, actual: jointCount)
            }
            let linkLengths = chain.links.map { $0.length }
            guard linkLengths.count >= 2 else {
                throw IKError.invalidParameters("Insufficient links for 2DOF solver")
            }
            
            let solver = TwoDOFPlanarSolver(link1Length: linkLengths[0], link2Length: linkLengths[1])
            if let result = solver.solve(target: target.position) {
                solution = [result.joint1, result.joint2]
            } else {
                solution = nil
            }
            
        case .threeDOFPlanar:
            guard jointCount == 3 else {
                throw IKError.invalidJointCount(expected: 3, actual: jointCount)
            }
            let linkLengths = chain.links.map { $0.length }
            guard linkLengths.count >= 3 else {
                throw IKError.invalidParameters("Insufficient links for 3DOF solver")
            }
            
            let solver = ThreeDOFPlanarSolver(
                link1Length: linkLengths[0],
                link2Length: linkLengths[1],
                link3Length: linkLengths[2]
            )
            if let result = solver.solve(target: target) {
                solution = [result.joint1, result.joint2, result.joint3]
            } else {
                solution = nil
            }
            
        case .sixDOF:
            guard jointCount == 6 else {
                throw IKError.invalidJointCount(expected: 6, actual: jointCount)
            }
            let linkLengths = chain.links.map { $0.length }
            guard linkLengths.count >= 3 else {
                throw IKError.invalidParameters("Insufficient links for 6DOF solver")
            }
            
            let solver = SixDOFAnalyticalSolver(
                link1Length: linkLengths[0],
                link2Length: linkLengths[1],
                link3Length: linkLengths[2]
            )
            solution = solver.solve(target: target)
        }
        
        guard let jointValues = solution else {
            return IKSolution(
                jointValues: initialGuess ?? Array(repeating: 0.0, count: jointCount),
                success: false,
                error: .infinity,
                iterations: 1,
                algorithm: .analytical
            )
        }
        
        let clampedValues = clampJointValues(jointValues)
        let currentTransform = calculateEndEffector(jointValues: clampedValues)
        let error = calculateError(target: target, current: currentTransform, parameters: parameters)
        
        return IKSolution(
            jointValues: clampedValues,
            success: true,
            error: error,
            iterations: 1,
            algorithm: .analytical
        )
    }
    
    private static func detectSolverType(from chain: KinematicChain) -> AnalyticalSolverType {
        switch chain.jointCount {
        case 2:
            return .twoDOFPlanar
        case 3:
            return .threeDOFPlanar
        case 6:
            return .sixDOF
        default:
            return .twoDOFPlanar // Default fallback
        }
    }
}

