#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif

public actor CCDSolver: InverseKinematicsSolvable {
    private let chain: KinematicChain
    
    public init(chain: KinematicChain) {
        self.chain = chain
    }
    
    public nonisolated var supportedAlgorithms: [IKAlgorithmType] {
        [.cyclicCoordinateDescent]
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
        guard algorithm == .cyclicCoordinateDescent else {
            throw IKError.unsupportedAlgorithm(algorithm)
        }
        
        let startValues = initialGuess ?? generateInitialGuess()
        guard startValues.count == jointCount else {
            throw IKError.invalidJointCount(expected: jointCount, actual: startValues.count)
        }
        
        return try await solveCCD(target: target, initialGuess: startValues, parameters: parameters)
    }
    
    private func solveCCD(
        target: Transform,
        initialGuess: [Double],
        parameters: IKParameters
    ) async throws -> IKSolution {
        var currentJoints = clampJointValues(initialGuess)
        var convergenceHistory: [Double] = []
        
        for iteration in 0..<parameters.maxIterations {
            let currentTransform = calculateEndEffector(jointValues: currentJoints)
            let error = calculateError(target: target, current: currentTransform, parameters: parameters)
            convergenceHistory.append(error)
            
            if error < parameters.tolerance {
                return IKSolution(
                    jointValues: currentJoints,
                    success: true,
                    error: error,
                    iterations: iteration,
                    algorithm: .cyclicCoordinateDescent,
                    convergenceHistory: convergenceHistory
                )
            }
            
            let jointTransforms = calculateJointTransforms(jointValues: currentJoints)
            let endEffectorPos = currentTransform.position
            
            for jointIndex in (0..<jointCount).reversed() {
                let jointPos = jointTransforms[jointIndex].position
                let jointAxis = chain.joints[jointIndex].axis
                
                let toEndEffector = endEffectorPos - jointPos
                let toTarget = target.position - jointPos
                
                if toEndEffector.magnitude < 1e-6 || toTarget.magnitude < 1e-6 {
                    continue
                }
                
                let rotationAxis = toEndEffector.cross(toTarget).normalized
                let dotProduct = toEndEffector.normalized.dot(toTarget.normalized)
                let clampedDot = max(-1.0, min(1.0, dotProduct))
                let rotationAngle = acos(clampedDot)
                
                if rotationAngle < 1e-6 {
                    continue
                }
                
                let projectedAxis = projectAxisToJoint(rotationAxis, jointAxis: jointAxis)
                let signedAngle = rotationAngle * (projectedAxis > 0 ? 1 : -1)
                
                let newJointValue = currentJoints[jointIndex] + signedAngle * parameters.stepSize
                currentJoints[jointIndex] = jointLimits[jointIndex].clamp(newJointValue)
                
                let newTransform = calculateEndEffector(jointValues: currentJoints)
                let newError = calculateError(target: target, current: newTransform, parameters: parameters)
                
                if newError >= error {
                    currentJoints[jointIndex] = jointLimits[jointIndex].clamp(currentJoints[jointIndex] - signedAngle * parameters.stepSize)
                }
            }
        }
        
        return IKSolution(
            jointValues: currentJoints,
            success: false,
            error: convergenceHistory.last ?? .infinity,
            iterations: parameters.maxIterations,
            algorithm: .cyclicCoordinateDescent,
            convergenceHistory: convergenceHistory
        )
    }
    
    private func projectAxisToJoint(_ axis: Vector3D, jointAxis: Vector3D) -> Double {
        return axis.dot(jointAxis)
    }
}

public actor CCDWithOrientationSolver: InverseKinematicsSolvable {
    private let chain: KinematicChain
    private let positionWeight: Double
    private let orientationWeight: Double
    
    public init(chain: KinematicChain, positionWeight: Double = 1.0, orientationWeight: Double = 0.1) {
        self.chain = chain
        self.positionWeight = positionWeight
        self.orientationWeight = orientationWeight
    }
    
    public nonisolated var supportedAlgorithms: [IKAlgorithmType] {
        [.cyclicCoordinateDescent]
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
        guard algorithm == .cyclicCoordinateDescent else {
            throw IKError.unsupportedAlgorithm(algorithm)
        }
        
        let startValues = initialGuess ?? generateInitialGuess()
        guard startValues.count == jointCount else {
            throw IKError.invalidJointCount(expected: jointCount, actual: startValues.count)
        }
        
        return try await solveCCDWithOrientation(target: target, initialGuess: startValues, parameters: parameters)
    }
    
    private func solveCCDWithOrientation(
        target: Transform,
        initialGuess: [Double],
        parameters: IKParameters
    ) async throws -> IKSolution {
        var currentJoints = clampJointValues(initialGuess)
        var convergenceHistory: [Double] = []
        
        for iteration in 0..<parameters.maxIterations {
            let currentTransform = calculateEndEffector(jointValues: currentJoints)
            
            let positionError = target.position.distance(to: currentTransform.position) * positionWeight
            let orientationError = calculateOrientationError(target: target.rotation, current: currentTransform.rotation) * orientationWeight
            let totalError = positionError + orientationError
            
            convergenceHistory.append(totalError)
            
            if totalError < parameters.tolerance {
                return IKSolution(
                    jointValues: currentJoints,
                    success: true,
                    error: totalError,
                    iterations: iteration,
                    algorithm: .cyclicCoordinateDescent,
                    convergenceHistory: convergenceHistory
                )
            }
            
            let _ = calculateJointTransforms(jointValues: currentJoints) // Note: Joint transforms calculated but not used
            
            for jointIndex in (0..<jointCount).reversed() {
                var bestAngle = 0.0
                var bestError = totalError
                
                let angleStep = parameters.stepSize
                let searchRange = [-angleStep * 2, -angleStep, 0, angleStep, angleStep * 2]
                
                for deltaAngle in searchRange {
                    let testValue = currentJoints[jointIndex] + deltaAngle
                    let clampedValue = jointLimits[jointIndex].clamp(testValue)
                    
                    var testJoints = currentJoints
                    testJoints[jointIndex] = clampedValue
                    
                    let testTransform = calculateEndEffector(jointValues: testJoints)
                    let testPosError = target.position.distance(to: testTransform.position) * positionWeight
                    let testOrientError = calculateOrientationError(target: target.rotation, current: testTransform.rotation) * orientationWeight
                    let testTotalError = testPosError + testOrientError
                    
                    if testTotalError < bestError {
                        bestError = testTotalError
                        bestAngle = deltaAngle
                    }
                }
                
                if bestAngle != 0 {
                    let newValue = currentJoints[jointIndex] + bestAngle
                    currentJoints[jointIndex] = jointLimits[jointIndex].clamp(newValue)
                }
            }
        }
        
        return IKSolution(
            jointValues: currentJoints,
            success: false,
            error: convergenceHistory.last ?? .infinity,
            iterations: parameters.maxIterations,
            algorithm: .cyclicCoordinateDescent,
            convergenceHistory: convergenceHistory
        )
    }
    
    private func calculateOrientationError(target: Quaternion, current: Quaternion) -> Double {
        let dot = abs(target.dot(current))
        let clampedDot = max(0.0, min(1.0, dot))
        return acos(clampedDot)
    }
}