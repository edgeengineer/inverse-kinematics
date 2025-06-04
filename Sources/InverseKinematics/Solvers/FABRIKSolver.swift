#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif

public actor FABRIKSolver: InverseKinematicsSolvable {
    private let chain: KinematicChain
    private let linkLengths: [Double]
    private let totalLength: Double
    
    public init(chain: KinematicChain) {
        self.chain = chain
        self.linkLengths = chain.links.map { $0.length }
        self.totalLength = linkLengths.reduce(0, +)
    }
    
    public nonisolated var supportedAlgorithms: [IKAlgorithmType] {
        [.fabrik]
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
        guard algorithm == .fabrik else {
            throw IKError.unsupportedAlgorithm(algorithm)
        }
        
        let startValues = initialGuess ?? generateInitialGuess()
        guard startValues.count == jointCount else {
            throw IKError.invalidJointCount(expected: jointCount, actual: startValues.count)
        }
        
        return try await solveFABRIK(target: target, initialGuess: startValues, parameters: parameters)
    }
    
    private func solveFABRIK(
        target: Transform,
        initialGuess: [Double],
        parameters: IKParameters
    ) async throws -> IKSolution {
        let basePosition = chain.baseTransform.position
        let targetPosition = target.position
        let distanceToTarget = basePosition.distance(to: targetPosition)
        
        if distanceToTarget > totalLength {
            let direction = (targetPosition - basePosition).normalized
            let reachableTarget = basePosition + direction * totalLength
            return try await solveFABRIKInternal(target: reachableTarget, initialGuess: initialGuess, parameters: parameters, originalTarget: targetPosition)
        } else {
            return try await solveFABRIKInternal(target: targetPosition, initialGuess: initialGuess, parameters: parameters, originalTarget: targetPosition)
        }
    }
    
    private func solveFABRIKInternal(
        target: Vector3D,
        initialGuess: [Double],
        parameters: IKParameters,
        originalTarget: Vector3D
    ) async throws -> IKSolution {
        var convergenceHistory: [Double] = []
        let basePosition = chain.baseTransform.position
        
        var jointPositions = calculateInitialJointPositions(from: initialGuess)
        
        for iteration in 0..<parameters.maxIterations {
            let currentError = jointPositions.last?.distance(to: originalTarget) ?? .infinity
            convergenceHistory.append(currentError)
            
            if currentError < parameters.tolerance {
                let jointAngles = calculateJointAngles(from: jointPositions)
                return IKSolution(
                    jointValues: clampJointValues(jointAngles),
                    success: true,
                    error: currentError,
                    iterations: iteration,
                    algorithm: .fabrik,
                    convergenceHistory: convergenceHistory
                )
            }
            
            jointPositions = forwardReaching(jointPositions: jointPositions, target: target)
            jointPositions = backwardReaching(jointPositions: jointPositions, base: basePosition)
            
            applyJointConstraints(&jointPositions)
        }
        
        let finalJointAngles = calculateJointAngles(from: jointPositions)
        
        return IKSolution(
            jointValues: clampJointValues(finalJointAngles),
            success: false,
            error: convergenceHistory.last ?? .infinity,
            iterations: parameters.maxIterations,
            algorithm: .fabrik,
            convergenceHistory: convergenceHistory
        )
    }
    
    private func calculateInitialJointPositions(from jointValues: [Double]) -> [Vector3D] {
        let transforms = calculateJointTransforms(jointValues: jointValues)
        var positions = transforms.map { $0.position }
        
        // Ensure we have base position at start
        if positions.isEmpty || positions.first?.distance(to: chain.baseTransform.position) ?? 0 > 1e-6 {
            positions.insert(chain.baseTransform.position, at: 0)
        }
        
        return positions
    }
    
    private func forwardReaching(jointPositions: [Vector3D], target: Vector3D) -> [Vector3D] {
        var positions = jointPositions
        
        if !positions.isEmpty {
            positions[positions.count - 1] = target
            
            for i in (0..<positions.count - 1).reversed() {
                let direction = (positions[i] - positions[i + 1]).normalized
                positions[i] = positions[i + 1] + direction * linkLengths[i]
            }
        }
        
        return positions
    }
    
    private func backwardReaching(jointPositions: [Vector3D], base: Vector3D) -> [Vector3D] {
        var positions = jointPositions
        
        if !positions.isEmpty {
            positions[0] = base
            
            for i in 0..<positions.count - 1 {
                let direction = (positions[i + 1] - positions[i]).normalized
                positions[i + 1] = positions[i] + direction * linkLengths[i]
            }
        }
        
        return positions
    }
    
    private func applyJointConstraints(_ jointPositions: inout [Vector3D]) {
        for i in 0..<min(jointPositions.count - 1, jointLimits.count) {
            let limits = jointLimits[i]
            
            if limits.min.isFinite && limits.max.isFinite {
                let currentAngle = calculateJointAngle(
                    previous: i > 0 ? jointPositions[i - 1] : chain.baseTransform.position,
                    current: jointPositions[i],
                    next: jointPositions[i + 1]
                )
                
                if !limits.contains(currentAngle) {
                    let clampedAngle = limits.clamp(currentAngle)
                    applyJointAngle(
                        &jointPositions,
                        jointIndex: i,
                        angle: clampedAngle
                    )
                }
            }
        }
    }
    
    private func calculateJointAngle(previous: Vector3D, current: Vector3D, next: Vector3D) -> Double {
        let v1 = (previous - current).normalized
        let v2 = (next - current).normalized
        
        let dot = v1.dot(v2)
        let clampedDot = max(-1.0, min(1.0, dot))
        
        return acos(clampedDot)
    }
    
    private func applyJointAngle(_ jointPositions: inout [Vector3D], jointIndex: Int, angle: Double) {
        guard jointIndex < jointPositions.count - 1 else { return }
        
        let current = jointPositions[jointIndex]
        let next = jointPositions[jointIndex + 1]
        let direction = (next - current).normalized
        
        let rotationAxis = Vector3D.unitZ
        let rotation = Quaternion(axis: rotationAxis, angle: angle)
        let newDirection = rotation.rotate(direction)
        
        jointPositions[jointIndex + 1] = current + newDirection * linkLengths[jointIndex]
    }
    
    private func calculateJointAngles(from jointPositions: [Vector3D]) -> [Double] {
        var angles: [Double] = []
        
        guard jointPositions.count >= 2 else {
            // Return zero angles if insufficient positions
            return Array(repeating: 0.0, count: chain.joints.count)
        }
        
        // Calculate angles between consecutive joint positions
        for i in 0..<chain.joints.count {
            let angle: Double
            
            if i == 0 {
                // First joint angle relative to base
                if jointPositions.count > 1 {
                    let direction = jointPositions[1] - jointPositions[0]
                    angle = atan2(direction.y, direction.x)
                } else {
                    angle = 0.0
                }
            } else if i < jointPositions.count - 1 {
                // Subsequent joint angles as the relative angle between links
                let prevDirection = jointPositions[i] - jointPositions[i - 1]
                let nextDirection = jointPositions[i + 1] - jointPositions[i]
                
                let prevAngle = atan2(prevDirection.y, prevDirection.x)
                let nextAngle = atan2(nextDirection.y, nextDirection.x)
                
                // Calculate relative angle
                var relativeAngle = nextAngle - prevAngle
                
                // Normalize angle to [-π, π]
                while relativeAngle > Double.pi {
                    relativeAngle -= 2 * Double.pi
                }
                while relativeAngle < -Double.pi {
                    relativeAngle += 2 * Double.pi
                }
                
                angle = relativeAngle
            } else {
                angle = 0.0
            }
            
            angles.append(angle)
        }
        
        return angles
    }
}

public actor AdvancedFABRIKSolver: InverseKinematicsSolvable {
    private let chain: KinematicChain
    private let linkLengths: [Double]
    private let totalLength: Double
    private let subBaseIndices: [Int]
    
    public init(chain: KinematicChain, subBaseIndices: [Int] = []) {
        self.chain = chain
        self.linkLengths = chain.links.map { $0.length }
        self.totalLength = linkLengths.reduce(0, +)
        self.subBaseIndices = subBaseIndices
    }
    
    public nonisolated var supportedAlgorithms: [IKAlgorithmType] {
        [.fabrik]
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
        guard algorithm == .fabrik else {
            throw IKError.unsupportedAlgorithm(algorithm)
        }
        
        let startValues = initialGuess ?? generateInitialGuess()
        guard startValues.count == jointCount else {
            throw IKError.invalidJointCount(expected: jointCount, actual: startValues.count)
        }
        
        return try await solveAdvancedFABRIK(target: target, initialGuess: startValues, parameters: parameters)
    }
    
    private func solveAdvancedFABRIK(
        target: Transform,
        initialGuess: [Double],
        parameters: IKParameters
    ) async throws -> IKSolution {
        var convergenceHistory: [Double] = []
        var jointPositions = calculateInitialJointPositions(from: initialGuess)
        
        let basePosition = chain.baseTransform.position
        let targetPosition = target.position
        
        for iteration in 0..<parameters.maxIterations {
            let currentError = calculatePositionError(jointPositions: jointPositions, target: targetPosition)
            convergenceHistory.append(currentError)
            
            if currentError < parameters.tolerance {
                let jointAngles = calculateJointAngles(from: jointPositions)
                return IKSolution(
                    jointValues: clampJointValues(jointAngles),
                    success: true,
                    error: currentError,
                    iterations: iteration,
                    algorithm: .fabrik,
                    convergenceHistory: convergenceHistory
                )
            }
            
            jointPositions = performMultiChainFABRIK(
                jointPositions: jointPositions,
                target: targetPosition,
                base: basePosition
            )
            
            applyAdvancedJointConstraints(&jointPositions)
        }
        
        let finalJointAngles = calculateJointAngles(from: jointPositions)
        
        return IKSolution(
            jointValues: clampJointValues(finalJointAngles),
            success: false,
            error: convergenceHistory.last ?? .infinity,
            iterations: parameters.maxIterations,
            algorithm: .fabrik,
            convergenceHistory: convergenceHistory
        )
    }
    
    private func performMultiChainFABRIK(
        jointPositions: [Vector3D],
        target: Vector3D,
        base: Vector3D
    ) -> [Vector3D] {
        var positions = jointPositions
        
        if subBaseIndices.isEmpty {
            positions = forwardReaching(jointPositions: positions, target: target)
            positions = backwardReaching(jointPositions: positions, base: base)
        } else {
            for subBaseIndex in subBaseIndices.reversed() {
                let subChainStart = subBaseIndex
                let subChainEnd = positions.count - 1
                
                if subChainStart < subChainEnd {
                    var subChain = Array(positions[subChainStart...subChainEnd])
                    subChain = forwardReaching(jointPositions: subChain, target: target)
                    subChain = backwardReaching(jointPositions: subChain, base: positions[subChainStart])
                    
                    for i in 0..<subChain.count {
                        positions[subChainStart + i] = subChain[i]
                    }
                }
            }
        }
        
        return positions
    }
    
    private func calculateInitialJointPositions(from jointValues: [Double]) -> [Vector3D] {
        let transforms = calculateJointTransforms(jointValues: jointValues)
        var positions = transforms.map { $0.position }
        
        // Ensure we have base position at start
        if positions.isEmpty || positions.first?.distance(to: chain.baseTransform.position) ?? 0 > 1e-6 {
            positions.insert(chain.baseTransform.position, at: 0)
        }
        
        return positions
    }
    
    private func forwardReaching(jointPositions: [Vector3D], target: Vector3D) -> [Vector3D] {
        var positions = jointPositions
        
        if !positions.isEmpty {
            positions[positions.count - 1] = target
            
            for i in (0..<positions.count - 1).reversed() {
                let segmentIndex = min(i, linkLengths.count - 1)
                let direction = (positions[i] - positions[i + 1]).normalized
                positions[i] = positions[i + 1] + direction * linkLengths[segmentIndex]
            }
        }
        
        return positions
    }
    
    private func backwardReaching(jointPositions: [Vector3D], base: Vector3D) -> [Vector3D] {
        var positions = jointPositions
        
        if !positions.isEmpty {
            positions[0] = base
            
            for i in 0..<positions.count - 1 {
                let segmentIndex = min(i, linkLengths.count - 1)
                let direction = (positions[i + 1] - positions[i]).normalized
                positions[i + 1] = positions[i] + direction * linkLengths[segmentIndex]
            }
        }
        
        return positions
    }
    
    private func applyAdvancedJointConstraints(_ jointPositions: inout [Vector3D]) {
        for i in 0..<min(jointPositions.count, jointLimits.count) {
            let limits = jointLimits[i]
            
            if limits.min.isFinite && limits.max.isFinite {
                let currentAngle = calculateJointAngleAtIndex(jointPositions, index: i)
                
                if !limits.contains(currentAngle) {
                    let clampedAngle = limits.clamp(currentAngle)
                    applyJointAngleAtIndex(&jointPositions, index: i, angle: clampedAngle)
                }
            }
        }
    }
    
    private func calculateJointAngleAtIndex(_ jointPositions: [Vector3D], index: Int) -> Double {
        guard index > 0 && index < jointPositions.count - 1 else { return 0.0 }
        
        let previous = jointPositions[index - 1]
        let current = jointPositions[index]
        let next = jointPositions[index + 1]
        
        let v1 = (previous - current).normalized
        let v2 = (next - current).normalized
        
        let dot = v1.dot(v2)
        let clampedDot = max(-1.0, min(1.0, dot))
        
        return acos(clampedDot)
    }
    
    private func applyJointAngleAtIndex(_ jointPositions: inout [Vector3D], index: Int, angle: Double) {
        guard index < jointPositions.count - 1 else { return }
        
        let current = jointPositions[index]
        let next = jointPositions[index + 1]
        let direction = (next - current).normalized
        
        let rotationAxis = Vector3D.unitZ
        let rotation = Quaternion(axis: rotationAxis, angle: angle)
        let newDirection = rotation.rotate(direction)
        
        let segmentIndex = min(index, linkLengths.count - 1)
        jointPositions[index + 1] = current + newDirection * linkLengths[segmentIndex]
    }
    
    private func calculatePositionError(jointPositions: [Vector3D], target: Vector3D) -> Double {
        guard let endEffectorPos = jointPositions.last else { return .infinity }
        return endEffectorPos.distance(to: target)
    }
    
    private func calculateJointAngles(from jointPositions: [Vector3D]) -> [Double] {
        var angles: [Double] = []
        
        guard jointPositions.count >= 2 else {
            // Return zero angles if insufficient positions
            return Array(repeating: 0.0, count: chain.joints.count)
        }
        
        // Calculate angles between consecutive joint positions
        for i in 0..<chain.joints.count {
            let angle: Double
            
            if i == 0 {
                // First joint angle relative to base
                if jointPositions.count > 1 {
                    let direction = jointPositions[1] - jointPositions[0]
                    angle = atan2(direction.y, direction.x)
                } else {
                    angle = 0.0
                }
            } else if i < jointPositions.count - 1 {
                // Subsequent joint angles as the relative angle between links
                let prevDirection = jointPositions[i] - jointPositions[i - 1]
                let nextDirection = jointPositions[i + 1] - jointPositions[i]
                
                let prevAngle = atan2(prevDirection.y, prevDirection.x)
                let nextAngle = atan2(nextDirection.y, nextDirection.x)
                
                // Calculate relative angle
                var relativeAngle = nextAngle - prevAngle
                
                // Normalize angle to [-π, π]
                while relativeAngle > Double.pi {
                    relativeAngle -= 2 * Double.pi
                }
                while relativeAngle < -Double.pi {
                    relativeAngle += 2 * Double.pi
                }
                
                angle = relativeAngle
            } else {
                angle = 0.0
            }
            
            angles.append(angle)
        }
        
        return angles
    }
}