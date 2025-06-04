#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif

public actor JacobianBasedSolver: InverseKinematicsSolvable {
    private let chain: KinematicChain
    
    public init(chain: KinematicChain) {
        self.chain = chain
    }
    
    public nonisolated var supportedAlgorithms: [IKAlgorithmType] {
        [.jacobianTranspose, .dampedLeastSquares, .selectivelyDampedLeastSquares]
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
        guard supportedAlgorithms.contains(algorithm) else {
            throw IKError.unsupportedAlgorithm(algorithm)
        }
        
        let startValues = initialGuess ?? generateInitialGuess()
        guard startValues.count == jointCount else {
            throw IKError.invalidJointCount(expected: jointCount, actual: startValues.count)
        }
        
        switch algorithm {
        case .jacobianTranspose:
            return try await solveJacobianTranspose(target: target, initialGuess: startValues, parameters: parameters)
        case .dampedLeastSquares:
            return try await solveDampedLeastSquares(target: target, initialGuess: startValues, parameters: parameters)
        case .selectivelyDampedLeastSquares:
            return try await solveSelectivelyDampedLeastSquares(target: target, initialGuess: startValues, parameters: parameters)
        default:
            throw IKError.unsupportedAlgorithm(algorithm)
        }
    }
    
    private func solveJacobianTranspose(
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
                    algorithm: .jacobianTranspose,
                    convergenceHistory: convergenceHistory
                )
            }
            
            let jacobian = calculateJacobian(jointValues: currentJoints)
            let jacobianT = transpose(jacobian)
            
            let positionError = target.position - currentTransform.position
            let rotationError = calculateRotationError(target: target.rotation, current: currentTransform.rotation)
            
            let errorVector = [
                positionError.x * parameters.positionWeight,
                positionError.y * parameters.positionWeight,
                positionError.z * parameters.positionWeight,
                rotationError.x * parameters.orientationWeight,
                rotationError.y * parameters.orientationWeight,
                rotationError.z * parameters.orientationWeight
            ]
            
            let deltaJoints = multiplyMatrixVector(jacobianT, errorVector)
            
            for i in 0..<currentJoints.count {
                currentJoints[i] += deltaJoints[i] * parameters.stepSize
            }
            
            currentJoints = clampJointValues(currentJoints)
        }
        
        return IKSolution(
            jointValues: currentJoints,
            success: false,
            error: convergenceHistory.last ?? .infinity,
            iterations: parameters.maxIterations,
            algorithm: .jacobianTranspose,
            convergenceHistory: convergenceHistory
        )
    }
    
    private func solveDampedLeastSquares(
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
                    algorithm: .dampedLeastSquares,
                    convergenceHistory: convergenceHistory
                )
            }
            
            let jacobian = calculateJacobian(jointValues: currentJoints)
            let jacobianT = transpose(jacobian)
            
            let positionError = target.position - currentTransform.position
            let rotationError = calculateRotationError(target: target.rotation, current: currentTransform.rotation)
            
            let errorVector = [
                positionError.x * parameters.positionWeight,
                positionError.y * parameters.positionWeight,
                positionError.z * parameters.positionWeight,
                rotationError.x * parameters.orientationWeight,
                rotationError.y * parameters.orientationWeight,
                rotationError.z * parameters.orientationWeight
            ]
            
            let jjt = multiplyMatrices(jacobian, jacobianT)
            let identity = createIdentityMatrix(size: jjt.count)
            let damped = addMatrices(jjt, scaleMatrix(identity, parameters.dampingFactor * parameters.dampingFactor))
            
            guard let dampedInverse = invertMatrix(damped) else {
                throw IKError.numericalInstability
            }
            
            let pseudoInverse = multiplyMatrices(jacobianT, dampedInverse)
            let deltaJoints = multiplyMatrixVector(pseudoInverse, errorVector)
            
            for i in 0..<currentJoints.count {
                currentJoints[i] += deltaJoints[i] * parameters.stepSize
            }
            
            currentJoints = clampJointValues(currentJoints)
        }
        
        return IKSolution(
            jointValues: currentJoints,
            success: false,
            error: convergenceHistory.last ?? .infinity,
            iterations: parameters.maxIterations,
            algorithm: .dampedLeastSquares,
            convergenceHistory: convergenceHistory
        )
    }
    
    private func solveSelectivelyDampedLeastSquares(
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
                    algorithm: .selectivelyDampedLeastSquares,
                    convergenceHistory: convergenceHistory
                )
            }
            
            let jacobian = calculateJacobian(jointValues: currentJoints)
            // Note: jacobianT not needed for SDLS which uses SVD
            
            let positionError = target.position - currentTransform.position
            let rotationError = calculateRotationError(target: target.rotation, current: currentTransform.rotation)
            
            let errorVector = [
                positionError.x * parameters.positionWeight,
                positionError.y * parameters.positionWeight,
                positionError.z * parameters.positionWeight,
                rotationError.x * parameters.orientationWeight,
                rotationError.y * parameters.orientationWeight,
                rotationError.z * parameters.orientationWeight
            ]
            
            let svd = performSVD(jacobian)
            let pseudoInverse = computeSelectivelyDampedPseudoInverse(svd: svd, damping: parameters.dampingFactor)
            let deltaJoints = multiplyMatrixVector(pseudoInverse, errorVector)
            
            for i in 0..<currentJoints.count {
                currentJoints[i] += deltaJoints[i] * parameters.stepSize
            }
            
            currentJoints = clampJointValues(currentJoints)
        }
        
        return IKSolution(
            jointValues: currentJoints,
            success: false,
            error: convergenceHistory.last ?? .infinity,
            iterations: parameters.maxIterations,
            algorithm: .selectivelyDampedLeastSquares,
            convergenceHistory: convergenceHistory
        )
    }
    
    private func calculateRotationError(target: Quaternion, current: Quaternion) -> Vector3D {
        let errorQuat = target * current.inverse
        return errorQuat.eulerAngles
    }
    
    private func transpose(_ matrix: [[Double]]) -> [[Double]] {
        guard !matrix.isEmpty else { return [] }
        let rows = matrix.count
        let cols = matrix[0].count
        
        var result = Array(repeating: Array(repeating: 0.0, count: rows), count: cols)
        
        for i in 0..<rows {
            for j in 0..<cols {
                result[j][i] = matrix[i][j]
            }
        }
        
        return result
    }
    
    private func multiplyMatrices(_ a: [[Double]], _ b: [[Double]]) -> [[Double]] {
        let rowsA = a.count
        let colsA = a[0].count
        let colsB = b[0].count
        
        var result = Array(repeating: Array(repeating: 0.0, count: colsB), count: rowsA)
        
        for i in 0..<rowsA {
            for j in 0..<colsB {
                for k in 0..<colsA {
                    result[i][j] += a[i][k] * b[k][j]
                }
            }
        }
        
        return result
    }
    
    private func multiplyMatrixVector(_ matrix: [[Double]], _ vector: [Double]) -> [Double] {
        var result = Array(repeating: 0.0, count: matrix.count)
        
        for i in 0..<matrix.count {
            for j in 0..<vector.count {
                result[i] += matrix[i][j] * vector[j]
            }
        }
        
        return result
    }
    
    private func addMatrices(_ a: [[Double]], _ b: [[Double]]) -> [[Double]] {
        var result = a
        
        for i in 0..<a.count {
            for j in 0..<a[0].count {
                result[i][j] += b[i][j]
            }
        }
        
        return result
    }
    
    private func scaleMatrix(_ matrix: [[Double]], _ scale: Double) -> [[Double]] {
        var result = matrix
        
        for i in 0..<matrix.count {
            for j in 0..<matrix[0].count {
                result[i][j] *= scale
            }
        }
        
        return result
    }
    
    private func createIdentityMatrix(size: Int) -> [[Double]] {
        var matrix = Array(repeating: Array(repeating: 0.0, count: size), count: size)
        
        for i in 0..<size {
            matrix[i][i] = 1.0
        }
        
        return matrix
    }
    
    private func invertMatrix(_ matrix: [[Double]]) -> [[Double]]? {
        let n = matrix.count
        var a = matrix
        var inv = createIdentityMatrix(size: n)
        
        for i in 0..<n {
            var maxRow = i
            for k in i + 1..<n {
                if abs(a[k][i]) > abs(a[maxRow][i]) {
                    maxRow = k
                }
            }
            
            if abs(a[maxRow][i]) < 1e-12 {
                return nil
            }
            
            if maxRow != i {
                a.swapAt(i, maxRow)
                inv.swapAt(i, maxRow)
            }
            
            let pivot = a[i][i]
            for j in 0..<n {
                a[i][j] /= pivot
                inv[i][j] /= pivot
            }
            
            for k in 0..<n {
                if k != i {
                    let factor = a[k][i]
                    for j in 0..<n {
                        a[k][j] -= factor * a[i][j]
                        inv[k][j] -= factor * inv[i][j]
                    }
                }
            }
        }
        
        return inv
    }
    
    private struct SVD {
        let u: [[Double]]
        let s: [Double]
        let vt: [[Double]]
    }
    
    private func performSVD(_ matrix: [[Double]]) -> SVD {
        return SVD(u: matrix, s: Array(repeating: 1.0, count: min(matrix.count, matrix[0].count)), vt: transpose(matrix))
    }
    
    private func computeSelectivelyDampedPseudoInverse(svd: SVD, damping: Double) -> [[Double]] {
        let threshold = 1e-6
        var dampedS: [Double] = []
        
        for s in svd.s {
            if s > threshold {
                dampedS.append(s / (s * s + damping * damping))
            } else {
                dampedS.append(0.0)
            }
        }
        
        var result = Array(repeating: Array(repeating: 0.0, count: svd.u.count), count: svd.vt[0].count)
        
        for i in 0..<result.count {
            for j in 0..<result[0].count {
                for k in 0..<dampedS.count {
                    if k < svd.vt.count && k < svd.u[0].count {
                        result[i][j] += svd.vt[k][i] * dampedS[k] * svd.u[j][k]
                    }
                }
            }
        }
        
        return result
    }
}