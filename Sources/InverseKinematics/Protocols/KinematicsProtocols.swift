#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif

public enum IKAlgorithmType: Sendable, Codable, CaseIterable {
    case analytical
    case jacobianTranspose
    case dampedLeastSquares
    case selectivelyDampedLeastSquares
    case cyclicCoordinateDescent
    case fabrik
    case newtonRaphson
}

public struct IKParameters: Sendable, Codable {
    public let tolerance: Double
    public let maxIterations: Int
    public let dampingFactor: Double
    public let stepSize: Double
    public let positionWeight: Double
    public let orientationWeight: Double
    
    public init(
        tolerance: Double = 1e-6,
        maxIterations: Int = 100,
        dampingFactor: Double = 0.01,
        stepSize: Double = 0.1,
        positionWeight: Double = 1.0,
        orientationWeight: Double = 1.0
    ) {
        self.tolerance = max(0, tolerance)
        self.maxIterations = max(1, maxIterations)
        self.dampingFactor = max(0, dampingFactor)
        self.stepSize = max(0, stepSize)
        self.positionWeight = max(0, positionWeight)
        self.orientationWeight = max(0, orientationWeight)
    }
    
    public static let `default` = IKParameters()
}

public struct IKSolution: Sendable, Codable {
    public let jointValues: [Double]
    public let success: Bool
    public let error: Double
    public let iterations: Int
    public let algorithm: IKAlgorithmType
    public let convergenceHistory: [Double]?
    
    public init(
        jointValues: [Double],
        success: Bool,
        error: Double,
        iterations: Int,
        algorithm: IKAlgorithmType,
        convergenceHistory: [Double]? = nil
    ) {
        self.jointValues = jointValues
        self.success = success
        self.error = max(0, error)
        self.iterations = max(0, iterations)
        self.algorithm = algorithm
        self.convergenceHistory = convergenceHistory
    }
    
    public static let failed = IKSolution(
        jointValues: [],
        success: false,
        error: .infinity,
        iterations: 0,
        algorithm: .analytical
    )
}

public extension IKSolution {
    /// Determines if the solution has converged based on the error metric and tolerance
    /// - Parameter tolerance: The tolerance threshold for convergence
    /// - Returns: true if the error is below or equal to the tolerance and the solution is valid
    func isConverged(tolerance: Double) -> Bool {
        guard !jointValues.isEmpty else { return false }
        guard error.isFinite else { return false }
        return error <= tolerance
    }
    
    /// Determines if the solution has converged using the default IK parameters tolerance
    /// - Returns: true if the error is below the default tolerance and the solution is valid
    func isConverged() -> Bool {
        return isConverged(tolerance: IKParameters.default.tolerance)
    }
    
    /// Analyzes the convergence behavior from the history
    /// - Returns: A tuple containing convergence analysis information
    func convergenceAnalysis() -> (hasImproved: Bool, finalImprovement: Double?, averageImprovement: Double?) {
        guard let history = convergenceHistory, history.count > 1 else {
            return (false, nil, nil)
        }
        
        let initialError = history.first!
        let finalError = history.last!
        let hasImproved = finalError < initialError
        let finalImprovement = initialError - finalError
        
        // Calculate average improvement per iteration
        var totalImprovement = 0.0
        var improvementCount = 0
        
        for i in 1..<history.count {
            let improvement = history[i-1] - history[i]
            if improvement > 0 {
                totalImprovement += improvement
                improvementCount += 1
            }
        }
        
        let averageImprovement = improvementCount > 0 ? totalImprovement / Double(improvementCount) : nil
        
        return (hasImproved, finalImprovement, averageImprovement)
    }
}

public protocol ForwardKinematicsCalculable: Sendable {
    func calculateEndEffector(jointValues: [Double]) -> Transform
    func calculateJointTransforms(jointValues: [Double]) -> [Transform]
    func calculateJacobian(jointValues: [Double], epsilon: Double) -> [[Double]]
}

public protocol InverseKinematicsSolvable: ForwardKinematicsCalculable {
    var supportedAlgorithms: [IKAlgorithmType] { get }
    var jointCount: Int { get }
    var jointLimits: [JointLimits] { get }
    
    func solveIK(
        target: Transform,
        initialGuess: [Double]?,
        algorithm: IKAlgorithmType,
        parameters: IKParameters
    ) async throws -> IKSolution
    
    func solveIK(
        target: Transform,
        initialGuess: [Double]?,
        parameters: IKParameters
    ) async throws -> IKSolution
}

public extension InverseKinematicsSolvable {
    func solveIK(
        target: Transform,
        initialGuess: [Double]? = nil,
        parameters: IKParameters = .default
    ) async throws -> IKSolution {
        let algorithm = supportedAlgorithms.first ?? .dampedLeastSquares
        return try await solveIK(
            target: target,
            initialGuess: initialGuess,
            algorithm: algorithm,
            parameters: parameters
        )
    }
    
    func solveIK(
        target: Transform,
        algorithm: IKAlgorithmType,
        parameters: IKParameters = .default
    ) async throws -> IKSolution {
        try await solveIK(
            target: target,
            initialGuess: nil,
            algorithm: algorithm,
            parameters: parameters
        )
    }
    
    func calculateError(target: Transform, current: Transform, parameters: IKParameters) -> Double {
        let positionError = target.position.distance(to: current.position) * parameters.positionWeight
        
        let rotationDifference = target.rotation.inverse * current.rotation
        let eulerError = rotationDifference.eulerAngles.magnitude * parameters.orientationWeight
        
        return positionError + eulerError
    }
    
    func generateInitialGuess(near target: Transform? = nil) -> [Double] {
        var guess: [Double] = []
        
        for limits in jointLimits {
            if limits.min.isFinite && limits.max.isFinite {
                let range = limits.max - limits.min
                let randomValue = limits.min + Double.random(in: 0...1) * range
                guess.append(randomValue)
            } else {
                guess.append(0.0)
            }
        }
        
        return guess
    }
    
    func clampJointValues(_ values: [Double]) -> [Double] {
        guard values.count == jointLimits.count else {
            // If joint limits count doesn't match, return values as-is
            return values
        }
        
        return zip(values, jointLimits).map { value, limits in
            limits.clamp(value)
        }
    }
}

public enum IKError: Error, Sendable, Equatable {
    case invalidJointCount(expected: Int, actual: Int)
    case unsupportedAlgorithm(IKAlgorithmType)
    case singularConfiguration
    case unreachableTarget
    case convergenceFailed(iterations: Int, finalError: Double)
    case invalidParameters(String)
    case numericalInstability
}

extension IKError: LocalizedError {
    public var errorDescription: String? {
        switch self {
        case .invalidJointCount(let expected, let actual):
            return "Invalid joint count: expected \(expected), got \(actual)"
        case .unsupportedAlgorithm(let algorithm):
            return "Unsupported algorithm: \(algorithm)"
        case .singularConfiguration:
            return "Singular configuration encountered"
        case .unreachableTarget:
            return "Target is unreachable"
        case .convergenceFailed(let iterations, let finalError):
            return "Convergence failed after \(iterations) iterations (final error: \(finalError))"
        case .invalidParameters(let message):
            return "Invalid parameters: \(message)"
        case .numericalInstability:
            return "Numerical instability detected"
        }
    }
}