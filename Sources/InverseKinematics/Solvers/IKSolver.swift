//
//  IKSolver.swift
//  InverseKinematics
//
//  Base protocol for IK solvers
//

#if canImport(FoundationEssentials)
import FoundationEssentials
#elseif canImport(Foundation)
import Foundation
#endif

/// The type of IK solver to use
public enum IKSolverType {
    /// Cyclic Coordinate Descent - Fast, heuristic method
    case ccd
    
    /// Forward And Backward Reaching Inverse Kinematics - Natural-looking chains
    case fabrik
    
    /// Jacobian-based method - Precise control for robotics
    case jacobian
}

/// Protocol that all IK solvers must conform to
public protocol IKSolver {
    /// The chain this solver operates on
    var chain: IKChain { get }
    
    /// Initializes the solver with an IK chain
    init(chain: IKChain)
    
    /// Solves the IK problem for the chain
    /// - Returns: True if the solution converged, false otherwise
    func solve() -> Bool
}

/// Factory for creating IK solvers
public struct IKSolverFactory {
    /// Creates an IK solver of the specified type
    /// - Parameters:
    ///   - type: The type of solver to create
    ///   - chain: The IK chain to solve
    /// - Returns: An IK solver instance
    public static func createSolver(type: IKSolverType, chain: IKChain) -> IKSolver {
        switch type {
        case .ccd:
            return CCDSolver(chain: chain)
        case .fabrik:
            return FABRIKSolver(chain: chain)
        case .jacobian:
            return JacobianSolver(chain: chain)
        }
    }
}
