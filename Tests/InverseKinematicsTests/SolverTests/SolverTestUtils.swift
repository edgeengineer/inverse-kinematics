import Testing
#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif
@testable import InverseKinematics

/// Shared utilities and helper functions for solver tests
enum SolverTestUtils {
    
    // MARK: - Floating-Point Comparison Helpers
    
    /// Default tolerance for floating-point comparisons
    static let defaultTolerance: Double = 1e-10
    
    /// Default position tolerance for IK solutions
    static let defaultPositionTolerance: Double = 1e-6
    
    /// Default orientation tolerance for IK solutions
    static let defaultOrientationTolerance: Double = 1e-6
    
    /// Checks if two floating-point values are approximately equal within tolerance
    static func isApproximatelyEqual(_ a: Double, _ b: Double, tolerance: Double = defaultTolerance) -> Bool {
        return abs(a - b) < tolerance
    }
    
    /// Checks if a floating-point value is approximately zero within tolerance
    static func isApproximatelyZero(_ value: Double, tolerance: Double = defaultTolerance) -> Bool {
        return abs(value) < tolerance
    }
    
    /// Enhanced floating-point comparison that handles special cases (NaN, infinity)
    static func isApproximatelyEqualSafe(_ a: Double, _ b: Double, tolerance: Double = defaultTolerance) -> Bool {
        // Handle special cases
        if a.isNaN || b.isNaN { return false }
        if a.isInfinite && b.isInfinite { return a.sign == b.sign }
        if a.isInfinite || b.isInfinite { return false }
        
        // Handle zero cases with absolute tolerance
        if abs(a) < tolerance && abs(b) < tolerance { return true }
        
        // Use relative tolerance for larger numbers
        let relativeTolerance = tolerance * max(abs(a), abs(b))
        return abs(a - b) <= max(tolerance, relativeTolerance)
    }
    
    /// Checks if a value is within a specified range (inclusive)
    static func isWithinRange(_ value: Double, min: Double, max: Double) -> Bool {
        return value >= min && value <= max
    }
    
    /// Checks if two Vector3D values are approximately equal within tolerance
    static func isApproximatelyEqual(_ a: Vector3D, _ b: Vector3D, tolerance: Double = defaultTolerance) -> Bool {
        return isApproximatelyEqual(a.x, b.x, tolerance: tolerance) &&
               isApproximatelyEqual(a.y, b.y, tolerance: tolerance) &&
               isApproximatelyEqual(a.z, b.z, tolerance: tolerance)
    }
    
    /// Checks if two Quaternion values are approximately equal within tolerance
    static func isApproximatelyEqual(_ a: Quaternion, _ b: Quaternion, tolerance: Double = defaultTolerance) -> Bool {
        // Handle quaternion double-cover: q and -q represent the same rotation
        let direct = isApproximatelyEqual(a.x, b.x, tolerance: tolerance) &&
                    isApproximatelyEqual(a.y, b.y, tolerance: tolerance) &&
                    isApproximatelyEqual(a.z, b.z, tolerance: tolerance) &&
                    isApproximatelyEqual(a.w, b.w, tolerance: tolerance)
        
        let negated = isApproximatelyEqual(a.x, -b.x, tolerance: tolerance) &&
                     isApproximatelyEqual(a.y, -b.y, tolerance: tolerance) &&
                     isApproximatelyEqual(a.z, -b.z, tolerance: tolerance) &&
                     isApproximatelyEqual(a.w, -b.w, tolerance: tolerance)
        
        return direct || negated
    }
    
    /// Checks if two Transform values are approximately equal within tolerance
    static func isApproximatelyEqual(_ a: Transform, _ b: Transform, 
                                   positionTolerance: Double = defaultPositionTolerance,
                                   orientationTolerance: Double = defaultOrientationTolerance) -> Bool {
        return isApproximatelyEqual(a.position, b.position, tolerance: positionTolerance) &&
               isApproximatelyEqual(a.rotation, b.rotation, tolerance: orientationTolerance)
    }
    
    /// Verifies that an IK solution is valid for the given target
    static func verifyIKSolution(
        chain: KinematicChain,
        solution: IKSolution,
        target: Transform,
        positionTolerance: Double = defaultPositionTolerance,
        orientationTolerance: Double = defaultOrientationTolerance
    ) -> Bool {
        guard solution.success else { return false }
        
        // Calculate the end-effector pose directly
        let endEffector = chain.endEffectorTransform(jointValues: solution.jointValues)
        
        // Check if the end-effector matches the target within tolerance
        return isApproximatelyEqual(endEffector, target, 
                                  positionTolerance: positionTolerance,
                                  orientationTolerance: orientationTolerance)
    }
    
    // MARK: - Standard Robot Configurations
    
    /// Creates a simple two-joint kinematic chain for testing
    static func createSimpleTwoJointChain() -> KinematicChain {
        var chain = KinematicChain(id: "two_joint")
        
        let joint1 = Joint(
            id: "j1",
            type: .revolute,
            axis: Vector3D.unitZ,
            limits: JointLimits(min: -Double.pi, max: Double.pi)
        )
        
        let joint2 = Joint(
            id: "j2",
            type: .revolute,
            axis: Vector3D.unitZ,
            limits: JointLimits(min: -Double.pi, max: Double.pi),
            parentTransform: Transform(position: Vector3D(x: 1.0, y: 0.0, z: 0.0))
        )
        
        let link1 = Link(id: "l1", length: 1.0)
        let link2 = Link(id: "l2", length: 1.0)
        
        chain.addJoint(joint1)
        chain.addJoint(joint2)
        chain.addLink(link1)
        chain.addLink(link2)
        
        return chain
    }
    
    /// Creates a three-joint kinematic chain for testing
    static func createThreeJointChain() -> KinematicChain {
        var chain = KinematicChain(id: "three_joint")
        
        let joint1 = Joint(
            id: "j1",
            type: .revolute,
            axis: Vector3D.unitZ,
            limits: JointLimits(min: -Double.pi, max: Double.pi)
        )
        
        let joint2 = Joint(
            id: "j2",
            type: .revolute,
            axis: Vector3D.unitZ,
            limits: JointLimits(min: -Double.pi, max: Double.pi),
            parentTransform: Transform(position: Vector3D(x: 1.0, y: 0.0, z: 0.0))
        )
        
        let joint3 = Joint(
            id: "j3",
            type: .revolute,
            axis: Vector3D.unitZ,
            limits: JointLimits(min: -Double.pi, max: Double.pi),
            parentTransform: Transform(position: Vector3D(x: 1.0, y: 0.0, z: 0.0))
        )
        
        let link1 = Link(id: "l1", length: 1.0)
        let link2 = Link(id: "l2", length: 1.0)
        let link3 = Link(id: "l3", length: 0.5)
        
        chain.addJoint(joint1)
        chain.addJoint(joint2)
        chain.addJoint(joint3)
        chain.addLink(link1)
        chain.addLink(link2)
        chain.addLink(link3)
        
        return chain
    }
    
    /// Creates a variable-length kinematic chain for testing
    static func createVariableJointChain(jointCount: Int) -> KinematicChain {
        var chain = KinematicChain(id: "variable_\(jointCount)_joint")
        
        for i in 0..<jointCount {
            let joint = Joint(
                id: "j\(i + 1)",
                type: .revolute,
                axis: Vector3D.unitZ,
                limits: JointLimits(min: -Double.pi, max: Double.pi),
                parentTransform: i == 0 ? Transform.identity : Transform(position: Vector3D(x: 1.0, y: 0.0, z: 0.0))
            )
            
            let link = Link(id: "l\(i + 1)", length: 1.0)
            
            chain.addJoint(joint)
            chain.addLink(link)
        }
        
        return chain
    }
    
    /// Calculates two-link forward kinematics for validation
    static func calculateTwoLinkForwardKinematics(
        link1: Double,
        link2: Double,
        joint1: Double,
        joint2: Double
    ) -> Vector3D {
        let x = link1 * cos(joint1) + link2 * cos(joint1 + joint2)
        let y = link1 * sin(joint1) + link2 * sin(joint1 + joint2)
        return Vector3D(x: x, y: y, z: 0.0)
    }
    
    // MARK: - Standard Test Targets
    
    /// Standard test targets for a two-link planar arm with link lengths [1.0, 1.0]
    static func standardTwoLinkTargets() -> [Transform] {
        return [
            // Easily reachable targets
            Transform(position: Vector3D(x: 1.5, y: 0.5, z: 0.0)),
            Transform(position: Vector3D(x: 0.0, y: 1.0, z: 0.0)),
            Transform(position: Vector3D(x: -1.0, y: 1.0, z: 0.0)),
            
            // Edge of workspace
            Transform(position: Vector3D(x: 2.0, y: 0.0, z: 0.0)),    // Fully extended
            Transform(position: Vector3D(x: -2.0, y: 0.0, z: 0.0)),   // Fully extended backward
            Transform(position: Vector3D(x: 0.0, y: 2.0, z: 0.0)),    // Fully extended upward
            Transform(position: Vector3D(x: 0.0, y: -2.0, z: 0.0)),   // Fully extended downward
            
            // Unreachable targets (outside workspace)
            Transform(position: Vector3D(x: 3.0, y: 0.0, z: 0.0)),    // Too far
            Transform(position: Vector3D(x: 0.0, y: 3.0, z: 0.0)),    // Too far up
            Transform(position: Vector3D(x: 0.1, y: 0.0, z: 0.0)),    // Too close (inside minimum reach)
            
            // Singular configurations
            Transform(position: Vector3D(x: 0.0, y: 0.0, z: 0.0)),    // At base (singularity)
        ]
    }
    
    /// Standard test targets for a three-link chain
    static func standardThreeLinkTargets() -> [Transform] {
        return [
            // Easily reachable targets
            Transform(position: Vector3D(x: 2.0, y: 0.5, z: 0.0)),
            Transform(position: Vector3D(x: 1.0, y: 1.5, z: 0.0)),
            Transform(position: Vector3D(x: -1.5, y: 1.0, z: 0.0)),
            
            // Edge of workspace
            Transform(position: Vector3D(x: 2.5, y: 0.0, z: 0.0)),    // Near maximum reach
            Transform(position: Vector3D(x: -2.5, y: 0.0, z: 0.0)),   // Near maximum reach backward
            
            // Unreachable targets
            Transform(position: Vector3D(x: 4.0, y: 0.0, z: 0.0)),    // Too far
            Transform(position: Vector3D(x: 0.2, y: 0.0, z: 0.0)),    // Too close
        ]
    }
    
    // MARK: - Boundary and Error Test Helpers
    
    /// Creates test data for boundary conditions
    static func boundaryTestValues() -> [Double] {
        return [
            0.0,
            Double.ulpOfOne,
            -Double.ulpOfOne,
            1e-15,
            -1e-15,
            1e15,
            -1e15,
            Double.pi,
            -Double.pi,
            2 * Double.pi,
            -2 * Double.pi
        ]
    }
    
    /// Creates invalid floating-point values for negative testing
    static func invalidFloatingPointValues() -> [Double] {
        return [
            Double.nan,
            Double.infinity,
            -Double.infinity
        ]
    }
    
    /// Creates invalid Vector3D values for negative testing
    static func invalidVector3DValues() -> [Vector3D] {
        return [
            Vector3D(x: Double.nan, y: 1.0, z: 1.0),
            Vector3D(x: 1.0, y: Double.nan, z: 1.0),
            Vector3D(x: 1.0, y: 1.0, z: Double.nan),
            Vector3D(x: Double.infinity, y: 1.0, z: 1.0),
            Vector3D(x: 1.0, y: Double.infinity, z: 1.0),
            Vector3D(x: 1.0, y: 1.0, z: Double.infinity),
            Vector3D(x: -Double.infinity, y: 1.0, z: 1.0)
        ]
    }
    
    /// Creates invalid joint value arrays for negative testing
    static func invalidJointValueArrays(expectedCount: Int) -> [[Double]] {
        return [
            [], // Empty array
            Array(repeating: 0.0, count: expectedCount - 1), // Too few values
            Array(repeating: 0.0, count: expectedCount + 1), // Too many values
            Array(repeating: Double.nan, count: expectedCount), // NaN values
            Array(repeating: Double.infinity, count: expectedCount), // Infinite values
        ]
    }
    
    /// Creates edge case Transform values for testing
    static func edgeCaseTransforms() -> [Transform] {
        return [
            Transform.identity,
            Transform(position: Vector3D.zero, rotation: Quaternion.identity),
            Transform(position: Vector3D(x: 1e-15, y: 1e-15, z: 1e-15)), // Very small position
            Transform(position: Vector3D(x: 1e15, y: 1e15, z: 1e15)), // Very large position
            Transform(rotation: Quaternion(axis: Vector3D.unitX, angle: 1e-15)), // Very small rotation
            Transform(rotation: Quaternion(axis: Vector3D.unitX, angle: 100 * Double.pi)), // Large rotation
        ]
    }
}