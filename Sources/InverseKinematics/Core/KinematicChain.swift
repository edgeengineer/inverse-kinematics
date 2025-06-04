#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif

public struct KinematicChain: Sendable, Codable {
    public let id: String
    public private(set) var joints: [Joint]
    public private(set) var links: [Link]
    public let baseTransform: Transform
    
    public init(
        id: String,
        joints: [Joint] = [],
        links: [Link] = [],
        baseTransform: Transform = .identity
    ) {
        self.id = id
        self.joints = joints
        self.links = links
        self.baseTransform = baseTransform
    }
    
    public var jointCount: Int { joints.count }
    public var linkCount: Int { links.count }
    
    public var jointValues: [Double] {
        joints.map { $0.value }
    }
    
    public mutating func setJointValues(_ values: [Double]) {
        precondition(values.count == joints.count, "Joint values count must match joints count")
        for i in 0..<joints.count {
            joints[i].setValue(values[i])
        }
    }
    
    public func withJointValues(_ values: [Double]) -> KinematicChain {
        var chain = self
        chain.setJointValues(values)
        return chain
    }
    
    public mutating func addJoint(_ joint: Joint) {
        joints.append(joint)
    }
    
    public mutating func addLink(_ link: Link) {
        links.append(link)
    }
    
    public mutating func insertJoint(_ joint: Joint, at index: Int) {
        joints.insert(joint, at: index)
    }
    
    public mutating func insertLink(_ link: Link, at index: Int) {
        links.insert(link, at: index)
    }
    
    public mutating func removeJoint(at index: Int) {
        joints.remove(at: index)
    }
    
    public mutating func removeLink(at index: Int) {
        links.remove(at: index)
    }
    
    public func getJoint(withId id: String) -> Joint? {
        joints.first { $0.id == id }
    }
    
    public func getLink(withId id: String) -> Link? {
        links.first { $0.id == id }
    }
    
    public func jointIndex(withId id: String) -> Int? {
        joints.firstIndex { $0.id == id }
    }
    
    public func linkIndex(withId id: String) -> Int? {
        links.firstIndex { $0.id == id }
    }
    
    public func forwardKinematics(jointValues: [Double]? = nil) -> [Transform] {
        let values = jointValues ?? self.jointValues
        precondition(values.count == joints.count, "Joint values count must match joints count")
        
        var transforms: [Transform] = []
        var currentTransform = baseTransform
        
        for i in 0..<joints.count {
            let joint = joints[i].withValue(values[i])
            
            // Apply parent transform for this joint
            currentTransform = currentTransform * joint.parentTransform
            
            // Apply joint transformation
            currentTransform = currentTransform * joint.transform
            transforms.append(currentTransform)
            
            // Apply link transformation if available
            if i < links.count {
                let link = links[i]
                currentTransform = currentTransform * link.endTransform
            }
        }
        
        return transforms
    }
    
    public func endEffectorTransform(jointValues: [Double]? = nil) -> Transform {
        let transforms = forwardKinematics(jointValues: jointValues)
        return transforms.last ?? baseTransform
    }
    
    public func jacobian(
        jointValues: [Double]? = nil, 
        epsilon: Double = 1e-6,
        config: PerformanceConfig = .balanced
    ) -> [[Double]] {
        let values = jointValues ?? self.jointValues
        let endEffector = endEffectorTransform(jointValues: values)
        
        var jacobian: [[Double]] = Array(repeating: Array(repeating: 0.0, count: joints.count), count: 6)
        
        for i in 0..<joints.count {
            var perturbedValues = values
            perturbedValues[i] += epsilon
            
            let perturbedEndEffector = endEffectorTransform(jointValues: perturbedValues)
            
            let deltaPosition = (perturbedEndEffector.position - endEffector.position) / epsilon
            jacobian[0][i] = deltaPosition.x
            jacobian[1][i] = deltaPosition.y
            jacobian[2][i] = deltaPosition.z
            
            let deltaRotation = perturbedEndEffector.rotation.inverse * endEffector.rotation
            let deltaEuler = deltaRotation.eulerAngles / epsilon
            jacobian[3][i] = deltaEuler.x
            jacobian[4][i] = deltaEuler.y
            jacobian[5][i] = deltaEuler.z
        }
        
        return jacobian
    }
    
    public func workspace(samples: Int = 1000) -> [Vector3D] {
        var points: [Vector3D] = []
        
        for _ in 0..<samples {
            var randomValues: [Double] = []
            
            for joint in joints {
                let range = joint.limits.max - joint.limits.min
                let randomValue = joint.limits.min + Double.random(in: 0...1) * range
                randomValues.append(randomValue)
            }
            
            let endEffector = endEffectorTransform(jointValues: randomValues)
            points.append(endEffector.position)
        }
        
        return points
    }
}

extension KinematicChain: Equatable {
    public static func == (lhs: KinematicChain, rhs: KinematicChain) -> Bool {
        lhs.id == rhs.id &&
        lhs.joints == rhs.joints &&
        lhs.links == rhs.links &&
        lhs.baseTransform == rhs.baseTransform
    }
}

extension KinematicChain: CustomStringConvertible {
    public var description: String {
        "KinematicChain(id: \"\(id)\", joints: \(joints.count), links: \(links.count))"
    }
}

extension KinematicChain: Identifiable {}