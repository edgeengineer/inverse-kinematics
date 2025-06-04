import Foundation

public enum JointType: Sendable, Codable {
    case revolute
    case prismatic
    case spherical
    case planar
    case cylindrical
    case fixed
}

public struct JointLimits: Sendable, Codable {
    public let min: Double
    public let max: Double
    
    public init(min: Double, max: Double) {
        precondition(min <= max, "Joint limit minimum must be less than or equal to maximum")
        self.min = min
        self.max = max
    }
    
    public static let unlimited = JointLimits(min: -.infinity, max: .infinity)
    
    public func clamp(_ value: Double) -> Double {
        Swift.max(min, Swift.min(max, value))
    }
    
    public func contains(_ value: Double) -> Bool {
        value >= min && value <= max
    }
}

public struct Joint: Sendable, Codable {
    public let id: String
    public let type: JointType
    public let axis: Vector3D
    public let limits: JointLimits
    public var value: Double
    public let parentTransform: Transform
    
    public init(
        id: String,
        type: JointType,
        axis: Vector3D = .unitZ,
        limits: JointLimits = .unlimited,
        value: Double = 0.0,
        parentTransform: Transform = .identity
    ) {
        self.id = id
        self.type = type
        self.axis = axis.normalized
        self.limits = limits
        self.value = limits.clamp(value)
        self.parentTransform = parentTransform
    }
    
    public var transform: Transform {
        switch type {
        case .revolute:
            let rotation = Quaternion(axis: axis, angle: value)
            return Transform(position: Vector3D.zero, rotation: rotation)
            
        case .prismatic:
            let translation = axis * value
            return Transform(position: translation, rotation: Quaternion.identity)
            
        case .spherical:
            return Transform.identity
            
        case .planar:
            return Transform.identity
            
        case .cylindrical:
            let rotation = Quaternion(axis: axis, angle: value)
            return Transform(position: Vector3D.zero, rotation: rotation)
            
        case .fixed:
            return Transform.identity
        }
    }
    
    public mutating func setValue(_ newValue: Double) {
        value = limits.clamp(newValue)
    }
    
    public func withValue(_ newValue: Double) -> Joint {
        var joint = self
        joint.setValue(newValue)
        return joint
    }
}

extension Joint: Equatable {
    public static func == (lhs: Joint, rhs: Joint) -> Bool {
        lhs.id == rhs.id &&
        lhs.type == rhs.type &&
        lhs.axis == rhs.axis &&
        lhs.limits.min == rhs.limits.min &&
        lhs.limits.max == rhs.limits.max &&
        abs(lhs.value - rhs.value) < Double.ulpOfOne &&
        lhs.parentTransform == rhs.parentTransform
    }
}

extension Joint: CustomStringConvertible {
    public var description: String {
        "Joint(id: \"\(id)\", type: \(type), value: \(value), limits: [\(limits.min), \(limits.max)])"
    }
}

extension Joint: Identifiable {}