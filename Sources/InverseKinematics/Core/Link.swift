import Foundation

public struct Link: Sendable, Codable {
    public let id: String
    public let length: Double
    public let offset: Vector3D
    public let localTransform: Transform
    public let mass: Double?
    public let centerOfMass: Vector3D?
    
    public init(
        id: String,
        length: Double,
        offset: Vector3D = .zero,
        localTransform: Transform = .identity,
        mass: Double? = nil,
        centerOfMass: Vector3D? = nil
    ) {
        self.id = id
        self.length = max(0, length)
        self.offset = offset
        self.localTransform = localTransform
        self.mass = mass
        self.centerOfMass = centerOfMass
    }
    
    public var endTransform: Transform {
        let translation = Vector3D(x: length, y: 0, z: 0) + offset
        let endPosition = localTransform.transformPoint(translation)
        return Transform(position: endPosition, rotation: localTransform.rotation)
    }
    
    public func transformAt(ratio: Double) -> Transform {
        let clampedRatio = max(0, min(1, ratio))
        let translation = Vector3D(x: length * clampedRatio, y: 0, z: 0) + offset
        let position = localTransform.transformPoint(translation)
        return Transform(position: position, rotation: localTransform.rotation)
    }
}

extension Link: Equatable {
    public static func == (lhs: Link, rhs: Link) -> Bool {
        lhs.id == rhs.id &&
        abs(lhs.length - rhs.length) < Double.ulpOfOne &&
        lhs.offset == rhs.offset &&
        lhs.localTransform == rhs.localTransform &&
        lhs.mass == rhs.mass &&
        lhs.centerOfMass == rhs.centerOfMass
    }
}

extension Link: CustomStringConvertible {
    public var description: String {
        "Link(id: \"\(id)\", length: \(length), offset: \(offset))"
    }
}

extension Link: Identifiable {}