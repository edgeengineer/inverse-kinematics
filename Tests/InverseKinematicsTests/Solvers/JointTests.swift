import Foundation
import Testing
@testable import InverseKinematics

struct JointTests {
    // Test joint creation and hierarchy
    @Test func testJointHierarchy() {
        let root = Joint(type: .fixed)
        let child1 = Joint(type: .revolute(axis: .up))
        let child2 = Joint(type: .revolute(axis: .right))
        
        root.addChild(child1)
        child1.addChild(child2)
        
        #expect(root.children.count == 1)
        #expect(child1.parent === root)
        #expect(child1.children.count == 1)
        #expect(child2.parent === child1)
        #expect(child2.children.count == 0)
    }
    
    // Test joint world position calculation
    @Test func testJointWorldPosition() {
        let root = Joint(type: .fixed, localPosition: .zero)
        let child = Joint(type: .fixed, localPosition: Vector3(x: 0, y: 1, z: 0))
        let grandchild = Joint(type: .fixed, localPosition: Vector3(x: 1, y: 0, z: 0))
        
        root.addChild(child)
        child.addChild(grandchild)
        
        #expect(root.worldPosition.x == 0)
        #expect(root.worldPosition.y == 0)
        #expect(root.worldPosition.z == 0)
        
        #expect(child.worldPosition.x == 0)
        #expect(child.worldPosition.y == 1)
        #expect(child.worldPosition.z == 0)
        
        #expect(grandchild.worldPosition.x == 1)
        #expect(grandchild.worldPosition.y == 1)
        #expect(grandchild.worldPosition.z == 0)
    }
    
    // Test joint world rotation calculation
    @Test func testJointWorldRotation() {
        let root = Joint(type: .fixed, localRotation: Quaternion.identity)
        let child = Joint(
            type: .fixed,
            localRotation: Quaternion(axis: Vector3.up, angle: Float.pi / 2)
        )
        
        root.addChild(child)
        
        // Child's world rotation should be the same as its local rotation
        // since the root has identity rotation
        #expect(abs(child.worldRotation.x - child.localRotation.x) < 1e-6)
        #expect(abs(child.worldRotation.y - child.localRotation.y) < 1e-6)
        #expect(abs(child.worldRotation.z - child.localRotation.z) < 1e-6)
        #expect(abs(child.worldRotation.w - child.localRotation.w) < 1e-6)
        
        // Change root rotation
        root.localRotation = Quaternion(axis: Vector3.right, angle: Float.pi / 2)
        
        // Child's world rotation should now be a combination of root and local rotation
        let expected = root.localRotation * child.localRotation
        #expect(abs(child.worldRotation.x - expected.x) < 1e-6)
        #expect(abs(child.worldRotation.y - expected.y) < 1e-6)
        #expect(abs(child.worldRotation.z - expected.z) < 1e-6)
        #expect(abs(child.worldRotation.w - expected.w) < 1e-6)
    }
    
    // Test joint constraints
    @Test func testJointConstraints() {
        // Test revolute joint angle constraints
        let revolute = Joint(
            type: .revolute(axis: .up),
            constraints: [JointConstraint(type: .angleLimit(min: -Float.pi / 4, max: Float.pi / 4))]
        )
        
        // Rotate beyond the limit
        revolute.localRotation = Quaternion(axis: Vector3.up, angle: Float.pi / 2)
        revolute.applyConstraints()
        
        // Should be clamped to max angle
        let angle = 2 * acos(abs(revolute.localRotation.w))
        #expect(abs(angle - Float.pi / 4) < 1e-6)
        
        // Test spherical joint cone constraint
        let spherical = Joint(
            type: .spherical,
            constraints: [JointConstraint(type: .coneLimit(axis: Vector3.forward, angle: Float.pi / 4))]
        )
        
        // Rotate beyond the cone
        spherical.localRotation = Quaternion(axis: Vector3.right, angle: Float.pi / 2)
        spherical.applyConstraints()
        
        // The angle between forward vector and rotated forward vector should be <= cone angle
        let forwardDir = Vector3.forward
        let rotatedDir = spherical.worldRotation.rotate(forwardDir)
        let angleBetween = acos(forwardDir.dot(rotatedDir).clamped(to: -1...1))
        
        #expect(angleBetween <= Float.pi / 4 + 1e-6)
    }
    
    // Test prismatic joint movement along its axis
    @Test func testPrismaticMovement() {
        let root = Joint(type: .fixed)
        let prismaticJoint = Joint(
            type: .prismatic(axis: .right), // Slides along X-axis
            localPosition: Vector3(x: 0, y: 0, z: 0),
            length: 0 // Length is often visual, position defines attachment
        )
        let endEffector = Joint(
            type: .fixed,
            localPosition: Vector3(x: 0, y: 0, z: 0) // Attach directly to prismatic joint origin
        )
        
        root.addChild(prismaticJoint)
        prismaticJoint.addChild(endEffector)
        
        // Initial state: prismatic joint at origin, end effector at origin relative to it.
        // World position of end effector should be (0, 0, 0)
        #expect(endEffector.worldPosition == .zero)

        // Apply a positive displacement (slide outwards)
        let displacement1: Float = 2.0
        prismaticJoint.localPosition += Vector3.right * displacement1
        // Expected end effector position: (0,0,0) + slide (2,0,0) = (2,0,0)
        #expect(abs(endEffector.worldPosition.x - displacement1) < 1e-6)
        #expect(abs(endEffector.worldPosition.y) < 1e-6)
        #expect(abs(endEffector.worldPosition.z) < 1e-6)

        // Apply a negative displacement (slide inwards)
        let displacement2: Float = -4.0
        prismaticJoint.localPosition += Vector3.right * displacement2 // Move back 4 units from current (2,0,0)
        // Expected end effector position: (2,0,0) + slide (-4,0,0) = (-2,0,0)
        let expectedX = displacement1 + displacement2
        #expect(abs(endEffector.worldPosition.x - expectedX) < 1e-6)
        #expect(abs(endEffector.worldPosition.y) < 1e-6)
        #expect(abs(endEffector.worldPosition.z) < 1e-6)
    }
    
    // Test a chain mixing revolute and prismatic joints (Corrected)
    @Test func testMixedPrismaticRevoluteChainCorrected() {
        let root = Joint(type: .fixed)
        let revoluteJoint = Joint(
            type: .revolute(axis: .up), // Rotates around Y
            localPosition: .zero,
            length: 1 // Defines position of next joint if relative position is zero
        )
        let prismaticJoint = Joint(
            type: .prismatic(axis: .forward), // Slides along its local Z-axis
            localPosition: Vector3(x: 0, y: 0, z: 1), // Position relative to revolute joint end
            length: 0 // Length doesn't determine position here
        )
        let endEffector = Joint(
            type: .fixed,
            localPosition: .zero // Attach to prismatic joint's origin
        )
        
        root.addChild(revoluteJoint)
        revoluteJoint.addChild(prismaticJoint)
        prismaticJoint.addChild(endEffector)
        
        // Initial state: 
        // Revolute at origin, rotates around Y.
        // Prismatic at (0,0,1) relative to revolute, slides along its Z (initially world Z).
        // End Effector at prismatic origin.
        // World Position = root(0,0,0) + revolute_offset(0,0,0) + revolute_rot(identity) * prismatic_offset(0,0,1) = (0,0,1)
        #expect(abs(endEffector.worldPosition.x - 0.0) < 1e-6)
        #expect(abs(endEffector.worldPosition.y - 0.0) < 1e-6)
        #expect(abs(endEffector.worldPosition.z - 1.0) < 1e-6)
        
        // Rotate revolute joint 90 degrees around Y
        revoluteJoint.localRotation = Quaternion(axis: .up, angle: Float.pi / 2)
        // World Position = root(0,0,0) + revolute_offset(0,0,0) + revolute_rot(Y,90) * prismatic_offset(0,0,1) = (1,0,0)
        #expect(abs(endEffector.worldPosition.x - 1.0) < 1e-6)
        #expect(abs(endEffector.worldPosition.y - 0.0) < 1e-6)
        #expect(abs(endEffector.worldPosition.z - 0.0) < 1e-6)
        
        // Slide prismatic joint outwards by 1 unit along its local forward axis
        let slideDistance: Float = 1.0
        // Apply slide displacement along the local forward axis
        prismaticJoint.localPosition += Vector3.forward * slideDistance
        // Previous localPosition was (0,0,1), new is (0,0,2)
        // Expected World Position = revolute_rotation * new_localPosition = Quat(Y, 90) * (0,0,2) = (2,0,0)
        #expect(abs(endEffector.worldPosition.x - 2.0) < 1e-6)
        #expect(abs(endEffector.worldPosition.y - 0.0) < 1e-6)
        #expect(abs(endEffector.worldPosition.z - 0.0) < 1e-6)
    }
    
    // Test prismatic joint distance constraints
    @Test func testPrismaticConstraints() {
        let root = Joint(type: .fixed)
        let slideAxis = Vector3.up
        let minLimit: Float = 0.5
        let maxLimit: Float = 2.0
        
        // Define the constraint
        let distanceConstraint = JointConstraint(type: .distanceLimit(min: minLimit, max: maxLimit))
        
        let prismaticJoint = Joint(
            type: .prismatic(axis: slideAxis),
            localPosition: slideAxis * 1.0, // Initial slide position (within limits)
            constraints: [distanceConstraint] // Add constraint
        )
        let endEffector = Joint(type: .fixed, localPosition: .zero)
        
        root.addChild(prismaticJoint)
        prismaticJoint.addChild(endEffector)
        
        // --- Get the constraint for applying logic ---
        guard let constraint = prismaticJoint.constraints.first(where: { 
            if case .distanceLimit = $0.type { return true } else { return false }
        }) else {
            #expect(Bool(false), "Constraint not found") // Fail test if constraint is missing
            return
        }
        
        // --- Initial position --- 
        #expect(abs(endEffector.worldPosition.x - 0.0) < 1e-6)
        #expect(abs(endEffector.worldPosition.y - 1.0) < 1e-6)
        #expect(abs(endEffector.worldPosition.z - 0.0) < 1e-6)
        
        // --- Attempt to slide beyond max limit --- 
        var currentSlide = prismaticJoint.localPosition.y // Assuming slide is along Y
        var desiredSlide = currentSlide + 1.5 // Propose moving to 2.5
        var clampedSlide = constraint.apply(to: desiredSlide, jointType: .prismatic(axis: slideAxis)) // Clamp using constraint
        prismaticJoint.localPosition = slideAxis * clampedSlide // Apply clamped position
        // Expected: Clamped to maxLimit 2.0
        #expect(abs(endEffector.worldPosition.x - 0.0) < 1e-6)
        #expect(abs(endEffector.worldPosition.y - maxLimit) < 1e-6)
        #expect(abs(endEffector.worldPosition.z - 0.0) < 1e-6)
        
        // --- Attempt to slide below min limit ---
        currentSlide = prismaticJoint.localPosition.y // Current is now 2.0
        desiredSlide = currentSlide - 2.0 // Propose moving to 0.0
        clampedSlide = constraint.apply(to: desiredSlide, jointType: .prismatic(axis: slideAxis)) // Clamp using constraint
        prismaticJoint.localPosition = slideAxis * clampedSlide // Apply clamped position
        // Expected: Clamped to minLimit 0.5
        #expect(abs(endEffector.worldPosition.x - 0.0) < 1e-6)
        #expect(abs(endEffector.worldPosition.y - minLimit) < 1e-6)
        #expect(abs(endEffector.worldPosition.z - 0.0) < 1e-6)
        
        // --- Attempt to slide within limits ---
        currentSlide = prismaticJoint.localPosition.y // Current is now 0.5
        desiredSlide = currentSlide + 0.5 // Propose moving to 1.0
        clampedSlide = constraint.apply(to: desiredSlide, jointType: .prismatic(axis: slideAxis)) // Apply constraint (shouldn't clamp)
        prismaticJoint.localPosition = slideAxis * clampedSlide // Apply clamped position
        // Expected: Moved to 1.0
        #expect(abs(endEffector.worldPosition.x - 0.0) < 1e-6)
        #expect(abs(endEffector.worldPosition.y - 1.0) < 1e-6)
        #expect(abs(endEffector.worldPosition.z - 0.0) < 1e-6)
    }
    
    // Test spherical joint cone constraints
    @Test func testSphericalConeConstraint() {
        let root = Joint(type: .fixed)
        let constraintAxis = Vector3.forward // Cone around Z-axis
        let maxAngle = Float.pi / 6 // 30 degrees
        
        // Define the constraint
        let coneConstraint = JointConstraint(type: .coneLimit(axis: constraintAxis, angle: maxAngle))
        
        let sphericalJoint = Joint(
            type: .spherical,
            localPosition: .zero,
            constraints: [coneConstraint]
        )
        root.addChild(sphericalJoint)
        
        // --- Get the constraint for applying logic ---
        guard let constraint = sphericalJoint.constraints.first(where: { 
            if case .coneLimit = $0.type { return true } else { return false }
        }) else {
            #expect(Bool(false), "Constraint not found")
            return
        }
        
        // --- Attempt rotation outside the cone (e.g., 45 degrees around Y-axis) ---
        let outsideRotation = Quaternion(axis: .up, angle: Float.pi / 4) // 45 degrees
        let clampedRotation1 = constraint.apply(to: outsideRotation, originalRotation: sphericalJoint.localRotation)
        sphericalJoint.localRotation = clampedRotation1
        
        // Verify the angle between the joint's new forward and the constraint axis is clamped to maxAngle
        let forwardAfterClamp1 = sphericalJoint.forwardDirection // This uses world rotation
        let angleAfterClamp1 = acos(forwardAfterClamp1.dot(constraintAxis).clamped(to: -1...1))
        #expect(abs(angleAfterClamp1 - maxAngle) < 1e-5)
        
        // --- Reset rotation --- 
        sphericalJoint.localRotation = .identity
        
        // --- Attempt rotation inside the cone (e.g., 15 degrees around X-axis) ---
        let insideRotation = Quaternion(axis: .right, angle: Float.pi / 12) // 15 degrees
        let originalRotationBeforeInside = sphericalJoint.localRotation
        let clampedRotation2 = constraint.apply(to: insideRotation, originalRotation: originalRotationBeforeInside)
        sphericalJoint.localRotation = clampedRotation2
        
        // Verify the rotation was not clamped (or is very close to the original target rotation)
        // We check if the angle between the new forward and constraint axis is close to the intended angle
        let forwardAfterClamp2 = sphericalJoint.forwardDirection
        let angleAfterClamp2 = acos(forwardAfterClamp2.dot(constraintAxis).clamped(to: -1...1))
        // The angle between (1,0,0) rotated 15deg around X (axis .right) and (0,0,1) (constraintAxis .forward) should still be 15 deg.
        #expect(abs(angleAfterClamp2 - (Float.pi / 12)) < 1e-5)
        // Also check the final rotation quaternion is close to the intended rotation
        #expect(Helpers.quaternionsApproximatelyEqual(sphericalJoint.localRotation, insideRotation))
    }
}

struct Helpers {
    static func quaternionsApproximatelyEqual(_ q1: Quaternion, _ q2: Quaternion, tolerance: Float = 1e-5) -> Bool {
        // Check if dot product is close to 1 (or -1, as q and -q represent the same rotation)
        let dot = abs(q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w)
        return abs(dot - 1.0) < tolerance
    }
}