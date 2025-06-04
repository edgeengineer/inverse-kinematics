import Testing
#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif
@testable import InverseKinematics

@Suite("Core Types Tests")
struct CoreTests {
    
    @Suite("Joint Tests")
    struct JointTests {
        
        @Test("Joint initialization")
        func testJointInit() {
            let joint = Joint(
                id: "test_joint",
                type: .revolute,
                axis: Vector3D.unitZ,
                limits: JointLimits(min: -Double.pi, max: Double.pi),
                value: 0.0
            )
            
            #expect(joint.id == "test_joint")
            #expect(joint.type == .revolute)
            #expect(joint.axis == Vector3D.unitZ)
            #expect(joint.value == 0.0)
        }
        
        @Test("Joint limits clamping")
        func testJointLimitsClamping() {
            let limits = JointLimits(min: -1.0, max: 1.0)
            
            #expect(limits.clamp(-2.0) == -1.0)
            #expect(limits.clamp(0.5) == 0.5)
            #expect(limits.clamp(2.0) == 1.0)
            #expect(limits.contains(0.5))
            #expect(!limits.contains(2.0))
        }
        
        @Test("Joint value setting with limits")
        func testJointValueSetting() {
            var joint = Joint(
                id: "test",
                type: .revolute,
                limits: JointLimits(min: -1.0, max: 1.0),
                value: 0.0
            )
            
            joint.setValue(2.0)
            #expect(joint.value == 1.0)
            
            joint.setValue(-2.0)
            #expect(joint.value == -1.0)
        }
        
        @Test("Revolute joint transform")
        func testRevoluteJointTransform() {
            let joint = Joint(
                id: "revolute",
                type: .revolute,
                axis: Vector3D.unitZ,
                value: Double.pi / 2
            )
            
            let transform = joint.transform
            let testPoint = Vector3D.unitX
            let rotated = transform.rotation.rotate(testPoint)
            
            #expect(abs(rotated.x) < 1e-10)
            #expect(abs(rotated.y - 1.0) < 1e-10)
            #expect(abs(rotated.z) < 1e-10)
        }
        
        @Test("Prismatic joint transform")
        func testPrismaticJointTransform() {
            let joint = Joint(
                id: "prismatic",
                type: .prismatic,
                axis: Vector3D.unitX,
                value: 2.0
            )
            
            let transform = joint.transform
            #expect(abs(transform.position.x - 2.0) < 1e-10)
            #expect(abs(transform.position.y) < 1e-10)
            #expect(abs(transform.position.z) < 1e-10)
        }
    }
    
    @Suite("Link Tests")
    struct LinkTests {
        
        @Test("Link initialization")
        func testLinkInit() {
            let link = Link(
                id: "test_link",
                length: 1.5,
                offset: Vector3D(x: 0.1, y: 0.0, z: 0.0)
            )
            
            #expect(link.id == "test_link")
            #expect(link.length == 1.5)
            #expect(link.offset == Vector3D(x: 0.1, y: 0.0, z: 0.0))
        }
        
        @Test("Link end transform")
        func testLinkEndTransform() {
            let link = Link(
                id: "test",
                length: 2.0,
                offset: Vector3D.zero
            )
            
            let endTransform = link.endTransform
            #expect(abs(endTransform.position.x - 2.0) < 1e-10)
            #expect(abs(endTransform.position.y) < 1e-10)
            #expect(abs(endTransform.position.z) < 1e-10)
        }
        
        @Test("Link transform at ratio")
        func testLinkTransformAtRatio() {
            let link = Link(id: "test", length: 4.0)
            
            let halfwayTransform = link.transformAt(ratio: 0.5)
            #expect(abs(halfwayTransform.position.x - 2.0) < 1e-10)
            
            let endTransform = link.transformAt(ratio: 1.0)
            #expect(abs(endTransform.position.x - 4.0) < 1e-10)
            
            let clampedTransform = link.transformAt(ratio: 1.5)
            #expect(abs(clampedTransform.position.x - 4.0) < 1e-10)
        }
    }
    
    @Suite("KinematicChain Tests")
    struct KinematicChainTests {
        
        @Test("KinematicChain initialization")
        func testKinematicChainInit() {
            let chain = KinematicChain(id: "test_chain")
            
            #expect(chain.id == "test_chain")
            #expect(chain.jointCount == 0)
            #expect(chain.linkCount == 0)
            #expect(chain.jointValues.isEmpty)
        }
        
        @Test("KinematicChain joint and link management")
        func testKinematicChainManagement() {
            var chain = KinematicChain(id: "test")
            
            let joint = Joint(id: "j1", type: .revolute)
            let link = Link(id: "l1", length: 1.0)
            
            chain.addJoint(joint)
            chain.addLink(link)
            
            #expect(chain.jointCount == 1)
            #expect(chain.linkCount == 1)
            #expect(chain.getJoint(withId: "j1") != nil)
            #expect(chain.getLink(withId: "l1") != nil)
        }
        
        @Test("KinematicChain joint values setting")
        func testKinematicChainJointValues() {
            var chain = KinematicChain(id: "test")
            
            let joint1 = Joint(id: "j1", type: .revolute, value: 0.0)
            let joint2 = Joint(id: "j2", type: .revolute, value: 0.0)
            
            chain.addJoint(joint1)
            chain.addJoint(joint2)
            
            chain.setJointValues([1.0, 2.0])
            
            #expect(chain.jointValues == [1.0, 2.0])
        }
        
        @Test("Simple two-link forward kinematics")
        func testSimpleForwardKinematics() {
            var chain = KinematicChain(id: "two_link")
            
            let joint1 = Joint(
                id: "shoulder",
                type: .revolute,
                axis: Vector3D.unitZ,
                value: 0.0
            )
            let joint2 = Joint(
                id: "elbow",
                type: .revolute,
                axis: Vector3D.unitZ,
                value: 0.0,
                parentTransform: Transform(position: Vector3D(x: 1.0, y: 0.0, z: 0.0))
            )
            
            let link1 = Link(id: "upper_arm", length: 1.0)
            let link2 = Link(id: "forearm", length: 1.0)
            
            chain.addJoint(joint1)
            chain.addJoint(joint2)
            chain.addLink(link1)
            chain.addLink(link2)
            
            let endEffector = chain.endEffectorTransform(jointValues: [0.0, 0.0])
            #expect(abs(endEffector.position.x - 2.0) < 1e-10)
            #expect(abs(endEffector.position.y) < 1e-10)
            #expect(abs(endEffector.position.z) < 1e-10)
        }
        
        @Test("KinematicChain workspace sampling")
        func testWorkspaceSampling() {
            var chain = KinematicChain(id: "test")
            
            let joint = Joint(
                id: "j1",
                type: .revolute,
                limits: JointLimits(min: -Double.pi, max: Double.pi)
            )
            let link = Link(id: "l1", length: 1.0)
            
            chain.addJoint(joint)
            chain.addLink(link)
            
            let workspace = chain.workspace(samples: 100)
            #expect(workspace.count == 100)
            
            for point in workspace {
                #expect(point.magnitude <= 1.0 + 1e-10)
            }
        }
    }
}