# Swift Inverse Kinematics Library

A cross-platform Swift 6 library for inverse kinematics (IK) calculations, suitable for games, 3D applications, and robotics.

NOT READY FOR PUBLIC USE

## Features

- **Cross-Platform**: Works on Apple platforms (iOS, macOS, watchOS, tvOS), Linux, and Windows
- **Multiple IK Solvers**:
  - **CCD (Cyclic Coordinate Descent)**: Fast and efficient for simple chains
  - **FABRIK (Forward And Backward Reaching Inverse Kinematics)**: Natural-looking motion with fast convergence
  - **Jacobian**: Precise control suitable for robotics applications
- **Comprehensive Joint System**:
  - Revolute joints (rotation around an axis)
  - Prismatic joints (translation along an axis)
  - Spherical joints (rotation in any direction)
  - Fixed joints (no movement)
- **Constraint System**:
  - Angle limits for revolute joints
  - Distance limits for prismatic joints
  - Cone limits for spherical joints
- **Math Utilities**:
  - Vector3: 3D vector operations
  - Quaternion: Rotation representation
  - Matrix4x4: Transformation matrices

## Installation

### Swift Package Manager

Add the following to your `Package.swift` file:

```swift
dependencies: [
    .package(url: "https://github.com/apache-edge/inverse-kinematics.git", from: "0.0.1")
]
```

Now in your target add:

```swift
.target(name: "YourTarget", dependencies: ["InverseKinematics"])
```

## Usage

### Basic Example

```swift
import InverseKinematics

// Create a simple arm with 3 joints
let (chain, solver) = InverseKinematics.createSimpleArm(numJoints: 3, jointLength: 1.0, solverType: .ccd)

// Set a target position for the end effector
chain.setGoal(position: Vector3(x: 1, y: 1, z: 1))

// Solve the IK problem
let solved = solver.solve()

if solved {
    print("IK solved successfully!")
} else {
    print("Could not reach the target")
}

// Access the positions of all joints
let joints = chain.getJointPath()
for (index, joint) in joints.enumerated() {
    print("Joint \(index) position: \(joint.worldPosition)")
}
```

### Creating a Custom Chain

```swift
import InverseKinematics

// Create joints
let root = InverseKinematics.createFixedJoint()
let shoulder = InverseKinematics.createRevoluteJoint(
    axis: .up,
    position: Vector3(x: 0, y: 0.5, z: 0),
    minAngle: -.pi/2,
    maxAngle: .pi/2,
    length: 1.0
)
let elbow = InverseKinematics.createRevoluteJoint(
    axis: .right,
    position: Vector3(x: 0, y: 0, z: 1.0),
    minAngle: 0,
    maxAngle: .pi*0.75,
    length: 1.0
)
let wrist = InverseKinematics.createSphericalJoint(
    position: Vector3(x: 0, y: 0, z: 1.0),
    coneAxis: .forward,
    coneAngle: .pi/4,
    length: 0.5
)
let endEffector = InverseKinematics.createFixedJoint(
    position: Vector3(x: 0, y: 0, z: 0.5)
)

// Build hierarchy
root.addChild(shoulder)
shoulder.addChild(elbow)
elbow.addChild(wrist)
wrist.addChild(endEffector)

// Create chain and solver
let chain = InverseKinematics.createChain(
    rootJoint: root,
    endEffector: endEffector,
    maxIterations: 30,
    positionTolerance: 0.01,
    orientationTolerance: 0.05
)
let solver = InverseKinematics.createSolver(type: .fabrik, chain: chain)

// Set goal with both position and orientation
chain.setGoal(
    position: Vector3(x: 1, y: 1, z: 1),
    orientation: Quaternion(axis: Vector3.up, angle: .pi/4)
)

// Solve
solver.solve()
```

### Using Different Solvers

```swift
// CCD Solver (fast, good for simple chains)
let ccdSolver = InverseKinematics.createSolver(type: .ccd, chain: chain)

// FABRIK Solver (natural-looking results)
let fabrikSolver = InverseKinematics.createSolver(type: .fabrik, chain: chain)

// Jacobian Solver (precise, good for robotics)
let jacobianSolver = InverseKinematics.createSolver(type: .jacobian, chain: chain)
```

## Advanced Features

### Joint Constraints

```swift
// Create a revolute joint with angle constraints
let joint = InverseKinematics.createRevoluteJoint(
    axis: .up,
    minAngle: -.pi/4,  // -45 degrees
    maxAngle: .pi/4    // 45 degrees
)

// Create a prismatic joint with distance constraints
let slider = InverseKinematics.createPrismaticJoint(
    axis: .forward,
    minDistance: 0,
    maxDistance: 2.0
)

// Create a spherical joint with cone constraints
let ball = InverseKinematics.createSphericalJoint(
    coneAxis: .up,
    coneAngle: .pi/3   // 60 degrees
)
```

### Robotic Arm Example

```swift
// Create a robotic arm with specific joint configuration
let (robotArm, solver) = InverseKinematics.createRoboticArm(solverType: .jacobian)

// Set target
robotArm.setGoal(position: Vector3(x: 2, y: 1, z: 1))

// Solve with high precision
solver.solve()
```

## Performance Considerations

- CCD is the fastest solver but may produce less natural results
- FABRIK offers a good balance between speed and quality
- Jacobian is more computationally expensive but provides precise control
- For real-time applications, consider:
  - Limiting the maximum number of iterations
  - Using a larger position/orientation tolerance
  - Caching joint positions when possible

## License

This library is available under the Apache License 2.0. See the LICENSE file for more info.
