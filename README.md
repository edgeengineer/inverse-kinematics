# Inverse Kinematics Swift Library

![Swift 6.1+](https://img.shields.io/badge/Swift-6.1+-orange.svg)
![Platforms](https://img.shields.io/badge/Platforms-iOS%2013+%20|%20macOS%2013+%20|%20tvOS%2013+%20|%20watchOS%206+%20|%20visionOS%201+-blue.svg)
![CI Status](https://github.com/edgeengineer/inverse-kinematics/workflows/Swift%20CI/badge.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)

A comprehensive, cross-platform Swift library for robotics inverse kinematics with modern Swift 6.1+ concurrency support.

## Features

- 🤖 **Multiple IK Algorithms**: Analytical, Jacobian-based, CCD, FABRIK
- 🚀 **Swift 6.1+ Ready**: Modern async/await with actor-based concurrency
- 📱 **Cross-Platform**: iOS, macOS, tvOS, watchOS, visionOS, Linux
- 🧮 **Comprehensive Math**: Vector3D, Quaternion, Transform, Matrix4x4
- 🔧 **Flexible Design**: Protocol-based architecture for extensibility
- ✅ **Well Tested**: 55+ comprehensive unit tests
- 📚 **Type Safe**: Leverages Swift's advanced type system

## Supported Algorithms

### Analytical Solvers
- **2-DOF Planar**: Standard 2-link planar arm configurations
- **3-DOF Planar**: 3-link planar arms with orientation control
- **Spherical Wrist**: 3-DOF spherical wrists (ZYZ, ZYX conventions)
- **6-DOF**: Combined arm + wrist analytical solutions

### Numerical Solvers
- **Jacobian Transpose**: Simple gradient-based approach
- **Damped Least Squares (DLS)**: Levenberg-Marquardt with damping
- **Selectively Damped Least Squares (SDLS)**: SVD-based with selective damping
- **Cyclic Coordinate Descent (CCD)**: Iterative joint optimization
- **FABRIK**: Forward And Backward Reaching Inverse Kinematics

## Installation

### Swift Package Manager

Add this to your `Package.swift`:

```swift
dependencies: [
    .package(url: "https://github.com/edgeengineer/inverse-kinematics.git", from: "0.0.2")
]
```

Or add it via Xcode:
1. File → Add Package Dependencies
2. Enter: `https://github.com/edgeengineer/inverse-kinematics`
3. Select version `0.0.2` or later

## Quick Start

### Basic Setup

```swift
import InverseKinematics

// Create a 2-link planar robot
var chain = KinematicChain(id: "robot_arm")

// Add joints
let shoulder = Joint(
    id: "shoulder",
    type: .revolute,
    axis: Vector3D.unitZ,
    limits: JointLimits(min: -Double.pi, max: Double.pi)
)

let elbow = Joint(
    id: "elbow", 
    type: .revolute,
    axis: Vector3D.unitZ,
    limits: JointLimits(min: -Double.pi/2, max: Double.pi/2),
    parentTransform: Transform(position: Vector3D(x: 1.0, y: 0.0, z: 0.0))
)

// Add links
let upperArm = Link(id: "upper_arm", length: 1.0)
let forearm = Link(id: "forearm", length: 0.8)

chain.addJoint(shoulder)
chain.addJoint(elbow)
chain.addLink(upperArm)
chain.addLink(forearm)
```

### Forward Kinematics

```swift
// Calculate end effector position
let jointValues = [0.5, -0.3] // radians
let endEffector = chain.endEffectorTransform(jointValues: jointValues)
print("End effector at: \(endEffector.position)")
```

### Inverse Kinematics

```swift
// Create an IK solver
let solver = JacobianBasedSolver(chain: chain)

// Define target pose
let target = Transform(
    position: Vector3D(x: 1.5, y: 0.5, z: 0.0),
    rotation: Quaternion.identity
)

// Solve IK
do {
    let solution = try await solver.solveIK(
        target: target,
        algorithm: .dampedLeastSquares,
        parameters: IKParameters(tolerance: 1e-6, maxIterations: 100)
    )
    
    if solution.success {
        print("Solution found: \(solution.jointValues)")
        print("Error: \(solution.error)")
        print("Iterations: \(solution.iterations)")
    }
} catch {
    print("IK failed: \(error)")
}
```

### Analytical Solvers

```swift
// Direct utility functions for specific configurations
let analyticalSolver = TwoDOFPlanarSolver(
    link1Length: 1.0,
    link2Length: 0.8
)

if let solution = analyticalSolver.solve(
    target: Vector3D(x: 1.5, y: 0.5, z: 0.0),
    elbowUp: true
) {
    print("Joint angles: \(solution.joint1), \(solution.joint2)")
}

// Or use protocol-conforming analytical solver with KinematicChain
let protocolSolver = AnalyticalSolver(chain: chain, type: .twoDOFPlanar)

do {
    let solution = try await protocolSolver.solveIK(
        target: target,
        algorithm: .analytical,
        parameters: .default
    )
    
    if solution.success {
        print("Analytical solution: \(solution.jointValues)")
    }
} catch {
    print("Analytical solve failed: \(error)")
}
```

### FABRIK Solver

```swift
// Fast iterative solver
let fabrikSolver = FABRIKSolver(chain: chain)

let solution = try await fabrikSolver.solveIK(
    target: target,
    algorithm: .fabrik,
    parameters: IKParameters(tolerance: 1e-3, maxIterations: 20)
)
```

## Algorithm Selection Guide

| Algorithm | Best For | Pros | Cons |
|-----------|----------|------|------|
| **Analytical** | Simple configurations | Exact, fast | Limited configurations |
| **Jacobian Transpose** | Quick approximations | Simple, stable | Slow convergence |
| **Damped Least Squares** | General purpose | Robust, handles singularities | Requires tuning |
| **CCD** | Real-time applications | Fast, intuitive | Local minima |
| **FABRIK** | Long kinematic chains | Fast, natural motion | Position-only |

## Requirements

- **Swift**: 6.1 or later
- **iOS**: 13.0+
- **macOS**: 13.0+
- **tvOS**: 13.0+
- **watchOS**: 6.0+
- **visionOS**: 1.0+
- **Linux**: Ubuntu 20.04+ (with Swift 6.1+)

## Architecture

```
InverseKinematics/
├── Math/                   # Core math types
│   ├── Vector3D.swift
│   ├── Quaternion.swift
│   ├── Transform.swift
│   └── Matrix4x4.swift
├── Core/                   # Robotics structures
│   ├── Joint.swift
│   ├── Link.swift
│   └── KinematicChain.swift
├── Protocols/              # IK protocols
│   └── KinematicsProtocols.swift
└── Solvers/               # IK algorithms
    ├── AnalyticalSolver.swift
    ├── JacobianSolver.swift
    ├── CCDSolver.swift
    └── FABRIKSolver.swift
```

## Performance

- **Optimized math operations** with efficient algorithms
- **Actor-based concurrency** for thread safety
- **Value semantics** for predictable performance
- **Minimal allocations** in hot paths

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

Please ensure all tests pass with `swift test`.

## Testing

Run the comprehensive test suite:

```bash
swift test
```

The library includes 55+ tests covering:
- Math type operations
- Forward kinematics validation
- IK solver correctness
- Edge cases and error conditions

## License

This project is licensed under the Apache 2.0 License 

## Acknowledgments

- Inspired by modern robotics research and industry best practices
- Built with Swift 6.1's advanced concurrency features
- Designed for both research and production applications