# Inverse Kinematics Swift Library

![Swift 6.1+](https://img.shields.io/badge/Swift-6.1+-orange.svg)
![Platforms](https://img.shields.io/badge/Platforms-iOS%2013+%20|%20macOS%2013+%20|%20tvOS%2013+%20|%20watchOS%206+%20|%20visionOS%201+%20|%20Linux-blue.svg)
![CI Status](https://github.com/edgeengineer/inverse-kinematics/workflows/Swift%20CI/badge.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)

A comprehensive, cross-platform Swift library for robotics inverse kinematics with modern Swift 6.1+ concurrency support.

## Features

- 🤖 **Multiple IK Algorithms**: Analytical, Jacobian-based, CCD, FABRIK
- 🚀 **Swift 6.1+ Ready**: Modern async/await with actor-based concurrency
- 📱 **Cross-Platform**: iOS, macOS, tvOS, watchOS, visionOS, Linux (Swift 6.1+)
- 🧮 **Comprehensive Math**: Vector3D, Quaternion, Transform, Matrix4x4
- 🔧 **Flexible Design**: Protocol-based architecture for extensibility
- ✅ **Well Tested**: 55+ comprehensive unit tests
- 📚 **Type Safe**: Leverages Swift's advanced type system
- ⚡ **Optimized**: Uses FoundationEssentials when available for better performance

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

## Performance Optimization Guide

This library provides **dual APIs** for performance optimization: standard implementations for compatibility and SIMD-optimized versions for performance-critical applications.

### Standard vs. SIMD APIs

#### Standard API (Default)
```swift
// Standard vector operations - maximum compatibility
let result = vector1.dot(vector2)
let cross = vector1.cross(vector2)
let normalized = vector.normalized

// Standard quaternion operations
let rotated = quaternion.rotate(vector)
let combined = quaternion1 * quaternion2
```

#### SIMD-Optimized API
```swift
// SIMD operations for performance-critical code
let result = vector1.optimizedDot(vector2, config: .highPerformance)
let cross = vector1.optimizedCross(vector2, config: .highPerformance)
let normalized = vector.optimizedNormalized(config: .highPerformance)

// SIMD quaternion operations
let rotated = quaternion.optimizedRotate(vector, config: .highPerformance)
let combined = quaternion1.optimizedMultiply(quaternion2, config: .highPerformance)
```

### Performance Configurations

Choose the appropriate performance profile for your use case:

```swift
// High-performance: Aggressive SIMD usage, parallel processing
let config = PerformanceConfig.highPerformance

// Balanced: Selective SIMD usage, conservative thresholds (default)
let config = PerformanceConfig.balanced  

// Precision: No SIMD, maximum numerical accuracy
let config = PerformanceConfig.precision
```

### Batch Operations

For large-scale operations, use the optimized batch functions:

```swift
// Batch dot products with automatic optimization
let dots = OptimizedMath.batchDotProducts(
    vectors1: largeVectorArray1,
    vectors2: largeVectorArray2,
    config: .highPerformance
)

// Batch transformations
let transformed = OptimizedMath.batchTransformPoints(
    points: pointCloud,
    transform: robotTransform,
    config: .highPerformance
)

// Batch quaternion interpolation for animations
let interpolated = OptimizedMath.batchQuaternionSlerp(
    quaternions1: startPoses,
    quaternions2: endPoses,
    t: 0.5,
    config: .highPerformance
)
```

### Performance Characteristics

Based on comprehensive testing with 1000 iterations:

| Operation | Standard Time | SIMD Time | Speedup | Recommendation |
|-----------|---------------|-----------|---------|----------------|
| **Vector Dot Product** | 0.116ms | 0.098ms | **1.18x** | ✅ Use SIMD for batch operations |
| **Quaternion Multiplication** | 0.135ms | 0.133ms | **1.01x** | ⚖️ Marginal benefit |
| **Vector Addition** | 0.109ms | 0.130ms | **0.84x** | ❌ Standard is faster |
| **Vector Cross Product** | 0.100ms | 0.115ms | **0.87x** | ❌ Standard is faster |
| **Quaternion Rotation** | 0.129ms | 0.126ms | **1.02x** | ⚖️ Marginal benefit |

### When to Use SIMD Optimizations

#### ✅ **Recommended for:**
- Batch processing large point clouds (>100 points)
- Real-time animation with many quaternion interpolations  
- High-frequency forward kinematics calculations
- Applications where every microsecond counts
- Large-scale workspace analysis

#### ❌ **Not recommended for:**
- Single vector/quaternion operations
- Educational or prototyping code
- Memory-constrained embedded systems
- Applications prioritizing numerical precision over speed
- Small-scale operations (<50 elements)

#### ⚖️ **Platform Considerations:**
- **Apple Silicon (M1/M2)**: SIMD optimizations most effective
- **Intel x86_64**: Moderate SIMD benefits
- **ARM (mobile)**: Variable performance depending on chip
- **Linux**: Performance varies by distribution and hardware

### Configuration Examples

```swift
// Real-time robotics application
let robotConfig = PerformanceConfig(
    useSIMDOptimizations: true,
    enableParallelProcessing: true,
    optimizationTolerance: 1e-8,
    optimizationThreshold: 50
)

// Scientific simulation requiring maximum precision
let scienceConfig = PerformanceConfig(
    useSIMDOptimizations: false,
    enableParallelProcessing: false,
    optimizationTolerance: 1e-15,
    optimizationThreshold: 1000
)

// General-purpose applications (default)
let generalConfig = PerformanceConfig.balanced
```

### Automatic Optimization

The library automatically chooses optimal implementations based on:
- **Problem size**: SIMD activates above configured thresholds
- **Platform capabilities**: Detected at runtime
- **User configuration**: Performance vs. precision trade-offs
- **Operation type**: Some operations benefit more from SIMD than others

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

- **Optimized math operations** with efficient algorithms and optional SIMD acceleration
- **FoundationEssentials support** for lightweight Foundation functionality when available
- **Actor-based concurrency** for thread safety
- **Value semantics** for predictable performance
- **Minimal allocations** in hot paths
- **Configurable performance** with automatic optimization selection

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