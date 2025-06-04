// swift-tools-version: 6.1
// The swift-tools-version declares the minimum version of Swift required to build this package.

import PackageDescription

let package = Package(
    name: "InverseKinematics",
    platforms: [
        .macOS(.v13),
        .iOS(.v13),
        .tvOS(.v13),
        .watchOS(.v6),
        .visionOS(.v1)
    ],
    products: [
        .library(
            name: "InverseKinematics",
            targets: ["InverseKinematics"]
        ),
    ],
    dependencies: [
        .package(url: "https://github.com/keyvariable/kvSIMD.swift.git", from: "1.1.0")
    ],
    targets: [
        .target(
            name: "InverseKinematics",
            dependencies: [
                .product(name: "kvSIMD", package: "kvSIMD.swift")
            ],
            swiftSettings: [
                .enableExperimentalFeature("StrictConcurrency")
            ]
        ),
        .testTarget(
            name: "InverseKinematicsTests",
            dependencies: ["InverseKinematics"],
            swiftSettings: [
                .enableExperimentalFeature("StrictConcurrency")
            ]
        ),
    ],
    swiftLanguageModes: [.v6]
)