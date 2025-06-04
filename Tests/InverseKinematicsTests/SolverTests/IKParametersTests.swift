import Testing
#if canImport(FoundationEssentials)
import FoundationEssentials
#else
import Foundation
#endif
@testable import InverseKinematics

@Suite("IK Parameters Tests")
struct IKParametersTests {
    
    @Test("IK parameters initialization")
    func testIKParametersInit() {
        let params = IKParameters()
        #expect(params.tolerance == 1e-6)
        #expect(params.maxIterations == 100)
        #expect(params.dampingFactor == 0.01)
        #expect(params.stepSize == 0.1)
        #expect(params.positionWeight == 1.0)
        #expect(params.orientationWeight == 1.0)
    }
    
    @Test("IK parameters custom values")
    func testIKParametersCustom() {
        let params = IKParameters(
            tolerance: 1e-4,
            maxIterations: 50,
            dampingFactor: 0.1,
            stepSize: 0.2,
            positionWeight: 2.0,
            orientationWeight: 0.5
        )
        
        #expect(params.tolerance == 1e-4)
        #expect(params.maxIterations == 50)
        #expect(params.dampingFactor == 0.1)
        #expect(params.stepSize == 0.2)
        #expect(params.positionWeight == 2.0)
        #expect(params.orientationWeight == 0.5)
    }
    
    @Test("IK parameters validation")
    func testIKParametersValidation() {
        let params = IKParameters(
            tolerance: -1.0,
            maxIterations: 0,
            dampingFactor: -0.1,
            stepSize: -0.2,
            positionWeight: -1.0,
            orientationWeight: -0.5
        )
        
        #expect(params.tolerance == 0.0)
        #expect(params.maxIterations == 1)
        #expect(params.dampingFactor == 0.0)
        #expect(params.stepSize == 0.0)
        #expect(params.positionWeight == 0.0)
        #expect(params.orientationWeight == 0.0)
    }
}