find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

set(OSIM_SIMULATION_TEST_LIBS
        osimCommon osimSimulation osimAnalyses osimActuators osimLepton)

OpenSimAddTest(NAME testAssemblySolver
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS}
        RESOURCES resources/PushUpToesOnGroundExactConstraints.osim
                  resources/PushUpToesOnGroundLessPreciseConstraints.osim
                  resources/PushUpToesOnGroundWithMuscles.osim
                  resources/knee_patella_ligament.osim
        )

OpenSimAddTest(NAME testContactGeometry
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS}
        RESOURCES resources/sphere_10cm_radius.obj
                  resources/sphere_10cm_radius.stl
                  resources/sphere_10cm_radius.vtp
        )

OpenSimAddTest(NAME testExponentialContact
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS})

OpenSimAddTest(NAME testForceProducer
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS})

OpenSimAddTest(NAME testForces
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS}
        RESOURCES resources/BouncingBallModelEF.osim
                  resources/BouncingBall_HuntCrossley.osim
                  resources/BouncingBall_SmoothSphereHalfSpace.osim
                  resources/BushingForceModel_30000.osim
                  resources/BushingForceOffsetModel_30000.osim
                  resources/PushUpToesOnGroundWithMuscles.osim
                  resources/bouncing_block_30000.osim
                  resources/external_force_data.sto
                  resources/sphere.obj
        )

OpenSimAddTest(NAME testFrames
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS}
        RESOURCES resources/double_pendulum.osim
        )

OpenSimAddTest(NAME testFunctionBasedPath
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS})

OpenSimAddTest(NAME testInitState
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS}
        RESOURCES resources/arm26.osim
                  resources/arm26_StaticOptimization_controls.xml
        )

OpenSimAddTest(NAME testInverseKinematicsSolver
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS})

OpenSimAddTest(NAME testManager
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS}
        RESOURCES resources/arm26.osim
        )

OpenSimAddTest(NAME testModelInterface
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS}
        RESOURCES resources/arm26.osim
        )

OpenSimAddTest(NAME testMomentArms
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS}
        RESOURCES resources/BothLegs22.osim
                  resources/CoupledCoordinatesMPPsMomentArmTest.osim
                  resources/MovingPathPointMomentArmTest.osim
                  resources/MovingPointCustomJointMomentArmTest.osim
                  resources/MultipleMPPsMomentArmTest.osim
                  resources/P2PBallCustomJointMomentArmTest.osim
                  resources/P2PBallJointMomentArmTest.osim
                  resources/P2PCustomJointMomentArmTest.osim
                  resources/PathOnConstrainedBodyMomentArmTest.osim
                  resources/WrapPathCustomJointMomentArmTest.osim
                  resources/gait2354_simbody.osim
                  resources/testMomentArmsConstraintA.osim
                  resources/testMomentArmsConstraintB.osim
                  resources/wrist_mass.osim
        )

OpenSimAddTest(NAME testMuscleMetabolicsProbes
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS})

OpenSimAddTest(NAME testNestedModelComponents
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS}
        RESOURCES resources/double_pendulum.osim
        )

OpenSimAddTest(NAME testPointToPointSpring
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS})

OpenSimAddTest(NAME testPoints
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS}
        RESOURCES resources/double_pendulum.osim
        )

OpenSimAddTest(NAME testPrescribedForce
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS})

OpenSimAddTest(NAME testProbes
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS})

OpenSimAddTest(NAME testReportersWithModel
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS})

OpenSimAddTest(NAME testSimulationUtilities
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS}
        RESOURCES resources/testSimulationUtilities_leg69_IK_stance_pre4.mot
                  resources/testSimulationUtilities_leg6dof9musc_20303.osim
        )

OpenSimAddTest(NAME testStatesDocument
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS})

OpenSimAddTest(NAME testStatesTrajectory
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS}
        RESOURCES resources/arm26.osim
                  resources/gait2354_simbody.osim
        )

OpenSimAddTest(NAME testStationDefinedFrame
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS}
        RESOURCES resources/StationDefinedFrames_Advanced.osim
                  resources/StationDefinedFrames_Basic.osim
        )

OpenSimAddTest(NAME testTableProcessor
        LINKLIBS ${OSIM_SIMULATION_TEST_LIBS})

OpenSimCopySharedTestFiles(std_subject01_walk1_states.sto)
