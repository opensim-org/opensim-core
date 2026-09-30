find_package(Catch2 REQUIRED
        HINTS "${OPENSIM_DEPENDENCIES_DIR}/catch2")

set(OSIM_COMMON_TEST_LIBS osimCommon ${SIMTK_ALL_LIBS})
set(OSIM_SHARED_DIR "${CMAKE_SOURCE_DIR}/OpenSim/Tests/shared")
set(OSIM_APPS_DIR "${CMAKE_SOURCE_DIR}/Applications")

OpenSimAddTest(NAME testAPDMDataReader
        LINKLIBS ${OSIM_COMMON_TEST_LIBS}
        RESOURCES "${OSIM_SHARED_DIR}/IMUData/APDM/apdm_format7.csv"
                  "${OSIM_SHARED_DIR}/IMUData/APDM/imuData01.csv"
        )

OpenSimAddTest(NAME testArray
        LINKLIBS ${OSIM_COMMON_TEST_LIBS})

OpenSimAddTest(NAME testAssertion
        LINKLIBS ${OSIM_COMMON_TEST_LIBS})

if(WITH_EZC3D)
    OpenSimAddTest(NAME testC3DFileAdapter
            LINKLIBS ${OSIM_COMMON_TEST_LIBS}
            RESOURCES "${OSIM_SHARED_DIR}/walking2.c3d"
                      "${OSIM_SHARED_DIR}/walking5.c3d"
                      resources/std_cop_walking2_grfs.sto
                      resources/std_cop_walking5_grfs.sto
                      resources/std_walking2_analog.sto
                      resources/std_walking2_grfs.sto
                      resources/std_walking2_markers.trc
                      resources/std_walking5_analog.sto
                      resources/std_walking5_grfs.sto
                      resources/std_walking5_markers.trc
            )
endif()

OpenSimAddTest(NAME testCommonUtilities
        LINKLIBS ${OSIM_COMMON_TEST_LIBS})

OpenSimAddTest(NAME testComponentInterface
        LINKLIBS ${OSIM_COMMON_TEST_LIBS}
        RESOURCES resources/dataWithEformat.trc
        )

OpenSimAddTest(NAME testDataTable
        LINKLIBS ${OSIM_COMMON_TEST_LIBS}
        RESOURCES resources/test.sto
        )

OpenSimAddTest(NAME testFunctionAdapter
        LINKLIBS ${OSIM_COMMON_TEST_LIBS})

OpenSimAddTest(NAME testFunctions
        LINKLIBS ${OSIM_COMMON_TEST_LIBS})

OpenSimAddTest(NAME testGCVSpline
        LINKLIBS ${OSIM_COMMON_TEST_LIBS})

OpenSimAddTest(NAME testLatinHypercubeDesign
        LINKLIBS ${OSIM_COMMON_TEST_LIBS})

OpenSimAddTest(NAME testMarkerData
        LINKLIBS ${OSIM_COMMON_TEST_LIBS}
        RESOURCES resources/dataWithEformat.trc
                  resources/dataWithNaNsOfDifferentCases.trc
                  resources/dataWithNaNsWithSpaces.trc
        )

OpenSimAddTest(NAME testNCSpline
        LINKLIBS ${OSIM_COMMON_TEST_LIBS})

OpenSimAddTest(NAME testPath
        LINKLIBS ${OSIM_COMMON_TEST_LIBS})

OpenSimAddTest(NAME testRootSolver
        LINKLIBS ${OSIM_COMMON_TEST_LIBS})

OpenSimAddTest(NAME testSTOFileAdapter
        LINKLIBS ${OSIM_COMMON_TEST_LIBS}
        RESOURCES "${OSIM_APPS_DIR}/Analyze/tests/resources/subject01_walk1_grf.mot"
                  "${OSIM_APPS_DIR}/Analyze/tests/resources/subject02_running_arms_ik.mot"
                  "${OSIM_APPS_DIR}/CMC/tests/resources/gait10dof18musc_subject01_walk_grf.mot"
                  "${OSIM_APPS_DIR}/CMC/tests/resources/runningModel_GRF_data.mot"
                  "${OSIM_APPS_DIR}/IK/tests/resources/std_subject01_walk1_ik.mot"
                  "${OSIM_SHARED_DIR}/gait10dof18musc_ik_CRLF_line_ending.mot"
                  resources/test.sto
        )

OpenSimAddTest(NAME testScopeExit
        LINKLIBS ${OSIM_COMMON_TEST_LIBS})

OpenSimAddTest(NAME testSerialization
        LINKLIBS ${OSIM_COMMON_TEST_LIBS}
        RESOURCES resources/obj1Bad.xml
                  resources/obj1Defaults.xml
        )

OpenSimAddTest(NAME testSmoothSegmentedFunctionFactory
        LINKLIBS ${OSIM_COMMON_TEST_LIBS})

OpenSimAddTest(NAME testStorage
        LINKLIBS ${OSIM_COMMON_TEST_LIBS}
        RESOURCES resources/dataWithNaNsOfDifferentCases.trc
                  resources/sampleOutputs.sto
                  resources/sampleOutputsSpatialVec.sto
                  resources/sampleOutputsVec3.sto
                  resources/test.sto
                  resources/testDiff.sto
        )

OpenSimAddTest(NAME testTRCFileAdapter
        LINKLIBS ${OSIM_COMMON_TEST_LIBS}
        RESOURCES "${OSIM_APPS_DIR}/IK/tests/resources/constraintTest.trc"
                  "${OSIM_APPS_DIR}/IK/tests/resources/subject01_synthetic_marker_data.trc"
                  "${OSIM_APPS_DIR}/Scale/tests/resources/subject01_static.trc"
                  "${OSIM_SHARED_DIR}/gait10dof18musc_walk_CRLF_line_ending.trc"
                  resources/dataWithBlanksForMissingMarkers.trc
                  resources/dataWithEformat.trc
                  resources/dataWithNaNsOfDifferentCases.trc
                  resources/dataWithNaNsWithSpaces.trc
                  resources/exampleFormat.trc
        )

OpenSimAddTest(NAME testXsensDataReader
        LINKLIBS ${OSIM_COMMON_TEST_LIBS}
        RESOURCES "${OSIM_SHARED_DIR}/IMUData/Xsens/MT2022_ACC_00B42D4B.txt"
                  "${OSIM_SHARED_DIR}/IMUData/Xsens/MT2022_ACC_GYRO_ROT_ALL_TAB_NAN_00B42D4B.txt"
                  "${OSIM_SHARED_DIR}/IMUData/Xsens/MT2022_ACC_GYRO_ROT_EULER_TAB_00B42D4F.txt"
                  "${OSIM_SHARED_DIR}/IMUData/Xsens/MT2022_ACC_GYRO_ROT_MAT_COMMA_00B42D4B.txt"
                  "${OSIM_SHARED_DIR}/IMUData/Xsens/MT2022_ACC_GYRO_ROT_MAT_SEMICOLON_00B42D4B.txt"
                  "${OSIM_SHARED_DIR}/IMUData/Xsens/MT2022_ACC_GYRO_ROT_MAT_SPACE_00B42D4B.txt"
                  "${OSIM_SHARED_DIR}/IMUData/Xsens/MT46_01-000_00B42D4D.txt"
                  "${OSIM_SHARED_DIR}/IMUData/Xsens/MT46_01-000_00B42D4E.txt"
                  "${OSIM_SHARED_DIR}/IMUData/Xsens/MT_01200312-002-000_00B474BA.txt"
                  "${OSIM_SHARED_DIR}/IMUData/Xsens/MT_01200454_000-000_00B40DE4.txt"
                  "${OSIM_SHARED_DIR}/IMUData/Xsens/MT_012005D6-000_sit_to_stand-000_00B421ED.txt"
                  "${OSIM_SHARED_DIR}/IMUData/Xsens/MT_012005D6_031-000_00B421AF.txt"
                  "${OSIM_SHARED_DIR}/IMUData/Xsens/MT_012005D6_031-000_00B4227B.txt"
                  "${OSIM_SHARED_DIR}/IMUData/Xsens/test1_00B421E6.txt"
        )
