
file(GLOB TEST_PROGS "*est*.cpp")
file(GLOB TEST_FILES *.osim *.xml *.sto *.mot *.trc)
file(GLOB TRC_TEST_FILES "${CMAKE_SOURCE_DIR}/Applications/*/test/*.trc")
file(GLOB MOT_TEST_FILES "${CMAKE_SOURCE_DIR}/Applications/*/test/*.mot")
file(GLOB C3D_TEST_FILES "${CMAKE_SOURCE_DIR}/OpenSim/Tests/shared/*.c3d")
file(GLOB XSENS_TEST_FILES "${CMAKE_SOURCE_DIR}/OpenSim/Tests/shared/IMUData/Xsens/*.txt")
file(GLOB APDM_TEST_FILES "${CMAKE_SOURCE_DIR}/OpenSim/Tests/shared/IMUData/APDM/*.csv")

list(APPEND TRC_TEST_FILES
    "${OPENSIM_SHARED_TEST_FILES_DIR}/gait10dof18musc_walk_CRLF_line_ending.trc")
list(APPEND MOT_TEST_FILES
    "${OPENSIM_SHARED_TEST_FILES_DIR}/gait10dof18musc_ik_CRLF_line_ending.mot")

if(NOT WITH_EZC3D)
    file(GLOB C3D_TESTPROG *testC3DFileAdapter.cpp)
    list(REMOVE_ITEM TEST_PROGS ${C3D_TESTPROG})
endif()

OpenSimAddTests(
    TESTPROGRAMS ${TEST_PROGS}
    DATAFILES ${TEST_FILES} ${C3D_TEST_FILES} ${TRC_TEST_FILES} ${XSENS_TEST_FILES} ${APDM_TEST_FILES}
              ${MOT_TEST_FILES}
    LINKLIBS osimCommon ${SIMTK_ALL_LIBS} 
    )
