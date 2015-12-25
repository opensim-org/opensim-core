
file(GLOB TEST_PROGS "*est*.cpp")
file(GLOB TEST_FILES *.osim *.xml *.sto *.mot *.trc)
file(GLOB TRC_TEST_FILES "${CMAKE_SOURCE_DIR}/Applications/*/test/*.trc")
file(GLOB MOT_TEST_FILES "${CMAKE_SOURCE_DIR}/Applications/*/test/*.mot")
file(GLOB C3D_TEST_FILES "${CMAKE_SOURCE_DIR}/OpenSim/Tests/shared/*.c3d")

OpenSimAddTests(
    TESTPROGRAMS ${TEST_PROGS}
    DATAFILES ${TEST_FILES} ${C3D_TEST_FILES} ${TRC_TEST_FILES} 
              ${MOT_TEST_FILES}
    LINKLIBS ${BTK_LIBRARIES} osimCommon ${SIMTK_ALL_LIBS} 
    )
