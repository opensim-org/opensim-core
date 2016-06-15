
file(GLOB TEST_PROGS "*est*.cpp")
file(GLOB TESTDELAY *testDelay.cpp)
list(REMOVE_ITEM TEST_PROGS ${TESTDELAY})

file(GLOB TEST_FILES *.osim *.xml *.sto *.mot *.trc)
file(GLOB TRC_TEST_FILES "${CMAKE_SOURCE_DIR}/Applications/*/test/*.trc")
file(GLOB MOT_TEST_FILES "${CMAKE_SOURCE_DIR}/Applications/*/test/*.mot")
file(GLOB C3D_TEST_FILES "${CMAKE_SOURCE_DIR}/OpenSim/Tests/shared/*.c3d")

if(NOT WITH_BTK)
    file(GLOB C3D_TESTPROG *testC3DFileAdapter.cpp)
    list(REMOVE_ITEM TEST_PROGS ${C3D_TESTPROG})
    unset(BTK_LIBRARIES)
endif()

OpenSimAddTests(
    TESTPROGRAMS ${TEST_PROGS}
    DATAFILES ${TEST_FILES} ${C3D_TEST_FILES} ${TRC_TEST_FILES} 
              ${MOT_TEST_FILES}
    LINKLIBS ${BTK_LIBRARIES} osimCommon ${SIMTK_ALL_LIBS} 
    )

OpenSimAddTests(
    TESTPROGRAMS testDelay.cpp
    LINKLIBS osimTools
    )
