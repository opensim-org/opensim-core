
# Automatically create tests using files named "test*.cpp"  // Adopted from SimTK core
FILE(GLOB TEST_PROGS "test*.cpp")

LINK_LIBRARIES(debug osimSimulation${CMAKE_DEBUG_POSTFIX}   optimized osimSimulation
               debug osimActuators${CMAKE_DEBUG_POSTFIX}    optimized osimActuators
               debug osimCommon${CMAKE_DEBUG_POSTFIX}   optimized osimCommon
               debug osimAnalyses${CMAKE_DEBUG_POSTFIX}     optimized osimAnalyses
               debug osimTools${CMAKE_DEBUG_POSTFIX} optimized osimTools
               ${SIMTK_ALL_LIBS}
)

FOREACH(TEST_PROG ${TEST_PROGS})
    GET_FILENAME_COMPONENT(TEST_ROOT ${TEST_PROG} NAME_WE)

    # Link with shared library
    ADD_EXECUTABLE(${TEST_ROOT} ${TEST_PROG})
    TARGET_LINK_LIBRARIES(${TEST_ROOT} ${LINK_LIBRARIES} )
    ADD_TEST(${TEST_ROOT} ${EXECUTABLE_OUTPUT_PATH}/${TEST_ROOT})
    SET_TARGET_PROPERTIES(${TEST_ROOT} 
        PROPERTIES 
        PROJECT_LABEL "Tests - ${TEST_ROOT}")
    # MESSAGE( "Adding test "${TEST_PROG} )

ENDFOREACH(TEST_PROG ${TEST_PROGS})

# Also need to copy data files (.osim, .xml, .sto, .trc, .mot) to run directory
FILE(GLOB TEST_FILES *.osim *.xml *.sto *.mot)

#
# Copy files to run dir
#
FOREACH (dataFile ${TEST_FILES})
 ADD_CUSTOM_COMMAND(
    TARGET testControllers
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${dataFile}
    ${OpenSim_BINARY_DIR}/OpenSim/Tools/Test)
    # MESSAGE( "IN file "${dataFile} )
ENDFOREACH (dataFile) 
