################################################################################
# Test DataTable class.
add_executable(testDataTable DataTable.cpp)
target_link_libraries(testDataTable ${Simbody_LIBRARIES} osimCommon)
add_test(testDataTable ${EXECUTABLE_OUTPUT_PATH}/testDataTable)
################################################################################

# Automatically create tests using files named "test*.cpp"  // Adopted from SimTK core
FILE(GLOB TEST_PROGS "*est*.cpp")

LINK_LIBRARIES(debug osimCommon${CMAKE_DEBUG_POSTFIX}   optimized osimCommon
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

# Also need to copy data files (.osim, .xml, .sto, .trc, .mot, .csv) to run directory
FILE(GLOB TEST_FILES *.osim *.xml *.sto *.mot *.trc *.csv)

#
# Copy files to run dir
#
FOREACH (dataFile ${TEST_FILES})
 ADD_CUSTOM_COMMAND(
    TARGET testSerialization
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${dataFile}
    ${OpenSim_BINARY_DIR}/OpenSim/Common/Test)
    # MESSAGE( "IN file "${dataFile} )
 ENDFOREACH (dataFile) 

