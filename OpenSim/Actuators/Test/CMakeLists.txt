#include_directories(${OpenSim_SOURCE_DIR} 
#           ${OpenSim_SOURCE_DIR}/Vendors)


link_libraries( debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
        debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
        debug osimActuators${CMAKE_DEBUG_POSTFIX} optimized osimActuators
        debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
        debug osimTools${CMAKE_DEBUG_POSTFIX} optimized osimTools
        ${Simbody_LIBRARIES})


file(GLOB TEST_PROGS "test*.cpp")

foreach(TEST_PROG ${TEST_PROGS})
    get_filename_component(TEST_ROOT ${TEST_PROG} NAME_WE)

    # Link with shared library
    add_executable(${TEST_ROOT} ${TEST_PROG})
    target_link_libraries(${TEST_ROOT} ${LINK_LIBRARIES})
    add_test(${TEST_ROOT} ${EXECUTABLE_OUTPUT_PATH}/${TEST_ROOT})
    set_target_properties(${TEST_ROOT} 
        PROPERTIES 
        PROJECT_LABEL "Tests - ${TEST_ROOT}")

endforeach(TEST_PROG ${TEST_PROGS})



#
# Testing
#

#IF (UNIX)
#  add_definitions(-fprofile-arcs -ftest-coverage)
#  link_libraries(gcov)
#ENDIF (UNIX)

#enable_testing()

#IF (EXECUTABLE_OUTPUT_PATH)
#  SET (TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
#ELSE (EXECUTABLE_OUTPUT_PATH)
#  SET (TEST_PATH .)
#ENDIF (EXECUTABLE_OUTPUT_PATH)

#add_test(testPrescribedForce ${TEST_PATH}/testPrescribedForce)
