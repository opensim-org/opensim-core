
link_libraries( 
        debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
        debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
        debug osimActuators${CMAKE_DEBUG_POSTFIX} optimized osimActuators
        debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
        debug osimTools${CMAKE_DEBUG_POSTFIX} optimized osimTools
        ${Simbody_LIBRARIES})

add_executable(testREADME testREADME.cpp)

target_link_libraries(testREADME ${LINK_LIBRARIES} )

file(GLOB TEST_FILES *.osim *.xml *.sto *.mot)

#
# Copy files to run dir
#
foreach(dataFile ${TEST_FILES})
    add_custom_command(
        TARGET testREADME
        COMMAND ${CMAKE_COMMAND}
        ARGS -E copy
        ${dataFile}
        ${OpenSim_BINARY_DIR}/OpenSim/Tests/README)
endforeach(dataFile)


#
# Testing
#

if(EXECUTABLE_OUTPUT_PATH)
  set(TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
else(EXECUTABLE_OUTPUT_PATH)
  set(TEST_PATH .)
endif(EXECUTABLE_OUTPUT_PATH)

add_test(testREADME ${TEST_PATH}/testREADME)

set_target_properties(testREADME PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Tests - testREADME")
