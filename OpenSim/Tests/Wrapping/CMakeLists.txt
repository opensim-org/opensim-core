include_directories(${OpenSim_SOURCE_DIR}
            ${OpenSim_SOURCE_DIR}/Vendors)


link_libraries(
        debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
        debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
        debug osimActuators${CMAKE_DEBUG_POSTFIX} optimized osimActuators
        debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
        debug osimTools${CMAKE_DEBUG_POSTFIX} optimized osimTools
        ${Simbody_LIBRARIES})

add_executable(testWrapping testWrapping.cpp)

target_link_libraries(testWrapping ${LINK_LIBRARIES} )

file(GLOB TEST_FILES *.osim *.xml *.sto *.mot)

#
# Copy files to run dir
#
foreach(dataFile ${TEST_FILES})
 add_custom_command(
    TARGET testWrapping
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${dataFile}
    ${OpenSim_BINARY_DIR}/OpenSim/Tests/Wrapping)
endforeach(dataFile)


#
# Testing
#

if(EXECUTABLE_OUTPUT_PATH)
  set(TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
else(EXECUTABLE_OUTPUT_PATH)
  set(TEST_PATH .)
endif(EXECUTABLE_OUTPUT_PATH)

add_test(testWrapping ${TEST_PATH}/testWrapping)

set_target_properties(testWrapping PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Tests - testWrapping")
