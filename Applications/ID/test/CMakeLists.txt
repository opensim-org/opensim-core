include_directories(${OpenSim_SOURCE_DIR}
            ${OpenSim_SOURCE_DIR}/Vendors)


link_libraries( debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
        debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
        debug osimActuators${CMAKE_DEBUG_POSTFIX} optimized osimActuators
        debug osimTools${CMAKE_DEBUG_POSTFIX} optimized osimTools
        debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
        ${Simbody_LIBRARIES}
        )

add_executable(testID testID.cpp)

file(GLOB TEST_FILES *.osim *.xml *.sto *.mot *.trc)

#
# Copy files to run dir
#
#add_dependencies(${TEST_ROOT} copyTestFiles)
foreach(dataFile ${TEST_FILES})
 add_custom_command(
    TARGET testID
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${dataFile}
    ${OpenSim_BINARY_DIR}/Applications/ID/test)
 endforeach(dataFile)


if(EXECUTABLE_OUTPUT_PATH)
  set(TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
else(EXECUTABLE_OUTPUT_PATH)
  set(TEST_PATH .)
endif(EXECUTABLE_OUTPUT_PATH)

add_test(testID ${TEST_PATH}/testID)

set_target_properties(testID PROPERTIES PROJECT_LABEL "Tests - testID")
