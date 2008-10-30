INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
		    ${OpenSim_SOURCE_DIR}/Vendors 		  
		    ${XERCES_INCLUDE_DIR})


LINK_LIBRARIES( debug osimCommon_d optimized osimCommon
		debug osimSimulation_d optimized osimSimulation
		debug osimAnalyses_d optimized osimAnalyses)

ADD_EXECUTABLE(testScale testScale.cpp)

FILE(GLOB TEST_FILES *.osim *.xml *.sto *.mot *.trc)

#
# Copy files to run dir
#
FOREACH (dataFile ${TEST_FILES})
 ADD_CUSTOM_COMMAND(
    TARGET testScale
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${dataFile}
    ${OpenSim_BINARY_DIR}/OpenSim/Applications/Scale/test)
 ENDFOREACH (dataFile) 


IF (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
ELSE (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH .)
ENDIF (EXECUTABLE_OUTPUT_PATH)

ADD_TEST(testScale ${TEST_PATH}/testScale)

SET_TARGET_PROPERTIES(testScale PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Tests - testScale")
