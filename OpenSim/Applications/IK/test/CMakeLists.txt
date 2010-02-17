INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
		    ${OpenSim_SOURCE_DIR}/Vendors 		  
		    ${XERCES_INCLUDE_DIR})


LINK_LIBRARIES( debug osimCommon_d optimized osimCommon
		debug osimSimulation_d optimized osimSimulation
		debug osimActuators_d optimized osimActuators
		debug osimTools_d optimized osimTools
		debug osimAnalyses_d optimized osimAnalyses
		${SIMTK_ALL_LIBS}
		)

ADD_EXECUTABLE(testIK testIK.cpp)

FILE(GLOB TEST_FILES *.osim *.xml *.sto *.mot *.trc)

#
# Copy files to run dir
#
FOREACH (dataFile ${TEST_FILES})
 ADD_CUSTOM_COMMAND(
    TARGET testIK
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${dataFile}
    ${OpenSim_BINARY_DIR}/OpenSim/Applications/IK/test)
 ENDFOREACH (dataFile) 


IF (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
ELSE (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH .)
ENDIF (EXECUTABLE_OUTPUT_PATH)

ADD_TEST(testIK ${TEST_PATH}/testIK)

SET_TARGET_PROPERTIES(testIK PROPERTIES PROJECT_LABEL "Tests - testIK")
