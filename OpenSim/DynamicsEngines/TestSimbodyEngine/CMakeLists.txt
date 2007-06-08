INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
						  ${OpenSim_SOURCE_DIR}/Vendors
						  ${XERCES_INCLUDE_DIR})

# cmake should automatically figure out what other libraries are needed
LINK_LIBRARIES(debug osimCommon_d osimSimulation_d osimSimbodyEngine_d osimAnalyses_d)

ADD_EXECUTABLE(testSimbodyEngine testSimbodyEngine.cpp)
        
INSTALL_TARGETS(/bin/ testSimbodyEngine)
#
# Testing
#

#IF (UNIX)
#  ADD_DEFINITIONS(-fprofile-arcs -ftest-coverage)
#  LINK_LIBRARIES(gcov)
#ENDIF (UNIX)

ENABLE_TESTING()

IF (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
ELSE (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH .)
ENDIF (EXECUTABLE_OUTPUT_PATH)

IF(NOT OPENSIM_MINIMAL_BUILD)
SUBDIRS(test)
ENDIF(NOT OPENSIM_MINIMAL_BUILD)
