SET(APPNAME analyze)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
						  ${OpenSim_SOURCE_DIR}/Vendors)

# cmake should automatically figure out what other libraries are needed
LINK_LIBRARIES(debug osimTools${CMAKE_DEBUG_POSTFIX} optimized osimTools
               debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
               debug osimActuators${CMAKE_DEBUG_POSTFIX} optimized osimActuators
               debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation)

ADD_EXECUTABLE(${APPNAME} ${APPNAME}.cpp)
INSTALL_TARGETS(/bin/ ${APPNAME})
SET_TARGET_PROPERTIES(${APPNAME} PROPERTIES PROJECT_LABEL "Applications - ${APPNAME}")

#
# Testing
#

#IF (UNIX)
#  ADD_DEFINITIONS(-fprofile-arcs -ftest-coverage)
#  LINK_LIBRARIES(gcov)
#ENDIF (UNIX)

#ENABLE_TESTING()

IF (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
ELSE (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH .)
ENDIF (EXECUTABLE_OUTPUT_PATH)

IF(BUILD_TESTING)
	SUBDIRS(test)
ENDIF(BUILD_TESTING)
