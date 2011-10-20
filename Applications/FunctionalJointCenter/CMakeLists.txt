SET(APPNAME fjc)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
		    ${OpenSim_SOURCE_DIR}/Vendors 		  
		    ${XERCES_INCLUDE_DIR})

FILE(GLOB SOURCE_FILES *.cpp)
FILE(GLOB INCLUDE_FILES *.h)

LINK_LIBRARIES( debug osimCommon_d optimized osimCommon
		debug osimSimulation_d optimized osimSimulation
		debug osimTools_d optimized osimTools
		${SIMTK_COMMON_LIB})
IF(WIN32)
    LINK_LIBRARIES( debug xerces-c_2D optimized  xerces-c_2)
ELSE(WIN32)
    LINK_LIBRARIES( debug xerces-c optimized  xerces-c)
ENDIF(WIN32)

ADD_EXECUTABLE(fjc ${SOURCE_FILES} ${INCLUDE_FILES})


INSTALL_TARGETS(/bin/ ${APPNAME})
SET_TARGET_PROPERTIES(${APPNAME} PROPERTIES PROJECT_LABEL "Applications - ${APPNAME}")

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
