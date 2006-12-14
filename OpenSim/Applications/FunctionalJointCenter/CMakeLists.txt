INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
		    ${OpenSim_SOURCE_DIR}/Vendors 		  
		    ${XERCES_INCLUDE_DIR})

FILE(GLOB SOURCE_FILES *.cpp)
FILE(GLOB INCLUDE_FILES *.h)

LINK_LIBRARIES( debug xerces-c_2D optimized  xerces-c_2 
		debug rdTools_d optimized rdTools
		debug rdSimulation_d optimized rdSimulation
		debug rdSQP_d optimized rdSQP)

ADD_DEFINITIONS(-Dworkflow_EXPORTS)

ADD_EXECUTABLE(fjc ${SOURCE_FILES} ${INCLUDE_FILES})


INSTALL_TARGETS(/bin/ fjc)

#
# Testing
#

IF (UNIX)
  ADD_DEFINITIONS(-fprofile-arcs -ftest-coverage)
  LINK_LIBRARIES(gcov)
ENDIF (UNIX)

ENABLE_TESTING()

IF (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH ${EXECUTABLE_OUTPUT_PATH})
ELSE (EXECUTABLE_OUTPUT_PATH)
  SET (TEST_PATH .)
ENDIF (EXECUTABLE_OUTPUT_PATH)
