INCLUDE_DIRECTORIES(
	${OpenSim_SOURCE_DIR}
	${OpenSim_SOURCE_DIR}/Vendors 		  
	${OpenSim_SOURCE_DIR}/Vendors/xerces-c-src2_4_0/src)

FILE(GLOB SOURCE_FILES *.cpp ../Scale/simmScalerImpl.cpp)
FILE(GLOB INCLUDE_FILES *.h  ../Scale/simmScalerImpl.h)

LINK_LIBRARIES(
	debug xerces-c_2D optimized  xerces-c_2 
	debug rdTools_d optimized rdTools
	debug rdSimulation_d optimized rdSimulation
	debug rdSQP_d optimized rdSQP)

ADD_DEFINITIONS(-Dworkflow_EXPORTS)

ADD_EXECUTABLE(ik ${SOURCE_FILES} ${INCLUDE_FILES})


INSTALL_TARGETS(/bin/ ik)

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

# Disable this testing for now since it's accounted for by Scaling
SUBDIRS(test)
