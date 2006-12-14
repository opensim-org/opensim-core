INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR}
                    ${OpenSim_SOURCE_DIR}/Vendors 
						  ${XERCES_INCLUDE_DIR})

LINK_LIBRARIES( debug rdTools_d optimized rdTools
		debug rdSimulation_d optimized rdSimulation
		debug suAnalyses_d optimized suAnalyses
		debug rdCMC_d optimized rdCMC)

ADD_EXECUTABLE(cmcgait cmcgait.cpp)

INSTALL_TARGETS(/bin/ cmcgait)
#
# Testing
#

IF (UNIX)
  ADD_DEFINITIONS(-fprofile-arcs -ftest-coverage)
  LINK_LIBRARIES(gcov)
ENDIF (UNIX)
