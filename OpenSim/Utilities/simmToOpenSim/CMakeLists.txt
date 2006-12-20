FILE(GLOB SOURCE_FILES *.c)
FILE(GLOB INCLUDE_FILES *.h)

IF(WIN32)
LINK_LIBRARIES(
	debug acpp_d optimized acpp
	)
ELSE(WIN32)
# Math library for linux
LINK_LIBRARIES(m)
ENDIF(WIN32)

ADD_EXECUTABLE(simmToOpenSim ${SOURCE_FILES} ${INCLUDE_FILES})

INSTALL_TARGETS(/bin/ simmToOpenSim)
