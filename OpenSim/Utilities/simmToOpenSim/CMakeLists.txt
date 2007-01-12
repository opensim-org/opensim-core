FILE(GLOB SOURCE_FILES *.c)
FILE(GLOB INCLUDE_FILES *.h)

IF(WIN32)
LINK_LIBRARIES(
	debug acpp_d optimized acpp
	)
ELSE(WIN32)
# m is math library for linux
LINK_LIBRARIES(
	m
	debug acpp_d optimized acpp
	)
ENDIF(WIN32)

ADD_EXECUTABLE(simmToOpenSim ${SOURCE_FILES} ${INCLUDE_FILES})

INSTALL_TARGETS(/bin/ simmToOpenSim)
