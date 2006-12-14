FILE(GLOB SOURCE_FILES *.c)
FILE(GLOB INCLUDE_FILES *.h)

LINK_LIBRARIES(
	debug acpp_d optimized acpp
	)

ADD_EXECUTABLE(simmToOpenSim ${SOURCE_FILES} ${INCLUDE_FILES})

INSTALL_TARGETS(/bin/ simmToOpenSim)
