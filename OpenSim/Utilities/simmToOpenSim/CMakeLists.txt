add_definitions(
    -DOPENSIM_BUILD=1
    -DINCLUDE_MOCAP_MODULE=1
    -DINCLUDE_EVA_REALTIME=0
    -DEXPERIMENTAL=0
    -DENGINE=1
    -DSIMMTOOPENSIM=1)
include_directories(${OpenSim_SOURCE_DIR} )

file(GLOB SOURCE_FILES *.c)
file(GLOB INCLUDE_FILES *.h)

if(WIN32)
link_libraries(
    debug acpp_d optimized acpp
    )
else(WIN32)
# m is math library for linux
link_libraries(
    m
    debug acpp${CMAKE_DEBUG_POSTFIX} optimized acpp
    )
endif(WIN32)

add_executable(simmToOpenSim ${SOURCE_FILES} ${INCLUDE_FILES})

set_target_properties(simmToOpenSim PROPERTIES
    PROJECT_LABEL "Utilities - simmToOpenSim")

install(TARGETS simmToOpenSim DESTINATION "${CMAKE_INSTALL_BINDIR}")

subdirs(acpp)
