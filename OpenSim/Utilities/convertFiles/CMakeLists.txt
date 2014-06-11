INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} ${OpenSim_SOURCE_DIR}/Vendors ${XERCES_INCLUDE_DIR})

FILE(GLOB SOURCE_FILES *.cpp)
FILE(GLOB INCLUDE_FILES *.h)

IF (BUILD_STATIC_OSIM_LIBS)
	ADD_DEFINITIONS(-DSTATIC_OSIM_LIBS)
ENDIF(BUILD_STATIC_OSIM_LIBS)

LINK_LIBRARIES(debug xerces-c_2D optimized xerces-c_2 
	debug osimTools_d optimized osimTools
	debug osimSimbodyEngine_d optimized osimSimbodyEngine)

ADD_EXECUTABLE(convertFiles ${SOURCE_FILES} ${INCLUDE_FILES})

SET_TARGET_PROPERTIES(convertFiles PROPERTIES ${EXCLUDE_IF_MINIMAL_BUILD} PROJECT_LABEL "Utilities - convertFiles")

INSTALL_TARGETS(/bin/ convertFiles)
