INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} ${OpenSim_SOURCE_DIR}/Vendors ${XERCES_INCLUDE_DIR})

FILE(GLOB SOURCE_FILES *.cpp)
FILE(GLOB INCLUDE_FILES *.h)

LINK_LIBRARIES(debug xerces-c_2D optimized xerces-c_2 
	debug osimTools_d optimized osimTools
	debug osimSdfastEngine_d optimized osimSdfastEngine)

ADD_EXECUTABLE(addComments ${SOURCE_FILES} ${INCLUDE_FILES})

INSTALL_TARGETS(/bin/ addComments)
