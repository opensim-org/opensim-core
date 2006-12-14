INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
		    ${OpenSim_SOURCE_DIR}/Vendors 		  
		    ${XERCES_INCLUDE_DIR})

FILE(GLOB SOURCE_FILES *.cpp)
FILE(GLOB INCLUDE_FILES *.h)

LINK_LIBRARIES(
	debug xerces-c_2D optimized  xerces-c_2 
	debug rdTools_d optimized rdTools
	debug rdSimulation_d optimized rdSimulation
	debug suAnalyses_d optimized suAnalyses
	debug rdBlock_d optimized rdBlock
)

ADD_EXECUTABLE(fallingBlock ${SOURCE_FILES} ${INCLUDE_FILES})

INSTALL_TARGETS(/bin/ fallingBlock)

