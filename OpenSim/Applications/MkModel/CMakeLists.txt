INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
		    ${OpenSim_SOURCE_DIR}/Vendors 		    
		    ${OpenSim_SOURCE_DIR}/Vendors/xerces-c-src2_4_0/src)


ADD_DEFINITIONS(-DSDFAST_ENGINE_EXPORTS)

LINK_LIBRARIES( debug rdTools_d optimized rdTools
		debug rdSimulation_d optimized rdSimulation
		debug suAnalyses_d optimized suAnalyses
		debug SdfastEngine_d optimized SdfastEngine)

ADD_EXECUTABLE(mkModel mkModel.cpp ${OpenSim_SOURCE_DIR}/OpenSim/Models/SdfastEngine/SdfastStubs.c ${OpenSim_SOURCE_DIR}/OpenSim/Subject/SdfastFileWriter.cpp)

##GET_TARGET_PROPERTY(buildLocation mkModel LOCATION)
##IF (${buildLocation} MATCHES ".+Debug.+")
##	MESSAGE(STATUS "Will Copy")
##ENDIF (${buildLocation} MATCHES ".+Debug.+")
##ADD_CUSTOM_COMMAND(
##   TARGET mkModel
##    COMMAND ${CMAKE_COMMAND}
##    ARGS -E copy
##    ${EXECUTABLE_OUTPUT_PATH}/Debug/mkModel.exe
##    ${buildLocation}/../mkModel_d.exe
##)
INSTALL_TARGETS(/bin/ mkModel)
