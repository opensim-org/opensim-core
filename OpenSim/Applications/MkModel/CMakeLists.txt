INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
		    ${OpenSim_SOURCE_DIR}/Vendors 		    
		    ${OpenSim_SOURCE_DIR}/Vendors/xerces-c-src2_4_0/src)


LINK_LIBRARIES( debug rdTools_d optimized rdTools
		debug rdSimulation_d optimized rdSimulation
		debug suAnalyses_d optimized suAnalyses)

ADD_EXECUTABLE(mkModel mkModel.cpp)

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
