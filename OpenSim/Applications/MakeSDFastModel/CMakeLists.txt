INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
						  ${OpenSim_SOURCE_DIR}/Vendors
						  ${XERCES_INCLUDE_DIR})

ADD_DEFINITIONS(-DOSIMSDFASTENGINE_EXPORTS)

# cmake should automatically figure out what other libraries are needed
LINK_LIBRARIES(
	debug osimSimulation_d optimized osimSimulation
	debug osimSdfastEngine_d optimized osimSdfastEngine)

ADD_EXECUTABLE(makeSDFastModel makeSDFastModel.cpp ${OpenSim_SOURCE_DIR}/OpenSim/DynamicsEngines/SdfastEngine/SdfastStubs.c ${OpenSim_SOURCE_DIR}/OpenSim/DynamicsEngines/SdfastEngine/SdfastStubsCPP.cpp)

##GET_TARGET_PROPERTY(buildLocation makeSDFastModel LOCATION)
##IF (${buildLocation} MATCHES ".+Debug.+")
##	MESSAGE(STATUS "Will Copy")
##ENDIF (${buildLocation} MATCHES ".+Debug.+")
##ADD_CUSTOM_COMMAND(
##   TARGET makeSDFastModel
##    COMMAND ${CMAKE_COMMAND}
##    ARGS -E copy
##    ${EXECUTABLE_OUTPUT_PATH}/Debug/makeSDFastModel.exe
##    ${buildLocation}/../makeSDFastModel_d.exe
##)
INSTALL_TARGETS(/bin/ makeSDFastModel)
