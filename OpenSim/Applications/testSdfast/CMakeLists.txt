INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
		    ${OpenSim_SOURCE_DIR}/Vendors 		    
			 ${XERCES_INCLUDE_DIR}
		    ${OpenSim_SOURCE_DIR}/OpenSim/DynamicsEngines/SdfastEngine/SIMMPipeline)

FILE(GLOB SOURCE_FILES testSdfast.cpp dyn/*.c dyn/*.cpp ${OpenSim_SOURCE_DIR}/OpenSim/DynamicsEngines/SdfastEngine/SIMMPipeline/*.cpp ${OpenSim_SOURCE_DIR}/OpenSim/DynamicsEngines/SdfastEngine/SIMMPipeline/*.c)
FILE(GLOB INCLUDE_FILES testSdfast.h dyn/*.h ${OpenSim_SOURCE_DIR}/OpenSim/DynamicsEngines/SdfastEngine/SIMMPipeline/*.h)

ADD_DEFINITIONS(-DOSIMSDFASTENGINE_EXPORTS)

LINK_LIBRARIES( debug osimCommon_d optimized osimCommon
		debug osimSimulation_d optimized osimSimulation
		debug osimAnalyses_d optimized osimAnalyses
		debug osimSdfastEngine_d optimized osimSdfastEngine)

ADD_EXECUTABLE(testSdfast ${SOURCE_FILES} ${INCLUDE_FILES})

##GET_TARGET_PROPERTY(buildLocation testSdfast LOCATION)
##IF (${buildLocation} MATCHES ".+Debug.+")
##	MESSAGE(STATUS "Will Copy")
##ENDIF (${buildLocation} MATCHES ".+Debug.+")
##ADD_CUSTOM_COMMAND(
##   TARGET testSdfast
##    COMMAND ${CMAKE_COMMAND}
##    ARGS -E copy
##    ${EXECUTABLE_OUTPUT_PATH}/Debug/testSdfast.exe
##    ${buildLocation}/../testSdfast_d.exe
##)
INSTALL_TARGETS(/bin/ testSdfast)
