cmake_minimum_required(VERSION 2.6)

# Define project
PROJECT (VisualizeModel)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} ${OpenSim_SOURCE_DIR}/Vendors)
# Identify the cpp file(s) that were to be built
FILE(GLOB SOURCE_FILES *.h *.cpp)
SET(SOURCE ${SOURCE_FILES})

ADD_EXECUTABLE(${TARGET} ${SOURCE})

TARGET_LINK_LIBRARIES(${TARGET}
    debug osimSimulation_d  optimized osimSimulation
    debug osimActuators_d   optimized osimActuators
    debug osimCommon_d      optimized osimCommon
    debug osimAnalyses_d    optimized osimAnalyses
    debug osimTools_d       optimized osimTools
    debug ${NameSpace}SimTKcommon_d optimized ${NameSpace}SimTKcommon
    debug ${NameSpace}SimTKmath_d optimized  ${NameSpace}SimTKmath
    debug ${NameSpace}SimTKsimbody_d optimized ${NameSpace}SimTKsimbody
    ${PLATFORM_LIBS}
)

IF(WIN32)
    SET(PLATFORM_LIBS  pthreadVC2)
ELSE (WIN32)
  SET(NameSpace "")
  IF(APPLE)
    SET(PLATFORM_LIBS  SimTKAtlas)
  ELSE(APPLE)
    SET(PLTAFORM_LIBS SimTKAtlas_Lin_generic)
  ENDIF(APPLE)
ENDIF (WIN32)

# This block copies the additional files into the running directory
# For example vtp, obj files. Add to the end for more extentions
FILE(GLOB DATA_FILES *.vtp *.obj)
FOREACH (dataFile ${DATA_FILES})
 ADD_CUSTOM_COMMAND(
    TARGET ${TARGET}
    COMMAND ${CMAKE_COMMAND}
    ARGS -E copy
    ${dataFile}
    ${OpenSimTugOfWar_BINARY_DIR})
 ENDFOREACH (dataFile) 

MARK_AS_ADVANCED(CMAKE_INSTALL_PREFIX)
MARK_AS_ADVANCED(EXECUTABLE_OUTPUT_PATH)
MARK_AS_ADVANCED(LIBRARY_OUTPUT_PATH)
