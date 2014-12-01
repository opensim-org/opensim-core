cmake_minimum_required(VERSION 2.6)

# Define project
PROJECT (VisualizeModel)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} ${OpenSim_SOURCE_DIR}/Vendors)
# Identify the cpp file(s) that were to be built
FILE(GLOB SOURCE_FILES *.h *.cpp)
SET(SOURCE ${SOURCE_FILES})

SET(APPNAME visualizeModel)
ADD_EXECUTABLE(${APPNAME} ${SOURCE})

SET_TARGET_PROPERTIES(${APPNAME}        
       PROPERTIES        
       PROJECT_LABEL "Utilities - ${APPNAME}"       
)

TARGET_LINK_LIBRARIES(${APPNAME}
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

MARK_AS_ADVANCED(CMAKE_INSTALL_PREFIX)
MARK_AS_ADVANCED(EXECUTABLE_OUTPUT_PATH)
MARK_AS_ADVANCED(LIBRARY_OUTPUT_PATH)
