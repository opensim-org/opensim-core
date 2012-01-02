SET(KIT JavaJNI)
SET(UKIT JAVAJNI)

IF(BUILD_JAVA_WRAPPING)	

INCLUDE(${CMAKE_ROOT}/Modules/FindJava.cmake)
INCLUDE(${CMAKE_ROOT}/Modules/FindJNI.cmake)
INCLUDE(${CMAKE_ROOT}/Modules/FindSWIG.cmake)

IF(JAVA_INCLUDE_PATH)

# Run command to generate OpenSimJNI_wrap.cxx
SET(swig_generated_file_fullname ${OpenSim_SOURCE_DIR}/OpenSim/Java/OpenSimJNI/OpenSimJNI_wrap.cxx)
SET(swig_interface_file_fullname ${OpenSim_SOURCE_DIR}/OpenSim/Java/swig/javaWrapOpenSim.i)
SET(SimTK_INCLUDE_PATH ${SIMTK_INCLUDE_DIR})		

ADD_CUSTOM_TARGET(JavaWrap ${SWIG_EXECUTABLE} -v -c++ -java -package org.opensim.modeling -I${OpenSim_SOURCE_DIR} -I${OpenSim_SOURCE_DIR}/OpenSim/Swig/ -I${SimTK_INCLUDE_PATH} -o ${swig_generated_file_fullname} -outdir ${OpenSim_SOURCE_DIR}/OpenSim/Java/src/org/opensim/modeling ${swig_interface_file_fullname})
SET_TARGET_PROPERTIES(JavaWrap PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD on)


FILE(GLOB SOURCE_FILES OpenSimContext.cpp)
FILE(GLOB INCLUDE_FILES ${swig_generated_file_fullname} OpenSimContext.h)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
					${OpenSim_SOURCE_DIR}/Vendors 
					${JAVA_INCLUDE_PATH}
					${JAVA_INCLUDE_PATH2}
)

SET(EXPORT_MACRO OSIM${UKIT}_EXPORTS)

LINK_LIBRARIES(osimCommon osimSimulation osimAnalyses osimTools osimSimmFileWriter)

ADD_LIBRARY(osim${KIT} SHARED ${SOURCE_FILES} ${INCLUDE_FILES}) 

SET_TARGET_PROPERTIES(osim${KIT} PROPERTIES
   DEFINE_SYMBOL ${EXPORT_MACRO}
   PROJECT_LABEL "Libraries - osim${KIT}"
)

ADD_DEPENDENCIES(osim${KIT} osimTools)

IF (WIN32)
	INSTALL_TARGETS(/sdk/lib/ RUNTIME_DIRECTORY /bin osim${KIT})
ELSE(WIN32)
	INSTALL_TARGETS(/bin/ RUNTIME_DIRECTORY /bin osim${KIT})
ENDIF(WIN32)

IF(BUILD_TESTING)
	SUBDIRS(Test)
ENDIF(BUILD_TESTING)


ENDIF (JAVA_INCLUDE_PATH)
ENDIF(BUILD_JAVA_WRAPPING)
