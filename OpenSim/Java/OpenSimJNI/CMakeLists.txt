SET(KIT JavaJNI)
SET(UKIT JAVAJNI)


INCLUDE(${CMAKE_ROOT}/Modules/FindJava.cmake)
INCLUDE(${CMAKE_ROOT}/Modules/FindJNI.cmake)
INCLUDE(${CMAKE_ROOT}/Modules/FindSWIG.cmake)

IF(JAVA_INCLUDE_PATH)

# Run command to generate OpenSimJNI_wrap.cxx
SET(swig_generated_file_fullname ${OpenSim_SOURCE_DIR}/OpenSim/Java/OpenSimJNI/OpenSimJNI_wrap.cxx)
SET(swig_interface_file_fullname ${OpenSim_SOURCE_DIR}/OpenSim/Java/swig/javaWrapOpenSim.i)

ADD_CUSTOM_TARGET(JavaWrap ${SWIG_EXECUTABLE} -v -c++ -java -package org.opensim.modeling -I${OpenSim_SOURCE_DIR} -I${OpenSim_SOURCE_DIR}/OpenSim/Resources/Swig/ -o ${swig_generated_file_fullname} -outdir ${OpenSim_SOURCE_DIR}/OpenSim/Applications/Gui/opensim/modeling/src/org/opensim/modeling ${swig_interface_file_fullname})

FILE(GLOB SOURCE_FILES ${swig_generated_file_fullname} Hooks/*.cpp)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
					${OpenSim_SOURCE_DIR}/Vendors 
					${XERCES_INCLUDE_DIR}
					${JAVA_INCLUDE_PATH}
					${JAVA_INCLUDE_PATH2}
)

SET(EXPORT_MACRO OSIM${UKIT}_EXPORTS)

LINK_LIBRARIES( osimCommon osimSimulation osimAnalyses osimTools
		debug xerces-c_2D optimized  xerces-c_2)

ADD_LIBRARY(osim${KIT} SHARED ${SOURCE_FILES}) 

SET_TARGET_PROPERTIES(osim${KIT} PROPERTIES
   DEFINE_SYMBOL ${EXPORT_MACRO}
)

ADD_DEPENDENCIES(osim${KIT} JavaWrap osimTools)

INSTALL_TARGETS(/lib/ RUNTIME_DIRECTORY /bin osim${KIT})


ENDIF (JAVA_INCLUDE_PATH)