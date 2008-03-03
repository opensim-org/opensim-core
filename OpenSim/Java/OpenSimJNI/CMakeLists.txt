SET(KIT JavaJNI)
SET(UKIT JAVAJNI)


INCLUDE(${CMAKE_ROOT}/Modules/FindJava.cmake)
INCLUDE(${CMAKE_ROOT}/Modules/FindJNI.cmake)
INCLUDE(${CMAKE_ROOT}/Modules/FindSWIG.cmake)

IF(JAVA_INCLUDE_PATH)

# Run command to generate OpenSimJNI_wrap.cxx
SET(swig_generated_file_fullname ${OpenSim_SOURCE_DIR}/OpenSim/Java/OpenSimJNI/OpenSimJNI_wrap.cxx)
SET(swig_interface_file_fullname ${OpenSim_SOURCE_DIR}/OpenSim/Java/swig/javaWrapOpenSim.i)
SET(SimTK_INCLUDE_PATH ${SIMTK_DIR}/core/include/)			
ADD_CUSTOM_TARGET(JavaWrap ${SWIG_EXECUTABLE} -v -c++ -java -package org.opensim.modeling -I${OpenSim_SOURCE_DIR} -I${OpenSim_SOURCE_DIR}/OpenSim/Resources/Swig/ -I${SimTK_INCLUDE_PATH} -o ${swig_generated_file_fullname} -outdir ${OpenSim_SOURCE_DIR}/OpenSim/Applications/Gui/opensim/modeling/src/org/opensim/modeling ${swig_interface_file_fullname})

SET_TARGET_PROPERTIES(JavaWrap PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD on)

FILE(GLOB SOURCE_FILES ${swig_generated_file_fullname} Hooks/*.cpp)
FILE(GLOB INCLUDE_FILES ${swig_generated_file_fullname} Hooks/*.h)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
					${OpenSim_SOURCE_DIR}/Vendors 
					${XERCES_INCLUDE_DIR}
					${JAVA_INCLUDE_PATH}
					${JAVA_INCLUDE_PATH2}
)

SET(EXPORT_MACRO OSIM${UKIT}_EXPORTS)

LINK_LIBRARIES( osimCommon osimSimulation osimAnalyses osimTools
		debug xerces-c_2D optimized  xerces-c_2)

ADD_LIBRARY(osim${KIT} SHARED ${SOURCE_FILES} ${INCLUDE_FILES}) 

SET_TARGET_PROPERTIES(osim${KIT} PROPERTIES
   DEFINE_SYMBOL ${EXPORT_MACRO}
   PROJECT_LABEL "Libraries - osim${KIT}"
)

ADD_DEPENDENCIES(osim${KIT} JavaWrap osimTools)

INSTALL_TARGETS(/lib/ RUNTIME_DIRECTORY /bin osim${KIT})


ENDIF (JAVA_INCLUDE_PATH)
