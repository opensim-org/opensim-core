SET(KIT pyOpenSim)
SET(UKIT PYOPENSIM)

IF(BUILD_PYTHON_WRAPPING)	

INCLUDE(${CMAKE_ROOT}/Modules/FindPythonLibs.cmake)
INCLUDE(${CMAKE_ROOT}/Modules/FindPythonInterp.cmake)


IF(PYTHONLIBS_FOUND)

# Run command to generate OpenSimJNI_wrap.cxx
SET(swig_generated_file_fullname ${OpenSim_SOURCE_DIR}/OpenSim/Wrapping/Python/pyOpenSim_wrap.cxx)
SET(swig_interface_file_fullname ${OpenSim_SOURCE_DIR}/OpenSim/Wrapping/Python/swig/pyWrapOpenSim.i)
SET(SimTK_INCLUDE_PATH ${SIMTK_INCLUDE_DIR})		

ADD_CUSTOM_TARGET(PyWrap ${SWIG_EXECUTABLE} -v -c++ -python -I${OpenSim_SOURCE_DIR} -I${Simbody_INCLUDE_DIR} 
	-o ${swig_generated_file_fullname} -outdir ${OpenSim_SOURCE_DIR}/OpenSim/Wrapping/Python/ ${swig_interface_file_fullname})


FILE(GLOB SOURCE_FILES OpenSimContext.cpp *.cxx)
FILE(GLOB INCLUDE_FILES ${swig_generated_file_fullname} OpenSimContext.h)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
					${OpenSim_SOURCE_DIR}/Vendors 
					${PYTHON_INCLUDE_PATH}
)

SET(EXPORT_MACRO OSIM${UKIT}_EXPORTS)

LINK_LIBRARIES(osimCommon osimSimulation osimAnalyses osimTools osimSimmFileWriter)

ADD_LIBRARY(osim${KIT} SHARED ${SOURCE_FILES} ${INCLUDE_FILES}) 

SET_TARGET_PROPERTIES(osim${KIT} PROPERTIES
   DEFINE_SYMBOL ${EXPORT_MACRO}
   PROJECT_LABEL "Libraries - osim${KIT}"
)

ADD_DEPENDENCIES(osim${KIT} osimTools)

# Shared libraries are needed at runtime for applications, so we put them
# at the top level in OpenSim/bin/*.dll (Windows) or OpenSim/lib/*.so (Linux)
# or OpemSim/lib/*.dylib (Mac). Windows .lib files, and Linux/Mac .a static
# archives are only needed at link time so go in sdk/lib.
IF(WIN32)
    INSTALL(TARGETS osim${KIT}
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION sdk/lib
            ARCHIVE DESTINATION sdk/lib)
ELSE()
    INSTALL(TARGETS osim${KIT}
            RUNTIME DESTINATION bin
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION sdk/lib)
ENDIF()



ENDIF (PYTHONLIBS_FOUND)
ENDIF(BUILD_PYTHON_WRAPPING)
