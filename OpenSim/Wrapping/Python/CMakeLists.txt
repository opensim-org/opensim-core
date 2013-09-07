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

SET_TARGET_PROPERTIES(PyWrap PROPERTIES PROJECT_LABEL "Python Bindings - Generate")

FILE(GLOB SOURCE_FILES OpenSimContext.cpp *.cxx)
FILE(GLOB INCLUDE_FILES ${swig_generated_file_fullname} OpenSimContext.h)

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR} 
					${OpenSim_SOURCE_DIR}/Vendors 
					${PYTHON_INCLUDE_PATH}
)

SET(EXPORT_MACRO OSIM${UKIT}_EXPORTS)

LINK_LIBRARIES(osimCommon osimSimulation osimAnalyses osimTools osimSimmFileWriter ${PYTHON_LIBRARIES})

ADD_LIBRARY(_${KIT} SHARED ${SOURCE_FILES} ${INCLUDE_FILES}) 

SET_TARGET_PROPERTIES(_${KIT} PROPERTIES
   DEFINE_SYMBOL ${EXPORT_MACRO}
   PROJECT_LABEL "Python Bindings - _${KIT}"
)
IF(WIN32)
	SET_TARGET_PROPERTIES(_${KIT}
    PROPERTIES 
    SUFFIX ".pyd")
ENDIF()

ADD_DEPENDENCIES(_${KIT} osimTools)
ADD_DEPENDENCIES(_${KIT} PyWrap)

# Shared libraries are needed at runtime for applications, so we put them
# at the top level in OpenSim/bin/*.dll (Windows) or OpenSim/lib/*.so (Linux)
# or OpemSim/lib/*.dylib (Mac). Windows .lib files, and Linux/Mac .a static
# archives are only needed at link time so go in sdk/lib.
IF(WIN32)
    INSTALL(TARGETS _${KIT}
            RUNTIME DESTINATION sdk/python/
            LIBRARY DESTINATION sdk/python/
            ARCHIVE DESTINATION sdk/python/)
ELSE()
    INSTALL(TARGETS _${KIT}
            RUNTIME DESTINATION sdk/python/
            LIBRARY DESTINATION sdk/python/
            ARCHIVE DESTINATION sdk/python/)
ENDIF()

#install .py files
FILE(GLOB PY_FILES "*.py")
INSTALL(FILES ${PY_FILES} DESTINATION sdk/python/)


ENDIF (PYTHONLIBS_FOUND)
ENDIF(BUILD_PYTHON_WRAPPING)
