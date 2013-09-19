SET(KIT opensim)
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

LINK_LIBRARIES(osimCommon osimSimulation osimAnalyses osimTools ${PYTHON_LIBRARIES})

ADD_LIBRARY(_${KIT} SHARED ${SOURCE_FILES} ${INCLUDE_FILES}) 

# Resulting library must be named _opensim.so on Unix, _opensim.pyd on Windows.
SET_TARGET_PROPERTIES(_${KIT} PROPERTIES
   DEFINE_SYMBOL ${EXPORT_MACRO}
   PROJECT_LABEL "Python Bindings - _${KIT}"
   PREFIX ""
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
            RUNTIME DESTINATION sdk/python/opensim
            LIBRARY DESTINATION sdk/python/opensim
            ARCHIVE DESTINATION sdk/python/opensim)
ELSE()
    INSTALL(TARGETS _${KIT}
            RUNTIME DESTINATION sdk/python/opensim
            LIBRARY DESTINATION sdk/python/opensim
            ARCHIVE DESTINATION sdk/python/opensim)
ENDIF()

#install .py files
INSTALL(FILES setup.py DESTINATION sdk/python)
INSTALL(FILES opensim.py DESTINATION sdk/python/opensim)
INSTALL(FILES __init__.py DESTINATION sdk/python/opensim)
INSTALL(DIRECTORY tests DESTINATION sdk/python/opensim)


ENDIF (PYTHONLIBS_FOUND)
ENDIF(BUILD_PYTHON_WRAPPING)
