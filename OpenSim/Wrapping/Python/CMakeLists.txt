SET(KIT opensim)
SET(UKIT PYOPENSIM)

IF(BUILD_PYTHON_WRAPPING)

FIND_PACKAGE(PythonLibs 2.7)
#FIND_PACKAGE(PythonInterp 2.7)

IF(NOT PYTHONLIBS_FOUND)
    MESSAGE(FATAL_ERROR "Cannot build Python wrapping because "
        "Python libraries were not found. Either turn "
        "BUILD_PYTHON_WRAPPING off or install Python "
        "libraries (on Ubuntu, sudo apt-get install python2.7-dev).")
ENDIF()


# Run command to generate OpenSimJNI_wrap.cxx
SET(swig_output_cxx_file_fullname
    ${CMAKE_CURRENT_BINARY_DIR}/pyOpenSim_wrap.cxx)
SET(swig_output_header_file_fullname
    ${CMAKE_CURRENT_BINARY_DIR}/pyOpenSim_wrap.h)
SET(swig_interface_file_fullname
    ${OpenSim_SOURCE_DIR}/OpenSim/Wrapping/Python/swig/pyWrapOpenSim.i)

# Using a custom command / custom target pair so that SWIG is only run when
# it's out of date. Previously, we used a custom target only. But, custom
# targets are always out of date.
ADD_CUSTOM_COMMAND(
    OUTPUT ${swig_output_cxx_file_fullname} ${swig_output_header_file_fullname}
    COMMAND ${SWIG_EXECUTABLE} -v -c++ -python
        #-debug-tmused # Which typemaps were used?
        -I${OpenSim_SOURCE_DIR}
        -I${Simbody_INCLUDE_DIR}
        -I${OpenSim_SOURCE_DIR}/OpenSim/Wrapping/
        -o ${swig_output_cxx_file_fullname}
        -outdir ${CMAKE_CURRENT_BINARY_DIR}
        ${swig_interface_file_fullname}
    DEPENDS ${swig_interface_file_fullname})

ADD_CUSTOM_TARGET(PyWrap
    DEPENDS ${swig_output_cxx_file_fullname}
        ${swig_output_header_file_fullname})

SET_TARGET_PROPERTIES(PyWrap PROPERTIES PROJECT_LABEL "Python Bindings - Generate")

SET(SOURCE_FILES "${swig_output_cxx_file_fullname}")
SET(INCLUDE_FILES "${swig_output_header_file_fullname}")

IF(${CMAKE_CXX_COMPILER_ID} MATCHES "GNU" OR
        ${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    # Turn off optimization for SWIG wrapper code. Optimization slows down
    # compiling and also requires lots of memory. Also, there's not much to gain
    # from an optimized wrapper file. 
    # Note that the last optimization flag is what counts for GCC. So an -O0
    # later on the command line overrides a previous -O2.
    SET(_COMPILE_FLAGS "-O0")

    IF (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
        # swig 2.0.10 uses the c++ register keyword, which Clang tells us is
        # deprecated. While still using swig 2.0.10, we suppress this warning.
        # In some versions of Clang, this warning is Wdeprecated-register.
        LIST(APPEND _COMPILE_FLAGS "-Wno-deprecated")
    ENDIF()

    SET_SOURCE_FILES_PROPERTIES("${swig_output_cxx_file_fullname}"
        PROPERTIES COMPILE_FLAGS "${_COMPILE_FLAGS}")
ENDIF()
# TODO disable optimization on Windows.

INCLUDE_DIRECTORIES(${OpenSim_SOURCE_DIR}
                    ${OpenSim_SOURCE_DIR}/Vendors
                    ${PYTHON_INCLUDE_PATH}
)

SET(EXPORT_MACRO OSIM${UKIT}_EXPORTS)

LINK_LIBRARIES(osimCommon osimSimulation osimAnalyses osimTools osimActuators
    ${PYTHON_LIBRARIES})

ADD_LIBRARY(_${KIT} SHARED ${SOURCE_FILES} ${INCLUDE_FILES})




# Resulting library must be named _opensim.so on Unix, _opensim.pyd on Windows.
SET_TARGET_PROPERTIES(_${KIT} PROPERTIES
   DEFINE_SYMBOL ${EXPORT_MACRO}
   PROJECT_LABEL "Python Bindings - _${KIT}"
   PREFIX ""
)
IF(WIN32)
    SET_TARGET_PROPERTIES(_${KIT} PROPERTIES SUFFIX ".pyd")
ELSEIF(APPLE)
    # Defaults to .dylib; change to .so.
    SET_TARGET_PROPERTIES(_${KIT} PROPERTIES SUFFIX ".so")
ENDIF()

# Shared libraries are needed at runtime for applications, so we put them
# at the top level in OpenSim/bin/*.dll (Windows) or OpenSim/lib/*.so (Linux)
# or OpemSim/lib/*.dylib (Mac). Windows .lib files, and Linux/Mac .a static
# archives are only needed at link time so go in sdk/lib.
INSTALL(TARGETS _${KIT}
        RUNTIME DESTINATION sdk/python/opensim
        LIBRARY DESTINATION sdk/python/opensim
        ARCHIVE DESTINATION sdk/python/opensim)

#install .py files
INSTALL(FILES setup.py DESTINATION sdk/python)
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/opensim.py
    DESTINATION sdk/python/opensim)
INSTALL(FILES __init__.py DESTINATION sdk/python/opensim)
INSTALL(DIRECTORY tests DESTINATION sdk/python/opensim)

ENDIF(BUILD_PYTHON_WRAPPING)
