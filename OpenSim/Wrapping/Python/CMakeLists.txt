set(KIT opensim)
set(UKIT PYOPENSIM)

if(BUILD_PYTHON_WRAPPING)

find_package(PythonLibs 2.7)
#find_package(PythonInterp 2.7)

if(NOT PYTHONLIBS_FOUND)
    message(FATAL_ERROR "Cannot build Python wrapping because "
        "Python libraries were not found. Either turn "
        "BUILD_PYTHON_WRAPPING off or install Python "
        "libraries (on Ubuntu, sudo apt-get install python2.7-dev).")
endif()


# Run command to generate OpenSimJNI_wrap.cxx
set(swig_output_cxx_file_fullname
    ${CMAKE_CURRENT_BINARY_DIR}/pyOpenSim_wrap.cxx)
set(swig_output_header_file_fullname
    ${CMAKE_CURRENT_BINARY_DIR}/pyOpenSim_wrap.h)
set(swig_interface_file_fullname
    ${OpenSim_SOURCE_DIR}/OpenSim/Wrapping/Python/swig/pyWrapOpenSim.i)

# Using a custom command / custom target pair so that SWIG is only run when
# it's out of date. Previously, we used a custom target only. But, custom
# targets are always out of date.
add_custom_command(
    OUTPUT opensim.py
        ${swig_output_cxx_file_fullname} ${swig_output_header_file_fullname}
    COMMAND ${SWIG_EXECUTABLE} -v -c++ -python
        #-debug-tmused # Which typemaps were used?
        -I${OpenSim_SOURCE_DIR}
        -I${OpenSim_SOURCE_DIR}/OpenSim/Wrapping/
        -I${Simbody_INCLUDE_DIR}
        -o ${swig_output_cxx_file_fullname}
        -outdir ${CMAKE_CURRENT_BINARY_DIR}
        ${swig_interface_file_fullname}
    DEPENDS ${swig_interface_file_fullname})

add_custom_target(PyWrap
    DEPENDS opensim.py ${swig_output_cxx_file_fullname}
        ${swig_output_header_file_fullname})

set_target_properties(PyWrap PROPERTIES PROJECT_LABEL "Python Bindings - Generate")

set(SOURCE_FILES "${swig_output_cxx_file_fullname}")
set(INCLUDE_FILES "${swig_output_header_file_fullname}")

if(${CMAKE_CXX_COMPILER_ID} MATCHES "GNU" OR
        ${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    # Turn off optimization for SWIG wrapper code. Optimization slows down
    # compiling and also requires lots of memory. Also, there's not much to gain
    # from an optimized wrapper file. 
    # Note that the last optimization flag is what counts for GCC. So an -O0
    # later on the command line overrides a previous -O2.
    set(_COMPILE_FLAGS "-O0")

    if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
        # swig 2.0.10 uses the c++ register keyword, which Clang tells us is
        # deprecated. While still using swig 2.0.10, we suppress this warning.
        # In some versions of Clang, this warning is Wdeprecated-register.
        list(APPEND _COMPILE_FLAGS "-Wno-deprecated")
    endif()

    set_source_files_properties("${swig_output_cxx_file_fullname}"
        PROPERTIES COMPILE_FLAGS "${_COMPILE_FLAGS}")
endif()
# TODO disable optimization on Windows.

include_directories(${OpenSim_SOURCE_DIR} 
                    ${OpenSim_SOURCE_DIR}/Vendors 
                    ${PYTHON_INCLUDE_PATH}
)

set(EXPORT_MACRO OSIM${UKIT}_EXPORTS)

link_libraries(osimCommon osimSimulation osimAnalyses osimTools osimActuators
    ${PYTHON_LIBRARIES})

add_library(_${KIT} SHARED ${SOURCE_FILES} ${INCLUDE_FILES})




# Resulting library must be named _opensim.so on Unix, _opensim.pyd on Windows.
set_target_properties(_${KIT} PROPERTIES
   DEFINE_SYMBOL ${EXPORT_MACRO}
   PROJECT_LABEL "Python Bindings - _${KIT}"
   PREFIX ""
)
if(WIN32)
    set_target_properties(_${KIT} PROPERTIES SUFFIX ".pyd")
elseif(APPLE)
    # Defaults to .dylib; change to .so.
    set_target_properties(_${KIT} PROPERTIES SUFFIX ".so")
endif()

# Shared libraries are needed at runtime for applications, so we put them
# at the top level in OpenSim/bin/*.dll (Windows) or OpenSim/lib/*.so (Linux)
# or OpemSim/lib/*.dylib (Mac). Windows .lib files, and Linux/Mac .a static
# archives are only needed at link time so go in sdk/lib.
install(TARGETS _${KIT}
        RUNTIME DESTINATION sdk/python/opensim
        LIBRARY DESTINATION sdk/python/opensim
        ARCHIVE DESTINATION sdk/python/opensim)

# Configure setup.py
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in
               ${CMAKE_CURRENT_BINARY_DIR}/setup.py @ONLY)

#install .py files
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.py DESTINATION sdk/python)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/opensim.py
    DESTINATION sdk/python/opensim)
install(FILES __init__.py DESTINATION sdk/python/opensim)
install(DIRECTORY tests DESTINATION sdk/python/opensim)

endif(BUILD_PYTHON_WRAPPING)
