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

set_target_properties(PyWrap PROPERTIES
    PROJECT_LABEL "Python - Generate source code"
    FOLDER "Wrapping (scripting)")

set(SOURCE_FILES OpenSimContext.cpp "${swig_output_cxx_file_fullname}")
set(INCLUDE_FILES OpenSimContext.h "${swig_output_header_file_fullname}")

# swig 2.0.10 uses the c++ register keyword, which Clang tells us is
# deprecated. While still using swig 2.0.10, we suppress this warning.
# In some versions of Clang, this warning is Wdeprecated-register.
if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    set_source_files_properties("${swig_output_cxx_file_fullname}"
        PROPERTIES COMPILE_FLAGS "-Wno-deprecated")
endif()

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
   PROJECT_LABEL "Python - _${KIT}"
   FOLDER "Wrapping (scripting)"
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
install(FILES ${OPENSIM_SHARED_TEST_FILES_DIR}/arm26.osim
              ${OPENSIM_SHARED_TEST_FILES_DIR}/gait10dof18musc_subject01.osim
        DESTINATION sdk/python/opensim/tests)
INSTALL(DIRECTORY tests DESTINATION sdk/python/opensim)

endif(BUILD_PYTHON_WRAPPING)
