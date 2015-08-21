set(KIT opensim)

if(BUILD_PYTHON_WRAPPING)

find_package(PythonLibs 2.7)
find_package(PythonInterp 2.7 REQUIRED)

if(NOT PYTHONLIBS_FOUND)
    message(FATAL_ERROR "Cannot build Python wrapping because "
        "Python libraries were not found. Either turn "
        "BUILD_PYTHON_WRAPPING off or install Python "
        "libraries (on Ubuntu, sudo apt-get install python2.7-dev).")
endif()


# Location of the opensim python package in the build directory, for testing.
set(PYTHON_PACKAGE_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/opensim")

# Generate source code for wrapper using SWIG.
# --------------------------------------------
set(swig_output_cxx_file_fullname
    ${CMAKE_CURRENT_BINARY_DIR}/pyOpenSim_wrap.cxx)
set(swig_output_header_file_fullname
    ${CMAKE_CURRENT_BINARY_DIR}/pyOpenSim_wrap.h)
set(swig_interface_file_fullname
    ${OpenSim_SOURCE_DIR}/OpenSim/Wrapping/Python/swig/pyWrapOpenSim.i)

# Using a custom command / custom target pair so that SWIG is only run when
# it's out of date. Previously, we used a custom target only. But, custom
# targets are always out of date.
# TODO can use `swig -M` to detect file dependencies for CMake.
add_custom_command(
    OUTPUT "${PYTHON_PACKAGE_BINARY_DIR}/opensim/opensim.py"
        ${swig_output_cxx_file_fullname} ${swig_output_header_file_fullname}
    COMMAND ${SWIG_EXECUTABLE} -v -c++ -python
        #-debug-tmused # Which typemaps were used?
        -I${OpenSim_SOURCE_DIR}
        -I${OpenSim_SOURCE_DIR}/OpenSim/Wrapping/
        -I${Simbody_INCLUDE_DIR}
        -o ${swig_output_cxx_file_fullname}
        -outdir "${PYTHON_PACKAGE_BINARY_DIR}/opensim"
        ${swig_interface_file_fullname}
    DEPENDS ${swig_interface_file_fullname})

add_custom_target(PyWrap
    DEPENDS "${PYTHON_PACKAGE_BINARY_DIR}/opensim/opensim.py"
        ${swig_output_cxx_file_fullname}
        ${swig_output_header_file_fullname})

set_target_properties(PyWrap PROPERTIES PROJECT_LABEL "Python Bindings - Generate")


# Compile python wrapper files.
# -----------------------------
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
        # TODO remove when we upgrade swig.
        set(_COMPILE_FLAGS "${_COMPILE_FLAGS} -Wno-deprecated")
    endif()

    set_source_files_properties("${swig_output_cxx_file_fullname}"
        PROPERTIES COMPILE_FLAGS "${_COMPILE_FLAGS}")
endif()
# TODO disable optimization on Windows.

include_directories(${OpenSim_SOURCE_DIR} 
                    ${OpenSim_SOURCE_DIR}/Vendors 
                    ${PYTHON_INCLUDE_PATH}
)

link_libraries(osimCommon osimSimulation osimAnalyses osimTools osimActuators
    ${PYTHON_LIBRARIES})

add_library(_${KIT} SHARED ${SOURCE_FILES} ${INCLUDE_FILES})

# Resulting library must be named _opensim.so on Unix, _opensim.pyd on Windows.
set_target_properties(_${KIT} PROPERTIES
    PROJECT_LABEL "Python Bindings - _${KIT}"
    PREFIX ""
    # Most shared libraries go into the root build dir; not so for _opensim.
    ARCHIVE_OUTPUT_DIRECTORY "${PYTHON_PACKAGE_BINARY_DIR}/opensim"
    LIBRARY_OUTPUT_DIRECTORY "${PYTHON_PACKAGE_BINARY_DIR}/opensim"
    RUNTIME_OUTPUT_DIRECTORY "${PYTHON_PACKAGE_BINARY_DIR}/opensim"
)
if(WIN32)
    set_target_properties(_${KIT} PROPERTIES SUFFIX ".pyd")
elseif(APPLE)
    # Defaults to .dylib; change to .so.
    set_target_properties(_${KIT} PROPERTIES SUFFIX ".so")
endif()

# Create complete python package in the build tree.
# -------------------------------------------------
# Configure setup.py
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in
    "${PYTHON_PACKAGE_BINARY_DIR}/setup.py" @ONLY)

# Copy source tree files into the build directory.
file(COPY __init__.py DESTINATION "${PYTHON_PACKAGE_BINARY_DIR}/opensim")

foreach(test_file tests/storage.sto "${OPENSIM_SHARED_TEST_FILES_DIR}/arm26.osim"
        "${OPENSIM_SHARED_TEST_FILES_DIR}/gait10dof18musc_subject01.osim")
    file(COPY "${test_file}"
         DESTINATION "${PYTHON_PACKAGE_BINARY_DIR}/opensim/tests")
endforeach()

# Test.
# -----
add_test(NAME python WORKING_DIRECTORY "${PYTHON_PACKAGE_BINARY_DIR}"
    # This command runs all the python tests in the tests directory from the
    # source tree. It's important to run the tests in the source tree so that
    # one can edit the tests and immediately re-run the tests without any
    # intermediate file copying.
    COMMAND "${PYTHON_EXECUTABLE}" -m unittest discover
                "${CMAKE_CURRENT_SOURCE_DIR}/tests" --verbose
    )
if(WIN32)
    set_tests_properties(python PROPERTIES ENVIRONMENT
        "PATH=${BUILD_BINARY_DIR}/${CMAKE_CFG_INTDIR}\\;$ENV{PATH}")
endif()

# Install python package.
# -----------------------
install(FILES "${PYTHON_PACKAGE_BINARY_DIR}/setup.py" DESTINATION sdk/python)
install(DIRECTORY "${PYTHON_PACKAGE_BINARY_DIR}/opensim" DESTINATION sdk/python)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests"
        DESTINATION sdk/python/opensim)

endif(BUILD_PYTHON_WRAPPING)
