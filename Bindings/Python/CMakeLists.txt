
# Find python.
# ============

# PythonInterp is supposed to come before PythonLibs.
find_package(PythonInterp 2.7 REQUIRED)
find_package(PythonLibs 2.7 REQUIRED)

# Location of the opensim python package in the build directory, for testing.
if(MSVC OR XCODE)
    # Multi-configuration generators like MSVC and XCODE use one build tree for
    # all configurations.
    set(OPENSIM_PYTHON_BINARY_DIR
        "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_CFG_INTDIR}")
else()
    set(OPENSIM_PYTHON_BINARY_DIR
        "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_BUILD_TYPE}")
endif()

# Helper macros/functions.
# ========================

# Helper function to for copying files into the python package.
macro(OpenSimPutFileInPythonPackage source_full_path relative_dest_dir)

    # Python package in the build tree.
    # ---------------------------------
    get_filename_component(file_name "${source_full_path}" NAME)
    set(binary_dest_full_path
        "${OPENSIM_PYTHON_BINARY_DIR}/${relative_dest_dir}/${file_name}")
    add_custom_command(
        DEPENDS "${source_full_path}"
        OUTPUT "${binary_dest_full_path}"
        COMMAND ${CMAKE_COMMAND} -E copy "${source_full_path}"
                                         "${binary_dest_full_path}"
        COMMENT "Copying ${source_full_path} to python package in build directory"
        VERBATIM
        )
    # This list is used to specify dependencies for the PythonBindings target.
    list(APPEND OPENSIM_PYTHON_PACKAGE_FILES "${binary_dest_full_path}")

    # Python package in the installation.
    # -----------------------------------
    install(FILES "${source_full_path}"
        DESTINATION "${OPENSIM_INSTALL_PYTHONDIR}/${relative_dest_dir}")

endmacro()

# Generates source code for python module and then compiles it.
# Here are the arguments:
# MODULE: Name of python module. The module is build with the interface file
#       named ${MODULE}_python.i.
# DEPENDS: Full paths of files that the SWIG command depends on. TODO REMOVE.
macro(OpenSimAddSwigPythonModule)
    # Parse arguments.
    # ----------------
    # http://www.cmake.org/cmake/help/v2.8.9/cmake.html#module:CMakeParseArguments
    set(options)
    set(oneValueArgs MODULE)
    set(multiValueArgs DEPENDS)
    cmake_parse_arguments(
        OSIMSWIGPY "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # Generate source code for wrapper using SWIG.
    # --------------------------------------------
    set(_output_file_prefix
        "${CMAKE_CURRENT_BINARY_DIR}/python_${OSIMSWIGPY_MODULE}_wrap")
    set(_output_cxx_file "${_output_file_prefix}.cxx")
    set(_output_header_file "${_output_file_prefix}.h")
    set(_interface_file
        "${CMAKE_CURRENT_SOURCE_DIR}/swig/${OSIMSWIGPY_MODULE}_python.i")

    add_custom_command(
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${OSIMSWIGPY_MODULE}.py"
            ${_output_cxx_file} ${_output_header_file}
        COMMAND ${SWIG_EXECUTABLE} -v -c++ -python
            #-debug-tmused # Which typemaps were used?
            -I${OpenSim_SOURCE_DIR}
            -I${OpenSim_SOURCE_DIR}/Bindings/
            -I${Simbody_INCLUDE_DIR}
            -o ${_output_cxx_file}
            -outdir "${CMAKE_CURRENT_BINARY_DIR}"
            ${_interface_file}
        DEPENDS ${_interface_file}
            ${OSIMSWIGPY_DEPENDS}
            COMMENT "Generating python bindings source code with SWIG; module ${OSIMSWIGPY_MODULE}."
        )

    # Compile python wrapper files into a library.
    # --------------------------------------------
    set(_libname _${OSIMSWIGPY_MODULE}) 

    # Used for specifying dependencies for PythonBindings.
    list(APPEND OPENSIM_PYTHON_PACKAGE_LIBRARY_TARGETS ${_libname})

    if(${CMAKE_CXX_COMPILER_ID} MATCHES "GNU" OR
            ${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
        # Turn off optimization for SWIG wrapper code. Optimization slows down
        # compiling and also requires lots of memory. Also, there's not much to gain
        # from an optimized wrapper file. 
        # Note that the last optimization flag is what counts for GCC. So an -O0
        # later on the command line overrides a previous -O2.
        set(_COMPILE_FLAGS "-O0")
    
        set_source_files_properties("${_output_cxx_file}"
            PROPERTIES COMPILE_FLAGS "${_COMPILE_FLAGS}")
    endif()
    # TODO disable optimization on Windows.
    
    include_directories(${OpenSim_SOURCE_DIR} 
                        ${OpenSim_SOURCE_DIR}/Vendors 
                        ${PYTHON_INCLUDE_PATH}
                        )
    

    add_library(${_libname} SHARED ${_output_cxx_file} ${_output_header_file})
    
    target_link_libraries(${_libname} osimTools ${PYTHON_LIBRARIES})
    
    # Set target properties for various platforms.
    # --------------------------------------------
    # Resulting library must be named _opensim.so on Unix, _opensim.pyd on Windows.
    set_target_properties(${_libname} PROPERTIES
        PROJECT_LABEL "Python - ${_libname}"
        FOLDER "Bindings"
        PREFIX ""
    )
    if(WIN32)
        set_target_properties(${_libname} PROPERTIES SUFFIX ".pyd")
    elseif(APPLE)
        # Defaults to .dylib; change to .so.
        set_target_properties(${_libname} PROPERTIES SUFFIX ".so")
    endif()
    
    if(${OPENSIM_USE_INSTALL_RPATH})
        set_target_properties(${_libname} PROPERTIES
            INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
            )
    endif()

    # Copy files into the build tree python package.
    # ----------------------------------------------
    # Copy the library file.
    add_custom_command(TARGET ${_libname} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:${_libname}>"
            "${OPENSIM_PYTHON_BINARY_DIR}/opensim/$<TARGET_FILE_NAME:${_libname}>"
        COMMENT "Copying ${_libname} library to python package in build directory."
        VERBATIM
        )

    # Copy the generated .py file to the per-config python package dir.
    OpenSimPutFileInPythonPackage(
        "${CMAKE_CURRENT_BINARY_DIR}/${OSIMSWIGPY_MODULE}.py" opensim)

    # Install the library.
    # --------------------
    # It's important that we use install(TARGETS) 
    # because this causes CMake to remove the build-tree RPATH from the library
    # (which is set temporarily for libraries in the build tree).
    install(TARGETS ${_libname} DESTINATION "${OPENSIM_INSTALL_PYTHONDIR}/opensim")

endmacro()


# Build python modules (generate binding source code and compile it).
# ===================================================================
OpenSimAddSwigPythonModule(MODULE simbody
    DEPENDS "${OpenSim_SOURCE_DIR}/Bindings/OpenSimHeaders.h"
            "swig/preliminaries_python.i"
            "${OpenSim_SOURCE_DIR}/Bindings/simbody.i"
    )
#OpenSimAddSwigPythonModule(MODULE opensim
#    DEPENDS "${OpenSim_SOURCE_DIR}/Bindings/OpenSimHeaders.h"
#            "swig/preliminaries_python.i"
#            "${OpenSim_SOURCE_DIR}/Bindings/preliminaries.i"
#            "${OpenSim_SOURCE_DIR}/Bindings/simbody.i"
#            "${OpenSim_SOURCE_DIR}/Bindings/opensim.i"
#    )


# Copy files to create complete package in the build tree.
# ========================================================
# This allows us to test the python package with ctest.
# Note: some of the commands to do this copying (for the swig-generated py
# files) appear above.

# Configure setup.py
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in
    "${CMAKE_CURRENT_BINARY_DIR}/setup.py" @ONLY)

# Copy the configured setup.py for each build configuration.
OpenSimPutFileInPythonPackage("${CMAKE_CURRENT_BINARY_DIR}/setup.py" ".")

# __init__.py.
OpenSimPutFileInPythonPackage("${CMAKE_CURRENT_SOURCE_DIR}/__init__.py" opensim)

# Test files. If you require more test resource files, list them here.
foreach(test_file
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/storage.sto"
        "${OPENSIM_SHARED_TEST_FILES_DIR}/arm26.osim"
        "${OPENSIM_SHARED_TEST_FILES_DIR}/gait10dof18musc_subject01.osim"
        )

    OpenSimPutFileInPythonPackage("${test_file}" opensim/tests)

endforeach()


# Umbrella target for assembling the python bindings in the build tree.
# =====================================================================
# This command must come *after* all calls to OpenSimPutFileInPythonPackage, as
# that macro assembles the OPENSIM_PYTHON_PACKAGE_FILES list.
add_custom_target(PythonBindings ALL DEPENDS ${OPENSIM_PYTHON_PACKAGE_FILES})

# Require the _opensim library to be built.
add_dependencies(PythonBindings ${OPENSIM_PYTHON_PACKAGE_LIBRARY_TARGETS})

set_target_properties(PythonBindings PROPERTIES
    PROJECT_LABEL "Python - umbrella target"
    FOLDER "Bindings")


# Test.
# =====
# This test runs all the python tests in the tests directory from the
# source tree. It's important to run the tests in the source tree so that
# one can edit the tests and immediately re-run the tests without any
# intermediate file copying.
# TODO when minimum cmake is v3.0.0 and add_test WORKING_DIRECTORY supports
# generator expressions, use the following instead:
#   add_test(NAME python
#       WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>"
#       COMMAND "${PYTHON_EXECUTABLE}" -m unittest discover
#                   --start-directory "${CMAKE_CURRENT_SOURCE_DIR}/tests"
#                   --verbose
#       )
if(MSVC OR XCODE)
    foreach(cfg ${CMAKE_CONFIGURATION_TYPES})
        add_test(NAME python_${cfg}
            WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${cfg}"
            COMMAND "${PYTHON_EXECUTABLE}" -m unittest discover
                --start-directory "${CMAKE_CURRENT_SOURCE_DIR}/tests"
                --verbose
            CONFIGURATIONS ${cfg}
            )
    endforeach()
else()
    add_test(NAME python
        WORKING_DIRECTORY "${OPENSIM_PYTHON_BINARY_DIR}"
        COMMAND "${PYTHON_EXECUTABLE}" -m unittest discover
            --start-directory "${CMAKE_CURRENT_SOURCE_DIR}/tests"
            --verbose
        )
endif()

if(WIN32)
    # On Windows, CMake cannot use RPATH to hard code the location of libraries
    # in the binary directory (DLL's don't have RPATH), so we must set PATH to
    # find the right libraries. The location of the libraries depends on the
    # build configuration, which is filled in for `$<CONFIG>`. We also don't
    # want to accidentally use a different OpenSim build/installation somewhere
    # on the machine.
    # TODO use the commented-out version when moving to CMake 3.0.
    #set_tests_properties(python PROPERTIES ENVIRONMENT
    #    "PATH=${CMAKE_BINARY_DIR}/$<CONFIG>")
    foreach(cfg ${CMAKE_CONFIGURATION_TYPES})
        set_tests_properties(python_${cfg} PROPERTIES ENVIRONMENT
            "PATH=${CMAKE_BINARY_DIR}/${cfg}")
    endforeach()
endif()

# Allow MSVC users to run only the python tests directly from the MSVC GUI.
# The python tests are run from RUN_TESTS, so no need to run this target as
# part of `BUILD_ALL` (e.g, in MSVC). Might need to set
# EXCLUDE_FROM_DEFAULT_BUILD to achieve this?
add_custom_target(RunPythonTests
    COMMAND ${CMAKE_CTEST_COMMAND} --tests-regex python
                                   ${OPENSIM_TEST_BUILD_CONFIG}
                                   --extra-verbose
    PROJECT_LABEL "Python - run tests"
    FOLDER "Bindings"
    )

add_dependencies(RunPythonTests PythonBindings)


# Install python package.
# =======================
# Most of the files are installed via the above functions/macros.
# Install the test scripts.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests"
        DESTINATION "${OPENSIM_INSTALL_PYTHONDIR}/opensim")
