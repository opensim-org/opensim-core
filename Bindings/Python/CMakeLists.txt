set(KIT opensim)

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
# Location of the opensim python package in the installation (relative).
set(OPENSIM_PYTHON_INSTALL_DIR sdk/python)


# Generate source code for wrapper using SWIG.
# ============================================
set(swig_output_cxx_file_fullname
    ${CMAKE_CURRENT_BINARY_DIR}/pyOpenSim_wrap.cxx)
set(swig_output_header_file_fullname
    ${CMAKE_CURRENT_BINARY_DIR}/pyOpenSim_wrap.h)
set(swig_interface_file_fullname
    ${CMAKE_CURRENT_SOURCE_DIR}/swig/pyWrapOpenSim.i)

# Using a custom command / custom target pair so that SWIG is only run when
# it's out of date. Previously, we used a custom target only. But, custom
# targets are always out of date.
# TODO can use `swig -M` to detect file dependencies for CMake.
add_custom_command(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/opensim.py"
        ${swig_output_cxx_file_fullname} ${swig_output_header_file_fullname}
    COMMAND ${SWIG_EXECUTABLE} -v -c++ -python
        #-debug-tmused # Which typemaps were used?
        -I${OpenSim_SOURCE_DIR}
        -I${OpenSim_SOURCE_DIR}/Bindings/
        -I${Simbody_INCLUDE_DIR}
        -o ${swig_output_cxx_file_fullname}
        -outdir "${CMAKE_CURRENT_BINARY_DIR}"
        ${swig_interface_file_fullname}
    DEPENDS ${swig_interface_file_fullname}
        "${OpenSim_SOURCE_DIR}/Bindings/opensim.i"
        "${OpenSim_SOURCE_DIR}/Bindings/OpenSimHeaders.h"
    COMMENT "Generating python bindings source code with SWIG."
    )


# Compile python wrapper files into _opensim.
# ===========================================
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

    set_source_files_properties("${swig_output_cxx_file_fullname}"
        PROPERTIES COMPILE_FLAGS "${_COMPILE_FLAGS}")
endif()
# TODO disable optimization on Windows.

include_directories(${OpenSim_SOURCE_DIR} 
                    ${OpenSim_SOURCE_DIR}/Vendors 
                    ${PYTHON_INCLUDE_PATH}
                    )


add_library(_${KIT} SHARED ${SOURCE_FILES} ${INCLUDE_FILES})

target_link_libraries(_${KIT} osimTools ${PYTHON_LIBRARIES})

# Set target properties for various platforms.
# --------------------------------------------
# Resulting library must be named _opensim.so on Unix, _opensim.pyd on Windows.
set_target_properties(_${KIT} PROPERTIES
    PROJECT_LABEL "Python - _${KIT}"
    FOLDER "Bindings"
    PREFIX ""
)
if(WIN32)
    set_target_properties(_${KIT} PROPERTIES SUFFIX ".pyd")
elseif(APPLE)
    # Defaults to .dylib; change to .so.
    set_target_properties(_${KIT} PROPERTIES SUFFIX ".so")
endif()

if(${OPENSIM_USE_INSTALL_RPATH})
    set_target_properties(_${KIT} PROPERTIES
        INSTALL_RPATH "${CMAKE_INSTALL_FULL_LIBDIR}"
        )
endif()
    
# Copy _opensim to the python package in the build directory.
# -----------------------------------------------------------
add_custom_command(TARGET _${KIT} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:_${KIT}>"
        "${OPENSIM_PYTHON_BINARY_DIR}/opensim/$<TARGET_FILE_NAME:_${KIT}>"
    COMMENT "Copying _opensim library to python package in build directory."
    VERBATIM
    )


# Copy files to create complete package in the build tree.
# ========================================================
# This allows us to test the python package with ctest.

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
    list(APPEND PYTHON_PACKAGE_FILES "${binary_dest_full_path}")

    # Python package in the installation.
    # -----------------------------------
    install(FILES "${source_full_path}"
        DESTINATION "${OPENSIM_PYTHON_INSTALL_DIR}/${relative_dest_dir}")

endmacro()

# Copy the generated opensim.py file to the per-config python package dir.
OpenSimPutFileInPythonPackage("${CMAKE_CURRENT_BINARY_DIR}/opensim.py" opensim)

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
add_custom_target(PythonBindings ALL
    DEPENDS "${OPENSIM_PYTHON_BINARY_DIR}/opensim/opensim.py"
        ${swig_output_cxx_file_fullname}
        ${swig_output_header_file_fullname}
        ${PYTHON_PACKAGE_FILES}
        )
# Require the _opensim library to be built.
add_dependencies(PythonBindings _${KIT})

set_target_properties(PythonBindings PROPERTIES
    PROJECT_LABEL "Python - assemble package"
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
    set_tests_properties(python PROPERTIES ENVIRONMENT
        "PATH=${CMAKE_BINARY_DIR}/$<CONFIG>")
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


# Install python package.
# =======================
# Most of the files are installed via OpenSimPutFileInPythonPackage.
# It's important that we use install(TARGETS) to install the _opensim library
# because this causes CMake to remove the build-tree RPATH from the library
# (which is set temporarily for libraries in the build tree).
install(TARGETS _${KIT} DESTINATION "${OPENSIM_PYTHON_INSTALL_DIR}/opensim")
# Install the test scripts.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests"
        DESTINATION "${OPENSIM_PYTHON_INSTALL_DIR}/opensim")
