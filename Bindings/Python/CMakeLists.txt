
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


# Avoid excessive compiler warnings.
# ==================================
# We set these COMPILE_OPTIONS in the root CMakeLists.txt.
set_directory_properties(PROPERTIES COMPILE_OPTIONS "")


# Helper macros.
# ==============

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
# DEPENDS: Names of other python modules on which this module depends. 
macro(OpenSimAddPythonModule)
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
        "${CMAKE_CURRENT_SOURCE_DIR}/swig/python_${OSIMSWIGPY_MODULE}.i")

    # We run swig once to get dependencies and then again to actually generate
    # the wrappers. This variable holds the parts of the swig command that
    # are shared between both invocations.
    set(_swig_common_args -c++ -python -py3
            -I${OpenSim_SOURCE_DIR}
            -I${OpenSim_SOURCE_DIR}/Bindings/
            -I${Simbody_INCLUDE_DIR}
            -v
            ${SWIG_FLAGS}
            ${_interface_file}
            )

    # Assemble dependencies. This macro runs a command during CMake's
    # configure step and fills the first argument with a list of the
    # dependencies.
    OpenSimFindSwigFileDependencies(_${OSIMSWIGPY_MODULE}_dependencies
        "${OSIMSWIGPY_MODULE} (Python)" "${_swig_common_args}")

    # Run swig.
    add_custom_command(
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${OSIMSWIGPY_MODULE}.py"
            ${_output_cxx_file} ${_output_header_file}
        COMMAND ${SWIG_EXECUTABLE}
            #-debug-tmused # Which typemaps were used?
            -v # verbose
            -o ${_output_cxx_file}
            ${SWIG_DOXYGEN_STRING}
            -outdir "${CMAKE_CURRENT_BINARY_DIR}"
            ${_swig_common_args}
        DEPENDS ${_${OSIMSWIGPY_MODULE}_dependencies}
            COMMENT "Generating python bindings source code with SWIG: ${OSIMSWIGPY_MODULE} module."
        )

    # Compile python wrapper files into a library.
    # --------------------------------------------
    set(_libname _${OSIMSWIGPY_MODULE}) 

    # Used for specifying dependencies for PythonBindings.
    list(APPEND OPENSIM_PYTHON_PACKAGE_LIBRARY_TARGETS ${_libname})

    # We purposefully wrap deprecated functions, so no need to see such
    # warnings.
    if(${CMAKE_CXX_COMPILER_ID} MATCHES "GNU" OR
            ${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
        # Turn off optimization for SWIG wrapper code. Optimization slows down
        # compiling and also requires lots of memory. Also, there's not much to
        # gain from an optimized wrapper file. 
        # Note that the last optimization flag is what counts for GCC. So an -O0
        # later on the command line overrides a previous -O2.
        set(_COMPILE_FLAGS "-O0 -Wno-deprecated-declarations")
    elseif(${CMAKE_CXX_COMPILER_ID} MATCHES "MSVC")
        # TODO disable optimization on Windows.
        # Don't warn about:
        # 4996: deprecated functions.
        # 4114: "const const T"
        set(_COMPILE_FLAGS "/wd4996 /wd4114")
    endif()
    if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang" AND
            NOT (${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS "10"))
        # SWIG uses the register keyword, which is deprecated in C++17.
        set(_COMPILE_FLAGS "${_COMPILE_FLAGS} -Wno-deprecated-register")
    endif()

    set_source_files_properties("${_output_cxx_file}"
        PROPERTIES COMPILE_FLAGS "${_COMPILE_FLAGS}")

    add_library(${_libname} SHARED ${_output_cxx_file} ${_output_header_file})

    target_include_directories(${_libname} PRIVATE
            "${Python3_INCLUDE_DIRS}"
            "${Python3_NumPy_INCLUDE_DIRS}")
    
    # On macOS, the python interpreter might link to the python library 
    # statically, in which case we do not want to link dynamically to the
    # python library. This situation occurs with Anaconda's python.
    # In this situation, we must add a linker flag to mark all undefined
    # symbols as having to be looked up at runtime, and we ignore
    # PYTHON_LIBRARIES.
    # https://github.com/opensim-org/opensim-core/issues/2771
    # https://stackoverflow.com/questions/25421479/clang-and-undefined-symbols-when-building-a-library
    # https://github.com/lisitsyn/shogun/commit/961f6109c2a8abab4941bcdde1a19dfe70c440f1
    # https://github.com/shogun-toolbox/shogun/issues/4068
    execute_process(COMMAND "${Python3_EXECUTABLE}" -c
        "import sysconfig; print(sysconfig.get_config_var('LDSHARED'))"
        OUTPUT_VARIABLE PYTHON_LDSHARED
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    if("${PYTHON_LDSHARED}" MATCHES "dynamic_lookup")
        set_target_properties(${_libname} PROPERTIES LINK_FLAGS
            "-undefined dynamic_lookup")
    else()
        target_link_libraries(${_libname} ${Python3_LIBRARIES})
    endif()

    target_link_libraries(${_libname} osimTools osimExampleComponents osimMoco)
    
    # Set target properties for various platforms.
    # --------------------------------------------
    # Resulting library must be named with .so on Unix, .pyd on Windows.
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
    
    # RPATH: We always set a relative RPATH but only use an absolute RPATH if
    # the python package is not standalone, as the libraries are not copied
    # into the python package. If Simbody libraries are not installed in the
    # same folder as the OpenSim libraries, then we must also provide the path
    # to the Simbody libraries within OpenSim's installation.
    OpenSimAddInstallRPATHSelf(TARGET ${_libname})
    OpenSimAddInstallRPATH(TARGET ${_libname} LOADER 
        FROM "${OPENSIM_INSTALL_PYTHONDIR}/opensim"
        TO "${CMAKE_INSTALL_LIBDIR}")
    if(NOT OPENSIM_PYTHON_STANDALONE)
        OpenSimAddInstallRPATHAbsolute(TARGET ${_libname}
            TO "${CMAKE_INSTALL_LIBDIR}")
    endif()
    OpenSimAddInstallRPATHSimbody(TARGET ${_libname} LOADER
        FROM "${CMAKE_INSTALL_LIBDIR}/opensim")
    if(NOT OPENSIM_PYTHON_STANDALONE)
        OpenSimAddInstallRPATHSimbody(TARGET ${_libname} ABSOLUTE)
    endif()

    # Copy files into the build tree python package.
    # ----------------------------------------------
    add_custom_command(TARGET ${_libname} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:${_libname}>"
            "${OPENSIM_PYTHON_BINARY_DIR}/opensim/$<TARGET_FILE_NAME:${_libname}>"
        COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/${OSIMSWIGPY_MODULE}.py"
            "${OPENSIM_PYTHON_BINARY_DIR}/opensim/${OSIMSWIGPY_MODULE}.py"
        COMMENT "Copying ${OSIMSWIGPY_MODULE}.py and ${_libname} to python package in build directory."
        VERBATIM
        )

    # Install the python module and compiled library.
    # -----------------------------------------------
    # For the shared library, it's important that we use install(TARGETS)
    # because this causes CMake to remove the build-tree RPATH from the library
    # (which is set temporarily for libraries in the build tree).
    install(TARGETS ${_libname} DESTINATION "${OPENSIM_INSTALL_PYTHONDIR}/opensim")

    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${OSIMSWIGPY_MODULE}.py"
        DESTINATION "${OPENSIM_INSTALL_PYTHONDIR}/opensim")

endmacro()


# Build python modules (generate binding source code and compile it).
# ===================================================================
OpenSimAddPythonModule(MODULE simbody)
OpenSimAddPythonModule(MODULE common)
OpenSimAddPythonModule(MODULE simulation)
OpenSimAddPythonModule(MODULE actuators)
OpenSimAddPythonModule(MODULE analyses)
OpenSimAddPythonModule(MODULE tools)
OpenSimAddPythonModule(MODULE examplecomponents)
OpenSimAddPythonModule(MODULE moco)


# Copy files to create complete package in the build tree.
# ========================================================
# This allows us to test the python package with ctest.
# Note: some of the commands to do this copying (for the swig-generated py
# files) appear above.

# Configure version.py.
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/version.py.in
    "${CMAKE_CURRENT_BINARY_DIR}/version.py" @ONLY)

# Copy the configured version.py for each build configuration.
OpenSimPutFileInPythonPackage("${CMAKE_CURRENT_BINARY_DIR}/version.py" opensim)

# Copy setup.py for each build configuration.
OpenSimPutFileInPythonPackage("${CMAKE_CURRENT_SOURCE_DIR}/setup.py" ".")

# __init__.py.
OpenSimPutFileInPythonPackage("${CMAKE_CURRENT_SOURCE_DIR}/__init__.py" opensim)

OpenSimPutFileInPythonPackage("${CMAKE_CURRENT_SOURCE_DIR}/report.py" opensim)
# for Python 3.8+/Windows we use a special script
# setup_win_python38.py to set additional dll directory in __init__.py on client machine
if (WIN32)
    OpenSimPutFileInPythonPackage("${CMAKE_CURRENT_SOURCE_DIR}/setup_win_python38.py" ".")
endif()

# Test files. If you require more test resource files, list them here.
foreach(test_file
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/storage.sto"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/gait2392_setup_forward_empty_model.xml"
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/gait2392_cmc_actuators_empty_model.xml"
        "${OPENSIM_SHARED_TEST_FILES_DIR}/arm26.osim"
        "${OPENSIM_SHARED_TEST_FILES_DIR}/gait10dof18musc_subject01.osim"
        "${CMAKE_SOURCE_DIR}/OpenSim/Sandbox/futureOrientationInverseKinematics.trc"
        "${CMAKE_SOURCE_DIR}/OpenSim/Common/Test/dataWithNaNsOfDifferentCases.trc"
        "${CMAKE_SOURCE_DIR}/Applications/Analyze/test/subject02_grf_HiFreq.mot"
        "${CMAKE_SOURCE_DIR}/Applications/IK/test/std_subject01_walk1_ik.mot"
        "${CMAKE_SOURCE_DIR}/OpenSim/Tests/shared/walking2.c3d"
        "${CMAKE_SOURCE_DIR}/OpenSim/Tests/shared/walking5.c3d"
        "${CMAKE_SOURCE_DIR}/OpenSim/Tests/shared/orientation_quats.sto"
        "${CMAKE_SOURCE_DIR}/OpenSim/Tests/shared/calibrated_model_imu.osim"
        )

    OpenSimPutFileInPythonPackage("${test_file}" opensim/tests)

endforeach()


# Umbrella target for assembling the python bindings in the build tree.
# =====================================================================
# This command must come *after* all calls to OpenSimPutFileInPythonPackage, as
# that macro assembles the OPENSIM_PYTHON_PACKAGE_FILES list.
add_custom_target(PythonBindings ALL DEPENDS ${OPENSIM_PYTHON_PACKAGE_FILES})

# Require the libraries to be built.
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
# It so happens that ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG> is the same as
# ${OPENSIM_PYTHON_BINARY_DIR}, but the former avoids an `if(MSVC OR XCODE)`.
add_test(NAME python_tests
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>"
    COMMAND "${Python3_EXECUTABLE}" -m unittest discover
                --start-directory "${CMAKE_CURRENT_SOURCE_DIR}/tests"
                --verbose
    )

# List the examples we want to test. Some examples might run too long for
# testing, or might require additional libraries.
# Leave off the .py extension; here, we are listing module names.
set(PYTHON_EXAMPLES_UNITTEST_ARGS
        build_simple_arm_model
        extend_OpenSim_Vec3_class
        posthoc_StatesTrajectory_example
        wiring_inputs_and_outputs_with_TableReporter
        )
if(OPENSIM_WITH_CASADI)
    list(APPEND PYTHON_EXAMPLES_UNITTEST_ARGS
            Moco.exampleKinematicConstraints
            Moco.exampleSlidingMass
            )
endif()
if(OPENSIM_WITH_TROPTER)
    list(APPEND PYTHON_EXAMPLES_UNITTEST_ARGS
            Moco.exampleOptimizeMass
            Moco.examplePredictAndTrack
            )
endif()
# Similar as above, but for the example files. These files aren't named as
# test_*.py, so we must specify a more general search pattern.
add_test(NAME python_examples
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>"
    COMMAND "${Python3_EXECUTABLE}" -m unittest
                ${PYTHON_EXAMPLES_UNITTEST_ARGS}
                --verbose
    )
set_tests_properties(python_examples PROPERTIES
        ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/examples")

foreach(folder tests examples)
    set_property(TEST python_${folder} APPEND PROPERTY ENVIRONMENT
        "OPENSIM_USE_VISUALIZER=0")
endforeach()

if(WIN32)
    # On Windows, CMake cannot use RPATH to hard code the location of libraries
    # in the binary directory (DLL's don't have RPATH), so we must set PATH to
    # find the right libraries. The location of the libraries depends on the
    # build configuration, which is filled in for `$<CONFIG>`. We also don't
    # want to accidentally use a different OpenSim build/installation somewhere
    # on the machine.
    foreach(folder tests examples)
        set_property(TEST python_${folder} APPEND PROPERTY ENVIRONMENT
            "PATH=${CMAKE_BINARY_DIR}/$<CONFIG>")
    endforeach()
endif()

# Allow MSVC users to run only the python tests directly from the MSVC GUI.
# The python tests are run from RUN_TESTS, so no need to run this target as
# part of `BUILD_ALL` (e.g, in MSVC). Might need to set
# EXCLUDE_FROM_DEFAULT_BUILD to achieve this?
add_custom_target(RunPythonTests
    COMMAND ${CMAKE_CTEST_COMMAND} --tests-regex python
                                   ${OPENSIM_TEST_BUILD_CONFIG}
                                   --extra-verbose)
set_target_properties(RunPythonTests PROPERTIES
    PROJECT_LABEL "Python - run tests"
    FOLDER "Bindings")

add_dependencies(RunPythonTests PythonBindings)


# Install python package.
# =======================
# Most of the files are installed via the above macros.
# Install the test scripts.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests"
        DESTINATION "${OPENSIM_INSTALL_PYTHONDIR}/opensim")

# Install example files (not installed next to the python package).
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/examples/"
        DESTINATION "${OPENSIM_INSTALL_PYTHONEXDIR}")
install(DIRECTORY ../Java/Matlab/OpenSenseExample
        DESTINATION "${OPENSIM_INSTALL_PYTHONEXDIR}"
        # Exclude Matlab m files
        PATTERN "*.m" EXCLUDE)
install(DIRECTORY ../Java/Matlab/examples/Moco/exampleEMGTracking
        DESTINATION "${OPENSIM_INSTALL_PYTHONEXDIR}/Moco"
        # Exclude Matlab m files
        PATTERN "*.m" EXCLUDE)
install(DIRECTORY ../Java/Matlab/examples/Moco/exampleSquatToStand
        DESTINATION "${OPENSIM_INSTALL_PYTHONEXDIR}/Moco"
        # Exclude Matlab m files
        PATTERN "*.m" EXCLUDE)
install(DIRECTORY OpenSenseExample
        DESTINATION "${OPENSIM_INSTALL_PYTHONEXDIR}")
