# PythonInterp is supposed to come before PythonLibs.
if(${MOCO_PYTHON_VERSION} STREQUAL "2")
    set(required_python_version 2.7)
elseif(${MOCO_PYTHON_VERSION} STREQUAL "3")
    set(required_python_version 3)
else()
    message(FATAL_ERROR "MOCO_PYTHON_VERSION should be '2' or '3'.")
endif()
find_package(PythonInterp ${required_python_version} REQUIRED)

set(WINLIBSUFFIX "pyd")
set(UNIXLIBSUFFIX "so")

# We update the python install dir to include the python version,
# now that we know it. We replace the token "VERSION" with the actual python
# version.
set(python_version_majmin "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")
string(REPLACE "VERSION" "${python_version_majmin}"
    MOCO_INSTALL_PYTHONDIR "${MOCO_INSTALL_PYTHONDIR}")
# This must be done before adding the OpenSim libraries, since
# MOCO_INSTALL_PYTHONDIR is used in OpenSimAddLibrary (in
# OpenSimMacros.cmake).

if(APPLE AND ${MOCO_PYTHON_VERSION} STREQUAL "2")
    # If you have Homebrew's Python2, then by default, PythonInterp finds
    # Apple's Python, but PythonLibs finds Homebrew's Python, causing
    # runtime crashes. This also occurs if one has Anaconda Python.
    # So we use the python-config executable to get the
    # correct library and include directory.
    # https://github.com/Homebrew/legacy-homebrew/issues/25118
    execute_process(COMMAND "${PYTHON_EXECUTABLE}-config" --prefix
        OUTPUT_VARIABLE python_prefix
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    string(CONCAT pyinc_desc
        "Location of Python header files, to compile bindings. "
        "Must be consistent with PYTHON_EXECUTABLE.")
    set(PYTHON_INCLUDE_DIR
        "${python_prefix}/include/python${python_version_majmin}/"
        CACHE PATH "${pyinc_desc}")

    string(CONCAT pylib_desc
        "Location of Python library, to compile bindings. "
        "Must be consistent with PYTHON_EXECUTABLE.")
    set(PYTHON_LIBRARY
        "${python_prefix}/lib/libpython${python_version_majmin}.dylib"
        CACHE FILEPATH "${pylib_desc}")
endif()
find_package(PythonLibs ${required_python_version} REQUIRED)
find_package(NumPy REQUIRED)

# Location of the opensim python package in the build directory, for testing.
if(MSVC OR XCODE)
    # Multi-configuration generators like MSVC and XCODE use one build tree for
    # all configurations.
    set(MOCO_PYTHON_BINARY_DIR
        "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_CFG_INTDIR}")
else()
    set(MOCO_PYTHON_BINARY_DIR
        "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_BUILD_TYPE}")
endif()

# Avoid excessive compiler warnings.
# ==================================
# We set these COMPILE_OPTIONS in the root CMakeLists.txt.
set_directory_properties(PROPERTIES COMPILE_OPTIONS "")

# Helper macros.
# ==============

# Helper function to for copying files into the python package.
macro(MocoPutFileInPythonPackage source_full_path relative_dest_dir)

    # Python package in the build tree.
    # ---------------------------------
    get_filename_component(file_name "${source_full_path}" NAME)
    set(binary_dest_full_path
        "${MOCO_PYTHON_BINARY_DIR}/${relative_dest_dir}/${file_name}")
    add_custom_command(
        DEPENDS "${source_full_path}"
        OUTPUT "${binary_dest_full_path}"
        COMMAND ${CMAKE_COMMAND} -E copy "${source_full_path}"
                                         "${binary_dest_full_path}"
        COMMENT "Copying ${source_full_path} to python package in build directory"
        VERBATIM
        )
    # This list is used to specify dependencies for the MocoPythonBindings target.
    list(APPEND MOCO_PYTHON_PACKAGE_FILES "${binary_dest_full_path}")

    # Python package in the installation.
    # -----------------------------------
    install(FILES "${source_full_path}"
        DESTINATION "${MOCO_INSTALL_PYTHONDIR}/${relative_dest_dir}")

endmacro()

# Generates source code for python module and then compiles it.
# Here are the arguments:
# MODULE: Name of python module. The module is build with the interface file
#       named ${MODULE}_python.i.
# DEPENDS: Names of other python modules on which this module depends. 
macro(MocoAddPythonModule)
    # Parse arguments.
    # ----------------
    # http://www.cmake.org/cmake/help/v2.8.9/cmake.html#module:CMakeParseArguments
    set(options)
    set(oneValueArgs MODULE)
    set(multiValueArgs DEPENDS)
    cmake_parse_arguments(
        MOCOSWIGPY "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # Generate source code for wrapper using SWIG.
    # --------------------------------------------
    set(_output_file_prefix
        "${CMAKE_CURRENT_BINARY_DIR}/python_${MOCOSWIGPY_MODULE}_wrap")
    set(_output_cxx_file "${_output_file_prefix}.cxx")
    set(_output_header_file "${_output_file_prefix}.h")
    set(_interface_file
        "${CMAKE_CURRENT_SOURCE_DIR}/swig/python_${MOCOSWIGPY_MODULE}.i")

    # We run swig once to get dependencies and then again to actually generate
    # the wrappers. This variable holds the parts of the swig command that
    # are shared between both invocations.
    set(_swig_common_args -c++ -python
            -I${CMAKE_SOURCE_DIR}/Moco
            -I${CMAKE_SOURCE_DIR}/opensim-core
            -I${CMAKE_SOURCE_DIR}/opensim-core/Bindings
            -I${CMAKE_SOURCE_DIR}/opensim-core/Bindings/Python/swig
            -I${Simbody_INCLUDE_DIR}
            ${SWIG_FLAGS}
            ${_interface_file}
            )

    # Assemble dependencies. This macro runs a command during CMake's
    # configure step and fills the first argument with a list of the
    # dependencies.
    OpenSimFindSwigFileDependencies(_${MOCOSWIGPY_MODULE}_dependencies
        ${MOCOSWIGPY_MODULE} "${_swig_common_args}")

    # Run swig.
    add_custom_command(
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${MOCOSWIGPY_MODULE}.py"
            ${_output_cxx_file} ${_output_header_file}
        COMMAND ${SWIG_EXECUTABLE}
            #-debug-tmused # Which typemaps were used?
            -v # verbose
            -o ${_output_cxx_file}
            -outdir "${CMAKE_CURRENT_BINARY_DIR}"
            ${_swig_common_args}
        DEPENDS ${_${MOCOSWIGPY_MODULE}_dependencies}
            COMMENT "Generating python bindings source code with SWIG: ${MOCOSWIGPY_MODULE} module."
        )

    # Compile python wrapper files into a library.
    # --------------------------------------------
    set(_libname _${MOCOSWIGPY_MODULE}) 

    # Used for specifying dependencies for MocoPythonBindings.
    list(APPEND MOCO_PYTHON_PACKAGE_LIBRARY_TARGETS ${_libname})

    # We purposefully wrap deprecated functions, so no need to see such
    # warnings.
    if("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU" OR
            "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
        # Turn off optimization for SWIG wrapper code. Optimization slows down
        # compiling and also requires lots of memory. Also, there's not much to
        # gain from an optimized wrapper file. 
        # Note that the last optimization flag is what counts for GCC. So an -O0
        # later on the command line overrides a previous -O2.
        set(_COMPILE_FLAGS "-O0 -Wno-deprecated-declarations")
    elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "MSVC")
        # TODO disable optimization on Windows.
        # Don't warn about:
        # 4996: deprecated functions.
        # 4114: "const const T"
        set(_COMPILE_FLAGS "/bigobj /wd4996 /wd4114")
    endif()
    set_source_files_properties("${_output_cxx_file}"
            PROPERTIES COMPILE_FLAGS "${_COMPILE_FLAGS}")

    add_library(${_libname} SHARED ${_output_cxx_file} ${_output_header_file})

    target_include_directories(${_libname} SYSTEM PRIVATE
        # For Bindings/OpenSimHeaders_common.h, etc.:
        "${CMAKE_SOURCE_DIR}/opensim-core"
        "${PYTHON_INCLUDE_PATH}"
        "${PYTHON_NUMPY_INCLUDE_DIR}")
    
    target_link_libraries(${_libname} osimMoco ${PYTHON_LIBRARIES})
    
    # Set target properties for various platforms.
    # --------------------------------------------
    # Resulting library must be named with .so on Unix, .pyd on Windows.
    set_target_properties(${_libname} PROPERTIES
        PROJECT_LABEL "Python - ${_libname}"
        FOLDER "Moco/Bindings"
        PREFIX ""
    )
    if(WIN32)
        set_target_properties(${_libname} PROPERTIES SUFFIX ".${WINLIBSUFFIX}")
    elseif(APPLE)
        # Defaults to .dylib; change to .so.
        set_target_properties(${_libname} PROPERTIES SUFFIX ".${UNIXLIBSUFFIX}")
    endif()

    MocoAddInstallRPATHSelf(TARGET ${_libname})
    MocoAddInstallRPATH(TARGET ${_libname} LOADER
            FROM "${MOCO_INSTALL_PYTHONDIR}/opensim"
            TO "${CMAKE_INSTALL_LIBDIR}")
    # TODO we don't yet support a standalone python package.
    # if(NOT OPENSIM_PYTHON_STANDALONE)
        MocoAddInstallRPATHAbsolute(TARGET ${_libname}
                TO "${CMAKE_INSTALL_LIBDIR}")
    # endif()
    MocoAddInstallRPATHSimbody(TARGET ${_libname} LOADER
            FROM "${CMAKE_INSTALL_LIBDIR}/opensim")
    # if(NOT OPENSIM_PYTHON_STANDALONE)
        MocoAddInstallRPATHSimbody(TARGET ${_libname} ABSOLUTE)
    # endif()

    # Copy files into the build tree python package.
    # ----------------------------------------------
    # Copy the library file.
    add_custom_command(TARGET ${_libname} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:${_libname}>"
            "${MOCO_PYTHON_BINARY_DIR}/opensim/$<TARGET_FILE_NAME:${_libname}>"
        COMMENT "Copying ${_libname} library to python package in build directory."
        VERBATIM
        )

    # Copy the generated .py file to the per-config python package dir.
    MocoPutFileInPythonPackage(
        "${CMAKE_CURRENT_BINARY_DIR}/${MOCOSWIGPY_MODULE}.py" opensim)

    # Install the library.
    # --------------------
    # It's important that we use install(TARGETS) 
    # because this causes CMake to remove the build-tree RPATH from the library
    # (which is set temporarily for libraries in the build tree).
    install(TARGETS ${_libname} DESTINATION
        "${MOCO_INSTALL_PYTHONDIR}/opensim")

endmacro()


# Build python modules (generate binding source code and compile it).
# ===================================================================
MocoAddPythonModule(MODULE moco)


# Copy files to create complete package in the build tree.
# ========================================================
# This allows us to test the python package with ctest.
# Note: some of the commands to do this copying (for the swig-generated py
# files) appear above.

# Configure version.py.
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/version.py.in
    "${CMAKE_CURRENT_BINARY_DIR}/version.py" @ONLY)

# Copy modules from opensim's installed python package.
set(libsuffix ${UNIXLIBSUFFIX})
if(WIN32)
    set(libsuffix ${WINLIBSUFFIX})
endif()
foreach(module simbody common simulation actuators analyses tools
        examplecomponents)
    MocoPutFileInPythonPackage(
        "${OpenSim_PYTHON_PACKAGE_DIR}/opensim/_${module}.${libsuffix}"
        opensim)
    MocoPutFileInPythonPackage(
        "${OpenSim_PYTHON_PACKAGE_DIR}/opensim/${module}.py" opensim)
endforeach()

# Copy the configured version.py for each build configuration.
MocoPutFileInPythonPackage("${CMAKE_CURRENT_BINARY_DIR}/version.py" opensim)

# Copy setup.py for each build configuration.
MocoPutFileInPythonPackage("${CMAKE_CURRENT_SOURCE_DIR}/setup.py" ".")

# __init__.py.
MocoPutFileInPythonPackage("${CMAKE_CURRENT_SOURCE_DIR}/__init__.py" opensim)

MocoPutFileInPythonPackage("${CMAKE_CURRENT_SOURCE_DIR}/report.py" opensim)

# Test files. If you require more test resource files, list them here.
foreach(test_file
        # None yet.
        )
    MocoPutFileInPythonPackage("${test_file}" opensim/tests)
endforeach()


# Umbrella target for assembling the python bindings in the build tree.
# =====================================================================
# This command must come *after* all calls to MocoPutFileInPythonPackage, as
# that macro assembles the MOCO_PYTHON_PACKAGE_FILES list.
add_custom_target(MocoPythonBindings ALL DEPENDS
    ${MOCO_PYTHON_PACKAGE_FILES})

# Require the libraries to be built.
add_dependencies(MocoPythonBindings
    ${MOCO_PYTHON_PACKAGE_LIBRARY_TARGETS})

set_target_properties(MocoPythonBindings PROPERTIES
    PROJECT_LABEL "Python - umbrella target"
    FOLDER "Moco/Bindings")


# Test.
# =====
if (BUILD_TESTING)
    # This test runs all the python tests in the tests directory from the
    # source tree. It's important to run the tests in the source tree so that
    # one can edit the tests and immediately re-run the tests without any
    # intermediate file copying.
    # It so happens that ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG> is the same as
    # ${MOCO_PYTHON_BINARY_DIR}, but the former avoids an `if(MSVC OR XCODE)`.
    add_test(NAME python_tests
            WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>"
            COMMAND "${PYTHON_EXECUTABLE}" -m unittest discover
            --start-directory "${CMAKE_CURRENT_SOURCE_DIR}/tests"
            --verbose
            )
    set_tests_properties(python_tests PROPERTIES
            ENVIRONMENT "OPENSIM_USE_VISUALIZER=0")
    # Similar as above, but for the example files. These files aren't named as
    # test_*.py, so we must specify a more general search pattern.
    add_test(NAME python_examples
            WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>"
            COMMAND "${PYTHON_EXECUTABLE}" -m unittest discover
            --start-directory "${CMAKE_CURRENT_SOURCE_DIR}/examples"
            --pattern *.py
            --verbose
            )
    set_property(TEST python_examples APPEND PROPERTY
            ENVIRONMENT "OPENSIM_USE_VISUALIZER=0")

    if(WIN32)
        # On Windows, CMake cannot use RPATH to hard code the location of
        # libraries in the binary directory (DLL's don't have RPATH), so we must
        # set PATH to find the right libraries. The location of the libraries
        # depends on the build configuration, which is filled in for
        # `$<CONFIG>`. We also don't want to accidentally use a different
        # OpenSim build/installation somewhere on the machine.
        foreach(folder tests examples)
            set_property(TEST python_${folder} APPEND PROPERTY ENVIRONMENT
                    "PATH=${CMAKE_BINARY_DIR}/$<CONFIG>"
                    )
        endforeach()
    endif()

    # Allow MSVC users to run only the python tests directly from the MSVC GUI.
    # The python tests are run from RUN_TESTS, so no need to run this target as
    # part of `BUILD_ALL` (e.g, in MSVC). Might need to set
    # EXCLUDE_FROM_DEFAULT_BUILD to achieve this?
    add_custom_target(RunMocoPythonTests
            COMMAND ${CMAKE_CTEST_COMMAND} --tests-regex python
            --build-config ${CMAKE_CFG_INTDIR}
            --extra-verbose)
    set_target_properties(RunMocoPythonTests PROPERTIES
            PROJECT_LABEL "Python - run tests"
            FOLDER "Moco/Bindings")

    add_dependencies(RunMocoPythonTests MocoPythonBindings)
endif()

# Install python package.
# =======================
# Most of the files are installed via the above macros.
# Install the test scripts.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests"
        DESTINATION "${MOCO_INSTALL_PYTHONDIR}/opensim")

# Install example files (not installed next to the python package).
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/examples/"
        DESTINATION "${MOCO_INSTALL_PYTHONEXDIR}")

install(FILES
        "${CMAKE_SOURCE_DIR}/Moco/plot_trajectory.py"
        "${CMAKE_SOURCE_DIR}/Moco/plot_casadi_sparsity.py"
        DESTINATION "${MOCO_INSTALL_PYTHONEXDIR}"
        )
