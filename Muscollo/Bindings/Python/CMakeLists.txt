# PythonInterp is supposed to come before PythonLibs.
if(${MUSCOLLO_PYTHON_VERSION} STREQUAL "2")
    set(required_python_version 2.7)
elseif(${MUSCOLLO_PYTHON_VERSION} STREQUAL "3")
    set(required_python_version 3)
else()
    message(FATAL_ERROR "MUSCOLLO_PYTHON_VERSION should be '2' or '3'.")
endif()
find_package(PythonInterp ${required_python_version} REQUIRED)

set(WINLIBSUFFIX "pyd")
set(UNIXLIBSUFFIX "so")

# We update the python install dir to include the python version,
# now that we know it. We replace the token "VERSION" with the actual python
# version.
set(python_version_majmin "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")
string(REPLACE "VERSION" "${python_version_majmin}"
    MUSCOLLO_INSTALL_PYTHONDIR "${MUSCOLLO_INSTALL_PYTHONDIR}")
# This must be done before adding the OpenSim libraries, since
# MUSCOLLO_INSTALL_PYTHONDIR is used in OpenSimAddLibrary (in
# OpenSimMacros.cmake).

if(APPLE AND ${MUSCOLLO_PYTHON_VERSION} STREQUAL "2")
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


# Location of the opensim python package in the build directory, for testing.
if(MSVC OR XCODE)
    # Multi-configuration generators like MSVC and XCODE use one build tree for
    # all configurations.
    set(MUSCOLLO_PYTHON_BINARY_DIR
        "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_CFG_INTDIR}")
else()
    set(MUSCOLLO_PYTHON_BINARY_DIR
        "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_BUILD_TYPE}")
endif()

# Avoid excessive compiler warnings.
# ==================================
# We set these COMPILE_OPTIONS in the root CMakeLists.txt.
set_directory_properties(PROPERTIES COMPILE_OPTIONS "")

# Helper macros.
# ==============

# Helper function to for copying files into the python package.
macro(MuscolloPutFileInPythonPackage source_full_path relative_dest_dir)

    # Python package in the build tree.
    # ---------------------------------
    get_filename_component(file_name "${source_full_path}" NAME)
    set(binary_dest_full_path
        "${MUSCOLLO_PYTHON_BINARY_DIR}/${relative_dest_dir}/${file_name}")
    add_custom_command(
        DEPENDS "${source_full_path}"
        OUTPUT "${binary_dest_full_path}"
        COMMAND ${CMAKE_COMMAND} -E copy "${source_full_path}"
                                         "${binary_dest_full_path}"
        COMMENT "Copying ${source_full_path} to python package in build directory"
        VERBATIM
        )
    # This list is used to specify dependencies for the MuscolloPythonBindings target.
    list(APPEND MUSCOLLO_PYTHON_PACKAGE_FILES "${binary_dest_full_path}")

    # Python package in the installation.
    # -----------------------------------
    install(FILES "${source_full_path}"
        DESTINATION "${MUSCOLLO_INSTALL_PYTHONDIR}/${relative_dest_dir}")

endmacro()

# Generates source code for python module and then compiles it.
# Here are the arguments:
# MODULE: Name of python module. The module is build with the interface file
#       named ${MODULE}_python.i.
# DEPENDS: Names of other python modules on which this module depends. 
macro(MuscolloAddPythonModule)
    # Parse arguments.
    # ----------------
    # http://www.cmake.org/cmake/help/v2.8.9/cmake.html#module:CMakeParseArguments
    set(options)
    set(oneValueArgs MODULE)
    set(multiValueArgs DEPENDS)
    cmake_parse_arguments(
        MUCOSWIGPY "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # Generate source code for wrapper using SWIG.
    # --------------------------------------------
    set(_output_file_prefix
        "${CMAKE_CURRENT_BINARY_DIR}/python_${MUCOSWIGPY_MODULE}_wrap")
    set(_output_cxx_file "${_output_file_prefix}.cxx")
    set(_output_header_file "${_output_file_prefix}.h")
    set(_interface_file
        "${CMAKE_CURRENT_SOURCE_DIR}/swig/python_${MUCOSWIGPY_MODULE}.i")

    # We run swig once to get dependencies and then again to actually generate
    # the wrappers. This variable holds the parts of the swig command that
    # are shared between both invocations.
    set(_swig_common_args -c++ -python
            -I${CMAKE_SOURCE_DIR}/Muscollo
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
    OpenSimFindSwigFileDependencies(_${MUCOSWIGPY_MODULE}_dependencies
        ${MUCOSWIGPY_MODULE} "${_swig_common_args}")

    # Run swig.
    add_custom_command(
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${MUCOSWIGPY_MODULE}.py"
            ${_output_cxx_file} ${_output_header_file}
        COMMAND ${SWIG_EXECUTABLE}
            #-debug-tmused # Which typemaps were used?
            -v # verbose
            -o ${_output_cxx_file}
            -outdir "${CMAKE_CURRENT_BINARY_DIR}"
            ${_swig_common_args}
        DEPENDS ${_${MUCOSWIGPY_MODULE}_dependencies}
            COMMENT "Generating python bindings source code with SWIG: ${MUCOSWIGPY_MODULE} module."
        )

    # Compile python wrapper files into a library.
    # --------------------------------------------
    set(_libname _${MUCOSWIGPY_MODULE}) 

    # Used for specifying dependencies for MuscolloPythonBindings.
    list(APPEND MUSCOLLO_PYTHON_PACKAGE_LIBRARY_TARGETS ${_libname})

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
    set_source_files_properties("${_output_cxx_file}"
        PROPERTIES COMPILE_FLAGS "${_COMPILE_FLAGS}")

    add_library(${_libname} SHARED ${_output_cxx_file} ${_output_header_file})

    target_include_directories(${_libname} SYSTEM PRIVATE
        # For Bindings/OpenSimHeaders_common.h, etc.:
        "${CMAKE_SOURCE_DIR}/opensim-core"
        "${PYTHON_INCLUDE_PATH}")
    
    target_link_libraries(${_libname} osimMuscollo ${PYTHON_LIBRARIES})
    
    # Set target properties for various platforms.
    # --------------------------------------------
    # Resulting library must be named with .so on Unix, .pyd on Windows.
    set_target_properties(${_libname} PROPERTIES
        PROJECT_LABEL "Python - ${_libname}"
        FOLDER "Muscollo/Bindings"
        PREFIX ""
    )
    if(WIN32)
        set_target_properties(${_libname} PROPERTIES SUFFIX ".${WINLIBSUFFIX}")
    elseif(APPLE)
        # Defaults to .dylib; change to .so.
        set_target_properties(${_libname} PROPERTIES SUFFIX ".${UNIXLIBSUFFIX}")
    endif()
    
    if(MUSCOLLO_COPY_DEPENDENCIES)
        # We always set a relative RPATH but only use an absolute RPATH if the
        # python package is not standalone, as the libraries are not copied
        # into the python package.
        if(APPLE)
            set(rpath_macro "\@loader_path")
        elseif(UNIX)
            set(rpath_macro "\$ORIGIN")
        endif()

        set(run_path_list "${rpath_macro}/")

        # Locate osim libraries, etc in the lib dir.
        file(RELATIVE_PATH py_dir_to_install_dir
            "${CMAKE_INSTALL_PREFIX}/${MUSCOLLO_INSTALL_PYTHONDIR}/opensim"
            "${CMAKE_INSTALL_PREFIX}")
        set(py_dir_to_lib_dir
            "${py_dir_to_install_dir}${CMAKE_INSTALL_LIBDIR}")
        list(APPEND run_path_list "${rpath_macro}/${py_dir_to_lib_dir}")

        # Absolute RPATH.
        # TODO we don't yet support a standalone python package.
        #if(NOT OPENSIM_PYTHON_STANDALONE)
            list(APPEND run_path_list
                    "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
        #endif()
        set_property(TARGET ${_libname} APPEND PROPERTY
            INSTALL_RPATH "${run_path_list}")
    endif()

    # Copy files into the build tree python package.
    # ----------------------------------------------
    # Copy the library file.
    add_custom_command(TARGET ${_libname} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:${_libname}>"
            "${MUSCOLLO_PYTHON_BINARY_DIR}/opensim/$<TARGET_FILE_NAME:${_libname}>"
        COMMENT "Copying ${_libname} library to python package in build directory."
        VERBATIM
        )

    # Copy the generated .py file to the per-config python package dir.
    MuscolloPutFileInPythonPackage(
        "${CMAKE_CURRENT_BINARY_DIR}/${MUCOSWIGPY_MODULE}.py" opensim)

    # Install the library.
    # --------------------
    # It's important that we use install(TARGETS) 
    # because this causes CMake to remove the build-tree RPATH from the library
    # (which is set temporarily for libraries in the build tree).
    install(TARGETS ${_libname} DESTINATION
        "${MUSCOLLO_INSTALL_PYTHONDIR}/opensim")

endmacro()


# Build python modules (generate binding source code and compile it).
# ===================================================================
MuscolloAddPythonModule(MODULE muscollo)


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
    MuscolloPutFileInPythonPackage(
        "${OpenSim_PYTHON_PACKAGE_DIR}/opensim/_${module}.${libsuffix}"
        opensim)
    MuscolloPutFileInPythonPackage(
        "${OpenSim_PYTHON_PACKAGE_DIR}/opensim/${module}.py" opensim)
endforeach()

# Copy the configured version.py for each build configuration.
MuscolloPutFileInPythonPackage("${CMAKE_CURRENT_BINARY_DIR}/version.py" opensim)

# Copy setup.py for each build configuration.
MuscolloPutFileInPythonPackage("${CMAKE_CURRENT_SOURCE_DIR}/setup.py" ".")

# __init__.py.
MuscolloPutFileInPythonPackage("${CMAKE_CURRENT_SOURCE_DIR}/__init__.py" opensim)

# Test files. If you require more test resource files, list them here.
foreach(test_file
        # None yet.
        )
    MuscolloPutFileInPythonPackage("${test_file}" opensim/tests)
endforeach()


# Umbrella target for assembling the python bindings in the build tree.
# =====================================================================
# This command must come *after* all calls to MuscolloPutFileInPythonPackage, as
# that macro assembles the MUSCOLLO_PYTHON_PACKAGE_FILES list.
add_custom_target(MuscolloPythonBindings ALL DEPENDS
    ${MUSCOLLO_PYTHON_PACKAGE_FILES})

# Require the libraries to be built.
add_dependencies(MuscolloPythonBindings
    ${MUSCOLLO_PYTHON_PACKAGE_LIBRARY_TARGETS})

set_target_properties(MuscolloPythonBindings PROPERTIES
    PROJECT_LABEL "Python - umbrella target"
    FOLDER "Muscollo/Bindings")


# Test.
# =====
# This test runs all the python tests in the tests directory from the
# source tree. It's important to run the tests in the source tree so that
# one can edit the tests and immediately re-run the tests without any
# intermediate file copying.
# It so happens that ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG> is the same as
# ${MUSCOLLO_PYTHON_BINARY_DIR}, but the former avoids an `if(MSVC OR XCODE)`.
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
    # On Windows, CMake cannot use RPATH to hard code the location of libraries
    # in the binary directory (DLL's don't have RPATH), so we must set PATH to
    # find the right libraries. The location of the libraries depends on the
    # build configuration, which is filled in for `$<CONFIG>`. We also don't
    # want to accidentally use a different OpenSim build/installation somewhere
    # on the machine.
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
add_custom_target(RunMuscolloPythonTests
    COMMAND ${CMAKE_CTEST_COMMAND} --tests-regex python
                                   --build-config ${CMAKE_CFG_INTDIR}
                                   --extra-verbose)
set_target_properties(RunMuscolloPythonTests PROPERTIES
    PROJECT_LABEL "Python - run tests"
    FOLDER "Muscollo/Bindings")

add_dependencies(RunMuscolloPythonTests MuscolloPythonBindings)

# Install python package.
# =======================
# Most of the files are installed via the above macros.
# Install the test scripts.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tests"
        DESTINATION "${MUSCOLLO_INSTALL_PYTHONDIR}/opensim")

# Install example files (not installed next to the python package).
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/examples/"
        DESTINATION "${MUSCOLLO_INSTALL_PYTHONEXDIR}")
