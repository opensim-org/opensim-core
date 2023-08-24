include(CMakeParseArguments)

# Add a relative path to the run-path of a shared library. The run-path allows
# a library to find the libraries it depends on without the need to set the
# (DY)LD_LIBRARY_PATH environment variable. This function nnly affects UNIX
# systems. Here are the arguments:
#
# TARGET: The CMake target to which the run-path should be added.
# EXECUTABLE, LOADER: Specify one of these flags to indicate whether the
#   run-path is relative to the executable's location or relative to the loader
#   (the library/executable that attempts to load the dependent library). Only
#   affects Mac.
# FROM: The path (relative to the OpenSim installation root) of the
#   executable/library that is loading a dependent library.
# TO: The path (relative to the OpenSim installation root) where the dependent
#   library is located.
function(OpenSimAddInstallRPATH)
    set(options EXECUTABLE LOADER)
    set(oneValueArgs TARGET FROM TO)
    set(multiValueArgs)
    cmake_parse_arguments(
        OSIMRP "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    if(OSIMRP_EXECUTABLE EQUAL OSIMRP_LOADER) 
        message(AUTHOR_WARNING "Cannot specify both EXECUTABLE and LOADER.")
    endif()

    if(NOT OPENSIM_USE_INSTALL_RPATH)
        return()
    endif()

    if(APPLE)
        if(OSIMRP_EXECUTABLE)
            set(rpath_macro "\@executable_path")
        elseif(OSIMRP_LOADER)
            set(rpath_macro "\@loader_path")
        endif()
    elseif(UNIX)
        set(rpath_macro "\$ORIGIN")
    endif()

    file(RELATIVE_PATH to_root "${CMAKE_INSTALL_PREFIX}/${OSIMRP_FROM}"
        "${CMAKE_INSTALL_PREFIX}")
    set_property(TARGET ${OSIMRP_TARGET} APPEND PROPERTY INSTALL_RPATH
        "${rpath_macro}/${to_root}${OSIMRP_TO}")
    message(STATUS "Set rpath '${rpath_macro}/${to_root}${OSIMRP_TO}' to '${OSIMRP_TARGET}'")
endfunction()

# Similar to OpenSimAddInstallRPATH except the run-path is the same directory
# loader (e.g., "@loader_path/"). Arguments:
#
# TARGET: The CMake target to which the run-path should be added.
function(OpenSimAddInstallRPATHSelf)
    set(options)
    set(oneValueArgs TARGET)
    set(multiValueArgs)
    cmake_parse_arguments(
        OSIMRP "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT OPENSIM_USE_INSTALL_RPATH)
        return()
    endif()

    # By specifing "" and "" for FROM and TO, the resulting RPATH will simply
    # be "${rpath_macro}/".
    OpenSimAddInstallRPATH(TARGET ${OSIMRP_TARGET} LOADER FROM "" TO "")

endfunction()

# Similar to OpenSimAddInstallRPATH, but specifically for adding a run-path to
# the Simbody libraries copied into the OpenSim installation. If Simbody is not
# copied into the OpenSim installation (OPENSIM_COPY_DEPENDENCIES=OFF) or the
# copied Simbody libraries are installed to the same folder as the OpenSim
# libraries (OPENSIM_INSTALL_UNIX_FHS=ON), then this function does nothing.
#
# TARGET: The CMake target to which the run-path should be added.
# ABSOLUTE: Add an absolute run-path to the Simbody libraries in the OpenSim
#   installation. If you specify this argument, then you should not specify
#   EXECTUABLE, LOADER, or FROM.
# EXECUTABLE, LOADER: Specify one of these flags to indicate whether the
#   run-path is relative to the executable's location or relative to the loader
#   (the library/executable that attempts to load the dependent library). Only
#   affects Mac.
# FROM: The path (relative to the OpenSim installation root) of the
#   executable/library that is loading a dependent Simbody library.
function(OpenSimAddInstallRPATHSimbody)
    set(options EXECUTABLE LOADER ABSOLUTE)
    set(oneValueArgs TARGET FROM)
    set(multiValueArgs)
    cmake_parse_arguments(
        OSIMRP "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    if(OSIMRP_ABSOLUTE AND (OSIMRP_FROM OR OSIMRP_EXECUTABLE OR OSIMRP_LOADER))
        message(AUTHOR_WARNING
            "Cannot specify both ABSOLUTE and FROM, EXECUTABLE, or LOADER.")
    endif()

    if(NOT OPENSIM_USE_INSTALL_RPATH OR NOT OPENSIM_COPY_DEPENDENCIES
            OR OPENSIM_INSTALL_UNIX_FHS)
        return()
    endif()

    file(RELATIVE_PATH simbody_root_to_simbody_lib_dir 
        "${Simbody_ROOT_DIR}" "${Simbody_LIB_DIR}")
    set(root_to_simbody_lib_dir
        "${OPENSIM_INSTALL_SIMBODYDIR}/${simbody_root_to_simbody_lib_dir}")

    if(OSIMRP_ABSOLUTE)
        set_property(TARGET ${OSIMRP_TARGET} APPEND PROPERTY INSTALL_RPATH
            "${CMAKE_INSTALL_PREFIX}/${root_to_simbody_lib_dir}")
    else()
        if(OSIMRP_EXECUTABLE)
            OpenSimAddInstallRPATH(TARGET ${OSIMRP_TARGET} EXECUTABLE
                FROM ${OSIMRP_FROM} TO ${root_to_simbody_lib_dir})
        elseif(OSIMRP_LOADER)
            OpenSimAddInstallRPATH(TARGET ${OSIMRP_TARGET} LOADER
                FROM ${OSIMRP_FROM} TO ${root_to_simbody_lib_dir})
        endif()
    endif()
endfunction()

# Similar to OpenSimAddInstallRPATH, but specifically for adding an absolute
# run-path.
#
# TARGET: The CMake target to which the run-path should be added.
# TO: The path (relative to the OpenSim installation root) where the dependent
#   library is located.
function(OpenSimAddInstallRPATHAbsolute)
    set(options)
    set(oneValueArgs TARGET TO)
    set(multiValueArgs)
    cmake_parse_arguments(
        OSIMRP "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT OPENSIM_USE_INSTALL_RPATH)
        return()
    endif()

    set_property(TARGET ${OSIMRP_TARGET} APPEND PROPERTY INSTALL_RPATH
        "${CMAKE_INSTALL_PREFIX}/${OSIMRP_TO}")
endfunction()

# Create an OpenSim API library. Here are the arguments:
# VENDORLIB: If this is a vendor library, specify "VENDORLIB" as the first
#   argument. Otherwise, omit.
# LOWERINCLUDEDIRNAME: When installing the headers for this library, make the
#   name of the library all lower-case (e.g., Lepton -> lepton).
# EXCLUDEFROMPYTHON: Do not install this library into the python package.
# KIT: Name of the library (e.g., Common).
# AUTHORS: A string listing authors of the library.
# LINKLIBS: List of libraries (targets) to link against.
# INCLUDES: List of header files for the library (obtain via file(GLOB ...)).
# SOURCES: List of cpp files for the library (obtain via file(GLOB ...)).
# TESTDIRS: List of subdirectories that contain tests (and a CMakeLists.txt).
# INCLUDEDIRS (optional): Affects how header files are installed. Use this if
#   the library directory contains subdirectories with header files. If this is
#   the case, this variable should be a list of those subdirectories (relative
#   paths). See OpenSim/Simulation/CMakeLists.txt for an example. If omitted,
#   all the headers specified under INCLUDES are installed into the same
#   directory in the installation tree.
# INCLUDEINSTALLREL (optional): If INCLUDEDIRS is specified, remove
#   "INCLUDEINSTALLREL" from the location of the installed headers.
#
# Here's an example from OpenSim/Common/CMakeLists.txt:
#
#   OpenSimAddLibrary(
#       KIT Common
#       AUTHORS "Clay_Anderson-Ayman_Habib_and_Peter_Loan"
#       LINKLIBS ${Simbody_LIBRARIES}
#       INCLUDES ${INCLUDES}
#       SOURCES ${SOURCES}
#       TESTDIRS "Test"
#       )
function(OpenSimAddLibrary)

    # Parse arguments.
    # ----------------
    # http://www.cmake.org/cmake/help/v2.8.9/cmake.html#module:CMakeParseArguments
    set(options VENDORLIB LOWERINCLUDEDIRNAME EXCLUDEFROMPYTHON)
    set(oneValueArgs KIT AUTHORS INCLUDEINSTALLREL)
    set(multiValueArgs LINKLIBS INCLUDES SOURCES TESTDIRS INCLUDEDIRS)
    cmake_parse_arguments(
        OSIMADDLIB "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    string(TOUPPER "${OSIMADDLIB_KIT}" OSIMADDLIB_UKIT)


    # Version stuff.
    # --------------
    set(OSIMADDLIB_LIBRARY_NAME osim${OSIMADDLIB_KIT})

    add_definitions(
        -DOPENSIM_${OSIMADDLIB_UKIT}_LIBRARY_NAME=${OSIMADDLIB_LIBRARY_NAME}
        -DOPENSIM_${OSIMADDLIB_UKIT}_MAJOR_VERSION=${OPENSIM_MAJOR_VERSION}
        -DOPENSIM_${OSIMADDLIB_UKIT}_MINOR_VERSION=${OPENSIM_MINOR_VERSION}
        -DOPENSIM_${OSIMADDLIB_UKIT}_BUILD_VERSION=${OPENSIM_PATCH_VERSION}
        -DOPENSIM_${OSIMADDLIB_UKIT}_COPYRIGHT_YEARS="2005-2017"
        -DOPENSIM_${OSIMADDLIB_UKIT}_AUTHORS="${AUTHORS}"
        -DOPENSIM_${OSIMADDLIB_UKIT}_TYPE="Shared"
        )


    # Add the library.
    # ----------------
    # These next few lines are the most important:

    # Create the library using the provided source and include files.
    add_library(${OSIMADDLIB_LIBRARY_NAME} SHARED
        ${OSIMADDLIB_SOURCES} ${OSIMADDLIB_INCLUDES})

    target_include_directories(${OSIMADDLIB_LIBRARY_NAME} 
        # Used when building this target:
        PRIVATE "${OpenSim_SOURCE_DIR}"
                "${OpenSim_SOURCE_DIR}/Vendors/lepton/include"
        # Used by other targets in this project:
        INTERFACE $<BUILD_INTERFACE:${OpenSim_SOURCE_DIR}>
                  $<BUILD_INTERFACE:${OpenSim_SOURCE_DIR}/Vendors/lepton/include>
        # Used by client projects using an installed OpenSim:
        INTERFACE $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
                  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/OpenSim>
        )

    # This target links to the libraries provided as arguments to this func.
    target_link_libraries(${OSIMADDLIB_LIBRARY_NAME} ${OSIMADDLIB_LINKLIBS})

    # This is for exporting classes on Windows.
    if(OSIMADDLIB_VENDORLIB)
	    set(OSIMADDLIB_FOLDER "Vendor Libraries")
    else()
		set(OSIMADDLIB_FOLDER "Libraries")
    endif()
    set_target_properties(${OSIMADDLIB_LIBRARY_NAME} PROPERTIES
       DEFINE_SYMBOL OSIM${OSIMADDLIB_UKIT}_EXPORTS
       FOLDER "${OSIMADDLIB_FOLDER}" # For Visual Studio.
    )

    # Install.
    # --------
    # Shared libraries are needed at runtime for applications, so we put them
    # at the top level in OpenSim/bin/*.dll (Windows) or OpenSim/lib/*.so
    # (Linux) or OpemSim/lib/*.dylib (Mac). Windows .lib files, and Linux/Mac
    # .a static archives are only needed at link time so go in sdk/lib.
    if(WIN32)
        set(OSIMADDLIB_LIBRARY_DESTINATION sdk/lib)
    else()
        set(OSIMADDLIB_LIBRARY_DESTINATION lib)
    endif()
    install(TARGETS ${OSIMADDLIB_LIBRARY_NAME}
        EXPORT OpenSimTargets
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        ARCHIVE DESTINATION "${OPENSIM_INSTALL_ARCHIVEDIR}")
    if(BUILD_PYTHON_WRAPPING AND OPENSIM_PYTHON_STANDALONE
            AND NOT OSIMADDLIB_EXCLUDEFROMPYTHON)
        if (WIN32)
            install(TARGETS ${OSIMADDLIB_LIBRARY_NAME}
                RUNTIME DESTINATION "${OPENSIM_INSTALL_PYTHONDIR}/opensim")
        else()
            install(TARGETS ${OSIMADDLIB_LIBRARY_NAME}
                LIBRARY DESTINATION "${OPENSIM_INSTALL_PYTHONDIR}/opensim")
        endif()
    endif()

    # Install headers.
    # ----------------
    set(_INCLUDE_PREFIX "${CMAKE_INSTALL_INCLUDEDIR}/OpenSim")
    if(OSIMADDLIB_LOWERINCLUDEDIRNAME)
        string(TOLOWER "${OSIMADDLIB_KIT}" OSIMADDLIB_LKIT)
        set(_INCLUDE_LIBNAME ${OSIMADDLIB_LKIT})
    else()
        set(_INCLUDE_LIBNAME ${OSIMADDLIB_KIT})
    endif()
    if(OSIMADDLIB_INCLUDEDIRS)
        foreach(dir ${OSIMADDLIB_INCLUDEDIRS})
            file(GLOB HEADERS ${dir}/*.h) # returns full pathnames
            if(OSIMADDLIB_INCLUDEINSTALLREL)
                # Remove "INCLUDEINSTALLREL" from the installed location.
                file(RELATIVE_PATH subdir
                    "${CMAKE_CURRENT_SOURCE_DIR}${OSIMADDLIB_INCLUDEINSTALLREL}"
                    "${CMAKE_CURRENT_SOURCE_DIR}${dir}")
            else()
                set(subdir ${dir})
            endif()
            install(FILES ${HEADERS}
                DESTINATION ${_INCLUDE_PREFIX}/${_INCLUDE_LIBNAME}/${subdir})
        endforeach(dir)
    else()
        install(FILES ${OSIMADDLIB_INCLUDES}
            DESTINATION ${_INCLUDE_PREFIX}/${_INCLUDE_LIBNAME})
    endif()

    # RPATH (so that libraries find library dependencies)
    # ---------------------------------------------------
    OpenSimAddInstallRPATHSelf(TARGET ${OSIMADDLIB_LIBRARY_NAME} LOADER)
    OpenSimAddInstallRPATHSimbody(TARGET ${OSIMADDLIB_LIBRARY_NAME} LOADER
        FROM "${CMAKE_INSTALL_LIBDIR}")

    # Testing.
    # --------
    enable_testing()

    if(BUILD_TESTING)
        foreach(OSIMADDLIB_TESTDIR ${OSIMADDLIB_TESTDIRS})
            subdirs("${OSIMADDLIB_TESTDIR}")
        endforeach()
    endif()

endfunction()

# Copy a file from the directory containing test files (model files, data,
# etc.) to the directory in which a test will be executed. This function makes
# it easy to re-use files that are used in tests. With an easier mechanism for
# re-using these files, we won't end up version-controlling the same file in
# multiple test directories.
#
# Arguments are a list of files in the test resources directory
# (OPENSIM_SHARED_TEST_FILES_DIR) to copy.
function(OpenSimCopySharedTestFiles)
    if(BUILD_TESTING)
        foreach(filename ${ARGN})
            file(COPY "${OPENSIM_SHARED_TEST_FILES_DIR}/${filename}"
                 DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
        endforeach()
    endif()
endfunction()

# Create test targets for this directory.
# TESTPROGRAMS: Names of test CPP files. One test will be created for each cpp
#   of these files.
# DATAFILES: Files necessary to run the test. These will be copied into the
#   corresponding build directory.
# LINKLIBS: Arguments to TARGET_LINK_LIBRARIES.
# SOURCES: Extra source files for the executable.
#
# Here's an example:
#   file(GLOB TEST_PROGRAMS "test*.cpp")
#   file(GLOB DATA_FILES *.osim *.xml *.sto *.mot)
#   OpenSimAddTests(
#       TESTPROGRAMS ${TEST_ROGRAMS}
#       DATAFILES ${DATA_FILES}
#       LINKLIBS osimCommon osimSimulation osimAnalyses
#       )
function(OpenSimAddTests)

    if(BUILD_TESTING)

        # Parse arguments.
        # ----------------
        # http://www.cmake.org/cmake/help/v2.8.9/cmake.html#module:CMakeParseArguments
        set(options)
        set(oneValueArgs)
        set(multiValueArgs TESTPROGRAMS DATAFILES LINKLIBS SOURCES)
        cmake_parse_arguments(
            OSIMADDTESTS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

        # If EXECUTABLE_OUTPUT_PATH is set, then that's where the tests will be
        # located. Otherwise, they are located in the current binary directory.
        if(EXECUTABLE_OUTPUT_PATH)
            set(TEST_PATH "${EXECUTABLE_OUTPUT_PATH}")
        else()
            set(TEST_PATH "${CMAKE_CURRENT_BINARY_DIR}")
        endif()

        # Make test targets.
        foreach(test_program ${OSIMADDTESTS_TESTPROGRAMS})
            # NAME_WE stands for "name without extension"
            get_filename_component(TEST_NAME ${test_program} NAME_WE)

            add_executable(${TEST_NAME} ${test_program}
                ${OSIMADDTESTS_SOURCES})
            target_link_libraries(${TEST_NAME} ${OSIMADDTESTS_LINKLIBS})
            add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
            set_target_properties(${TEST_NAME} PROPERTIES
                FOLDER "Tests"
				)

        endforeach()

        # Copy data files to build directory.
        foreach(data_file ${OSIMADDTESTS_DATAFILES})
            # This command re-copies the data files if they are modified;
            # custom commands don't do this.
            file(COPY "${data_file}" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
        endforeach()

        #if(UNIX)
        #  add_definitions(-fprofile-arcs -ftest-coverage)
        #  link_libraries(gcov)
        #endif(UNIX)

    endif()

endfunction()


# Create an application/executable. To be used in the Appliations directory.
# NAME: Name of the application. Must also be the name of the source file
#   containing main() (without the .cpp extension).
# SOURCES: Additional header/source files to compile into this target. 
#
# Here's an example:
#   OpenSimAddApplication(NAME opensim-cmd SOURCES opensim-cmd_run-tool.h)
function(OpenSimAddApplication)

    # Parse arguments.
    # ----------------
    # http://www.cmake.org/cmake/help/v2.8.9/cmake.html#module:CMakeParseArguments
    set(options)
    set(oneValueArgs NAME)
    set(multiValueArgs SOURCES)
    cmake_parse_arguments(
        OSIMADDAPP "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    
    # Build.
    add_executable(${OSIMADDAPP_NAME} ${OSIMADDAPP_NAME}.cpp
                                      ${OSIMADDAPP_SOURCES})
    target_link_libraries(${OSIMADDAPP_NAME} osimTools)
    set_target_properties(${OSIMADDAPP_NAME} PROPERTIES
        FOLDER "Applications")

    # Install.
    install(TARGETS ${OSIMADDAPP_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

    # RPATH (so that the executable finds libraries without using env. vars).
    OpenSimAddInstallRPATH(TARGET ${OSIMADDAPP_NAME} EXECUTABLE
        FROM "${CMAKE_INSTALL_BINDIR}" TO "${CMAKE_INSTALL_LIBDIR}")
    OpenSimAddInstallRPATHSimbody(TARGET ${OSIMADDAPP_NAME} EXECUTABLE
        FROM "${CMAKE_INSTALL_BINDIR}")

endfunction()


# Add a target to the OpenSim project for building an example with the given
# NAME. The example must be within a source file named ${NAME}.cpp, or could
# contain multiple executable files, listed via the EXECUTABLES argument
# (omitting the .cpp extension). This function also installs the example files
# with a CMakeLists that can find the OpenSim installation and build the
# example.
#
# If SUBDIR is provided, the example is installed into this subdirectory of the
# C++ Examples folder.
#
# This function can only be used from the source distribution of OpenSim
# (e.g., not via the UseOpenSim.cmake file in a binary distribution).
function(OpenSimAddExampleCXX)
    set(options)
    set(oneValueArgs NAME SUBDIR)
    set(multiValueArgs RESOURCES EXECUTABLES)
    cmake_parse_arguments(OSIMEX
            "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT OSIMEX_EXECUTABLES)
        set(OSIMEX_EXECUTABLES ${OSIMEX_NAME})
    endif()

    # Build the example in the build tree.
    foreach(exe ${OSIMEX_EXECUTABLES})
        add_executable(${exe} ${exe}.cpp)
        set_target_properties(${exe} PROPERTIES FOLDER "Examples")
        target_link_libraries(${exe} osimTools osimExampleComponents osimMoco)
    endforeach()
    file(COPY ${OSIMEX_RESOURCES} DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

    # Install files so that users can build the example.
    # We do not install pre-built binaries of the examples.
    set(_example_install_dir
            ${OPENSIM_INSTALL_CPPEXDIR}/${OSIMEX_SUBDIR}/${OSIMEX_NAME})
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
            DESTINATION "${_example_install_dir}"
            PATTERN "CMakeLists.txt" EXCLUDE)

    # These next two variables are to be configured below (they are not used
    # here, but within ExampleCMakeListsToInstall.txt.in).
    set(_example_name ${OSIMEX_NAME})
    set(_example_executables ${OSIMEX_EXECUTABLES})
    file(RELATIVE_PATH _opensim_install_hint
            "${CMAKE_INSTALL_PREFIX}/${_example_install_dir}"
            "${CMAKE_INSTALL_PREFIX}")
    configure_file(
        "${CMAKE_SOURCE_DIR}/OpenSim/Examples/ExampleCMakeListsToInstall.txt.in"
        "${CMAKE_CURRENT_BINARY_DIR}/CMakeListsToInstall.txt"
        @ONLY)

    install(FILES
            "${CMAKE_CURRENT_BINARY_DIR}/CMakeListsToInstall.txt"
            DESTINATION ${_example_install_dir}
            RENAME CMakeLists.txt)
endfunction()


# Add a target to the OpenSim project for building an example containing a
# plugin library and an executable.
# The directory containing the CMakeLists.txt invoking this macro must have 3
# source files: ${NAME}.h and ${NAME}.cpp are used to create the plugin, and
# and example${NAME}.cpp is compiled into an executable that uses the plugin.
# This function also installs the example file with a CMakeLists that can find
# the OpenSim installation and build the example.
#
# If SUBDIR is provided, the example is installed into this subdirectory of the
# C++ Examples folder.
#
# This function can only be used from the source distribution of OpenSim
# (e.g., not via the UseOpenSim.cmake file in a binary distribution).
function(OpenSimAddPluginExampleCXX)
    set(options)
    set(oneValueArgs NAME SUBDIR)
    set(multiValueArgs RESOURCES)
    cmake_parse_arguments(OSIMEX
            "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    add_library(osim${OSIMEX_NAME} SHARED
            ${OSIMEX_NAME}.h
            ${OSIMEX_NAME}.cpp
            osim${OSIMEX_NAME}DLL.h
            RegisterTypes_osim${OSIMEX_NAME}.h
            RegisterTypes_osim${OSIMEX_NAME}.cpp
            )
    set_target_properties(osim${OSIMEX_NAME} PROPERTIES
            FOLDER "Examples")
    target_link_libraries(osim${OSIMEX_NAME} osimTools osimExampleComponents
            osimMoco)

    string(TOUPPER ${OSIMEX_NAME} _example_name_upper)
    set_target_properties(osim${OSIMEX_NAME} PROPERTIES
            DEFINE_SYMBOL OSIM${_example_name_upper}_EXPORTS
            )

    add_executable(example${OSIMEX_NAME} example${OSIMEX_NAME}.cpp)
    set_target_properties(example${OSIMEX_NAME} PROPERTIES
            FOLDER "Examples")
    target_link_libraries(example${OSIMEX_NAME} osim${OSIMEX_NAME})
    file(COPY ${OSIMEX_RESOURCES} DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

    set(_example_install_dir
            ${OPENSIM_INSTALL_CPPEXDIR}/${OSIMEX_SUBDIR}/Plugins/example${OSIMEX_NAME})
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
            DESTINATION "${_example_install_dir}"
            PATTERN "CMakeLists.txt" EXCLUDE)

    # These next two variables are to be configured below (they are not used
    # here, but within ExampleCMakeListsToInstall.txt.in).
    set(_example_name ${OSIMEX_NAME})
    file(RELATIVE_PATH _opensim_install_hint
            "${CMAKE_INSTALL_PREFIX}/${_example_install_dir}"
            "${CMAKE_INSTALL_PREFIX}")
    configure_file(
        "${CMAKE_SOURCE_DIR}/OpenSim/Examples/PluginExampleCMakeListsToInstall.txt.in"
        "${CMAKE_CURRENT_BINARY_DIR}/CMakeListsToInstall.txt"
        @ONLY)

    install(FILES
            "${CMAKE_CURRENT_BINARY_DIR}/CMakeListsToInstall.txt"
            DESTINATION ${_example_install_dir}
            RENAME CMakeLists.txt)
endfunction()


# Function to install shared libraries (any platform) from a dependency install
# directory into the OpenSim installation. One use case is to install libraries
# into the python package.
# PREFIX: A common part of the library file names (e.g., 'SimTK' or 'ezc3d').
#         This is to avoid copying unrelated files from a folder like /usr/lib.
# DEP_LIBS_DIR_WIN: Directory to search for the dependency's library, on
#         Windows.
# DEP_LIBS_DIR_UNIX: Directory to search for the dependency's library, on
#         UNIX (APPLE and Linux). Specify only the lib directory to avoid
#         searching all of /usr/local (if the dependency is installed to a
#         system location like this).
# OSIM_DESTINATION: Destination of the libraries within OpenSim's installation.
function(OpenSimInstallDependencyLibraries PREFIX DEP_LIBS_DIR_WIN
        DEP_LIBS_DIR_UNIX OSIM_DESTINATION)
    if(WIN32)
        file(GLOB_RECURSE LIBS "${DEP_LIBS_DIR_WIN}/${PREFIX}*.dll")
    else()
        if(APPLE)
            set(lib_ext "dylib")
        else()
            set(lib_ext "so*") # Trailing * for version #s.
        endif()
        file(GLOB_RECURSE LIBS "${DEP_LIBS_DIR_UNIX}/lib${PREFIX}*.${lib_ext}")
    endif()
    if(NOT LIBS)
        message(FATAL_ERROR "Zero shared libraries found in directory "
            "${DEP_INSTALL_DIR}.")
    endif()
    install(FILES ${LIBS} DESTINATION "${OSIM_DESTINATION}")
endfunction()


# Copy DLL files from a dependency's installation into the
# build and install directories. This is a Windows-specific function enabled
# only on Windows. The intention is to allow the runtime loader to find all 
# the required DLLs without editing the PATH environment variable.
# Arguments:
#   DEP_NAME: Name of the dependency (used to name a custom target)
#   DEP_BIN_DIR: The directory in the dependency containing DLLs to copy.
# This is based on a similar function in OpenSimMacros.cmake.
function(OpenSimCopyDependencyDLLsForWin)
    # On Windows, copy dlls into the OpenSim binary directory.
    set(options)
    set(oneValueArgs DEP_NAME DEP_BIN_DIR)
    set(multiValueArgs)
    cmake_parse_arguments(OSIMCOPY
            "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    if(WIN32)
        file(GLOB DLLS ${OSIMCOPY_DEP_BIN_DIR}/*.dll)
        if(NOT DLLS)
            message(FATAL_ERROR "Zero DLLs found in directory "
                                "${OSIMCOPY_DEP_BIN_DIR}.")
        endif()
        set(DEST_DIR "${CMAKE_BINARY_DIR}/${CMAKE_CFG_INTDIR}")
        foreach(DLL IN LISTS DLLS)
            get_filename_component(DLL_NAME ${DLL} NAME)
            list(APPEND DLLS_DEST "${DEST_DIR}/${DLL_NAME}")
            add_custom_command(OUTPUT "${DEST_DIR}/${DLL_NAME}"
                COMMAND ${CMAKE_COMMAND} -E make_directory ${DEST_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy ${DLL} ${DEST_DIR}
                DEPENDS ${DLL}
                COMMENT "Copying ${DLL_NAME} from ${OSIMCOPY_DEP_BIN_DIR} to ${DEST_DIR}.")
        endforeach()
        add_custom_target(Copy_${OSIMCOPY_DEP_NAME}_DLLs ALL DEPENDS ${DLLS_DEST})
        set_target_properties(Copy_${OSIMCOPY_DEP_NAME}_DLLs PROPERTIES
            PROJECT_LABEL "Copy ${OSIMCOPY_DEP_NAME} DLLs")
        if(OPENSIM_COPY_DEPENDENCIES)
            install(FILES ${DLLS} DESTINATION ${CMAKE_INSTALL_BINDIR})
        endif()
    endif()
endfunction()


# Discover the file dependencies for an invocation of swig, for use with the
# DEPENDS field of an add_custom_command().
#
# OSIMSWIGDEP_RETURNVAL  is filled with a list of the dependencies.
# OSIMSWIGDEP_MODULE     is the name of the module (just for messages).
# OSIMSWIGDEP_INVOCATION is the SWIG command to use to check for dependencies.
#                        We append `-MM` to this, which asks SWIG for the
#                        dependencies.
macro(OpenSimFindSwigFileDependencies OSIMSWIGDEP_RETURNVAL
                                      OSIMSWIGDEP_MODULE
                                      OSIMSWIGDEP_INVOCATION)
    # We must use a macro instead of a function in order to return a value.

    # Assemble dependencies. This command is run during CMake's configure step.
    message(STATUS
        "Discovering dependencies for SWIG module ${OSIMSWIGDEP_MODULE}.")
    execute_process(COMMAND ${SWIG_EXECUTABLE}
            -MM # List dependencies, but omit files in SWIG library.
            ${OSIMSWIGDEP_INVOCATION}
        OUTPUT_VARIABLE _dependencies_makefile
        RESULT_VARIABLE _successfully_got_dependencies
            )
    # On Windows, _dependencies_makefile now contains something like this:
    # C:\opensim-core\Bindings\Python\swig\python_simbody_wrap.cxx: \
    #   C:\opensim-core\Bindings\Python\swig\python_simbody.i \
    #   C:\opensim-core\Bindings\Python\swig\numpy.i \
    #   ...<more header files and SWIG interface files>
    # Note: SWIG does not depend on cpp files.
    # Clean up the output, since it's in the form of a makefile
    # (and we just want a list of file paths).
    if(${_successfully_got_dependencies} EQUAL 0) # return code 0 is success.
        # '^.*cxx:' matches the first line of the makefile (the output file path).
        # '\\\\' matches a single \ (escape for CMake, and escape for regex).
        string(REGEX REPLACE "(^.*cxx:|\\\\\n)" "" ${OSIMSWIGDEP_RETURNVAL}
            ${_dependencies_makefile})
        # Replace spaces with semicolons to create a list of file paths.
        separate_arguments(${OSIMSWIGDEP_RETURNVAL})
    else()
        # In case the variable has a value for previous modules.
        unset(${OSIMSWIGDEP_RETURNVAL})
        # If someone ends up here, it's a bug in this CMakeLists.txt.
        message(AUTHOR_WARNING
            "Could not determine dependencies for SWIG module ${OSIMSWIGDEP_MODULE}.")
    endif()

    unset(_dependencies_makefile)
    unset(_successfully_got_dependencies)

endmacro()

