include(CMakeParseArguments)

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
    set(oneValueArgs KIT AUTHORS)
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
        -DOPENSIM_${OSIMADDLIB_UKIT}_COPYRIGHT_YEARS="2005-2014"
        -DOPENSIM_${OSIMADDLIB_UKIT}_AUTHORS="${AUTHORS}"
        -DOPENSIM_${OSIMADDLIB_UKIT}_TYPE="Shared"
        )


    # Add the library.
    # ----------------
    # These next few lines are the most important:
    # Specify the directories in OpenSim that contain header files.
    include_directories(${OpenSim_SOURCE_DIR})

    # Create the library using the provided source and include files.
    add_library(${OSIMADDLIB_LIBRARY_NAME} SHARED
        ${OSIMADDLIB_SOURCES} ${OSIMADDLIB_INCLUDES})

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
    set(_INCLUDE_PREFIX "${CMAKE_INSTALL_INCLUDEDIR}")
    if(OSIMADDLIB_VENDORLIB)
        set(_INCLUDE_PREFIX ${_INCLUDE_PREFIX}/Vendors)
    else()
        set(_INCLUDE_PREFIX ${_INCLUDE_PREFIX}/OpenSim)
    endif()
    if(OSIMADDLIB_LOWERINCLUDEDIRNAME)
        string(TOLOWER "${OSIMADDLIB_KIT}" OSIMADDLIB_LKIT)
        set(_INCLUDE_LIBNAME ${OSIMADDLIB_LKIT})
    else()
        set(_INCLUDE_LIBNAME ${OSIMADDLIB_KIT})
    endif()
    if(OSIMADDLIB_INCLUDEDIRS)
        foreach(dir ${OSIMADDLIB_INCLUDEDIRS})
            file(GLOB HEADERS ${dir}/*.h) # returns full pathnames
            install(FILES ${HEADERS}
                DESTINATION ${_INCLUDE_PREFIX}/${_INCLUDE_LIBNAME}/${dir})
        endforeach(dir)
    else()
        install(FILES ${OSIMADDLIB_INCLUDES}
            DESTINATION ${_INCLUDE_PREFIX}/${_INCLUDE_LIBNAME})
    endif()

    # RPATH (so that libraries find library dependencies)
    if(${OPENSIM_USE_INSTALL_RPATH})
        # TODO @loader_path only makes sense on macOS, so we need to revisit
        # for Linux (use $ORIGIN).

        set(run_path_list "\@loader_path/")
        # TODO if/when Simbody and BTK libraries are installed in their own
        # directories (not beside OpenSim libraries), we will have to add the
        # following (also for BTK; this commented code has not been tested):
        #if(${OPENSIM_COPY_SIMBODY})
        #    file(RELATIVE_PATH lib_dir_to_install_dir
        #        "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
        #        "${CMAKE_INSTALL_PREFIX}")
        #    # The location of Simbody's libraries within the Simbody root.
        #    file(RELATIVE_PATH simbody_root_dir_to_simbody_lib_dir
        #         "${SIMBODY_ROOT_DIR}" "${SIMBODY_LIB_DIR}")
        #        ) 
        #    # The location of Simbody's libraries relative to OpenSim's
        #    # installation root (OPENSIM_INSTALL_SIMBODYDIR does not exist).
        #    file(install_dir_to_simbody_lib_dir
        #       "${OPENSIM_INSTALL_SIMBODYDIR}${simbody_root_dir_to_simbody_lib_dir}"
        #       )
        #    set(lib_dir_to_simbody_lib_dir
        #        "${lib_dir_to_install_dir}${install_dir_to_simbody_lib_dir}")
        #    list(APPEND run_path_list
        #           "\@loader_path/${lib_dir_to_simbody_lib_dir}")
        #endif()
        # It is possible that on UNIX (APPLE and Linux), we will install
        # Simbody and BTK beside OpenSim, rather than in their own directories.
        # Use APPEND to include the BTK RPATH, set in CMAKE_INSTALL_RPATH.
        set_property(TARGET ${OSIMADDLIB_LIBRARY_NAME} APPEND PROPERTY
            INSTALL_RPATH "${run_path_list}"
            )
    endif()

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
            target_include_directories(${TEST_NAME}
                PRIVATE ${OpenSim_SOURCE_DIR} ${OpenSim_SOURCE_DIR}/Vendors)
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
    include_directories(${OpenSim_SOURCE_DIR} ${OpenSim_SOURCE_DIR}/Vendors)
    add_executable(${OSIMADDAPP_NAME} ${OSIMADDAPP_NAME}.cpp
                                      ${OSIMADDAPP_SOURCES})
    target_link_libraries(${OSIMADDAPP_NAME} osimTools)
    set_target_properties(${OSIMADDAPP_NAME} PROPERTIES
        FOLDER "Applications")

    # Install.
    install(TARGETS ${OSIMADDAPP_NAME} DESTINATION ${CMAKE_INSTALL_BINDIR})

    # RPATH (so that the executable finds libraries without using env. vars).
    if(${OPENSIM_USE_INSTALL_RPATH})
        # TODO @executable_path only makes sense on macOS, so we need to revisit
        # for Linux (use $ORIGIN).

        # bin_dir_to_install_dir is most likely "../"
        file(RELATIVE_PATH bin_dir_to_install_dir
            "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
            "${CMAKE_INSTALL_PREFIX}")
        set(bin_dir_to_lib_dir
            "${bin_dir_to_install_dir}${CMAKE_INSTALL_LIBDIR}")
        set_property(TARGET ${OSIMADDAPP_NAME} APPEND PROPERTY
            INSTALL_RPATH "\@executable_path/${bin_dir_to_lib_dir}"
            )
    endif()

endfunction()


# Function to install shared libraries (any platform) from a dependency install
# directory into the OpenSim installation. One use case is to install libraries
# into the python package.
# PREFIX: A common part of the library file names (e.g., 'SimTK' or 'BTK').
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


# Function to copy DLL files from dependency install directory into OpenSim 
# build and install directories. This is a Windows specific function enabled 
# only for Windows platform. Intention is to allow runtime loader to find all 
# the required DLLs without need for editing PATH variable.
function(CopyDependencyDLLsForWin DEP_NAME DEP_INSTALL_DIR)
    # On Windows, copy dlls into OpenSim binary directory.
    if(WIN32)
        file(GLOB_RECURSE DLLS ${DEP_INSTALL_DIR}/*.dll)
        if(NOT DLLS)
            message(FATAL_ERROR "Zero DLLs found in directory "
                                "${DEP_INSTALL_DIR}.")
        endif()
        foreach(DLL IN LISTS DLLS)
            get_filename_component(DLL_NAME ${DLL} NAME)
            set(DEST_DIR ${CMAKE_BINARY_DIR}/${CMAKE_CFG_INTDIR})
            add_custom_command(OUTPUT ${DLL_NAME}
                               COMMAND ${CMAKE_COMMAND} -E copy ${DLL} ${DEST_DIR}
                               COMMENT "Copying ${DLL_NAME} to ${DEST_DIR}")
            list(APPEND DLL_NAMES ${DLL_NAME})
        endforeach()
        add_custom_target("Copy_${DEP_NAME}_DLLs" ALL DEPENDS ${DLL_NAMES})
        install(FILES ${DLLS} DESTINATION ${CMAKE_INSTALL_BINDIR})
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
    # Clean up the output, since it's in the form of a makefile
    # (and we just want a list of file paths).
    if(${_successfully_got_dependencies} EQUAL 0) # return code 0 is success.
        # '^.*:' matches the first line of the makefile (the output file path).
        # '\\\\' matches a single \ (escape for CMake, and escape for regex).
        string(REGEX REPLACE "(^.*:|\\\\\n)" "" ${OSIMSWIGDEP_RETURNVAL}
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

