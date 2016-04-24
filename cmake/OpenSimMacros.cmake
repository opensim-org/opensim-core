include(CMakeParseArguments)

# Create an OpenSim API library. Here are the arguments:
# VENDORLIB: If this is a vendor library, specify "VENDORLIB" as the first
#   argument. Otherwise, omit.
# LOWERINCLUDEDIRNAME: When installing the headers for this library, make the
#   name of the library all lower-case (e.g., Lepton -> lepton).
# UNITY_EXCLUDE: Defined if UNITY_EXCLUDED_SOURCES is specified.
# KIT: Name of the library (e.g., Common).
# AUTHORS: A string listing authors of the library.
# LINKLIBS: List of libraries (targets) to link against.
# INCLUDES: List of header files for the library (obtain via file(GLOB ...)).
# SOURCES: List of cpp files for the library (obtain via file(GLOB ...)).
# TESTDIRS: List of subdirectories that contain tests (and a CMakeLists.txt).
# UNITY_EXCLUDED_SOURCES: List of sources to exclude from unity build in 
#   case of conflicts.
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
#       UNITY_EXCLUDED_SOURCES ${EXCLUDED_SOURCES}
#       )
function(OpenSimAddLibrary)

    # Parse arguments.
    # ----------------
    # http://www.cmake.org/cmake/help/v2.8.9/cmake.html#module:CMakeParseArguments
    set(options VENDORLIB LOWERINCLUDEDIRNAME UNITY_EXCLUDE)
    set(oneValueArgs KIT AUTHORS)
    set(multiValueArgs LINKLIBS INCLUDES SOURCES TESTDIRS UNITY_EXCLUDED_SOURCES INCLUDEDIRS)
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

    # Unity Build
    if(USE_UNITY_BUILD)
        if(OSIMADDLIB_UNITY_EXCLUDE)
            OsimUnityBuild(
                EXCLUDE_FROM_SOURCES
                UNIT_SUFFIX ${OSIMADDLIB_LIBRARY_NAME}
                PROJECT_SOURCES OSIMADDLIB_SOURCES
                EXCLUDED_SOURCES ${OSIMADDLIB_UNITY_EXCLUDED_SOURCES}
            )
        else()
            OsimUnityBuild(
                UNIT_SUFFIX ${OSIMADDLIB_LIBRARY_NAME}
                PROJECT_SOURCES OSIMADDLIB_SOURCES
            )
        endif()
    endif()

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
# SOURCES: Extra source files for the exectuable.
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
# APPNAME: Name of the application. Must also be the name of the source file
# containing main().
#
# Here's an example:
#   OpenSimAddApplication(forward)
function(OpenSimAddApplication APPNAME)

    include_directories(${OpenSim_SOURCE_DIR} ${OpenSim_SOURCE_DIR}/Vendors)
    add_executable(${APPNAME} ${APPNAME}.cpp)
    target_link_libraries(${APPNAME} osimTools)
    install(TARGETS ${APPNAME} DESTINATION "${CMAKE_INSTALL_BINDIR}")
    set_target_properties(${APPNAME} PROPERTIES
        FOLDER "Applications")

    if(${OPENSIM_USE_INSTALL_RPATH})
        # TODO @executable_path only makes sense on OSX, so if we use RPATH on
        # Linux we'll have to revisit.

        # bin_dir_to_install_dir is most likely "../"
        file(RELATIVE_PATH bin_dir_to_install_dir
            "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
            "${CMAKE_INSTALL_PREFIX}")
        set(bin_dir_to_lib_dir
            "${bin_dir_to_install_dir}${CMAKE_INSTALL_LIBDIR}")
        set_target_properties(${APPNAME} PROPERTIES
            INSTALL_RPATH "\@executable_path/${bin_dir_to_lib_dir}"
            )
    endif()

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
                               COMMAND cmake -E copy ${DLL} ${DEST_DIR}
                               COMMENT "Copying ${DLL_NAME} to ${DEST_DIR}")
            list(APPEND DLL_NAMES ${DLL_NAME})
        endforeach()
        add_custom_target("Copy_${DEP_NAME}_DLLs" ALL DEPENDS ${DLL_NAMES})
        install(FILES ${DLLS} DESTINATION ${CMAKE_INSTALL_BINDIR})
    endif()
endfunction()


# Create unity build
# EXCLUDE_FROM_SOURCES: Defined if EXCLUDED_SOURCES are provided.
# PROJECT_SOURCES: The sources that will be compiled into a single unit.
#   Should be provided by reference (e.g. SOURCES, not ${SOURCES}).
# EXCLUDED_SOURCES: List of sources to exclude from unity build in 
#   case of conflicts.
#
# Example:
#       OsimUnityBuild(
#           EXCLUDE_FROM_SOURCES
#           PROJECT_SOURCES sources
#           EXCLUDED_SOURCES ${excluded}
#       )
function(OsimUnityBuild)

    # Parse arguments.
    # ----------------
    # http://www.cmake.org/cmake/help/v2.8.9/cmake.html#module:CMakeParseArguments
    set(options EXCLUDE_FROM_SOURCES)
    set(oneValueArgs UNIT_SUFFIX)
    set(multiValueArgs PROJECT_SOURCES EXCLUDED_SOURCES)
    cmake_parse_arguments(
        OSIMUNITYBUILD "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    
    set(files ${${OSIMUNITYBUILD_PROJECT_SOURCES}})
    
    if (OSIMUNITYBUILD_EXCLUDE_FROM_SOURCES)
        list(REMOVE_ITEM files ${OSIMUNITYBUILD_EXCLUDED_SOURCES})
    endif()
    
    # Generate a unique filename for the unity build translation unit
    set(unit_build_file ${CMAKE_CURRENT_BINARY_DIR}/${OSIMUNITYBUILD_UNIT_SUFFIX}_UnityBuild.cpp)
    
    # Exclude all translation units from compilation
    set_source_files_properties(${files} PROPERTIES HEADER_FILE_ONLY true)
    
    # Open the unity build file
    file(WRITE ${unit_build_file} "// Unity Build generated by CMake\n")
    
    # Add include statement for each translation unit
    foreach(source_file ${files} )
        file( APPEND ${unit_build_file} "#include <${source_file}>\n")
    endforeach(source_file)
    
    # Complement list of translation units with the name of ub
    set(${OSIMUNITYBUILD_PROJECT_SOURCES} ${${OSIMUNITYBUILD_PROJECT_SOURCES}} ${unit_build_file} PARENT_SCOPE)  
    
endfunction()