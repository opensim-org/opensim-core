# These CMake functions serve to reduce duplication across CMakeLists.txt files.

include(CMakeParseArguments)

# Copy DLL files from a dependency's installation into the
# build directory. This is a Windows-specific function enabled 
# only on Windows. The intention is to allow the runtime loader to find all 
# the required DLLs without editing the PATH environment variable.
# Arguments:
#   DEP_NAME: Name of the dependency (used to name a custom target)
#   DEP_BIN_DIR: The directory in the dependency containing DLLs to copy.
#   INSTALL_DLLS (optional): If provided, then the dependency's DLLs are
#     installed into ${INSTALL_DLLS}.
# This is based on a similar function in OpenSimMacros.cmake.
function(MocoCopyDLLs)
    # On Windows, copy dlls into the Tropter binary directory.
    set(options)
    set(oneValueArgs DEP_NAME DEP_BIN_DIR INSTALL_DLLS)
    set(multiValueArgs)
    cmake_parse_arguments(MOCOCOPY
            "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    if(WIN32)
        file(GLOB DLLS ${MOCOCOPY_DEP_BIN_DIR}/*.dll)
        if(NOT DLLS)
            message(FATAL_ERROR "Zero DLLs found in directory "
                                "${MOCOCOPY_DEP_BIN_DIR}.")
        endif()
        set(DEST_DIR "${CMAKE_BINARY_DIR}/${CMAKE_CFG_INTDIR}")
        foreach(DLL IN LISTS DLLS)
            get_filename_component(DLL_NAME ${DLL} NAME)
            list(APPEND DLLS_DEST "${DEST_DIR}/${DLL_NAME}")
            add_custom_command(OUTPUT "${DEST_DIR}/${DLL_NAME}"
                COMMAND ${CMAKE_COMMAND} -E make_directory ${DEST_DIR}
                COMMAND ${CMAKE_COMMAND} -E copy ${DLL} ${DEST_DIR}
                DEPENDS ${DLL}
                COMMENT "Copying ${DLL_NAME} from ${MOCOCOPY_DEP_BIN_DIR} to ${DEST_DIR}.")
        endforeach()
        add_custom_target(Copy_${MOCOCOPY_DEP_NAME}_DLLs ALL DEPENDS ${DLLS_DEST})
        set_target_properties(Copy_${MOCOCOPY_DEP_NAME}_DLLs PROPERTIES
            PROJECT_LABEL "Copy ${MOCOCOPY_DEP_NAME} DLLs" FOLDER "Moco")
        if(MOCOCOPY_INSTALL_DLLS)
            install(FILES ${DLLS} DESTINATION ${MOCOCOPY_INSTALL_DLLS})
        endif()
    endif()
endfunction()

# Add a target to the Moco project for building an example with the given
# NAME. The example must be within a source file named ${NAME}.cpp, or could
# contain multiple executable files, listed via the EXECUTABLES argument
# (omitting the .cpp extension). This function also installs the example files
# with a CMakeLists that can find the Moco installation and build the example.
#
# This function can only be used from the source distribution of Moco
# (e.g., not via the UseOpenSimMoco.cmake file in a binary distribution).
function(MocoAddExampleCXX)
    set(options)
    set(oneValueArgs NAME)
    set(multiValueArgs RESOURCES EXECUTABLES)
    cmake_parse_arguments(MOCOEX
            "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT MOCOEX_EXECUTABLES)
        set(MOCOEX_EXECUTABLES ${MOCOEX_NAME})
    endif()

    # Build the example in the build tree.
    foreach(exe ${MOCOEX_EXECUTABLES})
        add_executable(${exe} ${exe}.cpp)
        set_target_properties(${exe} PROPERTIES FOLDER "Moco/Examples")
        target_link_libraries(${exe} osimMoco)
    endforeach()
    file(COPY ${MOCOEX_RESOURCES} DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

    # Install files so that users can build the example.
    # We do not install pre-built binaries of the examples.
    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            DESTINATION ${OPENSIM_INSTALL_CPPEXDIR}
            PATTERN "CMakeLists.txt" EXCLUDE)

    set(_example_install_dir ${OPENSIM_INSTALL_CPPEXDIR}/${MOCOEX_NAME})
    # These next two variables are to be configured below (they are not used
    # here, but within ExampleCMakeListsToInstall.txt.in).
    set(_example_name ${MOCOEX_NAME})
    set(_example_executables ${MOCOEX_EXECUTABLES})
    file(RELATIVE_PATH _moco_install_hint
            "${CMAKE_INSTALL_PREFIX}/${_example_install_dir}"
            "${CMAKE_INSTALL_PREFIX}")
    configure_file(
            "${CMAKE_SOURCE_DIR}/Moco/Examples/C++/ExampleCMakeListsToInstall.txt.in"
            "${CMAKE_CURRENT_BINARY_DIR}/CMakeListsToInstall.txt"
            @ONLY)

    install(FILES
            "${CMAKE_CURRENT_BINARY_DIR}/CMakeListsToInstall.txt"
            DESTINATION ${_example_install_dir}
            RENAME CMakeLists.txt)
endfunction()

# Add a target to the Moco project for building an example containing a plugin
# library and an executable.
# The directory containing the CMakeLists.txt invoking this macro must have 3
# source files: <MAIN>.h and <MAIN>.cpp are used to create the plugin, and
# and exampleMAIN.cpp is compiled into an executable that uses the plugin.
# This function also installs the example file with a CMakeLists that can find
# the Moco installation and build the example.
#
# This function can only be used from the source distribution of Moco
# (e.g., not via the UseOpenSimMoco.cmake file in a binary distribution).
function(MocoAddPluginExampleCXX)
    set(options)
    set(oneValueArgs NAME)
    set(multiValueArgs RESOURCES)
    cmake_parse_arguments(MOCOEX
            "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    add_library(osim${MOCOEX_NAME} SHARED
            ${MOCOEX_NAME}.h
            ${MOCOEX_NAME}.cpp
            osim${MOCOEX_NAME}DLL.h
            RegisterTypes_osim${MOCOEX_NAME}.h
            RegisterTypes_osim${MOCOEX_NAME}.cpp
            )
    set_target_properties(osim${MOCOEX_NAME} PROPERTIES
            FOLDER "Moco/Examples")
    target_link_libraries(osim${MOCOEX_NAME} osimMoco)

    string(TOUPPER ${MOCOEX_NAME} _example_name_upper)
    set_target_properties(osim${MOCOEX_NAME} PROPERTIES
            DEFINE_SYMBOL OSIM${_example_name_upper}_EXPORTS
            )

    add_executable(example${MOCOEX_NAME} example${MOCOEX_NAME}.cpp)
    set_target_properties(example${MOCOEX_NAME} PROPERTIES
            FOLDER "Moco/Examples")
    target_link_libraries(example${MOCOEX_NAME} osim${MOCOEX_NAME})
    file(COPY ${MOCOEX_RESOURCES} DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

    install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            DESTINATION ${OPENSIM_INSTALL_CPPEXDIR}/Plugins
            PATTERN "CMakeLists.txt" EXCLUDE)

    set(_example_install_dir
            ${OPENSIM_INSTALL_CPPEXDIR}/Plugins/example${MOCOEX_NAME})
    # These next two variables are to be configured below (they are not used
    # here, but within ExampleCMakeListsToInstall.txt.in).
    set(_example_name ${MOCOEX_NAME})
    file(RELATIVE_PATH _moco_install_hint
            "${CMAKE_INSTALL_PREFIX}/${_example_install_dir}"
            "${CMAKE_INSTALL_PREFIX}")
    configure_file(
            "${CMAKE_SOURCE_DIR}/Moco/Examples/C++/PluginExampleCMakeListsToInstall.txt.in"
            "${CMAKE_CURRENT_BINARY_DIR}/CMakeListsToInstall.txt"
            @ONLY)

    install(FILES
            "${CMAKE_CURRENT_BINARY_DIR}/CMakeListsToInstall.txt"
            DESTINATION ${_example_install_dir}
            RENAME CMakeLists.txt)
endfunction()

# Add a relative path to the run-path of a shared library. The run-path allows
# a library to find the libraries it depends on without the need to set the
# (DY)LD_LIBRARY_PATH environment variable. This function only affects UNIX
# systems. Here are the arguments:
#
# TARGET: The CMake target to which the run-path should be added.
# EXECUTABLE, LOADER: Specify one of these flags to indicate whether the
#   run-path is relative to the executable's location or relative to the loader
#   (the library/executable that attempts to load the dependent library). Only
#   affects Mac.
# FROM: The path (relative to the Moco installation root) of the
#   executable/library that is loading a dependent library.
# TO: The path (relative to the Moco installation root) where the dependent
#   library is located.
function(MocoAddInstallRPATH)
    set(options EXECUTABLE LOADER)
    set(oneValueArgs TARGET FROM TO)
    set(multiValueArgs)
    cmake_parse_arguments(
            MOCORP "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    if(MOCORP_EXECUTABLE EQUAL MOCORP_LOADER)
        message(AUTHOR_WARNING "Cannot specify both EXECUTABLE and LOADER.")
    endif()

    if(APPLE)
        if(MOCORP_EXECUTABLE)
            set(rpath_macro "\@executable_path")
        elseif(MOCORP_LOADER)
            set(rpath_macro "\@loader_path")
        endif()
    elseif(UNIX)
        set(rpath_macro "\$ORIGIN")
    endif()

    file(RELATIVE_PATH to_root "${CMAKE_INSTALL_PREFIX}/${MOCORP_FROM}"
            "${CMAKE_INSTALL_PREFIX}")
    set_property(TARGET ${MOCORP_TARGET} APPEND PROPERTY INSTALL_RPATH
            "${rpath_macro}/${to_root}${MOCORP_TO}")
endfunction()

# Similar to MocoAddInstallRPATH except the run-path is the same directory
# loader (e.g., "@loader_path/"). Arguments:
#
# TARGET: The CMake target to which the run-path should be added.
function(MocoAddInstallRPATHSelf)
    set(options)
    set(oneValueArgs TARGET)
    set(multiValueArgs)
    cmake_parse_arguments(
            MOCORP "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # By specifing "" and "" for FROM and TO, the resulting RPATH will simply
    # be "${rpath_macro}/".
    MocoAddInstallRPATH(TARGET ${MOCORP_TARGET} LOADER FROM "" TO "")

endfunction()

# Similar to MocoAddInstallRPATH, but specifically for adding a run-path to
# the Simbody libraries copied into the Moco installation. If Simbody is not
# copied into the Moco installation (OPENSIM_COPY_DEPENDENCIES=OFF) or the
# copied Simbody libraries are installed to the same folder as the Moco
# libraries (OPENSIM_INSTALL_UNIX_FHS=ON), then this function does nothing.
#
# TARGET: The CMake target to which the run-path should be added.
# ABSOLUTE: Add an absolute run-path to the Simbody libraries in the Moco
#   installation. If you specify this argument, then you should not specify
#   EXECTUABLE, LOADER, or FROM.
# EXECUTABLE, LOADER: Specify one of these flags to indicate whether the
#   run-path is relative to the executable's location or relative to the loader
#   (the library/executable that attempts to load the dependent library). Only
#   affects Mac.
# FROM: The path (relative to the Moco installation root) of the
#   executable/library that is loading a dependent Simbody library.
function(MocoAddInstallRPATHSimbody)
    set(options EXECUTABLE LOADER ABSOLUTE)
    set(oneValueArgs TARGET FROM)
    set(multiValueArgs)
    cmake_parse_arguments(
            MOCORP "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    if(MOCORP_ABSOLUTE AND (MOCORP_FROM OR MOCORP_EXECUTABLE OR MOCORP_LOADER))
        message(AUTHOR_WARNING
                "Cannot specify both ABSOLUTE and FROM, EXECUTABLE, or LOADER.")
    endif()

    if(NOT OPENSIM_COPY_DEPENDENCIES OR OPENSIM_INSTALL_UNIX_FHS)
        return()
    endif()

    file(RELATIVE_PATH simbody_root_to_simbody_lib_dir
            "${Simbody_ROOT_DIR}" "${Simbody_LIB_DIR}")
    set(root_to_simbody_lib_dir
            "${OPENSIM_INSTALL_SIMBODYDIR}/${simbody_root_to_simbody_lib_dir}")

    if(MOCORP_ABSOLUTE)
        set_property(TARGET ${MOCORP_TARGET} APPEND PROPERTY INSTALL_RPATH
                "${CMAKE_INSTALL_PREFIX}/${root_to_simbody_lib_dir}")
    else()
        if(MOCORP_EXECUTABLE)
            MocoAddInstallRPATH(TARGET ${MOCORP_TARGET} EXECUTABLE
                    FROM ${MOCORP_FROM} TO ${root_to_simbody_lib_dir})
        elseif(MOCORP_LOADER)
            MocoAddInstallRPATH(TARGET ${MOCORP_TARGET} LOADER
                    FROM ${MOCORP_FROM} TO ${root_to_simbody_lib_dir})
        endif()
    endif()
endfunction()

# Similar to MocoAddInstallRPATH, but specifically for adding an absolute
# run-path.
#
# TARGET: The CMake target to which the run-path should be added.
# TO: The path (relative to the Moco installation root) where the dependent
#   library is located.
function(MocoAddInstallRPATHAbsolute)
    set(options)
    set(oneValueArgs TARGET TO)
    set(multiValueArgs)
    cmake_parse_arguments(
            MOCORP "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    set_property(TARGET ${MOCORP_TARGET} APPEND PROPERTY INSTALL_RPATH
            "${CMAKE_INSTALL_PREFIX}/${MOCORP_TO}")
endfunction()
