# This CMake file implements the super-build procedure to download, configure,
# compile and install all OpenSim dependencies. Using super-build is optional.
# OpenSim does not use this file directly.

project(OpenSimDependencies)
cmake_minimum_required(VERSION 2.8)

include(ExternalProject)
include(CMakeParseArguments)

# Set the default for CMAKE_INSTALL_PREFIX.
function(SetDefaultCMakeInstallPrefix)
    get_filename_component(BASE_DIR ${CMAKE_BINARY_DIR} DIRECTORY)
    # Default install prefix for OpenSim dependencies. If user changes
    # CMAKE_INSTALL_PREFIX, this directory will be removed.
    set(DEFAULT_CMAKE_INSTALL_PREFIX 
        ${BASE_DIR}/opensim_dependencies_install
        CACHE
        INTERNAL
        "Default CMAKE_INSTALL_PREFIX for OpenSim dependencies.")

    if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        set(CMAKE_INSTALL_PREFIX 
            ${DEFAULT_CMAKE_INSTALL_PREFIX}
            CACHE
            PATH
            "Directory to install binaries of OpenSim dependencies."
            FORCE)
    endif()
endfunction()

# CMake doesn't clear prefix directories when user changes it. 
# Remove it to avoid confusion.
function(RemoveDefaultInstallDirIfEmpty DIR)
    file(GLOB CONTENTS ${DIR}/*)
    if(NOT CONTENTS)
        file(REMOVE_RECURSE ${DIR})
    endif()
endfunction()

# Set the default for CMAKE_BUILD_TYPE.
function(SetDefaultCMakeBuildType)
    # CMAKE_BUILD_TYPE is only applicable for single configuration generators.
    if(NOT CMAKE_CONFIGURATION_TYPES)
        set(DOCSTRING "Build type to use for dependencies. Possible values --")
        set(DOCSTRING "${DOCSTRING} Debug, Release, RelWithDebInfo,")
        set(DOCSTRING "${DOCSTRING} MinSizeRel.")

        set(CMAKE_BUILD_TYPE
            RelWithDebInfo
            CACHE
            STRING
            ${DOCSTRING})
    endif()
endfunction()

# Add a dependency. Arguments:
#   NAME       -- (Required) Name of the project.
#   URL        -- (Required) git repository to download the sources from.
#   TAG        -- (Required) git tag to checkout before commencing build.
#   CMAKE_ARGS -- (Optional) A CMake list of arguments to be passed to CMake 
#                 while building the project.
function(AddDependency)
    set(onevalueargs NAME URL TAG)
    set(multiValueArgs CMAKE_ARGS)
    cmake_parse_arguments(DEP "" "${onevalueargs}" "${multiValueArgs}" ${ARGN})

    # Check for presence of required arguments.
    if(NOT DEP_NAME OR NOT DEP_URL OR NOT DEP_TAG)
        set(MSG "One or more required arguments are missing. Please check the ")
        set(MSG "${MSG}AddDependency() call.")
        message(FATAL_ERROR ${MSG})
    endif()

    # Add a cache entry providing option for user to use (or not) superbuild.
    set(SUPERBUILD_${DEP_NAME} ON CACHE BOOL 
        "Automatically download, configure, build and install ${DEP_NAME}")

    if(SUPERBUILD_${DEP_NAME})
        set(SOURCE_DIR  ${CMAKE_SOURCE_DIR}/${DEP_NAME})
        set(BINARY_DIR  ${CMAKE_BINARY_DIR}/${DEP_NAME})
        set(INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/${DEP_NAME})

        set(DEFAULT_INSTALL_DIR ${DEFAULT_CMAKE_INSTALL_PREFIX}/${DEP_NAME})
        RemoveDefaultInstallDirIfEmpty(${DEFAULT_INSTALL_DIR})

        set(CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${INSTALL_DIR})
        if(NOT CMAKE_CONFIGURATION_TYPES)
            list(APPEND CMAKE_ARGS
                -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE})
        endif()
        list(APPEND CMAKE_ARGS ${DEP_CMAKE_ARGS})

        # Forward cmake arguments to dependencies.
        list(APPEND CMAKE_ARGS
             -DCMAKE_CXX_COMPILER:STRING=${CMAKE_CXX_COMPILER})
        list(APPEND CMAKE_ARGS
             -DCMAKE_C_COMPILER:STRING=${CMAKE_C_COMPILER})
        list(APPEND CMAKE_ARGS
             -DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS})
        list(APPEND CMAKE_ARGS
             -DCMAKE_CXX_FLAGS_DEBUG:STRING=${CMAKE_CXX_FLAGS_DEBUG})
        list(APPEND CMAKE_ARGS
             -DCMAKE_CXX_FLAGS_MINSIZEREL:STRING=${CMAKE_CXX_FLAGS_MINSIZEREL})
        list(APPEND CMAKE_ARGS
             -DCMAKE_CXX_FLAGS_RELEASE:STRING=${CMAKE_CXX_FLAGS_RELEASE})
        list(APPEND CMAKE_ARGS
             -DCMAKE_CXX_FLAGS_RELWITHDEBINFO:STRING=${CMAKE_CXX_FLAGS_RELWITHDEBINFO})
        list(APPEND CMAKE_ARGS
             -DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS})
        list(APPEND CMAKE_ARGS
             -DCMAKE_C_FLAGS_DEBUG:STRING=${CMAKE_C_FLAGS_DEBUG})
        list(APPEND CMAKE_ARGS
             -DCMAKE_C_FLAGS_MINSIZEREL:STRING=${CMAKE_C_FLAGS_MINSIZEREL})
        list(APPEND CMAKE_ARGS
             -DCMAKE_C_FLAGS_RELEASE:STRING=${CMAKE_C_FLAGS_RELEASE})
        list(APPEND CMAKE_ARGS
             -DCMAKE_C_FLAGS_RELWITHDEBINFO:STRING=${CMAKE_C_FLAGS_RELWITHDEBINFO})

        ExternalProject_Add(${DEP_NAME}
            TMP_DIR          ${BINARY_DIR}/tmp
            STAMP_DIR        ${BINARY_DIR}/stamp
            DOWNLOAD_DIR     ${SOURCE_DIR}
            GIT_REPOSITORY   ${DEP_URL}
            GIT_TAG          ${DEP_TAG}
            SOURCE_DIR       ${SOURCE_DIR}
            CMAKE_CACHE_ARGS ${CMAKE_ARGS}
            BINARY_DIR       ${BINARY_DIR}/build
            INSTALL_DIR      ${INSTALL_DIR})
    else()
        file(REMOVE_RECURSE ${CMAKE_BINARY_DIR}/${DEP_NAME})
        file(REMOVE_RECURSE ${CMAKE_INSTALL_PREFIX}/${DEP_NAME})
    endif()
endfunction()


SetDefaultCMakeInstallPrefix()
SetDefaultCMakeBuildType()

####################### Add dependencies below.

AddDependency(NAME       BTK
              # URL        https://github.com/Biomechanical-ToolKit/BTKCore.git
              URL        https://github.com/klshrinidhi/BTKCore.git
              TAG        ae0d2a43de0fe6c9ef2acce34a87c944dd113829
              CMAKE_ARGS -DBUILD_SHARED_LIBS:BOOL=ON)

AddDependency(NAME       simbody
              URL        https://github.com/simbody/simbody.git
              TAG        9f2123212dadcedcd680a8d8b3832d4fe40e1f8b
              CMAKE_ARGS -DBUILD_EXAMPLES:BOOL=OFF 
                         -DBUILD_TESTING:BOOL=OFF)
                     

AddDependency(NAME       docopt
              URL        https://github.com/docopt/docopt.cpp.git
              TAG        af03fa044ee1eff20819549b534ea86829a24a54)
 
#######################

RemoveDefaultInstallDirIfEmpty("${DEFAULT_CMAKE_INSTALL_PREFIX}")
