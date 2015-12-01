cmake_minimum_required(VERSION 2.8)

include(ExternalProject)
include(CMakeParseArguments)

# Set the default for CMAKE_INSTALL_PREFIX.
function(SetDefaultCMakeInstallPrefix)
    get_filename_component(BASE_DIR ${CMAKE_BINARY_DIR} DIRECTORY)
    set(CMAKE_INSTALL_PREFIX 
        ${BASE_DIR}/opensim_dependencies-install 
        CACHE
        PATH
        "Directory to install binaries of OpenSim dependencies."
        FORCE)
endfunction()

# Set the default for CMAKE_BUILD_TYPE.
function(SetDefaultCMakeBuildType)
    set(DOCSTRING "Build type to use for dependencies. Possible values --")
    set(DOCSTRING "${DOCSTRING} Debug, Release, RelWithDebInfo, MinSizeRel.")

    set(CMAKE_BUILD_TYPE
        RelWithDebInfo
        CACHE
        STRING
        ${DOCSTRING}
        FORCE)
endfunction()

# Add a dependency. Arguments:
#   NAME       -- (Required) Name of the project.
#   URL        -- (Required) git repository to download the sources from.
#   TAG        -- (Required) git tag to checkout before commencing build.
#   CMAKE_ARGS -- (Optional) A CMake list of arguments to be passed to CMake 
#                 while building the project.
function(AddDependency)
    set(onevalueargs NAME URL TAG CMAKE_ARGS)
    cmake_parse_arguments(DEP "" "${onevalueargs}" "" ${ARGN})

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

        set(CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${INSTALL_DIR}
                       -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE})
        list(APPEND CMAKE_ARGS ${DEP_CMAKE_ARGS})

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
    endif()
endfunction()


SetDefaultCMakeInstallPrefix()
SetDefaultCMakeBuildType()

####################### Add dependencies below.

AddDependency(NAME BTK
              URL  https://github.com/Biomechanical-ToolKit/BTKCore.git
              TAG  2c39651138d53537d4bfb72238cf5b085dd3ccf7)


AddDependency(NAME simbody
              URL  https://github.com/simbody/simbody.git
              TAG  ed67885db8983405e0724e63e62bf8e9316b2493)
