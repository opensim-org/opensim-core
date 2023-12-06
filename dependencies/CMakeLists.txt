# This CMake file implements the super-build procedure to download, configure,
# compile and install all OpenSim dependencies. Using super-build is optional.
# OpenSim does not use this file directly.

# We require a version of CMake that supports Visual Studio 2015.
cmake_minimum_required(VERSION 3.5)

project(OpenSimDependencies)

include(ExternalProject)
include(CMakeParseArguments)
include(CMakeDependentOption)

set(
    SIMBODY_EXTRA_CMAKE_ARGS ""
    CACHE STRING
    "extra arguments to forward when configuring Simbody's build. The format must be: VARNAME1:TYPE1=VALUE1;VARNAME2:TYPE2=VALUE2 (e.g. '-DSIMBODY_EXTRA_CMAKE_ARGS=-DSIMBODY_BUILD_VISUALIZER:BOOL=OFF;-DBUILD_USING_OTHER_LAPACK:PATH=/some/path')"
)

# Set the default for CMAKE_INSTALL_PREFIX.
function(SetDefaultCMakeInstallPrefix)
    get_filename_component(BASE_DIR ${CMAKE_SOURCE_DIR} DIRECTORY)
    # Move one directory up to the folder adjacent to the opensim-core folder.
    get_filename_component(BASE_DIR ${BASE_DIR} DIRECTORY)
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
#   DEFAULT    -- (Required) Default value for SUPERBUILD_${NAME} variable.
#   URL        -- (Required) URL for a zip or tar.gz file of the source code.
#   GIT_URL    -- (Required) git repository to download the sources from.
#   GIT_TAG    -- (Required) git tag to checkout before commencing build.
#   DEPENDS    -- (Optional) Other projects this project depends on.
#   CMAKE_ARGS -- (Optional) A CMake list of arguments to be passed to CMake 
#                 while building the project.
# You must provide either URL or GIT_URL and GIT_TAG, but not all 3.
function(AddDependency)
    set(onevalueargs NAME DEFAULT URL GIT_URL GIT_TAG)
    set(multiValueArgs DEPENDS CMAKE_ARGS)
    cmake_parse_arguments(DEP "" "${onevalueargs}" "${multiValueArgs}" ${ARGN})

    # Check for presence of required arguments.
    if(NOT (DEP_NAME AND ((DEP_GIT_URL AND DEP_GIT_TAG) OR DEP_URL)))
        set(MSG "One or more required arguments are missing. Please check the ")
        set(MSG "${MSG}AddDependency() call.")
        message(FATAL_ERROR ${MSG})
    endif()

    # Add a cache entry providing option for user to use (or not) superbuild.
    set(SUPERBUILD_${DEP_NAME} ${DEP_DEFAULT} CACHE BOOL
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
        if(APPLE)
            list(APPEND CMAKE_ARGS
                 -DCMAKE_OSX_DEPLOYMENT_TARGET:STRING=${CMAKE_OSX_DEPLOYMENT_TARGET})
        endif()
        if(SWIG_EXECUTABLE)
            list(APPEND CMAKE_ARGS -DSWIG_EXECUTABLE:FILEPATH=${SWIG_EXECUTABLE})
        endif()

        if(DEP_GIT_URL)
            ExternalProject_Add(${DEP_NAME}
                DEPENDS          ${DEP_DEPENDS}
                TMP_DIR          ${BINARY_DIR}/tmp
                STAMP_DIR        ${BINARY_DIR}/stamp
                GIT_REPOSITORY   ${DEP_GIT_URL}
                GIT_TAG          ${DEP_GIT_TAG}
                SOURCE_DIR       ${SOURCE_DIR}
                CMAKE_CACHE_ARGS ${CMAKE_ARGS}
                BINARY_DIR       ${BINARY_DIR}/build
                INSTALL_DIR      ${INSTALL_DIR})
        else()
            ExternalProject_Add(${DEP_NAME}
                DEPENDS          ${DEP_DEPENDS}
                TMP_DIR          ${BINARY_DIR}/tmp
                STAMP_DIR        ${BINARY_DIR}/stamp
                URL              ${DEP_URL}
                SOURCE_DIR       ${SOURCE_DIR}
                CMAKE_CACHE_ARGS ${CMAKE_ARGS}
                BINARY_DIR       ${BINARY_DIR}/build
                INSTALL_DIR      ${INSTALL_DIR})
        endif()
    else()
        file(REMOVE_RECURSE ${CMAKE_BINARY_DIR}/${DEP_NAME})
        file(REMOVE_RECURSE ${CMAKE_INSTALL_PREFIX}/${DEP_NAME})
    endif()
endfunction()


SetDefaultCMakeInstallPrefix()
SetDefaultCMakeBuildType()

####################### Add dependencies below.

AddDependency(NAME       ezc3d
              DEFAULT    OFF
              GIT_URL    https://github.com/pyomeca/ezc3d.git
              GIT_TAG    Release_1.5.6
              CMAKE_ARGS -DBUILD_EXAMPLE:BOOL=OFF)

AddDependency(NAME       simbody
              DEFAULT    ON
              GIT_URL    https://github.com/simbody/simbody.git
              GIT_TAG    930ae0feff0adb5aec184af62d14f9d138cacd48
              CMAKE_ARGS -DBUILD_EXAMPLES:BOOL=OFF
                         -DBUILD_TESTING:BOOL=OFF
                         ${SIMBODY_EXTRA_CMAKE_ARGS})

AddDependency(NAME       docopt
              DEFAULT    ON
              GIT_URL    https://github.com/docopt/docopt.cpp.git
              GIT_TAG    3dd23e3280f213bacefdf5fcb04857bf52e90917)


AddDependency(NAME       spdlog
              DEFAULT    ON
              GIT_URL    https://github.com/gabime/spdlog.git
              GIT_TAG    v1.4.1
              CMAKE_ARGS -DSPDLOG_BUILD_BENCH:BOOL=OFF
                         -DSPDLOG_BUILD_TESTS:BOOL=OFF
                         -DSPDLOG_BUILD_EXAMPLE:BOOL=OFF
                         -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON)
# Moco settings.
# --------------
option(OPENSIM_WITH_CASADI
        "Build CasADi support for Moco (MocoCasADiSolver)." OFF)
option(OPENSIM_WITH_TROPTER
        "Build the tropter optimal control library, for use in MocoTropterSolver." OFF)


if(OPENSIM_WITH_TROPTER OR OPENSIM_WITH_CASADI)
AddDependency(NAME    eigen
              DEFAULT ON
              URL     https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip)

mark_as_advanced(SUPERBUILD_eigen)

AddDependency(NAME colpack
              DEFAULT ON
              GIT_URL https://github.com/opensim-org/colpack.git
              GIT_TAG 72f691e91d59e8eb2123f258e67a4ddc72d105ee
              CMAKE_ARGS -DCMAKE_DEBUG_POSTFIX:STRING=_d
                         -DENABLE_EXAMPLES:BOOL=OFF
                         -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON)

mark_as_advanced(SUPERBUILD_colpack)

CMAKE_DEPENDENT_OPTION(SUPERBUILD_adolc "Automatically download, configure, build and install adolc" ON
                       "OPENSIM_WITH_TROPTER" OFF)
CMAKE_DEPENDENT_OPTION(SUPERBUILD_ipopt "Automatically download, configure, build and install ipopt" ON
                       "OPENSIM_WITH_TROPTER OR OPENSIM_WITH_CASADI" OFF)
endif()

if (OPENSIM_WITH_CASADI)
    CMAKE_DEPENDENT_OPTION(SUPERBUILD_casadi "Automatically download, configure, build and install casadi" ON
                           "OPENSIM_WITH_CASADI" OFF)
    mark_as_advanced(SUPERBUILD_casadi)
endif()

if (WIN32)

    if(SUPERBUILD_adolc)
        # ADOL-C: Download pre-built binaries.
        # TODO allow building from source, so we can use a Debug version of
        # ADOL-C.
        set(ADOLC_INSTALL_CMD "${CMAKE_COMMAND}" -E copy_directory
            "${CMAKE_BINARY_DIR}/ADOLC-prefix/src/ADOLC"
            "${CMAKE_INSTALL_PREFIX}/adol-c")
        ExternalProject_Add(adolc
            URL https://sourceforge.net/projects/myosin/files/adolc/adolc_sparse_x64.zip
            CONFIGURE_COMMAND ""
            BUILD_COMMAND ""
            INSTALL_COMMAND ${ADOLC_INSTALL_CMD})

        mark_as_advanced(SUPERBUILD_adolc)
    endif()


    if(SUPERBUILD_ipopt)
    
        # Ipopt: Download pre-built binaries built by chrisdembia.
        # This binary distribution comes with MUMPS and OpenBLAS.
        # Compiled with clang-cl 5.0 and flang, using conda.
        # Unfortunately, we do not have scripts to recreate this binary :(.
        set(IPOPT_INSTALL_CMD "${CMAKE_COMMAND}" -E copy_directory
            "${CMAKE_BINARY_DIR}/ipopt-prefix/src/ipopt"
            "${CMAKE_INSTALL_PREFIX}/ipopt")
        ExternalProject_Add(ipopt
            URL https://sourceforge.net/projects/myosin/files/ipopt/ipopt-mumps-flang-openblas-win64.zip
            CONFIGURE_COMMAND ""
            BUILD_COMMAND ""
            INSTALL_COMMAND ${IPOPT_INSTALL_CMD})

        # Previously attempted option:
        # Ipopt: Download pre-built binaries from the YARP/iCub project.
        # This binary distribution comes with MUMPS (which contains Fortran
        # code compiled with the proprietary Intel Fortran compiler).
        # set(IPOPT_INSTALL_CMD "${CMAKE_COMMAND}" -E copy_directory
            # "${CMAKE_BINARY_DIR}/ipopt-prefix/src/ipopt"
            # "${CMAKE_INSTALL_PREFIX}/ipopt")
        # ExternalProject_Add(ipopt
            # URL http://www.icub.org/download/3rd-party/ipopt-3.12.7_msvc14_x86_amd64.zip
            # CONFIGURE_COMMAND ""
            # BUILD_COMMAND ""
            # INSTALL_COMMAND ${IPOPT_INSTALL_CMD})

        # COIN-OR may make binaries available on GitHub (see release 3.13.2),
        # likely using Intel Fortran.
        # https://github.com/coin-or/Ipopt/releases
        mark_as_advanced(SUPERBUILD_ipopt)
    endif()
else()

    if(NOT XCODE)
        set(BUILD_FLAGS "-j4")
    endif()

    if(SUPERBUILD_adolc)
        # TODO copy source code into dependencies dir. OR make clean.
        # TODO rename to adol-c
        # TODO add debug symbols if building in Debug (but ADOL-C doesn't
        # compile with debug symbols right now).
        # TODO for macOS, will need to make ADOL-C change where it expects to
        # find ColPack's libraries (lib, not lib64).
        # https://github.com/Homebrew/homebrew-science/blob/master/adol-c.rb
        # TODO must have autoreconf, glibtoolize (brew install autoconf,
        # automake, libtool?)
        ExternalProject_Add(adolc
            DEPENDS           colpack
            GIT_REPOSITORY    https://github.com/coin-or/ADOL-C.git
            GIT_TAG           releases/2.6.3
            #SOURCE_DIR        "${CMAKE_SOURCE_DIR}/adol-c"
            #BINARY_DIR        "${CMAKE_SOURCE_DIR}/adol-c"
            INSTALL_DIR       "${CMAKE_INSTALL_PREFIX}/adol-c"
            CONFIGURE_COMMAND autoreconf -fi <SOURCE_DIR>
                COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR>
                                               --enable-sparse
                                               --with-colpack=${CMAKE_INSTALL_PREFIX}/colpack
                                               --with-boost=no
            BUILD_COMMAND ${CMAKE_MAKE_PROGRAM} ${BUILD_FLAGS}
            INSTALL_COMMAND ${CMAKE_MAKE_PROGRAM} install)
    endif()

    if(SUPERBUILD_ipopt)
        # TODO --enable-debug if building in Debug.
        # TODO must have gfortran for MUMPS (brew install gcc).
        # TODO CMake documentation says "Whether the current working directory
        # is preserved between commands is not defined. Behavior of shell
        # operators like && is not defined."
        # Patch the scripts that download Metis and MUMPS to use our
        # Sourceforge mirror. The original links are not reliable.
        ExternalProject_Add(ipopt
            URL https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.8.zip
            INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/ipopt"
            PATCH_COMMAND cd <SOURCE_DIR>/ThirdParty/Metis && patch < ${CMAKE_SOURCE_DIR}/get.Metis.patch
                  COMMAND cd <SOURCE_DIR>/ThirdParty/Metis && ./get.Metis
                  COMMAND cd <SOURCE_DIR>/ThirdParty/Mumps && patch < ${CMAKE_SOURCE_DIR}/get.Mumps.patch
                  COMMAND cd <SOURCE_DIR>/ThirdParty/Mumps && ./get.Mumps
            # GCC 10 generates a warning when compling MUMPS that we must suppress using
            # -fallow-argument-mismatch.
            # Suppress warnings treated as errors in Clang/LLVM with -Wno-error=implicit-function-declaration
            CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> ADD_FFLAGS=-fallow-argument-mismatch ADD_CFLAGS=-Wno-error=implicit-function-declaration
            BUILD_COMMAND ${CMAKE_MAKE_PROGRAM} ${BUILD_FLAGS}
            INSTALL_COMMAND ${CMAKE_MAKE_PROGRAM} install)
    endif()

endif()

if(SUPERBUILD_casadi)
    AddDependency(NAME       casadi
                  DEFAULT    ON
                  DEPENDS    ipopt
                  GIT_URL    https://github.com/casadi/casadi.git
                  # This commit is from casadi's develop branch, 2020-06-20
                  GIT_TAG    d074c221e22bd754f240f3597371418733e5efd4
                  CMAKE_ARGS -DWITH_IPOPT:BOOL=ON
                             -DWITH_THREAD:BOOL=ON
                             -DWITH_EXAMPLES:BOOL=OFF
                             -DPKG_CONFIG_USE_CMAKE_PREFIX_PATH:BOOL=ON
                             -DCMAKE_PREFIX_PATH:PATH=${CMAKE_INSTALL_PREFIX}/ipopt
                  )
endif()

#######################

RemoveDefaultInstallDirIfEmpty("${DEFAULT_CMAKE_INSTALL_PREFIX}")
