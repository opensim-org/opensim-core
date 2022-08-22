# This file was copied from iCub and modified by chrisdembia to fix the library dependencies expected on Windows.

#.rst:
# FindIPOPT
# ---------
#
# Try to locate the IPOPT library
#
# On non Windows systems, use pkg-config to try to locate the library,
# if this fails then try to locate the library in the directory pointed by
# the IPOPT_DIR enviromental variable.
#
# On Windows systems,  just try to find the library using the IPOPT_DIR 
# enviromental variable.  
#
# Create the following variables::
#
#  IPOPT_INCLUDE_DIRS - Directories to include to use IPOPT
#  IPOPT_LIBRARIES    - Default library to link against to use IPOPT
#  IPOPT_DEFINITIONS  - Flags to be added to linker's options
#  IPOPT_LINK_FLAGS   - Flags to be added to linker's options
#  IPOPT_FOUND        - If false, don't try to use IPOPT

#=============================================================================
# Copyright (C) 2008-2010 RobotCub Consortium
# Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
#   Authors: Ugo Pattacini <ugo.pattacini@iit.it>
#   Authors: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# YCM - Extra CMake Modules for YARP and friends
# Copyright 2013-2014 iCub Facility, Istituto Italiano di Tecnologia
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# 
# * Neither the names of iCub Facility, Istituto Italiano di Tecnologia,
#   nor the names of their contributors may be used to endorse or promote
#   products derived from this software without specific prior written
#   permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# ------------------------------------------------------------------------------
# 
# The above copyright and license notice applies to distributions of
# YCM in source and binary form.  Some source files contain additional
# notices of original copyright by their contributors; see each source
# for details.  Third-party software packages supplied with YCM under
# compatible licenses provide their own copyright notices documented in
# corresponding subdirectories.
# 
# ------------------------------------------------------------------------------
# 
# YCM is being developed by the iCub Facility, Istituto Italiano di
# Tecnologia.
# 
# Additional support to YCM was received from the FP7 EU project
# WALK-MAN (http://walk-man.eu/)

if(NOT WIN32)
  # On non Windows systems we use PkgConfig to find IPOPT
  find_package(PkgConfig QUIET)
  if(PKG_CONFIG_FOUND)

    if(IPOPT_FIND_VERSION)
      if(IPOPT_FIND_VERSION_EXACT)
        pkg_check_modules(_PC_IPOPT QUIET ipopt=${IPOPT_FIND_VERSION})
      else()
        pkg_check_modules(_PC_IPOPT QUIET ipopt>=${IPOPT_FIND_VERSION})
      endif()
    else()
      pkg_check_modules(_PC_IPOPT QUIET ipopt)
    endif()


    if(_PC_IPOPT_FOUND)
      set(IPOPT_INCLUDE_DIRS ${_PC_IPOPT_INCLUDE_DIRS} CACHE PATH "IPOPT include directory")
      set(IPOPT_DEFINITIONS ${_PC_IPOPT_CFLAGS_OTHER} CACHE STRING "Additional compiler flags for IPOPT")
      set(IPOPT_LIBRARIES "" CACHE STRING "IPOPT libraries" FORCE)
      foreach(_LIBRARY IN ITEMS ${_PC_IPOPT_LIBRARIES})
        find_library(${_LIBRARY}_PATH
                     NAMES ${_LIBRARY}
                     PATHS ${_PC_IPOPT_LIBRARY_DIRS})
        list(APPEND IPOPT_LIBRARIES ${${_LIBRARY}_PATH})
      endforeach()
    else()
      set(IPOPT_DEFINITIONS "")
    endif()

  endif()

  set(IPOPT_LINK_FLAGS "")

  # If pkg-config fails, try to find the package using IPOPT_DIR
  if(NOT _PC_IPOPT_FOUND)
    set(IPOPT_DIR_TEST $ENV{IPOPT_DIR})
    if(IPOPT_DIR_TEST)
      set(IPOPT_DIR $ENV{IPOPT_DIR} CACHE PATH "Path to IPOPT build directory")
    else()
      set(IPOPT_DIR /usr            CACHE PATH "Path to IPOPT build directory")
    endif()

    set(IPOPT_INCLUDE_DIRS ${IPOPT_DIR}/include/coin)
    find_library(IPOPT_LIBRARIES ipopt ${IPOPT_DIR}/lib
                                     ${IPOPT_DIR}/lib/coin
                                     NO_DEFAULT_PATH)

    if(IPOPT_LIBRARIES)
      find_file(IPOPT_DEP_FILE ipopt_addlibs_cpp.txt ${IPOPT_DIR}/share/doc/coin/Ipopt
                                                     ${IPOPT_DIR}/share/coin/doc/Ipopt
                                                     NO_DEFAULT_PATH)
      mark_as_advanced(IPOPT_DEP_FILE)

      if(IPOPT_DEP_FILE)
        # parse the file and acquire the dependencies
        file(READ ${IPOPT_DEP_FILE} IPOPT_DEP)
        string(REGEX REPLACE "-[^l][^ ]* " "" IPOPT_DEP ${IPOPT_DEP})
        string(REPLACE "-l"                "" IPOPT_DEP ${IPOPT_DEP})
        string(REPLACE "\n"                "" IPOPT_DEP ${IPOPT_DEP})
        string(REPLACE "ipopt"             "" IPOPT_DEP ${IPOPT_DEP})       # remove any possible auto-dependency
        string(REPLACE "Accelerate"        "" IPOPT_DEP ${IPOPT_DEP})
        separate_arguments(IPOPT_DEP)

        # use the find_library command in order to prepare rpath correctly
        foreach(LIB ${IPOPT_DEP})
          find_library(IPOPT_SEARCH_FOR_${LIB} ${LIB} ${IPOPT_DIR}/lib
                                                      ${IPOPT_DIR}/lib/coin
                                                      ${IPOPT_DIR}/lib/coin/ThirdParty
                                                      NO_DEFAULT_PATH)
          if(IPOPT_SEARCH_FOR_${LIB})
            # handle non-system libraries (e.g. coinblas)
            set(IPOPT_LIBRARIES ${IPOPT_LIBRARIES} ${IPOPT_SEARCH_FOR_${LIB}})
          else()
            # handle system libraries (e.g. gfortran)
            set(IPOPT_LIBRARIES ${IPOPT_LIBRARIES} ${LIB})
          endif()
          mark_as_advanced(IPOPT_SEARCH_FOR_${LIB})
        endforeach()
      endif()
    endif()

    set(IPOPT_DEFINITIONS "")
    set(IPOPT_LINK_FLAGS "")
  endif()

# Windows platforms
else()
  include(SelectLibraryConfigurations)

  set(IPOPT_DIR $ENV{IPOPT_DIR} CACHE PATH "Path to IPOPT build directory")

  set(IPOPT_INCLUDE_DIRS ${IPOPT_DIR}/include/coin)
  find_library(IPOPT_IPOPT_LIBRARY_RELEASE libipopt ${IPOPT_DIR}/lib
                                                    ${IPOPT_DIR}/lib/coin
                                                    NO_DEFAULT_PATH)
  find_library(IPOPT_IPOPT_LIBRARY_DEBUG   libipoptD ${IPOPT_DIR}/lib
                                                     ${IPOPT_DIR}/lib/coin
                                                     NO_DEFAULT_PATH)

  select_library_configurations(IPOPT_IPOPT)
  set(IPOPT_LIBRARIES ${IPOPT_IPOPT_LIBRARY})


  # if(IPOPT_IPOPT_LIBRARY)
  #   find_file(IPOPT_DEP_FILE ipopt_addlibs_cpp.txt ${IPOPT_DIR}/share/doc/coin/Ipopt
  #                                                  ${IPOPT_DIR}/share/coin/doc/Ipopt
  #                                                  NO_DEFAULT_PATH)
  #   mark_as_advanced(IPOPT_DEP_FILE)

  #   if(IPOPT_DEP_FILE)
  #     # parse the file and acquire the dependencies
  #     file(READ ${IPOPT_DEP_FILE} IPOPT_DEP)
  #     string(REPLACE "libipopt.lib"      "" IPOPT_DEP ${IPOPT_DEP}) # Already taken care of.
  #     separate_arguments(IPOPT_DEP)
  #     foreach(dep ${IPOPT_DEP})
  #       # Only keep the items that are library names (ending with .lib).
  #       if(dep MATCHES "\\.lib$")
  #         string(REGEX REPLACE ".lib$" "" libname ${dep})
  #         list(APPEND IPOPT_DEP_LIBNAMES "${libname}")
  #       endif()
  #     endforeach()

  #     # use the find_library command in order to prepare rpath correctly
  #     foreach(LIB ${IPOPT_DEP_LIBNAMES})
  #       find_library(IPOPT_SEARCH_FOR_${LIB} ${LIB} ${IPOPT_DIR}/lib
  #                                                   ${IPOPT_DIR}/lib/coin
  #                                                   ${IPOPT_DIR}/lib/coin/ThirdParty
  #                                                   NO_DEFAULT_PATH)
  #       if(IPOPT_SEARCH_FOR_${LIB})
  #         # handle non-system libraries (e.g. coinblas)
  #         set(IPOPT_LIBRARIES ${IPOPT_LIBRARIES} ${IPOPT_SEARCH_FOR_${LIB}})
  #       else()
  #         # handle system libraries (e.g. gfortran)
  #         set(IPOPT_LIBRARIES ${IPOPT_LIBRARIES} ${LIB})
  #       endif()
  #       mark_as_advanced(IPOPT_SEARCH_FOR_${LIB})
  #     endforeach()
  #   endif()
  # endif()
  # Some old version of binary releases of IPOPT have Intel fortran
  # libraries embedded in the library, newer releases require them to
  # be explicitly linked.
  if(IPOPT_IPOPT_LIBRARY)
    # FIXME Remove this check when CMake 2.8.11 or later is required
    if(NOT CMAKE_VERSION VERSION_LESS 2.8.11)
      get_filename_component(_MSVC_BINDIR "${CMAKE_LINKER}" PATH)
    else()
      get_filename_component(_MSVC_DIR "${CMAKE_LINKER}" DIRECTORY)
    endif()

    # Find the lib.exe executable
    find_program(LIB_EXECUTABLE
                 NAMES lib.exe
                 HINTS "${_MSVC_BINDIR}"
                       "C:/Program Files/Microsoft Visual Studio 10.0/VC/bin"
                       "C:/Program Files (x86)/Microsoft Visual Studio 10.0/VC/bin"
                       "C:/Program Files/Microsoft Visual Studio 11.0/VC/bin"
                       "C:/Program Files (x86)/Microsoft Visual Studio 11.0/VC/bin"
                       "C:/Program Files/Microsoft Visual Studio 12.0/VC/bin"
                       "C:/Program Files (x86)/Microsoft Visual Studio 12.0/VC/bin"
                       "C:/Program Files/Microsoft Visual Studio 14.0/VC/bin"
                       "C:/Program Files (x86)/Microsoft Visual Studio 14.0/VC/bin"
                 DOC "Path to the lib.exe executable")
    mark_as_advanced(LIB_EXECUTABLE)

    # backup PATH environment variable
    set(_path $ENV{PATH})

    # Add th MSVC "Common7/IDE" dir containing the dlls in the PATH when needed.
    get_filename_component(_MSVC_LIBDIR "${_MSVC_BINDIR}/../../Common7/IDE" ABSOLUTE)
    if(NOT EXISTS "${_MSVC_LIBDIR}")
      get_filename_component(_MSVC_LIBDIR "${_MSVC_BINDIR}/../../../Common7/IDE" ABSOLUTE)
    endif()

    if(EXISTS "${_MSVC_LIBDIR}")
      set(_MSVC_LIBDIR_FOUND 0)
      file(TO_CMAKE_PATH "$ENV{PATH}" _env_path)
      foreach(_dir ${_env_path})
        if("${_dir}" STREQUAL ${_MSVC_LIBDIR})
          set(_MSVC_LIBDIR_FOUND 1)
        endif()
      endforeach()
      if(NOT _MSVC_LIBDIR_FOUND)
        file(TO_NATIVE_PATH "${_MSVC_LIBDIR}" _MSVC_LIBDIR)
        set(ENV{PATH} "$ENV{PATH};${_MSVC_LIBDIR}")
      endif()
    endif()

    if(IPOPT_IPOPT_LIBRARY_RELEASE)
      set(_IPOPT_LIB ${IPOPT_IPOPT_LIBRARY_RELEASE})
    else()
      set(_IPOPT_LIB ${IPOPT_IPOPT_LIBRARY_DEBUG})
    endif()

    execute_process(COMMAND ${LIB_EXECUTABLE} /list "${_IPOPT_LIB}"
                    OUTPUT_VARIABLE _lib_output)
    set(ENV{PATH} "${_path}")
    unset(_path)

    if(NOT "${_lib_output}" MATCHES "libifcoremd.dll")
      # FIXME Remove this check when CMake 2.8.11 or later is required
      if(NOT CMAKE_VERSION VERSION_LESS 2.8.11)
        get_filename_component(_IPOPT_IPOPT_LIBRARY_DIR "${_IPOPT_LIB}" PATH)
      else()
        get_filename_component(_IPOPT_IPOPT_LIBRARY_DIR "${_IPOPT_LIB}" DIRECTORY)
      endif()

      # The *mt variants (and libifport) should not be required; they are
      # probably for if we use a static runtime library, but we intend to use the
      # the dynamic runtime library (MSVC /MD flag).
      foreach(_lib ifconsol
                   libifcoremd
                   libifportmd
                   libmmd
                   libirc
                   svml_dispmd)
        string(TOUPPER "${_lib}" _LIB)
        find_library(IPOPT_${_LIB}_LIBRARY_RELEASE ${_lib} ${_IPOPT_IPOPT_LIBRARY_DIR})
        find_library(IPOPT_${_LIB}_LIBRARY_DEBUG ${_lib}d ${_IPOPT_IPOPT_LIBRARY_DIR})
        select_library_configurations(IPOPT_${_LIB})
	    list(APPEND IPOPT_LIBRARIES ${IPOPT_${_LIB}_LIBRARY})
      endforeach()
    endif()
  endif()

  set(IPOPT_DEFINITIONS "")
  if(MSVC)
    set(IPOPT_LINK_FLAGS "/NODEFAULTLIB:libcmt.lib;libcmtd.lib")
  else()
    set(IPOPT_LINK_FLAGS "")
  endif()

endif()

mark_as_advanced(IPOPT_INCLUDE_DIRS
                 IPOPT_LIBRARIES
                 IPOPT_DEFINITIONS
                 IPOPT_LINK_FLAGS)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(IPOPT DEFAULT_MSG IPOPT_LIBRARIES)
