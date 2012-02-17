# FindSimbody.cmake
#
# Try to find the Simbody multibody dynamics package that is part of the
# SimTK biosimulation toolkit. This includes several libraries:
#     libSimTKsimbody
#     libSimTKmath
#     libSimTKcommon
#     libSimTKlapack
#     (and on Windows pthreads too)
# We expect to find all of these in the same place.
#
# This module looks for certain CMake variables on input and behaves 
# accordingly if they are present:
#
#   SimTK_SDK 	
#       If this is set (probably by the nightly build system) then that
#       will be the only place we'll look for Simbody. Otherwise we'll 
#       hunt around.
#   BUILD_USING_NAMESPACE
#       This is used to look for Simbody libraries that are prefixed by
#       a string. An underscore is appended to the "namespace".
#   BUILD_UNVERSIONED_LIBRARIES
#       If this is set we look for the non-versioned Simbody libraries.
#   BUILD_VERSIONED_LIBRARIES
#       If this is set we'll look for Simbody libraries that have the same
#       version number as the version of Molmodel we're building.
#
# Note that both versioned and unversioned libraries may be produced by the
# same build so we have to allow for both the above to be set.
#
# Created January 2011 by Michael Sherman
# Simbios National Center for Physics Based Simulation of Biological Structures
# Stanford University
#
# Once done this will define:
#
#   Simbody_FOUND - Whether search for Simbody libraries and headers succeeded.
#   Simbody_ROOT_DIR - the installation directory; all the pieces must be
#                      found together
#   Simbody_INCLUDE_DIR - location of Simbody.h
#   Simbody_LIB_DIR     - location of libSimbody.{a,so,dylib} or Simbody.lib
#   Simbody_BIN_DIR     - location of VisualizerGUI and .dll's on Windows
#   Simbody_LIBRARIES   - suitable for target_link_libraries(); includes
#                           both optimized and debug libraries if both are
#                           available
#   Simbody_STATIC_LIBRARIES - suitable for target_link_libraries(); includes
#                              both optimized and debug static libraries if
#                              both are available
#   Simbody_LIBRARIES_VN - version numbered alternative to Simbody_LIBRARIES
#   Simbody_STATIC_LIBRARIES_VN - version numbered alternative to 
#                                 Simbody_STATIC_LIBRARIES
#
# == Using Simbody ==
#
#     find_package(Simbody)
#     if(Simbody_FOUND)
#         include_directories(${Simbody_INCLUDE_DIR})
#         link_directories(${Simbody_LIB_DIR})
#         target_link_libraries(Foo ${Simbody_LIBRARIES})
#     endif()
#

cmake_minimum_required(VERSION 2.8)

set(NS)
if(BUILD_USING_NAMESPACE)
    set(NS "${BUILD_USING_NAMESPACE}_")
endif(BUILD_USING_NAMESPACE)

set(VN)
if(BUILD_VERSIONED_LIBRARIES)
    set(VN "_${MOLMODEL_VERSION}")
endif()

if (SimTK_SDK)
    # This is the SimTK.org nightly builds location.
    set(Simbody_SEARCH_PATHS ${SimTK_SDK})
else()
    set(Simbody_SEARCH_PATHS)
    # If the SimTK_INSTALL_DIR environment variable has been set, we'll
    # look for Simbody there.
#    set(INSTDIR $ENV{SimTK_INSTALL_DIR})
    if (SimTK_INSTALL_DIR)
        set(Simbody_SEARCH_PATHS "${SimTK_INSTALL_DIR}")
    endif()

    # UNIX includes Mac, Linux, and Cygwin
    if (UNIX)
        list(APPEND Simbody_SEARCH_PATHS /usr/local)
    endif()
    
    if (APPLE) # Mac only
        list(APPEND Simbody_SEARCH_PATHS /Developer)
    endif()
    
    # WIN32 includes Windows 32 & 64 bit, and Cygwin
    if (WIN32)
        if( ${CMAKE_SIZEOF_VOID_P} EQUAL 8 )
	    # 64 bit target on Win64
	    set(PROGFILE_DIR "$ENV{ProgramW6432}")
        else() # Target is 32 bit
	    # present if 64bit Windows
	    set(PROGFILE_DIR "$ENV{ProgramFiles(x86)}") 
	    if (NOT PROGFILE_DIR)
	        set(PROGFILE_DIR "$ENV{ProgramFiles}") # on 32bit Windows
	    endif()
        endif()
        list(APPEND Simbody_SEARCH_PATHS ${PROGFILE_DIR})
    endif()
endif(SimTK_SDK)

# We'll use the main Simbody header as the key to finding the installation
# root directory. We're assuming the header is in 
#    Simbody_INCLUDE_DIR == ${Simbody_ROOT_DIR}/include
# So stripping off the "/include" should give the root directory.

# Force this to be recalculated every time.
set(Simbody_INCLUDE_DIR "Simbody_INCLUDE_DIR-NOTFOUND" CACHE PATH
    "The Simbody and SimTK include directory." FORCE)

foreach(pth IN LISTS Simbody_SEARCH_PATHS)
  find_path(Simbody_INCLUDE_DIR 
    NAMES "SimTKsimbody.h" "Simbody.h"
    PATHS "${pth}"
    PATH_SUFFIXES "include" "SimTK/include" "simtk/include"
                  "Simbody/include" "simbody/include"
    NO_DEFAULT_PATH
    DOC "Location of top-level installed Simbody header files"
  )
endforeach()

# This will only be executed if the first loop fails. We're getting
# desperate!
find_path(Simbody_INCLUDE_DIR 
    NAMES "SimTKsimbody.h" "Simbody.h"
    PATH_SUFFIXES "include" "SimTK/include" "simtk/include"
                  "Simbody/include" "simbody/include"
    DOC "Location of top-level installed Simbody header files"
)

get_filename_component(Simbody_ROOT_DIR_TEMP "${Simbody_INCLUDE_DIR}" PATH)
set(Simbody_ROOT_DIR "${Simbody_ROOT_DIR_TEMP}" CACHE PATH
    "Where we found Simbody; use SimTK_INSTALL_DIR to change." FORCE)

set(Simbody_LIB_DIR ${Simbody_ROOT_DIR}/lib${LIB64} CACHE PATH
    "Location of Simbody and SimTK libraries." FORCE)
set(Simbody_BIN_DIR ${Simbody_ROOT_DIR}/bin CACHE PATH
    "Location of Simbody-related executables and (on Windows) dlls." FORCE)

set(Simbody_LIBRARY_LIST ${NS}SimTKsimbody;${NS}SimTKmath;${NS}SimTKcommon)

if (WIN32)
    set(Simbody_LAPACK_LIBRARY_LIST SimTKlapack)
else()
    set(Simbody_LAPACK_LIBRARY_LIST lapack;blas)
endif()


if (WIN32)
    set(Simbody_EXTRA_LIBRARY_LIST pthreadVC2)
elseif (APPLE)
    set(Simbody_EXTRA_LIBRARY_LIST pthread;dl)
else()
    set(Simbody_EXTRA_LIBRARY_LIST pthread;rt;dl)
endif()


#########################
# UNVERSIONED LIBRARIES #
#########################

if (BUILD_UNVERSIONED_LIBRARIES)
# Find out which of the unversioned libraries are available.
find_library(Simbody_LIBRARY NAMES ${NS}SimTKsimbody
    PATHS ${Simbody_LIB_DIR}
    DOC "This is the main Simbody library."
    NO_DEFAULT_PATH)
find_library(Simbody_STATIC_LIBRARY NAMES ${NS}SimTKsimbody_static
    PATHS ${Simbody_LIB_DIR}
    DOC "This is the main Simbody static library."
    NO_DEFAULT_PATH)
find_library(Simbody_DEBUG_LIBRARY NAMES ${NS}SimTKsimbody_d
    PATHS ${Simbody_LIB_DIR}
    DOC "This is the main Simbody debug library."
    NO_DEFAULT_PATH)
find_library(Simbody_STATIC_DEBUG_LIBRARY NAMES ${NS}SimTKsimbody_static_d
    PATHS ${Simbody_LIB_DIR}
    DOC "This is the main Simbody static debug library."
    NO_DEFAULT_PATH)


# Set composite Simbody_LIBRARIES variable
set(LIBS)
if(Simbody_LIBRARY AND Simbody_DEBUG_LIBRARY)
    foreach(libname IN LISTS Simbody_LIBRARY_LIST)
        set(LIBS ${LIBS} optimized "${libname}" debug "${libname}_d")
    endforeach()
elseif(Simbody_LIBRARY)
    foreach(libname IN LISTS Simbody_LIBRARY_LIST)
        set(LIBS ${LIBS} "${libname}")
    endforeach()
elseif(Simbody_DEBUG_LIBRARY)
    foreach(libname IN LISTS Simbody_LIBRARY_LIST)
        set(LIBS ${LIBS} "${libname}_d")
    endforeach()
endif()

if (LIBS)
    foreach(lapack_lib IN LISTS Simbody_LAPACK_LIBRARY_LIST)
        set(LIBS ${LIBS} "${lapack_lib}")
    endforeach()
    foreach(extra_lib IN LISTS Simbody_EXTRA_LIBRARY_LIST)
        set(LIBS ${LIBS} "${extra_lib}")
    endforeach()
    set(Simbody_LIBRARIES ${LIBS} CACHE STRING 
        "Simbody dynamic libraries" FORCE)
else()
    set(Simbody_LIBRARIES Simbody_LIBRARIES-NOTFOUND CACHE STRING 
        "Simbody dynamic libraries" FORCE)
endif()

# Static library
set(LIBS)
if(Simbody_STATIC_LIBRARY AND Simbody_STATIC_DEBUG_LIBRARY)
    foreach(libname IN LISTS Simbody_LIBRARY_LIST)
        set(LIBS ${LIBS} optimized "${libname}_static" 
		         debug     "${libname}_static_d")
    endforeach()
elseif(Simbody_STATIC_LIBRARY)
    foreach(libname IN LISTS Simbody_LIBRARY_LIST)
        set(LIBS ${LIBS} "${libname}_static")
    endforeach()
elseif(Simbody_STATIC_DEBUG_LIBRARY)
    foreach(libname IN LISTS Simbody_LIBRARY_LIST)
        set(LIBS ${LIBS} "${libname}_static_d")
    endforeach()
endif()

if (LIBS)
    # these aren't available in static
    foreach(lapack_lib IN LISTS Simbody_LAPACK_LIBRARY_LIST)
        set(LIBS ${LIBS} "${lapack_lib}")
    endforeach()
    foreach(extra_lib IN LISTS Simbody_EXTRA_LIBRARY_LIST)
        set(LIBS ${LIBS} "${extra_lib}")
    endforeach()
    set(Simbody_STATIC_LIBRARIES "${LIBS}" CACHE STRING 
        "Simbody static libraries" FORCE)
else()
    set(Simbody_STATIC_LIBRARIES Simbody_STATIC_LIBRARIES-NOTFOUND CACHE STRING 
        "Simbody static libraries" FORCE)
endif()
endif(BUILD_UNVERSIONED_LIBRARIES)


#######################
# VERSIONED LIBRARIES #
#######################

if (BUILD_VERSIONED_LIBRARIES)
# Find out which of the versioned libraries are available.
find_library(Simbody_LIBRARY_VN NAMES ${NS}SimTKsimbody${VN}
    PATHS ${Simbody_LIB_DIR}
    DOC "This is the main Simbody library (versioned)."
    NO_DEFAULT_PATH)
find_library(Simbody_STATIC_LIBRARY_VN NAMES ${NS}SimTKsimbody_static${VN}
    PATHS ${Simbody_LIB_DIR}
    DOC "This is the main Simbody static library (versioned)."
    NO_DEFAULT_PATH)
find_library(Simbody_DEBUG_LIBRARY_VN NAMES ${NS}SimTKsimbody${VN}_d
    PATHS ${Simbody_LIB_DIR}
    DOC "This is the main Simbody debug library (versioned)."
    NO_DEFAULT_PATH)
find_library(Simbody_STATIC_DEBUG_LIBRARY_VN 
	     NAMES ${NS}SimTKsimbody_static${VN}_d
    PATHS ${Simbody_LIB_DIR}
    DOC "This is the main Simbody static debug library (versioned)."
    NO_DEFAULT_PATH)


# Set composite Simbody_LIBRARIES_VN variable
set(LIBS)
if(Simbody_LIBRARY_VN AND Simbody_DEBUG_LIBRARY_VN)
    foreach(libname IN LISTS Simbody_LIBRARY_LIST)
        set(LIBS ${LIBS} optimized "${libname}${VN}" 
		         debug "${libname}${VN}_d")
    endforeach()
elseif(Simbody_LIBRARY_VN)
    foreach(libname IN LISTS Simbody_LIBRARY_LIST)
        set(LIBS ${LIBS} "${libname}${VN}")
    endforeach()
elseif(Simbody_DEBUG_LIBRARY_VN)
    foreach(libname IN LISTS Simbody_LIBRARY_LIST)
        set(LIBS ${LIBS} "${libname}${VN}_d")
    endforeach()
endif()

if (LIBS)
    foreach(lapack_lib IN LISTS Simbody_LAPACK_LIBRARY_LIST)
        set(LIBS ${LIBS} "${lapack_lib}")
    endforeach()
    foreach(extra_lib IN LISTS Simbody_EXTRA_LIBRARY_LIST)
        set(LIBS ${LIBS} "${extra_lib}")
    endforeach()
    set(Simbody_LIBRARIES_VN ${LIBS} CACHE STRING 
        "Simbody dynamic libraries" FORCE)
else()
    set(Simbody_LIBRARIES_VN Simbody_LIBRARIES_VN-NOTFOUND CACHE STRING 
        "Simbody dynamic libraries" FORCE)
endif()

# Static library
set(LIBS)
if(Simbody_STATIC_LIBRARY_VN AND Simbody_STATIC_DEBUG_LIBRARY_VN)
    foreach(libname IN LISTS Simbody_LIBRARY_LIST)
        set(LIBS ${LIBS} optimized "${libname}_static${VN}" 
		         debug     "${libname}_static${VN}_d")
    endforeach()
elseif(Simbody_STATIC_LIBRARY)
    foreach(libname IN LISTS Simbody_LIBRARY_LIST)
        set(LIBS ${LIBS} "${libname}_static${VN}")
    endforeach()
elseif(Simbody_STATIC_DEBUG_LIBRARY)
    foreach(libname IN LISTS Simbody_LIBRARY_LIST)
        set(LIBS ${LIBS} "${libname}_static${VN}_d")
    endforeach()
endif()

if (LIBS)
    # these aren't available in static
    foreach(lapack_lib IN LISTS Simbody_LAPACK_LIBRARY_LIST)
        set(LIBS ${LIBS} "${lapack_lib}")
    endforeach()
    foreach(extra_lib IN LISTS Simbody_EXTRA_LIBRARY_LIST)
        set(LIBS ${LIBS} "${extra_lib}")
    endforeach()
    set(Simbody_STATIC_LIBRARIES_VN "${LIBS}" CACHE STRING 
        "Simbody static libraries" FORCE)
else()
    set(Simbody_STATIC_LIBRARIES_VN Simbody_STATIC_LIBRARIES-NOTFOUND CACHE STRING 
        "Simbody static libraries" FORCE)
endif()
endif(BUILD_VERSIONED_LIBRARIES)



include(FindPackageHandleStandardArgs OPTIONAL)
find_package_handle_standard_args(Simbody DEFAULT_MSG 
	Simbody_INCLUDE_DIR)

# Not all the variables we produced need be returned.

if(BUILD_UNVERSIONED_LIBRARIES)
    unset(Simbody_LIBRARY CACHE)
    unset(Simbody_DEBUG_LIBRARY CACHE)
    unset(Simbody_STATIC_LIBRARY CACHE)
    unset(Simbody_STATIC_DEBUG_LIBRARY CACHE)
    mark_as_advanced(Simbody_LIBRARIES Simbody_STATIC_LIBRARIES)
endif()

if(BUILD_VERSIONED_LIBRARIES)
    unset(Simbody_LIBRARY_VN CACHE)
    unset(Simbody_DEBUG_LIBRARY_VN CACHE)
    unset(Simbody_STATIC_LIBRARY_VN CACHE)
    unset(Simbody_STATIC_DEBUG_LIBRARY_VN CACHE)
    mark_as_advanced(Simbody_LIBRARIES_VN Simbody_STATIC_LIBRARIES_VN)
endif()

# Leave Simbody_ROOT_DIR visible
mark_as_advanced(
    Simbody_INCLUDE_DIR
    Simbody_BIN_DIR
    Simbody_LIB_DIR
)
