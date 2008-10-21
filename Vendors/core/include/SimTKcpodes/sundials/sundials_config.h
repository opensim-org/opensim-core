/*
 * -----------------------------------------------------------------
 * $Revision: 1.2 $
 * $Date: 2006/11/08 00:48:24 $
 * ----------------------------------------------------------------- 
 * Programmer(s): Aaron Collier and Radu Serban @ LLNL
 * -----------------------------------------------------------------
 * Copyright (c) 2005, The Regents of the University of California.
 * Produced at the Lawrence Livermore National Laboratory.
 * All rights reserved.
 * For details, see the LICENSE file.
 *------------------------------------------------------------------
 * SUNDIALS configuration header file
 *------------------------------------------------------------------
 */


/***** ADDED BY SHERM 20061121 */
#include "SimTKcommon.h"

/* Keeps MS VC++ 8 quiet about sprintf, strcpy, etc. (sherm) */
#ifdef _MSC_VER
#pragma warning(disable:4996)
#endif

/* Shared libraries are messy in Visual Studio. We have to distinguish three
 * cases:
 *   (1) this header is being used to build the SimTKcpodes shared library (dllexport)
 *   (2) this header is being used by a *client* of the SimTKcpodes shared
 *       library (dllimport)
 *   (3) we are building the SimTKcpodes static library, or the client is
 *       being compiled with the expectation of linking with the
 *       SimTKcpodes static library (nothing special needed)
 * In the CMake script for building this library, we define one of the symbols
 *     SimTK_CPODES_BUILDING_{SHARED|STATIC}_LIBRARY
 * Client code normally has no special symbol defined, in which case we'll
 * assume it wants to use the shared library. However, if the client defines
 * the symbol SimTK_USE_STATIC_LIBRARIES we'll suppress the dllimport so
 * that the client code can be linked with static libraries. Note that
 * the client symbol is not library dependent, while the library symbols
 * affect only the SimTKcpodes library, meaning that other libraries can
 * be clients of this one. However, we are assuming all-static or all-shared.
*/

#ifdef WIN32
    #if defined(SimTK_CPODES_BUILDING_SHARED_LIBRARY)
        #define SUNDIALS_EXPORT __declspec(dllexport)
    #elif defined(SimTK_CPODES_BUILDING_STATIC_LIBRARY) || defined(SimTK_USE_STATIC_LIBRARIES)
        #define SUNDIALS_EXPORT
    #else
        /* i.e., a client of a shared library */
        #define SUNDIALS_EXPORT __declspec(dllimport)
    #endif
#else
    /* Linux, Mac */
    #define SUNDIALS_EXPORT
#endif

/* Define precision of SUNDIALS data type 'realtype' 
 * Depending on the precision level, one of the following 
 * three macros will be defined:
 *     #define SUNDIALS_SINGLE_PRECISION 1
 *     #define SUNDIALS_DOUBLE_PRECISION 1
 *     #define SUNDIALS_EXTENDED_PRECISION 1
 */
#ifdef SimTK_DEFAULT_PRECISION
    #if   (SimTK_DEFAULT_PRECISION == 1)
        #define SUNDIALS_SINGLE_PRECISION 1
    #elif (SimTK_DEFAULT_PRECISION == 2)
        #define SUNDIALS_DOUBLE_PRECISION 1
    #elif (SimTK_DEFAULT_PRECISION == 4)
        #define SUNDIALS_EXTENDED_PRECISION 1
    #endif
#else
    #define SUNDIALS_DOUBLE_PRECISION 1
#endif

/***** END OF SHERM'S ADDITIONS */

/* Define SUNDIALS version number */
#define SUNDIALS_PACKAGE_VERSION "2.3.0"

/* FCMIX: Define Fortran name-mangling macro 
 * Depending on the inferred scheme, one of the following 
 * six macros will be defined:
 *     #define F77_FUNC(name,NAME) name
 *     #define F77_FUNC(name,NAME) name ## _
 *     #define F77_FUNC(name,NAME) name ## __
 *     #define F77_FUNC(name,NAME) NAME
 *     #define F77_FUNC(name,NAME) NAME ## _
 *     #define F77_FUNC(name,NAME) NAME ## __
 */
#define F77_FUNC(name,NAME) name ## _
#define F77_FUNC_(name,NAME) name ## _


/* Use generic math functions 
 * If it was decided that generic math functions can be used, then
 *     #define SUNDIALS_USE_GENERIC_MATH 1
 * otherwise
 *     #define SUNDIALS_USE_GENERIC_MATH 0
 */
#define SUNDIALS_USE_GENERIC_MATH 1

/* FNVECTOR: Allow user to specify different MPI communicator
 * If it was found that the MPI implementation supports MPI_Comm_f2c, then
 *      #define SUNDIALS_MPI_COMM_F2C 1
 * otherwise
 *      #define SUNDIALS_MPI_COMM_F2C 0
 */

