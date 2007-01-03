/*******************************************************************************

   UNIVERSAL.H

   Author: Peter Loan

   Date: 16-JUL-90

   Copyright (c) 1992, 1993 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   This file is included into EVERY ".c" file, and thus should only
   contain "#includes" that every file needs.

*******************************************************************************/

#ifndef UNIVERSAL_H
#define UNIVERSAL_H

/* #define sgi           so math.h will include sgimath.h automatically */

#include <stdlib.h>

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <sys/types.h>
#include <malloc.h>
#include <float.h>

#if defined(_MSC_VER) && ! defined(__MWERKS__)
   /* Use the following macro to detect the Microsoft Visual C++ compiler:
    */
   #define MS_VISUAL_C 1
#endif

#define POLYGON_DISPLAY_HACK 1
#define STEADYCAM 1

#ifndef M_PI
#define M_PI   3.1415926535897932384626433832795
#define M_PI_2 (M_PI / 2.0)
#endif

#ifdef SIMM_RENDERMATIC
#define _3D_MATRIX_
#include "3d.h"
#endif

#define ENGINE
#define CONVERTER
// Ayman: Replacement for TOXML macro that was undefined and caused problems locating bone files
#define SIMM_OPENSIM	
// Eran: replaced glut dependency with some reasonable(?) typedefs
//#include "glut.h"     /* GLUT includes GL/gl.h and GL/glu.h for us */
//#include "glutDialogs.h"
typedef float GLfloat;
typedef unsigned int GLuint;
typedef int GLint;
typedef double GLdouble;

#include "sm.h"
#include "dp.h"

#include "macros.h"
#include "basic.h"
#include "events.h"
#include "color.h"
#include "simmkeys.h"

#include "modelplot.h"
#include "windowroot.h"
#include "mathtools.h"
//#include "glutglue.h"
#include "mocap.h"

#endif /*UNIVERSAL_H*/


