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

#if MEMORY_LEAK
  #define _CRTDBG_MAP_ALLOC
  #include <stdlib.h>
  #include <crtdbg.h>
  #define simm_malloc malloc
  #define simm_calloc calloc
  #define simm_realloc(ptr, size, rc) realloc(ptr, size)
  #define mstrcpy(dest, src) {int len=strlen(src)+1; *(dest) = (char*)malloc(len); strcpy(*(dest), src);}
#else
  #include <stdlib.h>
#endif

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <malloc.h>
#include <float.h>
#include <time.h>

#if defined(_MSC_VER)
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

#if OPENSIM_BUILD
typedef float GLfloat;
typedef unsigned int GLuint;
typedef int GLint;
typedef double GLdouble;
typedef unsigned int GLenum;
typedef unsigned char GLubyte;
#else
#include <GL/glut.h>     /* GLUT includes GL/gl.h and GL/glu.h for us */
#include <GL/glutDialogs.h>
#endif

#include "sm.h"
#include "dp.h"
#include "dp420.h"

#include "macros.h"
#include "basic.h"
#include "events.h"
#include "color.h"
#include "simmkeys.h"

#include "modelplot.h"
#include "windowroot.h"
#include "mathtools.h"
#if ! OPENSIM_BUILD
#include "glutglue.h"
#endif
#include "mocap.h"

#endif /*UNIVERSAL_H*/


