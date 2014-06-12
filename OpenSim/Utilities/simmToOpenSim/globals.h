/*******************************************************************************

   GLOBALS.H

   Author: Peter Loan

   Date: 13-APR-89

   Copyright (c) 1992, 1993 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

*******************************************************************************/

#ifndef GLOBALS_H
#define GLOBALS_H

extern Scene* gScene[];
extern ModelStruct* gModel[];
extern PlotStruct* gPlot[];
extern ToolStruct tool[];
#if ! OPENSIM_BUILD
extern RootStruct root;
#endif

extern char msg[];
extern char buffer[];
extern char errorbuffer[];
extern double origin[];
extern double x_axis[];
extern double y_axis[];
extern double z_axis[];

extern char program_full_name[];
extern char program_name[];
extern char program_version[];
extern char program_with_version[];
extern char program_date[];
extern SBoolean pipe_included;
extern SBoolean motion_post_included;
extern SBoolean motion_real_included;
extern char copyright_notice[];
extern char memory_message[];
extern char tool_message[];
extern char new_ascii_label[];

#endif /*GLOBALS_H*/

