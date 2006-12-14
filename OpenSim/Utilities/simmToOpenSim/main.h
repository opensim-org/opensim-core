/*******************************************************************************

   MAIN.H

   Author: Peter Loan

   Date: 13-APR-89

   Copyright (c) 1992-2005 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

*******************************************************************************/

#ifndef MAIN_H
#define MAIN_H

#include "functions.h"

#ifdef ENGINE
ModelStruct* model[2];
#else
ModelStruct* model[MODELBUFFER];
PlotStruct* plot[PLOTBUFFER];
ToolStruct tool[TOOLBUFFER];
RootStruct root;
#endif

char msg[CHARBUFFER];
char buffer[CHARBUFFER*10];
char errorbuffer[CHARBUFFER];
double origin[] = {0.0, 0.0, 0.0};
double x_axis[] = {1.0, 0.0, 0.0};
double y_axis[] = {0.0, 1.0, 0.0};
double z_axis[] = {0.0, 0.0, 1.0};

char program_full_name[] = "Software for Interactive Musculoskeletal Modeling";

char program_name[] = "SIMM";
char program_version[] = "4.1.2a10";
char program_with_version[20];

SBoolean pipe_included = no;
SBoolean motion_post_included = no;
SBoolean motion_real_included = no;

char program_date[] = __DATE__;
char copyright_notice[] = "Copyright (c) 1992-2006 MusculoGraphics (a division of Motion Analysis Corp.)";
char memory_message[] = "Ran out of memory.";
char tool_message[] = "Could not make SIMM tools.";

/* char new_ascii_label[] = "NORM_ASCII";  multiply defined in normio.h */

#endif /*MAIN_H*/

