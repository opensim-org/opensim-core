/*******************************************************************************

   DPFUNCTIONS.H - this header declares the DPTool Editor's public functions
      
   Author: Krystyne Blaikie (based on cefunctions.h by Kenny Smith)

   Date: 25-NOV-03

   Copyright (c) 1998 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

*******************************************************************************/

#ifndef DPFUNCTIONS_H
#define DPFUNCTIONS_H

/* defined in dptooltools.c */
void   makedptoolmenus(void);
void   make_dp_motion_menu(void);

void   dp_entervalue(SimmEvent se);
void   update_dp_forms(void);
void   update_dp_radiopanel(void);
void   update_dp_analogradiopanel(void);
void   update_dp_globalcheckboxpanel(void);
void   update_dp_sliderpanel(void);
void   update_dp_win_status(void);
void   do_dp_chpanel(WindowParams* win_parameters, int num);
void   do_dp_radiopanel(WindowParams* win_parameters, int num);
void   do_dp_analogradiopanel(WindowParams* win_parameters, int num);

void   dp_shorten_filename(char *in, char *out, int length);
void   move_dptool_help_text(int dummy_int, double slider_value, double delta);

void   dp_choose_working_dir(void);
void   dp_choose_dll(void);
void   dp_choose_input_file(void);
void   dp_choose_output_file(void);

void   dp_update_column_names(MotionSequence *motion);
void   dp_init_motion_sequence(MotionSequence *motion);

/* defined in dpsimulation.c */
void sendSimMessage(SimStruct* ss, int mess, void* data);
SimMessageType getSimMessage(SimStruct* ss, void** data);
SBoolean loadSimulationDll(ModelStruct* model);
SBoolean setupSimulation(ModelStruct* model);
void runSimulation(ModelStruct* model);
void pauseSimulation(ModelStruct* model);
void resumeSimulation(ModelStruct* model);
void resetSimulation(ModelStruct* model);
DWORD WINAPI simThreadMain(LPVOID lpParam);

#endif /* DPFUNCTIONS_H */
