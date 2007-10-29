/*******************************************************************************

   TOOLS.C

   Author: Peter Loan

   Date: 8-DEC-88

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Description: This file contains a bunch of generic tool
      routines that are used in a wide range of circumstances.

   Routines:
      finddistance     : computes distance between two points, spans ref frames
      get_path_between_frames   : finds the path betwen any two ref frames
      find_joint_between_frames : finds the joint spanning two ref frames
      getmuscnumber    : returns a muscle's number, given its name
      findvelocity     : finds the velocity of a musculotendon unit
      evaluate         : finds the current value of a reference equation (dof)
      error            : handles error messages
      setgencoord      : sets value of a gencoord, marks joint matrices dirty
      findfuncnum      : finds function number which uses specified gencoord
      member           : checks an int array for a specified member
      change_filename_suffix : puts a new suffix on a filename
      message          : prints a message to stdout
      gencoord_in_path : checks if a gencoord is used in a chain of joints
      print_4x4matrix  : prints a 4x4 matrix to stdout
      register_tool    : records tool info in array when tool is created
      strcat3          : concatenates three strings together
      mstrcpy          : like strcpy, but mallocs space for the copy first
      draw_title_area  : draws the title area at the top of each tool window
      check_title_area : checks if user hit a button in the title area
      print_time       : prints the current time to stdio (for benchmarking)
      make_time_string : converts current time into readable string
      convert_string   : converts special chars in a string to underscores
      set_viewport     : sets the viewport and resets MODELVIEW and PROJECTION mats

*******************************************************************************/

#include "universal.h"

#include <stdarg.h>
#include <errno.h>
#include <ctype.h>

#define _POSIX_ 1

#include <fcntl.h>

#if defined(WIN32)
   #include <time.h>
	#include <direct.h>
#elif defined(__linux__)
   #include <time.h>
	#include <sys/stat.h>
	#include <sys/types.h>
#else
   #define _IEEE 1
   #include <nan.h>
   #include <sys/time.h>
#endif

#include "globals.h"
#include "functions.h"
#include "normio.h"


/*************** DEFINES (for this file only) *********************************/
#define STOP_EVENT 0x1000
#define MAX_ITEMS_IN_QUEUE 101
#define MAXKEYS 10
#define BETA_VERSION 0

/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/
static char key_file[] = "key.simm";
static int key_offset[] = {11,8,6,10,9,11,8,9,7,9,10,8,11,7,10,8,6,9,7,11};
static int key_shuffle[] = {14,15,2,4,13,17,1,19,12,7,11,18,10,6,5,8,3,9,16,0};
static int last_index = -1;
unsigned short hourglasses[][16] = {
    {0x7FFE, 0x4002, 0x2004, 0x300C, 
    0x2894, 0x2424, 0x2244, 0x2244, 
    0x2344, 0x23C4, 0x27E4, 0x2FF4, 
    0x3FFC, 0x3FFC, 0x4FF2, 0x7FFE},

    {0x7FFE, 0x43C2, 0x2184, 0x300C, 
    0x2814, 0x2424, 0x2244, 0x2244, 
    0x22C4, 0x23C4, 0x27E4, 0x2FF4, 
    0x3FFC, 0x2FF4, 0x47E2, 0x7FFE},

    {0x7FFE, 0x47E2, 0x23C4, 0x308C, 
    0x2814, 0x2424, 0x2244, 0x2244, 
    0x22C4, 0x23C4, 0x27E4, 0x2FF4, 
    0x3FFC, 0x27F4, 0x4182, 0x7FFE},

    {0x7FFE, 0x4FE2, 0x27C4, 0x318C, 
    0x2914, 0x2424, 0x2244, 0x2244, 
    0x22C4, 0x23C4, 0x27E4, 0x2FF4, 
    0x37FC, 0x23F4, 0x4082, 0x7FFE},

    {0x7FFE, 0x4FF2, 0x27E4, 0x33CC, 
    0x2914, 0x2424, 0x2244, 0x2244, 
    0x2344, 0x23C4, 0x27E4, 0x2FF4, 
    0x37FC, 0x21C4, 0x4002, 0x7FFE},

    {0x7FFE, 0x4FFA, 0x27F4, 0x33EC, 
    0x2994, 0x2424, 0x2244, 0x2244, 
    0x22C4, 0x23C4, 0x27E4, 0x2FF4, 
    0x37CC, 0x2084, 0x4002, 0x7FFE},

    {0x7FFE, 0x5FFA, 0x2FF4, 0x37EC, 
    0x2B94, 0x2424, 0x2244, 0x2244, 
    0x2344, 0x23C4, 0x27E4, 0x2FF4, 
    0x318C, 0x2004, 0x4002, 0x7FFE},

    {0x7FFE, 0x7FFE, 0x3FF4, 0x37EC, 
    0x2BD4, 0x2424, 0x2244, 0x2244, 
    0x22C4, 0x23C4, 0x27E4, 0x2BD4, 
    0x300C, 0x2004, 0x4002, 0x7FFE},

    {0x7FFE, 0x7FFE, 0x3FFC, 0x3FFC, 
    0x2FD4, 0x2424, 0x2244, 0x2244, 
    0x2344, 0x23C4, 0x27E4, 0x2814, 
    0x300C, 0x2004, 0x4002, 0x7FFE},

    {0x7FFE, 0x7FFE, 0x3FFC, 0x3FFC, 
    0x2FF4, 0x26E4, 0x2244, 0x2244, 
    0x22C4, 0x23C4, 0x2424, 0x2814, 
    0x300C, 0x2004, 0x4002, 0x7FFE},

    {0x7FFE, 0x7FFE, 0x3FFC, 0x3FFC, 
    0x2FF4, 0x27E4, 0x23C4, 0x23C4, 
    0x2244, 0x2244, 0x2424, 0x2814, 
    0x300C, 0x2004, 0x4002, 0x7FFE},
};
char* keys[] = {
"null_key","null_key","null_key","null_key","null_key","null_key","null_key","null_key",
"backspace_key","tab_key","null_key","null_key","null_key","return_key","enter_key",
"null_key","null_key","null_key","null_key","null_key","null_key","null_key","null_key",
"null_key","null_key","null_key","null_key","escape_key","null_key","null_key",
"null_key","null_key","space_key","bang_key","double_quote_key","pound_sign_key",
"dollar_key","percent_key","ampersand_key","single_quote_key",
"left_paren_key","right_paren_key","asterisk_key","plus_key",
"comma_key","dash_key","period_key","slash_key","zero_key","one_key","two_key","three_key",
"four_key","five_key","six_key","seven_key","eight_key","nine_key","colon_key","semicolon_key",
"less_than_key","equals_key","greater_than_key","question_mark_key","at_sign_key",
"A_key","B_key","C_key","D_key","E_key","F_key","G_key","H_key","I_key","J_key","K_key",
"L_key","M_key","N_key","O_key","P_key","Q_key","R_key","S_key","T_key","U_key","V_key",
"W_key","X_key","Y_key","Z_key","left_bracket_key","backslash_key",
"right_bracket_key","carat_key","underscore_key","back_quote_key",
"a_key","b_key","c_key","d_key","e_key","f_key","g_key","h_key","i_key",
"j_key","k_key","l_key","m_key","n_key","o_key","p_key","q_key","r_key","s_key",
"t_key","u_key","v_key","w_key","x_key","y_key","z_key","left_brace_key",
"vertical_bar_key","right_brace_key","tilde_key","delete_key",
"leftmouse_button","middlemouse_button","rightmouse_button",
"mouse_motion","window_shut","window_quit","input_change","depth_change","window_thaw",
"window_freeze","f1_key","f2_key","f3_key","f4_key","f5_key","f6_key","f7_key",
"f8_key","f9_key","f10_key","f11_key","f12_key","left_arrow_key","up_arrow_key",
"right_arrow_key","down_arrow_key","page_up_key","page_down_key","home_key",
"end_key","insert_key","shift_key","control_key","alt_ket","caps_lock_key"};


/*************** GLOBAL VARIABLES (used in only a few files) ******************/


/*************** EXTERNED VARIABLES (declared in another file) ****************/


/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static double calc_muscle_segment_velocity(int mod, int musc, double p1[], int frame1,
					   double p2[], int frame2);
static void decrypt(char ciphertext[], char cleartext[]);
static SBoolean verify_date(int day, int month, int year);
unsigned sysid(unsigned char id[16]);


/* MAKE_STRING_LOWER_CASE: */

void make_string_lower_case(char str_buffer[])
{

   char c;
   int curpos = 0;

   while ((c=str_buffer[curpos++]) != STRING_TERMINATOR)
      if (c >= 65 && c <= 91)
         str_buffer[curpos-1] += 32;

}


/* CALC_VECTOR_LENGTH: This routine calculates the length of a vector. The
 * vector is defined by two points, which can be in different body segment
 * reference frames.
 */

double calc_vector_length(int mod, double p1[], int frame1, double p2[], int frame2)
{

   double ans, x, p3[3];

   p3[0] = p1[0];
   p3[1] = p1[1];
   p3[2] = p1[2];

   if (frame1 != frame2)
      convert(mod,p3,frame1,frame2);

   x = ((p2[0]-p3[0])*(p2[0]-p3[0]) + (p2[1]-p3[1])*(p2[1]-p3[1]) +
       (p2[2]-p3[2])*(p2[2]-p3[2]));

   ans = sqrt(x);

   return (ans);

}


/* GET_PATH_BETWEEN_FRAMES: This routine returns the path between two body
 * segment reference frames. The path consists of the list of body segments
 * in between the two specified frames.
 */

int* get_path_between_frames(int mod, int frame1, int frame2)
{

   return (model[mod]->pathptrs[model[mod]->numsegments*frame1+frame2]);

}



/* FIND_JOINT_BETWEEN_FRAMES: This routine finds the joint which spans the
 * two specified body segment reference frames. It also indicates whether
 * the joint is traversed forwards or backwards to get from the from_frame
 * to the to_frame. If the reference frames are not adjacent, it returns NULL.
 */

int find_joint_between_frames(int mod, int from_frame, int to_frame, Direction* dir)
{

   int i;

   for (i=0; i<model[mod]->numjoints; i++)
   {
      if (model[mod]->joint[i].from == from_frame && model[mod]->joint[i].to == to_frame)
      {
         *dir = FORWARD;
         return i;
      }

      if (model[mod]->joint[i].from == to_frame && model[mod]->joint[i].to == from_frame)
      {
         *dir = INVERSE;
         return i;
      }
   }

   return ZERO;

}

#ifndef CONVERTER

/* CALC_MUSCLE_TENDON_VELOCITY: Finds the velocity of a muscle-tendon unit.
 * Muscle-tendon velocity is the sum of the velocities of each muscle path
 * segment. This routine assumes that all gencoord velocities have already
 * been calculated.
 */

double calc_muscle_tendon_velocity(int mod, int musc)
{

   int end, start;
   double velocity = 0.0;
   MuscleStruct* muscl = &model[mod]->muscle[musc];

   /* NOTE:  For the time-being, check_wrapping_points() is #ifdef'd for the
    *   GUI version of SIMM only.  This is only because it was going to take
    *   quite a bit more effort to prepare various wrapping source files for
    *   compilation into the non-GUI SIMM engine, and I needed to get a version
    *   of 'scalemodel' to RIC sooner rather than later.  Eventaully we will
    *   want to move check_wrapping_points() and its dependancies into the
    *   non-GUI SIMM engine.  -- KMS 5/20/99
    */
   check_wrapping_points(model[mod], muscl);

   if (muscl->num_points < 2)
      return 0.0;

   for (start = 0; start < muscl->num_points - 1; start++)
   {
      end = start + 1;
      velocity += calc_muscle_segment_velocity(mod,musc,muscl->mp[start]->point,
                                               muscl->mp[start]->segment,
                                               muscl->mp[end]->point,
                                               muscl->mp[end]->segment);
   }

   /* Normalize the velocity */

   velocity /= (*muscl->optimal_fiber_length)*(*muscl->max_contraction_vel);

   return velocity;

}



/* CALC_MUSCLE_SEGMENT_VELOCITY: calculates the velocity of one segment of a
 * muscle-tendon path. The velocity of one segment is equal to the velocity
 * of one end of the segment with respect to the other, dotted with a unit
 * vector in the direction of the segment. The velocity of one end of the
 * segment is the sum of that point's partial velocities with respect to each
 * generalized coordinate, multiplied by the derivative of the generalized
 * coordinate (the joint velocities). If the point you're finding the velocity
 * of is called P and the reference frame of the other end of the segment is A,
 * then the velocity of P in A is:
 * 
 *   A P          A P                                                         
 *    V  =  SUM  ( V   *  dq / dt)    n = number of generalized coordinates
 *         0<i<=n   q       i                                                
 *                   i
 * 
 * If M is a unit vector in the direction of the muscle segment, then the
 * velocity of the muscle segment, VM, is:
 * 
 *      A P
 *  VM = V  .  M
 * 
 * 
 * Combining these two equations, we get:
 * 
 *                A P                                                         
 *   VM  =  SUM  ( V   *  dq / dt)   .   M
 *         0<i<=n   q       i                                                
 *                   i
 * 
 *  or,
 * 
 *                  A P                                                         
 *   VM  =  SUM  ( ( V  .  M)  *  dq / dt)
 *         0<i<=n     q             i                                                
 *                     i
 * 
 * 
 * 
 *          A P                                                         
 *   since  -V  .  M  =  moment arm of muscle segment w.r.t. q
 *            q                                               i
 *             i
 * 
 *   we get:
 * 
 * 
 *                
 *   VM  =  SUM  (-ma   *  dq / dt)
 *         0<i<=n    q       i
 *                    i
 * 
 * 
 * So to find the velocity of the muscle segment, we just sum all of the
 * segment's moment arms, each multiplied by the velocity of the generalized
 * coordinate. To show this more clearly for just one generalized coordinate:
 *        ma = -dl/dq
 *         w =  dq/dt
 *   -ma * w =  dl/dt = muscle-tendon velocity
 */

static double calc_muscle_segment_velocity(int mod, int musc, double p1[], int frame1,
					   double p2[], int frame2)
{

   int i;
   double genc_vel, velocity = 0.0;

   if (frame1 == frame2)
      return (0.0);

   for (i=0; i<model[mod]->numgencoords; i++)
      model[mod]->muscle[musc].momentarms[i] = 0.0;

   calc_segment_arms(mod,musc,p1,frame1,p2,frame2);

   for (i=0; i<model[mod]->numgencoords; i++)
   {
      if (model[mod]->gencoord[i].type == rotation_gencoord)
	 genc_vel = model[mod]->gencoord[i].velocity * DTOR;
      else
	 genc_vel = model[mod]->gencoord[i].velocity;
      velocity += (-model[mod]->muscle[musc].momentarms[i] * genc_vel);
   }

   return (velocity);

}

#endif // CONVERTER

/* EVALUATE_DOF: This routine calculates the current value of a dof. It stores
 * the value inside the dof structure, and also returns the value. If the dof
 * is a constant, then the value is already stored in var->value.
 */

double evaluate_dof(int mod, DofStruct* var)
{

   if (var->type == constant_dof)
      return (var->value);
   else
   {
      var->value = interpolate_spline(model[mod]->gencoord[var->gencoord].value,
			       &model[mod]->function[var->funcnum],zeroth,1.0,1.0);
      return (var->value);
   }

}

#ifndef ENGINE

void deiconify_message_window ()
{
#if ! NO_GUI

   int i, savedID = glutGetWindow();

   for (i = 0; i < root.numwindows; i++)
      if (root.window[i].win_parameters->id == root.messages.window_id)
         break;
   
   glutSetWindow(root.messages.window_id);
   
   if (i < root.numwindows && root.window[i].state == WINDOW_ICONIC)
      glutShowWindow();
   else
      glutPopWindow();
   
   glutSetWindow(savedID);

#endif
}

#endif /* ENGINE */

/* ERROR: this routine prints an error message depending on a string
 * and error status that are passed in.
 */

void error(ErrorAction action, char str_buffer[])
{

#ifndef ENGINE
   deiconify_message_window();
#endif

   if (str_buffer != NULL)
      message(str_buffer,HIGHLIGHT_TEXT,DEFAULT_MESSAGE_X_OFFSET);

   if (action == recover)
      message("Attempting to recover.",0,DEFAULT_MESSAGE_X_OFFSET);
   else if (action == abort_action)
      message("Action cancelled.",0,DEFAULT_MESSAGE_X_OFFSET);
   else if (action == exit_program)
   {
      putchar('\a');   /* was ringbell() */
      fflush(stdout);

      message("Program terminated.",HIGHLIGHT_TEXT,DEFAULT_MESSAGE_X_OFFSET);

#if ! NO_GUI
      fprintf(stderr,"Fatal Error.\nProgram terminated.");
#endif

      exit(0);
   }
}


int find_next_active_field(Form* form, int current_field, TextFieldAction tfa)
{

   int field, increment;

   if (tfa == goto_next_field)
      increment = 1;
   else if (tfa == goto_previous_field)
      increment = form->numoptions - 1;
   else
      return (current_field);

   field = (current_field+increment) % form->numoptions;

   while (field != current_field)
   {
      if ((form->option[field].active == yes) && (form->option[field].editable == yes))
//      if (form->option[field].visible == yes)
	 break;
      field = (field+increment) % form->numoptions;
   }

   return (field);

}


/* SET_GENCOORD_VALUE: this is an important little routine. It should be the
 * ONLY way that the value of a generalized coordinate is changed.
 * It sets the value, and then marks some conversion matrices invalid
 * so that they will not be used again without being recalculated.
 * The matrices which use the dof in question, as stored in the jointnum[]
 * array for that dof, are marked.
 */

int set_gencoord_value(int mod, int genc, double value, SBoolean solveLoopsAndConstraints)
{
#ifndef ENGINE
   int i;
   SBoolean solveLoops, solveConstraints, sol;
#endif
   GeneralizedCoord* gc;

   gc = &model[mod]->gencoord[genc];

   /* check whether the gencoord value has changed.  If not, don't bother
    * updating anything.  Also check the value in the model viewer window
    * to see whether that needs updating */
   if ((DABS(value - gc->value) <= gc->tolerance)
      && (DABS(value - model[mod]->gencslider.sl[genc].value) <= gc->tolerance))
      return 0;

#ifndef ENGINE
   if (gc->type == rotation_gencoord)
      checkGencoordRange(model[mod], genc, &value);

   model[mod]->gencform.option[genc].use_alternate_colors = no;
#endif

   if (gc->clamped == yes)
//   if ((gc->clamped == yes) && (solveLoopsAndConstraints == yes)) //added dkb apr 16 2003
   {
      if (value < gc->range.start)
	      value = gc->range.start;
      else if (value > gc->range.end)
	      value = gc->range.end;
   }
   else
   {
#ifndef ENGINE
      if (value < gc->range.start || value > gc->range.end)
	      model[mod]->gencform.option[genc].use_alternate_colors = yes;
#endif
   }

#ifndef ENGINE
   /* Resolve any closed loops in the model, then update the gencoord value
    * (which may have been changed to close the loops).  If any other
    * gencoord values are changed to close loops, resolveClosedLoops takes
    * care of changing their values.  If the solver could not find a valid
    * solution, the gencoord is not updated and the configuration does not
    * change.
    */

   if (solveLoopsAndConstraints == yes)
   {
      sol = solveLCAffectedByGC(model[mod], genc, &value);
      model[mod]->constraintsOK = sol;// && model[mod]->constraintsOK;
      model[mod]->loopsOK = sol;// && model[mod]->loopsOK;
      
      gc->value = value;
      if (sol == no)
         return 0;
   }
   else
   {
      /* loops and constraints are not solved, copy new value into gc */
      gc->value = value;
   }

   if (value < gc->range.start || value > gc->range.end)
      model[mod]->gencform.option[genc].use_alternate_colors = yes;

   model[mod]->gencslider.sl[genc].value = value;

   storeDoubleInForm(&model[mod]->gencform.option[genc], gc->value, 3);

   for (i=0; i<gc->numjoints; i++)
      invalidate_joint_matrix(model[mod],gc->jointnum[i]);

   /* hack so that ground-reaction forces are shown only during a motion */
   model[mod]->dis.applied_motion = NULL;

   /* if the gencoord being changed is a translational dof, then we need to
    * invalidate the current bounds of the scene to prevent the model from
    * sliding behind the far clipping plane.  -- added KMS 10/7/99
    */
   if (gc->type == translation_gencoord)
      model[mod]->max_diagonal_needs_recalc = yes;

#endif

   return 1;

}



void set_gencoord_velocity(int mod, int genc, double value)
{

   model[mod]->gencoord[genc].velocity = value;

}



/* FINDFUNCNUM: given the number of a generalized coordinate (dof), this
 * routine returns the number of the first function which has that dof
 * as the independent parameter.
 */

int findfuncnum(DofStruct dof, int gc)
{

   if (dof.type == function_dof && dof.gencoord == gc)
      return (dof.funcnum);

   return (-1);

}



char* get_suffix(char str[])
{

   int cp = 0;

   cp = strlen(str) - 1;

   while (cp >= 0 && str[cp] != '.')
      cp--;

   if (cp == 0)
      return (NULL);

   return (&str[cp+1]);

}


/* CHANGE_FILENAME_SUFFIX: this routine changes the suffix of a file name.
 * It scans the name for a "." (starting from the end) and assumes that
 * everything after the "." is the suffix which is to be changed. Examples:
 * input = "foo.bar", suffix = "tree" --------> output = "foo.tree"
 * input = "foo.foo", suffix = "bar" ---------> output = "foo.bar"
 * input = "foo", suffix = "bar" -------------> output = "foo.bar"
 * input = "foo.bar.tree", suffix = "rock" ---> output = "foo.bar.rock"
 */

void change_filename_suffix(char input[], char output[], char suffix[])
{
   int cp;

   cp = strlen(input) - 1;

   while (input[cp] != '.' && cp > 0)
      cp--;

	if (cp == 0)
	{
		if (suffix)
			sprintf(output,"%s.%s", input, suffix);
		else
			strcpy(output, input);
	}
   else
   {
		if (suffix)
		{
			strncpy(output, input, cp + 1);
			output[cp + 1] = STRING_TERMINATOR;
			strcat(output, suffix);
		}
		else
		{
			strncpy(output, input, cp);
			output[cp] = STRING_TERMINATOR;
		}
   }
}

#ifdef ENGINE
#define NO_GUI 1
#endif

void message(char message_str[], int format, int xoffset)
{
#if NO_GUI

   printf("%s\n", message_str);
   fflush(stdout);

#else

   int i, nl, winid;
   HelpStruct* hp;

#if 0
   printf("%s\n", message_str);
   fflush(stdout);
#endif

   /* If there are no lines malloced in the message structure, just print
    * the message to stdout and return.
    */

   if (root.messages.line == NULL)
   {
      printf("%s\n", message_str);
      return;
   }

   winid = glutGetWindow();

   hp = &root.messages;

   /* If the last line in the list is overwritable, or if the incoming
    * line is OVERWRITE_LAST_LINE, overwrite the last line with the
    * incoming one.
    */
   if (hp->num_lines > 0 &&
      ((hp->line[hp->num_lines - 1].format & OVERWRITABLE) || (format & OVERWRITE_LAST_LINE)))
   {
      hp->num_lines--;
      FREE_IFNOTNULL(hp->line[hp->num_lines].text);
   }

   if (hp->num_lines < hp->num_lines_malloced)
   {
      nl = hp->num_lines;
      (void)mstrcpy(&hp->line[nl].text,message_str);
      hp->line[nl].format = format;
      hp->line[nl].xoffset = xoffset;
      hp->num_lines++;
      if (hp->num_lines <= hp->lines_per_page)
         hp->sl.thumb_thickness = -1;
      else
         hp->sl.thumb_thickness = hp->lines_per_page*
         (hp->sl.shaft.y2-hp->sl.shaft.y1)/hp->num_lines;
   }
   else
   {
      FREE_IFNOTNULL(hp->line[0].text);
      for (i=0; i<hp->num_lines_malloced; i++)
      {
         hp->line[i].text = hp->line[i+1].text;
         hp->line[i].format = hp->line[i+1].format;
         hp->line[i].xoffset = hp->line[i+1].xoffset;
      }
      nl = hp->num_lines_malloced - 1;
      (void)mstrcpy(&hp->line[nl].text,message_str);
      hp->line[nl].format = format;
      hp->line[nl].xoffset = xoffset;
   }

   hp->sl.max_value = hp->num_lines*20.0;
   hp->sl.value = hp->sl.min_value;
   hp->starting_line = HELP_WINDOW_TEXT_Y_SPACING*MAX(0,hp->num_lines-hp->lines_per_page);

   draw_message_window(NULL,NULL);

   glutSetWindow(winid);

#endif /* ! NO_GUI */
}



/* -------------------------------------------------------------------------
   simm_printf - this routine provides printf-style output to the simm
      message window.  The 'hilite_text' parameter specifies whether the
      text is displayed normal or hilited.
   
   NOTE: this routine will buffer text until an end-of-line character is
      detected.  This allows you to build a single line message via multiple
      calls to this routine.  However, this means that you MUST TERMINATE
      EACH MESSAGE LINE WITH A '\n' CHARACTER FOR THE LINE TO BE SENT TO THE
      MESSAGE WINDOW.
---------------------------------------------------------------------------- */
public int simm_printf (SBoolean hilite_text, const char* format, ...)
{
   static char sMessageBuf[CHARBUFFER];
   
   va_list ap;
   int n, simmMsgFormat = 0;
   
   va_start(ap, format);
   n = vsprintf(msg, format, ap);
   va_end(ap);

#ifndef ENGINE
   if (hilite_text)
   {
      simmMsgFormat += HIGHLIGHT_TEXT;
      
      deiconify_message_window();
   }
#endif

   if (strchr(msg, '\n'))
   {
      char* p = strtok(msg, "\n");
      
      if (strlen(sMessageBuf) > 0)
      {
         if (p)
            strcat(sMessageBuf, p);
         
         message(sMessageBuf, simmMsgFormat, DEFAULT_MESSAGE_X_OFFSET);
         
         sMessageBuf[0] = '\0';
      }
      else if (p)
         message(p, simmMsgFormat, DEFAULT_MESSAGE_X_OFFSET);
      
      if (p)
         for (p = strtok(NULL, "\n"); p; p = strtok(NULL, "\n"))
            message(p, simmMsgFormat, DEFAULT_MESSAGE_X_OFFSET);
   }
   else
      strcat(sMessageBuf, msg);
   
   return n;
}



SBoolean gencoord_in_path(int mod, int frame1, int frame2, int genc)
{

   int i, j, joint;
   int* path;

   path = GET_PATH(mod,frame1,frame2);

   for (i=0; path[i] != model[mod]->numjoints+1; i++)
   {
      joint = ABS(path[i]) - 1;
      for (j=0; j<6; j++)
         if (model[mod]->joint[joint].dofs[j].type == function_dof)
            if (model[mod]->joint[joint].dofs[j].gencoord == genc)
               return (yes);
   }

   return (no);

}



void print_4x4matrix(double matrix[][4])
{

   int i, j;

   for (i=0; i<4; i++)
   {
      for (j=0; j<4; j++)
         printf("%8.5lf ", matrix[i][j]);
      printf("\n");
   }

}


#ifndef ENGINE
ToolStruct* register_tool(int struct_size, unsigned int event_mask,
			  void (*event_handler)(SimmEvent),
			  void (*command_handler)(char*),
			  SBoolean (*query_handler)(QueryType, void*),
			  char name[], int* ref_number)
{

   int i;

   for (i=0; i<TOOLBUFFER; i++)
      if (tool[i].used == no)
         break;

   if (i == TOOLBUFFER)
   {
      fprintf(stderr, "ERROR: tools array overflow.\n");
      return (NULL);
   }
   tool[i].used = yes;

   mstrcpy(&tool[i].name,name);

   tool[i].simm_event_mask = event_mask;
   tool[i].simm_event_handler = event_handler;
   tool[i].command_handler = command_handler;
   tool[i].query_handler = query_handler;

   tool[i].tool_struct = (void*)simm_calloc(1, struct_size);
   if (tool[i].tool_struct == NULL)
      error(exit_program,tool_message);

   *ref_number = root.numtools++;

   return (&tool[i]);

}
#endif


void strcat3(char dest[], char str1[], char str2[], char str3[])
{

   (void)strcpy(dest,str1);
   (void)strcat(dest,str2);
   (void)strcat(dest,str3);

}



/* MSTRCPY: this routine is like strcpy(), but it first mallocs space for
 * the copy of the string, and frees any space the destination pointer used
 * to point to.
 */

ReturnCode mstrcpy(char* dest_str[], const char original_str[])
{
   char* p;

#if 0
   if (*dest_str == original_str)
      return code_fine;

   //FREE_IFNOTNULL(*dest_str);
#endif

   if (original_str == NULL)
   {
      *dest_str = NULL;
      return code_fine;
   }

   p = (char*) simm_malloc(STRLEN(original_str) * sizeof(char));

	if (p == NULL)
   {
      *dest_str = NULL;
      return code_bad;
   }

   strcpy(p, original_str);

   *dest_str = p;

   return code_fine;
}



/* MSTRCAT: this routine is like strcat(), but it first mallocs space for
 * the copy of the string.
 */

ReturnCode mstrcat(char* old_str[], const char append_str[])
{

   int new_size;
   ReturnCode rc;

   new_size = strlen(*old_str) + strlen(append_str) + 1;

   if ((*old_str = (char*)simm_realloc(*old_str,new_size*sizeof(char),&rc)) == NULL)
      return (code_bad);

   (void)strcat(*old_str,append_str);

   return (code_fine);

}

#ifndef NO_GUI
#ifndef ENGINE

void draw_title_area(WindowParams* win_params, ModelStruct* ms, PlotStruct* ps,
		     int title_mask)
{

   simm_color(TOOL_TITLE_AREA_BACKGROUND);
   glRecti(win_params->vp.x1, win_params->vp.y2-TITLE_AREA_HEIGHT,
	   win_params->vp.x2+1, win_params->vp.y2+1);

   simm_color(TOOL_TITLE_AREA_BORDER);
   glBegin(GL_LINE_STRIP);
   glVertex2i(win_params->vp.x1-1, win_params->vp.y2-TITLE_AREA_HEIGHT);
   glVertex2i(win_params->vp.x2+1, win_params->vp.y2-TITLE_AREA_HEIGHT);
   glEnd();

   simm_color(TOOL_TITLE_AREA_TEXT);

   glueSetFont(root.gfont.largefont);

   if (title_mask & SHOW_MODEL)
   {
      glRasterPos2i(win_params->vp.x1+15, win_params->vp.y2-23);
      glueDrawString("Model: ");
      if (ms == NULL)
	 glueDrawString("none");
      else
	 glueDrawString(ms->name);
   }

   if (title_mask & SHOW_PLOT)
   {
      glRasterPos2i((win_params->vp.x1+win_params->vp.x2)/2-35, win_params->vp.y2-23);
      glueDrawString("Plot: ");
      if (ps == NULL)
         glueDrawString("none");
      else
         glueDrawString(ps->title);
   }

   if (title_mask & SHOW_MODEL)
   {
      root.model_selector.origin.x = win_params->vp.x1+15;
      root.model_selector.origin.y = win_params->vp.y2-TITLE_AREA_HEIGHT+5;
      draw_menu(&root.model_selector);
   }

   if (title_mask & SHOW_PLOT)
   {
      root.plot_selector.origin.x = (win_params->vp.x1+win_params->vp.x2)/2 - 35;
      root.plot_selector.origin.y = win_params->vp.y2-TITLE_AREA_HEIGHT+5;
      draw_menu(&root.plot_selector);
   }

   if (title_mask & SHOW_HELP)
   {
      root.help_selector.origin.x = win_params->vp.x2-85;
      root.help_selector.origin.y = win_params->vp.y2-TITLE_AREA_HEIGHT+5;
      draw_menu(&root.help_selector);
   }

}



typedef struct {
    TitleAreaCBParams params;
    ModelStruct*      model;
    PlotStruct*       plot;
    int               entry;
    title_area_cb     titleAreaCB;
} TitleAreaMenuCBData;

static void model_menu_cb(int item, void* userData)
{
    TitleAreaMenuCBData* data = (TitleAreaMenuCBData*) userData;
        
    int i, numstructs = 0;

    if (data->model && data->entry > 0)
       glueCheckMenuItem(root.modelmenu,data->entry,GLUE_UNCHECK);

    //highlight_menu_item(&root.model_selector,0,off,yes);

    if (item <= 0 || root.nummodels == 0)
        return;

    for (i=0; i<MODELBUFFER; i++)
    {
       if (model[i] == NULL)
          continue;
       if (++numstructs == item)
          break;
    }
    if (i == MODELBUFFER)
       return;
    data->params.struct_ptr = (void*)(model[i]);

    if (data->titleAreaCB)
        data->titleAreaCB(MODEL_SELECTED, &data->params);

    free(data);

}

static void plot_menu_cb(int item, void* userData)
{
    TitleAreaMenuCBData* data = (TitleAreaMenuCBData*) userData;
    int i, numstructs = 0;

    if (data->plot != NULL && data->entry > 0)
        glueCheckMenuItem(root.plotmenu,data->entry+1,GLUE_UNCHECK);
    //highlight_menu_item(&root.plot_selector,0,off,yes);

    if (item <= 0)
        return;

    if (item == 1)
    {
        data->params.struct_ptr = (void*) NULL;
    }
    else
    {
        item -= 1;
        if (item <= 0 || root.numplots == 0)
            return;
        for (i=0; i<PLOTBUFFER; i++)
        {
            if (plot[i] == NULL)
                continue;
            if (++numstructs == item)
                break;
        }
        if (i == PLOTBUFFER)
            return;
        data->params.struct_ptr = (void*)(plot[i]);
    }
        
    if (data->titleAreaCB)
        data->titleAreaCB(PLOT_SELECTED, &data->params);

    free(data);

}

static TitleAreaMenuCBData* alloc_title_area_data(WindowParams* win_params,
                                                  void*        struct_ptr,
                                                  int           entry,
                                                  ModelStruct*  ms,
                                                  PlotStruct*   ps,
                                                  title_area_cb titleAreaCB)
{
    TitleAreaMenuCBData* data = NULL;

    if (titleAreaCB)
    {
        data = (TitleAreaMenuCBData*) simm_malloc(sizeof(TitleAreaMenuCBData));

        data->params.win_params = win_params;
        data->params.struct_ptr = struct_ptr;
        data->entry      = entry;
        data->model      = ms;
        data->plot       = ps;
        data->titleAreaCB= titleAreaCB;
    }
    return data;
}


int check_title_area(int title_mask, int mx, int my, WindowParams* win_params,
                     void** struct_ptr, ModelStruct* ms, PlotStruct* ps,
                     title_area_cb titleAreaCB)
{

   int entry = 0;

   root.model_selector.origin.x = win_params->vp.x1+15;
   root.model_selector.origin.y = win_params->vp.y2-TITLE_AREA_HEIGHT+5;
   root.plot_selector.origin.x = (win_params->vp.x1+win_params->vp.x2)/2 - 35;
   root.plot_selector.origin.y = win_params->vp.y2-TITLE_AREA_HEIGHT+5;
   root.help_selector.origin.x = win_params->vp.x2-85;
   root.help_selector.origin.y = win_params->vp.y2-TITLE_AREA_HEIGHT+5;

   if ((title_mask & SHOW_MODEL) && (check_menu(&root.model_selector,mx,my) == 0))
   {
      highlight_menu_item(&root.model_selector,0,on,yes);
      if (ms != NULL)
      {
         if ((entry = find_model_ordinal(ms->modelnum)) > 0)
         {
            glueCheckMenuItem(root.modelmenu,entry,GLUE_CHECK);
         }
      }
      gluePopMenu(root.modelmenu, model_menu_cb,
         alloc_title_area_data(win_params, *struct_ptr, entry, ms, ps,
         titleAreaCB));
      return MODEL_SELECTED;
   }

   if ((title_mask & SHOW_PLOT) && (check_menu(&root.plot_selector,mx,my) == 0))
   {
      highlight_menu_item(&root.plot_selector,0,on,yes);
      if (ps != NULL)
      {
         if ((entry = find_plot_ordinal(ps->plotnum)) > 0)
         {
            glueCheckMenuItem(root.plotmenu,entry+1,GLUE_CHECK);
         }
      }
      gluePopMenu(root.plotmenu, plot_menu_cb,
         alloc_title_area_data(win_params, *struct_ptr, entry, ms, ps,
         titleAreaCB));
      return PLOT_SELECTED;
   }

   if ((title_mask & SHOW_HELP) && (check_menu(&root.help_selector,mx,my) == 0))
   {
      if (titleAreaCB)
      {
          TitleAreaMenuCBData* data = alloc_title_area_data(win_params,
                                                            *struct_ptr,
                                                            entry, ms, ps,
                                                            titleAreaCB);
          titleAreaCB(HELP_SELECTED, &data->params);

          free(data);
      }
      return HELP_SELECTED;
   }

   return NULL_SELECTED;

}

#endif /* ! NO_GUI */
#endif /* ENGINE */

#if !defined(WIN32) && !defined(__linux__) /* ==== TEMPORARILY COMMENTED-OUT TO COMPILE FOR WIN32 AND LINUX */

static struct timeval start_tp;
static struct timezone start_tzp;
struct timeval end_tp;
struct timezone end_tzp;

void start_timer(void)
{

   (void)gettimeofday(&start_tp,&start_tzp);

}



void stop_timer(void)
{

   (void)gettimeofday(&end_tp,&end_tzp);

}



void print_duration(char text_string[])
{

   long duration;

   duration = ((double)end_tp.tv_sec - (double)start_tp.tv_sec) * 1000000;
   duration += ((double)end_tp.tv_usec - (double)start_tp.tv_usec);
   printf("%40s\n",text_string);
   printf("elapsed time: %10.6lf sec.\n", (double)duration / 1000000.0);

}



void print_time(void)
{

   struct timeval tp;
   struct timezone tzp;

   (void)gettimeofday(&tp,&tzp);

   printf("time stamp: %d.%6d\n", (int)tp.tv_sec, (int)tp.tv_usec);

}
#endif /* WIN32 */



void make_time_string(char** time_string)
{
   time_t t = time(NULL);
   
   strftime(buffer, CHARBUFFER, "%m/%d/%Y %I:%M:%S %p", localtime(&t));
   
   mstrcpy(time_string, buffer);
   
#if 0
   
   struct timeval tp;
   struct timezone tzp;
   struct tm* cur_tm;
   char tmpbuf[20];

/* Get the current time */

   (void)gettimeofday(&tp,&tzp);

/* Convert the raw time data into the current date and time */

   cur_tm = localtime((const time_t*)&tp.tv_sec);

/* Format the date and time in the time_string */

   (void)sprintf(buffer,"%d/", cur_tm->tm_mon+1);

   (void)sprintf(tmpbuf,"%d/%d  %2d:", cur_tm->tm_mday, cur_tm->tm_year, cur_tm->tm_hour);
   (void)strcat(buffer,tmpbuf);

   if (cur_tm->tm_min < 10)
      (void)strcat(buffer,"0");
   (void)sprintf(tmpbuf,"%d:", cur_tm->tm_min);
   (void)strcat(buffer,tmpbuf);

   if (cur_tm->tm_sec < 10)
      (void)strcat(buffer,"0");
   (void)sprintf(tmpbuf,"%d", cur_tm->tm_sec);
   (void)strcat(buffer,tmpbuf);

   (void)mstrcpy(time_string,buffer);
#endif

}


/* CONVERT_STRING: this routine scans a string and converts all special
 * characters into underscores. A special character is any character
 * other than a letter or number. If prependUnderscore is 'yes,' this
 * function will prepend an underscore character if the string starts with
 * a number. It is assumed that the string already has space for this extra
 * character. The resulting string is one token, and can therefore be used
 * as a variable name in SIMM-written C code.
 *
 * Ayman: modified the function so that only XML meta characters ('<', '>', '&') are converted.
 */

void convert_string(char str[], SBoolean prependUnderscore)
{

   int i, len;

   len = strlen(str);

   for (i = 0; i < len; i++)
   {
	 if (str[i]=='>' || str[i]=='<' || str[i]=='&')
		 str[i]='_';
   }

}

/* convertSpacesInString: this routine scans a string and converts all spaces
 * into underscores.
 */

void convertSpacesInString(char str[])
{

   unsigned int i;

   for (i = 0; i < strlen(str); i++)
   {
      if (str[i] == 32)
         str[i] = '_';
   }

}

#ifndef ENGINE
void simm_color(int index)
{

   glColor3fv(root.color.cmap[index].rgb);

}


void set_hourglass_cursor(double percent)
{

   int index, num_cursors = 11, new_cursor = 20; 
   GLshort junk = 0xfff;

   if (percent > 99.99)
   {
      index = -1;
   }
   else
   {
      index = percent*num_cursors/100.0;
      if (index < 0)
	 index = 0;
      if (index >= num_cursors)
	 index = num_cursors - 1;
   }

   if (index == last_index)
      return;

   if (index == -1)
       glutSetCursor(GLUT_CURSOR_INHERIT);
   else
       glutSetCursor(GLUT_CURSOR_WAIT);

   last_index = index;

}


#ifndef WIN32

void verify_key(void)
{

   int i, day, month, year;
   FILE* fp;
   char userkey[21], cleartext[21], idtext[9], datetext[3];
   unsigned sid;
   SBoolean read_one, valid_id, valid_key, success;
   struct timeval tp;
   struct timezone tzp;
   struct tm* cur_tm;

   sid = sysid(0);

   (void)sprintf(idtext,"%8x", sid);
   for (i=0; i<8; i++)
      if (idtext[i] == ' ')
	 idtext[i] = '0';

   printf("Machine id: %s\n\n", idtext);

   (void)gettimeofday(&tp,&tzp);

   cur_tm = localtime((const time_t*)&tp.tv_sec);

   printf("Today's date: %2d/%2d/%2d\n\n", cur_tm->tm_mon+1, cur_tm->tm_mday,
	  cur_tm->tm_year+1900);

   (void)strcpy(buffer,root.simm_dir);
   (void)strcat(buffer,key_file);

#if 0
   pipe_included = yes;
   printf("Compile time: %s %s\n\n", __TIME__, __DATE__);
   if (cur_tm->tm_year == 99 && cur_tm->tm_mon < 9)
   {
      valid_key = yes;
      return;
   }
   else
   {
      valid_key = no;
      printf("This beta version of SIMM has expired. Call Pete for details.\n");
      exit(0);
   }
#endif

   if ((fp = simm_fopen(buffer,"r")) == NULL)
   {
      fprintf(stderr,"Unable to open SIMM key file: %s\n", buffer);
      exit(0);
   }

   for (i=0, read_one = no, valid_id = no, valid_key = no; i<MAXKEYS; i++)
   {
      if (fscanf(fp,"%20s", userkey) != 1)
         break;
      
      read_one = yes;
      
      decrypt(userkey,cleartext);
      
      for (i=0, success = yes; i<8; i++)
      {
         if (cleartext[i] != idtext[i])
         {
            success = no;
            break;
         }
      }
      if (success == yes)
      {
         valid_id = yes;
      }
      else
      {
         valid_id = no;
         continue;
      }

      datetext[0] = cleartext[8];
      datetext[1] = cleartext[9];
      datetext[2] = STRING_TERMINATOR;
      (void)sscanf(datetext,"%d", &month);

      datetext[0] = cleartext[10];
      datetext[1] = cleartext[11];
      (void)sscanf(datetext,"%d", &day);

      datetext[0] = cleartext[12];
      datetext[1] = cleartext[13];
      (void)sscanf(datetext,"%d", &year);

      if ((verify_date(day,month,year) == yes)
#if BETA_VERSION
	  && (((cur_tm->tm_year == 99) && (cur_tm->tm_mon < 11)) ||
	  ((cur_tm->tm_year == 100) && (cur_tm->tm_mon < 2)))
#endif
	  )
      {
         valid_key = yes;
      }
      else
      {
         valid_key = no;
         continue;
      }
      
      if (STRINGS_ARE_EQUAL(&cleartext[16],"PIPE"))
      {
         pipe_included = yes;
      }
      else if (STRINGS_ARE_EQUAL(&cleartext[16],"NOTP"))
      {
         pipe_included = no;
      }
      else /* Is either a bad, or pre-Y2K, password */
      {
         valid_key = no;
         valid_id = no;
         continue;
      }
   }

   (void)fclose(fp);

   if (valid_key == yes)
      return;

   if (read_one == no)
   {
      fprintf(stderr,"Error reading key from file %s\n", buffer);
      exit(0);
   }

   if (valid_id == yes)
   {
      fprintf(stderr,"This version of SIMM has expired.\n");
      exit(0);
   }

   fprintf(stderr,"The password key in %s is invalid.\nYou must enter a valid one in order to use SIMM on this workstation.\nCall the SIMM technical support line for details.\n", buffer);
	    exit(0);

}
#endif /* WIN32 */


static void decrypt(char ciphertext[], char cleartext[])
{

   int i, offset2, offset3;

   offset2 = ciphertext[0] - 100;

   ciphertext[2] -= offset2;
   ciphertext[3] -= offset2;
   ciphertext[6] -= offset2;
   ciphertext[9] -= offset2;
   ciphertext[13] -= offset2;
   ciphertext[14] -= offset2;
   ciphertext[16] -= offset2;
   ciphertext[19] -= offset2;

   offset3 = ciphertext[1] - 100;

   ciphertext[4] -= offset3;
   ciphertext[8] -= offset3;
   ciphertext[10] -= offset3;
   ciphertext[12] -= offset3;
   ciphertext[15] -= offset3;
   ciphertext[17] -= offset3;

   for (i=0; i<20; i++)
      ciphertext[i] -= key_offset[i];

   for (i=0; i<20; i++)
      cleartext[key_shuffle[i]] = ciphertext[i];

   cleartext[20] = STRING_TERMINATOR;

}


#ifndef WIN32

static SBoolean verify_date(int day, int month, int year)
{

   struct timeval tp;
   struct timezone tzp;
   struct tm* cur_tm;

   /* To workaround the Y2K problem, add 100 to post-2000 dates.
    * This would not work if any passwords were issued before 1990,
    * and will stop working in the year 2090.
    */
   if (year < 90)
      year += 100;

   /* When you made the key, the day and month started at 1. When you call
    * localtime(), it starts the month at 0 (and day at 1, go figure). So
    * subtract 1 from the month that is passed in.
    */

   month--;

   (void)gettimeofday(&tp,&tzp);

   cur_tm = localtime((const time_t*)&tp.tv_sec);

   /* To workaround the Y2K problem, add 100 to post-2000 dates.
    * This would not work if any passwords were issued before 1990,
    * and will stop working in the year 2090.
    */
   if (cur_tm->tm_year < 90)
      cur_tm->tm_year += 100;

   if (cur_tm->tm_year > year)
      return (no);
   else if (cur_tm->tm_year < year)
      return (yes);

   if (cur_tm->tm_mon > month)
      return (no);
   else if (cur_tm->tm_mon < month)
      return (yes);

   if (cur_tm->tm_mday > day)
      return (no);

   return (yes);

}
#endif /* WIN32 */

#endif /* ENGINE */

/* -------------------------------------------------------------------------
   simm_fopen - CodeWarrior's fopen() appears to have a bug in which the "w"
     and "w+" modes do not discard the previous contents of the file being
     opened.  Therefore you should always call this routine instead of fopen()
     to workaround the bug.
---------------------------------------------------------------------------- */
FILE* simm_fopen (const char* name, const char* mode)
{
#ifdef WIN32
   if (mode && mode[0] == 'w')
      remove(name);
#endif
   
   errno = 0;
   
   return fopen(name, mode);
}

/* -------------------------------------------------------------------------
   simm_open - CodeWarrior's open() appears to have a bug in which the
     O_CREAT mode does not discard the previous contents of the file being
     opened.  Therefore you should always call this routine instead of open()
     to workaround the bug.
---------------------------------------------------------------------------- */
int simm_open (const char *name, int oflag, ...)
{
#ifdef WIN32
   if ((oflag & O_CREAT) && ((oflag & O_WRONLY) || (oflag & O_TRUNC)))
      remove(name);
#endif
   
   errno = 0;
   
   return open(name, oflag);
}

/* ---------------------------------------------------------------------------
   simm_lookup_file - this routine looks for the specified file in a list
     of directory paths starting with the first path and moving forward
     through the list until the file is found or the list is exhausted.  If
     the file is found, it is opened using the specified 'mode', and the
     path+name of the file is returned in the buffer pointed to by 'pathList'.
   
   NOTE:  this routine uses strtok() to parse the specified 'pathList'.
     strtok() will modify the pathList by inserting NUL characters.
------------------------------------------------------------------------------ */
FILE* simm_lookup_file (char* pathList, const char* fileName, const char* mode)
{
#define PATH_SEPERATORS ",;"

   char* p;
   
   if (pathList == NULL || fileName == NULL || mode == NULL)
      return NULL;

   for (p = strtok(pathList, PATH_SEPERATORS); p; p = strtok(NULL, PATH_SEPERATORS))
   {
      FILE* f;
      
      char buf[1024] = "";
      
      strcpy(buf, p);
      
      if (buf[strlen(buf) - 1] != DIR_SEP_CHAR)
         strcat(buf, DIR_SEP_STRING);
      
      strcat(buf, fileName);
      
      f = simm_fopen(buf, mode);
      
      if (f)
      {
         strcpy(pathList, buf); /* return path+file in 'pathList' */
         
         return f;
      }
   }
   return NULL;
}

void* simm_malloc(unsigned mem_size)
{

   void* ptr;

   /* Temporary hack so that you don't have to check mem_size before calling
    * simm_malloc()-- Make sure you don't try to malloc 0 bytes because malloc
    * will [appropriately] return NULL. The long-term solution is to check all
    * places where simm_malloc() is called and be smart enough not to call
    * the routine if mem_size is 0.
    */

   if (mem_size <= 0)
      mem_size = sizeof(int);
#if 0
      mem_size += 512;
#endif
   ptr = malloc(mem_size);

   if (ptr == NULL)
   {
		// error() may need to malloc, so don't call it.
      //sprintf(errorbuffer,"Ran out of memory. Unable to malloc %d bytes.",
	      //(int)mem_size);
      //error(none,errorbuffer);
   }

   return (ptr);

}



void* simm_calloc(unsigned num_elements, unsigned elem_size)
{

   void* ptr;

   /* Temporary hack so that you don't have to check mem_size before calling
    * simm_calloc()-- Make sure you don't try to calloc 0 bytes because calloc
    * will [appropriately] return NULL. The long-term solution is to check all
    * places where simm_calloc() is called and be smart enough not to call
    * the routine if mem_size is 0.
    */

   if (num_elements*elem_size <= 0)
   {
      num_elements = 1;
      elem_size = sizeof(int);
   }

   ptr = calloc(num_elements, elem_size);

   if (ptr == NULL)
   {
      sprintf(errorbuffer,"Ran out of memory. Unable to calloc %d bytes.",
	      (int)(num_elements*elem_size));
      error(none,errorbuffer);
   }

   return (ptr);

}



void* simm_realloc(void* ptr, unsigned mem_size, ReturnCode* rc)
{

   void* new_ptr;

/* Temporary hack so that you don't have to check mem_size before calling
 * simm_realloc()-- Make sure you don't try to realloc 0 bytes because realloc
 * will [appropriately] return NULL. The long-term solution is to check all
 * places where simm_realloc() is called and be smart enough not to call
 * the routine if mem_size is 0.
 */

   if (mem_size <= 0)
      mem_size = sizeof(int);

   new_ptr = realloc(ptr, mem_size);

   if (new_ptr == NULL)
   {
      sprintf(errorbuffer,"Ran out of memory. Unable to realloc %d bytes.",
	      (int)mem_size);
      *rc = code_bad;
      return (ptr);
   }

   *rc = code_fine;
   return (new_ptr);

}


char* get_drawmode_name(DrawingMode drawmode)
{

   if (drawmode == wireframe)
      return ("wireframe");
   if (drawmode == solid_fill)
      return ("solid_fill");
   if (drawmode == flat_shading)
      return ("flat_shading");
   if (drawmode == gouraud_shading)
      return ("gouraud_shading");
   if (drawmode == outlined_polygons)
      return ("outlined_polygons");
   if (drawmode == no_surface)
      return ("none");
   if (drawmode == bounding_box)
      return ("bounding_box");

   /* this should really be an error, but return gouraud_shading
    * as if nothing is wrong.
    */
   return ("gouraud_shading");

}


char* get_simmkey_name(int keynum)
{

   return (keys[keynum]);

}


#ifndef ENGINE

/* SET_VIEWPORT: the MODELVIEW and PROJECTION matrices should
 * always be reset when you change the viewport. That's why
 * this utility routine exists.
 */

void set_viewport(int x1, int y1, int xsize, int ysize)
{

   glViewport(x1,y1,xsize,ysize);

   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();

}


/* SET_ORTHO2: this utility function sets a 2D ortho,
 * with or without the infamous 0.5 offset.
 *
 * NOTE: I replaced the "-0.5 +0.5" approach with one I read about in
 *  a document titled "OpenGL Correctness Tips" (from Microsoft's MSDN
 *  online docs).  This new approach involves a glTranslate by 0.375
 *  in the x and y direction.  -- KMS 11/19/98
 */

#define NEW_ORTHO_APPROACH 1

void set_ortho2o(Ortho box)
{
#if NEW_ORTHO_APPROACH
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   gluOrtho2D((int) box.x1, (int) box.x2, (int) box.y1, (int) box.y2);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   glTranslatef(0.375, 0.375, 0.0);
#else
   glOrtho(box.x1-0.5, box.x2+0.5, box.y1-0.5, box.y2+0.5, -1.0, 1.0);
#endif
}

void set_ortho2(double x1, double x2, double y1, double y2)
{
#if NEW_ORTHO_APPROACH
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   gluOrtho2D((int) x1, (int) x2, (int) y1, (int) y2);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   glTranslatef(0.375, 0.375, 0.0);
#else
   glOrtho(x1-0.5, x2+0.5, y1-0.5, y2+0.5, -1.0, 1.0);
#endif

}


/* -------------------------------------------------------------------------
   get_associated_model - return the ModelStruct currently associated with
      the specified window.
---------------------------------------------------------------------------- */
ModelStruct* get_associated_model (int i)
{
   ModelStruct* ms = NULL;
   
   if (i < 0)
      return NULL;
   
   if (root.window[i].type == MODEL)
   {
      ms = root.window[i].win_struct->model;
   }
   else if (root.window[i].type == TOOL)
   {
      ToolStruct* tool = root.window[i].win_struct->tool;
      
      if (tool && tool->query_handler)
         tool->query_handler(GET_TOOL_MODEL, &ms);
   }
   else // plot window or other
   {
      /* If a plot is the topmost window there is only one model,
       * go ahead and return it as the 'selected' model.
       */
      if (root.nummodels == 1)
      {
         int j;

         for (j=0; j<MODELBUFFER; j++)
         {
            if (model[j] != NULL)
            {
               ms = model[j];
               break;
            }
         }
      }
   }

   return ms;
}

/* -------------------------------------------------------------------------
   get_associated_plot - return the PlotStruct currently associated with
      the specified window.
---------------------------------------------------------------------------- */
PlotStruct* get_associated_plot (int i)
{
   PlotStruct* plot = NULL;
   
   if (i < 0)
      return NULL;
   
   if (root.window[i].type == PLOT)
   {
      plot = root.window[i].win_struct->plot;
   }
   else if (root.window[i].type == TOOL)
   {
      ToolStruct* tool = root.window[i].win_struct->tool;
      
      if (tool && tool->query_handler)
         tool->query_handler(GET_TOOL_PLOT, &plot);
   }
   return plot;
}

#endif /* ENGINE */

/* -------------------------------------------------------------------------
   lookup_polyhedron - this routine tries to read the specified polyhedron
     file by first checking the current directory, and then checking the
     standard SIMM directory for bones.
---------------------------------------------------------------------------- */
ReturnCode lookup_polyhedron (PolyhedronStruct* ph, char filename[], ModelStruct* ms)
{
   int i;
   char jointpath[1024], fullpath[1024], tmppath[1024];
   ReturnCode rc = code_bad;

   /* (0) strip the joint file name from ms->jointfilename to get just
    * the path to the joint file.
    */
   if (ms && ms->jointfilename)
   {
      strcpy(jointpath, ms->jointfilename);
      for (i = strlen(jointpath); i >= 0; i--)
      {
         if (jointpath[i] == '\\' || jointpath[i] == '/')
         {
            jointpath[i] = STRING_TERMINATOR;
            break;
         }
      }
		/* If the joint file name is just a file name with no path,
		 * make the jointpath empty so the next step will work.
		 */
		if (i == -1){
			jointpath[0] = '.';	// For jnt file name that has no absolute path we need '.'
			jointpath[1] = STRING_TERMINATOR;
		}
   }

   /* (1) first check the bone directory specified in the joint file: */
   if (rc == code_bad && ms && ms->bonepathname)
   {
      if (ms->jointfilename)
         build_full_path(jointpath, ms->bonepathname, tmppath);
      else
         build_full_path(NULL, ms->bonepathname, tmppath);
      build_full_path(tmppath, filename, fullpath);

      rc = read_polyhedron(ph, fullpath, yes);
   }
   
   /* (2) next check the joint file's local directory (PC only): */
   if (rc == code_bad)
   {
#ifdef WIN32
      if (ms->jointfilename)
         strcat3(tmppath, jointpath, DIR_SEP_STRING, "bones");
      else
         strcpy(tmppath, "bones");
      strcat3(fullpath, tmppath, DIR_SEP_STRING, filename);
   
      rc = read_polyhedron(ph, fullpath, yes);
#else
      rc = code_bad;
#endif
   }

#ifndef ENGINE
   /* (3) check the global bones directory: */
   if (rc == code_bad)
   {
      build_full_path(root.pref.bonefilepath, filename, fullpath);
      
      rc = read_polyhedron(ph, fullpath, yes);
   }

   /* (4) check the mocap bones directory: */
   if (rc == code_bad)
   {
      strcat3(tmppath, root.mocap_dir, DIR_SEP_STRING, "bones");
      strcat3(fullpath, tmppath, DIR_SEP_STRING, filename);
      rc = read_polyhedron(ph, fullpath, yes);
   }
#endif

   return rc;
}


/* -------------------------------------------------------------------------
   is_absolute_path - return yes if the specified path is an absolute path,
      otherwise return no.
---------------------------------------------------------------------------- */
public SBoolean is_absolute_path (const char* path)
{
   if (path == NULL)
      return no;
   
   while (path && *path && isspace(*path))
      path++;

#ifdef WIN32
   if (*path == '/' ||
       *path == DIR_SEP_CHAR ||
       strlen(path) >= 3 && path[1] == ':' && path[2] == DIR_SEP_CHAR)
   {
      return yes;
   }
   else
      return no;
#else
   return (SBoolean) (*path == DIR_SEP_CHAR);
#endif
}

/* -------------------------------------------------------------------------
   append_if_necessary - if the specified character 'c' is not the currently
     the last character in the specified string 'str', then append 'c' to 'str'.
---------------------------------------------------------------------------- */
void append_if_necessary (char* str, char c)
{
   int n = strlen(str);

	if (n == 0)
		return;

   if (str[n-1] != c)
   {
      str[n] = c;
      str[n+1] = '\0';
   }
}

/* -------------------------------------------------------------------------
   build_full_path - this routine combines a "preference path" with a
      "file path" that may contain absolute or relative path specifications
      into a full path.
---------------------------------------------------------------------------- */
void build_full_path (const char* prefPath, const char* filePath, char* fullPath)
{
    if (fullPath == NULL)
       return;
    
    /* ignore any leading spaces in 'prefPath' and 'filePath':
     */
    while (prefPath && *prefPath && isspace(*prefPath))
       prefPath++;
    
    while (filePath && *filePath && isspace(*filePath))
       filePath++;
    
    /* copy the preference path if necessary:
     */
    if (prefPath && ! is_absolute_path(filePath))
    {
       strcpy(fullPath, prefPath);
       
       strip_trailing_white_space(fullPath);
       
       append_if_necessary(fullPath, DIR_SEP_CHAR);
    }
    else
       *fullPath = '\0';
    
    /* then append the file path:
     */
    if (filePath)
    {
       strcat(fullPath, filePath);
       
       strip_trailing_white_space(fullPath);
    }
}

/* -------------------------------------------------------------------------
   get_filename_from_path - 
---------------------------------------------------------------------------- */
const char* get_filename_from_path (const char* pathname)
{
   char *p = NULL, *p1 = NULL, *p2 = NULL;

   p1 = strrchr(pathname, DIR_SEP_CHAR);
   p2 = strrchr(pathname, '/');

   p = MAX(p1,p2);

   return p ? (p+1) : pathname;
}

/* -------------------------------------------------------------------------
   get_pure_path_from_path - this function assumes that there is a filename
   on the end of the path! It has to because there is no way to distinguish
   between a file and a folder at the end of the path.
---------------------------------------------------------------------------- */
void get_pure_path_from_path (const char* fullPath, char** purePath)
{
   int len;
   char *p = NULL, *p1 = NULL, *p2 = NULL;

   p1 = strrchr(fullPath, DIR_SEP_CHAR);
   p2 = strrchr(fullPath, '/');
   p = MAX(p1,p2);

   if (p)
   {
      len = p - fullPath;
      *purePath = (char*)simm_malloc((len+1) * sizeof(char));
      strncpy(*purePath, fullPath, len);
      (*purePath)[len] = STRING_TERMINATOR;
   }
   else
   {
      mstrcpy(purePath, fullPath);
   }
}

/* -------------------------------------------------------------------------
   upperstr - 
---------------------------------------------------------------------------- */
public void upperstr (char* s)
{
   for ( ; *s; s++)
      *s = toupper(*s);
}

/* -------------------------------------------------------------------------
   lowerstr - 
---------------------------------------------------------------------------- */
public void lowerstr (char* s)
{
   for ( ; *s; s++)
      *s = tolower(*s);
}

/* ---------------------------------------------------------------------------
   read_double - I've found that some implementations of scanf("%lf") do
      not read "NaN" correctly.  This routine reads any floating-point number
      including "NaN".    -- KMS 4/21/00
   
   NOTE: this routine was originally static to analog.c.  In moving it here
      I had to replace the call to _read_token() with fscanf("%s").  I don't
      think this will have any side-effects.
------------------------------------------------------------------------------ */
public ReturnCode read_double (FILE* f, double* value)
{
   SBoolean eof = (SBoolean) (fscanf(f, "%s", buffer) != 1);

   if (eof)
      return code_bad;

   lowerstr(buffer);

   if (STRINGS_ARE_EQUAL(buffer, "nan"))
   {
#ifdef __MWERKS__
      *value = NAN;
#elif defined(MS_VISUAL_C) || defined(__linux__)
	  *value = 0.0;  /* ACK! can't seem to find a way to assign NAN in VC++!! */
#else
      NaN(*value);
#endif      
   }
   else if (isdigit(buffer[0]) || buffer[0] == '.' || buffer[0] == '-' || buffer[0] == '+')
      *value = atof(buffer);
   else
      return code_bad;

   return code_fine;
}


/* ---------------------------------------------------------------------------
   read_double_tab - I've found that some implementations of scanf("%lf") do
      not read "NaN" correctly.  This routine reads any floating-point number
      including "NaN".    -- KMS 4/21/00
   
   This function is like read_double(), but is designed for reading tab-delimited
   numbers (e.g., from XLS files). In these files, two tabs in a row means that
   a number field is empty-- detect this, read just the first tab from the file,
   and return 0.0 for the number.
------------------------------------------------------------------------------ */
public ReturnCode read_double_tab(FILE* f, double* value)
{
   SBoolean eof;
   char c;
   long position = ftell(f);

   /* Two tabs in a row is the only allowable way to specify an empty number field,
    * so read two characters from the file and see if they are both tabs.
    */
   c = fgetc(f);
   if (c == '\t')
   {
      position = ftell(f);
      c = fgetc(f);
      if (c == '\t')
      {
         /* Put the second tab back and return 0.0 for the empty field. */
         fseek(f, position, SEEK_SET);
         *value = 0.0;
         return code_fine;
      }
   }

   /* Go back to the saved position to read the number. */
   fseek(f, position, SEEK_SET);

   eof = (SBoolean) (fscanf(f, "%s", buffer) != 1);

   if (eof)
      return code_bad;

   lowerstr(buffer);

   if (STRINGS_ARE_EQUAL(buffer, "nan"))
   {
#ifdef __MWERKS__
      *value = NAN;
#elif defined(MS_VISUAL_C) || defined(__linux__)
	  *value = 0.0;  /* ACK! can't seem to find a way to assign NAN in VC++!! */
#else
      NaN(*value);
#endif      
   }
   else if (isdigit(buffer[0]) || buffer[0] == '.' || buffer[0] == '-' || buffer[0] == '+')
      *value = atof(buffer);
   else
      return code_bad;

   return code_fine;
}


#ifndef ENGINE
/* DRAWPOLYNOMIAL: */

void draw_polynomial(double x1, double y1, double x2, double b, double c,
			            double d, int steps, double lineWidth)
{

   int i;
   double stepsize, pnt[2];

   stepsize = (x2-x1)/steps;
   if (EQUAL_WITHIN_ERROR(stepsize,0.0))
      return;

//   glLineWidth(2.0);
   glLineWidth(lineWidth);

   glBegin(GL_LINE_STRIP);

   for (i=0; i<=steps; i++)
   {
      pnt[0] = i*stepsize;
      pnt[1] = y1 + pnt[0]*(b + pnt[0]*(c + pnt[0]*d));
      pnt[0] += x1;
      glVertex2dv(pnt);
   }

   glEnd();

   glLineWidth(1.0);

}

#endif

/* Find an unused function number in the function array.  If all are used
 * allocate space for more functions and add a new one onto the end of
 * the array.
 */ 
int findUnusedFunctionNumber(int mod)
{
   int i, funcnum;

   for (i=0; i<model[mod]->func_array_size; i++)
      if (model[mod]->function[i].used == no)
	      break;

   if (i == model[mod]->func_array_size)
      funcnum = enter_function(mod,-1);
   else
   {
      funcnum = i;
      model[mod]->numfunctions++;
   }

   if (funcnum == -1)
   {
      sprintf(buffer, "findUnusedFunctionNumber: Could not allocate a new function.\n");
      error(none, buffer);
   }
   return(funcnum);
}

/* return the highest user function number */
int findHighestUserFuncNum(int mod)
{
   int i, highest_user_num = -1;

   for (i=0; i<model[mod]->func_array_size; i++)
   {
	   if (model[mod]->function[i].used == yes &&
	      model[mod]->function[i].usernum > highest_user_num)
	      highest_user_num = model[mod]->function[i].usernum;
   }

   return(highest_user_num);

}

int strings_equal_case_insensitive(const char str1[], const char str2[])
{
#if defined(__linux__)
	return !strcasecmp(str1,str2);
#elif defined(WIN32)
	return !stricmp(str1,str2);
#else
   char buf1[1024];

   /* make the strings upper case and compare them */
   strcpy(buffer, str1);
   _strupr(buffer);

   strcpy(buf1, str2);
   _strupr(buf1);

   return !strcmp(buffer, buf1);
#endif
}

int strings_equal_n_case_insensitive(const char str1[], const char str2[], int n)
{
#if defined(__linux__)
	return !strncasecmp(str1,str2,n);
#elif defined(WIN32)
	return !strnicmp(str1,str2,n);
#else
   char buf1[1024];

   if ((int)strlen(str1) < n || (int)strlen(str2) < n)
      return 0;

   /* make the strings upper case and compare them */
   strncpy(buffer, str1, n);
   buffer[n] = STRING_TERMINATOR;
   _strupr(buffer);

   strncpy(buf1, str2, n);
   buf1[n] = STRING_TERMINATOR;
   _strupr(buf1);

   return !strcmp(buffer, buf1);
#endif
}


void addNameToString(char name[], char string[], int maxStringSize)
{
   int newLen = strlen(name) + 2; // size of name + ", "
   int curLen = strlen(string);

   /* Add the name to the string as long as there is room
    * for it plus ", " plus "..."
    */
   if (curLen + newLen + 5 < maxStringSize)
   {
      /* if curLen > 1, assume there's already a name in the string */
      if (curLen > 1)
         strcat(string, ", ");
      strcat(string, name);
   }
   else if (curLen + 5 < maxStringSize)
   {
      /* if there is room for "..." and the string doesn't already end with "...", add it. */
      if (strcmp(&string[curLen-3], "..."))
         strcat(string, ", ...");
   }
}


void simmPrintMultiLines(char string[], SBoolean hilite, int lineSize, int pixelIndent)
{
   int simmMsgFormat = 0;
   int len, start = 0, end = 0;

   if (!string || lineSize <= 0 || pixelIndent < 0)
      return;

#ifndef ENGINE
   if (hilite)
   {
      simmMsgFormat += HIGHLIGHT_TEXT;
      deiconify_message_window();
   }
#endif

   len = strlen(string);

   while (end < len)
   {
      end = start + lineSize - 1;  // -1 because you include the char at 'end'
      if (end >= len)
      {
         end = len;
      }
      else
      {
         for (; end > start; end--)
         {
            if (CHAR_IS_WHITE_SPACE(string[end]))
               break;
         }
         /* If end == start, there is no white space in the line,
          * so set end back to what it was and split a word.
          */
         if (end == start)
            end = start + lineSize - 1;
      }
      strncpy(buffer, &string[start], end - start + 1);
      buffer[end - start + 1] = STRING_TERMINATOR;
      message(buffer, simmMsgFormat, pixelIndent);
      start = end + 1;
   }
}


static SBoolean read_pref(FILE* file, char* name, char* value)
{
   char *p = value;
   
   while (1)
   {
      if (fscanf(file, "%s", name) != 1)
        return no;
   
      if (name[0] != '#')
         break;
      else
      {
         char buf[256];   /* ignore the commented line */
      
         fgets(buf, sizeof(buf), file);
      }
   }
   
   read_line(&file, value);
   _strip_outer_whitespace(value);
   
   return yes;
}


/* getprev - this routine is similar to the OS routine getenv().  In fact,
 *   under Unix, it simply calls getenv().  Under MS-Windows this routine
 *   opens the SIMM preference file and searches for the specified variable
 *   name.  If found it returns a pointer to the variable's value, just like
 *   getenv().  -- KMS 11/11/98
 */
const char* getpref(const char* prefname)
{
   char name[256];
   FILE* file;
   const char* result = NULL;

   strcpy(name, get_simm_resources_directory());
   strcat(name, "preferences.txt");

   file = simm_fopen(name, "r");

   if (!file)
   {
   	strcpy(name, get_simm_resources_directory());
   	strcat(name, "preferences");
   	file = simm_fopen(name, "r");
   }

   if (file)
   {
      static char value[256];

      while (read_pref(file, name, value))
      {
         if (STRINGS_ARE_EQUAL(name, prefname))
         {
            result = value;
            break;
         }
      }
      fclose(file);
   }

   return result;
}

#ifndef ENGINE

/* -------------------------------------------------------------------------
   lock_model - acquire the realtime mutex, but only if the model is currently
   receiving realtime motion data from EVaRT or from its simulation dll.
---------------------------------------------------------------------------- */
void lock_model(ModelStruct* ms)
{
   if (is_model_realtime(ms) == rtNotConnected)
      return;

   /* If model is NULL, acquire the locks for all models. */
   if (ms == NULL)
   {
      int i;

      for (i = 0; i < MODELBUFFER; i++)
      {
         if (model[i] && model[i]->modelLock)
            glutAcquireMutex(model[i]->modelLock);
      }
   }
   else
   {
      glutAcquireMutex(ms->modelLock);
   }
}


/* -------------------------------------------------------------------------
   unlock_model - release the realtime mutex, but only if the model is currently
   receiving realtime motion data from EVaRT or from its simulation dll.
---------------------------------------------------------------------------- */
void unlock_model(ModelStruct* ms)
{
   if (is_model_realtime(ms) == rtNotConnected)
      return;

   /* If model is NULL, release the locks for all models. */
   if (ms == NULL)
   {
      int i;

      for (i = 0; i < MODELBUFFER; i++)
      {
         if (model[i] && model[i]->modelLock)
            glutReleaseMutex(model[i]->modelLock);
      }
   }
   else
   {
      glutReleaseMutex(ms->modelLock);
   }
}

#endif

/* -------------------------------------------------------------------------
   is_model_realtime - returns the current state of the model's realtime
   connection:
      rtMocap:        connected to EVaRT
      rtSimulation:   connected to a simulation dll
      rtNotConnected: not connected to either
   If 'ms' is NULL this function scans all of the models and returns the
   first realtime state that is not rtNotConnected.
---------------------------------------------------------------------------- */
RTConnection is_model_realtime(ModelStruct* ms)
{
   if (ms == NULL)
   {
      int i;

      for (i = 0; i < MODELBUFFER; i++)
      {
         if (model[i] && model[i]->realtimeState != rtNotConnected)
            return model[i]->realtimeState;
      }
      return rtNotConnected;
   }
   else
   {
      return ms->realtimeState;
   }
}

int makeDir(const char aDirName[])
{
#ifdef __linux__
	return mkdir(aDirName,S_IRWXU);
#else
	return _mkdir(aDirName);
#endif
}
