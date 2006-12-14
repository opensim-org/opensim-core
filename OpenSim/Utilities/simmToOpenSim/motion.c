/*******************************************************************************

   MOTION.C

   Author: Peter Loan

   Date: 10-SEP-90

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Description:

   Routines:
      load_motion           : reads a motion file to fill a motion structure
      apply_motion_to_model : moves a model according to a motion
      column_names_match_model : checks motion column names
      check_motion_wrapping : checks boundaries of motion variable
      find_nth_motion       : returns the appropriate motion

*******************************************************************************/
#include <assert.h>

#ifdef __MWERKS__
   #include <unistd.h>
#endif

#include "universal.h"
#include "globals.h"
#include "functions.h"
#include "normio.h"
#include "normtools.h"
//#include "password.h"

/*************** DEFINES (for this file only) *********************************/
#define MOTION_OFFSET 0.0001
#define OTHER_NAMES_SIZE 3000

/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/

/* ==== DEFAULT MOTION OBJECTS:
 */
static MotionObject sDefaultMotionObjects[] = {
   {
      "force" ,              /* name */
      "arrow.asc",           /* filename */
      no,                    /* defined_in_file */
      { 0,0,0 },             /* startingPosition */
      { 1.0, 0.002, 1.0 },   /* startingScale (see NOTE below for more info) */
      { 0,0,0 },             /* starting XYZRotation */
      flat_shading,          /* drawmode */
      YY,                    /* vector_axis */
      "def_motion"           /* materialname */
   },
   {
      "ball" ,               /* name */
      "unit_sphere.asc",     /* filename */
      no,                    /* defined_in_file */
      { 0,0,0 },             /* startingPosition */
      { 1,1,1 },             /* startingScale */
      { 0,0,0 },             /* starting XYZRotation */
      gouraud_shading,       /* drawmode */
      YY,                    /* vector_axis */
      "def_motion"           /* materialname */
   },
   {
      "contact" ,            /* name */
      "arrow.asc",           /* filename */
      no,                    /* defined_in_file */
      { 0,0,0 },             /* startingPosition */
      { 1.0, 1.0, 1.0 },     /* startingScale */
      { 0,0,0 },             /* starting XYZRotation */
      flat_shading,          /* drawmode */
      YY,                    /* vector_axis */
      "def_motion"           /* materialname */
   },
   {
      "joint_force" ,        /* name */
      "arrow.asc",           /* filename */
      no,                    /* defined_in_file */
      { 0,0,0 },             /* startingPosition */
      { 1.0, 1.0, 1.0 },     /* startingScale */
      { 0,0,0 },             /* starting XYZRotation */
      flat_shading,          /* drawmode */
      YY,                    /* vector_axis */
      "def_joint_vector"     /* materialname */
   },
   {
      "joint_torque" ,       /* name */
      "arrow.asc",           /* filename */
      no,                    /* defined_in_file */
      { 0,0,0 },             /* startingPosition */
      { 1.0, 1.0, 1.0 },     /* startingScale */
      { 0,0,0 },             /* starting XYZRotation */
      flat_shading,          /* drawmode */
      YY,                    /* vector_axis */
      "def_joint_vector"     /* materialname */
   },
   {
      "spring_force" ,       /* name */
      "arrow.asc",           /* filename */
      no,                    /* defined_in_file */
      { 0,0,0 },             /* startingPosition */
      { 0.25, 1.0, 0.25 },     /* startingScale */
      { 0,0,0 },             /* starting XYZRotation */
      flat_shading,          /* drawmode */
      YY,                    /* vector_axis */
      "def_muscle_point"     /* materialname */
   },
   {
      "torque" ,             /* name */
      "arrow.asc",           /* filename */
      no,                    /* defined_in_file */
      { 0,0,0 },             /* startingPosition */
      { 1.0, 1.0, 1.0 },     /* startingScale */
      { 0,0,0 },             /* starting XYZRotation */
      flat_shading,          /* drawmode */
      YY,                    /* vector_axis */
      "def_joint_vector"     /* materialname */
   }
};

/* NOTE: the starting scale factors (0.75, 0.002, 0.75) for the default "force"
 *  arrow motion object were chosen to match SIMM's previous behavior.  -- KMS 12/20/99
 * NOTE: arrow.asc was changed to include the X and Z 0.75 factors. Thus the
 *  "force" scale factor was changed to 1.0, 0.002, 1.0 -- JPL 9/6/00
 */
 

/*************** GLOBAL VARIABLES (used in only a few files) ******************/


/*************** EXTERNED VARIABLES (declared in another file) ****************/
extern char natural_cubic_text[];
extern char gcv_text[];


/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static SBoolean column_names_match_model(ModelStruct* ms, MotionSequence* motion,
                                         int* num_other_data, char otherNames[]);
static void set_plot_cursors(ModelStruct *ms, MotionSequence* motion);
static void make_motion_curve_menu(int mod, MotionSequence* motion, int mnum);
static void store_motion_object_instance(int mod, MotionModelOptions* mopt, int seg, int motion_object, int compenant, int column);
static void calc_transform_mat(double vx, double vy, double vz, double mat[][4]);


#ifndef ENGINE
MotionSequence* load_motion(char filename[], int mod)
{

   int i, j, nk, len, rd, sd, numread, old_size, num_cols = 0;
   char key1[64], key2[64];
   int numstddevs, num_errors=0;
   SBoolean* col_is_std_dev;
   double** std_dev = NULL;
   char** col_name = NULL;
   FILE *fp;
   char fullpath[CHARBUFFER];
   MotionSequence* motion = NULL;
   ReturnCode rc;
   ModelStruct* ms = model[mod];

#ifdef WIN32
   strcpy(fullpath, filename);
#else
   if (filename[0] == DIR_SEP_CHAR)
       strcpy(fullpath, filename);
   else
       build_full_path(root.pref.jointfilepath, filename, fullpath);
#endif

   if ((fp = preprocess_file(fullpath,glutGetTempFileName(".motion"))) == NULL)
   {
      (void)sprintf(errorbuffer,"Unable to open motion file \"%s\"", fullpath);
      error(abort_action,errorbuffer);
      goto cleanup;
   }

   motion = createMotionStruct(ms);

   /* createMotionStruct() uses simm_calloc() to init the struct,
    * so you only need to init fields that are not 0.
    */
   motion->calc_derivatives = no;
   motion->wrap = no;
   motion->enforce_loops = yes;
   motion->enforce_constraints = yes;
   motion->is_realtime = no;
   motion->min_value = 0.0;
   motion->max_value = 1.0;
   motion->show_cursor = no;
   motion->event_color[0] = root.color.cmap[MISC_MOTION_EVENTS].rgb[0];
   motion->event_color[1] = root.color.cmap[MISC_MOTION_EVENTS].rgb[1];
   motion->event_color[2] = root.color.cmap[MISC_MOTION_EVENTS].rgb[2];
   motion->keys[0] = motion->keys[1] = null_key;
   mstrcpy(&motion->name, filename);

   while (1)
   {
      if (read_string(&fp,buffer) == EOF)
         break;

      if (buffer[0] == '#')
      {
         read_nonempty_line(&fp,buffer);
         continue;
      }

      if (STRINGS_ARE_EQUAL(buffer,"endheader"))
         break;

      else if (STRINGS_ARE_EQUAL(buffer,"name"))
      {
         if (read_line(&fp, buffer) != EOF)
         {
            strip_trailing_white_space(buffer);
            FREE_IFNOTNULL(motion->name);
            mstrcpy(&motion->name, buffer);
         }
         else
         {
            error(abort_action,"Error reading name of motion.");
            goto error_cleanup;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"wrap"))
      {
         motion->wrap = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "enforce_loops"))
      {
         if (read_string(&fp, buffer) == EOF)
            break;

         if (STRINGS_ARE_EQUAL(buffer,"no") || STRINGS_ARE_EQUAL(buffer,"off")
            || STRINGS_ARE_EQUAL(buffer, "false"))
            motion->enforce_loops = no;
         else if (STRINGS_ARE_EQUAL(buffer, "yes") || STRINGS_ARE_EQUAL(buffer, "on")
            ||STRINGS_ARE_EQUAL(buffer, "true"))
            motion->enforce_loops = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "enforce_constraints"))
      {
         if (read_string(&fp, buffer) == EOF)
            break;

         if (STRINGS_ARE_EQUAL(buffer,"no") || STRINGS_ARE_EQUAL(buffer,"off")
            || STRINGS_ARE_EQUAL(buffer, "false"))
            motion->enforce_constraints = no;
         else if (STRINGS_ARE_EQUAL(buffer, "yes") || STRINGS_ARE_EQUAL(buffer, "on")
            ||STRINGS_ARE_EQUAL(buffer, "true"))
            motion->enforce_constraints = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"datacolumns"))
      {
         if (fscanf(fp,"%d", &motion->number_of_datacolumns) != 1)
         {
            (void)sprintf(errorbuffer,"Error reading datacolumns in motion %s", motion->name);
            error(abort_action,errorbuffer);
            goto error_cleanup;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"keys"))
      {
         read_line(&fp,buffer);
         nk = sscanf(buffer,"%s %s", key1, key2);
         if (nk == 1)
            motion->keys[0] = motion->keys[1] = lookup_simm_key(key1);
         else if (nk == 2)
         {
            motion->keys[0] = lookup_simm_key(key1);
            motion->keys[1] = lookup_simm_key(key2);
         }
         else
         {
            (void)sprintf(errorbuffer,"Error reading keys for motion %s", motion->name);
            error(recover,errorbuffer);
            motion->keys[0] = motion->keys[1] = 0;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"datarows"))
      {
         if (fscanf(fp,"%d", &motion->number_of_datarows) != 1)
         {
            (void)sprintf(errorbuffer,"Error reading datarows for motion %s", motion->name);
            error(abort_action,errorbuffer);
            goto error_cleanup;
         }
         if (motion->number_of_datarows < 1)
         {
            (void)sprintf(errorbuffer,"\"datarows\" in motion %s must be greater than 0", motion->name);
            error(abort_action,errorbuffer);
            goto error_cleanup;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"otherdata"))
      {
         fscanf(fp,"%*d");
         error(none, "Keyword \'otherdata\' is no longer required in a motion header.");
      }
      else if (STRINGS_ARE_EQUAL(buffer,"range"))
      {
         if (fscanf(fp,"%lg %lg", &motion->min_value, &motion->max_value) != 2)
         {
            (void)sprintf(errorbuffer,"Error reading range for motion %s", motion->name);
            error(none,errorbuffer);
            error(none,"Using default values 0.0 to 100.0");
            motion->min_value = 0.0;
            motion->max_value = 100.0;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "units"))
      {
         read_line(&fp, buffer);
         _strip_outer_whitespace(buffer);
         mstrcpy(&motion->units, buffer);
      }
      else if (STRINGS_ARE_EQUAL(buffer,"cursor"))
      {
         if (fscanf(fp,"%f %f %f", &motion->cursor_color[0],
            &motion->cursor_color[1], &motion->cursor_color[2]) != 3)
         {
            (void)sprintf(errorbuffer,"Error reading cursor color for motion %s",
               motion->name);
            error(none,errorbuffer);
            error(none,"Using default values of 1.0 1.0 0.0");
            motion->cursor_color[0] = 1.0;
            motion->cursor_color[1] = 1.0;
            motion->cursor_color[2] = 0.0;
         }
         else
         {
         /* Support for 'old style' colors (range: 0 to 255). If any of the
         * color components is greater than 1.0, assume that it's an old
         * style color definition.
            */
            if (motion->cursor_color[0] > 1.0 ||
               motion->cursor_color[1] > 1.0 ||
               motion->cursor_color[2] > 1.0)
            {
               motion->cursor_color[0] /= 255.0;
               motion->cursor_color[1] /= 255.0;
               motion->cursor_color[2] /= 255.0;
            }
            for (i=0; i<3; i++)
            {
               if (motion->cursor_color[i] < 0.0)
                  motion->cursor_color[i] = 0.0;
            }
         }
         motion->show_cursor = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"event"))
      {
         if (motion->event)
         {
            motion->event = (MotionEvent*)simm_realloc(motion->event,
               (motion->num_events+1) * sizeof(MotionEvent), &rc);
         }
         else
         {
            motion->event = (MotionEvent*)simm_malloc((motion->num_events+1) * sizeof(MotionEvent));
         }

         if (!motion->event)
         {
            error(recover, "Unable to malloc space to hold motion event.");
         }
         else if (fscanf(fp,"%lg", &motion->event[motion->num_events].x_coord) != 1)
         {
            (void)sprintf(errorbuffer,"Error reading event time in motion %s", motion->name);
            error(recover,errorbuffer);
         }
         else if (read_nonempty_line(&fp,buffer) == EOF)
         {
            (void)sprintf(errorbuffer,"Error reading event name in motion %s", motion->name);
            error(recover,errorbuffer);
         }
         else
         {
            mstrcpy(&motion->event[motion->num_events].name,buffer);
            motion->num_events++;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"event_color"))
      {
         if (fscanf(fp,"%f %f %f", &motion->event_color[0],
            &motion->event_color[1], &motion->event_color[2]) != 3)
         {
            (void)sprintf(errorbuffer,"Error reading event_color for motion %s",
               motion->name);
            error(none,errorbuffer);
            error(none,"Using default values from color file");
            motion->event_color[0] = root.color.cmap[MISC_MOTION_EVENTS].rgb[0];
            motion->event_color[1] = root.color.cmap[MISC_MOTION_EVENTS].rgb[1];
            motion->event_color[2] = root.color.cmap[MISC_MOTION_EVENTS].rgb[2];
         }
         else
         {
         /* Support for 'old style' colors (range: 0 to 255). If any of the
         * color components is greater than 1.0, assume that it's an old
         * style color definition.
            */
            if (motion->event_color[0] > 1.0 ||
               motion->event_color[1] > 1.0 ||
               motion->event_color[2] > 1.0)
            {
               motion->event_color[0] /= 255.0;
               motion->event_color[1] /= 255.0;
               motion->event_color[2] /= 255.0;
            }
            for (i=0; i<3; i++)
            {
               if (motion->event_color[i] < 0.0)
                  motion->event_color[i] = 0.0;
            }
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"calc_derivatives"))
      {
         if (fscanf(fp,"%lf", &motion->time_step) != 1)
         {
            (void)sprintf(errorbuffer,"Error reading time_step for motion %s", motion->name);
            error(recover,errorbuffer);
         }
         else if (motion->time_step < 0.0)
            error(recover,"Time step must be greater than zero.");
         else
            motion->calc_derivatives = yes;
      }
#if INCLUDE_MOCAP_MODULE
      else if (STRINGS_ARE_EQUAL(buffer,"accept_realtime_motion"))
         motion->is_realtime = no; // this is no longer supported
      else if (STRINGS_ARE_EQUAL(buffer,"sliding_time_scale"))
         motion->sliding_time_scale = yes;
#endif
      else
      {
         (void)sprintf(errorbuffer,"Unrecognized string \"%s\" found in motion file.",
            buffer);
         error(recover,errorbuffer);
         num_errors++;
      }
      if (num_errors > 10)
      {
         error(none,"Too many errors to continue.");
         error(none,"Unable to load motion file.");
         goto error_cleanup;
      }
   }

   col_name = (char**)simm_calloc(motion->number_of_datacolumns, sizeof(char*));
   num_cols = motion->number_of_datacolumns;
   if (col_name == NULL)
   {
      error(none,"Unable to load motion.");
      goto error_cleanup;
   }

   numread = 0;
   for (i=0; i<motion->number_of_datacolumns; i++)
   {
      numread += fscanf(fp,"%s",buffer);
      mstrcpy(&col_name[i],buffer);
   }
   if (numread < motion->number_of_datacolumns)
   {
      (void)sprintf(buffer,"Error reading names of data columns. Only %d of %d read.",
	      numread, motion->number_of_datacolumns);
      error(abort_action,buffer);
      goto error_cleanup;
   }

   /* Peek ahead to see if there are more column names specified. */
   while (1)
   {
      int c = fgetc(fp);
      
      if (isdigit(c) || c == '.' || c == '-' || c == '+')
      {
         ungetc(c, fp);
         break;
      }
      else if (c == '\n' || c == '\r' || c == '\t')
      {
      }
      else if ( ! isspace(c))
      {
         ungetc(c, fp);
         (void)sprintf(buffer,"Extra column names are specified in motion file. Check names and value of \'datacolumns\'.");
         error(abort_action,buffer);
         goto error_cleanup;
      }
   }

   /* Figure out how many datacolumns are standard deviations. If there are 17
    * columns, and 5 are standard deviations, then you want to malloc an array
    * of size 12 for motiondata (and fill it with the 12 non-stddev data), and
    * an array of size 12 for the standard deviations (5 to be filled with data,
    * and 7 of them to be NULL).
    */

   col_is_std_dev = (SBoolean*)simm_malloc(motion->number_of_datacolumns*sizeof(SBoolean));

   for (i=0, numstddevs=0; i<motion->number_of_datacolumns; i++)
   {
      col_is_std_dev[i] = no;
      len = strlen(col_name[i]);
      if (len < 5)
         continue;
      if (STRINGS_ARE_EQUAL(&col_name[i][len-4],"_std"))
      {
         col_is_std_dev[i] = yes;
         numstddevs++;
      }
   }

   /* Now make sure that there is a "data" column for every "data_std" column. */

   for (i=0; i<motion->number_of_datacolumns; i++)
   {
      if (col_is_std_dev[i] == no)
         continue;
      len = strlen(col_name[i]);
      strcpy(buffer,col_name[i]);
      buffer[len-4] = STRING_TERMINATOR;
      for (j=0; j<motion->number_of_datacolumns; j++)
      {
         if (j == i)
            continue;
         if (STRINGS_ARE_EQUAL(buffer,col_name[j]))
            break;
      }
      if (j == motion->number_of_datacolumns)
      {
         (void)sprintf(errorbuffer,"Error reading motion file %s:", fullpath);
         error(none,errorbuffer);
         (void)sprintf(errorbuffer,"   You must specify %s when specifying %s",
            buffer, col_name[i]);
         error(abort_action,errorbuffer);
         goto error_cleanup;
      }
   }
   
   motion->number_of_datacolumns -= numstddevs;

   motion->motiondata = (double**)simm_malloc(motion->number_of_datacolumns*sizeof(double*));
   motion->data_std_dev = (double**)simm_malloc(motion->number_of_datacolumns*sizeof(double*));
   std_dev = (double**)simm_malloc(numstddevs*sizeof(double*));
   if (motion->motiondata == NULL || motion->data_std_dev == NULL || std_dev == NULL)
   {
      error(none,"Unable to load motion.");
      goto error_cleanup;
   }

   for (i=0; i<motion->number_of_datacolumns; i++)
   {
      motion->data_std_dev[i] = NULL;
      motion->motiondata[i] = (double*)simm_malloc(motion->number_of_datarows*sizeof(double));
      if (motion->motiondata[i] == NULL)
      {
         error(none,"Unable to load motion.");
         for (j=0; j<i; j++)
            FREE_IFNOTNULL(motion->motiondata[j]);
         goto error_cleanup;
      }
   }
   
   for (i=0; i<numstddevs; i++)
   {
      std_dev[i] = (double*)simm_malloc(motion->number_of_datarows*sizeof(double));
      if (std_dev[i] == NULL)
      {
         error(none,"Unable to load motion.");
         for (j=0; j<motion->number_of_datacolumns; j++)
            FREE_IFNOTNULL(motion->motiondata[j]);
         for (j=0; j<i; j++)
            FREE_IFNOTNULL(std_dev[i]);
         goto error_cleanup;
      }
   }
   
   for (i=0; i<motion->number_of_datarows; i++)
   {
      numread = 0;
      for (j=0, rd=0, sd=0; j<motion->number_of_datacolumns + numstddevs; j++)
      {
         if (col_is_std_dev[j] == no)
         {
            if (read_double(fp, &motion->motiondata[rd++][i]) == code_fine)
               numread++;
         }
         else
         {
            if (read_double(fp, &std_dev[sd++][i]) == code_fine)
               numread++;
         }
      }
      if (numread < motion->number_of_datacolumns + numstddevs)
      {
         (void)sprintf(buffer,"Error reading motion file %s:", fullpath);
         error(none,buffer);
         (void)sprintf(buffer,"   Only %d of %d datarows found.",
            i, motion->number_of_datarows);
         error(recover,buffer);
         motion->number_of_datarows = i;
         break;
      }
   }
   
   /* init the motion's model options array */
   motion->mopt.num_motion_object_instances = 0;
   motion->mopt.motion_object_instances = NULL;
   motion->mopt.gencoords = NULL;
   motion->mopt.genc_velocities = NULL;
   motion->mopt.muscles = NULL;
   motion->mopt.ligaments = NULL;
   motion->mopt.other_data = NULL;

   if (motion->number_of_datarows == 0 || motion->number_of_datacolumns == 0)
   {
      error(none,"Motion contains no recognizable data.");
      goto error_cleanup;
   }

   /* Now copy the column names that are not standard deviations to the columnname
    * variable.
    */
   motion->columnname = (char**)simm_malloc(motion->number_of_datacolumns*sizeof(char*));
   for (i=0, rd=0; i<motion->number_of_datacolumns + numstddevs; i++)
      if (col_is_std_dev[i] == no)
         mstrcpy(&motion->columnname[rd++],col_name[i]);

   /* Now that you have read in all the data, fill-in the std_dev array in the
    * motion structure with the right data.
    */
   for (i=0, sd=0; i<motion->number_of_datacolumns + numstddevs; i++)
   {
      if (col_is_std_dev[i] == no)
         continue;
      len = strlen(col_name[i]);
      strcpy(buffer,col_name[i]);
      buffer[len-4] = STRING_TERMINATOR;
      for (j=0; j<motion->number_of_datacolumns; j++)
      {
         if (STRINGS_ARE_EQUAL(buffer,motion->columnname[j]))
            break;
      }
      if (j == motion->number_of_datacolumns)
         continue;
      motion->data_std_dev[j] = std_dev[sd++];
   }

   if ( ! is_in_demo_mode())
   {
      (void)sprintf(buffer,"Read motion file %s", filename);
      message(buffer,0,DEFAULT_MESSAGE_X_OFFSET);
   }
   
   if (tie_motion_to_model(motion, model[mod]) == no)
   {
      (void)sprintf(buffer,"Did not tie motion %s to model %s.", motion->name, model[mod]->name);
      error(none,buffer);
      delete_motion(ms, motion);
      goto error_cleanup;
   }

   /* If user wants SIMM to calculate derivatives, calculate them now. */

   if (motion->calc_derivatives == yes)
   {
      if (setup_motion_derivatives(motion) == code_bad)
      {
         delete_motion(ms, motion);
         goto error_cleanup;
      }

      link_derivs_to_model(ms, motion);
   }

   goto close_and_cleanup;

error_cleanup:
   delete_motion(ms, motion);
   motion = NULL;

close_and_cleanup:
   (void)fclose(fp);
   (void)unlink(glutGetTempFileName(".motion"));

cleanup:
   if (col_name)
   {
      for (i = 0; i < num_cols; i++)
         FREE_IFNOTNULL(col_name[i]);
      FREE_IFNOTNULL(col_name);
   }

   return motion;
}


static void make_motion_curve_menu(int mod, MotionSequence* motion, int mnum)
{

   int i, gc, m, ref, motion_object;
   SplineType sType;
   double cutoffFreq;
   char* ptr = NULL;

   motion->mopt.gencoordmenu = glueCreateMenu("Generalized Coordinates");
   motion->mopt.musclemenu = glueCreateMenu("Muscle Activations");
   motion->mopt.forceplatemenu = glueCreateMenu("Force Plates");
   motion->mopt.segmentforcemenu = glueCreateMenu("Body Segment Forces");
   motion->mopt.markermenu = glueCreateMenu("Markers");
   motion->mopt.motionobjectmenu = glueCreateMenu("Motion Objects");
   motion->mopt.othermenu = glueCreateMenu("Other Variables");

   for (i=0; i<motion->number_of_datacolumns; i++)
   {
      if ((gc = name_is_gencoord(motion->columnname[i], model[mod], NULL, &sType, &cutoffFreq, no)) >= 0)
      {
         glueAddMenuEntryWithValue(motion->mopt.gencoordmenu, model[mod]->gencoord[gc].name, (mnum + 1) * MAXMOTIONCURVES+i);
      }
      else if ((gc = name_is_gencoord(motion->columnname[i], model[mod], "_vel", &sType, &cutoffFreq, no)) >= 0)
      {
         sprintf(buffer, "%s_vel", model[mod]->gencoord[gc].name);
         glueAddMenuEntryWithValue(motion->mopt.gencoordmenu, buffer, (mnum + 1) * MAXMOTIONCURVES+i);
      }
      else if ((m = name_is_muscle(model[mod], motion->columnname[i], NULL, &sType, &cutoffFreq, no)) >= 0)
      {
         glueAddMenuEntryWithValue(motion->mopt.musclemenu, model[mod]->muscle[m].name, (mnum + 1) * MAXMOTIONCURVES+i);
      }
      else if (get_ligament_index(mod,motion->columnname[i]) >= 0)
      {
         glueAddMenuEntryWithValue(motion->mopt.musclemenu, motion->columnname[i], (mnum + 1) * MAXMOTIONCURVES+i);
      }
      else if (name_is_marker(model[mod], motion->columnname[i], &motion_object, &ref, &sType, &cutoffFreq, no) >= 0 &&
               motion_object >= 0 && ref >= 0)
      {
         glueAddMenuEntryWithValue(motion->mopt.markermenu, motion->columnname[i], (mnum + 1) * MAXMOTIONCURVES+i);
      }
      else if (name_is_forceplate(model[mod], motion->columnname[i], &motion_object, &ref, &sType, &cutoffFreq, no) >= 0 &&
               motion_object >= 0 && ref >= 0)
      {
         glueAddMenuEntryWithValue(motion->mopt.forceplatemenu, motion->columnname[i], (mnum + 1) * MAXMOTIONCURVES+i);
      }
      else if (name_is_body_segment(model[mod], motion->columnname[i], &motion_object, &ref, &sType, &cutoffFreq, no) >= 0 &&
               motion_object >= 0 && ref >= 0)
      {
         /* If the column is a body segment force, add it to the segment force submenu.
          * Otherwise add it to the motion object submenu.
          */
         if ((STRINGS_ARE_EQUAL(model[mod]->motion_objects[motion_object].name, "force")) ||
             (STRINGS_ARE_EQUAL(model[mod]->motion_objects[motion_object].name, "torque")))
             glueAddMenuEntryWithValue(motion->mopt.segmentforcemenu,
                                       motion->columnname[i], (mnum + 1) * MAXMOTIONCURVES+i);
         else
            glueAddMenuEntryWithValue(motion->mopt.motionobjectmenu,
                                      motion->columnname[i], (mnum + 1) * MAXMOTIONCURVES+i);
      }
      else
      {
         glueAddMenuEntryWithValue(motion->mopt.othermenu, motion->columnname[i], (mnum + 1) * MAXMOTIONCURVES+i);
      }
   }

   motion->mopt.curvemenu = glueCreateMenu("Motion Curves");
   glueAddSubmenuEntry(motion->mopt.curvemenu,"Generalized Coordinates", motion->mopt.gencoordmenu);
   glueAddSubmenuEntry(motion->mopt.curvemenu,"Muscle Activations", motion->mopt.musclemenu);
   glueAddSubmenuEntry(motion->mopt.curvemenu,"Force Plates", motion->mopt.forceplatemenu);
   glueAddSubmenuEntry(motion->mopt.curvemenu,"Body Segment Forces", motion->mopt.segmentforcemenu);
   glueAddSubmenuEntry(motion->mopt.curvemenu,"Markers", motion->mopt.markermenu);
   glueAddSubmenuEntry(motion->mopt.curvemenu,"Motion Objects", motion->mopt.motionobjectmenu);
   glueAddSubmenuEntry(motion->mopt.curvemenu,"Other Variables", motion->mopt.othermenu);

}


ReturnCode setup_motion_derivatives(MotionSequence* motion)
{
   int i;

   motion->deriv_names = (char**)simm_malloc(motion->number_of_datacolumns * sizeof(char*));
   if (motion->deriv_names == NULL)
   {
      error(none, "Cannot calculate derivatives of motion curves.");
      return code_bad;
   }

   motion->deriv_data = (double**)simm_malloc(motion->number_of_datacolumns * sizeof(double*));
   if (motion->deriv_data == NULL)
   {
      error(none, "Cannot calculate derivatives of motion curves.");
      return code_bad;
   }

   for (i = 0; i < motion->number_of_datacolumns; i++)
   {
      sprintf(buffer,"%s_deriv", motion->columnname[i]);
      mstrcpy(&motion->deriv_names[i],buffer);
   }

   for (i = 0; i < motion->number_of_datacolumns; i++)
   {
      motion->deriv_data[i] = (double*)simm_calloc(motion->number_of_datarows, sizeof(double));
      if (motion->deriv_data[i] == NULL)
      {
         error(none, "Cannot calculate derivatives of motion curves.");
         return code_bad;
      }
   }

   calc_motion_derivatives(motion);

   return code_fine;
}

void calc_motion_derivatives(MotionSequence *motion)
{
   int i, j;
   SplineFunction func;

   malloc_function(&func, motion->number_of_datarows);
   func.defined = func.used = yes;
   func.type = natural_cubic;
   func.numpoints = motion->number_of_datarows;

   for (i = 0; i < motion->number_of_datacolumns; i++)
   {
      for (j = 0; j < func.numpoints; j++)
      {
         func.x[j] = j * motion->time_step; //TODO: make this work for variably spaced rows
         func.y[j] = motion->motiondata[i][j];
      }

      calc_spline_coefficients(&func);

      for (j = 0; j < func.numpoints; j++)
         motion->deriv_data[i][j] = interpolate_spline(func.x[j], &func, first, 1.0, 1.0);
   }

   free_function(&func);
}

int get_motion_number(ModelStruct* ms, MotionSequence* motion)
{
   int i;

   if (ms && motion)
   {
      for (i = 0; i < ms->motion_array_size; i++)
      {
         if (ms->motion[i] == motion)
            return i;
      }
   }

   return -1;
}


int get_motion_frame_number(MotionSequence* motion, double value)
{
   double percent;
   int frame;

   percent = (motion->mopt.current_value - motion->min_value) /
      (motion->max_value - motion->min_value);

   frame = percent * (motion->number_of_datarows - 1) + MOTION_OFFSET;

   return frame;
}


void link_derivs_to_model(ModelStruct* ms, MotionSequence* motion)
{

   int i, j, ref, motion_object, motion_num;
   SplineType sType;
   double cutoffFreq;
   char* ptr = NULL;

   motion_num = get_motion_number(ms, motion);

   if (motion_num < 0)
      return;

   /* Go thru the array of genc_velocities to see which ones are still NULL,
    * meaning that they were not supplied in the motion file. If one is
    * NULL, find the location in the columnname array of the name of the
    * gencoord, because that same location in the deriv_data array is the
    * SIMM-calculated gencoord derivative.
    */
   for (i=0; i<ms->numgencoords; i++)
   {
      if (motion->mopt.genc_velocities[i] == NULL)
      {
         for (j=0; j<motion->number_of_datacolumns; j++)
         {
            if (STRINGS_ARE_EQUAL(motion->columnname[j], ms->gencoord[i].name))
               break;
         }
         if (j == motion->number_of_datacolumns)
            continue;
         motion->mopt.genc_velocities[i] = motion->deriv_data[j];
      }
   }

   for (i=0; i<motion->number_of_datacolumns; i++)
   {
      if (name_is_gencoord(motion->columnname[i], ms, NULL, &sType, &cutoffFreq, no) >= 0)
          glueAddMenuEntryWithValue(motion->mopt.gencoordmenu, motion->deriv_names[i],
                                    (motion_num + 1) * MAXMOTIONCURVES + motion->number_of_datacolumns + i);

      else if (name_is_gencoord(motion->columnname[i], ms, "_vel", &sType, &cutoffFreq, no) >= 0)
          glueAddMenuEntryWithValue(motion->mopt.gencoordmenu, motion->deriv_names[i],
                                    (motion_num + 1) * MAXMOTIONCURVES + motion->number_of_datacolumns + i);

      else if (name_is_muscle(ms, motion->columnname[i], NULL, &sType, &cutoffFreq, no) >= 0)
          glueAddMenuEntryWithValue(motion->mopt.musclemenu, motion->deriv_names[i],
                                    (motion_num + 1) * MAXMOTIONCURVES + motion->number_of_datacolumns + i);

      else if (get_ligament_index(ms->modelnum, motion->columnname[i]) >= 0)
          glueAddMenuEntryWithValue(motion->mopt.musclemenu, motion->deriv_names[i],
                                    (motion_num + 1) * MAXMOTIONCURVES + motion->number_of_datacolumns + i);

      else if (name_is_marker(ms, motion->columnname[i], &motion_object, &ref, &sType, &cutoffFreq, no) >= 0 &&
               motion_object >= 0 && ref >= 0)
          glueAddMenuEntryWithValue(motion->mopt.markermenu, motion->deriv_names[i],
                                    (motion_num + 1) * MAXMOTIONCURVES + motion->number_of_datacolumns + i);
      else if (name_is_forceplate(ms, motion->columnname[i], &motion_object, &ref, &sType, &cutoffFreq, no) >= 0 &&
               motion_object >= 0 && ref >= 0)
          glueAddMenuEntryWithValue(motion->mopt.forceplatemenu, motion->deriv_names[i],
                                    (motion_num + 1) * MAXMOTIONCURVES + motion->number_of_datacolumns + i);
      else if (name_is_body_segment(ms, motion->columnname[i], &motion_object, &ref, &sType, &cutoffFreq, no) >= 0 &&
               motion_object >= 0 && ref >= 0)
      {
         /* If the column is a body segment force, add it to the segment force submenu.
          * Otherwise add it to the motion object submenu.
          */
         if ((STRINGS_ARE_EQUAL(ms->motion_objects[motion_object].name, "force")) ||
             (STRINGS_ARE_EQUAL(ms->motion_objects[motion_object].name, "torque")))
            glueAddMenuEntryWithValue(motion->mopt.segmentforcemenu, motion->deriv_names[i],
                                      (motion_num + 1) * MAXMOTIONCURVES + motion->number_of_datacolumns + i);
         else
            glueAddMenuEntryWithValue(motion->mopt.motionobjectmenu, motion->deriv_names[i],
                                      (motion_num + 1) * MAXMOTIONCURVES + motion->number_of_datacolumns + i);
      }
      else
      {
#if 0
         /* TODO: adding the derivs of 'otherdata' to the pop-up menu often creates a menu
          * that is too large to fit on the screen. For now, don't add the derivs. The ideal
          * solution would be to split the menu into more pieces so the derivs can be stored
          * in their own submenu.
          */
         glueAddMenuEntryWithValue(motion->mopt.othermenu, motion->deriv_names[i],
                                   (motion_num + 1) * MAXMOTIONCURVES + motion->number_of_datacolumns + i);
#endif
      }
   }

}



SBoolean tie_motion_to_model(MotionSequence* motion, ModelStruct* ms)
{

   int i, j, name_len, num_others, motion_num;
   char otherNames[OTHER_NAMES_SIZE];
   IntBox bbox;
   Form* form;
   ReturnCode rc, rc2;
   SliderArray* sa;

   motion->mopt.current_value = motion->min_value;
   motion->mopt.current_frame = 0;

   motion->mopt.gencoords = (double**)simm_malloc(ms->numgencoords * sizeof(double*));
   motion->mopt.genc_velocities = (double**)simm_malloc(ms->numgencoords * sizeof(double*));
   motion->mopt.muscles = (double**)simm_malloc(ms->nummuscles * sizeof(double*));
   motion->mopt.ligaments = (double**)simm_malloc(ms->numligaments * sizeof(double*));
   if ((motion->mopt.gencoords == NULL && ms->numgencoords > 0) ||
      (motion->mopt.muscles == NULL && ms->nummuscles > 0) ||
      (motion->mopt.ligaments == NULL && ms->numligaments > 0))
   {
      error(none,"Cannot link motion to model.");
      return no;
   }

   NULLIFY_STRING(otherNames);

   if (column_names_match_model(ms, motion, &num_others, otherNames) == no)
   {
      FREE_IFNOTNULL(motion->mopt.gencoords);
      FREE_IFNOTNULL(motion->mopt.genc_velocities);
      FREE_IFNOTNULL(motion->mopt.muscles);
      FREE_IFNOTNULL(motion->mopt.ligaments);
      FREE_IFNOTNULL(motion->mopt.other_data);

      if (motion->mopt.motion_object_instances)
      {
         for (j = 0; j < motion->mopt.num_motion_object_instances; j++)
            free_motion_object_instance(&motion->mopt.motion_object_instances[j]);
      }
	 
      FREE_IFNOTNULL(motion->mopt.motion_object_instances);

      motion->mopt.num_motion_object_instances = 0;
      return no;
   }

   /* Make sure the motion has a unique name */
   for (i = 0; i < ms->motion_array_size; i++)
   {
      if (ms->motion[i] && ms->motion[i] != motion &&
          STRINGS_ARE_EQUAL(ms->motion[i]->name, motion->name))
         break;
   }     
   if (i < ms->motion_array_size)
   {
      (void)sprintf(buffer,"%s (%d)", motion->name, get_motion_number(ms, motion) + 1);
      FREE_IFNOTNULL(motion->name);
      mstrcpy(&motion->name, buffer);
   }

   motion_num = get_motion_number(ms, motion);

   make_motion_curve_menu(ms->modelnum, motion, motion_num);

#if 0
   fprintf(stderr, "tie_motion_to_model: there are %d mo instances, p = %p\n",
      motion->mopt[i].num_motion_object_instances,
      motion->mopt[i].motion_object_instances);
#endif

   if (ms->num_motions == 1)
      glueRemoveMenuItem(ms->motionplotmenu, 1);
   glueAddSubmenuEntry(ms->motionplotmenu, motion->name, motion->mopt.curvemenu);
   //glueCheckMenuItem(ms->motionplotmenu, 1, GLUE_UNCHECK);
   glueAddMenuEntryWithValue(ms->motionmenu, motion->name, motion_num);
   //glueCheckMenuItem(ms->motionmenu, 1, GLUE_UNCHECK);

   /* Add a field to the gencoord form */
   form = &ms->gencform;
   form->option = (FormItem*)simm_realloc(form->option, (form->numoptions+1)*sizeof(FormItem), &rc);
   if (rc == code_bad)
   {
      error(none,"Cannot link motion to the model.");
      return no;
   }

   mstrcpy(&form->option[form->numoptions].name, motion->name);
   SET_BOX1221(form->option[form->numoptions].box,0,75,
      -FORM_FIELD_YSPACING*form->numoptions,
      form->option[form->numoptions].box.y2-FORM_FIELD_HEIGHT);
   storeDoubleInForm(&form->option[form->numoptions], motion->mopt.current_value, 3);
   form->option[form->numoptions].justify = yes;
   form->option[form->numoptions].active = yes;
   form->option[form->numoptions].visible = yes;
   form->option[form->numoptions].editable = yes;
   form->option[form->numoptions].use_alternate_colors = no;
   form->option[form->numoptions].decimal_places = 3;
   form->option[form->numoptions].data = motion;
   form->numoptions++;
   name_len = glueGetStringWidth(root.gfont.defaultfont, motion->name);
   if (name_len > ms->longest_genc_name)
      ms->longest_genc_name = name_len;

   /* Add a slider to the gencoord slider array */

   sa = &ms->gencslider;
   sa->sl = (Slider*)simm_realloc(sa->sl,(sa->numsliders+1)*sizeof(Slider),&rc);
   if (rc == code_bad)
   {
      error(none,"Cannot link motion to the model.");
      return no;
   }

   SET_BOX1221(bbox, 0, 170 + 2 * FORM_FIELD_HEIGHT, -FORM_FIELD_YSPACING * sa->numsliders,
               bbox.y2 - FORM_FIELD_HEIGHT);
   make_slider(&sa->sl[sa->numsliders], horizontal_slider, bbox, FORM_FIELD_HEIGHT,
               motion->min_value, motion->min_value, motion->max_value,
               (motion->max_value - motion->min_value)/
               (motion->number_of_datarows - 1), NULL, motion);
   sa->numsliders++;

   reposition_gencoord_sliders(ms);

   glueAddMenuEntryWithValue(ms->xvarmenu, motion->name, ms->numgencoords + motion_num);
#if 0
   glueCheckMenuItem(model[i]->xvarmenu,1,GLUE_UNCHECK); /* because of dopup bug */
#endif      
   ms->dis.devs = (int*)simm_realloc(ms->dis.devs, (ms->dis.numdevs + 2) * sizeof(int), &rc);
   ms->dis.dev_values = (int*)simm_realloc(ms->dis.dev_values,	(ms->dis.numdevs + 2) * sizeof(int), &rc2);
   if (rc == code_bad || rc2 == code_bad)
   {
      error(none,"Cannot link motion to the model.");
      return no;
   }

   ms->dis.devs[ms->dis.numdevs++] = motion->keys[0];
   ms->dis.devs[ms->dis.numdevs++] = motion->keys[1];

   (void)sprintf(buffer,"Linked motion %s to model %s (%d \'other\' data columns).",
      motion->name, ms->name, num_others);
   message(buffer,0,DEFAULT_MESSAGE_X_OFFSET);
   if (num_others > 0)
   {
      simm_printf(no, "The following have been loaded as \'otherdata\' in motion %s:\n", motion->name);
      simmPrintMultiLines(otherNames, no, 80, 50);
   }
   make_and_queue_simm_event(MOTION_ADDED, ms->motion[motion_num], ms->modelnum, ZERO);

   return yes;
}


/* -------------------------------------------------------------------------
   circularize_motion_index - this interesting routine was added to allow
      realtime motion to continuously stream into a fixed-duration
      MotionSequence, by wrapping the samples around the end of the buffer.
      
      -- KMS 2/23/00
---------------------------------------------------------------------------- */
static int circularize_motion_index (MotionSequence* motion, int i)
{
   if (motion->is_realtime && motion->realtime_circular_index > 0)
   {
      i += motion->realtime_circular_index;
      
      if (i >= motion->number_of_datarows)
         i -= motion->number_of_datarows;
   }

#if 0
   if (i < 0 || i >= motion->number_of_datarows)
      assert(0);
#endif

   return i;
}


int apply_motion_to_model(ModelStruct* ms, MotionSequence* motion, double value, SBoolean draw_plot)
{
   int i, item, motion_num;
   double percent;

   if (!motion)
      return 0;

   motion->mopt.current_value = value;

   if (motion->mopt.current_value < motion->min_value)
      motion->mopt.current_value  = motion->min_value;
   else if (motion->mopt.current_value > motion->max_value)
      motion->mopt.current_value  = motion->max_value;

   if (EQUAL_WITHIN_ERROR(motion->max_value, motion->min_value) || motion->number_of_datarows == 1)
   {
      motion->mopt.current_value = motion->min_value;
      motion->mopt.current_frame = 0;
   }
   else
   {
      percent = (motion->mopt.current_value - motion->min_value) /
	        (motion->max_value - motion->min_value);

      motion->mopt.current_frame = percent * (motion->number_of_datarows - 1) + MOTION_OFFSET;
#if 0
      printf("percent = %lf, frame = %d, value = %lf\n", percent, motion->mopt.current_frame,
         motion->mopt.current_value);
#endif
      motion->mopt.current_value = motion->min_value + motion->mopt.current_frame *
         (motion->max_value - motion->min_value) / (motion->number_of_datarows - 1);
   }

   motion->mopt.current_frame = circularize_motion_index(motion, motion->mopt.current_frame);

   if ((ms->numclosedloops > 0) && (ms->useIK == yes) && motion->enforce_loops == yes)
   {
      LoopStatus loopStatus;
      ConstraintStatus consStatus;
      /* if the gencoord is in the motion file, set it to the value in the 
       * motion file.  If it's not in the motion file and is in a loop, set it
       * to zero.  Then invalidate all the joint matrices, and solve all the
       * loops.  Gencoords get set to whatever new values are calculated by
       * the IK solver (may not be the same as those in the motion file.
       */
      for (i = 0; i < ms->numgencoords; i++)
      {
         int j;
         if (motion->mopt.gencoords[i] != NULL)
            ms->gencoord[i].value = motion->mopt.gencoords[i][motion->mopt.current_frame];
         else if (ms->gencoord[i].used_in_loop == yes)
            ms->gencoord[i].value = 0.0;
         for (j = 0; j < ms->gencoord[i].numjoints; j++)
            invalidate_joint_matrix(ms, ms->gencoord[i].jointnum[j]);
      }
      solveAllLoopsAndConstraints(ms, &loopStatus, &consStatus, yes);
      if (loopStatus == loopChanged)
      {
#if 0
         sprintf(buffer, "Loops solved using motion file values, values changed\n");
         error(none, buffer);
#endif
      }
   }
   else if (ms->num_constraint_objects > 0)// && motion->enforce_constraints == yes)
   {
      ConstraintStatus constraintStatus;
      LoopStatus loopStatus;
      /* if the gencoord is in the motion file, copy its value into the gc struct.
       * then invalidate all the joints that use it.
       * Solve all the constraints with the new values.  Gencoords are set
       * to whatever values are calculated by the constraint solver (may not
       * be the values in the motion file. Don't change any values if the
       * gencoord is locked.  If the constraints are unchanged, the gencoord
       * values must be set (set_gencoord_value was not called in solveall. 
       */
      for (i = 0; i < ms->numgencoords; i++)
      {
         int j;
         if ((motion->mopt.gencoords[i] != NULL) &&
            (ms->gencoord[i].locked == no))
         {
            ms->gencoord[i].value = motion->mopt.gencoords[i][motion->mopt.current_frame];
            for (j = 0; j < ms->gencoord[i].numjoints; j++)
               invalidate_joint_matrix(ms, ms->gencoord[i].jointnum[j]);
         }
      }

      solveAllLoopsAndConstraints(ms, &loopStatus, &constraintStatus, motion->enforce_constraints);//yes);
      if (constraintStatus == constraintUnchanged)
      {
         for (i = 0; i < ms->numgencoords; i++)
         {
            set_gencoord_value(ms->modelnum, i, ms->gencoord[i].value, no);
         }
      }
      else if (constraintStatus == constraintChanged)
      {
#if 0
         sprintf(buffer, "Constraints solved using motion file values, values changed\n");
         error(none, buffer);
#endif
      }
      //dkb aug 19, 2002
      for (i = 0; i < ms->numgencoords; i++)
      {
         set_gencoord_value(ms->modelnum, i, ms->gencoord[i].value, no);
      }
   }
   else
   {
      /* if solver is off or there are no loops, and there are no constraints,
       * set the gencoords to the values in the motion file */
      for (i = 0; i < ms->numgencoords; i++)
      {
         if (motion->mopt.gencoords[i] != NULL)
         {
            if (ms->gencoord[i].locked == no)
            {
               set_gencoord_value(ms->modelnum, i, 
                  motion->mopt.gencoords[i][motion->mopt.current_frame], no);
            }
            else
            {
               /* If the gencoord is locked, but still appears in the motion file,
                * print a warning message if the motion file value is not within
                * a tolerance equal to the default precision of motion file data.
                */

               if (NOT_EQUAL_WITHIN_TOLERANCE(
                  motion->mopt.gencoords[i][motion->mopt.current_frame],
                  ms->gencoord[i].value, 0.000001))
               {
                  sprintf(errorbuffer, "Gencoord %s is locked at %f\n",
                  ms->gencoord[i].name, ms->gencoord[i].value);
                  error(none, errorbuffer);
               }
            }
         }
      }
   }

   /* Find out which form item is linked to this motion and update it. */
   for (item = 0; item < ms->gencform.numoptions; item++)
   {
      if (ms->gencform.option[item].data == motion)
      {
         storeDoubleInForm(&ms->gencform.option[item], motion->mopt.current_value, 3);
         break;
      }
   }

   /* Find out which slider is linked to this motion and update it. */
   for (item = 0; item < ms->gencform.numoptions; item++)
   {
      if (ms->gencform.option[item].data == motion)
      {
         ms->gencslider.sl[item].value = motion->mopt.current_value;
         break;
      }
   }

   for (i=0; i<ms->numgencoords; i++)
   {
      if (motion->mopt.genc_velocities[i] != NULL)
         set_gencoord_velocity(ms->modelnum,i,motion->mopt.genc_velocities[i][motion->mopt.current_frame]);
      else
         set_gencoord_velocity(ms->modelnum,i,0.0);
   }

   for (i=0; i<ms->nummuscles; i++)
   {
      if (motion->mopt.muscles[i] != NULL)
         ms->muscle[i].activation = motion->mopt.muscles[i][motion->mopt.current_frame];
   }  
   
   for (i=0; i<ms->numligaments; i++)
   {
      if (motion->mopt.ligaments[i] != NULL)
         ms->ligament[i].activation = motion->mopt.ligaments[i][motion->mopt.current_frame];
   }

   ms->dis.applied_motion = motion;

   if (draw_plot == yes && motion->show_cursor == yes)
      set_plot_cursors(ms, motion);

   /* If there is no gencoord data in this motion (all columns of
    * data in motion file are "othercurves," then return 0, meaning you didn't
    * change the model configuration by trying to apply the motion to it. Otherwise
    * return 1, so the calling routine knows that it should redraw the model.
    * JPL/TODO 7/24/96: This doesn't work since standard deviations have been
    * added to motion files, because num_datacolumns  gets decremented by num_std_dev,
    * but num_other does not (so they may happen to be equal)

   if (motion->number_of_datacolumns == motion->number_of_othercurves)
      return 0;
   else
    */

      return 1;
}


static SBoolean column_names_match_model(ModelStruct* ms, MotionSequence* motion,
                                         int* num_other_data, char otherNames[])
{

   int i, ref, seg, genc, musc_index, lig_index, cur_len = 1;
   SplineType sType;
   double cutoffFreq;
   SBoolean full = no;
   char* ptr = NULL;

   *num_other_data = 0;

   for (i = 0; i < ms->numgencoords; i++)
   {
      motion->mopt.gencoords[i] = NULL;
      motion->mopt.genc_velocities[i] = NULL;
   }

   for (i = 0; i < ms->nummuscles; i++)
      motion->mopt.muscles[i] = NULL;

   for (i = 0; i < ms->numligaments; i++)
      motion->mopt.ligaments[i] = NULL;

   /* Since you don't yet know how many columns are other data, make room
    * for the maximum number, and realloc later.
    */
   motion->mopt.other_data =
      (double**)simm_malloc(motion->number_of_datacolumns*sizeof(double*));
   if (motion->mopt.other_data == NULL)
   {
      error(none,"Cannot link motion to the model.");
      return no;
   }
   
   for (i = 0; i < motion->number_of_datacolumns; i++)
      motion->mopt.other_data[i] = NULL;

   for (i = 0; i < motion->number_of_datacolumns; i++)
   {
      int motion_object;
      double cutoffFreq;
      SplineType sType;
      
      if ((genc = name_is_gencoord(motion->columnname[i], ms, NULL, &sType, &cutoffFreq, yes)) >= 0)
         motion->mopt.gencoords[genc] = motion->motiondata[i];
      else if ((genc = name_is_gencoord(motion->columnname[i], ms, "_vel", &sType, &cutoffFreq, yes)) >= 0)
         motion->mopt.genc_velocities[genc] = motion->motiondata[i];
      else if ((musc_index = name_is_muscle(ms, motion->columnname[i], NULL, &sType, &cutoffFreq, yes)) >= 0)
         motion->mopt.muscles[musc_index] = motion->motiondata[i];
      else if ((lig_index = get_ligament_index(ms->modelnum, motion->columnname[i])) >= 0)
         motion->mopt.ligaments[lig_index] = motion->motiondata[i];
      else if ((seg = name_is_marker(ms, motion->columnname[i], &motion_object, &ref, &sType, &cutoffFreq, yes)) >= 0 &&
               motion_object >= 0 && ref >= 0)
         store_motion_object_instance(ms->modelnum, &motion->mopt, seg, motion_object, ref, i);
      else if ((seg = name_is_forceplate(ms, motion->columnname[i], &motion_object, &ref, &sType, &cutoffFreq, yes)) >= 0 &&
               motion_object >= 0 && ref >= 0)
         store_motion_object_instance(ms->modelnum, &motion->mopt, seg, motion_object, ref, i);
      else if ((seg = name_is_body_segment(ms, motion->columnname[i], &motion_object, &ref, &sType, &cutoffFreq, yes)) >= 0 &&
               motion_object >= 0 && ref >= 0)
         store_motion_object_instance(ms->modelnum, &motion->mopt, seg, motion_object, ref, i);
      else
      {
         motion->mopt.other_data[(*num_other_data)++] = motion->motiondata[i];
         /* Build up a character string list of 'otherdata' names so you can
          * inform the user which ones you found.
          */
         addNameToString(motion->columnname[i], otherNames, OTHER_NAMES_SIZE);
      }
   }

   return yes;
}


/* -------------------------------------------------------------------------
   name_is_marker - parse the specified 'name' to
      discover its encoded marker name and animation component. If there is
      a match, use the "ball" motion object and return "ground" as the segment
      because marker data in a motion is always w.r.t. ground.
---------------------------------------------------------------------------- */
int name_is_marker(ModelStruct* ms, char name[], int* motion_object, int* component,
                   SplineType* splineType, double* cutoffFrequency, SBoolean stripEnd)
{
   int i, j, seg, maxLen, Mlen = 0, len = strlen(name);
   SBoolean foundOne;
   char *ptr, *newEnd;

   /* First check to see if the string begins with the name of a marker.
    * To handle models which have overlapping marker names, like
    * "L.Knee" and "L.Knee.Med", check for a match with all markers, and take
    * the longest match.
    */
   for (i = 0, foundOne = no, maxLen = -1; i < ms->numsegments; i++)
   {
      for (j = 0; j < ms->segment[i].numMarkers; j++)
      {
         Mlen = strlen(ms->segment[i].marker[j].name);
         if (len >= Mlen)
         {
            if (strings_equal_n_case_insensitive(name, ms->segment[i].marker[j].name, Mlen))
            {
               if (Mlen > maxLen)
               {
                  foundOne = yes;
                  maxLen = Mlen;
               }
            }
         }
      }
   }

   if (foundOne == no)
      return -1;

   /* Move ptr past the marker name. */
   ptr = &name[maxLen];
   len -= maxLen;

   /* The name is of a marker, so assume a segment of "ground" and then
    * look for the component name.
    */
   seg = ms->ground_segment;

   /* The motion object is the pre-defined "ball" object. */
   if (motion_object)
   {
      *motion_object = -1;

      for (i = 0; i < ms->num_motion_objects; i++)
      {
         if (STRINGS_ARE_EQUAL(ms->motion_objects[i].name, "ball"))
         {
            *motion_object = i;
            break;
         }
      }
      if (i == ms->num_motion_objects)
         return -1;
   }

   /* Now look for the component. */
   if (component)
   {
      *component = -1;

      /* determine the motion component from the next part of the name. */
      if (!strncmp(ptr, "_tx", 3) || !strncmp(ptr, "_px", 3))
         *component = MO_TX;
      else if (!strncmp(ptr, "_ty", 3) || !strncmp(ptr, "_py", 3))
         *component = MO_TY;
      else if (!strncmp(ptr, "_tz", 3) || !strncmp(ptr, "_pz", 3))
         *component = MO_TZ;
      else if (!strncmp(ptr, "_vx", 3))
         *component = MO_VX;
      else if (!strncmp(ptr, "_vy", 3))
         *component = MO_VY;
      else if (!strncmp(ptr, "_vz", 3))
         *component = MO_VZ;
      else if (!strncmp(ptr, "_sx", 3))
         *component = MO_SX;
      else if (!strncmp(ptr, "_sy", 3))
         *component = MO_SY;
      else if (!strncmp(ptr, "_sz", 3))
         *component = MO_SZ;
      else if (!strncmp(ptr, "_cr", 3))
         *component = MO_CR;
      else if (!strncmp(ptr, "_cg", 3))
         *component = MO_CG;
      else if (!strncmp(ptr, "_cb", 3))
         *component = MO_CB;
      else
         return -1;
   }
   else
   {
      return -1;
   }

   /* Move ptr past the component name. */
   ptr += 3;
   len -= 3;

   /* Store a pointer to the character right after the component.
    * This will be the new end of the string if stripEnd == yes.
    */
   newEnd = ptr;

   /* If splineType and cutoffFrequency are not NULL, check to see if the name ends in
    * natural_cubic_text or gcv_text (followed by an optional cutoff frequency). If it
    * does, set *splineType to the appropriate type. If no spline label is found, set
    * the type to step_func.
    */
   if (splineType && cutoffFrequency)
   {
      int matched_spl = 0;
      int spl_len = strlen(natural_cubic_text);
      int gcv_len = strlen(gcv_text);

      *splineType = step_function;
      *cutoffFrequency = -1.0; // by default there is no smoothing

      if (len >= spl_len)
      {
         if (!strncmp(ptr, natural_cubic_text, spl_len))
         {
            *splineType = natural_cubic;
            ptr += spl_len;
            len -= spl_len;
            matched_spl = 1;
         }
      }

      if (!matched_spl && len >= gcv_len)
      {
         if (!strncmp(ptr, gcv_text, gcv_len))
         {
            ptr += gcv_len;
            len -= gcv_len;
            *splineType = gcv_spline;
            if (len > 0)
            {
               char* intPtr = buffer;

               /* Move over the underscore and look for an integer. */
               if (*(ptr++) != '_')
               {
                  return -1;
               }
               else
               {
                  len--; /* for the underscore character */
                  while (*ptr >= '0' && *ptr <= '9')
                  {
                     *(intPtr++) = *(ptr++);
                     len--;
                  }
                  *intPtr = STRING_TERMINATOR;
                  *cutoffFrequency = atof(buffer);
               }
            }
         }
      }
   }

   /* If there are extra characters after the suffixes, return an error. */
   if (len > 0)
      return -1;

   /* Strip off the text for the spline type and cutoff frequency. */
   if (stripEnd == yes)
      *newEnd = STRING_TERMINATOR;

   return seg;

}


/* -------------------------------------------------------------------------
   name_is_forceplate - parse the specified 'name' to
      discover its encoded forceplate name and animation component. If there is
      a match, use the "force" motion object and return "ground" as the segment
      because forceplate data in a motion is always w.r.t. ground.
---------------------------------------------------------------------------- */
int name_is_forceplate(ModelStruct* ms, char name[], int* motion_object, int* component,
                       SplineType* splineType, double* cutoffFrequency, SBoolean stripEnd)
{
   int i, j, seg, plateNum, Flen = 0, len = strlen(name);
   char *ptr, *newEnd;

   /* If the column name starts with "forceplate" and a number, then
    * it is forceplate data.
    */
   if (sscanf(name, "forceplate%d", &plateNum) != 1)
      return -1;

   /* The name is of a forceplate, so assume a segment of "ground" and then
    * look for the component name.
    */
   seg = ms->ground_segment;

   sprintf(buffer, "forceplate%d", plateNum);
   Flen = strlen(buffer);

   /* Move ptr past the forceplate name and number. */
   ptr = &name[Flen];
   len -= Flen;

   /* The motion object is the pre-defined "force" object. */
   if (motion_object)
   {
      *motion_object = -1;

      for (i = 0; i < ms->num_motion_objects; i++)
      {
         if (STRINGS_ARE_EQUAL(ms->motion_objects[i].name, "force"))
         {
            *motion_object = i;
            break;
         }
      }
      if (i == ms->num_motion_objects)
         return -1;
   }

   /* Now look for the component. */
   if (component)
   {
      *component = -1;

      /* determine the motion component from the next part of the name. */
      if (!strncmp(ptr, "_tx", 3) || !strncmp(ptr, "_px", 3))
         *component = MO_TX;
      else if (!strncmp(ptr, "_ty", 3) || !strncmp(ptr, "_py", 3))
         *component = MO_TY;
      else if (!strncmp(ptr, "_tz", 3) || !strncmp(ptr, "_pz", 3))
         *component = MO_TZ;
      else if (!strncmp(ptr, "_vx", 3))
         *component = MO_VX;
      else if (!strncmp(ptr, "_vy", 3))
         *component = MO_VY;
      else if (!strncmp(ptr, "_vz", 3))
         *component = MO_VZ;
      else if (!strncmp(ptr, "_sx", 3))
         *component = MO_SX;
      else if (!strncmp(ptr, "_sy", 3))
         *component = MO_SY;
      else if (!strncmp(ptr, "_sz", 3))
         *component = MO_SZ;
      else if (!strncmp(ptr, "_cr", 3))
         *component = MO_CR;
      else if (!strncmp(ptr, "_cg", 3))
         *component = MO_CG;
      else if (!strncmp(ptr, "_cb", 3))
         *component = MO_CB;
      else
         return -1;
   }
   else
   {
      return -1;
   }

   /* Move ptr past the component name. */
   ptr += 3;
   len -= 3;

   /* Store a pointer to the character right after the component.
    * This will be the new end of the string if stripEnd == yes.
    */
   newEnd = ptr;

   /* If splineType and cutoffFrequency are not NULL, check to see if the name ends in
    * natural_cubic_text or gcv_text (followed by an optional cutoff frequency). If it
    * does, set *splineType to the appropriate type. If no spline label is found, set
    * the type to step_func.
    */
   if (splineType && cutoffFrequency)
   {
      int matched_spl = 0;
      int spl_len = strlen(natural_cubic_text);
      int gcv_len = strlen(gcv_text);

      *splineType = step_function;
      *cutoffFrequency = -1.0; // by default there is no smoothing

      if (len >= spl_len)
      {
         if (!strncmp(ptr, natural_cubic_text, spl_len))
         {
            *splineType = natural_cubic;
            ptr += spl_len;
            len -= spl_len;
            matched_spl = 1;
         }
      }

      if (!matched_spl && len >= gcv_len)
      {
         if (!strncmp(ptr, gcv_text, gcv_len))
         {
            ptr += gcv_len;
            len -= gcv_len;
            *splineType = gcv_spline;
            if (len > 0)
            {
               char* intPtr = buffer;

               /* Move over the underscore and look for an integer. */
               if (*(ptr++) != '_')
               {
                  return -1;
               }
               else
               {
                  len--; /* for the underscore character */
                  while (*ptr >= '0' && *ptr <= '9')
                  {
                     *(intPtr++) = *(ptr++);
                     len--;
                  }
                  *intPtr = STRING_TERMINATOR;
                  *cutoffFrequency = atof(buffer);
               }
            }
         }
      }
   }

   /* If there are extra characters after the suffixes, return an error. */
   if (len > 0)
      return -1;

   /* Strip off the text for the spline type and cutoff frequency. */
   if (stripEnd == yes)
      *newEnd = STRING_TERMINATOR;

   return seg;

}


/* -------------------------------------------------------------------------
   motion_object_instance_contains_component - 
---------------------------------------------------------------------------- */
static SBoolean motion_object_instance_contains_component (MotionObjectInstance* mi, int component)
{
   if (mi->channels)
   {
      int i;
      
      for (i = 0; i < mi->num_channels; i++)
         if (mi->channels[i].component == component)
            return yes;
   }
   return no;
}

/* -------------------------------------------------------------------------
   store_motion_object_instance - 
---------------------------------------------------------------------------- */
static void store_motion_object_instance (
   int mod,
   MotionModelOptions* mopt,
   int seg,
   int motion_object,
   int motion_component,
   int column)
{
   ReturnCode rc;
   int i;
   MotionObjectInstance* mi = NULL;
   
   /* scan the motion's object instance array to see if we need to create
    * a new motion object instance, or use an existing one:
    */
   for (i = mopt->num_motion_object_instances - 1; i >= 0; i--)
      if (mopt->motion_object_instances[i].segment == seg &&
          mopt->motion_object_instances[i].object == motion_object)
         break;
   
   if (i < 0 ||
       motion_object_instance_contains_component(&mopt->motion_object_instances[i],
                                                 motion_component))
   {
      /* add a new element to the array of motion object instances:
       */
      i = mopt->num_motion_object_instances++;
      
      if (mopt->motion_object_instances == NULL)
      {
         mopt->motion_object_instances = (MotionObjectInstance*)
            simm_malloc(mopt->num_motion_object_instances * sizeof(MotionObjectInstance));
      }
      else
      {
         mopt->motion_object_instances = (MotionObjectInstance*)
            simm_realloc(mopt->motion_object_instances,
                         mopt->num_motion_object_instances * sizeof(MotionObjectInstance),
                         &rc);
      }
   
      /* initialize a new motion object instance:
       */
      if (mopt->motion_object_instances)
      {
         MotionObject* mo = &model[mod]->motion_objects[motion_object];
         
         mi = &mopt->motion_object_instances[i];
         
         mi->object        = motion_object;
         mi->segment       = seg;
         mi->num_channels  = 0;
         mi->channels      = NULL;
         mi->drawmode      = mo->drawmode;
         mi->current_value = -1.0;
         mi->visible       = yes;

         mi->currentMaterial.name = NULL;
         copy_material(&model[mod]->dis.mat.materials[mo->material], &mi->currentMaterial);
      }
      else
         mopt->num_motion_object_instances--;
   }
   else
      mi = &mopt->motion_object_instances[i];
   
   /* add a new animation channel to the motion object instance:
    */
   if (mi)
   {
      i = mi->num_channels++;
      
      if (mi->channels == NULL)
      {
         mi->channels = (MotionObjectChannel*)
            simm_malloc(mi->num_channels * sizeof(MotionObjectChannel));
      }
      else
      {
         mi->channels = (MotionObjectChannel*)
            simm_realloc(mi->channels,
                         mi->num_channels * sizeof(MotionObjectChannel), &rc);
      }
      
      if (mi->channels)
      {
         mi->channels[i].component = motion_component;
         mi->channels[i].column = column;
      }
   }
} /* store_motion_object_instance */


double check_motion_wrapping(ModelStruct* ms, MotionSequence* motion, double change)
{
   double new_value, range;

   if (ms == NULL || motion == NULL)
      return 0.0;

   new_value = motion->mopt.current_value + change;

   if (motion->wrap == no)
      return new_value;

   range = motion->max_value - motion->min_value;

   while (new_value > motion->max_value)
      new_value -= range;

   while (new_value < motion->min_value)
      new_value += range;

   return new_value;
}


static void set_plot_cursors(ModelStruct* ms, MotionSequence* motion)
{

   int i, j, plot_windex;
   PlotStruct* ps;

   if (ms && motion)
   {
      for (i = 0; i < root.numplots; i++)
      {
         ps = find_nth_plot(i+1);
         if (ps == NULL)
            return;
         for (j = 0; j < ps->numcurves; j++)
         {
            if (ps->curve[j]->modelPtr == ms && ps->curve[j]->xvar.motion == motion)
            {
               ps->cursor_x = motion->mopt.current_value;
               ps->cursor_color[0] = motion->cursor_color[0];
               ps->cursor_color[1] = motion->cursor_color[1];
               ps->cursor_color[2] = motion->cursor_color[2];
               ps->cursor_modelPtr = ms;
               ps->cursor_motion = motion;
               plot_windex = get_window_index(PLOT,i);
               if (plot_windex != -1)
                  draw_plot(root.window[plot_windex].win_parameters, root.window[plot_windex].win_struct);
               break;
            }
         }
      }
   }
}


void delete_motion(ModelStruct* ms, MotionSequence* motion)
{
   int i, j, motion_num, num, item, name_len;
   ReturnCode rc, rc2;

   /* When a motion is deleted, it may not have been fully integrated into
    * the model yet (e.g., if an error occurs while filling in the frames
    * of data). So this function cannot assume that the motion already has
    * a slider bar or an entry in a pop-up menu.
    */
   if (ms && motion)
   {
      post_motion_event(ms, motion, get_motion_number(ms, motion), MOTION_DELETED);

      sprintf(buffer, "Deleted motion %s from model %s.", motion->name, ms->name);

      /* reset the current and applied motions */
      if (ms->dis.current_motion == motion)
         ms->dis.current_motion = NULL;
      if (ms->dis.applied_motion == motion)
         ms->dis.applied_motion = NULL;

      /* remove motion from plotmaker x variable menu */
      item = glueFindMenuItemWithLabel(ms->xvarmenu, motion->name);
      if (item >= 0)
         glueRemoveMenuItem(ms->xvarmenu, item);

      /* Remove the motion from the model's motion menu. */
      item = glueFindMenuItemWithLabel(ms->motionmenu, motion->name);
      if (item >= 0)
         glueRemoveMenuItem(ms->motionmenu, item);

      /* Remove the motion from the plot maker's motion curve menu,
       * and destroy all the submenus.
       */
      item = glueFindMenuItemWithLabel(ms->motionplotmenu, motion->name);
      if (item >= 0)
         glueRemoveMenuItem(ms->motionplotmenu, item);
      if (ms->num_motions == 1)
      {
         glueAddMenuEntry(ms->motionplotmenu, "none loaded");
         glueEnableMenuItem(ms->motionplotmenu, 1, GLUE_DISABLE);
      }

      if (motion->mopt.gencoordmenu > 0)
         glutDestroyMenu(motion->mopt.gencoordmenu);
      if (motion->mopt.musclemenu > 0)
         glutDestroyMenu(motion->mopt.musclemenu);
      if (motion->mopt.forceplatemenu > 0)
         glutDestroyMenu(motion->mopt.forceplatemenu);
      if (motion->mopt.segmentforcemenu > 0)
         glutDestroyMenu(motion->mopt.segmentforcemenu);
      if (motion->mopt.markermenu > 0)
         glutDestroyMenu(motion->mopt.markermenu);
      if (motion->mopt.motionobjectmenu > 0)
         glutDestroyMenu(motion->mopt.motionobjectmenu);
      if (motion->mopt.othermenu > 0)
         glutDestroyMenu(motion->mopt.othermenu);
      if (motion->mopt.curvemenu > 0)
         glutDestroyMenu(motion->mopt.curvemenu);

      /* find the position of the motion in the slider bars and gencoord form */
      for (i = ms->numgencoords, num = -1; i < ms->gencform.numoptions; i++)
      {
         if (STRINGS_ARE_EQUAL(ms->gencform.option[i].name, motion->name))
         {
            num = i;
            break;
         }
      }

      if (num >= 0 && num < ms->gencform.numoptions)
      {
         /* remove the motion from the gencoord form, move each item forward */
         FREE_IFNOTNULL(ms->gencform.option[num].name);
         for (i = num; i < ms->gencform.numoptions - 1; i++)
            memcpy(&ms->gencform.option[i], &ms->gencform.option[i + 1], sizeof(FormItem));
         ms->gencform.option = (FormItem*)simm_realloc(ms->gencform.option, 
                               (ms->gencform.numoptions - 1) * sizeof(FormItem), &rc);
         ms->gencform.numoptions--;

         /* reset the longest name */
         ms->longest_genc_name = 0;
         for (i = 0; i < ms->gencform.numoptions; i++)
         {
            name_len = glueGetStringWidth(root.gfont.defaultfont, ms->gencform.option[i].name);
            if (name_len > ms->longest_genc_name)
               ms->longest_genc_name = name_len;
         }

         /* Remove a slider from the gencoord slider array and fill in the hole. */
         for (i = num; i < ms->gencslider.numsliders - 1; i++)
            memcpy(&ms->gencslider.sl[i], &ms->gencslider.sl[i+1], sizeof(Slider));
         ms->gencslider.sl = (Slider*)simm_realloc(ms->gencslider.sl,
                             (ms->gencslider.numsliders - 1) * sizeof(Slider), &rc);
         ms->gencslider.numsliders--;
      
         reposition_gencoord_sliders(ms);

         if (num * 2 < ms->dis.numdevs)
         {
            for (i = num * 2; i < ms->dis.numdevs - 2; i++)
            {
               ms->dis.devs[i] = ms->dis.devs[i + 2];
               ms->dis.dev_values[i] = ms->dis.dev_values[i + 2];
            }
            ms->dis.devs = (int*)simm_realloc(ms->dis.devs, (ms->dis.numdevs - 2) * sizeof(int), &rc);
            ms->dis.dev_values = (int*)simm_realloc(ms->dis.dev_values,	(ms->dis.numdevs - 2) * sizeof(int), &rc2);
            ms->dis.numdevs -= 2;
         }
      }

      for (i = 0; i < ms->motion_array_size; i++)
      {
         if (ms->motion[i] == motion)
         {
            ms->motion[i] = NULL;
            ms->num_motions--;
            break;
         }
      }
   }

   if (motion)
   {
      /* Remove the motion pointer from all plots. */
      for (i = 0; i < root.numplots; i++)
      {
         PlotStruct* ps = find_nth_plot(i + 1);
         if (ps)
         {
            if (ps->cursor_motion == motion)
               ps->cursor_motion = NULL;

            for (j = 0; j < ps->numcurves; j++)
            {
               if (ps->curve[j]->xvar.motion == motion)
                  ps->curve[j]->xvar.motion = NULL;
            }
         }
      }

      free_motion(motion);

#if 0
      fprintf(stderr, "delete_motion: there are %d mo instances, p = %p\n",
         motion->mopt.num_motion_object_instances,
         motion->mopt.motion_object_instances);
#endif

      FREE_IFNOTNULL(motion);
      message(buffer, 0, DEFAULT_MESSAGE_X_OFFSET);
   }
}

void free_motion(MotionSequence* motion)
{
   if (motion)
   {
      int i;

      FREE_IFNOTNULL(motion->name);
      FREE_IFNOTNULL(motion->units);

      if (motion->motiondata)
      {
         for (i = 0; i < motion->number_of_datacolumns; i++)
            FREE_IFNOTNULL(motion->motiondata[i]);
         FREE_IFNOTNULL(motion->motiondata);
      }

      if (motion->columnname)
      {
         for (i = 0; i < motion->number_of_datacolumns; i++)
            FREE_IFNOTNULL(motion->columnname[i]);
         FREE_IFNOTNULL(motion->columnname);
      }

      if (motion->deriv_data)
      {
         for (i = 0; i < motion->number_of_datacolumns; i++)
            FREE_IFNOTNULL(motion->deriv_data[i]);
         FREE_IFNOTNULL(motion->deriv_data);
      }

      if (motion->deriv_names)
      {
         for (i = 0; i < motion->number_of_datacolumns; i++)
            FREE_IFNOTNULL(motion->deriv_names[i]);
         FREE_IFNOTNULL(motion->deriv_names);
      }

      if (motion->data_std_dev)
      {
         for (i = 0; i < motion->number_of_datacolumns; i++)
            FREE_IFNOTNULL(motion->data_std_dev[i]);
         FREE_IFNOTNULL(motion->data_std_dev);
      }

      if (motion->event)
      {
         for (i = 0; i < motion->num_events; i++)
            FREE_IFNOTNULL(motion->event[i].name);
         FREE_IFNOTNULL(motion->event);
      }

      if (motion->mopt.motion_object_instances)
      {
         for (i = 0; i < motion->mopt.num_motion_object_instances; i++)
            free_motion_object_instance(&motion->mopt.motion_object_instances[i]);
         FREE_IFNOTNULL(motion->mopt.motion_object_instances);
      }
   }
}

static void calc_transform_mat(double vx, double vy, double vz, double mat[][4])
{

   double angle, vec1[3], vec2[3], vec3[3], norm_vec2[3], norm_vec3[3];

   vec1[XX] = 0.0;
   vec1[YY] = 1.0;
   vec1[ZZ] = 0.0;

   vec2[XX] = vx;
   vec2[YY] = vy;
   vec2[ZZ] = vz;

   normalize_vector(vec2,norm_vec2);

   cross_vectors(vec1,norm_vec2,vec3);
   normalize_vector(vec3,norm_vec3);

   angle = acos(DOT_VECTORS(vec1,norm_vec2)) * RTOD;
/*
   printf("angle= %lf  vec2= %lf %lf %lf  vec3= %lf %lf %lf\n", angle,
	  norm_vec2[0], norm_vec2[1], norm_vec2[2], norm_vec3[0], norm_vec3[1], norm_vec3[2]);
*/
   make_4x4dircos_matrix(angle,norm_vec3,mat);

}

/* =========================================================================
 * ==== MOTION OBJECT SUPPORT:
 */

/* -------------------------------------------------------------------------
   add_default_motion_objects - 
---------------------------------------------------------------------------- */
void add_default_motion_objects (ModelStruct* ms)
{
   int i, n = sizeof(sDefaultMotionObjects) / sizeof(MotionObject);
   
   for (i = 0; i < n; i++)
   {
      MotionObject* mo;
      ReturnCode rc;
      
      int j = ms->num_motion_objects++;
      
      if (ms->motion_objects == NULL)
         ms->motion_objects = (MotionObject*) simm_malloc(ms->num_motion_objects * sizeof(MotionObject));
      else
         ms->motion_objects = (MotionObject*) simm_realloc(ms->motion_objects,
                                                           ms->num_motion_objects * sizeof(MotionObject),
                                                           &rc);
      if (ms->motion_objects == NULL)
      {
         ms->num_motion_objects = 0;
         return;
      }
      mo = &ms->motion_objects[j];
      
      memset(mo, 0, sizeof(MotionObject));
      
      mstrcpy(&mo->name, sDefaultMotionObjects[i].name);
      mstrcpy(&mo->filename, sDefaultMotionObjects[i].filename);
      rc = lookup_polyhedron(&mo->shape, mo->filename, ms);
      mstrcpy(&mo->materialname, sDefaultMotionObjects[i].materialname);
      mo->material = enter_material(ms, mo->materialname, declaring_element);
      mo->drawmode = sDefaultMotionObjects[i].drawmode;
      mo->vector_axis = sDefaultMotionObjects[i].vector_axis;
      
      for (j = 0; j < 3; j++)
      {
         mo->startingPosition[j]    = sDefaultMotionObjects[i].startingPosition[j];
         mo->startingScale[j]       = sDefaultMotionObjects[i].startingScale[j];
         mo->startingXYZRotation[j] = sDefaultMotionObjects[i].startingXYZRotation[j];
      }
   }
}

#endif /* ENGINE */

/* -------------------------------------------------------------------------
   read_motion_object - 
---------------------------------------------------------------------------- */
public ReturnCode read_motion_object (int mod, FILE** fp)
{
   ModelStruct* ms = model[mod];
   MotionObject* mo;
   ReturnCode rc = code_fine;
   
   int i;
   
   /* read the motion object's name:
    */
   if (fscanf(*fp,"%s", buffer) != 1)
   {
      error(abort_action, "Error reading name in motion object definition");
      return code_bad;
   }
   
   /* scan the list of existing motion objects to see if one already exists
    * with the name we just read:
    */
   for (i = 0; i < ms->num_motion_objects; i++)
      if (STRINGS_ARE_EQUAL(buffer, ms->motion_objects[i].name))
         break;
   
   /* create a new motion object if necessary:
    */
   if (i == ms->num_motion_objects)
   {
      i = ms->num_motion_objects++;
   
      if (ms->motion_objects == NULL)
         ms->motion_objects = (MotionObject*) simm_malloc(ms->num_motion_objects * sizeof(MotionObject));
      else
         ms->motion_objects = (MotionObject*) simm_realloc(ms->motion_objects,
                                                           ms->num_motion_objects * sizeof(MotionObject),
                                                           &rc);
      if (ms->motion_objects == NULL)
      {
         ms->num_motion_objects = 0;
         return code_bad;
      }
      mo = &ms->motion_objects[i];
      
      memset(mo, 0, sizeof(MotionObject));
   
      mstrcpy(&mo->name, buffer);
      
      mstrcpy(&mo->materialname, "def_motion");
      mo->material = enter_material(ms, mo->materialname, declaring_element);

      for (i = XX; i <= ZZ; i++)
      {
         mo->startingPosition[i] = 0.0;
         mo->startingScale[i] = 1.0;
         mo->startingXYZRotation[i] = 0.0;
      }
      mo->drawmode = gouraud_shading;
      mo->vector_axis = YY;
   }
   else
      mo = &ms->motion_objects[i];
      
   mo->defined_in_file = yes;
   
   /* read the motion object's parameters:
    */
   while (1)
   {
      if (read_string(fp, buffer) == EOF || STRINGS_ARE_EQUAL(buffer, "endmotionobject"))
         break;

      if (buffer[0] == '#')
      {
         read_nonempty_line(fp,buffer);
         continue;
      }

      if (STRINGS_ARE_EQUAL(buffer, "filename"))
      {
         if (fscanf(*fp, "%s", buffer) != 1)
         {
            simm_printf(yes, "Error reading filename for motion object: %s.\n", mo->name);
            return code_bad;
         }
         mstrcpy(&mo->filename, buffer);
#ifndef ENGINE
         if (lookup_polyhedron(&mo->shape, mo->filename, ms) != code_fine)
         {
            simm_printf(yes, "Error reading motion object: %s\n", mo->filename);
            return code_bad;
         }
#endif
      }
      if (STRINGS_ARE_EQUAL(buffer, "position") || STRINGS_ARE_EQUAL(buffer,"origin"))
      {
         if (fscanf(*fp, "%lg %lg %lg", &mo->startingPosition[0],
                                        &mo->startingPosition[1],
                                        &mo->startingPosition[2]) != 3)
         {
            simm_printf(yes, "Error reading position for motion object: %s.\n", mo->name);
            return code_bad;
         }
      }
      if (STRINGS_ARE_EQUAL(buffer, "scale"))
      {
         if (fscanf(*fp, "%lg %lg %lg", &mo->startingScale[0],
                                        &mo->startingScale[1],
                                        &mo->startingScale[2]) != 3)
         {
            simm_printf(yes, "Error reading scale for motion object: %s.\n", mo->name);
            return code_bad;
         }
      }
      if (STRINGS_ARE_EQUAL(buffer, "xyzrotation"))
      {
         if (fscanf(*fp, "%lg %lg %lg", &mo->startingXYZRotation[0],
                                        &mo->startingXYZRotation[1],
                                        &mo->startingXYZRotation[2]) != 3)
         {
            simm_printf(yes, "Error reading xyzrotation for motion object: %s.\n", mo->name);
            return code_bad;
         }
      }
      if (STRINGS_ARE_EQUAL(buffer, "material"))
      {
         if (fscanf(*fp, "%s", buffer) != 1)
         {
            simm_printf(yes, "Error reading material for motion object: %s.\n", mo->name);
            return code_bad;
         }
         if (mo->materialname)
            free(mo->materialname);
            
         mstrcpy(&mo->materialname, buffer);
         
         mo->material = enter_material(ms, mo->materialname, declaring_element);
      }
      if (STRINGS_ARE_EQUAL(buffer, "drawmode"))
      {
         if (read_drawmode(*fp, &mo->drawmode) != code_fine)
         {
            simm_printf(yes,"Error reading drawmode for motion object: %s.\n", mo->name);
            return code_bad;
         }
      }
      if (STRINGS_ARE_EQUAL(buffer, "vectoraxis"))
      {
         if (fscanf(*fp, "%s", buffer) != 1)
         {
            simm_printf(yes, "Error reading vectoraxis for motion object: %s.\n", mo->name);
            return code_bad;
         }
         switch (buffer[0])
         {
            case 'x': case 'X': mo->vector_axis = XX; break;
            case 'y': case 'Y': mo->vector_axis = YY; break;
            case 'z': case 'Z': mo->vector_axis = ZZ; break;
         }
      }
   }
   return rc;
}

#ifndef ENGINE

/* -------------------------------------------------------------------------
   update_motion_object_instance_state - 
---------------------------------------------------------------------------- */
static void update_motion_object_instance_state (ModelStruct* ms, MotionSequence* motion, MotionObjectInstance* mi)
{
   MotionModelOptions* mmo = &motion->mopt;

   if (NOT_EQUAL_WITHIN_ERROR(mi->current_value, mmo->current_value))
   {
      int i;
      int mod = ms->modelnum;
      MotionObject* mo = &ms->motion_objects[mi->object];

#if 0
      double percent = (motion->mopt[mod].current_value - motion->min_value) /
	               (motion->max_value - motion->min_value);
/* used to be + 0.5, but model was occasionally being drawn with old force data.
the -0.5 guarantees that the data is coherent (although 1 frame behind, perhaps
the number should be  + 0.49 so whole numbers do not get rounded up).
JPL 1/24/01: the frame number is now set and stored in the mmo struct before
this function is called. So don't recompute it, just use the old value.
*/
      int frame = percent * (motion->number_of_datarows - 1) + MOTION_OFFSET;
#endif

      double position[3] = {0,0,0}, scale[3] = {1,1,1}, vector[3] = {0,0,0};
      double axis[3], angle = 0, color[3] = {-1,-1,-1};
      DMatrix m;
      SBoolean hasVectorComponent = no, hasColorComponent = no;

#if 0
      printf("frame = %d, percent = %lf\n", frame, percent);
      frame = circularize_motion_index(motion, frame);
#endif

      /* retrieve values for the motion object instance's animation channels */
      for (i = 0; i < mi->num_channels; i++)
      {
         double* value = NULL;
         
         switch (mi->channels[i].component)
         {
            case MO_TX: value = &position[XX]; break;
            case MO_TY: value = &position[YY]; break;
            case MO_TZ: value = &position[ZZ]; break;
            
            case MO_VX: value = &vector[XX]; hasVectorComponent = yes; break;
            case MO_VY: value = &vector[YY]; hasVectorComponent = yes; break;
            case MO_VZ: value = &vector[ZZ]; hasVectorComponent = yes; break;
            
            case MO_SX: value = &scale[XX]; break;
            case MO_SY: value = &scale[YY]; break;
            case MO_SZ: value = &scale[ZZ]; break;
            
            case MO_CR: value = &color[0]; hasColorComponent = yes; break;
            case MO_CG: value = &color[1]; hasColorComponent = yes; break;
            case MO_CB: value = &color[2]; hasColorComponent = yes; break;
         }
         
         if (value)
         {
            *value = motion->motiondata[mi->channels[i].column][mmo->current_frame];
         }
      }
      for (i = 0; i < 3; i++)
      {
         position[i] += mo->startingPosition[i];
         scale[i] *= mo->startingScale[i];
      }
      
      /* update the motion object instance's current transform:
       */
      if (hasVectorComponent)
      {
         double u[3], magnetude = normalize_vector(vector, vector);
         
         scale[mo->vector_axis] *= magnetude;
         
         /* avoid drawing zero-length vectors:
          */
         if (EQUAL_WITHIN_ERROR(scale[mo->vector_axis], 0.0))
            mi->visible = no;
         else
            mi->visible = yes;
         
         switch (mo->vector_axis)
         {
            case XX: u[XX] = 1.0; u[YY] = 0.0; u[ZZ] = 0.0; break;
            case YY: u[XX] = 0.0; u[YY] = 1.0; u[ZZ] = 0.0; break;
            case ZZ: u[XX] = 0.0; u[YY] = 0.0; u[ZZ] = 1.0; break;
         }
         angle = acos(DOT_VECTORS(u, vector));
         
         if (EQUAL_WITHIN_ERROR(angle, 0.0) || EQUAL_WITHIN_ERROR(angle, M_PI))
         {
             axis[XX] = u[ZZ]; /* pick an arbitrary vector perpendicular to */
             axis[YY] = u[XX]; /* 'u' in cases where the cross-product will fail */
             axis[ZZ] = u[YY];
         }
         else
             cross_vectors(u, vector, axis);
      }
      
      identity_matrix(m);
      scale_matrix(m, scale);
      
      if (NOT_EQUAL_WITHIN_ERROR(mo->startingXYZRotation[XX], 0.0))
         x_rotate_matrix_bodyfixed(m, mo->startingXYZRotation[XX] * DTOR);
      
      if (NOT_EQUAL_WITHIN_ERROR(mo->startingXYZRotation[YY], 0.0))
         y_rotate_matrix_bodyfixed(m, mo->startingXYZRotation[YY] * DTOR);
      
      if (NOT_EQUAL_WITHIN_ERROR(mo->startingXYZRotation[ZZ], 0.0))
         z_rotate_matrix_bodyfixed(m, mo->startingXYZRotation[ZZ] * DTOR);
      
      if (hasVectorComponent && NOT_EQUAL_WITHIN_ERROR(angle, 0.0))
         rotate_matrix_axis_angle(m, axis, angle);
      
      translate_matrix(m, position);
      
      copy_4x4matrix(m, mi->currentXform);

      /* update the motion object instance's current material if necessary:
       */
      if (hasColorComponent)
      {
         if (color[0] >= 0.0)
            mi->currentMaterial.ambient[0] = color[0];
         
         if (color[1] >= 0.0)
            mi->currentMaterial.ambient[1] = color[1];
         
         if (color[2] >= 0.0)
            mi->currentMaterial.ambient[2] = color[2];
         
         if ( ! mi->currentMaterial.ambient_defined)
         {
            mi->currentMaterial.ambient[3] = 1.0;
            mi->currentMaterial.ambient_defined = yes;
         }
         make_mat_display_list(&mi->currentMaterial);
         make_highlight_mat_display_list(&mi->currentMaterial);
      }
      
      mi->current_value = mmo->current_value;
   }
} /* update_motion_object_instance_state */

/* -------------------------------------------------------------------------
   draw_motion_objects - 
---------------------------------------------------------------------------- */
public void draw_motion_objects (ModelStruct* ms, ModelDrawOptions* mdo)
{
   if (ms->dis.applied_motion)
   {
      int i;
      MotionSequence* motion = ms->dis.applied_motion;
      MotionModelOptions* mmo = &motion->mopt;

      if (mmo->num_motion_object_instances <= 0)
         return;

      glPushMatrix();

      for (i = 0; i < mmo->num_motion_object_instances; i++)
      {
         MotionObjectInstance* mi = &mmo->motion_object_instances[i];
         MotionObject* mo = &ms->motion_objects[mi->object];
         PolyhedronStruct* ph = &mo->shape;

         update_motion_object_instance_state(ms, motion, mi);

         if ( ! mi->visible || mi->drawmode == none)
            continue;

         /* transform to the motion object's local frame:
          */
         pushframe(ms->modelnum);
         movetoframe(ms->modelnum, ms->currentframe, mi->segment);
         load_double_matrix(mi->currentXform);

         /* apply the motion object's surface material:
          */
         if (mdo->mode == GL_RENDER)
         {
            if (mi->currentMaterial.normal_list == -1)
               make_mat_display_list(&mi->currentMaterial);

            if (mi->currentMaterial.highlighted_list == -1)
               make_highlight_mat_display_list(&mi->currentMaterial);

            if (mo->shape.selected)
               glCallList(mi->currentMaterial.highlighted_list);
            else
               glCallList(mi->currentMaterial.normal_list);
         }
         else
         {
            int color_value = 0;
            GLubyte color[3];

            /* NEED SOME CODE HERE TO INITIALIZE 'color_value', if the
             * motion object is to be pickable.
             */
            pack_int_into_color(color_value, color);
            glColor3ubv(color);
         }

         /* draw the motion object:
          */
         glEnable(GL_NORMALIZE);

         glShadeModel(GL_FLAT);

         if (mi->drawmode == gouraud_shading)
         {
            glShadeModel(GL_SMOOTH);
            draw_gouraud_bone(ms, ph, &ms->dis);
         }
         else if (mi->drawmode == flat_shading)
         {
            draw_flat_bone(ms, ph, &ms->dis);
         }
         else if (mi->drawmode == solid_fill)
         {
            if (mdo->mode == GL_RENDER)
            {
               glPushAttrib(GL_ENABLE_BIT);
               glDisable(GL_LIGHTING);
               glColor3fv(mi->currentMaterial.ambient);
            }
            draw_solid_fill_bone(ms, ph, 0, 0, mdo);
            
            if (mdo->mode == GL_RENDER)
               glPopAttrib();
         }
         else if (mi->drawmode == outlined_polygons)
         {
            calc_camera_vector(ms, ms->dis.z_vector);
            draw_outlined_bone(ms, ph, NULL, &ms->dis);
         }
         else if (mi->drawmode == wireframe)
         {
            draw_wireframe_bone(ms, ph, 0, 0, mdo);
         }
         else if (mi->drawmode == bounding_box)
         {
            calc_camera_vector(ms, ms->dis.z_vector);
            draw_bounding_box_bone(ph, 0, 0, mdo, &ms->dis);
         }
         popframe(ms->modelnum);
      }
      glPopMatrix();

      glDisable(GL_NORMALIZE);
   }
}


ReturnCode write_motion(MotionSequence *motion, const char filename[])
{
   int i, j;
   FILE* file = simm_fopen(filename, "w");

   if (file == NULL)
      return code_bad;

   fprintf(file, "/* %s\n */\n\n", filename);
   fprintf(file, "name %s\n", motion->name);
   fprintf(file, "datacolumns %d\n", motion->number_of_datacolumns);
   fprintf(file, "datarows %d\n", motion->number_of_datarows);
   fprintf(file, "range %lf %f\n", motion->min_value, motion->max_value);
   if (motion->units)
      fprintf(file, "units %s\n", motion->units);
   if (motion->wrap)
      fprintf(file, "wrap\n");

   if (motion->enforce_loops == yes)
      fprintf(file, "enforce_loops yes\n");
   else
      fprintf(file, "enforce_loops no\n");
   if (motion->enforce_constraints == yes)
      fprintf(file, "enforce_constraints yes\n");
   else
      fprintf(file, "enforce_constraints no\n");

   if (motion->show_cursor)
      fprintf(file, "cursor %.2f %.2f %.2f\n", motion->cursor_color[0], motion->cursor_color[1], motion->cursor_color[2]);

   for (i = 0; i < motion->num_events; i++)
      fprintf(file, "event %lf %s\n", motion->event[i].x_coord, motion->event[i].name);

   fprintf(file, "event_color %f %f %f\n", motion->event_color[0], motion->event_color[1], motion->event_color[2]);

   if (motion->calc_derivatives)
      fprintf(file, "calc_derivatives %f\n", motion->time_step);
   fprintf(file, "endheader\n\n");
   
   for (j = 0; j < motion->number_of_datacolumns; j++)
      fprintf(file, "%s\t", motion->columnname[j]);
   fprintf(file, "\n");
   
   for (i = 0; i < motion->number_of_datarows; i++)
   {
      for (j = 0; j < motion->number_of_datacolumns; j++)
         fprintf(file, "%lf\t", motion->motiondata[j][i]);
      
      fprintf(file, "\n");
   }
   fclose(file);

   return code_fine;
}


MotionSequence* createMotionStruct(ModelStruct* ms)
{
   int i;
   ReturnCode rc;
   MotionSequence* motion;

   /* Find, or make, an empty slot for this new motion. */
   for (i = 0; i < ms->motion_array_size; i++)
   {
      if (ms->motion[i] == NULL)
         break;
   }

   if (i == ms->motion_array_size)
   {
      int j, old_size = ms->motion_array_size;
      ms->motion_array_size += MOTION_ARRAY_INCREMENT;
      ms->motion = (MotionSequence**)simm_realloc(ms->motion,
         (unsigned)((ms->motion_array_size)*sizeof(MotionSequence*)), &rc);
      if (rc == code_bad)
         return NULL;

      for (j = old_size; j < ms->motion_array_size; j++)
         ms->motion[j] = NULL;
   }

   ms->motion[i] = (MotionSequence*)simm_calloc(1, sizeof(MotionSequence));
   if (ms->motion[i] == NULL)
      return NULL;

   motion = ms->motion[i];
   ms->num_motions++;

   return motion;
}


void post_motion_event(ModelStruct* ms, MotionSequence* motion, int motion_index, int eventCode)
{
   SimmEvent se;

   memset(&se, 0, sizeof(se));
   
   se.event_code = eventCode;
   se.struct_ptr = motion;
   se.field1 = ms->modelnum;
   se.field2 = motion_index;
   
   queue_simm_event(se);
}


MotionSequence* find_nth_motion(ModelStruct* ms, int motcount)
{
   int i, count = 0;

   if (ms && motcount >= 1)
   {
      for (i = 0; i < ms->motion_array_size; i++)
         if (ms->motion[i])
            if (++count == motcount)
               break;

      if (i == ms->motion_array_size)
         return NULL;

      return ms->motion[i];
   }
   else
   {
      return NULL;
   }
}

#endif /* ENGINE */
