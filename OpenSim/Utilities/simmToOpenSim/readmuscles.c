/*******************************************************************************

   READMUSCLES.C

   Author: Peter Loan

   Date: 8-DEC-88

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Description: This file contains routines that read-in muscle
      descriptions from an input file.

   Routines:
      readmuscles : main routine to read input file for muscle descriptions
      getmuscle   : a big if-else block to read in each muscle parameter
      read_tendonflcurve : reads a tendon force-length curve from a muscle file
      read_activeflcurve : reads an active force-length curve from a muscle file
      read_passiveflcurve : reads a passive force-length curve from muscle file
      read_forcecvelocitycurve : reads force-velocity curve from a muscle file
      read_muscle_parameter : reads a muscle parameter (e.g. pennation) from file
      read_dynamic_parameters : reads dynamic (muscle model) parameters 

*******************************************************************************/

#ifdef __MWERKS__
   #include <unistd.h>
#endif

#include "universal.h"
#include "globals.h"
#include "functions.h"

#include "wefunctions.h"


/*************** DEFINES (for this file only) *********************************/



/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/



/*************** EXTERNED VARIABLES (declared in another file) ****************/



/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static ReturnCode read_muscle(int mod, FILE **fp, MuscleStruct* muscl,
			      int muscle_number);
static ReturnCode read_muscle_function(FILE** fp, MuscleStruct* musc,
				       MuscleStruct* default_musc,
				       SplineFunction** mfunc,
				       SplineFunction* default_mfunc,
				       char ending[], char description[]);
static ReturnCode read_muscle_parameter(FILE** fp, MuscleStruct* ms,
					MuscleStruct* default_musc, double** param,
					double* def_param, char param_name[],
					double min_value, double max_value);
static ReturnCode read_excitation(FILE** fp, int mod, MuscleStruct* ms, MuscleStruct* default_musc);
static ReturnCode read_dynamic_parameters(FILE** fp, ModelStruct* ms);
static SBoolean check_dynamic_param_array(FILE** fp, MuscleStruct* muscl, char text_string[]);
static ReturnCode read_ligament(FILE **fp, ModelStruct* ms);
static void sort_muscle_groups(ModelStruct*);




/* READ_MUSCLE_FILE: this is the main routine for reading a muscle input
 * file. It runs cpp on the muscle file, then reads it in. When it finds
 * the string "beginmuscle" it calls read_muscle() read in a muscle
 * definition.
 */

ReturnCode read_muscle_file(ModelStruct* ms, char filename[], SBoolean* file_exists, SBoolean showTopLevelMessages)
{

   char tempFile[512];
   int i, count, muscle_errors = 0;
   SBoolean default_muscle_defined = no, any_errors = no;
   FILE *fp;
   ReturnCode rc;

#ifdef ENGINE
strcpy(tempFile,".muscles");
#else
#ifdef WIN32
   strcpy(tempFile, glutGetTempFileName("simm-muscles"));
#else
   strcpy(tempFile, glutGetTempFileName(".muscles"));
#endif
#endif

	if (file_exists)
      *file_exists = no;

   if ((fp = preprocess_file(filename,tempFile)) == NULL)
   {
      if (showTopLevelMessages == yes)
      {
         (void)sprintf(buffer,"Unable to open muscle input file %s:", filename);
         message(buffer,HIGHLIGHT_TEXT,DEFAULT_MESSAGE_X_OFFSET);
         message("Loading model without muscles.",HIGHLIGHT_TEXT,
            DEFAULT_MESSAGE_X_OFFSET+20);
      }
      return code_fine;
   }

   ms->nummuscles = 0;

   while (1)
   {
      if (read_string(&fp,buffer) == EOF)
         break;

      if (buffer[0] == '#')
      {
         read_nonempty_line(&fp,buffer);
         continue;
      }

      if (STRINGS_ARE_EQUAL(buffer,"beginmuscle"))
      {
         read_string(&fp,buffer);
         if (STRINGS_ARE_EQUAL(buffer,"defaultmuscle"))
         {
            if (default_muscle_defined == yes)
            {
               error(recover,"Default muscle already defined. Ignoring redefinition.");
               while (1)
               {
                  read_string(&fp,buffer);
                  if (STRINGS_ARE_EQUAL(buffer, "endmuscle"))
                     break;
               }
            }
            else if (ms->nummuscles > 0)
            {
               error(none,"Error reading default muscle:");
               error(abort_action,"    You must define the default muscle before any other muscles.");
               return code_bad;
            }
            else
            {
               default_muscle_defined = yes;
               nullify_muscle(&ms->default_muscle);
               ms->default_muscle.numWrapStructs = 0;
               mstrcpy(&ms->default_muscle.name, "defaultmuscle");
               init_dynamic_param_array(ms,&ms->default_muscle);
               (void)read_muscle(ms->modelnum,&fp,&ms->default_muscle,-1);
            }
         }
         else
         {
	       /* Whether or not the user defined a default muscle, you want to
	        * initialize certain elements of the default muscle structure
	        * before reading in any muscles. So do it here if you're about to
	        * read in the first muscle.
	        */
            if (ms->nummuscles == 0)
            {
               if (init_default_muscle(ms->modelnum) == code_bad)
                  return (code_bad);
            }

            /* Check for duplicate muscle names */
            for (i=0, count=0; i<ms->nummuscles; i++)
            {
               if (STRINGS_ARE_EQUAL(buffer,ms->muscle[i].name))
                  count++;
            }
            if (count > 0)
            {
               (void)sprintf(errorbuffer,"Warning: %d muscles with name %s",
                  count+1, buffer);
               error(none,errorbuffer);
            }
            if (ms->nummuscles == ms->muscle_array_size)
            {
               ms->muscle_array_size += MUSCLE_ARRAY_INCREMENT;
               ms->muscle = (MuscleStruct*)simm_realloc(ms->muscle,
                  ms->muscle_array_size*sizeof(MuscleStruct),&rc);
               if (rc == code_bad)
               {
                  ms->muscle_array_size -= MUSCLE_ARRAY_INCREMENT;
                  return (code_bad);
               }
            }
            if (init_muscle(ms->modelnum,ms->nummuscles) == code_bad)
               return (code_bad);
            mstrcpy(&ms->muscle[ms->nummuscles].name,buffer);
            rc = read_muscle(ms->modelnum,&fp,&ms->muscle[ms->nummuscles], ms->nummuscles);
            if (rc == code_fine)
            {
               init_mparray(&ms->muscle[ms->nummuscles]);
               ms->nummuscles++;
            }
            else
            {
               free(ms->muscle[ms->nummuscles].momentarms);
               error(none,"Cancelling this muscle definition.");
            }
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"begindynamicparameters"))
      {
         if (read_dynamic_parameters(&fp,ms) == code_bad)
            return (code_bad);
      }
      else if (STRINGS_ARE_EQUAL(buffer,"beginmusclesurface") ||
         STRINGS_ARE_EQUAL(buffer,"beginligament"))
      {
         if (read_ligament(&fp,ms) == code_bad)
            return (code_bad);
      }
      else if (STRINGS_ARE_EQUAL(buffer, "beginfunction"))
      {
         if (read_function(ms->modelnum, &fp, yes) == code_bad)
            return (code_bad);
      }
      else
      {
         (void)sprintf(errorbuffer,"Unrecognized string \"%s\" found in muscle file.",
            buffer);
         error(recover,errorbuffer);
         muscle_errors++;
      }
      if (muscle_errors > 10)
      {
         error(none,"Too many errors to continue.");
         return (code_bad);
      }
   }

   (void)fclose(fp);
   unlink((const char*) tempFile);
   
   /* check that all functions used in muscle points are defined */
   for (i=0; i<ms->func_array_size; i++)
   {
      if (ms->function[i].used == yes && ms->function[i].defined == no)
      {
         (void)sprintf(errorbuffer,"Function f%d referenced in muscle file but not defined in muscle file.",
            ms->function[i].usernum);
         error(none,errorbuffer);
         any_errors = yes;
      }
   }
   if (any_errors == yes)
      return code_bad;


   /* If the muscle file contained a default muscle and no other
    * muscles, then make sure that everything in the default muscle
    * is initialized properly. This is usually done right before
    * the first real muscle is read, but if there are none it is
    * done here.
    */
   if (ms->nummuscles == 0)
   {
      if (init_default_muscle(ms->modelnum) == code_bad)
         return code_bad;
   }

   sort_muscle_groups(ms);

   if ( ! is_in_demo_mode())
   {
      if (showTopLevelMessages == yes)
      {
         (void)sprintf(buffer,"Read muscle file %s", filename);
         message(buffer,0,DEFAULT_MESSAGE_X_OFFSET);
      }
   }
   
   if (file_exists)
      *file_exists = yes;

   return (code_fine);

}


static ModelStruct* sSortingMuscGroupsForModel = NULL;

static int _compare_muscle_names (const void* elem1, const void* elem2)
{
   return strcmp(sSortingMuscGroupsForModel->muscle[*(int*) elem1].name,
                 sSortingMuscGroupsForModel->muscle[*(int*) elem2].name);
}

static void sort_muscle_groups (ModelStruct* ms)
{
   int i;
   
   sSortingMuscGroupsForModel = ms;
   
   for (i = 0; i < ms->numgroups; i++)
      qsort(ms->muscgroup[i].muscle_index,
            ms->muscgroup[i].number_of_muscles, sizeof(int), _compare_muscle_names);
}



/* READ_MUSCLE: this routine reads a muscle definition from a file. It is
 * basically a big if-else block with one section for each muscle parameter.
 */

static ReturnCode read_muscle(int mod, FILE **fp, MuscleStruct* muscle,
                              int muscle_number)
{

   char name[CHARBUFFER];
   int i, num_read;
   SBoolean matched_string;
   ModelStruct* ms;

   ms = model[mod];

   while (1)
   {
      if (read_string(fp,buffer) == EOF)
         break;

      if (STRINGS_ARE_EQUAL(buffer,"beginpoints"))
      {
         if (muscle == &ms->default_muscle || muscle_number < 0)
         {
            while (1)
            {
               read_string(fp, buffer);
               if (STRINGS_ARE_EQUAL(buffer, "endpoints"))
                  break;
            }
            continue;
         }
         // don't read points for default muscle
         if (read_muscle_attachment_points(mod, fp, muscle) == code_bad)
         {
            (void)sprintf(errorbuffer,"Error reading coordinates for muscle %s.",
               muscle->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"begingroups"))
      {
         muscle->group = read_muscle_groups(mod,fp,&muscle->numgroups, muscle_number);
         if (muscle->group == NULL && muscle->numgroups != 0)
         {
            muscle->numgroups = 0;
            (void)sprintf(errorbuffer,"Error reading groups for muscle %s.",
                    muscle->name);
            error(recover,errorbuffer);
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"begintendonforcelengthcurve"))
      {
         if (read_muscle_function(fp,muscle,&ms->default_muscle,
            &muscle->tendon_force_len_curve,
            ms->default_muscle.tendon_force_len_curve,
            "endtendonforcelengthcurve","tendon_force_length_curve") == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"beginactiveforcelengthcurve"))
      {
         if (read_muscle_function(fp,muscle,&ms->default_muscle,
            &muscle->active_force_len_curve,
            ms->default_muscle.active_force_len_curve,
            "endactiveforcelengthcurve","active_force_length_curve") == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"beginpassiveforcelengthcurve"))
      {
         if (read_muscle_function(fp,muscle,&ms->default_muscle,
            &muscle->passive_force_len_curve,
            ms->default_muscle.passive_force_len_curve,
            "endpassiveforcelengthcurve",
            "passive_force_length_curve") == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"beginforcevelocitycurve"))
      {
         if (read_muscle_function(fp,muscle,&ms->default_muscle,
            &muscle->force_vel_curve,
            ms->default_muscle.force_vel_curve,
            "endforcevelocitycurve","force_velocity_curve") == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"max_contraction_velocity"))
      {
         if (read_muscle_parameter(fp,muscle,&ms->default_muscle,
            &muscle->max_contraction_vel,
            ms->default_muscle.max_contraction_vel,
            "max_contraction_velocity",
            0.0,DONT_CHECK_DOUBLE) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"max_force"))
      {
         if (read_muscle_parameter(fp,muscle,&ms->default_muscle,
            &muscle->max_isometric_force,
            ms->default_muscle.max_isometric_force,
            "max_force",0.0,DONT_CHECK_DOUBLE) == code_bad)
            return code_bad;
         muscle->force = *muscle->max_isometric_force;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"tendon_slack_length"))
      {
         if (read_muscle_parameter(fp,muscle,&ms->default_muscle,
            &muscle->resting_tendon_length,
            ms->default_muscle.resting_tendon_length,
            "tendon_slack_length",0.0,DONT_CHECK_DOUBLE) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"optimal_fiber_length"))
      {
         if (read_muscle_parameter(fp,muscle,&ms->default_muscle,
            &muscle->optimal_fiber_length,
            ms->default_muscle.optimal_fiber_length,
            "optimal_fiber_length",0.0,DONT_CHECK_DOUBLE) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"pennation_angle"))
      {
         if (read_muscle_parameter(fp,muscle,&ms->default_muscle,
            &muscle->pennation_angle,
            ms->default_muscle.pennation_angle,
            "pennation_angle",0.0,90.0) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"min_thickness"))
      {
         if (read_muscle_parameter(fp,muscle,&ms->default_muscle,
            &muscle->min_thickness,
            ms->default_muscle.min_thickness,
            "min_thickness",0.0,DONT_CHECK_DOUBLE) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"max_thickness"))
      {
         if (read_muscle_parameter(fp,muscle,&ms->default_muscle,
            &muscle->max_thickness,
            ms->default_muscle.max_thickness,
            "max_thickness",0.0,DONT_CHECK_DOUBLE) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"min_material"))
      {
         if (fscanf(*fp,"%s", name) != 1)
         {
            (void)sprintf(errorbuffer,"Error reading min_material for %s muscle", muscle->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         else
         {
            muscle->min_material = (int*)simm_malloc(sizeof(int));
            *muscle->min_material = enter_material(ms,name,just_checking_element);
            if (*muscle->min_material == -1)

            {
               free(muscle->min_material);
               muscle->min_material = ms->default_muscle.min_material;
               (void)sprintf(errorbuffer,"Error reading min_material for muscle %s (%s is undefined)",
                  muscle->name, name);
               error(none,errorbuffer);
               error(none,"Using default material for min_material.");
            }
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"max_material"))
      {
         if (fscanf(*fp,"%s", name) != 1)
         {
            (void)sprintf(errorbuffer,"Error reading max_material for muscle %s", muscle->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
         else
         {
            muscle->max_material = (int*)simm_malloc(sizeof(int));
            *muscle->max_material = enter_material(ms,name,just_checking_element);
            if (*muscle->max_material == -1)
            {
               free(muscle->max_material);
               muscle->max_material = ms->default_muscle.max_material;
               (void)sprintf(errorbuffer,"Error reading max_material for muscle %s (%s is undefined)",
                  muscle->name, name);
               error(none,errorbuffer);
               error(none,"Using default material for max_material.");
            }
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"activation"))
      {
         if (fscanf(*fp,"%lg", &muscle->activation) != 1)
         {
            (void)sprintf(errorbuffer,"Error reading activation for muscle %s.",
               muscle->name);
            error(none,errorbuffer);
            error(none,"Using a value of 1.0");
         }
         else
         {
            muscle->initial_activation = muscle->activation;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"beginexcitation"))
      {
         if (read_excitation(fp,mod,muscle,&ms->default_muscle) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"excitation_format"))
      {
         fscanf(*fp, "%s", buffer);
         if (STRINGS_ARE_EQUAL(buffer, "spline_fit") || STRINGS_ARE_EQUAL(buffer, "natural_cubic"))
         {
            muscle->excitation_format = (SplineType*)simm_malloc(sizeof(SplineType));
            *(muscle->excitation_format) = natural_cubic;
            if (muscle->excitation)
               muscle->excitation->type = natural_cubic;
         }
         else if (STRINGS_ARE_EQUAL(buffer,"gcv_spline"))
         {
            muscle->excitation_format = (SplineType*)simm_malloc(sizeof(SplineType));
            *(muscle->excitation_format) = natural_cubic;
            if (muscle->excitation)
               muscle->excitation->type = natural_cubic;
            // TODO: gcv is not yet supported, use natural_cubic
         }
         else if (STRINGS_ARE_EQUAL(buffer,"step_function"))
         {
            muscle->excitation_format = (SplineType*)simm_malloc(sizeof(SplineType));
            *(muscle->excitation_format) = step_function;
            if (muscle->excitation)
               muscle->excitation->type = step_function;
         }
         else
         {
            if (muscle != &ms->default_muscle && ms->default_muscle.excitation_format != NULL)
            {
               muscle->excitation_format = ms->default_muscle.excitation_format;
               sprintf(errorbuffer, "Error reading excitation_format for muscle %s.\n",
                       muscle->name);
               strcat(errorbuffer, "Using value from default muscle.");
               error(recover, errorbuffer);
            }
            else
            {
               sprintf(errorbuffer, "Error reading excitation_format for muscle %s.",
                       muscle->name);
               error(abort_action, errorbuffer);
               return code_bad;
            }
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"muscle_model"))
      {
         muscle->muscle_model_index = (int*)simm_malloc(sizeof(int));
         if (muscle->muscle_model_index == NULL)
            return code_bad;
         num_read = fscanf(*fp,"%d", muscle->muscle_model_index);
         if (num_read != 1 || *(muscle->muscle_model_index) < 1)
         {
            if (muscle != &ms->default_muscle &&	ms->default_muscle.muscle_model_index != NULL)
            {
               *(muscle->muscle_model_index) = *(ms->default_muscle.muscle_model_index);
               (void)sprintf(errorbuffer,"Error reading muscle_model for muscle %s.",
                  muscle->name);
               error(none,errorbuffer);
               error(none,"Using value from default muscle.");
            }
            else
            {
               (void)sprintf(errorbuffer,"Error reading muscle_model for muscle %s.",
                  muscle->name);
               error(abort_action,errorbuffer);
               return code_bad;
            }
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"visible"))
      {
         if (read_string(fp,buffer) == EOF)
            break;
         
         if (STRINGS_ARE_EQUAL(buffer,"no") || STRINGS_ARE_EQUAL(buffer,"false"))
            muscle->display = no;
         else if (STRINGS_ARE_EQUAL(buffer,"yes") || STRINGS_ARE_EQUAL(buffer,"true"))
            muscle->display = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"output"))
      {
         if (read_string(fp,buffer) == EOF)
            break;
         
         if (STRINGS_ARE_EQUAL(buffer,"no") || STRINGS_ARE_EQUAL(buffer,"false"))
            muscle->output = no;
         else if (STRINGS_ARE_EQUAL(buffer,"yes") || STRINGS_ARE_EQUAL(buffer,"true"))
            muscle->output = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"wrapobjects"))
      {
         sprintf(buffer, "Error reading wrap specification for muscle: %s", muscle->name);

         error(none, buffer);
         error(none, "  (1) The \"wrapobjects\" keyword has been changed to \"wrapobject\".");
         error(none, "  (2) Space characters are no longer allowed in wrap object names.");
         error(none, "  (3) You may now specify the ellipsoid wrapping method following the wrap");
         error(none, "      object\'s name with the keywords: \"hybrid\", \"midpoint\", or \"axial\".");
         error(none, "      The wrapping method defaults to hybrid if none is specified.");

         read_line(fp, buffer);
      }
      else if (STRINGS_ARE_EQUAL(buffer,"wrapobject"))
      {
         char* p;
         int   i, j;
         MuscleWrapStruct* wrap = NULL;
         ReturnCode rc;
         SBoolean algorithmSpecified = no;

         read_line(fp, buffer);

         p = strtok(buffer," \t");

         if (p)
         {
            wrap = (MuscleWrapStruct*)simm_malloc(sizeof(MuscleWrapStruct));

            if (wrap)
            {
               /* initialize the muscle wrap association */
               wrap->wrap_object = get_wrap_object(ms, p);
               wrap->startPoint = -1;
               wrap->endPoint = -1;

               if (wrap->wrap_object == -1)
               {
                  sprintf(errorbuffer,"Unknown wrap object (%s) in muscle %s. Object will be ignored.", p, muscle->name);
                  error(none,errorbuffer);
                  FREE_IFNOTNULL(wrap);
               }
               else
               {
                  wrap->wrap_algorithm = ms->wrapobj[wrap->wrap_object].wrap_algorithm;

                  for (i = 0; i < 2; i++)
                  {
                     wrap->mp_wrap[i].is_auto_wrap_point = yes;
                     wrap->mp_wrap[i].wrap_distance = 0.0;
                     wrap->mp_wrap[i].num_wrap_pts = 0;
                     wrap->mp_wrap[i].wrap_pts = NULL;
                     wrap->mp_wrap[i].isVia = no;
                     wrap->mp_wrap[i].viaRange.genc = -1;
                     wrap->mp_wrap[i].viaRange.start = UNDEFINED_DOUBLE;
                     wrap->mp_wrap[i].viaRange.end = UNDEFINED_DOUBLE;

                     for (j = 0; j < 3; j++)
                     {
                        wrap->mp_wrap[i].fcn_index[j] = -1;
                        wrap->mp_wrap[i].gencoord[j] = -1;
                     }
                  }

                  for (i = 0; i < 3; i++)
                  {
                     wrap->c[i] = MINMDOUBLE;
                     wrap->r1[i] = MINMDOUBLE;
                     wrap->r2[i] = MINMDOUBLE;
                  }
               }
            }
         }

         /* if a wrap algorithm was specified, override the default algorithm */
         p = strtok(NULL," \t");

         if (p && wrap)
         {
            for (i = 0; i < WE_NUM_WRAP_ALGORITHMS; i++)
            {
               if (STRINGS_ARE_EQUAL(p, get_wrap_algorithm_name(i)))
               {
                  wrap->wrap_algorithm = i;
                  algorithmSpecified = yes;
                  break;
               }
            }
         }

         /* if a range of muscle points was specified, override the defaults */
         if (algorithmSpecified == yes)
            p = strtok(NULL," \t");

         if (p && wrap)
         {
            if (STRINGS_ARE_EQUAL(p, "range"))
            {
               p = strtok(NULL," \t");
               if (p)
               {
                  wrap->startPoint = atoi(p);
                  if (wrap->startPoint == 0)
                  {
                     (void)sprintf(errorbuffer, "Error reading wrap starting range in muscle %s.", muscle->name);
                     error(recover,errorbuffer);
                  }
               }
               else
               {
                  (void)sprintf(errorbuffer, "Error reading wrap starting range in muscle %s.", muscle->name);
                  error(recover,errorbuffer);
               }
               p = strtok(NULL," \t");
               if (p)
               {
                  wrap->endPoint = atoi(p);
                  if (wrap->endPoint == 0)
                  {
                     (void)sprintf(errorbuffer, "Error reading wrap ending range in muscle %s.", muscle->name);
                     error(recover,errorbuffer);
                  }
               }
               else
               {
                  (void)sprintf(errorbuffer, "Error reading wrap ending range in muscle %s.", muscle->name);
                  error(recover,errorbuffer);
               }
               if (wrap->startPoint < 1)
                  wrap->startPoint = -1;
               if (wrap->endPoint < 1)
                  wrap->endPoint = -1;
               if (wrap->startPoint > wrap->endPoint && wrap->endPoint != -1)
               {
                  wrap->startPoint = wrap->endPoint - 1;
                  (void)sprintf(errorbuffer, "Changing wrap range to (%d, %d) for muscle %s",
                     wrap->startPoint, wrap->endPoint, muscle->name);
                  error(recover,errorbuffer);
               }
            }
            else
            {
               (void)sprintf(errorbuffer,"Unrecognized string \"%s\" found while reading muscle %s.",
                             p, muscle->name);
               error(recover,errorbuffer);
            }
         }

         /* Now add the wrap structure to the muscle */
         if (wrap)
         {
            if (muscle->numWrapStructs == 0)
               muscle->wrapStruct = (MuscleWrapStruct**)simm_malloc(sizeof(MuscleWrapStruct*));
            else
               muscle->wrapStruct = (MuscleWrapStruct**)simm_realloc(muscle->wrapStruct,
               (muscle->numWrapStructs+1) * sizeof(MuscleWrapStruct*), &rc);
            muscle->wrapStruct[muscle->numWrapStructs++] = wrap;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer,"endmuscle"))
      {
         return code_fine;
      }
      else
      {
         matched_string = check_dynamic_param_array(fp,muscle,buffer);

         if (matched_string == no)
         {
            (void)sprintf(errorbuffer,
               "Unrecognized string \"%s\" found while reading muscle %s.",
               buffer, muscle->name);
            error(recover,errorbuffer);
         }
      }
   }

   return code_fine;

}



/* READ_MUSCLE_PARAMETER: this routine reads a single muscle parameter from
 * a file. The parameter must be stored as a pointer to a double in the
 * muscle structure. If there is an error when reading the parameter, the
 * pointer will be set to the corresponding parameter in the default muscle.
 */

static ReturnCode read_muscle_parameter(FILE** fp, MuscleStruct* ms,
					MuscleStruct* default_musc, double** param,
					double* def_param, char param_name[],
					double min_value, double max_value)
{

   *param = (double *)simm_malloc(sizeof(double));
   if (*param == NULL)
      return (code_bad);

   if (fscanf(*fp,"%lg", *param) != 1)
   {
      if (ms != default_musc && def_param != NULL)
      {
	 free(*param);
	 *param = def_param;
	 (void)sprintf(errorbuffer,"Error reading %s for muscle %s.",
		       param_name, ms->name);
	 error(none,errorbuffer);
	 error(none,"Using value from default muscle.");
	 return (code_fine);
      }
      else
      {
	 free(*param);
	 (void)sprintf(errorbuffer,"Error reading %s for muscle %s.",
		 param_name, ms->name);
	 error(abort_action,errorbuffer);
	 return (code_bad);
      }
   }
   else
   {
      if (min_value != DONT_CHECK_DOUBLE)
      {
	 if (**param < min_value)
	 {
	    sprintf(errorbuffer,"Error reading %s for muscle %s:",
		    param_name, ms->name);
	    error(none,errorbuffer);
	    sprintf(errorbuffer,"   value of %lf is below min allowable value (%lf).",
		    **param, min_value);
	    error(none,errorbuffer);
	    free(*param);
	    return (code_bad);
	 }
      }

      if (max_value != DONT_CHECK_DOUBLE)
      {
	 if (**param > max_value)
	 {
	    sprintf(errorbuffer,"Error reading %s for muscle %s:\n",
		    param_name, ms->name);
	    error(none,errorbuffer);
	    sprintf(errorbuffer,"   value of %lf is above max allowable value (%lf).",
		    **param, max_value);
	    error(none,errorbuffer);
	    free(*param);
	    return (code_bad);
	 }
      }
   }

   return (code_fine);

}



/* READ_MUSCLE_FUNCTION: this routine reads a spline function for a muscle
 * (e.g. active force-length curve). If there is an error when reading from
 * the file, it sets the function equal to the corresponding function in the
 * default muscle.
 */

static ReturnCode read_muscle_function(FILE** fp, MuscleStruct* ms,
				       MuscleStruct* default_musc,
				       SplineFunction** mfunc,
				       SplineFunction* default_mfunc,
				       char ending[], char description[])
{

   *mfunc = (SplineFunction*)simm_malloc(sizeof(SplineFunction));
   if (*mfunc == NULL)
      return (code_bad);

   malloc_function(*mfunc,SPLINE_ARRAY_INCREMENT);

   if (read_double_array(fp,ending,description,*mfunc) == code_bad)
   {
      free((*mfunc)->x);
      free((*mfunc)->y);
      free((*mfunc)->b);
      free((*mfunc)->c);
      free((*mfunc)->d);
      free(*mfunc);

      if (ms != default_musc && default_mfunc != NULL)
      {
	 *mfunc = default_mfunc;
	 (void)sprintf(errorbuffer,
		       "Error reading %s for muscle %s.", description, ms->name);
	 error(none,errorbuffer);
	 error(none,"Using function from default muscle.");
	 return (code_fine);
      }
      else
      {
	 (void)sprintf(errorbuffer,
		     "Error reading %s for muscle %s.", description, ms->name);
	 error(abort_action,errorbuffer);
	 return (code_bad);
      }
   }
   (*mfunc)->type = natural_cubic;
   (*mfunc)->usernum = -1;
   (*mfunc)->defined = yes;
   (*mfunc)->used = yes;
   calc_spline_coefficients(*mfunc);

   return (code_fine);

}


static ReturnCode read_excitation(FILE** fp, int mod, MuscleStruct* muscl,
				  MuscleStruct* default_musc)
{

   /* First read the name of the abscissa of the function. It should be either
    * the word "time" or the name of a generalized coordinate, of which this
    * excitation pattern will be a function.
    */

   if (fscanf(*fp,"%s", buffer) != 1)
   {
      (void)sprintf(errorbuffer,"Error reading name of abscissa in excitation function for muscle %s", muscl->name);
      error(abort_action,errorbuffer);
      return (code_bad);
   }

   if (STRINGS_ARE_EQUAL(buffer,"time"))
   {
      muscl->excitation_abscissa = TIME;
   }
   else
   {
      muscl->excitation_abscissa = enter_gencoord(mod,buffer,no);
      if (muscl->excitation_abscissa == -1)
      {
         (void)sprintf(errorbuffer,"Error reading abscissa of excitation function for muscle %s", muscl->name);
         error(none,errorbuffer);
         (void)sprintf(errorbuffer,"%s is neither \"time\" nor the name of a generalized coordinate", buffer);
         error(abort_action,errorbuffer);
         /* added dkb */
         while (1)
         {
            fscanf(*fp, "%s", buffer);
            if (STRINGS_ARE_EQUAL(buffer, "endmuscle")) //"endexcitation"))
               break;
         }
         return (code_bad);
      }
   }

   muscl->excitation = (SplineFunction*)simm_malloc(sizeof(SplineFunction));
   if (muscl->excitation == NULL)
      return (code_bad);

   malloc_function(muscl->excitation,SPLINE_ARRAY_INCREMENT);
   muscl->excitation->type = none_defined;

   if (read_double_array(fp,"endexcitation","excitation-sequence",
      muscl->excitation) == code_bad)
   {
      free(muscl->excitation->x);
      free(muscl->excitation->y);

      if (muscl != default_musc && default_musc->excitation != NULL)
      {
         muscl->excitation = default_musc->excitation;
         (void)sprintf(errorbuffer,
            "Error reading excitation sequence for muscle %s.", muscl->name);
         error(none,errorbuffer);
         error(none,"Using same sequence as for default muscle.");
         return (code_fine);
      }
      else
      {
         (void)sprintf(errorbuffer,
            "Error reading excitation sequence for muscle %s.", muscl->name);
         error(abort_action,errorbuffer);
         return (code_bad);
      }
   }

   if (muscl->excitation_format)
      muscl->excitation->type = *muscl->excitation_format;
   calc_spline_coefficients(muscl->excitation);
   return (code_fine);

}


int get_muscle_index(int mod, char muscle_name[])
{

   int i;

   for (i=0; i<model[mod]->nummuscles; i++)
      if (STRINGS_ARE_EQUAL(muscle_name,model[mod]->muscle[i].name))
	  return (i);

   return (-1);

}



int get_ligament_index(int mod, char lig_name[])
{

   int i;

   for (i=0; i<model[mod]->numligaments; i++)
      if (STRINGS_ARE_EQUAL(lig_name,model[mod]->ligament[i].name))
	  return (i);

   return (-1);

}


/* INIT_DEFAULT_MUSCLE: this routine should be called after the default muscle
 * has been read in. It makes sure that certain parameters are defined so that
 * there are always defaults for them (e.g. min_thickness).
 */

ReturnCode init_default_muscle(int mod)
{

   /* If the user did not specify min and max thickness for the default muscle,
    * define these two parameters. Then when you read in the other muscles, there
    * are defaults already set-up for the muscles to point to. After reading in
    * the complete model (bones and all), these default values may be changed
    * to better correspond to the particular model.
    */

   model[mod]->default_muscle.display = yes;

   if (model[mod]->default_muscle.min_thickness == NULL)
   {
      model[mod]->default_muscle.min_thickness = (double*)simm_malloc(sizeof(double));
      if (model[mod]->default_muscle.min_thickness == NULL)
	      return code_bad;
      *model[mod]->default_muscle.min_thickness = 0.002;
      model[mod]->specified_min_thickness = no;
   }

   if (model[mod]->default_muscle.max_thickness == NULL)
   {
      model[mod]->default_muscle.max_thickness = (double*)simm_malloc(sizeof(double));
      if (model[mod]->default_muscle.max_thickness == NULL)
	      return code_bad;
      *model[mod]->default_muscle.max_thickness = 0.008;
      model[mod]->specified_max_thickness = no;
   }

   if (model[mod]->default_muscle.min_material == NULL)
   {
      model[mod]->default_muscle.min_material = (int*)simm_malloc(sizeof(int));
      *model[mod]->default_muscle.min_material = model[mod]->dis.mat.default_muscle_min_material;
   }

   if (model[mod]->default_muscle.max_material == NULL)
   {
      model[mod]->default_muscle.max_material = (int*)simm_malloc(sizeof(int));
      *model[mod]->default_muscle.max_material = model[mod]->dis.mat.default_muscle_max_material;
   }

   if (model[mod]->default_muscle.excitation_format == NULL)
   {
      model[mod]->default_muscle.excitation_format = (SplineType*)simm_malloc(sizeof(SplineType));
      *model[mod]->default_muscle.excitation_format = step_function;
   }

   return code_fine;

}



static ReturnCode read_dynamic_parameters(FILE** fp, ModelStruct* ms)
{

   int array_size = 10;
   ReturnCode rc;

   ms->num_dynamic_params = 0;
   ms->dynamic_param_names = (char**)simm_malloc(array_size*sizeof(char*));

   if (ms->dynamic_param_names == NULL)
      return code_bad;

   while (1)
   {
      if (fscanf(*fp,"%s",buffer) != 1)
      {
         error(none,"EOF while reading dynamic parameters");
         return code_bad;
      }

      if (STRINGS_ARE_EQUAL(buffer,"enddynamicparameters"))
         break;

      /* Check to see if you need to increase the size of the array before
       * reading any more parameter names.
       */

      if (ms->num_dynamic_params >= array_size)
      {
         array_size += 10;
         ms->dynamic_param_names = (char**)simm_realloc(ms->dynamic_param_names,
            array_size*sizeof(char*),&rc);

         if (rc == code_bad)
            return code_bad;
      }

      mstrcpy(&ms->dynamic_param_names[ms->num_dynamic_params],buffer);

      ms->num_dynamic_params++;
   }

   ms->dynamic_param_names = (char**)simm_realloc(ms->dynamic_param_names,
			      ms->num_dynamic_params*sizeof(char*),&rc);

   return rc;

}



ReturnCode init_dynamic_param_array(ModelStruct* ms, MuscleStruct* muscl)
{

   int i;

   muscl->num_dynamic_params = ms->num_dynamic_params;
   muscl->dynamic_param_names = ms->dynamic_param_names;
   muscl->dynamic_params = (double**)simm_malloc(muscl->num_dynamic_params*sizeof(double*));

   if (muscl->dynamic_params == NULL)
      return code_bad;

   for (i=0; i<muscl->num_dynamic_params; i++)
      muscl->dynamic_params[i] = NULL;

   return code_fine;

}



static SBoolean check_dynamic_param_array(FILE** fp, MuscleStruct* muscl, char text_string[])
{

   int i;

   for (i=0; i<muscl->num_dynamic_params; i++)
   {
      if (STRINGS_ARE_EQUAL(text_string,muscl->dynamic_param_names[i]))
      {
         muscl->dynamic_params[i] = (double*)simm_malloc(sizeof(double));
         fscanf(*fp,"%lg", muscl->dynamic_params[i]);
         return yes;
      }
   }

   return no;

}


static ReturnCode read_ligament(FILE **fp, ModelStruct* ms)
{

   ReturnCode rc;
   LigamentStruct* lig;
   LigamentLine* ligline;
   SBoolean dummy;

   if (ms->numligaments == ms->ligament_array_size)
   {
      ms->ligament_array_size += MUSCLE_ARRAY_INCREMENT;
      ms->ligament = (LigamentStruct*)simm_realloc(ms->ligament,
			ms->ligament_array_size*sizeof(LigamentStruct),&rc);
      if (rc == code_bad)
      {
	 ms->ligament_array_size -= MUSCLE_ARRAY_INCREMENT;
	 return (code_bad);
      }
   }

   lig = &ms->ligament[ms->numligaments];

   lig->display = yes;
   lig->numlines = 0;
   lig->line_array_size = MUSCLE_ARRAY_INCREMENT;
   lig->activation = 1.0;
   lig->drawmode = gouraud_shading;
   lig->show_normals = no;

   lig->line = (LigamentLine*)simm_malloc(lig->line_array_size*sizeof(LigamentLine));
   if (lig->line == NULL)
      return (code_bad);

   if (fscanf(*fp,"%s",buffer) != 1)
   {
      error(abort_action,"Error starting a ligament definition.");
      return (code_bad);
   }
   else
   {
      mstrcpy(&lig->name,buffer);      
   }

   while (1)
   {
      if (read_string(fp,buffer) == EOF)
         break;

      if (STRINGS_ARE_EQUAL(buffer,"beginpoints"))
      {
         if (lig->numlines == lig->line_array_size)
         {
            lig->line_array_size += MUSCLE_ARRAY_INCREMENT;
            lig->line = (LigamentLine*)simm_realloc(lig->line,
               lig->line_array_size*sizeof(LigamentLine),&rc);
            if (rc == code_bad)
            {
               lig->line_array_size -= MUSCLE_ARRAY_INCREMENT;
               return (code_bad);
            }
         }
         ligline = &lig->line[lig->numlines];
         ligline->numpoints = 0;
         ligline->pt_array_size = 0;
         ligline->has_wrapping_points = no;
         ligline->first_on_point = 0;
         ligline->num_on_points = 0;
         ligline->pt = read_ligament_attachment_points(ms->modelnum,fp,&ligline->numpoints,
            &ligline->pt_array_size,&ligline->has_wrapping_points,&dummy);
         if (ligline->pt == NULL)
         {
            (void)sprintf(errorbuffer,"Error reading coordinates for muscle surface %s.",
               lig->name);
            error(abort_action,errorbuffer);
            return (code_bad);
         }
         if (ligline->has_wrapping_points == no)
            ligline->num_on_points = ligline->numpoints;
         lig->numlines++;
      }
      else if (STRINGS_ARE_EQUAL(buffer,"endmusclesurface") ||
         STRINGS_ARE_EQUAL(buffer,"endligament"))
      {
         break;
      }
      else
      {
         (void)sprintf(errorbuffer,
            "Unrecognized string \"%s\" found while reading muscle surface %s.",
            buffer, lig->name);
         error(recover,errorbuffer);
      }
   }

   ms->numligaments++;

   return (code_fine);

}

/* set the array of mp points in the musclepoints array struct */
ReturnCode init_mparray(MuscleStruct* muscle)
{
   if (muscle->musclepoints == NULL)
      return code_fine;

   muscle->musclepoints->num_points = 0;
   muscle->musclepoints->mp_array_size = muscle->musclepoints->num_orig_points + 2 * muscle->numWrapStructs;
   muscle->musclepoints->mp = (MusclePoint**)simm_malloc(muscle->musclepoints->mp_array_size * sizeof(MusclePoint*));
   if (muscle->musclepoints->mp == NULL)
   {
      (void)sprintf(errorbuffer, "Could not allocate mp array for %s.\n", muscle->name);
      error(none, errorbuffer);
      return code_bad;
   }
   return code_fine;
}

/* NULLIFY_MUSCLE: */

void nullify_muscle(MuscleStruct* muscle)
{
   muscle->name = NULL;
   muscle->display = yes;
   muscle->output = yes;
   muscle->numgroups = 0;
   muscle->group = NULL;
   muscle->max_isometric_force = NULL;
   muscle->pennation_angle = NULL;
   muscle->optimal_fiber_length = NULL;
   muscle->resting_tendon_length = NULL;
   muscle->min_thickness = NULL;
   muscle->max_thickness = NULL;
   muscle->min_material = NULL;
   muscle->max_material = NULL;
   muscle->max_contraction_vel = NULL;
   muscle->force_vel_curve = NULL;
   muscle->num_dynamic_params = 0;
   muscle->dynamic_param_names = NULL;
   muscle->dynamic_params = NULL;
   muscle->excitation = NULL;
   muscle->excitation_format = NULL;
   muscle->muscle_model_index = NULL;
   muscle->nummomentarms = 0;
   muscle->momentarms = NULL;
   muscle->tendon_force_len_curve = NULL;
   muscle->active_force_len_curve = NULL;
   muscle->passive_force_len_curve = NULL;
   muscle->wrapStruct = NULL;
   muscle->excitation_abscissa = TIME;
   muscle->saved_copy = NULL;
   muscle->musclepoints = NULL;

}

void nullify_savemuscle(SaveMuscle* muscle)
{

   muscle->name = NULL;
   muscle->display = yes;
   muscle->output = yes;
   muscle->musclepoints = NULL;
   muscle->numgroups = 0;
   muscle->group = NULL;
   muscle->max_isometric_force = NULL;
   muscle->pennation_angle = NULL;
   muscle->optimal_fiber_length = NULL;
   muscle->resting_tendon_length = NULL;
   muscle->min_thickness = NULL;
   muscle->max_thickness = NULL;
   muscle->min_material = NULL;
   muscle->max_material = NULL;
   muscle->max_contraction_vel = NULL;
   muscle->force_vel_curve = NULL;
   muscle->dynamic_param_names = NULL;
   muscle->dynamic_params = NULL;
   muscle->excitation = NULL;
   muscle->excitation_format = NULL;
   muscle->muscle_model_index = NULL;
   muscle->momentarms = NULL;
   muscle->tendon_force_len_curve = NULL;
   muscle->active_force_len_curve = NULL;
   muscle->passive_force_len_curve = NULL;
   muscle->wrapStruct = NULL;
   muscle->excitation_abscissa = TIME;

}

