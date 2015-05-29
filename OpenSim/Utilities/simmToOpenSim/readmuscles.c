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
      read_dynamic_parameter_names : reads dynamic (muscle model) parameter names

*******************************************************************************/

#include "universal.h"
#include "globals.h"
#include "functions.h"

#include "wefunctions.h"


/*************** DEFINES (for this file only) *********************************/



/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/
dpFunctionType excitationType = dpFunctionTypeUndefined;


/*************** EXTERNED VARIABLES (declared in another file) ****************/



/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static ReturnCode read_muscle(ModelStruct* ms, FILE* fp, dpMuscleStruct* muscle, int muscle_number);
static ReturnCode read_muscle_function(FILE* fp, ModelStruct* model, dpMuscleStruct* muscle,
                                       dpFunction*** mfunc, dpFunction** default_mfunc,
                                       char ending[], char description[]);
static void finish_muscle_definition(FILE* fp);
static ReturnCode read_muscle_parameter(FILE* fp, dpMuscleStruct* ms,
                    dpMuscleStruct* default_musc, double** param,
                    double* def_param, char param_name[],
                    double min_value, double max_value);
static ReturnCode read_excitation(FILE* fp, ModelStruct* ms, dpMuscleStruct* muscle);
static ReturnCode read_dynamic_parameter_names(FILE* fp, dpMuscleStruct* muscle);
static SBoolean check_dynamic_param_array(FILE* fp, dpMuscleStruct* muscle, char text_string[]);
static ReturnCode read_ligament(FILE* fp, ModelStruct* ms);
static void sort_muscle_groups(ModelStruct*);




/* READ_MUSCLE_FILE: this is the main routine for reading a muscle input
 * file. It runs cpp on the muscle file, then reads it in. When it finds
 * the string "beginmuscle" it calls read_muscle() read in a muscle
 * definition.
 */
ReturnCode read_muscle_file(ModelStruct* ms, char filename[], SBoolean* file_exists, SBoolean showTopLevelMessages)
{
   char tempFileName[CHARBUFFER];
   int i, count, muscle_errors = 0;
   SBoolean default_muscle_defined = no, any_errors = no;
   FILE* fp;
   ReturnCode rc;

#if ENGINE
   strcpy(tempFileName, ".muscles");
#else
   strcpy(tempFileName, glutGetTempFileName("simm-muscles"));
#endif

    if (file_exists)
      *file_exists = no;

   if ((fp = preprocess_file(filename, tempFileName)) == NULL)
   {
      if (showTopLevelMessages == yes)
      {
         (void)sprintf(buffer, "Unable to open muscle input file %s:", filename);
         message(buffer, HIGHLIGHT_TEXT,DEFAULT_MESSAGE_X_OFFSET);
         message("Loading model without muscles.", HIGHLIGHT_TEXT, DEFAULT_MESSAGE_X_OFFSET+20);
      }
      return code_fine;
   }

   ms->nummuscles = 0;
    ms->default_muscle = (dpMuscleStruct*)simm_calloc(1, sizeof(dpMuscleStruct));

   // To support deprecated parameter, reset this for each model loaded.
   excitationType = dpStepFunction;

   while (1)
   {
      if (read_string(fp, buffer) == EOF)
         break;

      if (buffer[0] == '#')
      {
         read_nonempty_line(fp, buffer);
         continue;
      }

      if (STRINGS_ARE_EQUAL(buffer, "beginmuscle"))
      {
         read_string(fp, buffer);
         if (STRINGS_ARE_EQUAL(buffer, "defaultmuscle"))
         {
            if (default_muscle_defined == yes)
            {
               error(recover, "Default muscle already defined. Ignoring redefinition.");
               while (1)
               {
                  read_string(fp, buffer);
                  if (STRINGS_ARE_EQUAL(buffer, "endmuscle"))
                     break;
               }
            }
            else if (ms->nummuscles > 0)
            {
               error(none, "Error reading default muscle:");
               error(abort_action, "    You must define the default muscle before any other muscles.");
               return code_bad;
            }
            else
            {
               default_muscle_defined = yes;
               ms->default_muscle->display = yes;
               ms->default_muscle->output = yes;
               ms->default_muscle->excitation_abscissa = (void**)simm_malloc(sizeof(void*));
               *(ms->default_muscle->excitation_abscissa) = TIME;
               ms->default_muscle->index = DEFAULT_MUSCLE_INDEX;
               if (ms->default_muscle->num_dynamic_params > 0)
                  ms->default_muscle->dynamic_params = (double**)simm_calloc(ms->default_muscle->num_dynamic_params, sizeof(double*));
               mstrcpy(&ms->default_muscle->name, "defaultmuscle");
               (void)read_muscle(ms, fp, ms->default_muscle, -1);
            }
         }
         else
         {
            // Whether or not the user defined a default muscle, you want to
            // initialize certain elements of the default muscle structure
            // before reading in any muscles. So do it here if you're about to
            // read in the first muscle.
            if (ms->nummuscles == 0)
            {
               if (init_default_muscle(ms) == code_bad)
                  return code_bad;
            }

            /* Check for duplicate muscle names */
            for (i=0, count=0; i<ms->nummuscles; i++)
            {
               if (STRINGS_ARE_EQUAL(buffer, ms->muscle[i]->name))
                  count++;
            }
            if (count > 0)
            {
               (void)sprintf(errorbuffer, "Warning: %d muscles with name %s", count+1, buffer);
               error(none,errorbuffer);
            }
            if (ms->nummuscles == ms->muscle_array_size)
            {
               ms->muscle_array_size += MUSCLE_ARRAY_INCREMENT;
               ms->muscle = (dpMuscleStruct**)simm_realloc(ms->muscle,
                  ms->muscle_array_size*sizeof(dpMuscleStruct*), &rc);
               if (rc == code_bad)
               {
                  ms->muscle_array_size -= MUSCLE_ARRAY_INCREMENT;
                  return code_bad;
               }
            }
            ms->muscle[ms->nummuscles] = (dpMuscleStruct*)simm_malloc(sizeof(dpMuscleStruct));
            if (init_muscle(ms, ms->muscle[ms->nummuscles], ms->default_muscle) == code_bad)
               return code_bad;
            ms->muscle[ms->nummuscles]->index = ms->nummuscles;
            mstrcpy(&ms->muscle[ms->nummuscles]->name, buffer);
            rc = read_muscle(ms, fp, ms->muscle[ms->nummuscles], ms->nummuscles);
            if (rc == code_fine)
            {
               post_init_muscle_path(ms->muscle[ms->nummuscles]->path, ms->muscle[ms->nummuscles]->numWrapStructs);
               ms->nummuscles++;
            }
            else
            {
               free(ms->muscle[ms->nummuscles]->momentarms);
               error(none, "Cancelling this muscle definition.");
               finish_muscle_definition(fp);
            }
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "begindynamicparameters"))
      {
         if (default_muscle_defined == yes || ms->nummuscles > 0)
         {
            error(none, "Error reading dynamic parameter names:");
            error(abort_action, "    You must define the names before the default muscle or any other muscles.");
            return code_bad;
         }
         else
         {
            if (read_dynamic_parameter_names(fp, ms->default_muscle) == code_bad)
               return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "beginmusclesurface") || STRINGS_ARE_EQUAL(buffer, "beginligament"))
      {
         if (read_ligament(fp, ms) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer, get_function_tag(dpFunctionTypeUndefined, 0))) // old-style function definitions
      {
         if (read_function(ms, fp, yes, dpNaturalCubicSpline, get_function_tag(dpFunctionTypeUndefined, 1)) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer, get_function_tag(dpStepFunction, 0)))
      {
         if (read_function(ms, fp, yes, dpStepFunction, get_function_tag(dpStepFunction, 1)) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer, get_function_tag(dpLinearFunction, 0)))
      {
         if (read_function(ms, fp, yes, dpLinearFunction, get_function_tag(dpLinearFunction, 1)) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer, get_function_tag(dpNaturalCubicSpline, 0)))
      {
         if (read_function(ms, fp, yes, dpNaturalCubicSpline, get_function_tag(dpNaturalCubicSpline, 1)) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer, get_function_tag(dpGCVSpline, 0)))
      {
         if (read_function(ms, fp, yes, dpGCVSpline, get_function_tag(dpGCVSpline, 1)) == code_bad)
            return code_bad;
      }
      else
      {
         (void)sprintf(errorbuffer, "Unrecognized string \"%s\" found in muscle file.",
            buffer);
         error(recover,errorbuffer);
         muscle_errors++;
      }
      if (muscle_errors > 10)
      {
         error(none, "Too many errors to continue.");
         return code_bad;
      }
   }

   (void)fclose(fp);
   (void)unlink((const char*)tempFileName);
   
   /* check that all functions used in muscle points are defined */
   for (i=0; i<ms->func_array_size; i++)
   {
      if (ms->function[i] && ms->function[i]->used == dpYes && ms->function[i]->status == dpFunctionUndefined)
      {
         (void)sprintf(errorbuffer, "Function f%d referenced in muscle file but not defined in muscle file.", ms->function[i]->usernum);
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
      if (init_default_muscle(ms) == code_bad)
         return code_bad;
   }

   sort_muscle_groups(ms);

#if ! ENGINE
   if ( ! is_in_demo_mode())
   {
      if (showTopLevelMessages == yes)
      {
         (void)sprintf(buffer, "Read muscle file %s", filename);
         message(buffer, 0, DEFAULT_MESSAGE_X_OFFSET);
      }
   }
#endif

   if (file_exists)
      *file_exists = yes;

   return code_fine;

}


static ModelStruct* sSortingMuscGroupsForModel = NULL;

static int _compare_muscle_names (const void* elem1, const void* elem2)
{
   return strcmp(sSortingMuscGroupsForModel->muscle[*(int*) elem1]->name,
                 sSortingMuscGroupsForModel->muscle[*(int*) elem2]->name);
}

static void sort_muscle_groups (ModelStruct* ms)
{
   int i;
   
   sSortingMuscGroupsForModel = ms;
   
   for (i = 0; i < ms->numgroups; i++)
      qsort(ms->muscgroup[i].muscle_index,
            ms->muscgroup[i].num_muscles, sizeof(int), _compare_muscle_names);
}


/* READ_MUSCLE: this routine reads a muscle definition from a file. It is
 * basically a big if-else block with one section for each muscle parameter.
 */
static ReturnCode read_muscle(ModelStruct* ms, FILE* fp, dpMuscleStruct* muscle, int muscle_number)
{
   char name[CHARBUFFER];
   int i, num_read, function_num, funcIndex;

   while (1)
   {
      if (read_string(fp, buffer) == EOF)
         break;

      if (STRINGS_ARE_EQUAL(buffer, "beginpoints"))
      {
         // don't read points for default muscle
         if (muscle == ms->default_muscle || muscle_number < 0)
         {
            while (1)
            {
               read_string(fp, buffer);
               if (STRINGS_ARE_EQUAL(buffer, "endpoints"))
                  break;
            }
            continue;
         }

         muscle->path = (dpMusclePathStruct*)simm_calloc(1, sizeof(dpMusclePathStruct));
         if (read_muscle_path(ms, fp, muscle->path) == code_bad)
         {
            (void)sprintf(errorbuffer, "Error reading coordinates for muscle %s.", muscle->name);
            error(abort_action,errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "begingroups"))
      {
         muscle->group = read_muscle_groups(ms, fp, &muscle->numgroups, muscle_number);
         if (muscle->group == NULL && muscle->numgroups != 0)
         {
            muscle->numgroups = 0;
            (void)sprintf(errorbuffer, "Error reading groups for muscle %s.", muscle->name);
            error(recover,errorbuffer);
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "begintendonforcelengthcurve")) // deprecated
      {
         if (read_muscle_function(fp, ms, muscle,
            &muscle->tendon_force_len_func, ms->default_muscle->tendon_force_len_func,
            "endtendonforcelengthcurve", "tendon_force_length_curve") == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "tendon_force_length_curve"))
      {
         read_string(fp, buffer);
         if (sscanf(buffer, "f%d", &function_num) != 1)
         {
            (void)sprintf(errorbuffer, "Error reading tendon_force_length_curve for muscle %s.", muscle->name);
            if (muscle != ms->default_muscle && ms->default_muscle != NULL)
            {
               muscle->tendon_force_len_func = ms->default_muscle->tendon_force_len_func;
               error(none, errorbuffer);
               error(none, "Using function from default muscle.");
               return code_fine;
            }
            else
            {
               error(abort_action, errorbuffer);
               return code_bad;
            }
         }
         muscle->tendon_force_len_func = (dpFunction**)simm_malloc(sizeof(dpFunction*));
         if (muscle->tendon_force_len_func == NULL)
            return code_bad;
         *muscle->tendon_force_len_func = ms->function[enter_function(ms, function_num, yes)];
      }
      else if (STRINGS_ARE_EQUAL(buffer, "beginactiveforcelengthcurve")) // deprecated
      {
         if (read_muscle_function(fp, ms, muscle,
            &muscle->active_force_len_func, ms->default_muscle->active_force_len_func,
            "endactiveforcelengthcurve", "active_force_length_curve") == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "active_force_length_curve"))
      {
         read_string(fp, buffer);
         if (sscanf(buffer, "f%d", &function_num) != 1)
         {
            (void)sprintf(errorbuffer, "Error reading active_force_length_curve for muscle %s.", muscle->name);
            if (muscle != ms->default_muscle && ms->default_muscle != NULL)
            {
               muscle->active_force_len_func = ms->default_muscle->active_force_len_func;
               error(none, errorbuffer);
               error(none, "Using function from default muscle.");
               return code_fine;
            }
            else
            {
               error(abort_action, errorbuffer);
               return code_bad;
            }
         }
         muscle->active_force_len_func = (dpFunction**)simm_malloc(sizeof(dpFunction*));
         if (muscle->active_force_len_func == NULL)
            return code_bad;
         *muscle->active_force_len_func = ms->function[enter_function(ms, function_num, yes)];
      }
      else if (STRINGS_ARE_EQUAL(buffer, "beginpassiveforcelengthcurve")) // deprecated
      {
         if (read_muscle_function(fp, ms, muscle,
            &muscle->passive_force_len_func, ms->default_muscle->passive_force_len_func,
            "endpassiveforcelengthcurve", "passive_force_length_curve") == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "passive_force_length_curve"))
      {
         read_string(fp, buffer);
         if (sscanf(buffer, "f%d", &function_num) != 1)
         {
            (void)sprintf(errorbuffer, "Error reading passive_force_length_curve for muscle %s.", muscle->name);
            if (muscle != ms->default_muscle && ms->default_muscle != NULL)
            {
               muscle->passive_force_len_func = ms->default_muscle->passive_force_len_func;
               error(none, errorbuffer);
               error(none, "Using function from default muscle.");
               return code_fine;
            }
            else
            {
               error(abort_action, errorbuffer);
               return code_bad;
            }
         }
         muscle->passive_force_len_func = (dpFunction**)simm_malloc(sizeof(dpFunction*));
         if (muscle->passive_force_len_func == NULL)
            return code_bad;
         *muscle->passive_force_len_func = ms->function[enter_function(ms, function_num, yes)];
      }
      else if (STRINGS_ARE_EQUAL(buffer, "beginforcevelocitycurve")) // deprecated
      {
         if (read_muscle_function(fp, ms, muscle,
            &muscle->force_vel_func, ms->default_muscle->force_vel_func,
            "endforcevelocitycurve", "force_velocity_curve") == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "force_velocity_curve"))
      {
         read_string(fp, buffer);
         if (sscanf(buffer, "f%d", &function_num) != 1)
         {
            (void)sprintf(errorbuffer, "Error reading force_velocity_curve for muscle %s.", muscle->name);
            if (muscle != ms->default_muscle && ms->default_muscle != NULL)
            {
               muscle->force_vel_func = ms->default_muscle->force_vel_func;
               error(none, errorbuffer);
               error(none, "Using function from default muscle.");
               return code_fine;
            }
            else
            {
               error(abort_action, errorbuffer);
               return code_bad;
            }
         }
         muscle->force_vel_func = (dpFunction**)simm_malloc(sizeof(dpFunction*));
         if (muscle->force_vel_func == NULL)
            return code_bad;
         *muscle->force_vel_func = ms->function[enter_function(ms, function_num, yes)];
      }
      else if (STRINGS_ARE_EQUAL(buffer, "max_contraction_velocity"))
      {
         if (read_muscle_parameter(fp, muscle, ms->default_muscle,
            &muscle->max_contraction_vel, ms->default_muscle->max_contraction_vel,
            "max_contraction_velocity", 0.0, DONT_CHECK_DOUBLE) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "max_force"))
      {
         if (read_muscle_parameter(fp, muscle, ms->default_muscle,
            &muscle->max_isometric_force,
            ms->default_muscle->max_isometric_force,
            "max_force",0.0,DONT_CHECK_DOUBLE) == code_bad)
            return code_bad;
         muscle->force = *muscle->max_isometric_force;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "tendon_slack_length"))
      {
         if (read_muscle_parameter(fp, muscle, ms->default_muscle,
            &muscle->resting_tendon_length,
            ms->default_muscle->resting_tendon_length,
            "tendon_slack_length",0.0,DONT_CHECK_DOUBLE) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "optimal_fiber_length"))
      {
         if (read_muscle_parameter(fp, muscle, ms->default_muscle,
            &muscle->optimal_fiber_length,
            ms->default_muscle->optimal_fiber_length,
            "optimal_fiber_length",0.0,DONT_CHECK_DOUBLE) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "pennation_angle"))
      {
         if (read_muscle_parameter(fp, muscle, ms->default_muscle,
            &muscle->pennation_angle,
            ms->default_muscle->pennation_angle,
            "pennation_angle",0.0,90.0) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "min_thickness"))
      {
         if (read_muscle_parameter(fp, muscle, ms->default_muscle,
            &muscle->min_thickness,
            ms->default_muscle->min_thickness,
            "min_thickness",0.0,DONT_CHECK_DOUBLE) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "max_thickness"))
      {
         if (read_muscle_parameter(fp, muscle, ms->default_muscle,
            &muscle->max_thickness,
            ms->default_muscle->max_thickness,
            "max_thickness",0.0,DONT_CHECK_DOUBLE) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "min_material"))
      {
         if (fscanf(fp, "%s", name) != 1)
         {
            (void)sprintf(errorbuffer, "Error reading min_material for %s muscle", muscle->name);
            error(abort_action, errorbuffer);
            return code_bad;
         }
         else
         {
            muscle->min_material = (int*)simm_malloc(sizeof(int));
            *muscle->min_material = enter_material(ms,name,just_checking_element);
            if (*muscle->min_material == -1)
            {
               free(muscle->min_material);
               if (muscle == ms->default_muscle)
                  muscle->min_material = NULL; // will get reset later, in init_default_muscle()
               else
                  muscle->min_material = ms->default_muscle->min_material;
               (void)sprintf(errorbuffer, "Error reading min_material for muscle %s (%s is undefined)",
                  muscle->name, name);
               error(none,errorbuffer);
               error(none, "Using default material for min_material.");
            }
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "max_material"))
      {
         if (fscanf(fp, "%s", name) != 1)
         {
            (void)sprintf(errorbuffer, "Error reading max_material for muscle %s", muscle->name);
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
               if (muscle == ms->default_muscle)
                  muscle->max_material = NULL; // will get reset later, in init_default_muscle()
               else
                  muscle->max_material = ms->default_muscle->max_material;
               (void)sprintf(errorbuffer, "Error reading max_material for muscle %s (%s is undefined)",
                  muscle->name, name);
               error(none,errorbuffer);
               error(none, "Using default material for max_material.");
            }
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "activation"))
      {
         if (fscanf(fp, "%lg", &muscle->dynamic_activation) != 1)
         {
            (void)sprintf(errorbuffer, "Error reading activation for muscle %s.", muscle->name);
            error(none,errorbuffer);
            error(none, "Using a value of 1.0");
            muscle->dynamic_activation = 1.0;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "beginexcitation")) // deprecated
      {
         if (read_excitation(fp, ms, muscle) == code_bad)
            return code_bad;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "excitation"))
      {
         read_line(fp, buffer);
         if (sscanf(buffer, "f%d(%s)", &function_num, name) != 2) //TODO5.0: support spaces in here, like in read_refeq()?
         {
            (void)sprintf(errorbuffer, "Error reading excitation for muscle %s.", muscle->name);
            if (muscle != ms->default_muscle && ms->default_muscle != NULL)
            {
               muscle->excitation_func = ms->default_muscle->excitation_func;
               error(none, errorbuffer);
               error(none, "Using function from default muscle.");
               return code_fine;
            }
            else
            {
               error(abort_action, errorbuffer);
               return code_bad;
            }
         }
         muscle->excitation_abscissa = (void**)simm_malloc(sizeof(void*));
         if (muscle->excitation_abscissa == NULL)
            return code_bad;
         strip_brackets_from_string(name);
         if (STRINGS_ARE_EQUAL(name, "time"))
            *muscle->excitation_abscissa = TIME;
         else
            *muscle->excitation_abscissa = enter_gencoord(ms, name, no);
         if (*muscle->excitation_abscissa == NULL)
         {
            (void)sprintf(errorbuffer, "Error reading abscissa of excitation function for muscle %s", muscle->name);
            error(none, errorbuffer);
            (void)sprintf(errorbuffer, "%s is neither \"time\" nor the name of a generalized coordinate", buffer);
            error(abort_action, errorbuffer);
            finish_muscle_definition(fp);
            return code_bad;
         }
         muscle->excitation_func = (dpFunction**)simm_malloc(sizeof(dpFunction*));
         if (muscle->excitation_func == NULL)
            return code_bad;
         *muscle->excitation_func = ms->function[enter_function(ms, function_num, yes)];
      }
      else if (STRINGS_ARE_EQUAL(buffer, "excitation_format")) // deprecated
      {
         fscanf(fp, "%s", buffer);
         // If the excitation format is defined after the excitation, then you can store
         // the format directly in the excitation function. If it is defined before, store
         // it in a static global for use later.
         if (STRINGS_ARE_EQUAL(buffer, "spline_fit") || STRINGS_ARE_EQUAL(buffer, get_function_type_name(dpNaturalCubicSpline)))
         {
            if (muscle->excitation_func)
            {
               if ((*muscle->excitation_func)->type != dpNaturalCubicSpline)
               {
                  (*muscle->excitation_func)->type = dpNaturalCubicSpline;
                  calc_function_coefficients(*muscle->excitation_func);
               }
            }
            else
            {
               excitationType = dpNaturalCubicSpline;
            }
         }
         else if (STRINGS_ARE_EQUAL(buffer, get_function_type_name(dpGCVSpline)))
         {
            if (muscle->excitation_func)
            {
               if ((*muscle->excitation_func)->type != dpGCVSpline)
               {
                  (*muscle->excitation_func)->type = dpGCVSpline;
                  calc_function_coefficients(*muscle->excitation_func);
               }
            }
            else
            {
               excitationType = dpGCVSpline;
            }
         }
         else if (STRINGS_ARE_EQUAL(buffer, get_function_type_name(dpStepFunction)))
         {
            if (muscle->excitation_func)
            {
               if ((*muscle->excitation_func)->type != dpStepFunction)
               {
                  (*muscle->excitation_func)->type = dpStepFunction;
                  calc_function_coefficients(*muscle->excitation_func);
               }
            }
            else
            {
               excitationType = dpStepFunction;
            }
         }
         else if (STRINGS_ARE_EQUAL(buffer, get_function_type_name(dpLinearFunction)))
         {
            if (muscle->excitation_func)
            {
               if ((*muscle->excitation_func)->type != dpLinearFunction)
               {
                  (*muscle->excitation_func)->type = dpLinearFunction;
                  calc_function_coefficients(*muscle->excitation_func);
               }
            }
            else
            {
               excitationType = dpLinearFunction;
            }
         }
         else
         {
            sprintf(errorbuffer, "Error reading excitation_format for muscle %s.", muscle->name);
            error(abort_action, errorbuffer);
            return code_bad;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "muscle_model"))
      {
         muscle->muscle_model_index = (int*)simm_malloc(sizeof(int));
         if (muscle->muscle_model_index == NULL)
            return code_bad;
         num_read = fscanf(fp, "%d", muscle->muscle_model_index);
         if (num_read != 1 || *(muscle->muscle_model_index) < 1)
         {
            if (muscle != ms->default_muscle && ms->default_muscle->muscle_model_index != NULL)
            {
               *(muscle->muscle_model_index) = *(ms->default_muscle->muscle_model_index);
               (void)sprintf(errorbuffer, "Error reading muscle_model for muscle %s.",
                  muscle->name);
               error(none,errorbuffer);
               error(none, "Using value from default muscle.");
            }
            else
            {
               (void)sprintf(errorbuffer, "Error reading muscle_model for muscle %s.",
                  muscle->name);
               error(abort_action,errorbuffer);
               return code_bad;
            }
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "visible"))
      {
         if (read_string(fp, buffer) == EOF)
            break;
         
         if (STRINGS_ARE_EQUAL(buffer, "no") || STRINGS_ARE_EQUAL(buffer, "false"))
            muscle->display = no;
         else if (STRINGS_ARE_EQUAL(buffer, "yes") || STRINGS_ARE_EQUAL(buffer, "true"))
            muscle->display = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "output"))
      {
         if (read_string(fp, buffer) == EOF)
            break;
         
         if (STRINGS_ARE_EQUAL(buffer, "no") || STRINGS_ARE_EQUAL(buffer, "false"))
            muscle->output = no;
         else if (STRINGS_ARE_EQUAL(buffer, "yes") || STRINGS_ARE_EQUAL(buffer, "true"))
            muscle->output = yes;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "wrapobjects"))
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
      else if (STRINGS_ARE_EQUAL(buffer, "wrapobject"))
      {
         char* p;
         int   i, j;
         dpMuscleWrapStruct* wrap = NULL;
         ReturnCode rc;
         SBoolean algorithmSpecified = no;

         read_line(fp, buffer);

         p = strtok(buffer, " \t");

         if (p)
         {
            wrap = (dpMuscleWrapStruct*)simm_malloc(sizeof(dpMuscleWrapStruct));

            if (wrap)
            {
               /* initialize the muscle wrap association */
               wrap->wrap_object = get_wrap_object(ms, p);
               wrap->startPoint = -1;
               wrap->endPoint = -1;

               if (wrap->wrap_object == NULL)
               {
                  sprintf(errorbuffer, "Unknown wrap object (%s) in muscle %s. Object will be ignored.", p, muscle->name);
                  error(none,errorbuffer);
                  FREE_IFNOTNULL(wrap);
               }
               else
               {
                  wrap->wrap_algorithm = wrap->wrap_object->wrap_algorithm;

                  for (i = 0; i < 2; i++)
                     init_musclepoint(&wrap->mp_wrap[i]);

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
         p = strtok(NULL, " \t");

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
            p = strtok(NULL, " \t");

         if (p && wrap)
         {
            if (STRINGS_ARE_EQUAL(p, "range"))
            {
               p = strtok(NULL, " \t");
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
               p = strtok(NULL, " \t");
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
               (void)sprintf(errorbuffer, "Unrecognized string \"%s\" found while reading muscle %s.",
                             p, muscle->name);
               error(recover,errorbuffer);
            }
         }

         /* Now add the wrap structure to the muscle */
         if (wrap)
         {
            if (muscle->numWrapStructs == 0)
               muscle->wrapStruct = (dpMuscleWrapStruct**)simm_malloc(sizeof(dpMuscleWrapStruct*));
            else
               muscle->wrapStruct = (dpMuscleWrapStruct**)simm_realloc(muscle->wrapStruct,
               (muscle->numWrapStructs+1) * sizeof(dpMuscleWrapStruct*), &rc);
            muscle->wrapStruct[muscle->numWrapStructs++] = wrap;
         }
      }
      else if (STRINGS_ARE_EQUAL(buffer, "endmuscle"))
      {
         return code_fine;
      }
      else
      {
         SBoolean matched_string = check_dynamic_param_array(fp, muscle, buffer);

         if (matched_string == no)
         {
            (void)sprintf(errorbuffer, "Unrecognized string \"%s\" found while reading muscle %s.",
               buffer, muscle->name);
            error(recover, errorbuffer);
         }
      }
   }

   return code_fine;
}


/* This function scans forward to the end of a muscle definition.
 * It should be called only after encountering an error in the definition,
 * as it skips over everything until "endmuscle" is found.
 */
static void finish_muscle_definition(FILE* fp)
{
   while (1)
   {
      fscanf(fp, "%s", buffer);
      if (STRINGS_ARE_EQUAL(buffer, "endmuscle"))
         break;
   }
}


/* READ_MUSCLE_PARAMETER: this routine reads a single muscle parameter from
 * a file. The parameter must be stored as a pointer to a double in the
 * muscle structure. If there is an error when reading the parameter, the
 * pointer will be set to the corresponding parameter in the default muscle.
 */
static ReturnCode read_muscle_parameter(FILE* fp, dpMuscleStruct* ms,
                    dpMuscleStruct* default_musc, double** param,
                    double* def_param, char param_name[],
                    double min_value, double max_value)
{
   *param = (double *)simm_malloc(sizeof(double));
   if (*param == NULL)
      return code_bad;

   if (fscanf(fp, "%lg", *param) != 1)
   {
      if (ms != default_musc && def_param != NULL)
      {
         free(*param);
         *param = def_param;
         (void)sprintf(errorbuffer, "Error reading %s for muscle %s.",
            param_name, ms->name);
         error(none,errorbuffer);
         error(none, "Using value from default muscle.");
         return code_fine;
      }
      else
      {
         free(*param);
         (void)sprintf(errorbuffer, "Error reading %s for muscle %s.",
            param_name, ms->name);
         error(abort_action,errorbuffer);
         return code_bad;
      }
   }
   else
   {
      if (min_value != DONT_CHECK_DOUBLE)
      {
         if (**param < min_value)
         {
            sprintf(errorbuffer, "Error reading %s for muscle %s:",
               param_name, ms->name);
            error(none,errorbuffer);
            sprintf(errorbuffer, "   value of %lf is below min allowable value (%lf).",
               **param, min_value);
            error(none,errorbuffer);
            free(*param);
            return code_bad;
         }
      }

      if (max_value != DONT_CHECK_DOUBLE)
      {
         if (**param > max_value)
         {
            sprintf(errorbuffer, "Error reading %s for muscle %s:\n",
               param_name, ms->name);
            error(none,errorbuffer);
            sprintf(errorbuffer, "   value of %lf is above max allowable value (%lf).",
               **param, max_value);
            error(none,errorbuffer);
            free(*param);
            return code_bad;
         }
      }
   }

   return code_fine;
}



/* READ_MUSCLE_FUNCTION: this routine reads a function for a muscle
 * (e.g. active force-length curve). If there is an error when reading from
 * the file, it sets the function equal to the corresponding function in the
 * default muscle.
 */
static ReturnCode read_muscle_function(FILE* fp, ModelStruct* model, dpMuscleStruct* muscle,
                                       dpFunction*** mfunc, dpFunction** default_mfunc,
                                       char ending[], char description[])
{
   dpFunction newFunc;

   *mfunc = (dpFunction**)simm_malloc(sizeof(dpFunction*));
   if (*mfunc == NULL)
      return code_bad;

   malloc_function(&newFunc, FUNCTION_ARRAY_INCREMENT);

   if (read_double_array(fp, ending, description, &newFunc) == code_bad)
   {
      free_function(&newFunc, no);
      free(*mfunc);

      if (muscle != model->default_muscle && default_mfunc != NULL)
      {
         *mfunc = default_mfunc;
         (void)sprintf(errorbuffer, "Error reading %s for muscle %s.", description, muscle->name);
         error(none, errorbuffer);
         error(none, "Using function from default muscle.");
         return code_fine;
      }
      else
      {
         (void)sprintf(errorbuffer, "Error reading %s for muscle %s.", description, muscle->name);
         error(abort_action, errorbuffer);
         return code_bad;
      }
   }

   newFunc.type = dpNaturalCubicSpline;
   newFunc.cutoff_frequency = -1.0;

   calc_function_coefficients(&newFunc);

   *(*mfunc) = load_simm_function(model, &newFunc, yes);

   return code_fine;
}


static ReturnCode read_excitation(FILE* fp, ModelStruct* ms, dpMuscleStruct* muscle)
{
   int abscissa = -2;
   dpFunction newFunc;

   /* First read the name of the abscissa of the function. It should be either
    * the word "time" or the name of a generalized coordinate, of which this
    * excitation pattern will be a function.
    */
   if (fscanf(fp, "%s", buffer) != 1)
   {
      (void)sprintf(errorbuffer, "Error reading name of abscissa in excitation function for muscle %s", muscle->name);
      error(abort_action, errorbuffer);
      return code_bad;
   }

   muscle->excitation_abscissa = (void**)simm_malloc(sizeof(void*));
   if (muscle->excitation_abscissa == NULL)
      return code_bad;

   if (STRINGS_ARE_EQUAL(buffer, "time"))
   {
      *muscle->excitation_abscissa = TIME;
   }
   else
   {
      *muscle->excitation_abscissa = enter_gencoord(ms, buffer, no);
      if (*muscle->excitation_abscissa == NULL)
      {
         (void)sprintf(errorbuffer, "Error reading abscissa of excitation function for muscle %s", muscle->name);
         error(none,errorbuffer);
         (void)sprintf(errorbuffer, "%s is neither \"time\" nor the name of a generalized coordinate", buffer);
         error(abort_action, errorbuffer);
         finish_muscle_definition(fp);
         return code_bad;
      }
   }

   muscle->excitation_func = (dpFunction**)simm_malloc(sizeof(dpFunction*));
   if (muscle->excitation_func == NULL)
      return code_bad;

   malloc_function(&newFunc, FUNCTION_ARRAY_INCREMENT);
   newFunc.type = excitationType;
   newFunc.cutoff_frequency = -1.0;

   if (read_double_array(fp, "endexcitation", "excitation-sequence", &newFunc) == code_bad)
   {
      free_function(&newFunc, no);

      if (muscle != ms->default_muscle && ms->default_muscle->excitation_func != NULL)
      {
         muscle->excitation_func = ms->default_muscle->excitation_func;
         (void)sprintf(errorbuffer, "Error reading excitation sequence for muscle %s.", muscle->name);
         error(none, errorbuffer);
         error(none, "Using same sequence as for default muscle.");
         return code_fine;
      }
      else
      {
         (void)sprintf(errorbuffer, "Error reading excitation sequence for muscle %s.", muscle->name);
         error(abort_action, errorbuffer);
         return code_bad;
      }
   }

   calc_function_coefficients(&newFunc);

   *muscle->excitation_func = load_simm_function(ms, &newFunc, yes);

    return code_fine;
}


/* INIT_DEFAULT_MUSCLE: this routine should be called after the default muscle
 * has been read in. It makes sure that certain parameters are defined so that
 * there are always defaults for them (e.g. min_thickness).
 */
ReturnCode init_default_muscle(ModelStruct* ms)
{
   /* If the user did not specify min and max thickness for the default muscle,
    * define these two parameters. Then when you read in the other muscles, there
    * are defaults already set-up for the muscles to point to. After reading in
    * the complete model (bones and all), these default values may be changed
    * to better correspond to the particular model.
    */
   ms->default_muscle->index = DEFAULT_MUSCLE_INDEX;
   ms->default_muscle->display = yes;

   if (ms->default_muscle->min_thickness == NULL)
   {
      ms->default_muscle->min_thickness = (double*)simm_malloc(sizeof(double));
      if (ms->default_muscle->min_thickness == NULL)
          return code_bad;
      *ms->default_muscle->min_thickness = 0.002;
      ms->specified_min_thickness = no;
   }

   if (ms->default_muscle->max_thickness == NULL)
   {
      ms->default_muscle->max_thickness = (double*)simm_malloc(sizeof(double));
      if (ms->default_muscle->max_thickness == NULL)
          return code_bad;
      *ms->default_muscle->max_thickness = 0.008;
      ms->specified_max_thickness = no;
   }

   if (ms->default_muscle->min_material == NULL)
   {
      ms->default_muscle->min_material = (int*)simm_malloc(sizeof(int));
      *ms->default_muscle->min_material = ms->dis.mat.default_muscle_min_material;
   }

   if (ms->default_muscle->max_material == NULL)
   {
      ms->default_muscle->max_material = (int*)simm_malloc(sizeof(int));
      *ms->default_muscle->max_material = ms->dis.mat.default_muscle_max_material;
   }

   return code_fine;
}


static ReturnCode read_dynamic_parameter_names(FILE* fp, dpMuscleStruct* muscle)
{
   int array_size = 10;
   ReturnCode rc = code_fine;

   muscle->num_dynamic_params = 0;
   muscle->dynamic_param_names = (char**)simm_malloc(array_size * sizeof(char*));

   if (muscle->dynamic_param_names == NULL)
      return code_bad;

   while (1)
   {
      if (fscanf(fp, "%s", buffer) != 1)
      {
         error(none, "EOF while reading dynamic parameters");
         return code_bad;
      }

      if (STRINGS_ARE_EQUAL(buffer, "enddynamicparameters"))
         break;

      // Check to see if you need to increase the size of the array before
      // reading any more parameter names.
      if (muscle->num_dynamic_params >= array_size)
      {
         array_size += 10;
         muscle->dynamic_param_names = (char**)simm_realloc(muscle->dynamic_param_names,
            array_size*sizeof(char*), &rc);

         if (rc == code_bad)
            return code_bad;
      }

      mstrcpy(&muscle->dynamic_param_names[muscle->num_dynamic_params], buffer);

      muscle->num_dynamic_params++;
   }

   muscle->dynamic_param_names = (char**)simm_realloc(muscle->dynamic_param_names,
      muscle->num_dynamic_params*sizeof(char*), &rc);

   return rc;
}


ReturnCode init_dynamic_param_array(dpMuscleStruct* muscle, dpMuscleStruct* default_muscle)
{
   int i;

   muscle->num_dynamic_params = default_muscle->num_dynamic_params;
   muscle->dynamic_param_names = default_muscle->dynamic_param_names;
   muscle->dynamic_params = (double**)simm_calloc(muscle->num_dynamic_params, sizeof(double*));

   if (muscle->dynamic_params == NULL)
      return code_bad;

   for (i=0; i<muscle->num_dynamic_params; i++)
      muscle->dynamic_params[i] = default_muscle->dynamic_params[i];

   return code_fine;
}


static SBoolean check_dynamic_param_array(FILE* fp, dpMuscleStruct* muscle, char text_string[])
{
   int i;

   for (i=0; i<muscle->num_dynamic_params; i++)
   {
      if (STRINGS_ARE_EQUAL(text_string, muscle->dynamic_param_names[i]))
      {
         muscle->dynamic_params[i] = (double*)simm_malloc(sizeof(double));
         fscanf(fp, "%lg", muscle->dynamic_params[i]);
         return yes;
      }
   }

   return no;
}


static ReturnCode read_ligament(FILE* fp, ModelStruct* ms)
{
   ReturnCode rc = code_fine;
   LigamentStruct* lig;
   dpMusclePathStruct* ligline;

   if (ms->numligaments == ms->ligament_array_size)
   {
      ms->ligament_array_size += MUSCLE_ARRAY_INCREMENT;
      ms->ligament = (LigamentStruct*)simm_realloc(ms->ligament,
         ms->ligament_array_size*sizeof(LigamentStruct),&rc);
      if (rc == code_bad)
      {
         ms->ligament_array_size -= MUSCLE_ARRAY_INCREMENT;
         return code_bad;
      }
   }

   lig = &ms->ligament[ms->numligaments];

   lig->display = yes;
   lig->numlines = 0;
   lig->line_array_size = 10;
   lig->activation = 1.0;
   lig->drawmode = gouraud_shading;
   lig->show_normals = no;

   lig->line = (dpMusclePathStruct*)simm_malloc(lig->line_array_size*sizeof(dpMusclePathStruct));
   if (lig->line == NULL)
      return code_bad;

   if (fscanf(fp, "%s", buffer) != 1)
   {
      error(abort_action, "Error starting a ligament definition.");
      return code_bad;
   }
   else
   {
      mstrcpy(&lig->name, buffer);      
   }

   while (1)
   {
      if (read_string(fp, buffer) == EOF)
         break;

      if (STRINGS_ARE_EQUAL(buffer, "beginpoints"))
      {
         if (lig->numlines == lig->line_array_size)
         {
            lig->line_array_size += 10;
            lig->line = (dpMusclePathStruct*)simm_realloc(lig->line, lig->line_array_size*sizeof(dpMusclePathStruct), &rc);
            if (rc == code_bad)
            {
               lig->line_array_size -= 10;
               return code_bad;
            }
         }
         ligline = &lig->line[lig->numlines];
         if (read_muscle_path(ms, fp, ligline) == code_bad)
         {
            (void)sprintf(errorbuffer, "Error reading coordinates for muscle surface %s.", lig->name);
            error(abort_action, errorbuffer);
            return code_bad;
         }
         else
         {
            post_init_muscle_path(ligline, 0); //TODO5.0 once ligaments support wrap objects, change '0' to lig->numWrapStructs
         }
         lig->numlines++;
      }
      else if (STRINGS_ARE_EQUAL(buffer, "endmusclesurface") || STRINGS_ARE_EQUAL(buffer, "endligament"))
      {
         break;
      }
      else
      {
         (void)sprintf(errorbuffer, "Unrecognized string \"%s\" found while reading muscle surface %s.", buffer, lig->name);
         error(recover, errorbuffer);
      }
   }

   ms->numligaments++;

   return code_fine;
}


/* set the array of mp points in the musclepoints array struct */
ReturnCode post_init_muscle_path(dpMusclePathStruct* path, int numWrapStructs)
{
   if (path == NULL)
      return code_fine;

   path->num_points = 0;
   path->mp_array_size = path->num_orig_points + 2 * numWrapStructs;
   path->mp = (dpMusclePoint**)simm_malloc(path->mp_array_size * sizeof(dpMusclePoint*));
   if (path->mp == NULL)
   {
      error(none, "Ran out of memory.");
      return code_bad;
   }
   return code_fine;
}
