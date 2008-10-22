/*******************************************************************************

   SAVEMUSCLES.C

   Authors: Peter Loan

   Date: 8-DEC-89

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Description:

   Routines:
      save_all_muscles              : makes copies of all muscle structures for one model
      save_current_muscle           : makes a copy of the specified muscle structure
      save_default_muscle           : makes a copy of the default muscle structure
      restore_all_muscles           : copies saved muscle structures back to originals
      restore_current_muscle        : copies a saved muscle structure back to original
      restore_default_muscle        : copies the saved default muscle into the original
      save_all_musclepaths          : makes copies of all musclepath structures for one model
      restore_all_musclepaths       : copies all saved musclepath structures back to originals
      save_current_musclepath       : makes a copy of the muscle path of the current muscle
      restore_current_musclepath    : copies a saved musclepath back to original
      alloc_func                    : allocates memory for a spline-interpolated function
      nullify_muscle                : sets to NULL all pointer elements of muscle struct
      printmuscle                   : prints to stdout a muscle structure

*******************************************************************************/

#include "universal.h"
#include "globals.h"
#include "functions.h"


/*************** DEFINES (for this file only) *********************************/



/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/
static char save_error_msg1[] = "Not enough memory to save muscles.";
static char save_error_msg2[] = "Previously saved muscles deleted.";
static char save_error_msg3[] = "Save muscles to file for security.";
static char restore_error_msg1[] = "Not enough memory to restore muscles.";
static char restore_error_msg2[] = "Working copy of muscles deleted.";
static char restore_error_msg3[] = "Free-up some memory and try to restore again.";


/*************** EXTERNED VARIABLES (declared in another file) ****************/



/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static ReturnCode copy_muscle_from_saved(SaveMuscle* from, MuscleStruct* to, MuscleStruct* deffrom,
		      MuscleStruct* defto);
static ReturnCode copy_muscle_to_saved(MuscleStruct* from, SaveMuscle* to, MuscleStruct* current_dm,
		      MuscleStruct* saved_dm);
static ReturnCode copy_musclepath_to_saved(MusclePathStruct* from, SaveMusclePath* to, MuscleStruct *owner);
static ReturnCode copy_musclepath_from_saved(SaveMusclePath* from, MusclePathStruct* to);
static void nullify_savemusclepath(SaveMusclePath *mpa);
static void nullify_musclepath(MusclePathStruct *mpa);


/* SAVE_MUSCLES: copy all the muscles as well as the default muscle into the save structure.
 */
ReturnCode save_all_muscles(int mod)
{
   int i, j;

   /* first free the memory used by the previously-saved muscles */
   free_savedmuscles(model[mod]->save.muscle,&model[mod]->save.default_muscle,
              model[mod]->save.numsavedmuscs);
   model[mod]->save.muscle = NULL;
   free_defmusc(&model[mod]->save.default_muscle);

   /* copy the default muscle to the savemuscle structure */
   if (copy_default_muscle(&model[mod]->default_muscle, &model[mod]->save.default_muscle) == code_bad)
   {
      error(none,save_error_msg1);
      error(none,save_error_msg2);
      error(none,save_error_msg3);
      model[mod]->save.numsavedmuscs = 0;
      return code_bad;
   }

   /* malloc memory for all the muscle structures to be saved */
   model[mod]->save.numsavedmuscs = model[mod]->nummuscles;
   model[mod]->save.muscle = (SaveMuscle*)simm_malloc(model[mod]->save.numsavedmuscs*sizeof(SaveMuscle));
   if (model[mod]->save.muscle == NULL)
   {
      error(none,save_error_msg1);
      error(none,save_error_msg2);
      error(none,save_error_msg3);
      model[mod]->save.numsavedmuscs = 0;
      return code_bad;
   }

   /* copy the muscle structures to the savemuscle structure and store the location of the saved copy */
   for (i=0; i<model[mod]->nummuscles; i++)
   {
      if (copy_muscle_to_saved(&model[mod]->muscle[i],&model[mod]->save.muscle[i],
               &model[mod]->default_muscle, &model[mod]->save.default_muscle) == code_bad)
      {
         error(none,save_error_msg1);
         error(none,save_error_msg2);
         error(none,save_error_msg3);
         model[mod]->save.numsavedmuscs = 0;
         return code_bad;
      }
      model[mod]->muscle[i].saved_copy = &model[mod]->save.muscle[i];
   }
   return code_fine;
}

/* SAVE_CURRENT_MUSCLE: save a copy of the currently selected muscle into the list
 * of saved muscles.  If the muscle is already in the list, copy the information.
 * If the muscle is not already in the list, allocate space fot it and then copy it. 
 * If the muscle inherits any parameters from the default muscle, when copied into
 * the save structure, the inherited values come from the SAVED default muscle, not the
 * current one.
 */
ReturnCode save_current_muscle(int mod, MuscleStruct *muscle)
{
   SaveMuscle *saved = NULL;
   ReturnCode rc;

   if (muscle->saved_copy != NULL)  // a saved copy already exists
   {
      saved = muscle->saved_copy;
   }
   else     // add an entry for this muscle
   {
      int index = model[mod]->save.numsavedmuscs;
      model[mod]->save.numsavedmuscs++;
      model[mod]->save.muscle = (SaveMuscle*)simm_realloc(model[mod]->save.muscle, 
         model[mod]->save.numsavedmuscs*sizeof(SaveMuscle), &rc);
      if (rc != code_fine)
      {
         char buffer[256];
         sprintf(buffer, "Could not save %s", muscle->name);
         error(none, buffer);
         return code_bad;
      }
      saved = &model[mod]->save.muscle[index];
   }
   // freeing the saved muscle causes crash in copy_muscle_to_saved - dkb June 2008
   //free_savedmuscle(saved, &model[mod]->save.default_muscle);

   if (copy_muscle_to_saved(muscle, saved,
               &model[mod]->default_muscle, &model[mod]->save.default_muscle) == code_bad)
   {
      error(none,save_error_msg1);
      error(none,save_error_msg2);
      error(none,save_error_msg3);
      model[mod]->save.numsavedmuscs = 0;
      return code_bad;
   }

   /* store the pointer to the saved copy */
   muscle->saved_copy = saved;

   return code_fine;
}

/* SAVE_DEFAULT_MUSCLE: copy the default muscle into the save structure.  Check all
 * the muscles in the save structure.  If they inherit any parameters from the default,
 * make them inherit from the new saved default.
 */
ReturnCode save_default_muscle(int mod)
{
   int i, j;
   SBoolean *afl, *pfl, *tfl, *fv, *exc, *name, *Fmo, *pen, *ofl, *rtl, *mcv, *min_thick, *max_thick;
   SBoolean *min_mat, *max_mat, *musc_model, *exc_format, **dp;
   ReturnCode rc;
   
   afl = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   pfl = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   tfl = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   fv = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   exc = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   name = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   Fmo = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   pen = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   ofl = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   rtl = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   mcv = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   min_thick = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   max_thick = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   min_mat = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   max_mat = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   musc_model = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   exc_format = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   dp = (SBoolean **)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean *));
   for (i = 0; i < model[mod]->nummuscles; i++)
      dp[i] = (SBoolean *)simm_malloc(model[mod]->muscle[i].num_dynamic_params * sizeof(SBoolean));

   for (i = 0; i < model[mod]->save.numsavedmuscs; i++)
   {
      name[i] = Fmo[i] = pen[i] = ofl[i] = rtl[i] = mcv[i] = no;
      min_thick[i] = max_thick[i] = min_mat[i] = max_mat[i] = musc_model[i] = exc_format[i] = no;

      if (model[mod]->save.muscle[i].name == model[mod]->save.default_muscle.name)
         name[i] = yes;
      if (model[mod]->save.muscle[i].max_isometric_force == model[mod]->save.default_muscle.max_isometric_force)
         Fmo[i] = yes;
      if (model[mod]->save.muscle[i].pennation_angle == model[mod]->save.default_muscle.pennation_angle)
         pen[i] = yes;
      if (model[mod]->save.muscle[i].optimal_fiber_length == model[mod]->save.default_muscle.optimal_fiber_length)
         ofl[i] = yes;
      if (model[mod]->save.muscle[i].resting_tendon_length == model[mod]->save.default_muscle.resting_tendon_length)
         rtl[i] = yes;
      if (model[mod]->save.muscle[i].max_contraction_vel == model[mod]->save.default_muscle.max_contraction_vel)
         mcv[i] = yes;
      if (model[mod]->save.muscle[i].min_thickness == model[mod]->save.default_muscle.min_thickness)
         min_thick[i] = yes;
      if (model[mod]->save.muscle[i].max_thickness == model[mod]->save.default_muscle.max_thickness)
         max_thick[i] = yes;
      if (model[mod]->save.muscle[i].min_material == model[mod]->save.default_muscle.min_material)
         min_mat[i] = yes;
      if (model[mod]->save.muscle[i].max_material == model[mod]->save.default_muscle.max_material)
         max_mat[i] = yes;

      //dyn_params
      for (j = 0; j < model[mod]->save.muscle[i].num_dynamic_params; j++)
      {
         dp[i][j] = no;
         if (model[mod]->save.muscle[i].dynamic_params[j] == model[mod]->save.default_muscle.dynamic_params[j])
            dp[i][j] = yes;
      }
      
      // functions
      afl[i] = pfl[i] = tfl[i] = fv[i] = exc[i] = no;
      
      if (model[mod]->save.muscle[i].active_force_len_curve == model[mod]->save.default_muscle.active_force_len_curve)
         afl[i] = yes;
      if (model[mod]->save.muscle[i].passive_force_len_curve == model[mod]->save.default_muscle.passive_force_len_curve)
         pfl[i] = yes;
      if (model[mod]->save.muscle[i].tendon_force_len_curve == model[mod]->save.default_muscle.tendon_force_len_curve)
         tfl[i] = yes;
      if (model[mod]->save.muscle[i].force_vel_curve == model[mod]->save.default_muscle.force_vel_curve)
         fv[i] = yes;
      if (model[mod]->save.muscle[i].excitation == model[mod]->save.default_muscle.excitation)
         exc[i] = yes;
      if (model[mod]->save.muscle[i].excitation_format == model[mod]->save.default_muscle.excitation_format)
         exc_format[i] = yes;
      if (model[mod]->save.muscle[i].muscle_model_index == model[mod]->save.default_muscle.muscle_model_index)
         musc_model[i] = yes;

   }

   free_defmusc(&model[mod]->save.default_muscle);
   rc =  copy_default_muscle(&model[mod]->default_muscle, &model[mod]->save.default_muscle);
   if (rc == code_bad)
      return rc;

   for (i = 0; i < model[mod]->nummuscles; i++)
   {
      if (afl[i] == yes)
      {
         model[mod]->save.muscle[i].active_force_len_curve = NULL;
         model[mod]->save.muscle[i].active_force_len_curve = model[mod]->save.default_muscle.active_force_len_curve;
      }
      if (pfl[i] == yes)
      {
         model[mod]->save.muscle[i].passive_force_len_curve = NULL;
         model[mod]->save.muscle[i].passive_force_len_curve = model[mod]->save.default_muscle.passive_force_len_curve;
      }
      if (tfl[i] == yes)
      {
         model[mod]->save.muscle[i].tendon_force_len_curve = NULL;
         model[mod]->save.muscle[i].tendon_force_len_curve = model[mod]->save.default_muscle.tendon_force_len_curve;         
      }
      if (fv[i] == yes)
      {
         model[mod]->save.muscle[i].force_vel_curve = NULL;
         model[mod]->save.muscle[i].force_vel_curve = model[mod]->save.default_muscle.force_vel_curve;
      }
      if (exc[i] == yes)
      {
         model[mod]->save.muscle[i].excitation = NULL;
         model[mod]->save.muscle[i].excitation = model[mod]->save.default_muscle.excitation;
      }
      if (name[i] == yes)
         model[mod]->save.muscle[i].name = model[mod]->save.default_muscle.name;
      if (Fmo[i] == yes)
         model[mod]->save.muscle[i].max_isometric_force = model[mod]->save.default_muscle.max_isometric_force;
      if (pen[i] == yes)
         model[mod]->save.muscle[i].pennation_angle = model[mod]->save.default_muscle.pennation_angle;
      if (ofl[i] == yes)
         model[mod]->save.muscle[i].optimal_fiber_length =model[mod]->save.default_muscle.optimal_fiber_length;
      if (rtl[i] == yes)
         model[mod]->save.muscle[i].resting_tendon_length = model[mod]->save.default_muscle.resting_tendon_length;
      if (mcv[i] == yes)
         model[mod]->save.muscle[i].max_contraction_vel = model[mod]->save.default_muscle.max_contraction_vel;
      if (min_thick[i] == yes)
         model[mod]->save.muscle[i].min_thickness = model[mod]->save.default_muscle.min_thickness;
      if (max_thick[i] == yes)
         model[mod]->save.muscle[i].max_thickness = model[mod]->save.default_muscle.max_thickness;
      if (min_mat[i] == yes)
         model[mod]->save.muscle[i].min_material = model[mod]->save.default_muscle.min_material;
      if (max_mat[i] == yes)
         model[mod]->save.muscle[i].max_material = model[mod]->save.default_muscle.max_material;
      
      for (j = 0; j < model[mod]->save.muscle[i].num_dynamic_params; j++)
         if (dp[i][j] == yes)
            model[mod]->save.muscle[i].dynamic_params[j] = model[mod]->save.default_muscle.dynamic_params[j];
      if (musc_model[i] == yes)
         model[mod]->save.muscle[i].muscle_model_index = model[mod]->save.default_muscle.muscle_model_index;
      if (exc_format[i] == yes)
         model[mod]->save.muscle[i].excitation_format = model[mod]->save.default_muscle.excitation_format;
   }
   return code_fine;
}


/* RESTORE_MUSCLES: copy all the muscles as well as the default muscle from the save structure.
 */
ReturnCode restore_all_muscles(int mod)
{
   int i, j;

   if (model[mod]->save.muscle == NULL)
   {
      error(none,"There are no saved copies of this model\'s muscles.");
      return code_bad;
   }

   /* first free the memory used by the current muscles */
   free_muscles(model[mod]->muscle,&model[mod]->default_muscle,
              model[mod]->nummuscles);
   model[mod]->muscle = NULL;
   free_defmusc(&model[mod]->default_muscle);

   /* copy the saved default muscle to the current default muscle structure */
   if (copy_default_muscle(&model[mod]->save.default_muscle, &model[mod]->default_muscle) == code_bad)
   {
      error(none,restore_error_msg1);
      error(none,restore_error_msg2);
      error(none,restore_error_msg3);
      model[mod]->nummuscles = 0;
      return code_bad;
   }

   /* malloc memory for the to-be-restored muscles */
   model[mod]->nummuscles = model[mod]->save.numsavedmuscs;
   while (model[mod]->nummuscles > model[mod]->muscle_array_size)
      model[mod]->muscle_array_size += MUSCLE_ARRAY_INCREMENT;
   model[mod]->muscle = (MuscleStruct*)simm_malloc(model[mod]->muscle_array_size*sizeof(MuscleStruct));
   if (model[mod]->muscle == NULL)
   {
      error(none,restore_error_msg1);
      error(none,restore_error_msg2);
      error(none,restore_error_msg3);
      model[mod]->nummuscles = 0;
      return code_bad;
   }

   /* copy the saved muscles to the current muscle structure */
   for (i=0; i<model[mod]->save.numsavedmuscs; i++)
   {
      if (copy_muscle_from_saved(&model[mod]->save.muscle[i],&model[mod]->muscle[i],
         &model[mod]->save.default_muscle,&model[mod]->default_muscle) == code_bad)
      {
         error(none,restore_error_msg1);
         error(none,restore_error_msg2);
         error(none,restore_error_msg3);
         model[mod]->nummuscles = 0;
         return code_bad;
      }
   }

   return code_fine;
}





/* RESTORE_CURRENT_MUSCLE: restore the current muscle from its saved copy.
 * If the muscle is already in the list, copy the information.
 * If the saved muscle inherits any parameters from the saved default muscle, when copied into
 * the model structure, the inherited values come from the MODEL default muscle, not the
 * saved one.
 */
ReturnCode restore_current_muscle(int mod, MuscleStruct *muscle)
{
   int i, j;
   SaveMuscle *saved = NULL;

   if (model[mod]->nummuscles <= 0)
      return code_bad;

   if (model[mod]->save.muscle == NULL)
   {
      error(none,"There are no saved copies of this model\'s muscles.");
      return code_bad;
   }

   if (muscle->saved_copy == NULL)
   {
      char buffer[256];
      sprintf(buffer, "There is no saved copy of muscle %s.", muscle->name);
      error(none, buffer);
      return code_bad;
   }
   else
   {
      saved = muscle->saved_copy;
   }
   free_muscle(muscle, &model[mod]->default_muscle);

   if (copy_muscle_from_saved(saved, muscle,
      &model[mod]->save.default_muscle,&model[mod]->default_muscle) == code_bad)
   {
      error(none,"Could not restore current muscle");
      return code_bad;
   }

   return code_fine;
}


/* RESTORE_DEFAULT_MUSCLE: copy the default muscle from the save structure into the model.  Check all
 * the muscles in the model.  If they inherit any parameters from the default,
 * make them inherit from the new default.
 */
ReturnCode restore_default_muscle(int mod)
{
   int i, j;
   SBoolean *afl, *pfl, *tfl, *fv, *exc, *name, *Fmo, *pen, *ofl, *rtl, *mcv, *min_thick, *max_thick;
   SBoolean *min_mat, *max_mat, *musc_model, *exc_format, **dp;
   MuscleStruct *def, *saved;
   ReturnCode rc;

   def = &model[mod]->default_muscle;
   saved = &model[mod]->save.default_muscle;

   afl = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   pfl = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   tfl = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   fv = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   exc = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   name = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   Fmo = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   pen = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   ofl = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   rtl = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   mcv = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   min_thick = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   max_thick = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   min_mat = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   max_mat = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   musc_model = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   exc_format = (SBoolean *)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean));
   dp = (SBoolean **)simm_malloc(model[mod]->nummuscles * sizeof(SBoolean *));
   for (i = 0; i < model[mod]->nummuscles; i++)
      dp[i] = (SBoolean *)simm_malloc(model[mod]->muscle[i].num_dynamic_params * sizeof(SBoolean));
   for (i = 0; i < model[mod]->nummuscles; i++)
   {
      name[i] = Fmo[i] = pen[i] = ofl[i] = rtl[i] = mcv[i] = no;
      min_thick[i] = max_thick[i] = min_mat[i] = max_mat[i] = musc_model[i] = exc_format[i] = no;

      if (model[mod]->muscle[i].name == def->name)
         name[i] = yes;
      if (model[mod]->muscle[i].max_isometric_force == def->max_isometric_force)
         Fmo[i] = yes;
      if (model[mod]->muscle[i].pennation_angle == def->pennation_angle)
         pen[i] = yes;
      if (model[mod]->muscle[i].optimal_fiber_length == def->optimal_fiber_length)
         ofl[i] = yes;
      if (model[mod]->muscle[i].resting_tendon_length == def->resting_tendon_length)
         rtl[i] = yes;
      if (model[mod]->muscle[i].max_contraction_vel == def->max_contraction_vel)
         mcv[i] = yes;
      if (model[mod]->muscle[i].min_thickness == def->min_thickness)
         min_thick[i] = yes;
      if (model[mod]->muscle[i].max_thickness == def->max_thickness)
         max_thick[i] = yes;
      if (model[mod]->muscle[i].min_material == def->min_material)
         min_mat[i] = yes;
      if (model[mod]->muscle[i].max_material == def->max_material)
         max_mat[i] = yes;

      //dyn_params
      for (j = 0; j < model[mod]->muscle[i].num_dynamic_params; j++)
      {
         dp[i][j] = no;
         if (model[mod]->muscle[i].dynamic_params[j] == def->dynamic_params[j])
            dp[i][j] = yes;
      }
      
      // functions
      afl[i] = pfl[i] = tfl[i] = fv[i] = exc[i] = no;
      
      if (model[mod]->muscle[i].active_force_len_curve == def->active_force_len_curve)
         afl[i] = yes;
      if (model[mod]->muscle[i].passive_force_len_curve == def->passive_force_len_curve)
         pfl[i] = yes;
      if (model[mod]->muscle[i].tendon_force_len_curve == def->tendon_force_len_curve)
         tfl[i] = yes;
      if (model[mod]->muscle[i].force_vel_curve == def->force_vel_curve)
         fv[i] = yes;
      if (model[mod]->muscle[i].excitation == def->excitation)
         exc[i] = yes;
      if (model[mod]->muscle[i].excitation_format == def->excitation_format)
         exc_format[i] = yes;
      if (model[mod]->muscle[i].muscle_model_index == def->muscle_model_index)
         musc_model[i] = yes;

   }

   free_defmusc(&model[mod]->default_muscle);
   rc =  copy_default_muscle(&model[mod]->save.default_muscle, &model[mod]->default_muscle);
   if (rc == code_bad)
      return rc;

   for (i = 0; i < model[mod]->nummuscles; i++)
   {
      if (afl[i] == yes)
      {
         model[mod]->muscle[i].active_force_len_curve = NULL;
         model[mod]->muscle[i].active_force_len_curve = model[mod]->default_muscle.active_force_len_curve;
      }
      if (pfl[i] == yes)
      {
         model[mod]->muscle[i].passive_force_len_curve = NULL;
         model[mod]->muscle[i].passive_force_len_curve = model[mod]->default_muscle.passive_force_len_curve;
      }
      if (tfl[i] == yes)
      {
         model[mod]->muscle[i].tendon_force_len_curve = NULL;
         model[mod]->muscle[i].tendon_force_len_curve = model[mod]->default_muscle.tendon_force_len_curve;         
      }
      if (fv[i] == yes)
      {
         model[mod]->muscle[i].force_vel_curve = NULL;
         model[mod]->muscle[i].force_vel_curve = model[mod]->default_muscle.force_vel_curve;
      }
      if (exc[i] == yes)
      {
         model[mod]->muscle[i].excitation = NULL;
         model[mod]->muscle[i].excitation = model[mod]->default_muscle.excitation;
      }
      if (name[i] == yes)
         model[mod]->muscle[i].name = model[mod]->default_muscle.name;
      if (Fmo[i] == yes)
         model[mod]->muscle[i].max_isometric_force = model[mod]->default_muscle.max_isometric_force;
      if (pen[i] == yes)
         model[mod]->muscle[i].pennation_angle = model[mod]->default_muscle.pennation_angle;
      if (ofl[i] == yes)
         model[mod]->muscle[i].optimal_fiber_length =model[mod]->default_muscle.optimal_fiber_length;
      if (rtl[i] == yes)
         model[mod]->muscle[i].resting_tendon_length = model[mod]->default_muscle.resting_tendon_length;
      if (mcv[i] == yes)
         model[mod]->muscle[i].max_contraction_vel = model[mod]->default_muscle.max_contraction_vel;
      if (min_thick[i] == yes)
         model[mod]->muscle[i].min_thickness = model[mod]->default_muscle.min_thickness;
      if (max_thick[i] == yes)
         model[mod]->muscle[i].max_thickness = model[mod]->default_muscle.max_thickness;
      if (min_mat[i] == yes)
         model[mod]->muscle[i].min_material = model[mod]->default_muscle.min_material;
      if (max_mat[i] == yes)
         model[mod]->muscle[i].max_material = model[mod]->default_muscle.max_material;
      //dyn_params
      for (j = 0; j < model[mod]->muscle[i].num_dynamic_params; j++)
         if (dp[i][j] == yes)
            model[mod]->muscle[i].dynamic_params[j] = model[mod]->default_muscle.dynamic_params[j];
      if (musc_model[i] == yes)
         model[mod]->muscle[i].muscle_model_index = model[mod]->default_muscle.muscle_model_index;
      if (exc_format[i] == yes)
         model[mod]->muscle[i].excitation_format = model[mod]->default_muscle.excitation_format;
   }
   return code_fine;
}


// ---------------------------------  MUSCLE POINTS --------------------------------------------------------
/* SAVE_ALL_MUSCLEPOINTS: */
ReturnCode save_all_musclepaths(int mod)
{
   int i, j;

   if (model[mod]->nummuscles <= 0)
      return code_fine;

   /* first free the memory used by the previously-saved musclepoints */
   free_savedmusclepaths(model[mod]->save.musclepath, model[mod]->save.numsavedpaths);
   model[mod]->save.musclepath = NULL;

   /* malloc memory for all the muscle structures to be saved */
   model[mod]->save.numsavedpaths = model[mod]->nummuscles;
   model[mod]->save.musclepath =
         (SaveMusclePath*)simm_malloc(model[mod]->save.numsavedpaths*sizeof(SaveMusclePath));

   if (model[mod]->save.musclepath == NULL)
   {
      error(none,  "Could not save musclepoints.");
      model[mod]->save.numsavedpaths = 0;
      return code_bad;
   }

   /* copy the musclepoint arrays to the save structure */
   for (i=0; i<model[mod]->nummuscles; i++)
   {
      if (model[mod]->muscle[i].musclepoints->num_orig_points <= 0)
         continue;
      if (copy_musclepath_to_saved(model[mod]->muscle[i].musclepoints,&model[mod]->save.musclepath[i], &model[mod]->muscle[i]) == code_bad)
      {
         error(none,save_error_msg1);
         error(none,save_error_msg2);
         error(none,save_error_msg3);
         model[mod]->save.numsavedpaths = 0;
         return code_bad;
      }
      // save the pointer to the owner muscle
      //model[mod]->save.musclepath[i].owner = &model[mod]->muscle[i];
      //save the location of the stored copy
      model[mod]->muscle[i].musclepoints->saved_copy = &model[mod]->save.musclepath[i];

   /* Set all the refpt variables back to their original indices. If the muscles
    * are restored later, each muscle point in the model structure will point
    * back to the saved copy so that the "netmove" distance can be computed
    * correctly for each point. Points that the user added had a refpt of -1,
    * meaning that there was no corresponding saved point to do a netmove from.
    * But once the muscles are saved, that previously new point now has a copy
    * of itself in the saved-muscle structure.
    */
//      for (j=0; j<*(model[mod]->muscle[i].num_orig_points); j++)
//         model[mod]->muscle[i].musclepoints->mp_orig[j].refpt = j;
   }

   return code_fine;
}



/* RESTORE_ALL_MUSCLEPOINTS: */

ReturnCode restore_all_musclepaths(int mod)
{
   int i;
   MuscleStruct *owner = NULL;

   if (model[mod]->nummuscles <= 0)
      return code_bad;

   if (model[mod]->save.musclepath == NULL)
   {
      error(none,"There are no saved copies of this model\'s musclepoints.");
      return code_bad;
   }

   // ?? first free the memory used by the current paths - how to do if not an array

   /* copy the saved paths into the current muscles if they exist */
   for (i=0; i<model[mod]->save.numsavedpaths; i++)
   {
      owner = model[mod]->save.musclepath[i].owner;
      if (owner == NULL)
      {
        // error(none, "THe owner of the path you are trying to restore no longer exists.\n");
         continue;
//         return code_bad;
      }
      if (copy_musclepath_from_saved(&model[mod]->save.musclepath[i], owner->musclepoints) == code_bad)
      {
         error(none,restore_error_msg1);
         error(none,restore_error_msg2);
         error(none,restore_error_msg3);
         return code_bad;
      }
      // allocate mp[] 
      init_mparray(owner);
      /* Invalidate the wrapping calculations because a joint may have moved
      * since the muscle was last saved.
      */
      owner->wrap_calced = no;
   }

   return code_fine;
}


/* SAVE_CURRENT_MUSCLEPOINTS: save a copy of the currently selected muscle path into the list
 * of saved paths.  If the muscle path is already in the list, copy the information.
 * If the muscle is not already in the list, allocate space and then copy. */
ReturnCode save_current_musclepath(int mod, MuscleStruct *muscle)
{
   int i, j;
   SaveMusclePath *saved = NULL;
   ReturnCode rc;

   if (model[mod]->save.musclepath == NULL)
      return code_bad;

   // see if a saved copy already exists

   if (muscle->musclepoints->saved_copy != NULL)  // a saved copy already exists
   {
      saved = muscle->musclepoints->saved_copy;
   }
   else     // add an entry for this muscle
   {
      int index = model[mod]->save.numsavedpaths;
      model[mod]->save.numsavedpaths++;
      model[mod]->save.musclepath = (SaveMusclePath*)simm_realloc(model[mod]->save.musclepath, 
         model[mod]->save.numsavedpaths*sizeof(SaveMusclePath), &rc);
      if (rc != code_fine)
      {
         char buffer[256];
         sprintf(buffer, "Could not save path for %s.", muscle->name);
         error(none, buffer);
         return code_bad;
      }
      saved = &model[mod]->save.musclepath[index];
   }

   free_savedmusclepath(saved);

   if (copy_musclepath_to_saved(muscle->musclepoints, saved, muscle) == code_bad)
   {
      error(none,save_error_msg1);
      error(none,save_error_msg2);
      error(none,save_error_msg3);
      model[mod]->save.numsavedmuscs = 0;
      return code_bad;
   }

   /* store the pointer to the saved copy */
   muscle->musclepoints->saved_copy = saved;

   return code_fine;

   /* Set all the refpt variables back to their original indices. If the muscles
    * are restored later, each muscle point in the model structure will point
    * back to the saved copy so that the "netmove" distance can be computed
    * correctly for each point. Points that the user added had a refpt of -1,
    * meaning that there was no corresponding saved point to do a netmove from.
    * But once the muscles are saved, that previously new point now has a copy
    * of itself in the saved-muscle structure.
    */
//      for (j=0; j<*(model[mod]->muscle[i].num_orig_points); j++)
//         model[mod]->muscle[i].musclepoints->mp_orig[j].refpt = j;
   

   return code_fine;
}



/* RESTORE_MUSCLES: */

ReturnCode restore_current_musclepath(int mod, MuscleStruct *muscle)
{
   int i, j;
   SaveMusclePath *saved = NULL;

   if (model[mod]->nummuscles <= 0)
      return code_bad;

   if (model[mod]->save.musclepath == NULL)
   {
      error(none,"There are no saved copies of this model\'s musclepaths.");
      return code_bad;
   }

   if (muscle->musclepoints->saved_copy == NULL)
   {
      char buffer[256];
      sprintf(buffer, "There is no saved copy of the musclepath for %s.", muscle->name);
      error(none, buffer);
      return code_bad;
   }
   else
   {
      saved = muscle->musclepoints->saved_copy;
   }

   free_musclepath(muscle->musclepoints);

   if (copy_musclepath_from_saved(saved, muscle->musclepoints) == code_bad)
   {
      error(none,"Could not restore musclepath for current muscle");
      return code_bad;
   }

   init_mparray(muscle);
      /* Invalidate the wrapping calculations because a joint may have moved
      * since the muscle was last saved.
      */
   muscle->wrap_calced = no;
   return code_fine;

}






/* COPY_MUSCLE FROM SAVED: copy a muscle from a saved muscle structure.  If the muscle inherits a 
 * parameter from the saved default muscle, when restored, it inherits the value from the
 * current default muscle.  The wrapping is invalidated so it can be recalculated later.
 * Store the pointer to the saved copy.
 * Musclepoints ???.  
 */
static ReturnCode copy_muscle_from_saved(SaveMuscle* from, MuscleStruct* to, MuscleStruct* saved_dm, MuscleStruct* current_dm)
{
   int i, j, mem_size;

   nullify_muscle(to);

   if (from->name == saved_dm->name)
      to->name = current_dm->name;
   else if (mstrcpy(&to->name,from->name) == code_bad)
      return code_bad;

   to->display = from->display;
   to->wrap_calced = no;
   to->output = from->output;
   to->musclepoints = from->musclepoints;   

   to->numWrapStructs = from->numWrapStructs;
   if (to->numWrapStructs > 0)
   {
      if ((to->wrapStruct = (MuscleWrapStruct**)simm_malloc(to->numWrapStructs * sizeof(MuscleWrapStruct*))) == NULL)
         return code_bad;
      for (i = 0; i < to->numWrapStructs; i++)
      {
         if ((to->wrapStruct[i] = (MuscleWrapStruct*)simm_malloc(sizeof(MuscleWrapStruct))) == NULL)
            return code_bad;
         memcpy(to->wrapStruct[i], from->wrapStruct[i], sizeof(MuscleWrapStruct));
         for (j = 0; j < 2; j++)
         {
            if (from->wrapStruct[i]->mp_wrap[j].num_wrap_pts > 0)
            {
               to->wrapStruct[i]->mp_wrap[j].num_wrap_pts = from->wrapStruct[i]->mp_wrap[j].num_wrap_pts;
               mem_size = to->wrapStruct[i]->mp_wrap[j].num_wrap_pts * 3 * sizeof(double);
               to->wrapStruct[i]->mp_wrap[j].wrap_pts = (double*)simm_malloc(mem_size);
               memcpy(to->wrapStruct[i]->mp_wrap[j].wrap_pts, from->wrapStruct[i]->mp_wrap[j].wrap_pts, mem_size);
            }
            else
            {
               to->wrapStruct[i]->mp_wrap[j].num_wrap_pts = 0;
               to->wrapStruct[i]->mp_wrap[j].wrap_pts = NULL;
            }
            to->wrapStruct[i]->mp_wrap[j].isVia = from->wrapStruct[i]->mp_wrap[j].isVia;
            to->wrapStruct[i]->mp_wrap[j].viaRange = from->wrapStruct[i]->mp_wrap[j].viaRange;
         }
      }
   }

   to->numgroups = from->numgroups;
   if (to->numgroups == 0)
      to->group = NULL;
   else
   {
      if ((to->group = (int*)simm_malloc(to->numgroups*sizeof(int))) == NULL)
         return code_bad;
      for (i=0; i<to->numgroups; i++)
         to->group[i] = from->group[i];
   }

   if (copy_nddouble(from->max_isometric_force,&to->max_isometric_force,saved_dm->max_isometric_force,current_dm->max_isometric_force) == code_bad)
      return code_bad;
   if (copy_nddouble(from->pennation_angle,&to->pennation_angle, saved_dm->pennation_angle,current_dm->pennation_angle) == code_bad)
      return code_bad;
   if (copy_nddouble(from->min_thickness,&to->min_thickness, saved_dm->min_thickness,current_dm->min_thickness) == code_bad)
      return code_bad;
   if (copy_nddouble(from->max_thickness,&to->max_thickness, saved_dm->max_thickness,current_dm->max_thickness) == code_bad)
      return code_bad;
   if (copy_nddouble(from->optimal_fiber_length,&to->optimal_fiber_length, saved_dm->optimal_fiber_length,current_dm->optimal_fiber_length) == code_bad)
      return code_bad;
   if (copy_nddouble(from->resting_tendon_length,&to->resting_tendon_length, saved_dm->resting_tendon_length,current_dm->resting_tendon_length) == code_bad)
      return code_bad;
   if (copy_nddouble(from->max_contraction_vel,&to->max_contraction_vel, saved_dm->max_contraction_vel,current_dm->max_contraction_vel) == code_bad)
      return code_bad;

   if (copy_ndint(from->min_material, &to->min_material, saved_dm->min_material, current_dm->min_material) == code_bad)
      return code_bad;
   if (copy_ndint(from->max_material, &to->max_material, saved_dm->max_material, current_dm->max_material) == code_bad)
      return code_bad;
   if (copy_ndint(from->muscle_model_index,&to->muscle_model_index, saved_dm->muscle_model_index,current_dm->muscle_model_index) == code_bad)
      return code_bad;

   to->activation = from->activation;
   to->initial_activation = from->initial_activation;

   to->num_dynamic_params = from->num_dynamic_params;
   if (to->num_dynamic_params > 0)
   {
      to->dynamic_param_names = from->dynamic_param_names;
      to->dynamic_params = (double**)simm_malloc(to->num_dynamic_params * sizeof(double*));

      if (to->dynamic_params == NULL)
         return code_bad;

      for (i = 0; i < to->num_dynamic_params; i++)
      {
         if (copy_nddouble(from->dynamic_params[i], &to->dynamic_params[i],
            saved_dm->dynamic_params[i], current_dm->dynamic_params[i]) == code_bad)
            return code_bad;
      }
   }
   else
   {
      to->dynamic_param_names = NULL;
      to->dynamic_params = NULL;
   }

   to->nummomentarms = from->nummomentarms;
   if ((to->momentarms = (double*)simm_malloc(to->nummomentarms*sizeof(double))) == NULL)
      return code_bad;
   for (i=0; i<to->nummomentarms; i++)
      to->momentarms[i] = from->momentarms[i];

   if (from->active_force_len_curve == saved_dm->active_force_len_curve)
      to->active_force_len_curve = current_dm->active_force_len_curve;
   else
   {
      if (alloc_func(&to->active_force_len_curve,from->active_force_len_curve->coefficient_array_size) == code_bad)
         return code_bad;
      copy_function(from->active_force_len_curve,to->active_force_len_curve);
   }

   if (from->passive_force_len_curve == saved_dm->passive_force_len_curve)
      to->passive_force_len_curve = current_dm->passive_force_len_curve;
   else
   {
      if (alloc_func(&to->passive_force_len_curve,from->passive_force_len_curve->coefficient_array_size) == code_bad)
         return code_bad;
      copy_function(from->passive_force_len_curve,to->passive_force_len_curve);
   }

   if (from->tendon_force_len_curve == saved_dm->tendon_force_len_curve)
      to->tendon_force_len_curve = current_dm->tendon_force_len_curve;
   else
   {
      if (alloc_func(&to->tendon_force_len_curve,from->tendon_force_len_curve->coefficient_array_size) == code_bad)
         return code_bad;
      copy_function(from->tendon_force_len_curve,to->tendon_force_len_curve);
   }

   if (from->force_vel_curve == saved_dm->force_vel_curve)
      to->force_vel_curve = current_dm->force_vel_curve;
   else
   {
      if (alloc_func(&to->force_vel_curve,from->force_vel_curve->coefficient_array_size) == code_bad)
         return code_bad;
      copy_function(from->force_vel_curve,to->force_vel_curve);
   }

   to->excitation_index = from->excitation_index;
   to->excitation_abscissa = from->excitation_abscissa;
   if (from->excitation == saved_dm->excitation)
      to->excitation = current_dm->excitation;
   else
   {
      if (from->excitation == NULL)
         to->excitation = NULL;
      else
      {
         if (alloc_func(&to->excitation,from->excitation->coefficient_array_size) == code_bad)
            return code_bad;
         copy_function(from->excitation,to->excitation);
      }
   }
   if (from->excitation_format == saved_dm->excitation_format)
      to->excitation_format = current_dm->excitation_format;
   else
   {
      if (from->excitation_format == NULL)
         to->excitation_format = NULL;
      else
      {
         if ((to->excitation_format = (SplineType*)simm_malloc(sizeof(SplineType))) == NULL)
            return code_bad;
         *(to->excitation_format) = *(from->excitation_format);
      }
   }

   // store the location of the saved copy
   to->saved_copy = from;

   return code_fine;
}

/* COPY_MUSCLE TO SAVED: copy a muscle into a saved muscle structure.  If the muscle inherits a 
 * parameter from the current default muscle, when saved, it inherits the value from the
 * saved default muscle.  The wrapping is invalidated so it can be recalculated later.
 * Musclepoints ???.  
 */
static ReturnCode copy_muscle_to_saved(MuscleStruct* from, SaveMuscle* to, MuscleStruct* current_dm,
                            MuscleStruct* saved_dm)
{
   int i, j, mem_size;

   nullify_savemuscle(to);

   if (from->name == current_dm->name)
      to->name = saved_dm->name;
   else if (mstrcpy(&to->name,from->name) == code_bad)
      return code_bad;

   to->display = from->display;
   to->musclepoints = from->musclepoints;
   to->wrap_calced = no;
   to->output = from->output;

   to->numWrapStructs = from->numWrapStructs;
   if (to->numWrapStructs > 0)
   {
      if ((to->wrapStruct = (MuscleWrapStruct**)simm_malloc(to->numWrapStructs * sizeof(MuscleWrapStruct*))) == NULL)
         return code_bad;
      for (i = 0; i < to->numWrapStructs; i++)
      {
         if ((to->wrapStruct[i] = (MuscleWrapStruct*)simm_malloc(sizeof(MuscleWrapStruct))) == NULL)
            return code_bad;
         memcpy(to->wrapStruct[i], from->wrapStruct[i], sizeof(MuscleWrapStruct));
         for (j = 0; j < 2; j++)
         {
            if (from->wrapStruct[i]->mp_wrap[j].num_wrap_pts > 0)
            {
               to->wrapStruct[i]->mp_wrap[j].num_wrap_pts = from->wrapStruct[i]->mp_wrap[j].num_wrap_pts;
               mem_size = to->wrapStruct[i]->mp_wrap[j].num_wrap_pts * 3 * sizeof(double);
               to->wrapStruct[i]->mp_wrap[j].wrap_pts = (double*)simm_malloc(mem_size);
               memcpy(to->wrapStruct[i]->mp_wrap[j].wrap_pts, from->wrapStruct[i]->mp_wrap[j].wrap_pts, mem_size);
            }
            else
            {
               to->wrapStruct[i]->mp_wrap[j].num_wrap_pts = 0;
               to->wrapStruct[i]->mp_wrap[j].wrap_pts = NULL;
            }
            to->wrapStruct[i]->mp_wrap[j].isVia = from->wrapStruct[i]->mp_wrap[j].isVia;
            to->wrapStruct[i]->mp_wrap[j].viaRange = from->wrapStruct[i]->mp_wrap[j].viaRange;
         }
      }
   }

   to->numgroups = from->numgroups;
   if (to->numgroups == 0)
      to->group = NULL;
   else
   {
      if ((to->group = (int*)simm_malloc(to->numgroups*sizeof(int))) == NULL)
         return code_bad;
      for (i=0; i<to->numgroups; i++)
         to->group[i] = from->group[i];
   }

   if (copy_nddouble(from->max_isometric_force,&to->max_isometric_force, current_dm->max_isometric_force,saved_dm->max_isometric_force) == code_bad)
      return code_bad;
   if (copy_nddouble(from->pennation_angle,&to->pennation_angle, current_dm->pennation_angle,saved_dm->pennation_angle) == code_bad)
      return code_bad;
   if (copy_nddouble(from->min_thickness,&to->min_thickness, current_dm->min_thickness,saved_dm->min_thickness) == code_bad)
      return code_bad;
   if (copy_nddouble(from->max_thickness,&to->max_thickness, current_dm->max_thickness,saved_dm->max_thickness) == code_bad)
      return code_bad;
   if (copy_nddouble(from->optimal_fiber_length,&to->optimal_fiber_length, current_dm->optimal_fiber_length,saved_dm->optimal_fiber_length) == code_bad)
      return code_bad;
   if (copy_nddouble(from->resting_tendon_length,&to->resting_tendon_length, current_dm->resting_tendon_length,saved_dm->resting_tendon_length) == code_bad)
      return code_bad;
   if (copy_nddouble(from->max_contraction_vel,&to->max_contraction_vel, current_dm->max_contraction_vel,saved_dm->max_contraction_vel) == code_bad)
      return code_bad;

   if (copy_ndint(from->min_material, &to->min_material, current_dm->min_material, saved_dm->min_material) == code_bad)
      return code_bad;
   if (copy_ndint(from->max_material, &to->max_material, current_dm->max_material, saved_dm->max_material) == code_bad)
      return code_bad;
   if (copy_ndint(from->muscle_model_index,&to->muscle_model_index, current_dm->muscle_model_index,saved_dm->muscle_model_index) == code_bad)
      return code_bad;

   to->activation = from->activation;
   to->initial_activation = from->initial_activation;

   to->num_dynamic_params = from->num_dynamic_params;
   if (to->num_dynamic_params > 0)
   {
      to->dynamic_param_names = from->dynamic_param_names;
      to->dynamic_params = (double**)simm_malloc(to->num_dynamic_params * sizeof(double*));
      if (to->dynamic_params == NULL)
         return code_bad;
      for (i = 0; i < to->num_dynamic_params; i++)
      {
         if (copy_nddouble(from->dynamic_params[i], &to->dynamic_params[i],
                           current_dm->dynamic_params[i], saved_dm->dynamic_params[i]) == code_bad)
            return code_bad;
      }
   }
   else
   {
      to->dynamic_param_names = NULL;
      to->dynamic_params = NULL;
   }

   to->nummomentarms = from->nummomentarms;
   if ((to->momentarms = (double*)simm_malloc(to->nummomentarms*sizeof(double))) == NULL)
      return code_bad;
   for (i=0; i<to->nummomentarms; i++)
      to->momentarms[i] = from->momentarms[i];

   if (from->active_force_len_curve == current_dm->active_force_len_curve)
      to->active_force_len_curve = saved_dm->active_force_len_curve;
   else
   {
      if (alloc_func(&to->active_force_len_curve,from->active_force_len_curve->coefficient_array_size) == code_bad)
         return code_bad;
      copy_function(from->active_force_len_curve,to->active_force_len_curve);
   }

   if (from->passive_force_len_curve == current_dm->passive_force_len_curve)
      to->passive_force_len_curve = saved_dm->passive_force_len_curve;
   else
   {
      if (alloc_func(&to->passive_force_len_curve,from->passive_force_len_curve->coefficient_array_size) == code_bad)
         return code_bad;
      copy_function(from->passive_force_len_curve,to->passive_force_len_curve);
   }

   if (from->tendon_force_len_curve == current_dm->tendon_force_len_curve)
      to->tendon_force_len_curve = saved_dm->tendon_force_len_curve;
   else
   {
      if (alloc_func(&to->tendon_force_len_curve,from->tendon_force_len_curve->coefficient_array_size) == code_bad)
         return code_bad;
      copy_function(from->tendon_force_len_curve,to->tendon_force_len_curve);
   }

   if (from->force_vel_curve == current_dm->force_vel_curve)
      to->force_vel_curve = saved_dm->force_vel_curve;
   else
   {
      if (alloc_func(&to->force_vel_curve,from->force_vel_curve->coefficient_array_size) == code_bad)
         return code_bad;
      copy_function(from->force_vel_curve,to->force_vel_curve);
   }

   to->excitation_index = from->excitation_index;
   to->excitation_abscissa = from->excitation_abscissa;
   if (from->excitation == current_dm->excitation)
      to->excitation = saved_dm->excitation;
   else
   {
      if (from->excitation == NULL)
         to->excitation = NULL;
      else
      {
         if (alloc_func(&to->excitation,from->excitation->coefficient_array_size) == code_bad)
            return code_bad;
         copy_function(from->excitation,to->excitation);
      }
   }
   if (from->excitation_format == current_dm->excitation_format)
      to->excitation_format = saved_dm->excitation_format;
   else
   {
      if (from->excitation_format == NULL)
         to->excitation_format = NULL;
      else
      {
         if ((to->excitation_format = (SplineType*)simm_malloc(sizeof(SplineType))) == NULL)
            return code_bad;
         *(to->excitation_format) = *(from->excitation_format);
      }
   }

   return code_fine;
}







/* SAVE_MUSCLE_GROUPS: */

void save_muscle_groups(int mod)
{

   int i, j;

   if (model[mod]->numgroups <= 0)
      return;

   /* first free the memory used by the previously saved groups */
   for (i = 0; i < model[mod]->save.numsavedmuscgroups; i++)
   {
      FREE_IFNOTNULL(model[mod]->save.muscgroup[i].muscle_index);
   }
   model[mod]->save.muscgroup = NULL;
   model[mod]->save.numsavedmuscgroups = model[mod]->numgroups;

   /* malloc memory for all the muscle structures to be saved */
   model[mod]->save.muscgroup =
         (MuscleGroup*)simm_malloc(model[mod]->save.numsavedmuscgroups*sizeof(MuscleGroup));

   if (model[mod]->save.muscgroup == NULL)
   {
      error(none,"Could not save muscle groups.");
      model[mod]->save.numsavedmuscgroups = 0;
      return;
   }

   /* copy the muscle groups to the save structure */
   for (i=0; i<model[mod]->numgroups; i++)
   {
      model[mod]->save.muscgroup[i].muscindex_array_size = MUSCINDEX_ARRAY_INCREMENT;
      model[mod]->save.muscgroup[i].number_of_muscles = model[mod]->muscgroup[i].number_of_muscles;
      while (model[mod]->save.muscgroup[i].number_of_muscles >= model[mod]->save.muscgroup[i].muscindex_array_size)
         model[mod]->save.muscgroup[i].muscindex_array_size += MUSCINDEX_ARRAY_INCREMENT;
      model[mod]->save.muscgroup[i].muscle_index = (int *)simm_malloc(model[mod]->save.muscgroup[i].muscindex_array_size * sizeof(int));
      for (j = 0; j < model[mod]->save.muscgroup[i].number_of_muscles; j++)
         model[mod]->save.muscgroup[i].muscle_index[j] = model[mod]->muscgroup[i].muscle_index[j];
      //mstrcpy(&model[mod]->save.muscgroup[i].name, model[mod]->muscgroup[i].name);
      //model[mod]->save.muscgroup[i].number_of_columns = model[mod]->muscgroup[i].number_of_muscles;
      //model[mod]->save.muscgroup[i].column_width = model[mod]->muscgroup[i].number_of_muscles;
      
   }
}

/* RESTORE_MUSCLE_GROUPS: */

ReturnCode restore_muscle_groups(int mod)
{
   ReturnCode rc;
   int i, j;

   if (model[mod]->save.numsavedmuscgroups <= 0)
      return code_bad;

   if (model[mod]->save.muscgroup == NULL)
   {
      error(none, "No saved copy of muscle groups exists.");
      model[mod]->numgroups = 0;
      return code_bad;
   }

   /* copy the muscle groups to the save structure */
   for (i=0; i<model[mod]->numgroups; i++)
   {
      model[mod]->muscgroup[i].number_of_muscles = model[mod]->save.muscgroup[i].number_of_muscles;
      if (model[mod]->muscgroup[i].number_of_muscles >= model[mod]->muscgroup[i].muscindex_array_size)
      {
         j = 0;
         while (model[mod]->muscgroup[i].number_of_muscles >= model[mod]->muscgroup[i].muscindex_array_size )
         {
            j++;
         model[mod]->muscgroup[i].muscindex_array_size += MUSCINDEX_ARRAY_INCREMENT;
         }

         model[mod]->muscgroup[i].muscle_index = (int*)simm_realloc(model[mod]->muscgroup[i].muscle_index,
            model[mod]->muscgroup[i].muscindex_array_size*sizeof(int),&rc);
         if (rc == code_bad)
         {
            model[mod]->muscgroup[i].muscindex_array_size -= j * MUSCINDEX_ARRAY_INCREMENT;
            return code_bad;
         }
      }

      for (j = 0; j < model[mod]->muscgroup[i].number_of_muscles; j++)
         model[mod]->muscgroup[i].muscle_index[j] = model[mod]->save.muscgroup[i].muscle_index[j];
   }
   return code_fine;
}


/* PRINTMUSCLE: */

void printmuscle(MuscleStruct* musc)
{

   int i;

   if (musc->name == NULL)
      printf("name: ptr is null\n");
   else
      printf("name: %s\n", musc->name);

   printf("display: %d\n", musc->display);

   printf("numpoints: %d\n", musc->musclepoints->num_orig_points);
   for (i=0; i<musc->musclepoints->num_orig_points; i++)
      printf("(%d) %lf %lf %lf segment %d\n", i,
      musc->musclepoints->mp_orig[i].point[0],
      musc->musclepoints->mp_orig[i].point[1],
      musc->musclepoints->mp_orig[i].point[2],
      musc->musclepoints->mp_orig[i].segment);

   if (musc->max_isometric_force == NULL)
      printf("max_isometric_force: ptr is null\n");
   else
      printf("max_isometric_force: %lf\n", *musc->max_isometric_force);

   printf("\n");
   (void)fflush(stdout);

}



/* COPY_MP_TO_SAVED: copy the muscle's mp_orig array to the save structure
 * store a copy of the owner muscle in the save structure
 */
static ReturnCode copy_musclepath_to_saved(MusclePathStruct *from, SaveMusclePath* to, MuscleStruct *owner)
{
   int i;

   nullify_savemusclepath(to);

   // store the original array size
   to->mp_orig_array_size = from->mp_orig_array_size;

   // store the number of points
   to->num_orig_points = from->num_orig_points;

   // store the points
   to->mp_orig = (MusclePoint*)simm_malloc(to->mp_orig_array_size*sizeof(MusclePoint));
   if (to->mp_orig == NULL)
      return code_bad;
   for (i=0; i<to->num_orig_points; i++)
      if (copy_musclepoint(&from->mp_orig[i],&to->mp_orig[i]) == code_bad)
         return code_bad;

   to->owner = owner;

   return code_fine;

}

/* COPY_MP_FROM_SAVED: copy the muscle's mp_orig array to the save structure
 * store a copy of the owner muscle in the save structure
 */
static ReturnCode copy_musclepath_from_saved(SaveMusclePath* from, MusclePathStruct *to )
{
   int i;

   nullify_musclepath(to);

   // store the original array size
   to->mp_orig_array_size = from->mp_orig_array_size;

   // store the number of points
   to->num_orig_points = from->num_orig_points;

   // restore the points - 
   to->mp_orig = (MusclePoint*)simm_malloc((to->mp_orig_array_size)*sizeof(MusclePoint));
   if (to->mp_orig == NULL)
      return code_bad;
   for (i=0; i<to->num_orig_points; i++)
      if (copy_musclepoint(&from->mp_orig[i],&to->mp_orig[i]) == code_bad)
         return code_bad;

      // dkb apr 2008 - points no longer restored by muscle editor, use musclepoint editor
  // to->mp_array_size = *(to->num_orig_points) + (to->numWrapStructs * 2);
  // to->mp = (MusclePoint**)simm_malloc(sizeof(MusclePoint*) * to->mp_array_size);
  // to->num_points = 0;
   to->saved_copy = from;

   return code_fine;

}

static void nullify_savemusclepath(SaveMusclePath *mpa)
{
   mpa->mp_orig_array_size = 0;
   mpa->num_orig_points = 0;
   mpa->mp_orig = NULL;
   mpa->owner = NULL;
   mpa->temp_index = -1;
}

static void nullify_musclepath(MusclePathStruct *mpa)
{
   mpa->mp_orig_array_size = 0;
   mpa->num_orig_points = 0;
   mpa->mp_orig = NULL;
   mpa->mp = NULL;
   mpa->mp_array_size = 0;
   mpa->num_points = 0;
   mpa->saved_copy = NULL;
}
