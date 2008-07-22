/*******************************************************************************

   SAVEMUSCLES.C

   Authors: Peter Loan

   Date: 8-DEC-89

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Description:

   Routines:
      save_muscles        : makes copies of all muscle structures for one model
      restore_muscles     : copies saved muscle structures back to originals
      copy_musc           : copies an individual muscle structure
      copy_default_muscle : copies one default muscle structure to another
      alloc_func          : allocates memory for a spline-interpolated function
      nullify_muscle      : sets to NULL all pointer elements of muscle struct
      printmuscle         : prints to stdout a muscle structure

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
static ReturnCode copy_nondefault_dyn_params(MuscleStruct* from, MuscleStruct* to,
					     MuscleStruct* deffrom, MuscleStruct* defto);
static ReturnCode copy_nonnull_dyn_params(MuscleStruct* from, MuscleStruct* to);



/* SAVE_MUSCLES: */

void save_muscles(int mod)
{

   int i, j;

   if (model[mod]->nummuscles <= 0)
      return;

   /* first free the memory used by the previously-saved muscles */
   free_muscs(model[mod]->save.muscle,&model[mod]->save.default_muscle,
              model[mod]->save.numsavedmuscs);
   model[mod]->save.muscle = NULL;
   free_defmusc(&model[mod]->save.default_muscle);
   model[mod]->save.numsavedmuscs = model[mod]->nummuscles;

   /* copy the default muscle to the savemuscle structure */
   if (copy_default_muscle(&model[mod]->default_muscle,
			  &model[mod]->save.default_muscle) == code_bad)
   {
      error(none,save_error_msg1);
      error(none,save_error_msg2);
      error(none,save_error_msg3);
      model[mod]->save.numsavedmuscs = 0;
      return;
   }

   /* malloc memory for all the muscle structures to be saved */
   model[mod]->save.muscle =
         (MuscleStruct*)simm_malloc(model[mod]->save.numsavedmuscs*sizeof(MuscleStruct));

   if (model[mod]->save.muscle == NULL)
   {
      error(none,save_error_msg1);
      error(none,save_error_msg2);
      error(none,save_error_msg3);
      model[mod]->save.numsavedmuscs = 0;
      return;
   }

   /* copy the muscle structures to the savemuscle structure */
   for (i=0; i<model[mod]->nummuscles; i++)
   {
      if (copy_musc(&model[mod]->muscle[i],&model[mod]->save.muscle[i],
         &model[mod]->default_muscle,&model[mod]->save.default_muscle) == code_bad)
      {
         error(none,save_error_msg1);
         error(none,save_error_msg2);
         error(none,save_error_msg3);
         model[mod]->save.numsavedmuscs = 0;
         return;
      }

   /* Set all the refpt variables back to their original indices. If the muscles
    * are restored later, each muscle point in the model structure will point
    * back to the saved copy so that the "netmove" distance can be computed
    * correctly for each point. Points that the user added had a refpt of -1,
    * meaning that there was no corresponding saved point to do a netmove from.
    * But once the muscles are saved, that previously new point now has a copy
    * of itself in the saved-muscle structure.
    */
      for (j=0; j<*(model[mod]->muscle[i].num_orig_points); j++)
         model[mod]->muscle[i].mp_orig[j].refpt = j;
   }
}



/* RESTORE_MUSCLES: */

ReturnCode restore_muscles(int mod)
{

   int i;

   if (model[mod]->nummuscles <= 0)
      return code_bad;

//   for (i=0; i<model[mod]->nummuscles; i++)
//      printmuscle(&model[mod]->muscle[i]);

   if (model[mod]->save.muscle == NULL)
   {
      error(none,"There are no saved copies of this model\'s muscles.");
      return code_bad;
   }

   /* first free the memory used by the current muscles */

   free_muscs(model[mod]->muscle,&model[mod]->default_muscle,
              model[mod]->nummuscles);
   model[mod]->muscle = NULL;
   free_defmusc(&model[mod]->default_muscle);
   model[mod]->nummuscles = model[mod]->save.numsavedmuscs;

   /* copy the saved default muscle to the current default muscle structure */

   if (copy_default_muscle(&model[mod]->save.default_muscle,
			  &model[mod]->default_muscle) == code_bad)
   {
      error(none,restore_error_msg1);
      error(none,restore_error_msg2);
      error(none,restore_error_msg3);
      model[mod]->nummuscles = 0;
      return code_bad;
   }

   /* malloc memory for the to-be-restored muscles */

   model[mod]->muscle =
         (MuscleStruct*)simm_malloc(model[mod]->nummuscles*sizeof(MuscleStruct));

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
      if (copy_musc(&model[mod]->save.muscle[i],&model[mod]->muscle[i],
         &model[mod]->save.default_muscle,&model[mod]->default_muscle) == code_bad)
      {
         error(none,restore_error_msg1);
         error(none,restore_error_msg2);
         error(none,restore_error_msg3);
         model[mod]->nummuscles = 0;
         return code_bad;
      }
   }

   /* Invalidate the wrapping calculations because a joint may have moved
    * since the muscle was last saved.
    */

   for (i=0; i<model[mod]->nummuscles; i++)
      model[mod]->muscle[i].wrap_calced = no;

#if 0
   sprintf(buffer,"Restored all muscles in %s.", model[mod]->name);
   message(buffer,0,DEFAULT_MESSAGE_X_OFFSET);
#endif


//   for (i=0; i<model[mod]->nummuscles; i++)
//      printmuscle(&model[mod]->muscle[i]);

   return code_fine;

}



/* COPY_MUSC: */

ReturnCode copy_musc(MuscleStruct* from, MuscleStruct* to, MuscleStruct* deffrom,
                            MuscleStruct* defto)
{

   int i, j, mem_size;

   nullify_muscle(to);

   if (from->name == deffrom->name)
      to->name = defto->name;
   else if (mstrcpy(&to->name,from->name) == code_bad)
      return code_bad;

   to->display = from->display;
   to->has_wrapping_points = from->has_wrapping_points;
   to->has_force_points = from->has_force_points;
   to->mp_orig_array_size = from->mp_orig_array_size;

   if (from->num_orig_points == deffrom->num_orig_points)
      to->num_orig_points = defto->num_orig_points;
   else
   {
      if ((to->num_orig_points = (int*)simm_malloc(sizeof(int))) == NULL)
         return code_bad;
      *to->num_orig_points = *from->num_orig_points;
   }

   if (from->mp_orig == deffrom->mp_orig)
      to->mp_orig = defto->mp_orig;
   else
   {
      to->mp_orig = (MusclePoint*)simm_malloc((to->mp_orig_array_size)*sizeof(MusclePoint));
      if (to->mp_orig == NULL)
         return code_bad;
      for (i=0; i<*to->num_orig_points; i++)
         if (copy_mps(&from->mp_orig[i],&to->mp_orig[i]) == code_bad)
            return code_bad;
   }

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
            if (from->wrapStruct[i]->mp_wrap[j].numranges > 0)
            {
               to->wrapStruct[i]->mp_wrap[j].numranges = from->wrapStruct[i]->mp_wrap[j].numranges;
               mem_size = to->wrapStruct[i]->mp_wrap[j].numranges * sizeof(PointRange);
               to->wrapStruct[i]->mp_wrap[j].ranges = (PointRange*)simm_malloc(mem_size);
               memcpy(to->wrapStruct[i]->mp_wrap[j].wrap_pts, from->wrapStruct[i]->mp_wrap[j].wrap_pts, mem_size);
            }
            else
            {
               to->wrapStruct[i]->mp_wrap[j].numranges = 0;
               to->wrapStruct[i]->mp_wrap[j].ranges = NULL;
            }
         }
      }
   }

   /* The mp[] array is not copied; rather, space is allocated for it,
    * and the muscle's wrapping is invalidated so that mp[] will be set later.
    */
   to->wrap_calced = no;
   to->mp_array_size = *(to->num_orig_points) + (to->numWrapStructs * 2);
   to->mp = (MusclePoint**)simm_malloc(sizeof(MusclePoint*) * to->mp_array_size);
   to->num_points = 0;

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

   if (copy_nddouble(from->max_isometric_force,&to->max_isometric_force,deffrom->max_isometric_force,defto->max_isometric_force) == code_bad)
      return code_bad;
   if (copy_nddouble(from->pennation_angle,&to->pennation_angle,
                deffrom->pennation_angle,defto->pennation_angle) == code_bad)
      return code_bad;
   if (copy_nddouble(from->min_thickness,&to->min_thickness,
                deffrom->min_thickness,defto->min_thickness) == code_bad)
      return code_bad;
   if (copy_nddouble(from->max_thickness,&to->max_thickness,
                deffrom->max_thickness,defto->max_thickness) == code_bad)
      return code_bad;
   if (copy_nddouble(from->optimal_fiber_length,&to->optimal_fiber_length,
                deffrom->optimal_fiber_length,defto->optimal_fiber_length) == code_bad)
      return code_bad;
   if (copy_nddouble(from->resting_tendon_length,&to->resting_tendon_length,
                deffrom->resting_tendon_length,defto->resting_tendon_length) == code_bad)
      return code_bad;

   if (copy_ndint(from->min_material, &to->min_material, deffrom->min_material, defto->min_material) == code_bad)
      return code_bad;

   if (copy_ndint(from->max_material, &to->max_material, deffrom->max_material, defto->max_material) == code_bad)
      return code_bad;

   to->activation = from->activation;
   to->initial_activation = from->initial_activation;

   if (copy_nddouble(from->max_contraction_vel,&to->max_contraction_vel,
                deffrom->max_contraction_vel,defto->max_contraction_vel) == code_bad)
      return code_bad;
   if (copy_nondefault_dyn_params(from,to,deffrom,defto) == code_bad)
      return code_bad;

   to->nummomentarms = from->nummomentarms;
   if ((to->momentarms = (double*)simm_malloc(to->nummomentarms*sizeof(double))) == NULL)
      return code_bad;
   for (i=0; i<to->nummomentarms; i++)
      to->momentarms[i] = from->momentarms[i];

   if (from->active_force_len_curve == deffrom->active_force_len_curve)
      to->active_force_len_curve = defto->active_force_len_curve;
   else
   {
      if (alloc_func(&to->active_force_len_curve,from->active_force_len_curve->numpoints) == code_bad)
         return code_bad;
      copy_function(from->active_force_len_curve,to->active_force_len_curve);
   }

   if (from->passive_force_len_curve == deffrom->passive_force_len_curve)
      to->passive_force_len_curve = defto->passive_force_len_curve;
   else
   {
      if (alloc_func(&to->passive_force_len_curve,from->passive_force_len_curve->numpoints) == code_bad)
         return code_bad;
      copy_function(from->passive_force_len_curve,to->passive_force_len_curve);
   }

   if (from->tendon_force_len_curve == deffrom->tendon_force_len_curve)
      to->tendon_force_len_curve = defto->tendon_force_len_curve;
   else
   {
      if (alloc_func(&to->tendon_force_len_curve,from->tendon_force_len_curve->numpoints) == code_bad)
         return code_bad;
      copy_function(from->tendon_force_len_curve,to->tendon_force_len_curve);
   }
   if (from->force_vel_curve == deffrom->force_vel_curve)
      to->force_vel_curve = defto->force_vel_curve;
   else
   {
      if (alloc_func(&to->force_vel_curve,from->force_vel_curve->numpoints) == code_bad)
         return code_bad;
      copy_function(from->force_vel_curve,to->force_vel_curve);
   }

   to->excitation_index = from->excitation_index;
   to->excitation_abscissa = from->excitation_abscissa;
   if (from->excitation == deffrom->excitation)
      to->excitation = defto->excitation;
   else
   {
      if (from->excitation == NULL)
         to->excitation = NULL;
      else
      {
         if (alloc_func(&to->excitation,from->excitation->numpoints) == code_bad)
            return code_bad;
         copy_function(from->excitation,to->excitation);
      }
   }
   if (from->excitation_format == deffrom->excitation_format)
      to->excitation_format = defto->excitation_format;
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

   if (copy_ndint(from->muscle_model_index,&to->muscle_model_index,
		  deffrom->muscle_model_index,defto->muscle_model_index) == code_bad)
      return code_bad;

   to->output = from->output;

   return code_fine;

}



/* COPY_DEFAULT_MUSCLE: */

ReturnCode copy_default_muscle(MuscleStruct* from, MuscleStruct* to)
{

   int i;

   if (from->name == NULL)
      to->name = NULL;
   else
      mstrcpy(&to->name,from->name);

   to->has_wrapping_points = from->has_wrapping_points;
   to->has_force_points = from->has_force_points;
   to->mp_orig_array_size = from->mp_orig_array_size;

   if (from->num_orig_points == NULL)
   {
      to->num_orig_points = NULL;
      to->mp_orig = NULL;
   }
   else
   {
      if ((to->num_orig_points = (int*)simm_malloc(sizeof(int))) == NULL)
         return code_bad;
      *to->num_orig_points = *from->num_orig_points;
      to->mp_orig = (MusclePoint*)simm_malloc((to->mp_orig_array_size)*sizeof(MusclePoint));
      if (to->mp_orig == NULL)
         return code_bad;
      for (i=0; i<*to->num_orig_points; i++)
         if (copy_mps(&from->mp_orig[i],&to->mp_orig[i]) == code_bad)
            return code_bad;
   }

   to->nummomentarms = 0;
   to->momentarms = NULL;

   to->numgroups = from->numgroups;
   if (to->numgroups == 0)
      to->group = NULL;
   else
   {
      if ((to->group = (int*)simm_malloc((to->numgroups)*sizeof(int))) == NULL)
         return code_bad;
      for (i=0; i<(to->numgroups); i++)
         to->group[i] = from->group[i];
   }

   if (copy_nndouble(from->max_isometric_force,&to->max_isometric_force) == code_bad)
      return code_bad;
   if (copy_nndouble(from->pennation_angle,&to->pennation_angle) == code_bad)
      return code_bad;
   if (copy_nndouble(from->min_thickness,&to->min_thickness) == code_bad)
      return code_bad;
   if (copy_nndouble(from->max_thickness,&to->max_thickness) == code_bad)
      return code_bad;
   if (copy_nndouble(from->max_contraction_vel,&to->max_contraction_vel) == code_bad)
      return code_bad;
   if (copy_nonnull_dyn_params(from,to) == code_bad)
      return code_bad;

   if (copy_nndouble(from->optimal_fiber_length,&to->optimal_fiber_length) == code_bad)
      return code_bad;
   if (copy_nndouble(from->resting_tendon_length,&to->resting_tendon_length) == code_bad)
      return code_bad;

   if (from->min_material == NULL)
      to->min_material = NULL;
   else
   {
      if ((to->min_material = (int*)simm_malloc(sizeof(int))) == NULL)
         return code_bad;
      *to->min_material = *from->min_material;
   }

   if (from->max_material == NULL)
      to->max_material = NULL;
   else
   {
      if ((to->max_material = (int*)simm_malloc(sizeof(int))) == NULL)
         return code_bad;
      *to->max_material = *from->max_material;
   }

   to->activation = from->activation;
   to->initial_activation = from->initial_activation;

   if (from->active_force_len_curve == NULL)
      to->active_force_len_curve = NULL;
   else
   {
      if (alloc_func(&to->active_force_len_curve,from->active_force_len_curve->numpoints) == code_bad)
         return code_bad;
      copy_function(from->active_force_len_curve,to->active_force_len_curve);
   }

   if (from->passive_force_len_curve == NULL)
      to->passive_force_len_curve = NULL;
   else
   {
      if (alloc_func(&to->passive_force_len_curve,from->passive_force_len_curve->numpoints) == code_bad)
         return code_bad;
      copy_function(from->passive_force_len_curve,to->passive_force_len_curve);
   }

   if (from->tendon_force_len_curve == NULL)
      to->tendon_force_len_curve = NULL;
   else
   {
      if (alloc_func(&to->tendon_force_len_curve,from->tendon_force_len_curve->numpoints) == code_bad)
         return code_bad;
      copy_function(from->tendon_force_len_curve,to->tendon_force_len_curve);
   }
   if (from->force_vel_curve == NULL)
      to->force_vel_curve = NULL;
   else
   {
      if (alloc_func(&to->force_vel_curve,from->force_vel_curve->numpoints) == code_bad)
         return code_bad;
      copy_function(from->force_vel_curve,to->force_vel_curve);
   }

   to->excitation_index = from->excitation_index;
   to->excitation_abscissa = from->excitation_abscissa;
   if (from->excitation == NULL)
      to->excitation = NULL;
   else
   {
      if (alloc_func(&to->excitation,from->excitation->numpoints) == code_bad)
         return code_bad;
      copy_function(from->excitation,to->excitation);
   }

   if (from->excitation_format == NULL)
      to->excitation_format = NULL;
   else
   {
      if ((to->excitation_format = (SplineType*)simm_malloc(sizeof(SplineType))) == NULL)
         return code_bad;
      *(to->excitation_format) = *(from->excitation_format);
   }

   if (copy_nnint(from->muscle_model_index,&to->muscle_model_index) == code_bad)
      return code_bad;

   return code_fine;

}



/* ALLOC_FUNC: */

ReturnCode alloc_func(SplineFunction** func, int pts)
{

   if ((*func = (SplineFunction*)simm_malloc(sizeof(SplineFunction))) == NULL)
      return code_bad;
   if (((*func)->x = (double*)simm_malloc(pts*sizeof(double))) == NULL)
      return code_bad;
   if (((*func)->y = (double*)simm_malloc(pts*sizeof(double))) == NULL)
      return code_bad;
   if (((*func)->b = (double*)simm_malloc(pts*sizeof(double))) == NULL)
      return code_bad;
   if (((*func)->c = (double*)simm_malloc(pts*sizeof(double))) == NULL)
      return code_bad;
   if (((*func)->d = (double*)simm_malloc(pts*sizeof(double))) == NULL)
      return code_bad;

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
      error(none,"could not save muscle groups");
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
   printf("has_wrapping_points: %d\n", musc->has_wrapping_points);

   if (musc->num_orig_points == NULL)
      printf("numpoints: ptr is null\n");
   else
   {
      printf("numpoints: %d\n", *musc->num_orig_points);
      for (i=0; i<*musc->num_orig_points; i++)
         printf("(%d) %lf %lf %lf segment %d\n", i,
                musc->mp_orig[i].point[0],
                musc->mp_orig[i].point[1],
                musc->mp_orig[i].point[2],
                musc->mp_orig[i].segment);
   }

   if (musc->max_isometric_force == NULL)
      printf("max_isometric_force: ptr is null\n");
   else
      printf("max_isometric_force: %lf\n", *musc->max_isometric_force);

   printf("\n");
   (void)fflush(stdout);

}


static ReturnCode copy_nondefault_dyn_params(MuscleStruct* from, MuscleStruct* to,
					                              MuscleStruct* deffrom, MuscleStruct* defto)
{
   int i;

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
                           deffrom->dynamic_params[i], defto->dynamic_params[i]) == code_bad)
            return code_bad;
      }
   }
   else
   {
      to->dynamic_param_names = NULL;
      to->dynamic_params = NULL;
   }

   return code_fine;
}



static ReturnCode copy_nonnull_dyn_params(MuscleStruct* from, MuscleStruct* to)
{
   int i;

   to->num_dynamic_params = from->num_dynamic_params;

   if (to->num_dynamic_params > 0)
   {
      to->dynamic_param_names = from->dynamic_param_names;
      to->dynamic_params = (double**)simm_malloc(to->num_dynamic_params * sizeof(double*));

      if (to->dynamic_params == NULL)
         return code_bad;

      for (i = 0; i < to->num_dynamic_params; i++)
      {
         if (copy_nndouble(from->dynamic_params[i], &to->dynamic_params[i]) == code_bad)
            return code_bad;
      }
   }
   else
   {
      to->dynamic_param_names = NULL;
      to->dynamic_params = NULL;
   }

   return code_fine;
}
