/*******************************************************************************

   COPY.C

   Author: Peter Loan

   Date: 17-MAY-89

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Description: This file contains routines that copy values
      in some structure or array to another structure or array.

   Routines:
      copy_muscle         : copies an individual muscle structure
      copy_default_muscle : copies one default muscle structure to another
      copy_4x4matrix : copies a 4x4 matrix
      copy_1x4vector : copies a 1x4 array (vector)
      copy_dof       : copies a REFEQ structure which holds a dof
      copy_function  : copies a function's points and coefficients
      copy_mps       : copies a dpMusclePoint array
      copy_nndouble  : mallocs and copies a double if 'from' is not null
      copy_nddouble  : mallocs and copies a double if 'from' is not default
      copy_nnint     : mallocs and copies a int if 'from' is not null
      copy_ndint     : mallocs and copies a int if 'from' is not default

*******************************************************************************/

#include "universal.h"
#include "globals.h"
#include "functions.h"
#include "defunctions.h"


/*************** DEFINES (for this file only) *********************************/



/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/



/*************** EXTERNED VARIABLES (declared in another file) ****************/



/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static ReturnCode copy_nondefault_dyn_params(dpMuscleStruct* from, dpMuscleStruct* to,
                         dpMuscleStruct* deffrom, dpMuscleStruct* defto);
static ReturnCode copy_nonnull_dyn_params(dpMuscleStruct* from, dpMuscleStruct* to);
static MotionSequence* copy_motion(MotionSequence* from);
static dpQStruct* get_q_from_gencoord_dp(dpModelStruct* model, GeneralizedCoord *genc);
static ReturnCode copy_muscle_path_dp(dpMusclePathStruct *from, dpMusclePathStruct *to, dpModelStruct *modelTo);
static ReturnCode copy_muscle_point_dp(dpMusclePoint *from, dpMusclePoint *to, dpModelStruct *modelTo);


void copy_1x4vector(double from[], double to[])
{
   to[0] = from[0];
   to[1] = from[1];
   to[2] = from[2];
   to[3] = from[3];
}


void copy_point(dpCoord3D *from, dpCoord3D *to)
{
   to->xyz[0] = from->xyz[0];
   to->xyz[1] = from->xyz[1];
   to->xyz[2] = from->xyz[2];
}

/* COPY_DOF: copies a dof. Used to save and restore dofs in the joint
 * editor and to copy dofs from the model-file-reading buffer to a
 * model structure.
 */
void copy_dof(DofStruct* from, DofStruct* to)
{
   to->type = from->type;
   to->value = from->value;
   to->function = from->function;
   to->gencoord = from->gencoord;
}


void copy_gencoord_info(GeneralizedCoord *from, SaveGencoords *to)
{
   to->value = from->value;
   to->velocity = from->velocity;
   to->tolerance = from->tolerance;
   to->pd_stiffness = from->pd_stiffness;
   to->default_value = from->default_value;
   to->clamped = from->clamped;
   to->locked = from->locked;
   to->restraintFuncActive = from->restraintFuncActive;
   to->range = from->range;
   to->restraint_function = from->restraint_function;
   to->used_in_model = from->used_in_model;
}


/* COPY_FUNCTION: copies a function structure. This structure contains
 * the original (x,y) data points and the coefficients (b,c,d) of
 * the function interpolants. It assumes that space for the x, y, a, b, c
 * arrays has already been allocated, and the size of this space
 * is stored in to->coefficient_array_size.
 */
void copy_function(dpFunction* from, dpFunction* to)
{
    int numToCopy = _MIN(to->coefficient_array_size, from->coefficient_array_size);

    to->type = from->type;
    to->numpoints = from->numpoints;
    to->source = from->source;
    to->status = from->status;
    to->used = from->used;
    to->usernum = from->usernum;
    to->cutoff_frequency = from->cutoff_frequency;

   memcpy(to->x, from->x, numToCopy * sizeof(double));
   memcpy(to->y, from->y, numToCopy * sizeof(double));
   memcpy(to->b, from->b, numToCopy * sizeof(double));
   memcpy(to->c, from->c, numToCopy * sizeof(double));
   memcpy(to->d, from->d, numToCopy * sizeof(double));
}

/* COPY_MUSCLE_PATH: copy the muscle paths - all the original points (mp_orig),
 * and the active points (mp), from one path struct to another.
 */
ReturnCode copy_muscle_path(dpMusclePathStruct *from, dpMusclePathStruct *to)
{
   int i;

   if (from == NULL || to == NULL)
      return code_bad;

   to->num_orig_points = from->num_orig_points;
   to->mp_orig_array_size = to->num_orig_points;
   to->mp_orig = (dpMusclePoint*)simm_malloc(to->mp_orig_array_size * sizeof(dpMusclePoint));
   for (i=0; i<to->num_orig_points; i++)
   {
      if (copy_muscle_point(&from->mp_orig[i], &to->mp_orig[i]) == code_bad)
         return code_bad;
   }

   to->num_points = 0;
   to->mp_array_size = from->mp_array_size;
   to->mp = (dpMusclePoint**)simm_calloc(to->mp_array_size, sizeof(dpMusclePoint*));
   if (to->mp == NULL)
      return code_bad;

   return code_fine;
}

/* COPY_MUSCLE_POINT: copy the individual point from a path
 * Allocate space for the wrapping points 
 */
ReturnCode copy_muscle_point(dpMusclePoint* from, dpMusclePoint* to)
{
   if (from == NULL || to == NULL)
      return code_bad;

   memcpy(to, from, sizeof(dpMusclePoint));

   if (to->num_wrap_pts > 0)
   {
      to->wrap_pts = (double*)simm_malloc(to->num_wrap_pts*3*sizeof(double));
      if (to->wrap_pts == NULL)
         return code_bad;
      memcpy(to->wrap_pts, from->wrap_pts, to->num_wrap_pts * 3 * sizeof(double));
   }

   return code_fine;
}

/* COPY_MUSCLE_PATH_DP: copy the muscle paths - all the original points (mp_orig),
 * and the active points (mp), from one path struct to another that will be used by
 * a Dynamics Pipeline model.
 */
static ReturnCode copy_muscle_path_dp(dpMusclePathStruct *from, dpMusclePathStruct *to, dpModelStruct *modelTo)
{
   int i;

   if (from == NULL || to == NULL)
      return code_bad;

   to->num_orig_points = from->num_orig_points;
   to->mp_orig_array_size = to->num_orig_points;
   to->mp_orig = (dpMusclePoint*)simm_malloc(to->mp_orig_array_size * sizeof(dpMusclePoint));
   for (i=0; i<to->num_orig_points; i++)
   {
      if (copy_muscle_point_dp(&from->mp_orig[i], &to->mp_orig[i], modelTo) == code_bad)
         return code_bad;
   }

   to->num_points = 0;
   to->mp_array_size = from->mp_array_size;
   to->mp = (dpMusclePoint**)simm_calloc(to->mp_array_size, sizeof(dpMusclePoint*));
   if (to->mp == NULL)
      return code_bad;

   return code_fine;
}

/* COPY_MUSCLE_POINT_DP: copy the individual point from a path.  Since destination
 * model is a dynamics Pipeline model, don't want to store gencoord information in
 * GeneralizedCoord struct.  Find the equivalent QStruct and store that.
 * Allocate space for the wrapping points.
 */
static ReturnCode copy_muscle_point_dp(dpMusclePoint* from, dpMusclePoint* to, dpModelStruct *modelTo)
{
   int i;

   if (from == NULL || to == NULL)
      return code_bad;

   memcpy(to, from, sizeof(dpMusclePoint));

   if (to->num_wrap_pts > 0)
   {
      to->wrap_pts = (double*)simm_malloc(to->num_wrap_pts*3*sizeof(double));
      if (to->wrap_pts == NULL)
         return code_bad;
      memcpy(to->wrap_pts, from->wrap_pts, to->num_wrap_pts * 3 * sizeof(double));
   }

   if (to->isVia)
      to->viaRange.gencoord = (void *)get_q_from_gencoord_dp(modelTo, (GeneralizedCoord*)from->viaRange.gencoord);

   if (to->isMovingPoint)
   {
      for (i = 0; i < 3; i++)
      {
         if (from->gencoord[i] != NULL)
            to->gencoord[i] = (void *)get_q_from_gencoord_dp(modelTo, (GeneralizedCoord*)from->gencoord[i]);
      }
   }
   return code_fine;
}

/* COPY_DEFAULT_MUSCLE: copy the default muscle to another structure used by a SIMM model. */
ReturnCode copy_default_muscle(dpMuscleStruct* from, dpMuscleStruct* to, ModelStruct* modelTo)
{
   int i;

   if (from == NULL || to == NULL)
      return code_fine;

   if (from->name == NULL)
      to->name = NULL;
   else
      mstrcpy(&to->name,from->name);

   to->index = from->index;

   // The default muscle cannot contain path information.
   to->path = NULL;

   to->nummomentarms = 0;
   to->momentarms = NULL;

   if (copy_nonnull_dyn_params(from, to) == code_bad)
      return code_bad;

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

   if (copy_nndouble(from->optimal_fiber_length,&to->optimal_fiber_length) == code_bad)
      return code_bad;
   if (copy_nndouble(from->resting_tendon_length,&to->resting_tendon_length) == code_bad)
      return code_bad;

   if (copy_nnint(from->min_material,&to->min_material) == code_bad)
      return code_bad;
   if (copy_nnint(from->max_material,&to->max_material) == code_bad)
      return code_bad;

   to->dynamic_activation = from->dynamic_activation;

   if (from->active_force_len_func == NULL)
      to->active_force_len_func = NULL;
   else
   {
      to->active_force_len_func = (dpFunction**)simm_malloc(sizeof(dpFunction*));
      if (to->active_force_len_func == NULL)
         return code_bad;
      *to->active_force_len_func = *from->active_force_len_func;
   }

   if (from->passive_force_len_func == NULL)
      to->passive_force_len_func = NULL;
   else
   {
      to->passive_force_len_func = (dpFunction**)simm_malloc(sizeof(dpFunction*));
      if (to->passive_force_len_func == NULL)
         return code_bad;
      *to->passive_force_len_func = *from->passive_force_len_func;
   }

   if (from->tendon_force_len_func == NULL)
      to->tendon_force_len_func = NULL;
   else
   {
      to->tendon_force_len_func = (dpFunction**)simm_malloc(sizeof(dpFunction*));
      if (to->tendon_force_len_func == NULL)
         return code_bad;
      *to->tendon_force_len_func = *from->tendon_force_len_func;
   }

   if (from->force_vel_func == NULL)
      to->force_vel_func = NULL;
   else
   {
      to->force_vel_func = (dpFunction**)simm_malloc(sizeof(dpFunction*));
      if (to->force_vel_func == NULL)
         return code_bad;
      *to->force_vel_func = *from->force_vel_func;
   }

   if (from->excitation_abscissa == NULL)
    {
      to->excitation_abscissa = from->excitation_abscissa;
    }
    else
   {
      to->excitation_abscissa = (void**)simm_malloc(sizeof(void*));
      if (to->excitation_abscissa == NULL)
         return code_bad;
        if (*from->excitation_abscissa == TIME)
            *to->excitation_abscissa = TIME;
        else
      {
            *to->excitation_abscissa = (void*)enter_gencoord(modelTo, ((GeneralizedCoord*)(*from->excitation_abscissa))->name, no);
         // If you got a NULL in the destination, but the source has an abscissa, something went wrong
         if (*to->excitation_abscissa == NULL && *from->excitation_abscissa != NULL)
            return code_bad;
      }

   }

   if (from->excitation_func == NULL)
      to->excitation_func = NULL;
   else
   {
      to->excitation_func = (dpFunction**)simm_malloc(sizeof(dpFunction*));
      if (to->excitation_func == NULL)
         return code_bad;
      *to->excitation_func = *from->excitation_func;
   }

   if (copy_nnint(from->muscle_model_index, &to->muscle_model_index) == code_bad)
      return code_bad;

   // Not used in SIMM.
   to->numStateParams = 0;
   to->stateParams = NULL;

   return code_fine;
}

/* COPY_DEFAULT_MUSCLE_DP : copy the default muscle to another muscle structure that
 * will be used by a Dynamics Pipeline model.  If the default muscle has an excitation
 * function defined with an abscissa other than time, the gencoord should not be stored
 * as a GeneralizedCoord struct, but as a QStruct.
 */
ReturnCode copy_default_muscle_dp(dpMuscleStruct* from, dpMuscleStruct* to, dpModelStruct* modelTo)
{
   int i;

   if (from == NULL || to == NULL)
      return code_fine;

   if (from->name == NULL)
      to->name = NULL;
   else
      mstrcpy(&to->name,from->name);

   to->index = from->index;

   // The default muscle cannot contain path information.
   to->path = NULL;

   to->nummomentarms = 0;
   to->momentarms = NULL;

   if (copy_nonnull_dyn_params(from, to) == code_bad)
      return code_bad;

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

   if (copy_nndouble(from->optimal_fiber_length,&to->optimal_fiber_length) == code_bad)
      return code_bad;
   if (copy_nndouble(from->resting_tendon_length,&to->resting_tendon_length) == code_bad)
      return code_bad;

   if (copy_nnint(from->min_material,&to->min_material) == code_bad)
      return code_bad;
   if (copy_nnint(from->max_material,&to->max_material) == code_bad)
      return code_bad;

   to->dynamic_activation = from->dynamic_activation;

   if (from->active_force_len_func == NULL)
      to->active_force_len_func = NULL;
   else
   {
      to->active_force_len_func = (dpFunction**)simm_malloc(sizeof(dpFunction*));
      if (to->active_force_len_func == NULL)
         return code_bad;
      *to->active_force_len_func = *from->active_force_len_func;
   }

   if (from->passive_force_len_func == NULL)
      to->passive_force_len_func = NULL;
   else
   {
      to->passive_force_len_func = (dpFunction**)simm_malloc(sizeof(dpFunction*));
      if (to->passive_force_len_func == NULL)
         return code_bad;
      *to->passive_force_len_func = *from->passive_force_len_func;
   }

   if (from->tendon_force_len_func == NULL)
      to->tendon_force_len_func = NULL;
   else
   {
      to->tendon_force_len_func = (dpFunction**)simm_malloc(sizeof(dpFunction*));
      if (to->tendon_force_len_func == NULL)
         return code_bad;
      *to->tendon_force_len_func = *from->tendon_force_len_func;
   }

   if (from->force_vel_func == NULL)
      to->force_vel_func = NULL;
   else
   {
      to->force_vel_func = (dpFunction**)simm_malloc(sizeof(dpFunction*));
      if (to->force_vel_func == NULL)
         return code_bad;
      *to->force_vel_func = *from->force_vel_func;
   }

   if (from->excitation_abscissa == NULL)
    {
      to->excitation_abscissa = from->excitation_abscissa;
    }
    else
   {
      to->excitation_abscissa = (void**)simm_malloc(sizeof(void*));
      if (to->excitation_abscissa == NULL)
         return code_bad;
        if (*from->excitation_abscissa == TIME)
            *to->excitation_abscissa = TIME;
        else
      {
         *to->excitation_abscissa = (void*)get_q_from_gencoord_dp(modelTo, (GeneralizedCoord*)(*from->excitation_abscissa));
         // If you got a NULL in the destination, but the source has an abscissa, something went wrong
         if (*to->excitation_abscissa == NULL && *from->excitation_abscissa != NULL)
            return code_bad;
      }
   }

   if (from->excitation_func == NULL)
      to->excitation_func = NULL;
   else
   {
      to->excitation_func = (dpFunction**)simm_malloc(sizeof(dpFunction*));
      if (to->excitation_func == NULL)
         return code_bad;
      *to->excitation_func = *from->excitation_func;
   }

   if (copy_nnint(from->muscle_model_index, &to->muscle_model_index) == code_bad)
      return code_bad;

   // Not used in SIMM.
   to->numStateParams = 0;
   to->stateParams = NULL;

   return code_fine;
}

/* COPY_MUSCLE: copy a muscle to another struct used by a SIMM model. */
ReturnCode copy_muscle(dpMuscleStruct* from, dpMuscleStruct* to, dpMuscleStruct* deffrom, dpMuscleStruct *defto, ModelStruct* modelTo)
{
   int i, j, mem_size;

   memcpy(to, from, sizeof(dpMuscleStruct));

#if MEMORY_LEAK
   if (from->name == deffrom->name)
      to->name = deffrom->name;
   else
   {
      mstrcpy(&to->name,from->name);
      if (to->name == NULL)
         return code_bad;
   }
#else
   if (from->name == deffrom->name)
      to->name = deffrom->name;
   else if (mstrcpy(&to->name,from->name) == code_bad)
      return code_bad;
#endif

   if (to->numWrapStructs > 0)
   {
      if ((to->wrapStruct = (dpMuscleWrapStruct**)simm_malloc(to->numWrapStructs * sizeof(dpMuscleWrapStruct*))) == NULL)
         return code_bad;
      for (i = 0; i < to->numWrapStructs; i++)
      {
         if ((to->wrapStruct[i] = (dpMuscleWrapStruct*)simm_malloc(sizeof(dpMuscleWrapStruct))) == NULL)
            return code_bad;
         memcpy(to->wrapStruct[i], from->wrapStruct[i], sizeof(dpMuscleWrapStruct));
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

   if (from->path)
   {
      to->path = (dpMusclePathStruct*)simm_calloc(1, sizeof(dpMusclePathStruct));
        //free_muscle(to, defto); // just to test!

      if (copy_muscle_path(from->path, to->path) == code_bad)
         return code_bad;
   }
   to->wrap_calced = no;

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

   if (copy_nddouble(from->max_contraction_vel,&to->max_contraction_vel,
                deffrom->max_contraction_vel,defto->max_contraction_vel) == code_bad)
      return code_bad;

   if (copy_nondefault_dyn_params(from,to,deffrom,defto) == code_bad)
      return code_bad;

   if ((to->momentarms = (double*)simm_malloc(to->nummomentarms*sizeof(double))) == NULL)
      return code_bad;
   for (i=0; i<to->nummomentarms; i++)
      to->momentarms[i] = from->momentarms[i];

   if (from->active_force_len_func == deffrom->active_force_len_func)
      to->active_force_len_func = defto->active_force_len_func;
   else
   {
      if (from->active_force_len_func == NULL)
         to->active_force_len_func = NULL;
      else
      {
         if ((to->active_force_len_func = (dpFunction**)simm_malloc(sizeof(dpFunction*))) == NULL)
            return code_bad;
         *to->active_force_len_func = *from->active_force_len_func;
      }
   }

   if (from->passive_force_len_func == deffrom->passive_force_len_func)
      to->passive_force_len_func = defto->passive_force_len_func;
   else
   {
      if (from->passive_force_len_func == NULL)
         to->passive_force_len_func = NULL;
      else
      {
         if ((to->passive_force_len_func = (dpFunction**)simm_malloc(sizeof(dpFunction*))) == NULL)
            return code_bad;
         *to->passive_force_len_func = *from->passive_force_len_func;
      }
   }

   if (from->tendon_force_len_func == deffrom->tendon_force_len_func)
      to->tendon_force_len_func = defto->tendon_force_len_func;
   else
   {
      if (from->tendon_force_len_func == NULL)
         to->tendon_force_len_func = NULL;
      else
      {
         if ((to->tendon_force_len_func = (dpFunction**)simm_malloc(sizeof(dpFunction*))) == NULL)
            return code_bad;
         *to->tendon_force_len_func = *from->tendon_force_len_func;
      }
   }

   if (from->force_vel_func == deffrom->force_vel_func)
      to->force_vel_func = defto->force_vel_func;
   else
   {
      if (from->force_vel_func == NULL)
         to->force_vel_func = NULL;
      else
      {
         if ((to->force_vel_func = (dpFunction**)simm_malloc(sizeof(dpFunction*))) == NULL)
            return code_bad;
         *to->force_vel_func = *from->force_vel_func;
      }
   }

   if (from->excitation_abscissa == deffrom->excitation_abscissa)
      to->excitation_abscissa = defto->excitation_abscissa;
   else
   {
      if (from->excitation_abscissa == NULL)
        {
         to->excitation_abscissa = NULL;
        }
      else
      {
         if ((to->excitation_abscissa = (void**)simm_malloc(sizeof(void*))) == NULL)
            return code_bad;
            if (*from->excitation_abscissa == TIME)
                *to->excitation_abscissa = TIME;
            else
         {
            *to->excitation_abscissa = (void*)enter_gencoord(modelTo, ((GeneralizedCoord*)(*from->excitation_abscissa))->name, no);
            // If you got a NULL in the destination, but the source has an abscissa, something went wron
            if (*to->excitation_abscissa == NULL && *from->excitation_abscissa != NULL)
               return code_bad;     
         }
      }
   }

   if (from->excitation_func == deffrom->excitation_func)
      to->excitation_func = defto->excitation_func;
   else
   {
      if (from->excitation_func == NULL)
         to->excitation_func = NULL;
      else
      {
         if ((to->excitation_func = (dpFunction**)simm_malloc(sizeof(dpFunction*))) == NULL)
            return code_bad;
         *to->excitation_func = *from->excitation_func;
      }
   }

   if (copy_ndint(from->muscle_model_index,&to->muscle_model_index,
          deffrom->muscle_model_index,defto->muscle_model_index) == code_bad)
      return code_bad;

   to->saved = NULL;

   // Not used in SIMM.
   to->numStateParams = 0;
   to->stateParams = NULL;

   return code_fine;
}

/* COPY_MUSCLE_DP: copy the muscle into a structure that will be used by a Dynamics Pipeline model.
 * All references to gencoords in the muscle (via points, moving muscle points, excitation abscissae),
 * should refer to the QSTruct, not the GeneralizedCoord struct.
 */
ReturnCode copy_muscle_dp(dpMuscleStruct* from, dpMuscleStruct* to, dpMuscleStruct* deffrom, dpMuscleStruct *defto, dpModelStruct* modelTo)
{
   int i, j, mem_size;

   memcpy(to, from, sizeof(dpMuscleStruct));

#if MEMORY_LEAK
   if (from->name == deffrom->name)
      to->name = deffrom->name;
   else
   {
      mstrcpy(&to->name,from->name);
      if (to->name == NULL)
         return code_bad;
   }
#else
   if (from->name == deffrom->name)
      to->name = deffrom->name;
   else if (mstrcpy(&to->name,from->name) == code_bad)
      return code_bad;
#endif

   if (to->numWrapStructs > 0)
   {
      if ((to->wrapStruct = (dpMuscleWrapStruct**)simm_malloc(to->numWrapStructs * sizeof(dpMuscleWrapStruct*))) == NULL)
         return code_bad;
      for (i = 0; i < to->numWrapStructs; i++)
      {
         if ((to->wrapStruct[i] = (dpMuscleWrapStruct*)simm_malloc(sizeof(dpMuscleWrapStruct))) == NULL)
            return code_bad;
         memcpy(to->wrapStruct[i], from->wrapStruct[i], sizeof(dpMuscleWrapStruct));
         // need to allocate space for wrap_object otherwise element in DP model points to element
         // in SIMM model.  Changes to DP model (adjusting segment index for SD segments) 
         // cause unwanted changes in SIMM model
         // find the wrap object in dp model...
//         to->wrapStruct[i]->wrap_object = (dpWrapObject *) simm_malloc(sizeof(dpWrapObject));
         for (j = 0; j < modelTo->num_wrap_objects; j++)
         {
            if (STRINGS_ARE_EQUAL(from->wrapStruct[i]->wrap_object->name, modelTo->wrap_object[j].name))
            {
               to->wrapStruct[i]->wrap_object = &modelTo->wrap_object[j];
            }
         }
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

   if (from->path)
   {
      to->path = (dpMusclePathStruct*)simm_calloc(1, sizeof(dpMusclePathStruct));
        //free_muscle(to, defto); // just to test!

      if (copy_muscle_path_dp(from->path, to->path, modelTo) == code_bad)
         return code_bad;
   }
   to->wrap_calced = no;

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

   if (copy_nddouble(from->max_contraction_vel,&to->max_contraction_vel,
                deffrom->max_contraction_vel,defto->max_contraction_vel) == code_bad)
      return code_bad;

   if (copy_nondefault_dyn_params(from,to,deffrom,defto) == code_bad)
      return code_bad;

   if ((to->momentarms = (double*)simm_malloc(to->nummomentarms*sizeof(double))) == NULL)
      return code_bad;
   for (i=0; i<to->nummomentarms; i++)
      to->momentarms[i] = from->momentarms[i];

   if (from->active_force_len_func == deffrom->active_force_len_func)
      to->active_force_len_func = defto->active_force_len_func;
   else
   {
      if (from->active_force_len_func == NULL)
         to->active_force_len_func = NULL;
      else
      {
         if ((to->active_force_len_func = (dpFunction**)simm_malloc(sizeof(dpFunction*))) == NULL)
            return code_bad;
         *to->active_force_len_func = *from->active_force_len_func;
      }
   }

   if (from->passive_force_len_func == deffrom->passive_force_len_func)
      to->passive_force_len_func = defto->passive_force_len_func;
   else
   {
      if (from->passive_force_len_func == NULL)
         to->passive_force_len_func = NULL;
      else
      {
         if ((to->passive_force_len_func = (dpFunction**)simm_malloc(sizeof(dpFunction*))) == NULL)
            return code_bad;
         *to->passive_force_len_func = *from->passive_force_len_func;
      }
   }

   if (from->tendon_force_len_func == deffrom->tendon_force_len_func)
      to->tendon_force_len_func = defto->tendon_force_len_func;
   else
   {
      if (from->tendon_force_len_func == NULL)
         to->tendon_force_len_func = NULL;
      else
      {
         if ((to->tendon_force_len_func = (dpFunction**)simm_malloc(sizeof(dpFunction*))) == NULL)
            return code_bad;
         *to->tendon_force_len_func = *from->tendon_force_len_func;
      }
   }

   if (from->force_vel_func == deffrom->force_vel_func)
      to->force_vel_func = defto->force_vel_func;
   else
   {
      if (from->force_vel_func == NULL)
         to->force_vel_func = NULL;
      else
      {
         if ((to->force_vel_func = (dpFunction**)simm_malloc(sizeof(dpFunction*))) == NULL)
            return code_bad;
         *to->force_vel_func = *from->force_vel_func;
      }
   }

   if (from->excitation_abscissa == deffrom->excitation_abscissa)
      to->excitation_abscissa = defto->excitation_abscissa;
   else
   {
      if (from->excitation_abscissa == NULL)
        {
         to->excitation_abscissa = NULL;
        }
      else
      {
         if ((to->excitation_abscissa = (void**)simm_malloc(sizeof(void*))) == NULL)
            return code_bad;
            if (*from->excitation_abscissa == TIME)
                *to->excitation_abscissa = TIME;
            else
         {
                *to->excitation_abscissa = (void*)get_q_from_gencoord_dp(modelTo, (GeneralizedCoord*)(*from->excitation_abscissa));
            // If you got a NULL in the destination, but the source has an abscissa, something went wron
            if (*to->excitation_abscissa == NULL && *from->excitation_abscissa != NULL)
               return code_bad;
         }
      }
   }

   if (from->excitation_func == deffrom->excitation_func)
      to->excitation_func = defto->excitation_func;
   else
   {
      if (from->excitation_func == NULL)
         to->excitation_func = NULL;
      else
      {
         if ((to->excitation_func = (dpFunction**)simm_malloc(sizeof(dpFunction*))) == NULL)
            return code_bad;
         *to->excitation_func = *from->excitation_func;
      }
   }

   if (copy_ndint(from->muscle_model_index,&to->muscle_model_index,
          deffrom->muscle_model_index,defto->muscle_model_index) == code_bad)
      return code_bad;

   to->saved = NULL;

   // Not used in SIMM.
   to->numStateParams = 0;
   to->stateParams = NULL;

   return code_fine;
}


ReturnCode copy_nndouble(double* from, double** to)
{
   if (from == NULL)
      *to = NULL;
   else
   {
      if ((*to = (double*)simm_malloc(sizeof(double))) == NULL)
         return code_bad;
      **to = *from;
   }

   return code_fine;
}


ReturnCode copy_nddouble(double* from, double** to, double* deffrom, double* defto)
{
   if (from == deffrom)
      *to = defto;
   else
   {
      if (from == NULL)
         *to = NULL;
      else
      {
         if ((*to = (double*)simm_malloc(sizeof(double))) == NULL)
            return code_bad;
         **to = *from;
      }
   }

   return code_fine;
}


ReturnCode copy_nnint(int* from, int** to)
{
   if (from == NULL)
      *to = NULL;
   else
   {
      if ((*to = (int*)simm_malloc(sizeof(int))) == NULL)
         return code_bad;
      **to = *from;
   }

   return code_fine;
}


ReturnCode copy_ndint(int* from, int** to, int* deffrom, int* defto)
{
   if (from == deffrom)
      *to = defto;
   else
   {
      if (from == NULL)
         *to = NULL;
      else
      {
         if ((*to = (int*)simm_malloc(sizeof(int))) == NULL)
            return code_bad;
         **to = *from;
      }
   }

   return code_fine;
}

WorldObject* copy_world_objects(WorldObject from[], int num)
{
   int i;
   WorldObject* to = NULL;

   if (num <= 0)
      return to;

   to = (WorldObject*)simm_malloc(sizeof(WorldObject)*num);
   memcpy(to, from, sizeof(WorldObject)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      mstrcpy(&to[i].filename, from[i].filename);
      if (from[i].wobj != NULL)
      {
         to[i].wobj = (PolyhedronStruct*)simm_malloc(sizeof(PolyhedronStruct));
         copy_polyhedron(from[i].wobj, to[i].wobj);
      }
   }

   return to;
}

JointStruct* copy_joints(ModelStruct* model, JointStruct from[], int num)
{
   int i, j;
   JointStruct* to = NULL;

   if (num <= 0)
      return to;

   to = (JointStruct*)simm_malloc(sizeof(JointStruct)*num);
   memcpy(to, from, sizeof(JointStruct)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      mstrcpy(&to[i].sd_name, from[i].sd_name);
      mstrcpy(&to[i].solverType, from[i].solverType);
      for (j=0; j<6; j++)
      {
         mstrcpy(&to[i].dofs[j].sd.name, from[i].dofs[j].sd.name);
         mstrcpy(&to[i].dofs[j].sd.con_name, from[i].dofs[j].sd.con_name);
            if (from[i].dofs[j].function)
                to[i].dofs[j].function = getFunctionByUserNumber(model, from[i].dofs[j].function->usernum);
            if (from[i].dofs[j].gencoord)
                to[i].dofs[j].gencoord = enter_gencoord(model, from[i].dofs[j].gencoord->name, no);
      }
      if (model->numsegments > 0)
      {
         to[i].in_seg_ground_path = (SBoolean*)simm_malloc(model->numsegments*sizeof(SBoolean));
         memcpy(to[i].in_seg_ground_path, from[i].in_seg_ground_path, model->numsegments*sizeof(SBoolean));
      }
#if INCLUDE_MOCAP_MODULE
      mstrcpy(&to[i].mocap_segment, from[i].mocap_segment);
#endif
   }

   return to;
}

SegmentStruct* copy_segments(SegmentStruct from[], int num)
{
   int i, j;
   SegmentStruct* to = NULL;

   if (num <= 0)
      return to;

   to = (SegmentStruct*)simm_malloc(sizeof(SegmentStruct)*num);
   memcpy(to, from, sizeof(SegmentStruct)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      if (from[i].numBones > 0)
      {
         to[i].bone = (PolyhedronStruct*)simm_malloc(sizeof(PolyhedronStruct)*to[i].numBones);
         for (j=0; j<to[i].numBones; j++)
            copy_polyhedron(&from[i].bone[j], &to[i].bone[j]);
      }
      to[i].boneArraySize = from[i].numBones;
      if (from[i].numgroups > 0)
      {
         to[i].group = (int*)simm_malloc(sizeof(int)*to[i].numgroups);
         memcpy(to[i].group, from[i].group, sizeof(int)*to[i].numgroups);
      }
      if (from[i].numSpringPoints > 0)
      {
         to[i].springPoint = (SpringPoint*)simm_malloc(sizeof(SpringPoint)*to[i].numSpringPoints);
         memcpy(to[i].springPoint, from[i].springPoint, sizeof(SpringPoint)*to[i].numSpringPoints);
         for (j=0; j<from[i].numSpringPoints; j++)
            mstrcpy(&to[i].springPoint[j].name, from[i].springPoint[j].name);
      }
      to[i].springPointArraySize = from[i].numSpringPoints;
      if (from[i].springFloor != NULL)
      {
         to[i].springFloor = (SpringFloor*)simm_malloc(sizeof(SpringFloor));
         memcpy(to[i].springFloor, from[i].springFloor, sizeof(SpringFloor));
         mstrcpy(&to[i].springFloor->name, from[i].springFloor->name);
         mstrcpy(&to[i].springFloor->filename, from[i].springFloor->filename);
         if (from[i].springFloor->poly != NULL)
         {
            to[i].springFloor->poly = (PolyhedronStruct*)simm_malloc(sizeof(PolyhedronStruct));
            copy_polyhedron(from[i].springFloor->poly, to[i].springFloor->poly);
         }
         to[i].springFloor->numPoints = 0;
         to[i].springFloor->points = NULL;
      }
      if (from[i].numContactObjects > 0)
      {
         to[i].contactObject = (ContactObject*)simm_malloc(sizeof(ContactObject)*to[i].numContactObjects);
         memcpy(to[i].contactObject, from[i].contactObject, sizeof(ContactObject)*to[i].numContactObjects);
         for (j=0; j<from[i].numContactObjects; j++)
         {
            mstrcpy(&to[i].contactObject[j].name, from[i].contactObject[j].name);
            mstrcpy(&to[i].contactObject[j].filename, from[i].contactObject[j].filename);
            if (from[i].contactObject[j].poly != NULL)
            {
               to[i].contactObject[j].poly = (PolyhedronStruct*)simm_malloc(sizeof(PolyhedronStruct));
               copy_polyhedron(from[i].contactObject[j].poly, to[i].contactObject[j].poly);
            }
         }
      }
      to[i].contactObjectArraySize = from[i].numContactObjects;
      if (from[i].forceMatte != NULL)
      {
         to[i].forceMatte = (ContactObject*)simm_malloc(sizeof(ContactObject));
         memcpy(to[i].forceMatte, from[i].forceMatte, sizeof(ContactObject));
         mstrcpy(&to[i].forceMatte->name, from[i].forceMatte->name);
         mstrcpy(&to[i].forceMatte->filename, from[i].forceMatte->filename);
         if (from[i].forceMatte->poly != NULL)
         {
            to[i].forceMatte->poly = (PolyhedronStruct*)simm_malloc(sizeof(PolyhedronStruct));
            copy_polyhedron(from[i].forceMatte->poly, to[i].forceMatte->poly);
         }
      }
      if (from[i].numMarkers > 0)
      {
         to[i].marker = (Marker**)simm_malloc(sizeof(Marker*) * from[i].numMarkers);
         for (j=0; j<from[i].numMarkers; j++)
         {
            to[i].marker[j] = (Marker*)simm_malloc(sizeof(Marker));
            memcpy(to[i].marker[j], from[i].marker[j], sizeof(Marker));
            mstrcpy(&to[i].marker[j]->name, from[i].marker[j]->name);
         }
      }
      else
      {
         // from[i].marker may be non-NULL even if from[i] has no markers.
         // So set to[i].marker to NULL so it doesn't keep a copy of the pointer.
         to[i].marker = NULL;
      }
      to[i].markerArraySize = from[i].numMarkers;
#if INCLUDE_BONE_EDITOR_EXTRAS
      to[i].pts_file = NULL;
      to[i].raw_vertices = NULL;
#endif
      if (from[i].num_deforms > 0)
      {
         to[i].deform = (DeformObject*)simm_malloc(sizeof(DeformObject)*from[i].num_deforms);
         memcpy(to[i].deform, from[i].deform, sizeof(DeformObject)*from[i].num_deforms);
         for (j=0; j<from[i].num_deforms; j++)
         {
            mstrcpy(&to[i].deform[j].name, from[i].deform[j].name);
            to[i].deform[j].innerBox = (float*)simm_malloc(sizeof(float)*144);
            memcpy(to[i].deform[j].innerBox, from[i].deform[j].innerBox, sizeof(float)*144);
            to[i].deform[j].innerBoxUndeformed = (float*)simm_malloc(sizeof(float)*144);
            memcpy(to[i].deform[j].innerBoxUndeformed, from[i].deform[j].innerBoxUndeformed, sizeof(float)*144);
            to[i].deform[j].outerBox = (float*)simm_malloc(sizeof(float)*144);
            memcpy(to[i].deform[j].outerBox, from[i].deform[j].outerBox, sizeof(float)*144);
            to[i].deform[j].outerBoxUndeformed = (float*)simm_malloc(sizeof(float)*144);
            memcpy(to[i].deform[j].outerBoxUndeformed, from[i].deform[j].outerBoxUndeformed, sizeof(float)*144);
         }
      }
      to[i].deform_obj_array_size = from[i].num_deforms;
#if INCLUDE_MOCAP_MODULE
      mstrcpy(&to[i].gait_scale_segment, from[i].gait_scale_segment);
      mstrcpy(&to[i].mocap_segment, from[i].mocap_segment);
      mstrcpy(&to[i].mocap_scale_chain_end1, from[i].mocap_scale_chain_end1);
      mstrcpy(&to[i].mocap_scale_chain_end2, from[i].mocap_scale_chain_end2);
#endif
   }

   return to;
}

void copy_menu(Menu* to, Menu* from)
{
   int i;

   memcpy(to, from, sizeof(Menu));
   mstrcpy(&to->title, from->title);
   if (from->numoptions > 0)
   {
      to->option = (MenuItem*)simm_malloc(sizeof(MenuItem)*from->numoptions);
      memcpy(to->option, from->option, sizeof(MenuItem)*from->numoptions);
      for (i=0; i<from->numoptions; i++)
         mstrcpy(&to->option[i].name, from->option[i].name);
   }
}

MuscleGroup* copy_muscle_groups(MuscleGroup from[], int num)
{
   int i;
   MuscleGroup* to = NULL;

   if (num <= 0)
      return to;

   to = (MuscleGroup*)simm_malloc(sizeof(MuscleGroup)*num);
   memcpy(to, from, sizeof(MuscleGroup)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      if (from[i].num_muscles > 0)
      {
         to[i].muscle_index = (int*)simm_malloc(sizeof(int)*from[i].num_muscles);
         memcpy(to[i].muscle_index, from[i].muscle_index, sizeof(int)*from[i].num_muscles);
      }
      to[i].muscindex_array_size = from[i].num_muscles;
      copy_menu(&to[i].menu, &from[i].menu);
   }

   return to;
}

SegmentGroup* copy_segment_groups(SegmentGroup from[], int num)
{
   int i;
   SegmentGroup* to = NULL;

   if (num <= 0)
      return to;

   to = (SegmentGroup*)simm_malloc(sizeof(SegmentGroup)*num);
   memcpy(to, from, sizeof(SegmentGroup)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      if (from[i].num_segments > 0)
      {
         to[i].segment = (int*)simm_malloc(sizeof(int)*from[i].num_segments);
         memcpy(to[i].segment, from[i].segment, sizeof(int)*from[i].num_segments);
      }
      to[i].seg_array_size = from[i].num_segments;
   }

   return to;
}

GencoordGroup* copy_gencoord_groups(GencoordGroup from[], int num)
{
   int i;
   GencoordGroup* to = NULL;

   if (num <= 0)
      return to;

   to = (GencoordGroup*)simm_malloc(sizeof(GencoordGroup)*num);
   memcpy(to, from, sizeof(GencoordGroup)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      if (from[i].num_gencoords > 0)
      {
         to[i].gencoord = (GeneralizedCoord**)simm_malloc(sizeof(GeneralizedCoord*) * from[i].num_gencoords);
         memcpy(to[i].gencoord, from[i].gencoord, sizeof(GeneralizedCoord*) * from[i].num_gencoords);
      }
      to[i].genc_array_size = from[i].num_gencoords;
   }

   return to;
}

GeneralizedCoord** copy_gencoords(ModelStruct* model, GeneralizedCoord* from[], int num)
{
   int i;
   GeneralizedCoord** to = NULL;

   if (num <= 0)
      return to;

   to = (GeneralizedCoord**)simm_malloc(sizeof(GeneralizedCoord*)*num);

   for (i=0; i<num; i++)
   {
      to[i] = (GeneralizedCoord*)simm_malloc(sizeof(GeneralizedCoord));
      memcpy(to[i], from[i], sizeof(GeneralizedCoord));
      mstrcpy(&to[i]->name, from[i]->name);
      if (from[i]->numjoints > 0)
      {
         to[i]->jointnum = (int*)simm_malloc(sizeof(int)*from[i]->numjoints);
         memcpy(to[i]->jointnum, from[i]->jointnum, sizeof(int)*from[i]->numjoints);
      }
      if (from[i]->numgroups > 0)
      {
         to[i]->group = (int*)simm_malloc(sizeof(int)*to[i]->numgroups);
         memcpy(to[i]->group, from[i]->group, sizeof(int)*to[i]->numgroups);
      }
        if (from[i]->restraint_function)
            to[i]->restraint_function = getFunctionByUserNumber(model, from[i]->restraint_function->usernum);
        if (from[i]->min_restraint_function)
            to[i]->min_restraint_function = getFunctionByUserNumber(model, from[i]->min_restraint_function->usernum);
        if (from[i]->max_restraint_function)
            to[i]->max_restraint_function = getFunctionByUserNumber(model, from[i]->max_restraint_function->usernum);

#if INCLUDE_MOCAP_MODULE
      mstrcpy(&to[i]->mocap_segment, from[i]->mocap_segment);
#endif
   }

   return to;
}

dpFunction** copy_functions(dpFunction* from[], int fromArraySize, int* toArraySize)
{
   int i, index = 0;
   dpFunction** to = NULL;

   *toArraySize = 0;
   for (i = 0; i < fromArraySize; i++)
      if (from[i] && from[i]->used == dpYes)
         (*toArraySize)++;

   if (*toArraySize <= 0)
      return to;

   to = (dpFunction**)simm_malloc(sizeof(dpFunction*)*(*toArraySize));

   for (i=0; i<fromArraySize; i++)
   {
      if (from[i] && from[i]->used == dpYes)
      {
         to[index] = (dpFunction*)simm_calloc(1, sizeof(dpFunction));
         malloc_function(to[index], from[i]->numpoints);
         copy_function(from[i], to[index]);
         index++;
      }
   }

   return to;
}

/* COPY_MUSCLES: copy all the muscles from one model to another */
ReturnCode copy_muscles(ModelStruct* from, ModelStruct* to)
{
   int i;

   if (from->default_muscle == NULL)
   {
      to->default_muscle = NULL;
   }
   else
   {
      to->default_muscle = (dpMuscleStruct*)simm_calloc(1, sizeof(dpMuscleStruct));
      if (copy_default_muscle(from->default_muscle, to->default_muscle, to) == code_bad)
         return code_bad;
   }

   to->nummuscles = from->nummuscles;

   if (to->nummuscles == 0)
   {
      to->muscle = NULL;
      to->muscle_array_size = 0;
      return code_fine;
   }

   // Malloc memory for the array of muscle pointers.
   to->muscle = (dpMuscleStruct**)simm_malloc(from->nummuscles*sizeof(dpMuscleStruct*));
   to->muscle_array_size = to->nummuscles;

   // Copy the muscle structures.
   for (i=0; i<from->nummuscles; i++)
   {
      to->muscle[i] = (dpMuscleStruct*)simm_calloc(1, sizeof(dpMuscleStruct));
      if (copy_muscle(from->muscle[i], to->muscle[i], from->default_muscle, to->default_muscle, to) == code_bad)
         return code_bad;
   }
   return code_fine;
}

/* COPY_LIGAMENTS: copy the ligament structure */
LigamentStruct* copy_ligaments(LigamentStruct from[], int num)
{
   int i, j;
   LigamentStruct* to = NULL;

   if (num <= 0)
      return to;

   to = (LigamentStruct*)simm_malloc(sizeof(LigamentStruct) * num);
   memcpy(to, from, sizeof(LigamentStruct) * num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      if (from[i].line)
      {
         to[i].line = (dpMusclePathStruct*)simm_calloc(from[i].numlines, sizeof(dpMusclePathStruct));
         for (j=0; j<from[i].numlines; j++)
         {
            if (copy_muscle_path(&from[i].line[j], &to[i].line[j]) == code_bad)
               return NULL;
         }
      }
      to[i].wrap_calced = no;
   }

   return to;
}

/* Copy only pointer-based sections of ModelDisplayStruct.*/
void copy_display_struct(ModelDisplayStruct* from, ModelDisplayStruct* to)
{
   int i;

   if (from == NULL || to == NULL)
      return;

   for (i=0; i<MAXSAVEDVIEWS; i++)
   {
      if (from->view_name[i]) {
         mstrcpy(&to->view_name[i], from->view_name[i]);
      } else {
         to->view_name[i] = NULL;
      }
   }
   to->applied_motion = NULL;
   to->current_motion = NULL;
   if (from->nummuscleson > 0 && from->muscleson != NULL)
   {
      to->muscleson = (int*)simm_malloc(sizeof(int)*from->nummuscleson);
      memcpy(to->muscleson, from->muscleson, sizeof(int)*from->nummuscleson);
   }
   to->devs = NULL;
   to->dev_values = NULL;

   if (from->mat.num_materials > 0 && from->mat.materials != NULL)
   {
      to->mat.materials = (MaterialStruct*)simm_calloc(from->mat.num_materials,sizeof(MaterialStruct));
      for (i=0; i<from->mat.num_materials; i++)
         copy_material(&from->mat.materials[i], &to->mat.materials[i]);
      to->mat.num_materials = from->mat.num_materials;
      to->mat.material_array_size = to->mat.num_materials;
   }
}

ContactPair* copy_contact_pairs(ContactPair from[], int num)
{
   int i;
   ContactPair* to = NULL;

   if (num <= 0)
      return to;

   to = (ContactPair*)simm_malloc(sizeof(ContactPair)*num);
   memcpy(to, from, sizeof(ContactPair)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].body1, from[i].body1);
      mstrcpy(&to[i].body2, from[i].body2);
   }

   return to;
}

ContactGroup* copy_contact_groups(ContactGroup from[], int num)
{
   int i, j;
   ContactGroup* to = NULL;

   if (num <= 0)
      return to;

   to = (ContactGroup*)simm_malloc(sizeof(ContactGroup)*num);
   memcpy(to, from, sizeof(ContactGroup)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      to[i].element = (char**)simm_malloc(sizeof(char*)*from[i].numElements);
      for (j=0; j<from[i].numElements; j++)
         mstrcpy(&to[i].element[j], to[i].element[j]);
   }

   return to;
}

MotionObject* copy_motion_objects(MotionObject from[], int num)
{
   int i;
   MotionObject* to = NULL;

   if (num <= 0)
      return to;

   to = (MotionObject*)simm_malloc(sizeof(MotionObject)*num);
   memcpy(to, from, sizeof(MotionObject)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      mstrcpy(&to[i].filename, from[i].filename);
      mstrcpy(&to[i].materialname, from[i].materialname);
      copy_polyhedron(&from[i].shape, &to[i].shape);
      to[i].shape.gl_display = 0;
   }

   return to;
}

dpWrapObject** copy_wrap_objects(dpWrapObject* from[], int num)
{
   int i;
   dpWrapObject** to = NULL;

   if (num <= 0)
      return to;

   to = (dpWrapObject**)simm_malloc(sizeof(dpWrapObject*)*num);

   for (i=0; i<num; i++)
   {
      to[i] = (dpWrapObject*)simm_malloc(sizeof(dpWrapObject));
      memcpy(to[i], from[i], sizeof(dpWrapObject));
      mstrcpy(&to[i]->name, from[i]->name);
   }

   return to;
}

Deformity* copy_deformities(Deformity from[], int num, ModelStruct* ms)
{
   int i, j;
   Deformity* to = NULL;

   if (num <= 0)
      return to;

   to = (Deformity*)simm_malloc(sizeof(Deformity)*num);
   memcpy(to, from, sizeof(Deformity)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      to[i].deform_name = (char**)simm_malloc(sizeof(char*)*from[i].num_deforms);
      to[i].deform = (DeformObject**)simm_malloc(sizeof(DeformObject*)*from[i].num_deforms);
      for (j=0; j<from[i].num_deforms; j++)
      {
         mstrcpy(&to[i].deform_name[j], from[i].deform_name[j]);
         to[i].deform[j] = lookup_deform(ms, from[i].deform_name[j]);
      }
   }

   return to;
}

ConstraintPoint* copy_constraint_points(ConstraintPoint from[], int num)
{
   int i;
   ConstraintPoint* to = NULL;

   if (num <= 0)
      return to;

   to = (ConstraintPoint*)simm_malloc(sizeof(ConstraintPoint)*num);
   memcpy(to, from, sizeof(ConstraintPoint)*num);

   for (i=0; i<num; i++)
      mstrcpy(&to[i].name, from[i].name);

   return to;
}

ConstraintObject* copy_constraint_objects(ConstraintObject from[], int num)
{
   int i;
   ConstraintObject* to = NULL;

   if (num <= 0)
      return to;

   to = (ConstraintObject*)simm_malloc(sizeof(ConstraintObject)*num);
   memcpy(to, from, sizeof(ConstraintObject)*num);

   for (i=0; i<num; i++)
   {
      mstrcpy(&to[i].name, from[i].name);
      to[i].points = copy_constraint_points(from[i].points, from[i].numPoints);
      to[i].cp_array_size = from[i].numPoints;
      to[i].joints = (int*)simm_malloc(sizeof(int)*from[i].num_jnts);
      memcpy(to[i].joints, from[i].joints, sizeof(int)*from[i].num_jnts);
      to[i].qs = (GeneralizedCoord**)simm_malloc(sizeof(GeneralizedCoord*)*from[i].num_qs);
      memcpy(to[i].qs, from[i].qs, sizeof(GeneralizedCoord*)*from[i].num_qs);
   }

   return to;
}

MotionSequence** copy_motions(MotionSequence* from[], int num)
{
   int i;
   MotionSequence** to = NULL;

   if (num <= 0)
      return to;

   to = (MotionSequence**)simm_malloc(sizeof(MotionSequence*)*num);

   for (i=0; i<num; i++)
      to[i] = copy_motion(from[i]);

   return to;
}

/* COPY_MOTION: does not copy all of a motion. In particular,
 * the mopt is copied only with memcpy().
 */
static MotionSequence* copy_motion(MotionSequence* from)
{
   int j;
   MotionSequence* to = NULL;

   if (from == NULL)
      return to;

   to = (MotionSequence*)simm_malloc(sizeof(MotionSequence));

   memcpy(to, from, sizeof(MotionSequence));
   mstrcpy(&to->name, from->name);
   if (from->deriv_names)
   {
      to->deriv_names = (char**)simm_malloc(sizeof(char*)*from->number_of_datacolumns);
      for (j=0; j<from->number_of_datacolumns; j++)
         mstrcpy(&to->deriv_names[j], from->deriv_names[j]);
   }
   if (from->deriv_data)
   {
      to->deriv_data = (double**)simm_malloc(sizeof(double*)*from->number_of_datacolumns);
      for (j=0; j<from->number_of_datacolumns; j++)
      {
         to->deriv_data[j] = (double*)simm_malloc(sizeof(double)*from->number_of_datarows);
         memcpy(to->deriv_data[j], from->deriv_data[j], sizeof(double)*from->number_of_datarows);
      }
   }
   mstrcpy(&to->units, from->units);
   if (from->columnname)
   {
      to->columnname = (char**)simm_malloc(sizeof(char*)*from->number_of_datacolumns);
      for (j=0; j<from->number_of_datacolumns; j++)
         mstrcpy(&to->columnname[j], from->columnname[j]);
   }
   if (from->motiondata)
   {
      to->motiondata = (double**)simm_malloc(sizeof(double*)*from->number_of_datacolumns);
      for (j=0; j<from->number_of_datacolumns; j++)
      {
         to->motiondata[j] = (double*)simm_malloc(sizeof(double)*from->number_of_datarows);
         memcpy(to->motiondata[j], from->motiondata[j], sizeof(double)*from->number_of_datarows);
      }
   }
   if (from->data_std_dev)
   {
      to->data_std_dev = (double**)simm_malloc(sizeof(double*)*from->number_of_datacolumns);
      for (j=0; j<from->number_of_datacolumns; j++)
      {
         if (from->data_std_dev[j])
         {
            to->data_std_dev[j] = (double*)simm_malloc(sizeof(double)*from->number_of_datarows);
            memcpy(to->data_std_dev[j], from->data_std_dev[j], sizeof(double)*from->number_of_datarows);
         }
         else
         {
            to->data_std_dev[j] = NULL;
         }
      }
   }
   if (from->num_events > 0)
   {
      to->event = (smMotionEvent*)simm_malloc(sizeof(smMotionEvent)*from->num_events);
      memcpy(to->event, from->event, sizeof(smMotionEvent)*from->num_events);
      for (j=0; j<from->num_events; j++)
         mstrcpy(&to->event[j].name, from->event[j].name);
   }

   return to;
}

ModelStruct* copy_model(ModelStruct* ms)
{
   int i;

   ModelStruct* copy = (ModelStruct*)simm_malloc(sizeof(ModelStruct));
   memcpy(copy, ms, sizeof(ModelStruct));
   memset(&copy->save, 0, sizeof(ModelSave)); // saved version is not copied

   mstrcpy(&copy->name, ms->name);
   mstrcpy(&copy->forceUnits, ms->forceUnits);
   mstrcpy(&copy->lengthUnits, ms->lengthUnits);
   mstrcpy(&copy->HTRtranslationUnits, ms->HTRtranslationUnits);

    for (i=0; i<2*GENBUFFER; i++)
        if (ms->genc_help[i].text)
            mstrcpy(&copy->genc_help[i].text, ms->genc_help[i].text);

   // copy pathptrs
   copy->pathptrs = NULL;
   // copy segment_drawing_order
   copy->segment_drawing_order = NULL;
   // copy gencslider
   copy->gencslider.sl = NULL;
   copy->gencslider.numsliders = 0;
   // copy gencform
   copy->gencform.title = NULL;
   copy->gencform.option = NULL;
   copy->gencform.numoptions = 0;
   // copy gc_chpanel
   copy->gc_chpanel.title = NULL;
   copy->gc_chpanel.checkbox = NULL;
   copy->gc_chpanel.numoptions = 0;
   // copy gc_lockPanel
   copy->gc_lockPanel.title = NULL;
   copy->gc_lockPanel.checkbox = NULL;
   copy->gc_lockPanel.numoptions = 0;
   // copy dynparamsform
   copy->dynparamsform.title = NULL;
   copy->dynparamsform.option = NULL;
   copy->dynparamsform.numoptions = 0;

   mstrcpy(&copy->jointfilename, ms->jointfilename);
   mstrcpy(&copy->bonepathname, ms->bonepathname);
   mstrcpy(&copy->musclefilename, ms->musclefilename);
   for (i=0; i<copy->num_motion_files; i++)
      mstrcpy(&copy->motionfilename[i], ms->motionfilename[i]);
   mstrcpy(&copy->mocap_dir, ms->mocap_dir);

    // Functions must be copied first so that all components that use them can
    // get the appropriate pointer from the copied function array.
   copy->function = copy_functions(ms->function, ms->func_array_size, &copy->func_array_size);

    // Gencoords must be copied before joints because DOFs contain pointers
    // to gencoords.
   copy->gencgroup = copy_gencoord_groups(ms->gencgroup, ms->numgencgroups);
   copy->gencgroup_array_size = ms->numgencgroups;
   copy->gencoord = copy_gencoords(copy, ms->gencoord, ms->numgencoords);
   copy->genc_array_size = ms->numgencoords;

   copy->worldobj = copy_world_objects(ms->worldobj, ms->numworldobjects);
   copy->world_array_size = ms->numworldobjects;

    // Segments have to be copied before joints, because of joint->in_seg_ground_path.
   copy->seggroup = copy_segment_groups(ms->seggroup, ms->numseggroups);
   copy->seggroup_array_size = ms->numseggroups;
   copy->segment = copy_segments(ms->segment, ms->numsegments);
   copy->segment_array_size = ms->numsegments;

   copy->joint = copy_joints(copy, ms->joint, ms->numjoints);
   copy->joint_array_size = ms->numjoints;

   copy->muscgroup = copy_muscle_groups(ms->muscgroup, ms->numgroups);
   copy->muscgroup_array_size = ms->numgroups;
   copy_muscles(ms, copy);

   copy->ligament = copy_ligaments(ms->ligament, ms->numligaments);
   copy->ligament_array_size = ms->numligaments;

   copy_display_struct(&ms->dis, &copy->dis);

   copy->contactPair = copy_contact_pairs(ms->contactPair, ms->numContactPairs);
   copy->contactPairArraySize = ms->numContactPairs;

   copy->contactGroup = copy_contact_groups(ms->contactGroup, ms->numContactGroups);
   copy->contactGroupArraySize = ms->numContactGroups;

   copy->motion_objects = copy_motion_objects(ms->motion_objects, ms->num_motion_objects);

   copy->wrapobj = copy_wrap_objects(ms->wrapobj, ms->num_wrap_objects);
   copy->wrap_object_array_size = ms->num_wrap_objects;

   copy->deformity = copy_deformities(ms->deformity, ms->num_deformities, ms);
   copy->deformity_array_size = ms->num_deformities;

   // copy loop

   copy->constraintobj = copy_constraint_objects(ms->constraintobj, ms->num_constraint_objects);
   copy->constraint_object_array_size = ms->num_constraint_objects;

   copy->motion = copy_motions(ms->motion, ms->num_motions);
   copy->motion_array_size = ms->num_motions;

   check_definitions(copy);

   return copy;
}

static ReturnCode copy_nondefault_dyn_params(dpMuscleStruct* from, dpMuscleStruct* to,
                                                  dpMuscleStruct* deffrom, dpMuscleStruct* defto)
{
   int i;

   to->num_dynamic_params = from->num_dynamic_params;

   if (to->num_dynamic_params > 0)
   {
      // A muscle's param name array is always the same as its default muscle's.
      to->dynamic_param_names = defto->dynamic_param_names;
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


/* This function copies the parameter names and all of the non-null values.
 */
static ReturnCode copy_nonnull_dyn_params(dpMuscleStruct* from, dpMuscleStruct* to)
{
   to->num_dynamic_params = from->num_dynamic_params;

   if (to->num_dynamic_params > 0)
   {
      int i;

      to->dynamic_param_names = (char**)simm_malloc(to->num_dynamic_params * sizeof(char*));
      if (to->dynamic_param_names == NULL)
         return code_bad;

      for (i=0; i<to->num_dynamic_params; i++)
         mstrcpy(&to->dynamic_param_names[i], from->dynamic_param_names[i]);

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


ReturnCode alloc_func(dpFunction** func, int size)
{

   if ((*func = (dpFunction*)simm_malloc(sizeof(dpFunction))) == NULL)
      return code_bad;
   if (((*func)->x = (double*)simm_malloc(size*sizeof(double))) == NULL)
      return code_bad;
   if (((*func)->y = (double*)simm_malloc(size*sizeof(double))) == NULL)
      return code_bad;
   if (((*func)->b = (double*)simm_malloc(size*sizeof(double))) == NULL)
      return code_bad;
   if (((*func)->c = (double*)simm_malloc(size*sizeof(double))) == NULL)
      return code_bad;
   if (((*func)->d = (double*)simm_malloc(size*sizeof(double))) == NULL)
      return code_bad;

   return code_fine;

}

/* Given a gencoord struct, return the QStruct that has the same name */
static dpQStruct* get_q_from_gencoord_dp(dpModelStruct* model, GeneralizedCoord *genc)
{
    int i;

   if (model->q == NULL)
      return NULL;

    for (i=0; i<model->nq; i++)
    {
      if (&model->q[i] == NULL)
         return NULL;
        if (STRINGS_ARE_EQUAL(genc->name, model->q[i].name))
            return &model->q[i];
    }

    return NULL;
}
