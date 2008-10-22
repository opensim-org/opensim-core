/*******************************************************************************

   FREE.C

   Author: Peter Loan

   Date: 07-MAY-90

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Description:

   Routines:
      free_model       : frees a model structure
      free_plot        : frees a plot structure
      nullify_function : frees all space malloced for a function
      free_function    : frees the space malloced inside a function structure
      free_muscles       : frees memory malloced for muscle structure elements
      free_musclepoints       : frees memory malloced for muscle point structure elements
      free_defmusc     : frees memory malloced for defaultmuscle elements
      free_and_nullify : frees memory and sets the pointer to NULL
      free_menu        : frees a menu structure
      free_form        : frees a form structure

*******************************************************************************/

#include "universal.h"
#include "globals.h"
#include "functions.h"


/*************** DEFINES (for this file only) *********************************/
	


/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/



/*************** EXTERNED VARIABLES (declared in another file) ****************/
#ifdef WIN32
#if ! OPENSIM_BUILD
extern ModelStruct* sMotionModel;
#endif
#endif

/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/


void free_model(int mod)
{
   if (model[mod] != NULL)
   {
#ifdef WIN32
#if ! OPENSIM_BUILD
      if (sMotionModel == model[mod])
         sMotionModel = NULL;
#endif
#endif
      freeModelStruct(model[mod]);
      model[mod] = NULL;
   }
}

void freeModelStruct(ModelStruct* ms)
{
   int i, j;

   FREE_IFNOTNULL(ms->name);
   FREE_IFNOTNULL(ms->pathptrs);
   FREE_IFNOTNULL(ms->jointfilename);
   FREE_IFNOTNULL(ms->musclefilename);

   for (i=0; i<ms->numgroups; i++)
   {
      FREE_IFNOTNULL(ms->muscgroup[i].name);
      free_menu(&ms->muscgroup[i].menu);
   }
   FREE_IFNOTNULL(ms->muscgroup);

   free_form(&ms->gencform);

   for (i=0; i<ms->numjoints; i++)
   {
      FREE_IFNOTNULL(ms->joint[i].name);
      FREE_IFNOTNULL(ms->joint[i].solverType);
      FREE_IFNOTNULL(ms->joint[i].in_seg_ground_path);
#if INCLUDE_MOCAP_MODULE
      FREE_IFNOTNULL(ms->joint[i].mocap_segment);
#endif
   }
   FREE_IFNOTNULL(ms->joint);

   for (i=0; i<ms->numsegments; i++)
   {
      if (ms->segment[i].defined == no)
         continue;
      FREE_IFNOTNULL(ms->segment[i].name);
      for (j=0; j<ms->segment[i].numBones; j++)
      {
         FREE_IFNOTNULL(ms->segment[i].bone[j].name);
         /*
         FREE_IFNOTNULL(ms->segment[i].bone[j].vert);
         FREE_IFNOTNULL(ms->segment[i].bone[j].poly);
         */
      }
      for (j=0; j<ms->segment[i].numSpringPoints; j++)
      {
         FREE_IFNOTNULL(ms->segment[i].springPoint[j].name);
         FREE_IFNOTNULL(ms->segment[i].springPoint[j].floorName);
      }
      FREE_IFNOTNULL(ms->segment[i].springPoint);

      if (ms->segment[i].springFloor)
      {
         FREE_IFNOTNULL(ms->segment[i].springFloor->name);
         FREE_IFNOTNULL(ms->segment[i].springFloor->filename);
         FREE_IFNOTNULL(ms->segment[i].springFloor->poly);
         FREE_IFNOTNULL(ms->segment[i].springFloor->points);
         FREE_IFNOTNULL(ms->segment[i].springFloor);
      }

      for (j=0; j<ms->segment[i].numContactObjects; j++)
      {
         FREE_IFNOTNULL(ms->segment[i].contactObject[j].name);
         FREE_IFNOTNULL(ms->segment[i].contactObject[j].filename);
         FREE_IFNOTNULL(ms->segment[i].contactObject[j].poly);
      }
      FREE_IFNOTNULL(ms->segment[i].contactObject);

      FREE_IFNOTNULL(ms->segment[i].marker);

#if INCLUDE_MOCAP_MODULE
      FREE_IFNOTNULL(ms->segment[i].gait_scale_segment);
      FREE_IFNOTNULL(ms->segment[i].mocap_segment);
      FREE_IFNOTNULL(ms->segment[i].mocap_scale_chain_end1);
      FREE_IFNOTNULL(ms->segment[i].mocap_scale_chain_end2);
#endif
   }
   FREE_IFNOTNULL(ms->segment);
   FREE_IFNOTNULL(ms->wrapobj);

   for (i=0; i<ms->numgencoords; i++)
   {
      if (ms->gencoord[i].defined == yes)
      {
	      FREE_IFNOTNULL(ms->gencoord[i].name);
	      FREE_IFNOTNULL(ms->gencoord[i].jointnum);
#if INCLUDE_MOCAP_MODULE
         FREE_IFNOTNULL(ms->gencoord[i].mocap_segment);
#endif
      }
   }
   FREE_IFNOTNULL(ms->gencoord);

   for (i = 0; i < ms->num_deformities; i++)
   {
      if (ms->deformity[i].deform_name)
      {
         for (j = 0; j < ms->deformity[i].num_deforms; j++)
            FREE_IFNOTNULL(ms->deformity[i].deform_name[j]);

         FREE_IFNOTNULL(ms->deformity[i].deform_name);
      }

      FREE_IFNOTNULL(ms->deformity[i].deform);
   }

#ifndef ENGINE

   /* NOTE: the model's window *must* be set as the current GL context, otherwise
    *  the glDeleteList() calls below will produce unpredictable bad results.
    */
   for (i=0; i<ms->dis.mat.num_materials; i++)
   {
      if (ms->dis.mat.materials[i].normal_list != -1)
         glDeleteLists(ms->dis.mat.materials[i].normal_list,1);
      if (ms->dis.mat.materials[i].highlighted_list != -1)
         glDeleteLists(ms->dis.mat.materials[i].highlighted_list,1);
      FREE_IFNOTNULL(ms->dis.mat.materials[i].name);
   }
   FREE_IFNOTNULL(ms->dis.mat.materials);

   free_muscles(ms->muscle,&ms->default_muscle,ms->nummuscles);
   //free_musclepoints(ms->musclepoints, ms->nummusclepoints)

   free_defmusc(&ms->default_muscle);
#endif

   for (i=0; i<ms->func_array_size; i++)
      if (ms->function[i].defined == yes)
	 nullify_function(&ms->function[i], no);

   FREE_IFNOTNULL(ms->function);

#ifndef ENGINE
   free_savedmuscles(ms->save.muscle,&ms->save.default_muscle, ms->save.numsavedmuscs);
   free_savedmusclepaths(ms->save.musclepath, ms->save.numsavedpaths);
   free_defmusc(&ms->save.default_muscle);

   for (i = 0; i < ms->num_motion_objects; i++)
      free_motion_object(&ms->motion_objects[i], ms);
   
   FREE_IFNOTNULL(ms->motion_objects);
#endif
   
/*
   for (i=0; i<ms->numjoints; i++)
   {
      FREE_IFNOTNULL(ms->save.jnts[i].name);
      for (j=0; j<6; j++)
	 FREE_IFNOTNULL(ms->save.jnts[i].dofs[j].element);
   }
   FREE_IFNOTNULL(ms->save.jnts);
   for (i=0; i<ms->numgencoords; i++)
      FREE_IFNOTNULL(ms->save.gencs[i].name);
   FREE_IFNOTNULL(ms->save.gencs);
*/

   FREE_IFNOTNULL(ms);
}

#ifndef ENGINE
void free_plot(int plotnum)
{

   int i;

   FREE_IFNOTNULL(plot[plotnum]->title);
   FREE_IFNOTNULL(plot[plotnum]->xname);
   /* JPL 11/2/00 TODO: for some reason, freeing the yname is causing
    * a crash, so remove it for now.
    */
/*   FREE_IFNOTNULL(plot[plotnum]->yname);*/

   for (i=0; i<plot[plotnum]->numcurves; i++)
   {
      FREE_IFNOTNULL(plot[plotnum]->curve[i]->xvalues);
      FREE_IFNOTNULL(plot[plotnum]->curve[i]->yvalues);
      FREE_IFNOTNULL(plot[plotnum]->curve[i]->name);
      FREE_IFNOTNULL(plot[plotnum]->curve[i]);
   }

   FREE_IFNOTNULL(plot[plotnum]);

   plot[plotnum] = NULL;

}
#endif


void nullify_function(SplineFunction* func, SBoolean freeTheFuncToo)
{

   if (func == NULL)
      return;

   FREE_IFNOTNULL(func->x);
   FREE_IFNOTNULL(func->y);
   FREE_IFNOTNULL(func->b);
   FREE_IFNOTNULL(func->c);
   FREE_IFNOTNULL(func->d);

   if (freeTheFuncToo)
      free(func);
}



void free_function(SplineFunction* func)
{

   if (func == NULL || func->used == no || func->defined == no)
      return;

   FREE_IFNOTNULL(func->x);
   FREE_IFNOTNULL(func->y);
   FREE_IFNOTNULL(func->b);
   FREE_IFNOTNULL(func->c);
   FREE_IFNOTNULL(func->d);

   func->used = func->defined = no;

}

/* FREE_MUSCS: frees the muscle structure, but does NOT free the musclepoints or the saved_copy
 * since these may be used elswhere */

void free_muscle(MuscleStruct *muscle, MuscleStruct* dm)
{
   int i, j;

   if (muscle == NULL)
      return;

   if (muscle->name != dm->name && muscle->name != NULL)
      FREE_IFNOTNULL(muscle->name);

   if (muscle->group != dm->group && muscle->group != NULL)
      FREE_IFNOTNULL(muscle->group);
   if (muscle->max_isometric_force != dm->max_isometric_force && muscle->max_isometric_force != NULL)
      FREE_IFNOTNULL(muscle->max_isometric_force);
   if (muscle->pennation_angle != dm->pennation_angle && muscle->pennation_angle != NULL)
      FREE_IFNOTNULL(muscle->pennation_angle);
   if (muscle->min_thickness != dm->min_thickness && muscle->min_thickness != NULL)
      FREE_IFNOTNULL(muscle->min_thickness);
   if (muscle->max_thickness != dm->max_thickness && muscle->max_thickness != NULL)
      FREE_IFNOTNULL(muscle->max_thickness);
   if (muscle->min_material != dm->min_material && muscle->min_material != NULL) //dkb apr 2008 
      FREE_IFNOTNULL(muscle->min_material);
   if (muscle->max_material != dm->max_material && muscle->max_material != NULL) //dkb apr 2008
      FREE_IFNOTNULL(muscle->max_material);
   if (muscle->max_contraction_vel != dm->max_contraction_vel && muscle->max_contraction_vel != NULL)
      FREE_IFNOTNULL(muscle->max_contraction_vel);
   if (muscle->force_vel_curve != dm->force_vel_curve)
      nullify_function(muscle->force_vel_curve, yes);
   if (muscle->optimal_fiber_length != dm->optimal_fiber_length && muscle->optimal_fiber_length != NULL)
      FREE_IFNOTNULL(muscle->optimal_fiber_length);
   if (muscle->resting_tendon_length != dm->resting_tendon_length && muscle->resting_tendon_length != NULL)
      FREE_IFNOTNULL(muscle->resting_tendon_length);
   if (muscle->momentarms != dm->momentarms && muscle->momentarms != NULL)
      FREE_IFNOTNULL(muscle->momentarms);
   if (muscle->active_force_len_curve != dm->active_force_len_curve)
      nullify_function(muscle->active_force_len_curve, yes);
   if (muscle->passive_force_len_curve != dm->passive_force_len_curve)
      nullify_function(muscle->passive_force_len_curve, yes);
   if (muscle->tendon_force_len_curve != dm->tendon_force_len_curve)
      nullify_function(muscle->tendon_force_len_curve, yes);
   if (muscle->wrapStruct)
   {
      for (j = 0; j < muscle->numWrapStructs; j++)
      {
         FREE_IFNOTNULL(muscle->wrapStruct[j]->mp_wrap[0].wrap_pts);
         FREE_IFNOTNULL(muscle->wrapStruct[j]->mp_wrap[1].wrap_pts);
         FREE_IFNOTNULL(muscle->wrapStruct[j]);
      }
      FREE_IFNOTNULL(muscle->wrapStruct);
   }
   if (muscle->muscle_model_index != dm->muscle_model_index && muscle->muscle_model_index != NULL) //dkb apr 2008 
      FREE_IFNOTNULL(muscle->muscle_model_index);

   if (muscle->dynamic_params)
   {
      for (j = 0; j < muscle->num_dynamic_params; j++)
      {
         if (muscle->dynamic_params[j] != dm->dynamic_params[j] && muscle->dynamic_params[j] != NULL)
            FREE_IFNOTNULL(muscle->dynamic_params[j]);
      }
   }

}

/* FREE_MUSCS: frees the muscle structure, but does NOT free the musclepoints or the saved_copy
 * since these may be used elswhere */

void free_muscles(MuscleStruct musc[], MuscleStruct* dm, int num)
{

   int i, j;

   if (musc == NULL)
      return;

   for (i=0; i<num; i++)
   {
      if (musc[i].name != dm->name && musc[i].name != NULL)
         FREE_IFNOTNULL(musc[i].name);

      if (musc[i].group != dm->group && musc[i].group != NULL)
         FREE_IFNOTNULL(musc[i].group);
      if (musc[i].max_isometric_force != dm->max_isometric_force && musc[i].max_isometric_force != NULL)
         FREE_IFNOTNULL(musc[i].max_isometric_force);
      if (musc[i].pennation_angle != dm->pennation_angle && musc[i].pennation_angle != NULL)
         FREE_IFNOTNULL(musc[i].pennation_angle);
      if (musc[i].min_thickness != dm->min_thickness && musc[i].min_thickness != NULL)
         FREE_IFNOTNULL(musc[i].min_thickness);
      if (musc[i].max_thickness != dm->max_thickness && musc[i].max_thickness != NULL)
         FREE_IFNOTNULL(musc[i].max_thickness);
      if (musc[i].min_material != dm->min_material && musc[i].min_material != NULL) //dkb apr 2008 
         FREE_IFNOTNULL(musc[i].min_material);
      if (musc[i].max_material != dm->max_material && musc[i].max_material != NULL) //dkb apr 2008
         FREE_IFNOTNULL(musc[i].max_material);
      if (musc[i].max_contraction_vel != dm->max_contraction_vel && musc[i].max_contraction_vel != NULL)
         FREE_IFNOTNULL(musc[i].max_contraction_vel);
      if (musc[i].force_vel_curve != dm->force_vel_curve)
         nullify_function(musc[i].force_vel_curve, yes);
      if (musc[i].optimal_fiber_length != dm->optimal_fiber_length && musc[i].optimal_fiber_length != NULL)
         FREE_IFNOTNULL(musc[i].optimal_fiber_length);
      if (musc[i].resting_tendon_length != dm->resting_tendon_length && musc[i].resting_tendon_length != NULL)
         FREE_IFNOTNULL(musc[i].resting_tendon_length);
      if (musc[i].momentarms != dm->momentarms && musc[i].momentarms != NULL)
         FREE_IFNOTNULL(musc[i].momentarms);
      if (musc[i].active_force_len_curve != dm->active_force_len_curve)
         nullify_function(musc[i].active_force_len_curve, yes);
      if (musc[i].passive_force_len_curve != dm->passive_force_len_curve)
         nullify_function(musc[i].passive_force_len_curve, yes);
      if (musc[i].tendon_force_len_curve != dm->tendon_force_len_curve)
         nullify_function(musc[i].tendon_force_len_curve, yes);
      if (musc[i].wrapStruct)
      {
         for (j = 0; j < musc[i].numWrapStructs; j++)
         {
            FREE_IFNOTNULL(musc[i].wrapStruct[j]->mp_wrap[0].wrap_pts);
            FREE_IFNOTNULL(musc[i].wrapStruct[j]->mp_wrap[1].wrap_pts);
            FREE_IFNOTNULL(musc[i].wrapStruct[j]);
         }
         FREE_IFNOTNULL(musc[i].wrapStruct);
      }
      if (musc[i].muscle_model_index != dm->muscle_model_index && musc[i].muscle_model_index != NULL) //dkb apr 2008 
         FREE_IFNOTNULL(musc[i].muscle_model_index);

      if (musc[i].dynamic_params)
      {
         for (j = 0; j < musc[i].num_dynamic_params; j++)
         {
            if (musc[i].dynamic_params[j] != dm->dynamic_params[j] && musc[i].dynamic_params[j] != NULL)
               FREE_IFNOTNULL(musc[i].dynamic_params[j]);
         }
      }
      // DKB to do - free dynamic parameters ??
   }

   FREE_IFNOTNULL(musc);
}

/* FREE_MUSCLEPOINTS: */

void free_musclepoints(MusclePoint mp[], int num)
{
   int i, j;

   if (mp == NULL)
      return;

/*   for (i=0; i<num; i++)
   {
      if (mp[i].num_orig_points != NULL)
      {
         for (j=0; j<*mp[i].num_orig_points; j++)
         {
            if (mp[i].musclepoints->mp_orig[j].numranges > 0 && mp[i].musclepoints->mp_orig[j].ranges != NULL)
               FREE_IFNOTNULL(mp[i].musclepoints->mp_orig[j].ranges);
         }
         FREE_IFNOTNULL(mp[i].num_orig_points);
      }
      if (mp[i].musclepoints->mp_orig != NULL)
         FREE_IFNOTNULL(mp[i].musclepoints->mp_orig);

   }
*/
   FREE_IFNOTNULL(mp);
}



/* FREE_DEFMUSC: */

void free_defmusc(MuscleStruct* dm)
{

   int i;

   free_and_nullify((void**)&dm->name);
   free_and_nullify((void**)&dm->group);
   free_and_nullify((void**)&dm->max_isometric_force);
   free_and_nullify((void**)&dm->pennation_angle);
   free_and_nullify((void**)&dm->min_thickness);
   free_and_nullify((void**)&dm->max_thickness);
   free_and_nullify((void**)&dm->min_material);//dm->min_material = NULL; // why not free and nullify??
   free_and_nullify((void**)&dm->max_material);//dm->max_material = NULL;
   free_and_nullify((void**)&dm->muscle_model_index);
   free_and_nullify((void**)&dm->max_contraction_vel);
   nullify_function(dm->force_vel_curve, yes);
   dm->force_vel_curve = NULL;

   free_and_nullify((void**)&dm->optimal_fiber_length);
   free_and_nullify((void**)&dm->resting_tendon_length);
   free_and_nullify((void**)&dm->momentarms);

   nullify_function(dm->active_force_len_curve, yes);
   dm->active_force_len_curve = NULL;
   nullify_function(dm->passive_force_len_curve, yes);
   dm->passive_force_len_curve = NULL;
   nullify_function(dm->tendon_force_len_curve, yes);
   dm->tendon_force_len_curve = NULL;

   // dkb to do free dynamic parameters, moment arms, excitation format
}

/* FREE_MUSCS: */
void free_savedmuscles(SaveMuscle musc[], MuscleStruct* dm, int num)
{
   int i, j;

   if (musc == NULL)
      return;

   for (i=0; i<num; i++)
   {
      if (musc[i].name != dm->name && musc[i].name != NULL)
         FREE_IFNOTNULL(musc[i].name);

      if (musc[i].group != dm->group && musc[i].group != NULL)
         FREE_IFNOTNULL(musc[i].group);
      if (musc[i].max_isometric_force != dm->max_isometric_force && musc[i].max_isometric_force != NULL)
         FREE_IFNOTNULL(musc[i].max_isometric_force);
      if (musc[i].pennation_angle != dm->pennation_angle && musc[i].pennation_angle != NULL)
         FREE_IFNOTNULL(musc[i].pennation_angle);
      if (musc[i].min_thickness != dm->min_thickness && musc[i].min_thickness != NULL)
         FREE_IFNOTNULL(musc[i].min_thickness);
      if (musc[i].max_thickness != dm->max_thickness && musc[i].max_thickness != NULL)
         FREE_IFNOTNULL(musc[i].max_thickness);
      if (musc[i].max_contraction_vel != dm->max_contraction_vel && musc[i].max_contraction_vel != NULL)
         FREE_IFNOTNULL(musc[i].max_contraction_vel);
      if (musc[i].force_vel_curve != dm->force_vel_curve)
         nullify_function(musc[i].force_vel_curve, yes);
      if (musc[i].optimal_fiber_length != dm->optimal_fiber_length && musc[i].optimal_fiber_length != NULL)
         FREE_IFNOTNULL(musc[i].optimal_fiber_length);
      if (musc[i].resting_tendon_length != dm->resting_tendon_length && musc[i].resting_tendon_length != NULL)
         FREE_IFNOTNULL(musc[i].resting_tendon_length);
      if (musc[i].momentarms != dm->momentarms && musc[i].momentarms != NULL)
         FREE_IFNOTNULL(musc[i].momentarms);
      if (musc[i].active_force_len_curve != dm->active_force_len_curve)
         nullify_function(musc[i].active_force_len_curve, yes);
      if (musc[i].passive_force_len_curve != dm->passive_force_len_curve)
         nullify_function(musc[i].passive_force_len_curve, yes);
      if (musc[i].tendon_force_len_curve != dm->tendon_force_len_curve)
         nullify_function(musc[i].tendon_force_len_curve, yes);
      if (musc[i].wrapStruct)
      {
         for (j = 0; j < musc[i].numWrapStructs; j++)
         {
            FREE_IFNOTNULL(musc[i].wrapStruct[j]->mp_wrap[0].wrap_pts);
            FREE_IFNOTNULL(musc[i].wrapStruct[j]->mp_wrap[1].wrap_pts);
            FREE_IFNOTNULL(musc[i].wrapStruct[j]);
         }
         FREE_IFNOTNULL(musc[i].wrapStruct);
      }
   }

   FREE_IFNOTNULL(musc);
}

/* FREE_MUSCS: */
void free_savedmuscle(SaveMuscle *saved, MuscleStruct* dm)
{
   int i, j;

   if (saved == NULL)
      return;

   if (saved->name != dm->name && saved->name != NULL)
      FREE_IFNOTNULL(saved->name);

   if (saved->group != dm->group && saved->group != NULL)
      FREE_IFNOTNULL(saved->group);
   if (saved->max_isometric_force != dm->max_isometric_force && saved->max_isometric_force != NULL)
      FREE_IFNOTNULL(saved->max_isometric_force);
   if (saved->pennation_angle != dm->pennation_angle && saved->pennation_angle != NULL)
      FREE_IFNOTNULL(saved->pennation_angle);
   if (saved->min_thickness != dm->min_thickness && saved->min_thickness != NULL)
      FREE_IFNOTNULL(saved->min_thickness);
   if (saved->max_thickness != dm->max_thickness && saved->max_thickness != NULL)
      FREE_IFNOTNULL(saved->max_thickness);
   if (saved->max_contraction_vel != dm->max_contraction_vel && saved->max_contraction_vel != NULL)
      FREE_IFNOTNULL(saved->max_contraction_vel);
   if (saved->force_vel_curve != dm->force_vel_curve)
      nullify_function(saved->force_vel_curve, yes);
   if (saved->optimal_fiber_length != dm->optimal_fiber_length && saved->optimal_fiber_length != NULL)
      FREE_IFNOTNULL(saved->optimal_fiber_length);
   if (saved->resting_tendon_length != dm->resting_tendon_length && saved->resting_tendon_length != NULL)
      FREE_IFNOTNULL(saved->resting_tendon_length);
   if (saved->momentarms != dm->momentarms && saved->momentarms != NULL)
      FREE_IFNOTNULL(saved->momentarms);
   if (saved->active_force_len_curve != dm->active_force_len_curve)
      nullify_function(saved->active_force_len_curve, yes);
   if (saved->passive_force_len_curve != dm->passive_force_len_curve)
      nullify_function(saved->passive_force_len_curve, yes);
   if (saved->tendon_force_len_curve != dm->tendon_force_len_curve)
      nullify_function(saved->tendon_force_len_curve, yes);
   if (saved->wrapStruct)
   {
      for (j = 0; j < saved->numWrapStructs; j++)
      {
         FREE_IFNOTNULL(saved->wrapStruct[j]->mp_wrap[0].wrap_pts);
         FREE_IFNOTNULL(saved->wrapStruct[j]->mp_wrap[1].wrap_pts);
         FREE_IFNOTNULL(saved->wrapStruct[j]);
      }
      FREE_IFNOTNULL(saved->wrapStruct);
   }


   FREE_IFNOTNULL(saved);
}

/* FREE_SAVEDMUSCLEPATHS: free all the saved muscle paths */
void free_savedmusclepaths(SaveMusclePath path[], int num)
{
   int i;

   if (path == NULL)
      return;

   for (i=0; i<num; i++)
   {
      if (path[i].mp_orig != NULL)
         FREE_IFNOTNULL(path[i].mp_orig);
   }
   FREE_IFNOTNULL(path);
}

void free_savedmusclepath(SaveMusclePath *path)
{
   int i, j;

   if (path == NULL)
      return;

   if (path->mp_orig != NULL)
      FREE_IFNOTNULL(path->mp_orig);
}

/* FREE_MUSCLEPATHS: free all the muscle paths */
void free_musclepaths(MusclePathStruct path[], int num)
{
   int i;

   if (path == NULL)
      return;

   for (i=0; i<num; i++)
   {
      if (path[i].mp_orig != NULL)
         FREE_IFNOTNULL(path[i].mp_orig);
   }
   FREE_IFNOTNULL(path);
}

/* FREE_MUSCLEPATHS: free one muscle path */
void free_musclepath(MusclePathStruct *path)
{

   if (path == NULL)
      return;

   if (path->mp_orig != NULL)
      FREE_IFNOTNULL(path->mp_orig);
   
}

void free_and_nullify(void** ptr)
{

   if (*ptr == NULL)
      return;

   free(*ptr);

   *ptr = NULL;

}



void free_menu(Menu* mn)
{

   int i;

   for (i=0; i<mn->numoptions; i++)
      FREE_IFNOTNULL(mn->option[i].name);

   FREE_IFNOTNULL(mn->title);
   FREE_IFNOTNULL(mn->option);

}



void free_form(Form* frm)
{

   int i;

   for (i=0; i<frm->numoptions; i++)
      FREE_IFNOTNULL(frm->option[i].name);

   FREE_IFNOTNULL(frm->title);
   FREE_IFNOTNULL(frm->option);

}


/* -------------------------------------------------------------------------
   free_motion_object - 
---------------------------------------------------------------------------- */
public void free_motion_object(MotionObject* mo, ModelStruct* ms)
{
   if (mo)
   {
      FREE_IFNOTNULL(mo->name);
      FREE_IFNOTNULL(mo->filename);
      FREE_IFNOTNULL(mo->materialname);
      free_polyhedron(&mo->shape, no, ms);
   }
}

#ifndef ENGINE

/* -------------------------------------------------------------------------
   free_motion_object_instance - 
---------------------------------------------------------------------------- */
public void free_motion_object_instance (MotionObjectInstance* mi)
{
   if (mi)
   {
      mi->num_channels = 0;
      
      if (mi->currentMaterial.normal_list != -1)
         glDeleteLists(mi->currentMaterial.normal_list, 1);
      
      if (mi->currentMaterial.highlighted_list != -1)
         glDeleteLists(mi->currentMaterial.highlighted_list, 1);
      
      FREE_IFNOTNULL(mi->channels);
   }
}
#endif
