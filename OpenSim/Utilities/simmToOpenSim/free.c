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
      free_muscs       : frees memory malloced for muscle structure elements
      free_defmusc     : frees memory malloced for defaultmuscle elements
      free_and_nullify : frees memory and sets the pointer to NULL
      free_menu        : frees a menu structure
      free_form        : frees a form structure

*******************************************************************************/

#include "universal.h"
#include "globals.h"
#include "functions.h"
#include "normtools.h"


/*************** DEFINES (for this file only) *********************************/
	


/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/



/*************** EXTERNED VARIABLES (declared in another file) ****************/
#ifdef WIN32
extern ModelStruct* sMotionModel;
#endif

/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/



void free_model(int mod)
{

   int i, j;

#ifdef WIN32
   if (sMotionModel == model[mod])
     sMotionModel = NULL;
#endif

   FREE_IFNOTNULL(model[mod]->name);
   FREE_IFNOTNULL(model[mod]->pathptrs);
   FREE_IFNOTNULL(model[mod]->jointfilename);
   FREE_IFNOTNULL(model[mod]->musclefilename);

   for (i=0; i<model[mod]->numgroups; i++)
   {
      FREE_IFNOTNULL(model[mod]->muscgroup[i].name);
      free_menu(&model[mod]->muscgroup[i].menu);
   }
   FREE_IFNOTNULL(model[mod]->muscgroup);

   free_form(&model[mod]->gencform);

   for (i=0; i<model[mod]->numjoints; i++)
   {
      FREE_IFNOTNULL(model[mod]->joint[i].name);
      FREE_IFNOTNULL(model[mod]->joint[i].solverType);
      FREE_IFNOTNULL(model[mod]->joint[i].in_seg_ground_path);
#if INCLUDE_MOCAP_MODULE
      FREE_IFNOTNULL(model[mod]->joint[i].mocap_segment);
#endif
   }
   FREE_IFNOTNULL(model[mod]->joint);

   for (i=0; i<model[mod]->numsegments; i++)
   {
      if (model[mod]->segment[i].defined == no)
         continue;
      FREE_IFNOTNULL(model[mod]->segment[i].name);
      for (j=0; j<model[mod]->segment[i].numBones; j++)
      {
         FREE_IFNOTNULL(model[mod]->segment[i].bone[j].name);
         /*
         FREE_IFNOTNULL(model[mod]->segment[i].bone[j].vert);
         FREE_IFNOTNULL(model[mod]->segment[i].bone[j].poly);
         */
      }
      for (j=0; j<model[mod]->segment[i].numSpringPoints; j++)
      {
         FREE_IFNOTNULL(model[mod]->segment[i].springPoint[j].name);
         FREE_IFNOTNULL(model[mod]->segment[i].springPoint[j].floorName);
      }
      FREE_IFNOTNULL(model[mod]->segment[i].springPoint);

      if (model[mod]->segment[i].springFloor)
      {
         FREE_IFNOTNULL(model[mod]->segment[i].springFloor->name);
         FREE_IFNOTNULL(model[mod]->segment[i].springFloor->filename);
         FREE_IFNOTNULL(model[mod]->segment[i].springFloor->poly);
         FREE_IFNOTNULL(model[mod]->segment[i].springFloor->points);
         FREE_IFNOTNULL(model[mod]->segment[i].springFloor);
      }

      for (j=0; j<model[mod]->segment[i].numContactObjects; j++)
      {
         FREE_IFNOTNULL(model[mod]->segment[i].contactObject[j].name);
         FREE_IFNOTNULL(model[mod]->segment[i].contactObject[j].filename);
         FREE_IFNOTNULL(model[mod]->segment[i].contactObject[j].poly);
      }
      FREE_IFNOTNULL(model[mod]->segment[i].contactObject);

      FREE_IFNOTNULL(model[mod]->segment[i].marker);

#if INCLUDE_MOCAP_MODULE
      FREE_IFNOTNULL(model[mod]->segment[i].gait_scale_segment);
      FREE_IFNOTNULL(model[mod]->segment[i].mocap_segment);
      FREE_IFNOTNULL(model[mod]->segment[i].mocap_scale_chain_end1);
      FREE_IFNOTNULL(model[mod]->segment[i].mocap_scale_chain_end2);
#endif
   }
   FREE_IFNOTNULL(model[mod]->segment);
   FREE_IFNOTNULL(model[mod]->wrapobj);

   for (i=0; i<model[mod]->numgencoords; i++)
   {
      if (model[mod]->gencoord[i].defined == yes)
      {
	      FREE_IFNOTNULL(model[mod]->gencoord[i].name);
	      FREE_IFNOTNULL(model[mod]->gencoord[i].jointnum);
#if INCLUDE_MOCAP_MODULE
         FREE_IFNOTNULL(model[mod]->gencoord[i].mocap_segment);
#endif
      }
   }
   FREE_IFNOTNULL(model[mod]->gencoord);

   for (i = 0; i < model[mod]->num_deformities; i++)
   {
      if (model[mod]->deformity[i].deform_name)
      {
         for (j = 0; j < model[mod]->deformity[i].num_deforms; j++)
            FREE_IFNOTNULL(model[mod]->deformity[i].deform_name[j]);

         FREE_IFNOTNULL(model[mod]->deformity[i].deform_name);
      }

      FREE_IFNOTNULL(model[mod]->deformity[i].deform);
   }

#ifndef ENGINE

   /* NOTE: the model's window *must* be set as the current GL context, otherwise
    *  the glDeleteList() calls below will produce unpredictable bad results.
    */
   for (i=0; i<model[mod]->dis.mat.num_materials; i++)
   {
      if (model[mod]->dis.mat.materials[i].normal_list != -1)
         glDeleteLists(model[mod]->dis.mat.materials[i].normal_list,1);
      if (model[mod]->dis.mat.materials[i].highlighted_list != -1)
         glDeleteLists(model[mod]->dis.mat.materials[i].highlighted_list,1);
      FREE_IFNOTNULL(model[mod]->dis.mat.materials[i].name);
   }
   FREE_IFNOTNULL(model[mod]->dis.mat.materials);

   free_muscs(model[mod]->muscle,&model[mod]->default_muscle,model[mod]->nummuscles);

   free_defmusc(&model[mod]->default_muscle);
#endif

   for (i=0; i<model[mod]->numfunctions; i++)
      if (model[mod]->function[i].defined == yes)
	 nullify_function(&model[mod]->function[i], no);

   FREE_IFNOTNULL(model[mod]->function);

#ifndef ENGINE
   free_muscs(model[mod]->save.muscle,&model[mod]->save.default_muscle,model[mod]->save.numsavedmuscs);
   free_defmusc(&model[mod]->save.default_muscle);

   for (i = 0; i < model[mod]->num_motion_objects; i++)
      free_motion_object(&model[mod]->motion_objects[i], model[mod]);
   
   FREE_IFNOTNULL(model[mod]->motion_objects);
#endif
   
/*
   for (i=0; i<model[mod]->numjoints; i++)
   {
      FREE_IFNOTNULL(model[mod]->save.jnts[i].name);
      for (j=0; j<6; j++)
	 FREE_IFNOTNULL(model[mod]->save.jnts[i].dofs[j].element);
   }
   FREE_IFNOTNULL(model[mod]->save.jnts);
   for (i=0; i<model[mod]->numgencoords; i++)
      FREE_IFNOTNULL(model[mod]->save.gencs[i].name);
   FREE_IFNOTNULL(model[mod]->save.gencs);
*/

   FREE_IFNOTNULL(model[mod]);
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


#ifndef ENGINE
/* FREE_MUSCS: */

void free_muscs(MuscleStruct musc[], MuscleStruct* dm, int num)
{

   int i, j;

   if (musc == NULL)
      return;

   for (i=0; i<num; i++)
   {
      if (musc[i].name != dm->name && musc[i].name != NULL)
         FREE_IFNOTNULL(musc[i].name);
      if (musc[i].num_orig_points != dm->num_orig_points && musc[i].num_orig_points != NULL)
      {
         for (j=0; j<*musc[i].num_orig_points; j++)
         {
            if (musc[i].mp_orig[j].numranges > 0 && musc[i].mp_orig[j].ranges != NULL)
               FREE_IFNOTNULL(musc[i].mp_orig[j].ranges);
         }
         FREE_IFNOTNULL(musc[i].num_orig_points);
      }
      if (musc[i].mp_orig != dm->mp_orig && musc[i].mp_orig != NULL)
         FREE_IFNOTNULL(musc[i].mp_orig);

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
            FREE_IFNOTNULL(musc[i].wrapStruct[j]->mp_wrap[0].ranges);
            FREE_IFNOTNULL(musc[i].wrapStruct[j]->mp_wrap[1].wrap_pts);
            FREE_IFNOTNULL(musc[i].wrapStruct[j]->mp_wrap[1].ranges);
            FREE_IFNOTNULL(musc[i].wrapStruct[j]);
         }
         FREE_IFNOTNULL(musc[i].wrapStruct);
      }
   }

   FREE_IFNOTNULL(musc);
}



/* FREE_DEFMUSC: */

void free_defmusc(MuscleStruct* dm)
{

   int i;

   free_and_nullify((void**)&dm->name);
   if (dm->num_orig_points != NULL)
   {
      for (i=0; i<*dm->num_orig_points; i++)
	 free_and_nullify((void**)&dm->mp_orig[i].ranges);
      free_and_nullify((void**)&dm->num_orig_points);
   }
   free_and_nullify((void**)&dm->mp_orig);
   free_and_nullify((void**)&dm->group);
   free_and_nullify((void**)&dm->max_isometric_force);
   free_and_nullify((void**)&dm->pennation_angle);
   free_and_nullify((void**)&dm->min_thickness);
   free_and_nullify((void**)&dm->max_thickness);
   dm->min_material = NULL;
   dm->max_material = NULL;
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

}
#endif


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
