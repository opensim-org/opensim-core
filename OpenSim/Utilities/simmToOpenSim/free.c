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
      free_muscles     : frees memory malloced for muscle structure elements
      free_default_muscle : frees memory malloced for defaultmuscle elements
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
#if ! ENGINE
extern ModelStruct* sMotionModel;
#endif

/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static void free_segment(SegmentStruct* seg, ModelStruct* ms);
static void free_saved_segment(SaveSegments* seg, ModelStruct* ms);


void free_model(int mod)
{
   if (gModel[mod] != NULL)
   {
#if ! ENGINE
      if (sMotionModel == gModel[mod])
         sMotionModel = NULL;
#endif
      freeModelStruct(gModel[mod]);
      gModel[mod] = NULL;
   }
}

void freeModelStruct(ModelStruct* ms)
{
   int i, j, k;

   FREE_IFNOTNULL(ms->name);
   if (ms->pathptrs != NULL)
   {
      for (i=0; i<ms->numsegments*ms->numsegments; i++)
         FREE_IFNOTNULL(ms->pathptrs[i]);
      free(ms->pathptrs);
   }
   FREE_IFNOTNULL(ms->jointfilename);
   FREE_IFNOTNULL(ms->musclefilename);
   FREE_IFNOTNULL(ms->bonepathname);
   FREE_IFNOTNULL(ms->mocap_dir);
   for (i=0; i<ms->num_motion_files; i++)
      FREE_IFNOTNULL(ms->motionfilename[i]);

   for (i=0; i<ms->numgroups; i++)
   {
      FREE_IFNOTNULL(ms->muscgroup[i].name);
      FREE_IFNOTNULL(ms->muscgroup[i].muscle_index);
      free_menu(&ms->muscgroup[i].menu);
   }
   FREE_IFNOTNULL(ms->muscgroup);
   for (i=0; i<ms->save.numsavedmuscgroups; i++)
   {
      FREE_IFNOTNULL(ms->save.muscgroup[i].name);
      FREE_IFNOTNULL(ms->save.muscgroup[i].muscle_index);
   }
   FREE_IFNOTNULL(ms->save.muscgroup);

   free_form(&ms->gencform);
   free_form(&ms->dynparamsform);

   for (i = 0; i < ms->gc_chpanel.numoptions; i++)
      FREE_IFNOTNULL(ms->gc_chpanel.checkbox[i].name);
   FREE_IFNOTNULL(ms->gc_chpanel.checkbox);
   //FREE_IFNOTNULL(ms->gc_chpanel.title); title uses static char

   for (i = 0; i < ms->gc_lockPanel.numoptions; i++)
      FREE_IFNOTNULL(ms->gc_lockPanel.checkbox[i].name);
   FREE_IFNOTNULL(ms->gc_lockPanel.checkbox);
   //FREE_IFNOTNULL(ms->gc_lockPanel.title); title uses static char

   for (i = 0; i < ms->numseggroups; i++)
   {
      FREE_IFNOTNULL(ms->seggroup[i].name);
      FREE_IFNOTNULL(ms->seggroup[i].segment);
   }
   FREE_IFNOTNULL(ms->seggroup);

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

   for (i=0; i<ms->save.numsavedjnts; i++)
      FREE_IFNOTNULL(ms->save.joint[i].name);
   FREE_IFNOTNULL(ms->save.joint);

   for (i=0; i<ms->numsegments; i++)
      free_segment(&ms->segment[i], ms);
   FREE_IFNOTNULL(ms->segment);
   for (i=0; i<ms->save.numsavedsegments; i++)
      free_saved_segment(&ms->save.segment[i], ms);
   FREE_IFNOTNULL(ms->save.segment);

   for (i=0; i<ms->save.num_markers; i++)
      FREE_IFNOTNULL(ms->save.marker[i].name);
   FREE_IFNOTNULL(ms->save.marker);

   for (i=0; i<ms->num_wrap_objects; i++)
   {
      FREE_IFNOTNULL(ms->wrapobj[i]->name);
      FREE_IFNOTNULL(ms->wrapobj[i]);
   }
   FREE_IFNOTNULL(ms->wrapobj);
   for (i=0; i<ms->save.num_wrap_objects; i++)
      FREE_IFNOTNULL(ms->save.wrap_object[i].name);
   FREE_IFNOTNULL(ms->save.wrap_object);

   for (i=0; i<ms->numgencoords; i++)
   {
      if (ms->gencoord[i]->defined == yes)
      {
          FREE_IFNOTNULL(ms->gencoord[i]->name);
          FREE_IFNOTNULL(ms->gencoord[i]->jointnum);
#if INCLUDE_MOCAP_MODULE
         FREE_IFNOTNULL(ms->gencoord[i]->mocap_segment);
#endif
          FREE_IFNOTNULL(ms->gencoord[i]->group);
      }
   }
   FREE_IFNOTNULL(ms->gencoord);
   FREE_IFNOTNULL(ms->save.gencoord);

   for (i = 0; i < 2*GENBUFFER; i++)
      FREE_IFNOTNULL(ms->genc_help[i].text);

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
   FREE_IFNOTNULL(ms->deformity);

   for (i = 0; i < ms->numligaments; i++)
   {
      FREE_IFNOTNULL(ms->ligament[i].name);
      for (j = 0; j < ms->ligament[i].numlines; j++)
      {
         FREE_IFNOTNULL(ms->ligament[i].line[j].mp_orig);
         FREE_IFNOTNULL(ms->ligament[i].line[j].mp);
      }
      FREE_IFNOTNULL(ms->ligament[i].line);
   }
   FREE_IFNOTNULL(ms->ligament);

   free_muscles(ms);
   free_default_muscle(ms->default_muscle);

   for (i=0; i<ms->func_array_size; i++)
      free_function(ms->function[i], yes);
   FREE_IFNOTNULL(ms->function);

   if (ms->save.function)
   {
      for (i=0; i<ms->func_array_size; i++)
         free_function(ms->save.function[i], yes);
      FREE_IFNOTNULL(ms->save.function);
   }

#if ! ENGINE
   for (i = 0; i < ms->num_motion_objects; i++)
      free_motion_object(&ms->motion_objects[i], ms);

   FREE_IFNOTNULL(ms->motion_objects);
#endif

   FREE_IFNOTNULL(ms->save.muscwrap_associations);

   for (i = 0; i < ms->numworldobjects; i++)
   {
      FREE_IFNOTNULL(ms->worldobj[i].name);
      FREE_IFNOTNULL(ms->worldobj[i].filename);
      if (ms->worldobj[i].wobj)
         free_polyhedron(ms->worldobj[i].wobj, yes, ms);
   }
   FREE_IFNOTNULL(ms->worldobj);

   for (i = 0; i < ms->save.num_deforms; i++)
   {
      FREE_IFNOTNULL(ms->save.deform[i].name);
      FREE_IFNOTNULL(ms->save.deform[i].innerBox);
      FREE_IFNOTNULL(ms->save.deform[i].innerBoxUndeformed);
      FREE_IFNOTNULL(ms->save.deform[i].outerBox);
      FREE_IFNOTNULL(ms->save.deform[i].outerBoxUndeformed);
   }
   FREE_IFNOTNULL(ms->save.deform);

   for (i = 0; i < ms->num_constraint_objects; i++)
   {
      FREE_IFNOTNULL(ms->constraintobj[i].name);
      FREE_IFNOTNULL(ms->constraintobj[i].joints);
      FREE_IFNOTNULL(ms->constraintobj[i].qs);
      for (j = 0; j < ms->constraintobj[i].numPoints; j++)
         FREE_IFNOTNULL(ms->constraintobj[i].points[j].name);
      FREE_IFNOTNULL(ms->constraintobj[i].points);
   }
   FREE_IFNOTNULL(ms->constraintobj);

   for (i = 0; i < ms->save.num_constraint_objects; i++)
   {
      FREE_IFNOTNULL(ms->save.constraintobj[i].name);
      FREE_IFNOTNULL(ms->save.constraintobj[i].joints);
      FREE_IFNOTNULL(ms->save.constraintobj[i].qs);
      for (j = 0; j < ms->save.constraintobj[i].numPoints; j++)
         FREE_IFNOTNULL(ms->save.constraintobj[i].points[j].name);
      FREE_IFNOTNULL(ms->save.constraintobj[i].points);
   }
   FREE_IFNOTNULL(ms->save.constraintobj);

   for (i = 0; i < ms->save.num_conspt_associations; i++)
   {
      for (j = 0; j < ms->save.conspt_associations[i].numPoints; j++)
         FREE_IFNOTNULL(ms->save.conspt_associations[i].savedPoints[j].name);
      FREE_IFNOTNULL(ms->save.conspt_associations[i].savedPoints);
   }
   FREE_IFNOTNULL(ms->save.conspt_associations);

   FREE_IFNOTNULL(ms->segment_drawing_order);

   for (i = 0; i < MAXSAVEDVIEWS; i++)
      FREE_IFNOTNULL(ms->dis.view_name[i]);

   for (i = 0; i < ms->numgencgroups; i++)
   {
      FREE_IFNOTNULL(ms->gencgroup[i].name);
      FREE_IFNOTNULL(ms->gencgroup[i].gencoord);
   }
   FREE_IFNOTNULL(ms->gencgroup);

   FREE_IFNOTNULL(ms->gencslider.sl);
   FREE_IFNOTNULL(ms->dis.devs);
   FREE_IFNOTNULL(ms->dis.dev_values);
   FREE_IFNOTNULL(ms->dis.muscleson);
   FREE_IFNOTNULL(ms->forceUnits);
   FREE_IFNOTNULL(ms->lengthUnits);

   // The motions are deleted by delete_model() so that the appropriate
   // events can be generated, so all that remains here is the array of
   // motion structure pointers.
   FREE_IFNOTNULL(ms->motion);

   FREE_IFNOTNULL(ms);
}

static void free_segment(SegmentStruct* seg, ModelStruct* ms)
{
   int j;

   if (seg->defined == no)
      return;

   FREE_IFNOTNULL(seg->name);
   for (j=0; j<seg->numBones; j++)
      free_polyhedron(&seg->bone[j], no, ms);
   FREE_IFNOTNULL(seg->bone);
   for (j=0; j<seg->numSpringPoints; j++)
   {
      FREE_IFNOTNULL(seg->springPoint[j].name);
   }
   FREE_IFNOTNULL(seg->springPoint);
   FREE_IFNOTNULL(seg->group);

   if (seg->springFloor)
   {
      FREE_IFNOTNULL(seg->springFloor->name);
      FREE_IFNOTNULL(seg->springFloor->filename);
      free_polyhedron(seg->springFloor->poly, yes, ms);
      FREE_IFNOTNULL(seg->springFloor->points);
      FREE_IFNOTNULL(seg->springFloor);
   }

   for (j=0; j<seg->numContactObjects; j++)
   {
      FREE_IFNOTNULL(seg->contactObject[j].name);
      FREE_IFNOTNULL(seg->contactObject[j].filename);
      free_polyhedron(seg->contactObject[j].poly, yes, ms);
   }
   FREE_IFNOTNULL(seg->contactObject);

   if (seg->forceMatte)
   {
      FREE_IFNOTNULL(seg->forceMatte->name);
      FREE_IFNOTNULL(seg->forceMatte->filename);
      free_polyhedron(seg->forceMatte->poly, yes, ms);
      FREE_IFNOTNULL(seg->forceMatte);
   }

   for (j=0; j<seg->numMarkers; j++)
   {
      FREE_IFNOTNULL(seg->marker[j]->name);
      FREE_IFNOTNULL(seg->marker[j]);
   }
   FREE_IFNOTNULL(seg->marker);

   for (j=0; j<seg->num_deforms; j++)
   {
      FREE_IFNOTNULL(seg->deform[j].name);
      FREE_IFNOTNULL(seg->deform[j].innerBox);
      FREE_IFNOTNULL(seg->deform[j].innerBoxUndeformed);
      FREE_IFNOTNULL(seg->deform[j].outerBox);
      FREE_IFNOTNULL(seg->deform[j].outerBoxUndeformed);
   }
   FREE_IFNOTNULL(seg->deform);

#if INCLUDE_MOCAP_MODULE
   FREE_IFNOTNULL(seg->gait_scale_segment);
   FREE_IFNOTNULL(seg->mocap_segment);
   FREE_IFNOTNULL(seg->mocap_scale_chain_end1);
   FREE_IFNOTNULL(seg->mocap_scale_chain_end2);
#endif
}

static void free_saved_segment(SaveSegments* seg, ModelStruct* ms)
{
   int j;

   FREE_IFNOTNULL(seg->name);
   for (j=0; j<seg->numSpringPoints; j++)
   {
      FREE_IFNOTNULL(seg->springPoint[j].name);
   }
   FREE_IFNOTNULL(seg->springPoint);

   if (seg->springFloor)
   {
      FREE_IFNOTNULL(seg->springFloor->name);
      FREE_IFNOTNULL(seg->springFloor->filename);
      free_polyhedron(seg->springFloor->poly, yes, ms);
      FREE_IFNOTNULL(seg->springFloor->points);
      FREE_IFNOTNULL(seg->springFloor);
   }

   for (j=0; j<seg->numContactObjects; j++)
   {
      FREE_IFNOTNULL(seg->contactObject[j].name);
      FREE_IFNOTNULL(seg->contactObject[j].filename);
      free_polyhedron(seg->contactObject[j].poly, yes, ms);
   }
   FREE_IFNOTNULL(seg->contactObject);

   if (seg->forceMatte)
   {
      FREE_IFNOTNULL(seg->forceMatte->name);
      FREE_IFNOTNULL(seg->forceMatte->filename);
      free_polyhedron(seg->forceMatte->poly, yes, ms);
      FREE_IFNOTNULL(seg->forceMatte);
   }
}

#if ! ENGINE
void free_plot(int plotnum)
{
   int i, j;

   FREE_IFNOTNULL(gPlot[plotnum]->title);
   FREE_IFNOTNULL(gPlot[plotnum]->xname);
   /* JPL 11/2/00 TODO: for some reason, freeing the yname is causing
    * a crash, so remove it for now.
    */
/*   FREE_IFNOTNULL(gPlot[plotnum]->yname);*/

   for (i=0; i<gPlot[plotnum]->numcurves; i++)
   {
      FREE_IFNOTNULL(gPlot[plotnum]->curve[i]->xvalues);
      FREE_IFNOTNULL(gPlot[plotnum]->curve[i]->yvalues);
      FREE_IFNOTNULL(gPlot[plotnum]->curve[i]->name);
      if (gPlot[plotnum]->curve[i]->num_events > 0)
      {
         for (j=0; j<gPlot[plotnum]->curve[i]->num_events; j++)
            FREE_IFNOTNULL(gPlot[plotnum]->curve[i]->event[j].name);
         FREE_IFNOTNULL(gPlot[plotnum]->curve[i]->event);
      }
      FREE_IFNOTNULL(gPlot[plotnum]->curve[i]);
   }

   if (gPlot[plotnum]->num_file_events > 0)
   {
      for (j=0; j<gPlot[plotnum]->num_file_events; j++)
         FREE_IFNOTNULL(gPlot[plotnum]->file_event[j].name);
      FREE_IFNOTNULL(gPlot[plotnum]->file_event);
   }

   FREE_IFNOTNULL(gPlot[plotnum]);

   gPlot[plotnum] = NULL;
}
#endif


void free_muscle(dpMuscleStruct *muscle, dpMuscleStruct* dm)
{
   int i;

   if (muscle == NULL)
      return;

   if (muscle->name != dm->name)
      FREE_IFNOTNULL(muscle->name);

   if (muscle->path)
   {
      FREE_IFNOTNULL(muscle->path->mp_orig);
      FREE_IFNOTNULL(muscle->path->mp);
      FREE_IFNOTNULL(muscle->path);
   }

   if (muscle->group != dm->group)
      FREE_IFNOTNULL(muscle->group);
   if (muscle->max_isometric_force != dm->max_isometric_force)
      FREE_IFNOTNULL(muscle->max_isometric_force);
   if (muscle->pennation_angle != dm->pennation_angle)
      FREE_IFNOTNULL(muscle->pennation_angle);
   if (muscle->min_thickness != dm->min_thickness)
      FREE_IFNOTNULL(muscle->min_thickness);
   if (muscle->max_thickness != dm->max_thickness)
      FREE_IFNOTNULL(muscle->max_thickness);
   if (muscle->min_material != dm->min_material)
      FREE_IFNOTNULL(muscle->min_material);
   if (muscle->max_material != dm->max_material)
      FREE_IFNOTNULL(muscle->max_material);
   if (muscle->max_contraction_vel != dm->max_contraction_vel)
      FREE_IFNOTNULL(muscle->max_contraction_vel);
   if (muscle->optimal_fiber_length != dm->optimal_fiber_length)
      FREE_IFNOTNULL(muscle->optimal_fiber_length);
   if (muscle->resting_tendon_length != dm->resting_tendon_length)
      FREE_IFNOTNULL(muscle->resting_tendon_length);
   if (muscle->momentarms != dm->momentarms)
      FREE_IFNOTNULL(muscle->momentarms);
   if (muscle->active_force_len_func != dm->active_force_len_func)
      FREE_IFNOTNULL(muscle->active_force_len_func);
   if (muscle->passive_force_len_func != dm->passive_force_len_func)
      FREE_IFNOTNULL(muscle->passive_force_len_func);
   if (muscle->tendon_force_len_func != dm->tendon_force_len_func)
      FREE_IFNOTNULL(muscle->tendon_force_len_func);
   if (muscle->force_vel_func != dm->force_vel_func)
      FREE_IFNOTNULL(muscle->force_vel_func);
   if (muscle->excitation_func != dm->excitation_func)
      FREE_IFNOTNULL(muscle->excitation_func);

   if (muscle->wrapStruct)
   {
      for (i = 0; i < muscle->numWrapStructs; i++)
      {
         FREE_IFNOTNULL(muscle->wrapStruct[i]->mp_wrap[0].wrap_pts);
         FREE_IFNOTNULL(muscle->wrapStruct[i]->mp_wrap[1].wrap_pts);
         FREE_IFNOTNULL(muscle->wrapStruct[i]);
      }
      FREE_IFNOTNULL(muscle->wrapStruct);
   }
   if (muscle->muscle_model_index != dm->muscle_model_index)
      FREE_IFNOTNULL(muscle->muscle_model_index);

   if (muscle->dynamic_params)
   {
      for (i = 0; i < muscle->num_dynamic_params; i++)
      {
         if (muscle->dynamic_params[i] != dm->dynamic_params[i])
            FREE_IFNOTNULL(muscle->dynamic_params[i]);
      }
      FREE_IFNOTNULL(muscle->dynamic_params);
   }
}

void free_muscles(ModelStruct* model)
{
   int i;

   if (model == NULL)
      return;

   for (i=0; i<model->nummuscles; i++)
   {
      free_muscle(model->muscle[i], model->default_muscle);
      FREE_IFNOTNULL(model->muscle[i]);
   }

   FREE_IFNOTNULL(model->muscle);
}

/* FREE_DEFMUSC: */
void free_default_muscle(dpMuscleStruct* dm)
{
   int i;

   if (dm == NULL)
      return;

   FREE_IFNOTNULL(dm->name);
   FREE_IFNOTNULL(dm->group);
   FREE_IFNOTNULL(dm->max_isometric_force);
   FREE_IFNOTNULL(dm->pennation_angle);
   FREE_IFNOTNULL(dm->min_thickness);
   FREE_IFNOTNULL(dm->max_thickness);
   FREE_IFNOTNULL(dm->min_material);
   FREE_IFNOTNULL(dm->max_material);
   FREE_IFNOTNULL(dm->muscle_model_index);
   FREE_IFNOTNULL(dm->max_contraction_vel);

   FREE_IFNOTNULL(dm->optimal_fiber_length);
   FREE_IFNOTNULL(dm->resting_tendon_length);
   FREE_IFNOTNULL(dm->momentarms);

   FREE_IFNOTNULL(dm->tendon_force_len_func);
   FREE_IFNOTNULL(dm->active_force_len_func);
   FREE_IFNOTNULL(dm->passive_force_len_func);
   FREE_IFNOTNULL(dm->force_vel_func);
   FREE_IFNOTNULL(dm->excitation_func);

   for (i = 0; i < dm->num_dynamic_params; i++)
      FREE_IFNOTNULL(dm->dynamic_params[i]);
   FREE_IFNOTNULL(dm->dynamic_params);

   for (i = 0; i < dm->num_dynamic_params; i++)
      FREE_IFNOTNULL(dm->dynamic_param_names[i]);
   FREE_IFNOTNULL(dm->dynamic_param_names);
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

void free_checkbox_panel(CheckBoxPanel* panel)
{
   int i;

   for (i=0; i<panel->numoptions; i++)
      FREE_IFNOTNULL(panel->checkbox[i].name);

   FREE_IFNOTNULL(panel->title);
   FREE_IFNOTNULL(panel->checkbox);
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

/* -------------------------------------------------------------------------
   free_motion_object_instance - 
---------------------------------------------------------------------------- */
public void free_motion_object_instance(MotionObjectInstance* mi, ModelStruct* model)
{
   if (mi)
   {
      FREE_IFNOTNULL(mi->name);
      mi->num_channels = 0;
      
#if ! ENGINE
      if (mi->currentMaterial.normal_list)
         glDeleteLists(mi->currentMaterial.normal_list, 1);

      if (mi->currentMaterial.highlighted_list)
         glDeleteLists(mi->currentMaterial.highlighted_list, 1);

      delete_display_list(mi->aux_display_obj, model);
#endif

      FREE_IFNOTNULL(mi->channels);
   }
}

#if ! ENGINE

void delete_display_list(GLuint display_list, ModelStruct* model)
{
   if (display_list)
   {
      if (model)
      {
         // TODO_SCENE: the model for this display list may be in more than one scene
         // (window). To delete the display list, you have to glutSetWindow to the one
         // that was current when the display list was created. For now, assume that
         // this is the first scene that contains the model.
         int savedWindow = glutGetWindow();
         Scene* scene = get_first_scene_containing_model(model);

         if (scene)
         {
            glutSetWindow(scene->window_glut_id);
            glDeleteLists(display_list, 1);
         }
         glutSetWindow(savedWindow);
      }
      else
      {
         glDeleteLists(display_list, 1);
      }
   }
}


void delete_polyhedron_display_list(PolyhedronStruct* ph, ModelStruct* model)
{
   if (ph && ph->gl_display)
   {
      if (model)
      {
         // TODO_SCENE: the polyhedron has only one display list, but the model
         // may be in more than one scene (window). To delete the display list,
         // you have to glutSetWindow to the one that was current when the display
         // list was created. For now, assume that this is the first scene that
         // contains the model.
         int savedWindow = glutGetWindow();
         Scene* scene = get_first_scene_containing_model(model);

         if (scene)
         {
            glutSetWindow(scene->window_glut_id);
            glDeleteLists(ph->gl_display, 1);
            ph->gl_display = 0;
         }
         glutSetWindow(savedWindow);
      }
      else
      {
         glDeleteLists(ph->gl_display, 1);
         ph->gl_display = 0;
      }
   }
}


void delete_segment_display_lists(SegmentStruct* seg, ModelStruct* model)
{
   if (seg && model)
   {
      // TODO_SCENE: the segment's polyhedra have only one display list each,
      // but the model may be in more than one scene (window). To delete the display
      // lists, you have to glutSetWindow to the one that was current when the display
      // lists were created. For now, assume that this is the first scene that
      // contains the model.
      int i, savedWindow = glutGetWindow();
      Scene* scene = get_first_scene_containing_model(model);

      if (scene)
      {
         glutSetWindow(scene->window_glut_id);
         for (i=0; i<seg->numBones; i++)
         {
            glDeleteLists(seg->bone[i].gl_display, 1);
            seg->bone[i].gl_display = 0;
         }
      }
      glutSetWindow(savedWindow);
   }
}

#endif
