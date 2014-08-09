/*******************************************************************************

   MODELUTILS.C

   Author: Peter Loan

   Date: 03-JAN-91

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Description: 

   Routines:
      delete_model            : deletes a model
      check_gencoord_wrapping : checks boundaries of gencoord value
      makegencform            : makes the gencoord form for a model
      getjointvarnum          : given a dof name, returns a dof number
      getjointvarname         : given dof number, returns dof name (e.g. tx)

*******************************************************************************/

#include "universal.h"
#include "globals.h"
#include "functions.h"

/*************** DEFINES (for this file only) *********************************/



/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/
static void delete_display_lists(ModelStruct* model);
static void destroy_model_menus(ModelStruct* model);


/*************** EXTERNED VARIABLES (declared in another file) ****************/



/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/

#if OPENSMAC
#undef ENGINE
#define ENGINE 1
#endif

#if ! ENGINE

void delete_scene(Scene* scene)
{
   int i;

   // TODO_SCENE
   delete_model(scene->model[0]);

   FREE_IFNOTNULL(scene->model);
   FREE_IFNOTNULL(scene->snapshot_file_suffix);
   FREE_IFNOTNULL(scene->movie_file);
   free(scene);

   for (i=0; i<SCENEBUFFER; i++)
   {
      if (gScene[i] == scene)
      {
         gScene[i] = NULL;
         break;
      }
   }

   adjust_main_menu();
}

/* DELETE_MODEL: this routine deletes a model from the system. It closes the
 * model window, frees the model structures, deletes the model window from
 * the window list, updates the pop-up model menu, and checks to make sure
 * no tools are currently set to the model.
 */
void delete_model(ModelStruct* ms)
{
   int i, j;
   char buf[1024];
   MotionModelOptions* mmo;
   Scene* scene = get_first_scene_containing_model(ms);

   if (is_model_realtime(ms) == rtMocap)
   {
#if INCLUDE_EVA_REALTIME
      stop_realtime_mocap_stream();
#endif
   }
   else if (is_model_realtime(ms) == rtSimulation)
   {
#if ! SIMM_VIEWER
      // DPTODO
#endif
   }

   glutDeleteMutex(ms->modelLock);

   glutSetWindow(scene->window_glut_id);

   /* Post the MODEL_DELETED event before deleting the motions
    * (which will post MOTION_DELETED events for each one). This
    * requires more checking by each tool when handling a
    * MOTION_DELETED event (to see if the model still exists or
    * not), but is more efficient because the tool will not
    * update itself for each motion deleted and then update itself
    * again when the whole model is deleted.
    * The model pointer is included in the MODEL_DELETED event so
    * that the tools can check to see if they are currently set to
    * the deleted model. But the modelnum is also needed to index
    * into the array of tool/model options (e.g., meop, pmop).
    */
   make_and_queue_simm_event(MODEL_DELETED, ms->modelnum, ms, NULL, ZERO, ZERO);

   for (i = 0; i < ms->motion_array_size; i++)
   {
      if (ms->motion[i])
      {
         delete_motion(ms, ms->motion[i], no);
         FREE_IFNOTNULL(ms->motion[i]);
      }
   }

   destroy_model_menus(ms);

   // In order to delete the window from the GUI as quickly as possible,
   // only the tasks that must be done before the window is
   // removed are done before calling delete_window(). This
   // means that deleting the bone display lists, which used
   // to be done inside free_model(), is done separately, here.
   // These display lists can only be deleted when the window
   // in which they were created is current.
   delete_display_lists(ms);

   delete_window(scene->window_glut_id);

   sprintf(buf, "Deleted model %s.", ms->name);

   purge_simm_events_for_struct(scene, SCENE_INPUT_EVENT);
   purge_simm_events_for_model(ms->modelnum, MODEL_ADDED);

   free_model(ms->modelnum);

   root.nummodels--;

   updatemodelmenu();

   message(buf, 0, DEFAULT_MESSAGE_X_OFFSET);
}

static void delete_display_lists(ModelStruct* model)
{
   int i, j;
   Scene* scene = get_first_scene_containing_model(model);
   if (scene && scene->window_glut_id != -1)
   {
      int savedID = glutGetWindow();
      glutSetWindow(scene->window_glut_id);

      // Delete the muscle display lists.
      glDeleteLists(model->dis.muscle_cylinder_id, 1);
      glDeleteLists(model->dis.muscle_point_id, 1);

      // Delete the bone, springfloor, contact object, and force matte display lists.
      for (i=0; i<model->numsegments; i++)
      {
         if (model->segment[i].defined == yes)
         {
            for (j=0; j<model->segment[i].numBones; j++)
               delete_polyhedron_display_list(&model->segment[i].bone[j], NULL);
            for (j=0; j<model->segment[i].numContactObjects; j++)
               delete_polyhedron_display_list(model->segment[i].contactObject[j].poly, NULL);
            if (model->segment[i].springFloor)
               delete_polyhedron_display_list(model->segment[i].springFloor->poly, NULL);
            if (model->segment[i].forceMatte)
               delete_polyhedron_display_list(model->segment[i].forceMatte->poly, NULL);
         }
      }

      // Delete the wrap object display lists.
      for (i=0; i<model->num_wrap_objects; i++)
         glDeleteLists(model->wrapobj[i]->display_list, 1);

      // Delete the world object display lists.
      for (i=0; i<model->numworldobjects; i++)
         delete_polyhedron_display_list(model->worldobj[i].wobj, NULL);

      // Delete the constraint object display lists.
      for (i=0; i<model->num_constraint_objects; i++)
         glDeleteLists(model->constraintobj[i].display_list, 1);

      // Delete the materials.
      for (i=0; i<model->dis.mat.num_materials; i++)
      {
         if (model->dis.mat.materials[i].normal_list)
            glDeleteLists(model->dis.mat.materials[i].normal_list,1);
         if (model->dis.mat.materials[i].highlighted_list)
            glDeleteLists(model->dis.mat.materials[i].highlighted_list,1);
         FREE_IFNOTNULL(model->dis.mat.materials[i].name);
      }
      FREE_IFNOTNULL(model->dis.mat.materials);

      glutSetWindow(savedID);
   }
}

static void destroy_model_menus(ModelStruct* model)
{
   int i;

   glutDestroyMenu(model->musclegroupmenu);
   glutDestroyMenu(model->jointmenu);
   glutDestroyMenu(model->xvarmenu);
   glutDestroyMenu(model->gencoordmenu);
   glutDestroyMenu(model->gencoordmenu2);
   if (model->gencoord_group_menu)
      glutDestroyMenu(model->gencoord_group_menu);
   glutDestroyMenu(model->momentgencmenu);
   glutDestroyMenu(model->momentarmgencmenu);
   glutDestroyMenu(model->momentarmnumgencmenu);
   glutDestroyMenu(model->maxmomentgencmenu);
   glutDestroyMenu(model->segmentmenu);
   glutDestroyMenu(model->motionplotmenu);
   glutDestroyMenu(model->motionmenu);
   glutDestroyMenu(model->motionwithrealtimemenu);
   glutDestroyMenu(model->material_menu);
   if (model->markerMenu)
      glutDestroyMenu(model->markerMenu);

   for (i = 0; i < model->numsegments; i++)
      glutDestroyMenu(model->segment[i].drawmodemenu);

   for (i = 0; i < model->numseggroups; i++)
      glutDestroyMenu(model->seggroup[i].displaymodemenu);

   for (i = 0; i < model->numworldobjects; i++)
      glutDestroyMenu(model->worldobj[i].drawmodemenu);

   glutDestroyMenu(model->dis.view_menu);
   glutDestroyMenu(model->dis.allsegsdrawmodemenu);
   glutDestroyMenu(model->dis.allligsdrawmodemenu);
   glutDestroyMenu(model->dis.allworlddrawmodemenu);
   glutDestroyMenu(model->dis.alldrawmodemenu);
   if (model->dis.eachsegdrawmodemenu > 0)
      glutDestroyMenu(model->dis.eachsegdrawmodemenu);
   glutDestroyMenu(model->dis.maindrawmodemenu);
}

#endif /* ENGINE */


double check_gencoord_wrapping(GeneralizedCoord* gc, double change)
{
   double new_value, range;

   new_value = gc->value + change;

   /* If the gencoord is not allowed to wrap, then clamp
    * it within its range of motion. You do not need to
    * check if the gencoord is unclamped because that
    * variable is only relevant when reading gencoord values
    * from a motion file or typing them into the gencoord
    * field in the ModelViewer (and in neither case is this
    * function called).
    */
   if (gc->wrap == no)
   {
      if (new_value < gc->range.start)
     return (gc->range.start);
      else if (new_value > gc->range.end)
     return (gc->range.end);
      else
     return (new_value);
   }

   range = gc->range.end - gc->range.start;

   while (new_value > gc->range.end)
      new_value -= range;

   while (new_value < gc->range.start)
      new_value += range;

   return (new_value);
}


#if ! ENGINE

/* MAKEGENCFORM: this routine makes the form for changing gencoord values.
 * It also makes the checkbox panel for setting clamp/unclamp 
 * for the gencoords and lock/unlock for the gencoords.
 */

ReturnCode makegencform(ModelStruct* ms)
{

   int i, name_len;
   Form* form;
   CheckBoxPanel* check;

   form = &ms->gencform;
   form->numoptions = ms->numgencoords;
   form->selected_item = -1;
   form->cursor_position = 0;
   form->highlight_start = 0;
   form->title = NULL;

   ms->longest_genc_name = 0;

   form->option = (FormItem*)simm_malloc(form->numoptions*sizeof(FormItem));
   if (form->option == NULL)
      return code_bad;

   for (i=0; i<form->numoptions; i++)
   {
      form->option[i].justify = yes;
      form->option[i].active = yes;
      form->option[i].visible = yes;
      form->option[i].editable = yes;
      form->option[i].use_alternate_colors = no;
      form->option[i].decimal_places = 3;
      form->option[i].data = NULL;
      mstrcpy(&form->option[i].name, ms->gencoord[i]->name);
      name_len = glueGetStringWidth(root.gfont.defaultfont, ms->gencoord[i]->name);
      if (name_len > ms->longest_genc_name)
         ms->longest_genc_name = name_len;
      SET_BOX1221(form->option[i].box,0,75,-FORM_FIELD_YSPACING*i,
         form->option[i].box.y2-FORM_FIELD_HEIGHT);
   }

   /* make the gencoord checkbox panel */
   check = &ms->gc_chpanel;
   check->title = "C";
   check->type = normal_checkbox;
   check->numoptions = ms->numgencoords;
   check->checkbox = (CheckBox*)simm_malloc(check->numoptions*sizeof(CheckBox));
   if (check->checkbox == NULL)
      error(exit_program,tool_message);

   for (i=0; i<check->numoptions; i++)
   {
      check->checkbox[i].just = right;
      check->checkbox[i].active = yes;
      check->checkbox[i].visible = yes;
      if (ms->gencoord[i]->clamped == no)
     check->checkbox[i].state = off;
      else
     check->checkbox[i].state = on;
      check->checkbox[i].name = NULL;
      check->checkbox[i].box.x1 = form->option[i].box.x1;
      check->checkbox[i].box.x2 = check->checkbox[i].box.x1 + CHECKBOX_XSIZE*2/3;
      check->checkbox[i].box.y1 = form->option[i].box.y1;
      check->checkbox[i].box.y2 = check->checkbox[i].box.y1 + CHECKBOX_YSIZE*2/3;
      check->checkbox[i].use_alternate_colors = no;
   }

   /* make the gencoord lock checkbox panel */
   check = &ms->gc_lockPanel;
   check->title = "L"; //NULL;
   check->type = normal_checkbox;
   check->numoptions = ms->numgencoords;
   check->checkbox = (CheckBox*)simm_malloc(check->numoptions*sizeof(CheckBox));
   if (check->checkbox == NULL)
      error(exit_program,tool_message);

   for (i=0; i<check->numoptions; i++)
   {
      check->checkbox[i].just = right;
      check->checkbox[i].active = yes;
      check->checkbox[i].visible = yes;
      if (ms->gencoord[i]->locked == no)
          check->checkbox[i].state = off;
      else
          check->checkbox[i].state = on;
      check->checkbox[i].name = NULL;
      check->checkbox[i].box.x1 = form->option[i].box.x1;
      check->checkbox[i].box.x2 = check->checkbox[i].box.x1 + CHECKBOX_XSIZE*2/3;
      check->checkbox[i].box.y1 = form->option[i].box.y1;
      check->checkbox[i].box.y2 = check->checkbox[i].box.y1 + CHECKBOX_YSIZE*2/3;
   }

   return code_fine;

}


ReturnCode make_dynparams_form(ModelStruct* ms)
{
   int i, name_len, num_options;
   Form* form;

   if (ms->default_muscle)
      num_options = ms->default_muscle->num_dynamic_params + 1; // one extra for muscle model
   else
      num_options = 1; // one extra for muscle model

   form = &ms->dynparamsform;
   form->numoptions = num_options;
   form->selected_item = -1;
   form->cursor_position = 0;
   form->highlight_start = 0;
   form->title = NULL;

   ms->longest_dynparam_name = 0;

   form->option = (FormItem*)simm_malloc(form->numoptions*sizeof(FormItem));
   if (form->option == NULL)
      return code_bad;

   ms->dynparamsSize = 0;
   for (i=0; i<form->numoptions; i++)
   {
      form->option[i].justify = yes;
      form->option[i].active = yes;
      form->option[i].visible = yes;
      form->option[i].editable = yes;
      form->option[i].use_alternate_colors = no;
      form->option[i].data = NULL;
      if (i == num_options - 1)
      {
         form->option[i].decimal_places = 0;
         mstrcpy(&form->option[i].name, "muscle_model");
      }
      else
      {
         form->option[i].decimal_places = 3;
         mstrcpy(&form->option[i].name, ms->default_muscle->dynamic_param_names[i]);
      }
      name_len = glueGetStringWidth(root.gfont.defaultfont, form->option[i].name);
      if (name_len > ms->longest_dynparam_name)
          ms->longest_dynparam_name = name_len;
      SET_BOX1221(form->option[i].box, 0, 90,-FORM_FIELD_YSPACING*i,
          form->option[i].box.y2-FORM_FIELD_HEIGHT);
      ms->dynparamsSize = form->option[i].box.y2 - FORM_FIELD_HEIGHT;
   }

   ms->longest_dynparam_name -= 30;

   return code_fine;
}

#endif /* ENGINE */


/* GETJOINTVARNUM: */

int getjointvarnum(char string[])
{

   if (STRINGS_ARE_EQUAL(string,"r1"))
      return (0);
   if (STRINGS_ARE_EQUAL(string,"r2"))
      return (1);
   if (STRINGS_ARE_EQUAL(string,"r3"))
      return (2);
   if (STRINGS_ARE_EQUAL(string,"tx"))
      return (3);
   if (STRINGS_ARE_EQUAL(string,"ty"))
      return (4);
   if (STRINGS_ARE_EQUAL(string,"tz"))
      return (5);
   if (STRINGS_ARE_EQUAL(string,"order"))
      return (6);
   if (STRINGS_ARE_EQUAL(string,"axis1"))
      return (7);
   if (STRINGS_ARE_EQUAL(string,"axis2"))
      return (8);
   if (STRINGS_ARE_EQUAL(string,"axis3"))
      return (9);
   if (STRINGS_ARE_EQUAL(string,"segments"))
      return (10);

   return (-1);

}



/* GETJOINTVARNAME: */

char* getjointvarname(int num)
{

   switch (num)
   {
      case 0:
         return ("r1");
      case 1:
         return ("r2");
      case 2:
         return ("r3");
      case 3:
         return ("tx");
      case 4:
         return ("ty");
      case 5:
         return ("tz");
      case 6:
         return ("order");
      case 7:
         return ("axis1");
      case 8:
         return ("axis2");
      case 9:
         return ("axis3");
      case 10:
         return ("name");
      default:
         return ("");
   }

}


#if ! ENGINE

SBoolean isVisible(double pt[])
{
   if (pt[0] >= UNDEFINED_DOUBLE)
      return no;
   if (pt[1] >= UNDEFINED_DOUBLE)
      return no;
   if (pt[2] >= UNDEFINED_DOUBLE)
      return no;
   return yes;
}

void hack_tool_updates(ModelStruct* model, int model_index)
{
   int i;
   SimmEvent se;

   /* Hack: update all the tools so they know the model was deleted.
    * This is done in cases where the updates must happen immediately,
    * rather than just pushing a SIMM event on the queue.
    */
   se.event_code = MODEL_DELETED;
   se.model_index = model_index;
   se.struct_ptr1 = (void*)model;
   se.struct_ptr2 = NULL;
   se.field1 = ZERO;
   se.field2 = ZERO;
   se.mouse_x = 0;
   se.mouse_y = 0;
   se.key_modifiers = 0;
   se.window_id = -1;
   se.object = 0;

   for (i = 0; i < TOOLBUFFER; i++)
   {
      if (tool[i].used == yes && (se.event_code & tool[i].simm_event_mask) == se.event_code)
         (*tool[i].simm_event_handler)(se);
   }
}

#endif /* ENGINE */
