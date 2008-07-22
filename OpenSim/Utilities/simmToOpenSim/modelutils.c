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
static void destroy_model_menus(ModelStruct* ms);


/*************** EXTERNED VARIABLES (declared in another file) ****************/



/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/

#if OPENSIM_CONVERTER
#define ENGINE
#endif

#ifndef ENGINE

/* DELETE_MODEL: this routine deletes a model from the system. It closes the
 * model window, frees the model structures, deletes the model window from
 * the window list, updates the pop-up model menu, and checks to make sure
 * no tools are currently set to the model.
 */

void delete_model(ModelStruct* ms)
{

   int i,j, windex;
   char buf[1024];
   MotionModelOptions* mmo;

   if ((windex = get_window_index(MODEL,ms->modelnum)) == -1)
      return;

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

   glutSetWindow(root.window[windex].win_parameters->id);

   /* Post the MODEL_DELETED event before deleting the motions
    * (which will post MOTION_DELETED events for each one). This
    * requires more checking by each tool when handling a
    * MOTION_DELETED event (to see if the model still exists or
    * not), but is more efficient because the tool will not
    * update itself for each motion deleted and then update itself
    * again when the whole model is deleted.
    */
   make_and_queue_simm_event(MODEL_DELETED,(void*)ms,ZERO,ZERO);

   for (i = 0; i < ms->motion_array_size; i++)
      if (ms->motion[i])
         delete_motion(ms, ms->motion[i]);

   destroy_model_menus(ms);

   sprintf(buf, "Deleted model %s.", ms->name);

   free_model(ms->modelnum);

   root.nummodels--;

   delete_window(root.window[windex].win_parameters->id);

   updatemodelmenu();
   
   purge_simm_events_for_struct(ms, MODEL_INPUT_EVENT);
   purge_simm_events_for_struct(ms, MODEL_ADDED);

   message(buf, 0, DEFAULT_MESSAGE_X_OFFSET);
}

static void destroy_model_menus(ModelStruct* ms)
{
   int i;

   glutDestroyMenu(ms->musclegroupmenu);
   glutDestroyMenu(ms->jointmenu);
   glutDestroyMenu(ms->jointmenu2);
   glutDestroyMenu(ms->jointmenu3);
   glutDestroyMenu(ms->jointmenu4);
   glutDestroyMenu(ms->jointmenu5);
   glutDestroyMenu(ms->xvarmenu);
   glutDestroyMenu(ms->gencoordmenu);
   glutDestroyMenu(ms->gencoordmenu2);
   if (ms->gencoord_group_menu)
      glutDestroyMenu(ms->gencoord_group_menu);
   glutDestroyMenu(ms->momentgencmenu);
   glutDestroyMenu(ms->momentarmgencmenu);
   glutDestroyMenu(ms->momentarmnumgencmenu);
   glutDestroyMenu(ms->maxmomentgencmenu);
   glutDestroyMenu(ms->doftypemenu);
   glutDestroyMenu(ms->segmentmenu);
   glutDestroyMenu(ms->motionplotmenu);
   glutDestroyMenu(ms->motionmenu);
   glutDestroyMenu(ms->material_menu);
   glutDestroyMenu(ms->markerMenu);

   for (i = 0; i < ms->numsegments; i++)
      glutDestroyMenu(ms->segment[i].drawmodemenu);
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


#ifndef ENGINE

/* MAKEGENCFORM: this routine makes the form for changing gencoord values.
 * It also makes the checkbox panel for setting clamp/unclamp 
 * for the gencoords and lock/unlock for the gencoords.
 */

ReturnCode makegencform(int mod)
{

   int i, name_len;
   Form* form;
   CheckBoxPanel* check;

   form = &model[mod]->gencform;
   form->numoptions = model[mod]->numgencoords;
   form->selected_item = -1;
   form->cursor_position = 0;
   form->highlight_start = 0;
   form->title = NULL;

   model[mod]->longest_genc_name = 0;

   form->option = (FormItem*)simm_malloc(form->numoptions*sizeof(FormItem));
   if (form->option == NULL)
      return (code_bad);

   for (i=0; i<form->numoptions; i++)
   {
      form->option[i].justify = yes;
      form->option[i].active = yes;
      form->option[i].visible = yes;
      form->option[i].editable = yes;
      form->option[i].use_alternate_colors = no;
      form->option[i].decimal_places = 3;
      form->option[i].data = NULL;
      mstrcpy(&form->option[i].name,model[mod]->gencoord[i].name);
      name_len = glueGetStringWidth(root.gfont.defaultfont,model[mod]->gencoord[i].name);
      if (name_len > model[mod]->longest_genc_name)
         model[mod]->longest_genc_name = name_len;
      SET_BOX1221(form->option[i].box,0,75,-FORM_FIELD_YSPACING*i,
         form->option[i].box.y2-FORM_FIELD_HEIGHT);
   }

   /* make the gencoord checkbox panel */
   check = &model[mod]->gc_chpanel;
   check->title = "C";
   check->x_edge = 2;
   check->y_edge = 2;
   check->type = normal_checkbox;
   check->numoptions = model[mod]->numgencoords;
   check->checkbox = (CheckBox*)simm_malloc(check->numoptions*sizeof(CheckBox));
   if (check->checkbox == NULL)
      error(exit_program,tool_message);

   for (i=0; i<check->numoptions; i++)
   {
      check->checkbox[i].just = right;
      check->checkbox[i].active = yes;
      check->checkbox[i].visible = yes;
      if (model[mod]->gencoord[i].clamped == no)
	 check->checkbox[i].state = off;
      else
	 check->checkbox[i].state = on;
      check->checkbox[i].name = NULL;
      check->checkbox[i].box.x1 = form->option[i].box.x1;
      check->checkbox[i].box.x2 = check->checkbox[i].box.x1 + CHECKBOX_XSIZE*2/3;
      check->checkbox[i].box.y1 = form->option[i].box.y1;
      check->checkbox[i].box.y2 = check->checkbox[i].box.y1 + CHECKBOX_YSIZE*2/3;
   }

   /* make the gencoord lock checkbox panel */
   check = &model[mod]->gc_lockPanel;
   check->title = "L"; //NULL;
   check->x_edge = 2;
   check->y_edge = 2;
   check->type = normal_checkbox;
   check->numoptions = model[mod]->numgencoords;
   check->checkbox = (CheckBox*)simm_malloc(check->numoptions*sizeof(CheckBox));
   if (check->checkbox == NULL)
      error(exit_program,tool_message);

   for (i=0; i<check->numoptions; i++)
   {
      check->checkbox[i].just = right;
      check->checkbox[i].active = yes;
      check->checkbox[i].visible = yes;
      if (model[mod]->gencoord[i].locked == no)
	      check->checkbox[i].state = off;
      else
	      check->checkbox[i].state = on;
      check->checkbox[i].name = NULL;
      check->checkbox[i].box.x1 = form->option[i].box.x1;
      check->checkbox[i].box.x2 = check->checkbox[i].box.x1 + CHECKBOX_XSIZE*2/3;
      check->checkbox[i].box.y1 = form->option[i].box.y1;
      check->checkbox[i].box.y2 = check->checkbox[i].box.y1 + CHECKBOX_YSIZE*2/3;
   }

   return (code_fine);

}


ReturnCode make_dynparams_form(int mod)
{
   int i, name_len;
   Form* form;

   form = &model[mod]->dynparamsform;
   form->numoptions = model[mod]->num_dynamic_params + 1; // one extra for muscle model
   form->selected_item = -1;
   form->cursor_position = 0;
   form->highlight_start = 0;
   form->title = NULL;

   model[mod]->longest_dynparam_name = 0;

   form->option = (FormItem*)simm_malloc(form->numoptions*sizeof(FormItem));
   if (form->option == NULL)
      return code_bad;

   model[mod]->dynparamsSize = 0;
   for (i=0; i<form->numoptions; i++)
   {
      form->option[i].justify = yes;
      form->option[i].active = yes;
      form->option[i].visible = yes;
      form->option[i].editable = yes;
      form->option[i].use_alternate_colors = no;
      form->option[i].data = NULL;
      if (i >= model[mod]->num_dynamic_params)
      {
         form->option[i].decimal_places = 0;
         mstrcpy(&form->option[i].name, "muscle_model");
      }
      else
      {
         form->option[i].decimal_places = 3;
         mstrcpy(&form->option[i].name, model[mod]->dynamic_param_names[i]);
      }
      name_len = glueGetStringWidth(root.gfont.defaultfont, form->option[i].name);
      if (name_len > model[mod]->longest_dynparam_name)
	      model[mod]->longest_dynparam_name = name_len;
      SET_BOX1221(form->option[i].box, 0, 90,-FORM_FIELD_YSPACING*i,
		  form->option[i].box.y2-FORM_FIELD_HEIGHT);
      model[mod]->dynparamsSize = form->option[i].box.y2 - FORM_FIELD_HEIGHT;
   }

model[mod]->longest_dynparam_name -= 30;

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


#ifndef ENGINE

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

void hack_tool_updates(ModelStruct* ms)
{
   int i;
   SimmEvent se;

   /* Hack: update all the tools so they know the model was deleted.
    * This is done in cases where the updates must happen immediately,
    * rather than just pushing a SIMM event on the queue.
    */
   se.event_code = MODEL_DELETED;
   se.struct_ptr = (void*)ms;
   se.field1 = ZERO;
   se.field2 = ZERO;
   se.mouse_x = 0;
   se.mouse_y = 0;
   se.key_modifiers = 0;
   se.window_id = -1;

   for (i = 0; i < TOOLBUFFER; i++)
   {
      if (tool[i].used == yes && (se.event_code & tool[i].simm_event_mask) == se.event_code)
         (*tool[i].simm_event_handler)(se);
   }
}

#endif /* ENGINE */
