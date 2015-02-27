/*******************************************************************************

   WETOOLS.C

   Author: Kenny Smith (based on pmtools.c by Peter Loan)

   Date: 22-OCT-98

   Copyright (c) 1992-8 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Description:

   Routines:
      makewrapeditormenus     : makes all the WrapEditor menus
      setwemenus             : sets the positions of the menus
      update_we_forms        : updates the WrapEditor form fields
      we_entervalue          : handles the editing of form fields
      draw_we_help_window    : draws the help window
      we_help_input          : handles user events in the help window
      move_we_help_text      : scrolls the help text up and down

*******************************************************************************/

#include "universal.h"
#include "wrapeditor.h"

#include "globals.h"
#include "functions.h"
#include "wefunctions.h"

#if OPENSMAC || SIMM_VIEWER
#undef ENGINE
#define ENGINE 1
#endif

#if ! ENGINE

/*************** DEFINES (for this file only) *********************************/
#define CHECK_MIN_VALUE 0
#define CHECK_MAX_VALUE 1


/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/
static char* weoptions[] = {
    "wrap object>",
    "segment>",
    "muscles>",
    "save all",
    "restore all",
    "delete",
    "+","-",
    "clear",
    "reset"
};

static char* weform[] = {
    "object name",
    "radius  x","y","z",
    "x","y","z",
    "x","y","z"
};
static char* wrap_type_panelstr[]      = { "sphere", "cylinder", "ellipsoid", "torus" };
static char* wrap_method_panelstr[]    = { "hybrid", "midpoint", "axial" };
static char* quadrant_panelstr[]       = { "x","y","z" };
static char* quadrant_checkstr[]       = { "positive","negative" };
static char* active_visible_panelstr[] = { "active", "visible", "show pts", "trackball" };
static char* transform_panelstr[]      = { "local frame", "parent frame" };


/*************** EXTERNED VARIABLES (declared in another .c file) *************/
extern WrapEditorStruct* we;
extern WindowParams*     we_win_params;
extern WinUnion*         we_win_union;

/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static void setwemenus(void);


int is_current_wrap_object(ModelStruct* ms, dpWrapObject* wo)
{
   if (we)
   {
      WEModelOptions* weop = &we->weop[ms->modelnum];

      if (wo == weop->wrap_object)
      {
         if (ms == we->model)
            return 2;
         else
            return 1;
      }
   }
   return 0;
}


void makewrapeditormenus(void)
{
   int i, thumb_thickness;
   IntBox bbox;
   Menu* ms;
   Form* form;
   CheckBoxPanel* check;

   /* make the plot popup menu */
    //TODO5.0: why is this here????????
   root.plotmenu = glueCreateMenu("Plots");

   glutAddMenuEntry("new", 1);

   /* make the main wrapeditor menu */
   ms = &we->optionsmenu;
   ms->title = NULL;
   ms->type = normal_menu;
   ms->numoptions = sizeof(weoptions)/sizeof(char*);
   ms->option = (MenuItem*)simm_malloc(ms->numoptions*sizeof(MenuItem));
   if (ms->option == NULL)
      error(exit_program,tool_message);

   for (i=0; i<ms->numoptions; i++)
   {
      ms->option[i].active = (SBoolean) (i == 0);
//     ms->option[i].visible = (SBoolean) (i == 0);
      ms->option[i].visible = yes;
      mstrcpy(&ms->option[i].name,weoptions[i]);
   }

   /* make the options form */
   form = &we->optionsform;
   form->title = NULL;
   form->selected_item = -1;
   form->cursor_position = 0;
   form->highlight_start = 0;
   form->numoptions = sizeof(weform)/sizeof(char*);
   form->option = (FormItem*)simm_malloc(form->numoptions*sizeof(FormItem));
   if (form->option == NULL)
      error(exit_program,tool_message);

   for (i=0; i<form->numoptions; i++)
   {
      form->option[i].active = yes;
      form->option[i].editable = no;
      form->option[i].visible = no;
      form->option[i].use_alternate_colors = no;
      form->option[i].justify = no;
      if (i == 4 || i == 5 || i == 6)
         form->option[i].decimal_places = 2;
      else
         form->option[i].decimal_places = 3;
      form->option[i].data = NULL;
      mstrcpy(&form->option[i].name,weform[i]);
   }

   /* make the wrap type radio-button panel */
   check = &we->wrap_type_radiopanel;
   check->title = "";
   check->type = radio_checkbox;
   check->numoptions = sizeof(wrap_type_panelstr)/sizeof(char*);
   check->checkbox = (CheckBox*)simm_malloc(check->numoptions*sizeof(CheckBox));
   if (check->checkbox == NULL)
      error(exit_program,tool_message);

   for (i=0; i<check->numoptions; i++)
   {
      check->checkbox[i].just = right;
      check->checkbox[i].active = yes;
      check->checkbox[i].visible = yes;
      mstrcpy(&check->checkbox[i].name,wrap_type_panelstr[i]);
   }

#if USER_SPECIFIED_WRAP_METHOD
   /* make the wrap plane radio-button panel */
   check = &we->wrap_method_radiopanel;
   check->title = "";
   check->type = radio_checkbox;
   check->numoptions = sizeof(wrap_method_panelstr) / sizeof(char*);
   check->checkbox = (CheckBox*)simm_malloc(check->numoptions*sizeof(CheckBox));
   if (check->checkbox == NULL)
      error(exit_program,tool_message);

   for (i=0; i<check->numoptions; i++)
   {
      check->checkbox[i].just = right;
      check->checkbox[i].active = yes;
      check->checkbox[i].visible = yes;
      mstrcpy(&check->checkbox[i].name, wrap_method_panelstr[i]);
   }
#endif

   /* make the wrap quadrant radio-button panel */
   check = &we->quadrant_radiopanel;
   check->title = "constrain to quadrant:";
   check->type = radio_checkbox;
   check->numoptions = sizeof(quadrant_panelstr)/sizeof(char*);
   check->checkbox = (CheckBox*)simm_malloc(check->numoptions*sizeof(CheckBox));
   if (check->checkbox == NULL)
      error(exit_program,tool_message);

   for (i=0; i<check->numoptions; i++)
   {
      check->checkbox[i].just = right;
      check->checkbox[i].active = yes;
      check->checkbox[i].visible = yes;
      mstrcpy(&check->checkbox[i].name,quadrant_panelstr[i]);
   }

   /* make the wrap quadrant check-button panel */
   check = &we->quadrant_checkpanel;
   check->title = "";
   check->type = normal_checkbox;
   check->numoptions = sizeof(quadrant_checkstr)/sizeof(char*);
   check->checkbox = (CheckBox*)simm_malloc(check->numoptions*sizeof(CheckBox));
   if (check->checkbox == NULL)
      error(exit_program,tool_message);

   for (i=0; i<check->numoptions; i++)
   {
      check->checkbox[i].just = right;
      check->checkbox[i].active = yes;
      check->checkbox[i].visible = yes;
      mstrcpy(&check->checkbox[i].name,quadrant_checkstr[i]);
   }

   /* make the active/visible checkbox panel */
   check = &we->active_visible_checkpanel;
   check->title = "";
   check->type = normal_checkbox;
   check->numoptions = sizeof(active_visible_panelstr)/sizeof(char*);
   check->checkbox = (CheckBox*)simm_malloc(check->numoptions*sizeof(CheckBox));
   if (check->checkbox == NULL)
      error(exit_program,tool_message);

   for (i=0; i<check->numoptions; i++)
   {
      check->checkbox[i].just = right;
      check->checkbox[i].active = yes;
      check->checkbox[i].visible = yes;
      mstrcpy(&check->checkbox[i].name,active_visible_panelstr[i]);
   }

   /* make the transform radio-button panel */
   check = &we->transform_radiopanel;
   check->title = "transform in:";
   check->type = radio_checkbox;
   check->numoptions = sizeof(transform_panelstr)/sizeof(char*);
   check->checkbox = (CheckBox*)simm_malloc(check->numoptions*sizeof(CheckBox));
   if (check->checkbox == NULL)
      error(exit_program,tool_message);

   for (i=0; i<check->numoptions; i++)
   {
      check->checkbox[i].just = right;
      check->checkbox[i].active = yes;
      check->checkbox[i].visible = yes;
      mstrcpy(&check->checkbox[i].name,transform_panelstr[i]);
   }

   setwemenus();

   /* Make the scroll bar for the tool window itself */
   bbox.x1 = 0;
   bbox.x2 = WE_SLIDER_WIDTH;
   bbox.y1 = 0;
   bbox.y2 = 100;         /* is changed during every redraw */
   thumb_thickness = 50;  /* is changed during every redraw */

   make_slider(&we->win_slider,vertical_slider,bbox,thumb_thickness,
           (double)we->canvas_height,0.0,(double)we->canvas_height,10.0,NULL,NULL);

}



static void setwemenus(void)
{

   Menu* ms;
   Form* form;
   CheckBoxPanel* check;

   /* Set the positions of the main menu boxes */
   ms = &we->optionsmenu;

   SET_BOX1221(ms->option[WE_WRAP_OBJECT].box,
           -85,ms->option[WE_WRAP_OBJECT].box.x1+110,
           5,ms->option[WE_WRAP_OBJECT].box.y2-MENU_ITEM_HEIGHT);

   SET_BOX1221(ms->option[WE_CHOOSE_SEGMENT].box,ms->option[WE_WRAP_OBJECT].box.x1,
           ms->option[WE_WRAP_OBJECT].box.x2,ms->option[WE_WRAP_OBJECT].box.y1,
           ms->option[WE_CHOOSE_SEGMENT].box.y2-MENU_ITEM_HEIGHT);

   SET_BOX1221(ms->option[WE_MUSCLEGROUPS].box,ms->option[WE_WRAP_OBJECT].box.x1,
           ms->option[WE_WRAP_OBJECT].box.x2,ms->option[WE_CHOOSE_SEGMENT].box.y1,
           ms->option[WE_MUSCLEGROUPS].box.y2-MENU_ITEM_HEIGHT);

   SET_BOX1221(ms->option[WE_SAVE_ALL].box,ms->option[WE_WRAP_OBJECT].box.x1,
           ms->option[WE_WRAP_OBJECT].box.x2,
           ms->option[WE_MUSCLEGROUPS].box.y1 - 10,
           ms->option[WE_SAVE_ALL].box.y2-MENU_ITEM_HEIGHT);

   SET_BOX1221(ms->option[WE_RESTORE_ALL].box,ms->option[WE_WRAP_OBJECT].box.x1,
           ms->option[WE_WRAP_OBJECT].box.x2,
           ms->option[WE_SAVE_ALL].box.y1 - 5,
           ms->option[WE_RESTORE_ALL].box.y2-MENU_ITEM_HEIGHT);

   SET_BOX1221(ms->option[WE_DELETE_OBJECT].box,ms->option[WE_WRAP_OBJECT].box.x1,
           ms->option[WE_WRAP_OBJECT].box.x2,
           ms->option[WE_RESTORE_ALL].box.y1 - 5,
           ms->option[WE_DELETE_OBJECT].box.y2-MENU_ITEM_HEIGHT);

   SET_BOX1221(ms->option[WE_APPLY_POS_XFORM].box,
           ms->option[WE_WRAP_OBJECT].box.x1 + 38,
           ms->option[WE_APPLY_POS_XFORM].box.x1 + 30,
           ms->option[WE_DELETE_OBJECT].box.y1 - 143,
           ms->option[WE_APPLY_POS_XFORM].box.y2 - MENU_ITEM_HEIGHT);

   SET_BOX1221(ms->option[WE_APPLY_NEG_XFORM].box,
           ms->option[WE_APPLY_POS_XFORM].box.x2,
           ms->option[WE_APPLY_NEG_XFORM].box.x1 + 30,
           ms->option[WE_APPLY_POS_XFORM].box.y2,
           ms->option[WE_APPLY_NEG_XFORM].box.y2 - MENU_ITEM_HEIGHT);

   SET_BOX1221(ms->option[WE_CLEAR_XFORM].box,
           ms->option[WE_APPLY_NEG_XFORM].box.x2 + 5,
           ms->option[WE_CLEAR_XFORM].box.x1 + 60,
           ms->option[WE_APPLY_NEG_XFORM].box.y2,
           ms->option[WE_CLEAR_XFORM].box.y2 - MENU_ITEM_HEIGHT);

   SET_BOX1221(ms->option[WE_IDENTITY_XFORM].box,
           ms->option[WE_CLEAR_XFORM].box.x2 + 230,
           ms->option[WE_IDENTITY_XFORM].box.x1 + 75,
           ms->option[WE_CLEAR_XFORM].box.y1 - 10,
           ms->option[WE_IDENTITY_XFORM].box.y2 - MENU_ITEM_HEIGHT);

   /* Set the positions of the options form boxes */
   form = &we->optionsform;

   form->option[WE_OBJECT_NAME].box.x1 = 0;
   form->option[WE_OBJECT_NAME].box.x2 = form->option[WE_OBJECT_NAME].box.x1 + 275;
   form->option[WE_OBJECT_NAME].box.y2 = 0;
   form->option[WE_OBJECT_NAME].box.y1 = form->option[WE_OBJECT_NAME].box.y2 - FORM_FIELD_HEIGHT;

   form->option[WE_RADIUS_X].box.x1 = form->option[WE_OBJECT_NAME].box.x1;
   form->option[WE_RADIUS_X].box.x2 = form->option[WE_RADIUS_X].box.x1 + 70;
   form->option[WE_RADIUS_X].box.y2 = form->option[WE_OBJECT_NAME].box.y2 - 2 * FORM_FIELD_YSPACING;
   form->option[WE_RADIUS_X].box.y1 = form->option[WE_RADIUS_X].box.y2 - FORM_FIELD_HEIGHT;

   form->option[WE_RADIUS_Y].box.x1 = form->option[WE_RADIUS_X].box.x1;
   form->option[WE_RADIUS_Y].box.x2 = form->option[WE_RADIUS_X].box.x2;
   form->option[WE_RADIUS_Y].box.y1 = form->option[WE_RADIUS_X].box.y1 - FORM_FIELD_YSPACING;
   form->option[WE_RADIUS_Y].box.y2 = form->option[WE_RADIUS_Y].box.y1 + FORM_FIELD_HEIGHT;

   form->option[WE_RADIUS_Z].box.x1 = form->option[WE_RADIUS_Y].box.x1;
   form->option[WE_RADIUS_Z].box.x2 = form->option[WE_RADIUS_Y].box.x2;
   form->option[WE_RADIUS_Z].box.y1 = form->option[WE_RADIUS_Y].box.y1 - FORM_FIELD_YSPACING;
   form->option[WE_RADIUS_Z].box.y2 = form->option[WE_RADIUS_Z].box.y1 + FORM_FIELD_HEIGHT;

   form->option[WE_TRANSLATE_X].box.x1 = form->option[WE_OBJECT_NAME].box.x1 - 215;
   form->option[WE_TRANSLATE_X].box.x2 = form->option[WE_TRANSLATE_X].box.x1 + 60;
   form->option[WE_TRANSLATE_X].box.y1 = form->option[WE_OBJECT_NAME].box.y1 - 236;
   form->option[WE_TRANSLATE_X].box.y2 = form->option[WE_TRANSLATE_X].box.y1 + FORM_FIELD_HEIGHT;

   form->option[WE_TRANSLATE_Y].box.x1 = form->option[WE_TRANSLATE_X].box.x1;
   form->option[WE_TRANSLATE_Y].box.x2 = form->option[WE_TRANSLATE_X].box.x2;
   form->option[WE_TRANSLATE_Y].box.y1 = form->option[WE_TRANSLATE_X].box.y1 - FORM_FIELD_YSPACING + 7;
   form->option[WE_TRANSLATE_Y].box.y2 = form->option[WE_TRANSLATE_Y].box.y1 + FORM_FIELD_HEIGHT;

   form->option[WE_TRANSLATE_Z].box.x1 = form->option[WE_TRANSLATE_Y].box.x1;
   form->option[WE_TRANSLATE_Z].box.x2 = form->option[WE_TRANSLATE_Y].box.x2;
   form->option[WE_TRANSLATE_Z].box.y1 = form->option[WE_TRANSLATE_Y].box.y1 - FORM_FIELD_YSPACING + 7;
   form->option[WE_TRANSLATE_Z].box.y2 = form->option[WE_TRANSLATE_Z].box.y1 + FORM_FIELD_HEIGHT;

   form->option[WE_ROTATE_X].box.x1 = form->option[WE_TRANSLATE_X].box.x2 + 30;
   form->option[WE_ROTATE_X].box.x2 = form->option[WE_ROTATE_X].box.x1 + 60;
   form->option[WE_ROTATE_X].box.y1 = form->option[WE_TRANSLATE_X].box.y1;
   form->option[WE_ROTATE_X].box.y2 = form->option[WE_ROTATE_X].box.y1 + FORM_FIELD_HEIGHT;

   form->option[WE_ROTATE_Y].box.x1 = form->option[WE_ROTATE_X].box.x1;
   form->option[WE_ROTATE_Y].box.x2 = form->option[WE_ROTATE_X].box.x2;
   form->option[WE_ROTATE_Y].box.y1 = form->option[WE_ROTATE_X].box.y1 - FORM_FIELD_YSPACING + 7;
   form->option[WE_ROTATE_Y].box.y2 = form->option[WE_ROTATE_Y].box.y1 + FORM_FIELD_HEIGHT;

   form->option[WE_ROTATE_Z].box.x1 = form->option[WE_ROTATE_Y].box.x1;
   form->option[WE_ROTATE_Z].box.x2 = form->option[WE_ROTATE_Y].box.x2;
   form->option[WE_ROTATE_Z].box.y1 = form->option[WE_ROTATE_Y].box.y1 - FORM_FIELD_YSPACING + 7;
   form->option[WE_ROTATE_Z].box.y2 = form->option[WE_ROTATE_Z].box.y1 + FORM_FIELD_HEIGHT;

   /* set the positions of the wrap type radio buttons */
   check = &we->wrap_type_radiopanel;

   check->checkbox[WE_SPHERE].box.x1 = 0;
   check->checkbox[WE_SPHERE].box.x2 = check->checkbox[WE_SPHERE].box.x1 + CHECKBOX_XSIZE;
   check->checkbox[WE_SPHERE].box.y2 = 0;
   check->checkbox[WE_SPHERE].box.y1 = check->checkbox[WE_SPHERE].box.y2 - CHECKBOX_YSIZE;

   check->checkbox[WE_CYLINDER].box.x1 = check->checkbox[WE_SPHERE].box.x1;
   check->checkbox[WE_CYLINDER].box.x2 = check->checkbox[WE_SPHERE].box.x2;
   check->checkbox[WE_CYLINDER].box.y2 = check->checkbox[WE_SPHERE].box.y2 - FORM_FIELD_HEIGHT;
   check->checkbox[WE_CYLINDER].box.y1 = check->checkbox[WE_CYLINDER].box.y2 - CHECKBOX_YSIZE;

   check->checkbox[WE_ELLIPSOID].box.x1 = check->checkbox[WE_CYLINDER].box.x1;
   check->checkbox[WE_ELLIPSOID].box.x2 = check->checkbox[WE_CYLINDER].box.x2;
   check->checkbox[WE_ELLIPSOID].box.y2 = check->checkbox[WE_CYLINDER].box.y2 - FORM_FIELD_HEIGHT;
   check->checkbox[WE_ELLIPSOID].box.y1 = check->checkbox[WE_ELLIPSOID].box.y2 - CHECKBOX_YSIZE;

   check->checkbox[WE_TORUS].box.x1 = check->checkbox[WE_ELLIPSOID].box.x1;
   check->checkbox[WE_TORUS].box.x2 = check->checkbox[WE_ELLIPSOID].box.x2;
   check->checkbox[WE_TORUS].box.y2 = check->checkbox[WE_ELLIPSOID].box.y2 - FORM_FIELD_HEIGHT;
   check->checkbox[WE_TORUS].box.y1 = check->checkbox[WE_TORUS].box.y2 - CHECKBOX_YSIZE;

#if USER_SPECIFIED_WRAP_METHOD
   /* set the positions of the wrap plane radio buttons */
   check = &we->wrap_method_radiopanel;

   check->checkbox[0].box.x1 = 0;
   check->checkbox[0].box.x2 = check->checkbox[0].box.x1 + CHECKBOX_XSIZE;
   check->checkbox[0].box.y2 = 0;
   check->checkbox[0].box.y1 = check->checkbox[0].box.y2 - CHECKBOX_YSIZE;

   check->checkbox[1].box.x1 = check->checkbox[0].box.x1;
   check->checkbox[1].box.x2 = check->checkbox[0].box.x2;
   check->checkbox[1].box.y2 = check->checkbox[0].box.y2 - FORM_FIELD_HEIGHT;
   check->checkbox[1].box.y1 = check->checkbox[1].box.y2 - CHECKBOX_YSIZE;

   check->checkbox[2].box.x1 = check->checkbox[1].box.x1;
   check->checkbox[2].box.x2 = check->checkbox[1].box.x2;
   check->checkbox[2].box.y2 = check->checkbox[1].box.y2 - FORM_FIELD_HEIGHT;
   check->checkbox[2].box.y1 = check->checkbox[2].box.y2 - CHECKBOX_YSIZE;
#endif

   /* set the positions of the wrap quadrant radio buttons */
   check = &we->quadrant_radiopanel;

   check->checkbox[WE_X_QUADRANT].box.x1 = 10;
   check->checkbox[WE_X_QUADRANT].box.x2 = check->checkbox[WE_X_QUADRANT].box.x1 + CHECKBOX_XSIZE;
   check->checkbox[WE_X_QUADRANT].box.y2 = -10;
   check->checkbox[WE_X_QUADRANT].box.y1 = check->checkbox[WE_X_QUADRANT].box.y2 - CHECKBOX_YSIZE;

   check->checkbox[WE_Y_QUADRANT].box.x1 = check->checkbox[WE_X_QUADRANT].box.x1;
   check->checkbox[WE_Y_QUADRANT].box.x2 = check->checkbox[WE_X_QUADRANT].box.x2;
   check->checkbox[WE_Y_QUADRANT].box.y2 = check->checkbox[WE_X_QUADRANT].box.y2 - FORM_FIELD_HEIGHT;
   check->checkbox[WE_Y_QUADRANT].box.y1 = check->checkbox[WE_Y_QUADRANT].box.y2 - CHECKBOX_YSIZE;

   check->checkbox[WE_Z_QUADRANT].box.x1 = check->checkbox[WE_Y_QUADRANT].box.x1;
   check->checkbox[WE_Z_QUADRANT].box.x2 = check->checkbox[WE_Y_QUADRANT].box.x2;
   check->checkbox[WE_Z_QUADRANT].box.y2 = check->checkbox[WE_Y_QUADRANT].box.y2 - FORM_FIELD_HEIGHT;
   check->checkbox[WE_Z_QUADRANT].box.y1 = check->checkbox[WE_Z_QUADRANT].box.y2 - CHECKBOX_YSIZE;

   /* set the positions of the wrap quadrant check buttons */
   check = &we->quadrant_checkpanel;

   check->checkbox[WE_POSITIVE_QUADRANT].box.x1 = we->quadrant_radiopanel.checkbox[WE_X_QUADRANT].box.x2 + 40;
   check->checkbox[WE_POSITIVE_QUADRANT].box.x2 = check->checkbox[WE_POSITIVE_QUADRANT].box.x1 + CHECKBOX_XSIZE;
   check->checkbox[WE_POSITIVE_QUADRANT].box.y2 = we->quadrant_radiopanel.checkbox[WE_X_QUADRANT].box.y2;
   check->checkbox[WE_POSITIVE_QUADRANT].box.y1 = check->checkbox[WE_POSITIVE_QUADRANT].box.y2 - CHECKBOX_YSIZE;

   check->checkbox[WE_NEGATIVE_QUADRANT].box.x1 = check->checkbox[WE_POSITIVE_QUADRANT].box.x1;
   check->checkbox[WE_NEGATIVE_QUADRANT].box.x2 = check->checkbox[WE_NEGATIVE_QUADRANT].box.x1 + CHECKBOX_XSIZE;
   check->checkbox[WE_NEGATIVE_QUADRANT].box.y2 = check->checkbox[WE_POSITIVE_QUADRANT].box.y2 - FORM_FIELD_HEIGHT;
   check->checkbox[WE_NEGATIVE_QUADRANT].box.y1 = check->checkbox[WE_NEGATIVE_QUADRANT].box.y2 - CHECKBOX_YSIZE;

   /* set the positions of the checkboxes */
   check = &we->active_visible_checkpanel;

   check->checkbox[WE_ACTIVE_CHBOX].box.x1 = 0;
   check->checkbox[WE_ACTIVE_CHBOX].box.x2 = check->checkbox[WE_ACTIVE_CHBOX].box.x1 + CHECKBOX_XSIZE;
   check->checkbox[WE_ACTIVE_CHBOX].box.y2 = 13;
   check->checkbox[WE_ACTIVE_CHBOX].box.y1 = check->checkbox[WE_ACTIVE_CHBOX].box.y2 - CHECKBOX_YSIZE;

   check->checkbox[WE_VISIBLE_CHBOX].box.x1 = check->checkbox[WE_ACTIVE_CHBOX].box.x1;
   check->checkbox[WE_VISIBLE_CHBOX].box.x2 = check->checkbox[WE_ACTIVE_CHBOX].box.x2;
   check->checkbox[WE_VISIBLE_CHBOX].box.y2 = check->checkbox[WE_ACTIVE_CHBOX].box.y2 - FORM_FIELD_HEIGHT;
   check->checkbox[WE_VISIBLE_CHBOX].box.y1 = check->checkbox[WE_VISIBLE_CHBOX].box.y2 - CHECKBOX_YSIZE;

   check->checkbox[WE_SHOW_PTS_CHBOX].box.x1 = check->checkbox[WE_VISIBLE_CHBOX].box.x1;
   check->checkbox[WE_SHOW_PTS_CHBOX].box.x2 = check->checkbox[WE_VISIBLE_CHBOX].box.x2;
   check->checkbox[WE_SHOW_PTS_CHBOX].box.y2 = check->checkbox[WE_VISIBLE_CHBOX].box.y2 - FORM_FIELD_HEIGHT;
   check->checkbox[WE_SHOW_PTS_CHBOX].box.y1 = check->checkbox[WE_SHOW_PTS_CHBOX].box.y2 - CHECKBOX_YSIZE;

   check->checkbox[WE_TRACKBALL_CHBOX].box.x1 = -230;
   check->checkbox[WE_TRACKBALL_CHBOX].box.x2 = check->checkbox[WE_TRACKBALL_CHBOX].box.x1 + CHECKBOX_XSIZE;
   check->checkbox[WE_TRACKBALL_CHBOX].box.y2 = -162;
   check->checkbox[WE_TRACKBALL_CHBOX].box.y1 = check->checkbox[WE_TRACKBALL_CHBOX].box.y2 - CHECKBOX_YSIZE;

   /* set the positions of the checkboxes */
   check = &we->transform_radiopanel;

   check->checkbox[WE_LOCAL_FRAME_RBTN].box.x1 = 10;
   check->checkbox[WE_LOCAL_FRAME_RBTN].box.x2 = check->checkbox[WE_LOCAL_FRAME_RBTN].box.x1 + CHECKBOX_XSIZE;
   check->checkbox[WE_LOCAL_FRAME_RBTN].box.y2 = -10;
   check->checkbox[WE_LOCAL_FRAME_RBTN].box.y1 = check->checkbox[WE_LOCAL_FRAME_RBTN].box.y2 - CHECKBOX_YSIZE;

   check->checkbox[WE_PARENT_FRAME_RBTN].box.x1 = check->checkbox[WE_LOCAL_FRAME_RBTN].box.x1;
   check->checkbox[WE_PARENT_FRAME_RBTN].box.x2 = check->checkbox[WE_LOCAL_FRAME_RBTN].box.x2;
   check->checkbox[WE_PARENT_FRAME_RBTN].box.y2 = check->checkbox[WE_LOCAL_FRAME_RBTN].box.y2 - FORM_FIELD_HEIGHT;
   check->checkbox[WE_PARENT_FRAME_RBTN].box.y1 = check->checkbox[WE_PARENT_FRAME_RBTN].box.y2 - CHECKBOX_YSIZE;
}



void update_we_forms(void)
{
   Form*           form = &we->optionsform;
   WEModelOptions* weop = &we->weop[we->model->modelnum];
   
   if (weop->wrap_object)
   {
      dpWrapObject* wo = weop->wrap_object;
      
      storeStringInForm(&form->option[WE_OBJECT_NAME], wo->name);
      storeDoubleInForm(&form->option[WE_RADIUS_X], wo->radius[0], 4);
      
      if (wo->wrap_type == dpWrapCylinder)
         storeDoubleInForm(&form->option[WE_RADIUS_Y], wo->height, 4);
      else
         storeDoubleInForm(&form->option[WE_RADIUS_Y], wo->radius[1], 4);
      
      storeDoubleInForm(&form->option[WE_RADIUS_Z], wo->radius[2], 4);
   }
   else {
      storeStringInForm(&form->option[WE_OBJECT_NAME], NULL);
      storeStringInForm(&form->option[WE_RADIUS_X], NULL);
      storeStringInForm(&form->option[WE_RADIUS_Y], NULL);
      storeStringInForm(&form->option[WE_RADIUS_Z], NULL);
   }
   
   if (weop->translate.xyz[0] != 0.0)
      storeDoubleInForm(&form->option[WE_TRANSLATE_X], weop->translate.xyz[0], 4);
   else
      storeStringInForm(&form->option[WE_TRANSLATE_X], NULL);
   
   if (weop->translate.xyz[1] != 0.0)
      storeDoubleInForm(&form->option[WE_TRANSLATE_Y], weop->translate.xyz[1], 4);
   else
      storeStringInForm(&form->option[WE_TRANSLATE_Y], NULL);
   
   if (weop->translate.xyz[2] != 0.0)
      storeDoubleInForm(&form->option[WE_TRANSLATE_Z], weop->translate.xyz[2], 4);
   else
      storeStringInForm(&form->option[WE_TRANSLATE_Z], NULL);


   if (weop->rotate.xyz[0] != 0.0)
      storeDoubleInForm(&form->option[WE_ROTATE_X], weop->rotate.xyz[0], 3);
   else
      storeStringInForm(&form->option[WE_ROTATE_X], NULL);
   
   if (weop->rotate.xyz[1] != 0.0)
      storeDoubleInForm(&form->option[WE_ROTATE_Y], weop->rotate.xyz[1], 3);
   else
      storeStringInForm(&form->option[WE_ROTATE_Y], NULL);
   
   if (weop->rotate.xyz[2] != 0.0)
      storeDoubleInForm(&form->option[WE_ROTATE_Z], weop->rotate.xyz[2], 3);
   else
      storeStringInForm(&form->option[WE_ROTATE_Z], NULL);
}



void we_entervalue(SimmEvent se)
{
   int i, j, rc;
   double rd;
   Form* form = &we->optionsform;
   WEModelOptions* weop = &we->weop[we->model->modelnum];
   dpWrapObject* wo = weop->wrap_object;
   TextFieldAction tfa;
   
   switch (we->selected_item)
   {
      case WE_OBJECT_NAME:
         rc = get_string(form, se, &tfa, no);
         if (rc == STRING_NOT_DONE)
            return;
         if (rc != KEEP_OLD_STRING)
         {
            // Make sure the wrap object has a unique name.
            for (i=0; i<we->model->num_wrap_objects; i++)
            {
               if (we->model->wrapobj[i] != wo && STRINGS_ARE_EQUAL(we->model->wrapobj[i]->name, form->option[form->selected_item].valuestr))
               {
                  (void)sprintf(errorbuffer, "The name %s is already being used by another wrap object.", form->option[form->selected_item].valuestr);
                  error(none, errorbuffer);
                  break;
               }
            }
            if (i < we->model->num_wrap_objects)
               break;

            free(wo->name);
            mstrcpy(&wo->name, form->option[form->selected_item].valuestr);

            /* Send simm event to tell Muscle Editor about the name change. */
            //make_and_queue_simm_event(WRAP_OBJECT_CHANGED, (void*)we->model, weop->wrap_object, ZERO);
         }
         break;

      default:
          rd = get_double(form, se, &tfa);
          if (rd == DOUBLE_NOT_DONE)
             return;
         
         if (rd != KEEPOLDDOUBLE && wo)
         {
            switch (we->selected_item)
            {
               case WE_RADIUS_X:
                 if (rd > 0.0)
                 {
                    wo->radius[0] = rd;
                    wo->display_list_is_stale = dpYes;
                    inval_model_wrapping(we->model, weop->wrap_object);
                 }
                 else
                 {
                    error(none, "Value must be greater than zero.");
                 }
                  break;
               
               case WE_RADIUS_Y:
                 if (rd > 0.0)
                 {
                    wo->radius[1] = rd;
                    if (wo->wrap_type == dpWrapCylinder)
                       wo->height = rd;
                    wo->display_list_is_stale = dpYes;
                    inval_model_wrapping(we->model, weop->wrap_object);
                 }
                 else
                 {
                    error(none, "Value must be greater than zero.");
                 }
                  break;
               
               case WE_RADIUS_Z:
                 if (rd > 0.0)
                 {
                    wo->radius[2] = rd;
                    wo->display_list_is_stale = dpYes;
                    inval_model_wrapping(we->model, weop->wrap_object);
                 }
                 else
                 {
                    error(none, "Value must be greater than zero.");
                 }
                  break;
               
               case WE_TRANSLATE_X: weop->translate.xyz[0] = rd; break;
               case WE_TRANSLATE_Y: weop->translate.xyz[1] = rd; break;
               case WE_TRANSLATE_Z: weop->translate.xyz[2] = rd; break;
               
               case WE_ROTATE_X: weop->rotate.xyz[0] = rd; break;
               case WE_ROTATE_Y: weop->rotate.xyz[1] = rd; break;
               case WE_ROTATE_Z: weop->rotate.xyz[2] = rd; break;
            }
            
         }
         break;
   }

   if (tfa == goto_previous_field || tfa == goto_next_field)
   {
      we->selected_item = find_next_active_field(form,we->selected_item,tfa);
      form->selected_item = we->selected_item;
      form->cursor_position = strlen(form->option[form->selected_item].valuestr);
      form->highlight_start = 0;
   }
   else
   {
      we->current_mode = WE_TOP_LEVEL;
      we->selected_item = -1;
      form->selected_item = -1;
      form->cursor_position = 0;
      form->highlight_start = form->cursor_position;
   }
   
   /* send WRAP_OBJECT_CHANGED event to tell the Muscle Editor about the change. */
   make_and_queue_simm_event(WRAP_OBJECT_CHANGED, we->model->modelnum, wo, NULL, ZERO, ZERO);
   update_we_forms();
   display_wrapeditor(we_win_params, we_win_union);
}


public void draw_we_help_window(WindowParams* win_parameters, WinUnion* win_struct)
{
   draw_help_window(&we->help);
}



public void we_help_input(WindowParams* win_parameters, WinUnion* win_struct,
              SimmEvent se)
{
   if (se.field1 == window_shut)
   {
      delete_window(we->help.window_id);
      we->help.window_id = NO_WINDOW;
      return;
   }

   if (se.field1 != select_button || se.field2 != key_pressed)
      return;

   if (check_slider(&we->help.sl,se,
            move_we_help_text,DUMMY_INT) == yes)
      draw_help_window(&we->help);
}



public void move_we_help_text(int dummy_int, double slider_value, double delta)
{
   we->help.starting_line = we->help.sl.max_value - slider_value;

   draw_help_window(&we->help);
}



void apply_xform_to_wrapobj (double factor)
{
   if (we->model)
   {
      WEModelOptions* weop = &we->weop[we->model->modelnum];
      
      if (weop->wrap_object)
      {
         dpCoord3D translate = { 0.0, 0.0, 0.0 };
         dpCoord3D rotate    = { 0.0, 0.0, 0.0 };
         DMatrix m;
         
         dpWrapObject* wo = weop->wrap_object;
         
         /* collect transform input from form fields:
          */
         if (*we->optionsform.option[WE_TRANSLATE_X].valuestr)
            sscanf(we->optionsform.option[WE_TRANSLATE_X].valuestr, "%lf", &translate.xyz[0]);
         
         if (*we->optionsform.option[WE_TRANSLATE_Y].valuestr)
            sscanf(we->optionsform.option[WE_TRANSLATE_Y].valuestr, "%lf", &translate.xyz[1]);
         
         if (*we->optionsform.option[WE_TRANSLATE_Z].valuestr)
            sscanf(we->optionsform.option[WE_TRANSLATE_Z].valuestr, "%lf", &translate.xyz[2]);
         
         if (*we->optionsform.option[WE_ROTATE_X].valuestr)
            sscanf(we->optionsform.option[WE_ROTATE_X].valuestr, "%lf", &rotate.xyz[0]);
         
         if (*we->optionsform.option[WE_ROTATE_Y].valuestr)
            sscanf(we->optionsform.option[WE_ROTATE_Y].valuestr, "%lf", &rotate.xyz[1]);
         
         if (*we->optionsform.option[WE_ROTATE_Z].valuestr)
            sscanf(we->optionsform.option[WE_ROTATE_Z].valuestr, "%lf", &rotate.xyz[2]);
         
         /* build transform matrix:
          */
         identity_matrix(m);
         rotate_matrix_axis_angle(m, wo->rotation_axis.xyz, wo->rotation_angle);
         
         if (weop->xform_frame == WE_LOCAL_FRAME)
         {
             x_rotate_matrix_bodyfixed(m, factor * rotate.xyz[0] * DTOR);
             y_rotate_matrix_bodyfixed(m, factor * rotate.xyz[1] * DTOR);
             z_rotate_matrix_bodyfixed(m, factor * rotate.xyz[2] * DTOR);
         } else {
             x_rotate_matrix_spacefixed(m, factor * rotate.xyz[0] * DTOR);
             y_rotate_matrix_spacefixed(m, factor * rotate.xyz[1] * DTOR);
             z_rotate_matrix_spacefixed(m, factor * rotate.xyz[2] * DTOR);
         }
         extract_rotation(m, &wo->rotation_axis, &wo->rotation_angle);
         
         if (weop->xform_frame == WE_LOCAL_FRAME)
         {
             dpCoord3D v;

             v.xyz[0] = factor * translate.xyz[0];
             v.xyz[1] = factor * translate.xyz[1];
             v.xyz[2] = factor * translate.xyz[2];

             transform_vec(m, v.xyz);
             
             wo->translation.xyz[0] += v.xyz[0];
             wo->translation.xyz[1] += v.xyz[1];
             wo->translation.xyz[2] += v.xyz[2];
             
             wo->undeformed_translation.xyz[0] += v.xyz[0];
             wo->undeformed_translation.xyz[1] += v.xyz[1];
             wo->undeformed_translation.xyz[2] += v.xyz[2];
         } else {
             wo->translation.xyz[0] += factor * translate.xyz[0];
             wo->translation.xyz[1] += factor * translate.xyz[1];
             wo->translation.xyz[2] += factor * translate.xyz[2];
             
             wo->undeformed_translation.xyz[0] += factor * translate.xyz[0];
             wo->undeformed_translation.xyz[1] += factor * translate.xyz[1];
             wo->undeformed_translation.xyz[2] += factor * translate.xyz[2];
         }
         
         /* apply transform to wrap object:
          */
         wo->xforms_valid = no;
         recalc_xforms(wo);
         inval_model_wrapping(we->model, weop->wrap_object);
         /* send WRAP_OBJECT_CHANGED event to tell the Muscle Editor about the change. */
         make_and_queue_simm_event(WRAP_OBJECT_CHANGED, we->model->modelnum, wo, NULL, ZERO, ZERO);
         queue_model_redraw(we->model);
      }
   }
}

void clear_we_xform_form ()
{
   storeStringInForm(&we->optionsform.option[WE_TRANSLATE_X], NULL);
   storeStringInForm(&we->optionsform.option[WE_TRANSLATE_Y], NULL);
   storeStringInForm(&we->optionsform.option[WE_TRANSLATE_Z], NULL);
   
   storeStringInForm(&we->optionsform.option[WE_ROTATE_X], NULL);
   storeStringInForm(&we->optionsform.option[WE_ROTATE_Y], NULL);
   storeStringInForm(&we->optionsform.option[WE_ROTATE_Z], NULL);
   
   if (we->model)
   {
      WEModelOptions* weop = &we->weop[we->model->modelnum];
      
      weop->translate.xyz[0] = 0.0;
      weop->translate.xyz[1] = 0.0;
      weop->translate.xyz[2] = 0.0;
      
      weop->rotate.xyz[0] = 0.0;
      weop->rotate.xyz[1] = 0.0;
      weop->rotate.xyz[2] = 0.0;
   }
   display_wrapeditor(we_win_params, we_win_union);
}



void reset_wrapobj_xform ()
{
   if (we->model)
   {
      WEModelOptions* weop = &we->weop[we->model->modelnum];
      
      if (weop->wrap_object)
      {
         dpWrapObject* wo = weop->wrap_object;
         
         wo->undeformed_translation.xyz[0] = wo->translation.xyz[0] = 0.0;
         wo->undeformed_translation.xyz[1] = wo->translation.xyz[1] = 0.0;
         wo->undeformed_translation.xyz[2] = wo->translation.xyz[2] = 0.0;
         
         wo->rotation_axis.xyz[0] = 1.0;
         wo->rotation_axis.xyz[1] = 0.0;
         wo->rotation_axis.xyz[2] = 0.0;
         
         wo->rotation_angle = 0.0;
         
         wo->xforms_valid = no;
         
         recalc_xforms(wo);
         inval_model_wrapping(we->model, weop->wrap_object);
         /* send WRAP_OBJECT_CHANGED event to tell the Muscle Editor about the change. */
         make_and_queue_simm_event(WRAP_OBJECT_CHANGED, we->model->modelnum, wo, NULL, ZERO, ZERO);
         queue_model_redraw(we->model);
         display_wrapeditor(we_win_params, we_win_union);
      }
   }
}

/* save all the wrap objects */
void save_all_wrap_objects (ModelStruct* ms)
{
   if (ms && ms->num_wrap_objects > 0)
   {
      int i, j, k;
      size_t nBytes = ms->num_wrap_objects * sizeof(dpWrapObject);

      /* dealloc previous saved wrap object arrays */
      if (ms->save.wrap_object)
      {
         for (i = 0; i < ms->save.num_wrap_objects; i++)
            FREE_IFNOTNULL(ms->save.wrap_object[i].name);
         FREE_IFNOTNULL(ms->save.wrap_object);
      }

      /* alloc new saved wrap object arrays */
      ms->save.wrap_object = (dpWrapObject*) simm_malloc(nBytes);

      if (ms->save.wrap_object)
      {
         ms->save.num_wrap_objects = ms->num_wrap_objects;

         /* copy wrap objects and their names */
         memcpy(ms->save.wrap_object, ms->wrapobj, nBytes);

         for (i = 0; i < ms->num_wrap_objects; i++)
            mstrcpy(&ms->save.wrap_object[i].name, ms->wrapobj[i]->name);

         /* save muscle -> wrap object associations */
         if (ms->save.muscwrap_associations)
         {
            free(ms->save.muscwrap_associations);
            ms->save.muscwrap_associations = NULL;
         }

         ms->save.num_muscwrap_associations = 0;

         for (i = 0; i < ms->nummuscles; i++)
            ms->save.num_muscwrap_associations += ms->muscle[i]->numWrapStructs;

         if (ms->save.num_muscwrap_associations > 0)
         {
            ms->save.muscwrap_associations = (MuscWrapAssoc*) simm_malloc(
                       ms->save.num_muscwrap_associations * sizeof(MuscWrapAssoc));

            if (ms->save.muscwrap_associations)
            {
               for (i = 0, k = 0; i < ms->nummuscles; i++)
               {
                  for (j = 0; j < ms->muscle[i]->numWrapStructs; j++)
                  {
                     ms->save.muscwrap_associations[k].muscle  = i;
                     ms->save.muscwrap_associations[k].musc_wrap_index = j;
                     ms->save.muscwrap_associations[k].start_pt = ms->muscle[i]->wrapStruct[j]->startPoint;
                     ms->save.muscwrap_associations[k].end_pt = ms->muscle[i]->wrapStruct[j]->endPoint;
                     ms->save.muscwrap_associations[k].wrap_algorithm = ms->muscle[i]->wrapStruct[j]->wrap_algorithm;
                     ms->save.muscwrap_associations[k].wrap_object = ms->muscle[i]->wrapStruct[j]->wrap_object;
                     k++;
                  }
               }
            }
         }
      }
   }
}


void we_track_cb(void* data, SimmEvent se)
{
   int i, count, we_vals[WE_MAX_DEVS];
   int model_windex;

   WrapEditorTracker* tracker = (WrapEditorTracker*) data;
   Scene* scene = tracker->scene;
   ModelStruct* ms = tracker->model;
   ModelDisplayStruct* dis = &ms->dis;
   WEModelOptions* weop = &we->weop[ms->modelnum];
   dpWrapObject* wo;

   SBoolean redraw = no;
   double   z_dist, tmp_matrix[4][4], inv_view_matrix[4][4];
   double   wpt[4], wpt2[4], owpt[4], owpt2[4];
   int      bpan_mx_new, bpan_my_new;
   double   bpan_wx_new, bpan_wy_new, bpan_wz_new;
   int      mx_new, my_new;
   double   wx_new, wy_new, wz_new;
   double   angle, origin[4], new_origin[4], naxis[4], axis[4], x_percent, cursor_movement;

   getDev(we->numdevs, we->devs, we_vals);
   
   for (count = 0, i = 0; i < we->numdevs; i++)
   {
      if (we_vals[i] == 1)
         count++;
   }  
   if (count == 0)
   {
      REMOVE_TRACKER();
      return;
   }
   
   wo = weop->wrap_object;
   
   if (wo->visible == no)
      return;
#if 1
   if (wo->wrap_type == dpWrapCylinder && se.field1 == space_key && se.field2 == key_released)
   {
      wo->wrap_algorithm = ! wo->wrap_algorithm;
      inval_model_wrapping(we->model, weop->wrap_object);
      queue_model_redraw(ms);
      return;
   }
#endif
   
   if (we_vals[WE_MOVE_WRAP_OBJECT_KEY])
   {
      mx_new = se.mouse_x;
      my_new = se.mouse_y;
      
      if (weop->trackball_rotation)
      {
         if (we_vals[WE_TRACKBALL_KEY])
         {
            z_dist = scene->tz;
            
            find_world_coords(scene, mx_new, my_new,
               z_dist, &wx_new, &wy_new, &wz_new);
            
            if (tracker->mx_old == -1 || tracker->my_old == -1)
            {
               tracker->mx_old = mx_new;
               tracker->my_old = my_new;
               tracker->wx_old = wx_new;
               tracker->wy_old = wy_new;
               tracker->wz_old = wz_new;
            }
            else if (tracker->mx_old != mx_new || tracker->my_old != my_new)
            {
               origin[0] = origin[1] = origin[2] = 0.0;
               origin[3] = 1.0;
               
               axis[0] = tracker->wy_old - wy_new;
               axis[1] = wx_new - tracker->wx_old;
               axis[2] = 0.0;
               
               normalize_vector(axis, naxis);
               naxis[3] = 1.0;
               
               invert_4x4transform(scene->transform_matrix, tmp_matrix);
               mult_4x4matrix_by_vector(tmp_matrix, naxis, axis);
               mult_4x4matrix_by_vector(tmp_matrix, origin, new_origin);
               
               axis[0] -= new_origin[0];
               axis[1] -= new_origin[1];
               axis[2] -= new_origin[2];
               
               normalize_vector(axis, naxis);
               
               naxis[0] -= origin[0];
               naxis[1] -= origin[1];
               naxis[2] -= origin[2];
               
               normalize_vector(naxis, naxis);
               convert_vector(ms, naxis, ms->ground_segment, wo->segment);
               normalize_vector(naxis, naxis);
               
               /* if cursor moves a full screen width, rotate by 90 degrees */
               cursor_movement = sqrt((mx_new-tracker->mx_old)*(mx_new-tracker->mx_old) +
                  (my_new-tracker->my_old)*(my_new-tracker->my_old));
               
               x_percent = cursor_movement / (double)(scene->viewport[2]);
               
               angle = x_percent * 90.0;
               
               if (angle >= 0.0 && angle <= 180.0)
               {
                  DMatrix m;
                  
                  identity_matrix(m);
                  rotate_matrix_axis_angle(m, wo->rotation_axis.xyz,
                     wo->rotation_angle);
                  rotate_matrix_axis_angle(m, naxis, angle * DTOR);
                  
                  extract_rotation(m, &wo->rotation_axis, &wo->rotation_angle);
                  
                  redraw = yes;
                  
                  tracker->mx_old = mx_new;
                  tracker->my_old = my_new;
                  tracker->wx_old = wx_new;
                  tracker->wy_old = wy_new;
                  tracker->wz_old = wz_new;
               }
            }
         }
         else
         {
            tracker->wx_old = tracker->wy_old = tracker->wz_old = -1.0;
            tracker->mx_old = tracker->my_old = -1;
         }
         
         if (we_vals[WE_PAN_WRAP_OBJECT_KEY])
         {
            wpt[0] = wpt[1] = wpt[2] = 0.0;
            convert_from_wrap_object_frame(wo, wpt);
            wpt[3] = 1.0;
            
            convert(ms, wpt, wo->segment, ms->ground_segment);
            mult_4x4matrix_by_vector(scene->transform_matrix, wpt, wpt2);
            
            z_dist = wpt2[2];
            bpan_mx_new = se.mouse_x;
            bpan_my_new = se.mouse_y;
            
            find_world_coords(scene, bpan_mx_new, bpan_my_new,
               z_dist, &bpan_wx_new, &bpan_wy_new, &bpan_wz_new);
            
            if (tracker->bpan_mx_old == -1 || tracker->bpan_my_old == -1)
            {
               tracker->bpan_mx_old = bpan_mx_new;
               tracker->bpan_my_old = bpan_my_new;
               tracker->bpan_wx_old = bpan_wx_new;
               tracker->bpan_wy_old = bpan_wy_new;
               tracker->bpan_wz_old = bpan_wz_new;
            }
            else if (tracker->bpan_mx_old != bpan_mx_new ||
               tracker->bpan_my_old != bpan_my_new)
            {
               wpt[0] = bpan_wx_new;
               wpt[1] = bpan_wy_new;
               wpt[2] = bpan_wz_new;
               wpt[3] = 1.0;
               
               invert_4x4transform(scene->transform_matrix, inv_view_matrix);
               
#if !STEADYCAM
               if (dis->camera_segment >= 0)
                  copy_4x4matrix(*get_ground_conversion(ms->modelnum,
                  dis->camera_segment, to_ground), tmp_matrix);
               else
                  reset_4x4matrix(tmp_matrix);
               append_4x4matrix(inv_view_matrix,tmp_matrix);
#endif
               mult_4x4matrix_by_vector(inv_view_matrix,wpt,wpt2);
               convert(ms, wpt2, ms->ground_segment, wo->segment);
               
               owpt[0] = tracker->bpan_wx_old;
               owpt[1] = tracker->bpan_wy_old;
               owpt[2] = tracker->bpan_wz_old;
               owpt[3] = 1.0;
               
               mult_4x4matrix_by_vector(inv_view_matrix,owpt,owpt2);
               convert(ms, owpt2, ms->ground_segment, wo->segment);
               
               wo->translation.xyz[0] += (wpt2[XX] - owpt2[XX]);
               wo->translation.xyz[1] += (wpt2[YY] - owpt2[YY]);
               wo->translation.xyz[2] += (wpt2[ZZ] - owpt2[ZZ]);
               
               wo->undeformed_translation.xyz[0] += (wpt2[XX] - owpt2[XX]);
               wo->undeformed_translation.xyz[1] += (wpt2[YY] - owpt2[YY]);
               wo->undeformed_translation.xyz[2] += (wpt2[ZZ] - owpt2[ZZ]);
               
               tracker->bpan_mx_old = bpan_mx_new;
               tracker->bpan_my_old = bpan_my_new;
               tracker->bpan_wx_old = bpan_wx_new;
               tracker->bpan_wy_old = bpan_wy_new;
               tracker->bpan_wz_old = bpan_wz_new;
               redraw = yes;
            }
         }
         else
         {
            tracker->bpan_wx_old = tracker->bpan_wy_old = tracker->bpan_wz_old = -1.0;
            tracker->bpan_mx_old = tracker->bpan_my_old = -1;
         }
         
         if (we_vals[WE_ZOOM_WRAP_OBJECT_KEY])
         {
            if (tracker->zoom_mx_old == -1 || tracker->zoom_my_old == -1)
            {
               tracker->zoom_mx_old = mx_new;
               tracker->zoom_my_old = my_new;
               
               wpt[0] = wpt[1] = 0.0;
               wpt[2] = wpt[3] = 1.0;
               
               invert_4x4transform(scene->transform_matrix, tmp_matrix);
               
               mult_4x4matrix_by_vector(tmp_matrix, wpt, wpt2);
               convert(ms, wpt2, ms->ground_segment, wo->segment);
               
               tracker->zoom_vec.xyz[0] = wpt2[0];
               tracker->zoom_vec.xyz[1] = wpt2[1];
               tracker->zoom_vec.xyz[2] = wpt2[2];
               
               wpt[0] = wpt[1] = wpt[2] = 0.0;
               wpt[3] = 1.0;
               
               mult_4x4matrix_by_vector(tmp_matrix, wpt, wpt2);
               convert(ms, wpt2, ms->ground_segment, wo->segment);
               
               tracker->zoom_vec.xyz[0] -= wpt2[0];
               tracker->zoom_vec.xyz[1] -= wpt2[1];
               tracker->zoom_vec.xyz[2] -= wpt2[2];
            }
            else if (tracker->zoom_mx_old != mx_new || tracker->zoom_my_old != my_new)
            {
               double netmove = (mx_new - tracker->zoom_mx_old) * 0.002;
               
               wo->translation.xyz[0] += netmove * tracker->zoom_vec.xyz[0];
               wo->translation.xyz[1] += netmove * tracker->zoom_vec.xyz[1];
               wo->translation.xyz[2] += netmove * tracker->zoom_vec.xyz[2];
               
               wo->undeformed_translation.xyz[0] += netmove * tracker->zoom_vec.xyz[0];
               wo->undeformed_translation.xyz[1] += netmove * tracker->zoom_vec.xyz[1];
               wo->undeformed_translation.xyz[2] += netmove * tracker->zoom_vec.xyz[2];
               
               redraw = yes;
               
               tracker->zoom_mx_old = mx_new;
               tracker->zoom_my_old = my_new;
            }
         }
         else
            tracker->zoom_mx_old = tracker->zoom_my_old = -1;
       }
       else /* trackball turned off, do x,y,z rotation */
       {
          double  new_rot_angle;
          DMatrix m;
          
          identity_matrix(m);
          rotate_matrix_axis_angle(m, wo->rotation_axis.xyz, wo->rotation_angle);
          
          if (we_vals[WE_ROTATE_X_KEY])
          {
             if (CURSOR_IN_VIEWPORT(mx_new, my_new, scene->viewport))
             {
                new_rot_angle = DISTANCE_FROM_MIDPOINT(mx_new, scene->viewport) * 0.1;
                
                if (weop->xform_frame == WE_LOCAL_FRAME)
                   x_rotate_matrix_bodyfixed(m, new_rot_angle * DTOR);
                else
                   x_rotate_matrix_spacefixed(m, new_rot_angle * DTOR);
                
                redraw = yes;
             }
          }
          if (we_vals[WE_ROTATE_Y_KEY])
          {
             if (CURSOR_IN_VIEWPORT(mx_new, my_new, scene->viewport))
             {
                new_rot_angle = DISTANCE_FROM_MIDPOINT(mx_new, scene->viewport) * 0.1;
                
                if (weop->xform_frame == WE_LOCAL_FRAME)
                   y_rotate_matrix_bodyfixed(m, new_rot_angle * DTOR);
                else
                   y_rotate_matrix_spacefixed(m, new_rot_angle * DTOR);
                
                redraw = yes;
             }
          }
          if (we_vals[WE_ROTATE_Z_KEY])
          {
             if (CURSOR_IN_VIEWPORT(mx_new, my_new, scene->viewport))
             {
                new_rot_angle = DISTANCE_FROM_MIDPOINT(mx_new, scene->viewport) * 0.1;
                
                if (weop->xform_frame == WE_LOCAL_FRAME)
                   z_rotate_matrix_bodyfixed(m, new_rot_angle * DTOR);
                else
                   z_rotate_matrix_spacefixed(m, new_rot_angle * DTOR);
                
                redraw = yes;
             }
          }
          if (redraw)
             extract_rotation(m, &wo->rotation_axis, &wo->rotation_angle);
       }
   }
   
   if (redraw == yes)
   {
      wo->xforms_valid = no;
      recalc_xforms(wo);
      inval_model_wrapping(ms, weop->wrap_object);
      queue_model_redraw(ms);
      if (ms == we->model)
         display_wrapeditor(we_win_params, we_win_union);
   }
}


static SBoolean sEnableDebugShapes = no;

/* -------------------------------------------------------------------------
   enable_debug_shapes
---------------------------------------------------------------------------- */
void enable_debug_shapes (SBoolean state)
{
   sEnableDebugShapes = state;
}

#if VISUAL_WRAPPING_DEBUG

/* -------------------------------------------------------------------------
   add_debug_point - add a point for debugging display.
---------------------------------------------------------------------------- */
void add_debug_point (
    dpWrapObject* wo,
    double factor,
    double pt[],
    float radius,
    const char* name,
    const float* color)
{
   int i = wo->num_debug_glyphs;

   if ( ! sEnableDebugShapes)
       return;

   if (i >= MAX_DEBUG_GLYPHS)
      return;
   
   wo->glyph[i].isLine = no;
   wo->glyph[i].p1[0] = pt[0] / factor;
   wo->glyph[i].p1[1] = pt[1] / factor;
   wo->glyph[i].p1[2] = pt[2] / factor;
   wo->glyph[i].radius = radius / factor;
   wo->glyph[i].name1 = name;
   wo->glyph[i].name2 = NULL;
   wo->glyph[i].color = color;
   
   wo->num_debug_glyphs++;
}

/* -------------------------------------------------------------------------
   add_debug_line - add a line segment for debugging display.
---------------------------------------------------------------------------- */
void add_debug_line (
    dpWrapObject* wo,
    double factor,
    double pt1[],
    double pt2[],
    float lineWidth,
    const char* name1,
    const char* name2,
    const float* color)
{
   int i = wo->num_debug_glyphs;

   if ( ! sEnableDebugShapes)
       return;
   
   if (i >= MAX_DEBUG_GLYPHS)
      return;
   
   wo->glyph[i].isLine = yes;
   wo->glyph[i].p1[0] = pt1[0] / factor;
   wo->glyph[i].p1[1] = pt1[1] / factor;
   wo->glyph[i].p1[2] = pt1[2] / factor;
   wo->glyph[i].p2[0] = pt2[0] / factor;
   wo->glyph[i].p2[1] = pt2[1] / factor;
   wo->glyph[i].p2[2] = pt2[2] / factor;
   wo->glyph[i].radius = lineWidth;
   wo->glyph[i].name1 = name1;
   wo->glyph[i].name2 = name2;
   wo->glyph[i].color = color;
   
   wo->num_debug_glyphs++;
}

/* -------------------------------------------------------------------------
   lerp_clr - interpolate between two rgb colors.
---------------------------------------------------------------------------- */
float* lerp_clr (const float start[3], const float end[3], double t, float color[3])
{
    if (t < 0.0)
    {
       color[0] = start[0];
       color[1] = start[1];
       color[2] = start[2];
    }
    else if (t > 1.0)
    {
       color[0] = end[0];
       color[1] = end[1];
       color[2] = end[2];
    }
    else
    {
       color[0] = start[0] + t * (end[0] - start[0]);
       color[1] = start[1] + t * (end[1] - start[1]);
       color[2] = start[2] + t * (end[2] - start[2]);
    }
    return color;
}

#endif /* VISUAL_WRAPPING_DEBUG */

#endif /* ! ENGINE */

void recalc_xforms(dpWrapObject* wo)
{
   if ( ! wo->xforms_valid)
   {
      identity_matrix(wo->from_local_xform);

      if (wo->rotation_angle != 0.0)
         rotate_matrix_axis_angle(wo->from_local_xform, wo->rotation_axis.xyz, wo->rotation_angle);
   
      translate_matrix(wo->from_local_xform, wo->translation.xyz);
   
      invert_4x4transform(wo->from_local_xform, wo->to_local_xform);
   
      wo->xforms_valid = yes;
   }
}

const char* get_wrap_type_name (int i)
{
   switch (i) {
      case dpWrapSphere:
         return "sphere";
      case dpWrapCylinder:
         return "cylinder";
      case dpWrapEllipsoid:
         return "ellipsoid";
      case dpWrapTorus:
         return "torus";
   }

   return "none";
}

const char* get_wrap_algorithm_name (int i)
{
   static char* names[WE_NUM_WRAP_ALGORITHMS] = { "hybrid", "midpoint", "axial" };
   
   if (i >= 0 && i < WE_NUM_WRAP_ALGORITHMS)
      return names[i];
   
   return NULL;
}

SBoolean query_muscle_wrap_association (dpMuscleStruct* muscle, dpWrapObject* wrap_object)
{
   /* See if the given wrap object is used in the given muscle.
    * You don't care which slot it's in, just that it's there.
    */
   if (muscle && wrap_object)
   {
      int i;

      for (i = 0; i < muscle->numWrapStructs; i++)
         if (muscle->wrapStruct[i]->wrap_object == wrap_object)
            return yes;
   }

   return no;
}
