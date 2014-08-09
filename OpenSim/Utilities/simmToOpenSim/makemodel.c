/*******************************************************************************

   MAKEMODEL.C

   Author: Peter Loan

   Date: 26-APR-89

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.

   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Description: This file contains routines that read in bone
      descriptions and make bone objects.

   Routines:
      add_model              : main routine to make a model and its bones
      make_gencoord_sliders  : makes the gencoord slider bars
      get_modelstruct        : mallocs a model structure for new model
      add_model_window       : creates a model window
      post_process_bones     : sets some bone variables after files are read
      make_modelpopups       : makes the pop-up menus for a model
      scene_input            : input handler for model window
      model_deletion_confirm : confirms the deletion of a model
      check_definitions      : make sure all joint file elements were defined
      size_model             : determines good size for the model window

*******************************************************************************/

#include <ctype.h>
#include <direct.h>
#include <fcntl.h>
#include <errno.h>

#include "universal.h"

#include "globals.h"
#include "functions.h"
#include "normio.h"
#if ! OPENSIM_BUILD
#include "modelviewer.h"
#include "password.h"
#endif


/*************** DEFINES (for this file only) *********************************/
#define WINDOW_WIDTH 420
#define WINDOW_MARGIN 160

/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/
static char* model_deletion_text = "Are you sure that you want to delete this model?";
static Scene* scene_to_confirm;
static char* rot_label[] = {"X","Y","Z"};

/*************** EXTERNED VARIABLES (declared in another file) ****************/
extern char badLoopErrorMsg[];
extern char gencoordResidualErrorMsg[];
extern char badConstraintErrorMsg[];
extern char badGencoordErrorMsg2[];

/*************** GLOBAL VARIABLES (used in only a few files) *****************/
char archive_password[] = "P.i3h78sw,l1";

/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static SBoolean joint_uses_segment(ModelStruct* ms, int jointnum, int segmentnum,
                                   int* other_segment);
static ReturnCode make_gencoord_sliders(ModelStruct* ms);
static ReturnCode add_model_window(ModelStruct* ms, int suggested_width,
                                   int suggested_height);
static void make_modelpopups(ModelStruct* ms);
#if ! ENGINE
static void init_tracked_file_options(ModelStruct* model, glutTRCOptions* options, const char inputFileName[]);
static void handle_right_click(Scene* scene, int mouse_x, int mouse_y);
static void call_simm_event_handler(const char tool_id[], SimmEvent se);
#endif
static double initializeEmgWindowSize();
static int initializeEmgSmoothingPasses();
static ReturnCode add_model_to_scene(Scene* scene, ModelStruct* model);


Scene* get_scene(void)
{
   int i;
   Scene* sc;

   for (i=0; i<SCENEBUFFER; i++)
      if (gScene[i] == NULL)
         break;

   if (i == SCENEBUFFER)
      return NULL;

   sc = (Scene*)simm_calloc(1, sizeof(Scene));
   gScene[i] = sc;

   if (sc == NULL)
      return NULL;

   sc->scene_num = i;

   return sc;
}


ModelStruct* get_modelstruct(void)
{
   int i;
   ModelStruct* ms;

   for (i=0; i<MODELBUFFER; i++)
      if (gModel[i] == NULL)
         break;

   if (i == MODELBUFFER)
      return NULL;

   ms = (ModelStruct*)simm_calloc(1, sizeof(ModelStruct));
   gModel[i] = ms;

   if (ms == NULL)
      return NULL;

   ms->modelnum = i;

   return ms;
}


ModelStruct* get_model_by_name(const char name[])
{
   int i;

   for (i=0; i<MODELBUFFER; i++)
      if (gModel[i] && STRINGS_ARE_EQUAL(gModel[i]->name, name))
         return gModel[i];

   return NULL;
}

#if ! ENGINE
#if ! OPENSMAC
#if ! CORTEX_PLUGIN

/* ADD_MODEL: this routine controls the making of a model. It gets a model-file
 * name and a muscle-file name from the user and reads-in the model definition.
 * After some processing and arranging of the definition, it sets up a model
 * window to display it.
 */
ReturnCode add_model(char jointfilename[], char musclefilename[],
                     int suggested_win_width, int* modelIndex, SBoolean showTopLevelMessages)
{
   int i, j, window_width, window_height;
   ModelStruct* ms;
   Scene* scene;
   MotionSequence* motion = NULL;
   SBoolean muscle_file_exists = no;
   char fullpath[CHARBUFFER];
   ReturnCode rc;

   /* For now, make a new scene for each model. */
   scene = get_scene();
   ms = get_modelstruct();

   if (scene == NULL || ms == NULL)
   {
      if (showTopLevelMessages == yes)
         error(none,"Unable to make new model (perhaps too many models already)");
      return code_bad;
   }

   init_scene(scene);

   if (init_model(ms) == code_bad)
   {
      if (showTopLevelMessages == yes)
         error(none,"Unable to add another model.");
      return code_bad;
   }

   strcpy(fullpath, jointfilename);

   if (read_model_file(scene, ms, fullpath, showTopLevelMessages) == code_bad)
   {
      free_model(ms->modelnum);
      if (showTopLevelMessages == yes)
         error(none,"Unable to load model.");
      return code_bad;
   }

   if (check_definitions(ms) == code_bad)
   {
      free_model(ms->modelnum);
      return code_bad;
   }

   find_ground_joint(ms); /* must be called before makepaths() ! */

   /* determine whether the model has any closed loops and if so, set one
    * of the joints as the loop joint. */
   markLoopJoints(ms);
   if (ms->numclosedloops == 0)
      ms->useIK = no;

   if (makepaths(ms) == code_bad)
   {
      free_model(ms->modelnum);
      return code_bad;
   }

   /* create structure to hold loop information and store all info 
    * If the loop joint was changed (user had entered a different one),
    * recalculate the paths and remake the loops */
   if (makeLoops(ms) == yes)
   {
      if (makepaths(ms) == code_bad)
      {
         free_model(ms->modelnum);
         return code_bad;
      }      
      makeLoops(ms);
   }
   updateLoopInfo(ms);

   /* determine which gencoords are affected by constraints in the model
    * and create the structure to hold constraint information. */
   markAffectedGencoords(ms);
   updateConstraintInfo(ms);

   if (find_segment_drawing_order(ms) == code_bad)
   {
      free_model(ms->modelnum);
      return code_bad;
   }

   if ( ! is_in_demo_mode())
   {
      if (showTopLevelMessages == yes)
      {
         (void)sprintf(buffer,"Read joint file %s", fullpath);
         message(buffer, 0, DEFAULT_MESSAGE_X_OFFSET);
      }
   }
   
   /* try to load muscle file, first check for a muscle file specified
    * within the joint file.
    */
   if (ms->musclefilename &&
       STRINGS_ARE_NOT_EQUAL(ms->musclefilename, "NULL") &&
       STRINGS_ARE_NOT_EQUAL(ms->musclefilename, "null") &&
       STRINGS_ARE_NOT_EQUAL(ms->musclefilename, "no_muscles_for_this_model"))
   {
      if (musclefilename && strlen(musclefilename) > 0)
      {
         char* mname = strrchr(musclefilename, DIR_SEP_CHAR);

         if (mname)
            mname++;
         else
            mname = musclefilename;

         if (showTopLevelMessages == yes)
         {
            sprintf(buffer, "Overriding muscle file specified in joint file (%s)", ms->musclefilename);
            error(none, buffer);
            sprintf(buffer, "with muscle file chosen in file browser (%s).", mname);
            error(none, buffer);
         }

         FREE_IFNOTNULL(ms->musclefilename);
         mstrcpy(&ms->musclefilename, musclefilename);
         strcpy(fullpath, musclefilename);
      }
      else
      {
         char* purePath = NULL;
         get_pure_path_from_path(ms->jointfilename, &purePath);
         build_full_path(purePath, ms->musclefilename, fullpath);
         FREE_IFNOTNULL(purePath);
      }
   }
   else if (musclefilename != NULL) /* try user-selected musclefile */
   {
      strcpy(fullpath, musclefilename);
   }
   else
   {
      fullpath[0] = '\0';
   }
   
   /* if a muscle file was found, try to load it:
    */
   if (fullpath[0] && read_muscle_file(ms, fullpath, &muscle_file_exists, showTopLevelMessages) == code_bad)
   {
      free_model(ms->modelnum);
      if (showTopLevelMessages == yes)
         error(none,"Unable to load muscle definitions");
      return code_bad;
   }

   /* If you made it to here, then the joint and muscle files were both valid,
    * and so another model will be added to the system.
    */
   root.nummodels++;
   root.modelcount++;

   /* If a model name was not specified in the joints file, make a default one.
    * If a name was specified but it's the same as the name of an existing model,
    * append a suffix so it's a little different.
    */
   if (ms->name == NULL)
   {
      (void)sprintf(buffer,"model %d", root.modelcount);
      ms->name = (char*)simm_malloc(250*sizeof(char));
      if (ms->name == NULL)
      {
         free_model(ms->modelnum);
         return code_bad;
      }
      (void)strcpy(ms->name,buffer);
   }
   else
   {
      for (i=0; i<MODELBUFFER; i++)
      {
         if (gModel[i] == NULL || i == ms->modelnum)
            continue;
         if (STRINGS_ARE_EQUAL(ms->name, gModel[i]->name))
         {
            (void)sprintf(buffer," (%d)", ms->modelnum + 1);
            (void)strcat(ms->name, buffer);
            break;
         }
      }
   }

   if (muscle_file_exists == yes && ms->musclefilename == NULL)
      mstrcpy(&ms->musclefilename, musclefilename);

   if (makegencform(ms) == code_bad)
   {
      free_model(ms->modelnum);
      return code_bad;
   }

   if (init_gencoords(ms) == code_bad)
   {
      free_model(ms->modelnum);
      return code_bad;
   }

   set_gencoord_info(ms);

   if (init_model_display(ms) == code_bad)
   {
      free_model(ms->modelnum);
      return code_bad;
   }

   if (make_gencoord_sliders(ms) == code_bad)
   {
      free_model(ms->modelnum);
      return code_bad;
   }

   if (make_dynparams_form(ms) == code_bad)
   {
      free_model(ms->modelnum);
      return code_bad;
   }

   post_process_bones(ms);

   /* solve any loops or constraints in the system */
   solve_initial_loops_and_constraints(ms);
   
   if (make_muscle_menus(ms) == code_bad)
   {
      free_model(ms->modelnum);
      return code_bad;
   }

   make_modelpopups(ms);

   /* Now that you know how many of each thing there are (segments, muscle groups, etc.),
    * realloc the arrays to free unused space. Don't realloc the muscle lists, function lists,
    * muscle groups, world objects, and gencoords since the user can add more of them interactively.
    * rc should always be code_fine since you're reallocing to a smaller size.
    */
   if (ms->numsegments > 0)
   {
      ms->segment = (SegmentStruct*)simm_realloc(ms->segment, ms->numsegments*sizeof(SegmentStruct),&rc);
   }
   else
   {
      FREE_IFNOTNULL(ms->segment);
   }
   if (ms->numjoints > 0)
   {
      ms->joint = (JointStruct*)simm_realloc(ms->joint, ms->numjoints*sizeof(JointStruct),&rc);
   }
   else
   {
      FREE_IFNOTNULL(ms->joint);
   }

   ms->wrap_object_array_size = ms->num_wrap_objects;
   if (ms->wrap_object_array_size == 0)
       ms->wrap_object_array_size = 1;
   ms->wrapobj = (dpWrapObject**)simm_realloc(ms->wrapobj,
                       ms->wrap_object_array_size*sizeof(dpWrapObject*),&rc);

   ms->constraint_object_array_size = ms->num_constraint_objects;
   if (ms->constraint_object_array_size == 0)
       ms->constraint_object_array_size = CONSTRAINT_OBJECT_ARRAY_INCREMENT;
   ms->constraintobj = (ConstraintObject*)simm_realloc(ms->constraintobj,
                       ms->constraint_object_array_size*sizeof(ConstraintObject),&rc);

   /* make a model window structure and register it with the window manager */
   if (scene->windowWidth > 0)
      window_width = scene->windowWidth;
   else
      window_width = suggested_win_width;

   window_height = scene->windowHeight;

   size_model_display(scene, ms, &window_width, &window_height);

   /* If the user did not define all possible model views,
    * then fill-in the unused ones with a standard transform.
    */
   for (i = ms->dis.num_file_views; i < MAXSAVEDVIEWS; i++)
   {
      reset_4x4matrix(ms->dis.saved_view[i]);
      ms->dis.saved_view[i][3][0] = scene->tx;
      ms->dis.saved_view[i][3][1] = scene->ty;
      ms->dis.saved_view[i][3][2] = scene->tz;
   }

   /* Now apply the default view to the model */
   if (ms->dis.default_view >= 0 && ms->dis.default_view < MAXSAVEDVIEWS)
   {
      copy_4x4matrix(ms->dis.saved_view[ms->dis.default_view], scene->transform_matrix);
      scene->tx = ms->dis.saved_view[ms->dis.default_view][3][0];
      scene->ty = ms->dis.saved_view[ms->dis.default_view][3][1];
      scene->tz = ms->dis.saved_view[ms->dis.default_view][3][2];
   }

   if (make_scene_window(scene, window_width, window_height) == code_bad)
   {
      //TODO_SCENE free_scene(scene);
      return code_bad;
   }

   if (add_model_to_scene(scene, ms) == code_bad)
   {
      free_model(ms->modelnum);
      return code_bad;
   }

#if ! ENGINE
   load_model_textures(ms);
#endif

   make_gencoord_help_text(ms);

   for (i = 0; i < ms->numgencoords; i++)
   {
      if (ms->gencoord[i]->slider_visible == yes)
      {
         ms->gencslider.sl[i].visible = yes;
         ms->gencform.option[i].active = yes;
         ms->gencform.option[i].visible = yes;
         ms->gc_chpanel.checkbox[i].active = yes;
         ms->gc_chpanel.checkbox[i].visible = yes;
         ms->gc_lockPanel.checkbox[i].active = yes;
         ms->gc_lockPanel.checkbox[i].visible = yes;
      }
      else
      {
         ms->gencslider.sl[i].visible = no;
         ms->gencform.option[i].active = no;
         ms->gencform.option[i].visible = no;
         ms->gc_chpanel.checkbox[i].active = no;
         ms->gc_chpanel.checkbox[i].visible = no;
         ms->gc_lockPanel.checkbox[i].active = no;
         ms->gc_lockPanel.checkbox[i].visible = no;
      }
   }

#if 0
   /* If there are more than 30 gencoords, then hide them all in
    * the Model Viewer. If there are less than 30, turn on all
    * 'used' gencoords, and turn off all 'unused' ones.
    */
   if (ms->numgencoords > 30)
   {
      for (i = 0; i < ms->numgencoords; i++)
      {
         ms->gencslider.sl[i].visible = no;
         ms->gencform.option[i].active = no;
         ms->gc_chpanel.checkbox[i].active = no;
         ms->gc_lockPanel.checkbox[i].active = no;
         ms->gencform.option[i].visible = no;
         ms->gc_chpanel.checkbox[i].visible = no;
         ms->gc_lockPanel.checkbox[i].visible = no;
      }
   }
   else
   {
      for (i = 0; i < ms->numgencoords; i++)
      {
         if (ms->gencoord[i]->used_in_model == yes)
         {
            ms->gencslider.sl[i].visible = yes;
            ms->gencform.option[i].active = yes;
            ms->gc_chpanel.checkbox[i].active = yes;
            ms->gc_lockPanel.checkbox[i].active = yes;
            ms->gencform.option[i].visible = yes;
            ms->gc_chpanel.checkbox[i].visible = yes;
            ms->gc_lockPanel.checkbox[i].visible = yes;
         }
         else
         {
            ms->gencslider.sl[i].visible = no;
            ms->gencform.option[i].active = no;
            ms->gc_chpanel.checkbox[i].active = no;
            ms->gc_lockPanel.checkbox[i].active = no;
            ms->gencform.option[i].visible = no;
            ms->gc_chpanel.checkbox[i].visible = no;
            ms->gc_lockPanel.checkbox[i].visible = no;
         }
      }
   }
#endif

   reposition_gencoord_sliders(ms);

   /* Make display lists for all the materials. This must be done
    * while the current window is set to the model window (some
    * sort of weird OpenGL thing, at least on the SGI).
    */
   if (scene->window_index != -1)
   {
      glutSetWindow(root.window[scene->window_index].win_parameters->id);
      for (i=0; i<ms->dis.mat.num_materials; i++)
      {
         make_mat_display_list(&ms->dis.mat.materials[i]);
         make_highlight_mat_display_list(&ms->dis.mat.materials[i]);
      }
   }

   /* If the user specified motion files in the joints file,
    * read them in now.
    */
   for (i = 0; i < ms->num_motion_files; i++)
      motion = load_motion(ms->motionfilename[i], ms->modelnum, showTopLevelMessages);

   if (motion && ms->num_motion_files == 1)
      apply_motion_to_model(ms, ms->motion[0], ms->motion[0]->min_value, yes, yes);

   if (modelIndex)
     *modelIndex = ms->modelnum;
   
   return code_fine;
}
#endif /* CORTEX_PLUGIN */


void resize_model_display(Scene* scene, ModelStruct* ms)
{
   int i;

   size_model_display(scene, ms, &scene->windowWidth, &scene->windowHeight);

   // Copy the newly calculated standard transform to all of the
   // model views that were not defined by the user.
   for (i = ms->dis.num_file_views; i < MAXSAVEDVIEWS; i++)
   {
      reset_4x4matrix(ms->dis.saved_view[i]);
      ms->dis.saved_view[i][3][0] = scene->tx;
      ms->dis.saved_view[i][3][1] = scene->ty;
      ms->dis.saved_view[i][3][2] = scene->tz;
   }

   // Now apply the default view to the model.
   if (ms->dis.default_view >= 0 && ms->dis.default_view < MAXSAVEDVIEWS)
      copy_4x4matrix(ms->dis.saved_view[ms->dis.default_view], scene->transform_matrix);
}

static ReturnCode make_gencoord_sliders(ModelStruct* ms)
{
   int i;
   double stepsize;
   IntBox bbox;
   SliderArray* sa;

   sa = &ms->gencslider;

   sa->numsliders = ms->numgencoords;

   ms->gencslider.sl = (Slider*)simm_malloc(sa->numsliders * sizeof(Slider));
   if (ms->gencslider.sl == NULL)
      return code_bad;

   for (i=0; i<sa->numsliders; i++)
   {
      SET_BOX1221(bbox,0,170+2*FORM_FIELD_HEIGHT,-FORM_FIELD_YSPACING*i,
         bbox.y2-FORM_FIELD_HEIGHT);
      if (ms->gencoord[i]->type == translation_gencoord)
         stepsize = 0.001;
      else
         stepsize = 1.0;
      make_slider(&sa->sl[i],horizontal_slider,bbox,FORM_FIELD_HEIGHT-2,
         ms->gencoord[i]->value,
         ms->gencoord[i]->range.start,
         ms->gencoord[i]->range.end,stepsize,NULL,NULL);
   }

   return code_fine;
}


unsigned short curs2[] = {
0x3455, 0x6452, 0x6321, 0x3563,
};

static ReturnCode make_scene_window(Scene* scene, int suggested_width, int suggested_height)
{
   WindowParams mwin;
   WinUnion wun;
   IntBox prefpos;
   int i, offset = scene->scene_num % 6;

   mwin.minwidth = 30;
   mwin.minheight = 50;

   if (scene->windowX1 > 0 && scene->windowY1 > 0)
      glueSetWindowPlacement(GLUT_NORMAL_WINDOW, GLUE_NORTHWEST,
         suggested_width, suggested_height,
         scene->windowX1, scene->windowY1, &prefpos);
   else
      glueSetWindowPlacement(GLUT_NORMAL_WINDOW, GLUE_NORTHEAST,
         suggested_width, suggested_height,
         -offset * TITLE_BAR_HEIGHT, offset * TITLE_BAR_HEIGHT, &prefpos);

   scene->window_glut_id = mwin.id = glueOpenWindow("scene", no, GLUE_DEPTH_BUFFERED);

   mwin.name = (char*)simm_malloc(256*sizeof(char));
   if (mwin.name == NULL)
      return code_bad;

   (void)sprintf(mwin.name, "Scene %d", scene->scene_num);
   //(void)strcpy(mwin.name, ms->name);
   glutSetIconTitle(mwin.name);
   glutSetWindowTitle(mwin.name);

   init_model_lighting();

   wun.scene = scene;

   scene->window_index = add_window(&mwin, &wun, SCENE, scene->scene_num, no, drawscene, update_scene, scene_input);

   if (scene->window_index == -1)
   {
      glutDestroyWindow(mwin.id);
      return code_bad;
   }

   glutSetWindowData(mwin.id, scene->window_index);

   FREE_IFNOTNULL(mwin.name);

   return code_fine;
}


static ReturnCode add_model_to_scene(Scene* scene, ModelStruct* model)
{
   glutSetWindow(scene->window_glut_id);

   //TODO_SCENE: for now, window takes name of this model
   (void)strcpy(buffer, model->name);
   glutSetIconTitle(buffer);
   glutSetWindowTitle(buffer);
   FREE_IFNOTNULL(root.window[scene->window_index].win_parameters->name);
   mstrcpy(&root.window[scene->window_index].win_parameters->name, buffer);

   scene->num_models = 1;
   scene->model = (ModelStruct**)simm_calloc(1, sizeof(ModelStruct*));
   scene->model[0] = model;

   updatemodelmenu();

   //TODO5.0: should the scene have a title?
   //what should the movie file name be?
   sprintf(buffer, "%s.avi", model->name);
   mstrcpy(&scene->movie_file, buffer);

   make_and_queue_simm_event(MODEL_ADDED, model->modelnum, NULL, NULL, ZERO, ZERO);

   return code_fine;
}


static void make_modelpopups(ModelStruct* ms)
{
   int i, j, item_count;

   /* Make the joint menu */
   ms->jointmenu = glueCreateMenu("Joints");
   for (i=0; i<ms->numjoints; i++)
      glueAddMenuEntry(ms->jointmenu, ms->joint[i].name);

   make_gencoord_popup_menus(ms);
   /* Make the muscle group menu */
   ms->musclegroupmenu = glueCreateMenu("Muscle Groups");
   for (i=0; i<ms->numgroups; i++)
      glueAddMenuEntry(ms->musclegroupmenu,ms->muscgroup[i].name);

   glueAddMenuEntry(ms->musclegroupmenu, "---");
   glueAddMenuEntryWithValue(ms->musclegroupmenu, "all on",  ALL_MUSCLES_ON);
   glueAddMenuEntryWithValue(ms->musclegroupmenu, "all off", ALL_MUSCLES_OFF);

   /* If there are no muscles, disable all the menu items. They will be
    * enabled if and when a new muscle is created.
    */
   if (ms->nummuscles == 0)
   {
      int numItems = glueGetNumMenuItems(ms->musclegroupmenu);
      for (i = 0; i < numItems; i++)
         glueEnableMenuItem(ms->musclegroupmenu, i + 1, GLUE_DISABLE);
   }

   ms->markerMenu = -1; // gets created in ke_add_model()

   /* Make the segment menu */
   ms->segmentmenu = glueCreateMenu("Body Segments");
   for (i=0; i<ms->numsegments; i++)
       glueAddMenuEntry(ms->segmentmenu,ms->segment[i].name);

   /* Make the motion menus */
   ms->motionmenu = glueCreateMenu("Motions");
   glueAddMenuEntry(ms->motionmenu, "none loaded");
   glueEnableMenuItem(ms->motionmenu, 1, GLUE_DISABLE);
   ms->motionwithrealtimemenu = glueCreateMenu("Motions");
   ms->motionplotmenu = glueCreateMenu("Motions");
   glueAddMenuEntry(ms->motionplotmenu, "none loaded");
   glueEnableMenuItem(ms->motionplotmenu, 1, GLUE_DISABLE);

   /* Add the motion curve menu to the bottom of the x-var menu. */
   glueAddSubmenuEntry(ms->xvarmenu, "motion curves", ms->motionplotmenu);
   glueAddMenuEntryWithValue(ms->xvarmenu, "---", -1);

   if (is_module_present(MOTION_REAL))
   {
      const char* evaHost = get_preference("EVART_MACHINE");
      
      if (evaHost)
         sprintf(buffer, "realtime connection to %s", evaHost);
      else
         sprintf(buffer, "realtime connection");
   
      glueAddMenuEntryWithValue(ms->motionwithrealtimemenu, buffer, REALTIME_MENU_ITEM);
      glueAddMenuEntryWithValue(ms->motionwithrealtimemenu, "---", REALTIME_MENU_ITEM + 1);
   }

   ms->dis.view_menu = glueCreateMenu("Cameras");
   for (i=0; i<ms->dis.num_file_views; i++)
       glueAddMenuEntry(ms->dis.view_menu,ms->dis.view_name[i]);

   for (i=ms->dis.num_file_views; i<MAXSAVEDVIEWS; i++)
   {
      (void)sprintf(buffer,"camera %d", i+1);
      glueAddMenuEntry(ms->dis.view_menu,buffer);
   }

   for (i=0; i<ms->numsegments; i++)
   {
      ms->segment[i].drawmodemenu = glueCreateMenu("Draw Mode");
      glueAddMenuEntryWithValue(ms->segment[i].drawmodemenu,"gouraud shaded", 10*i);
      glueAddMenuEntryWithValue(ms->segment[i].drawmodemenu,"flat shaded", 10*i+1);
      glueAddMenuEntryWithValue(ms->segment[i].drawmodemenu,"solid fill", 10*i+2);
      glueAddMenuEntryWithValue(ms->segment[i].drawmodemenu,"wireframe", 10*i+3);
      glueAddMenuEntryWithValue(ms->segment[i].drawmodemenu,"outlined", 10*i+4);
      glueAddMenuEntryWithValue(ms->segment[i].drawmodemenu,"bounding box", 10*i+5);
      glueAddMenuEntryWithValue(ms->segment[i].drawmodemenu,"none", 10*i+6);
      glueAddMenuEntryWithValue(ms->segment[i].drawmodemenu,"---", 10*i+7);
      glueAddMenuEntryWithValue(ms->segment[i].drawmodemenu,"normal vectors", 10*i+8);
      glueAddMenuEntryWithValue(ms->segment[i].drawmodemenu,"---", 10*i+10);
      glueAddMenuEntryWithValue(ms->segment[i].drawmodemenu,"segment axes", 10*i+9);
   }

   ms->material_menu = glueCreateMenu("Materials");
   for (i=1; i<ms->dis.mat.num_materials; i++)
      glueAddMenuEntryWithValue(ms->material_menu, ms->dis.mat.materials[i].name, START_MATERIALS + i);

   update_drawmode_menus(ms);

   make_function_menu(ms);
}

void make_gencoord_popup_menus(ModelStruct* model)
{
   int i, j;

   // Make the gencoord menu.
   if (model->gencoordmenu != -1)
      glutDestroyMenu(model->gencoordmenu);
   model->gencoordmenu = glueCreateMenu("Gencoords");
   for (i=0; i<model->numgencoords; i++)
       glueAddMenuEntry(model->gencoordmenu, model->gencoord[i]->name);

   if (model->gencoordmenu2 != -1)
      glutDestroyMenu(model->gencoordmenu2);
   model->gencoordmenu2 = glueCreateMenu(NULL);
   for (i = 0; i < model->numgencoords; i++)
      glueAddMenuEntry(model->gencoordmenu2, model->gencoord[i]->name);

   // Make the gencoord groups menu.
   if (model->numgencgroups > 0)
   {
      if (model->gencoord_group_menu != -1)
         glutDestroyMenu(model->gencoord_group_menu);
      model->gencoord_group_menu = glueCreateMenu("Gencoord Groups");

      for (i = 0; i < model->numgencgroups; i++)
         glueAddMenuEntryWithValue(model->gencoord_group_menu, model->gencgroup[i].name, i + 2 * MAX_GENCOORDS);

      glueAddMenuEntry(model->gencoord_group_menu, "---");
      glueAddSubmenuEntry(model->gencoord_group_menu, "individual gencoords", model->gencoordmenu);
      glueAddMenuEntryWithValue(model->gencoord_group_menu, "all on",  4 * MAX_GENCOORDS);
      glueAddMenuEntryWithValue(model->gencoord_group_menu, "all off", 4 * MAX_GENCOORDS + 1);
   }
   else
   {
      model->gencoord_group_menu = 0;
      glueAddMenuEntry(model->gencoordmenu, "---");
      glueAddMenuEntryWithValue(model->gencoordmenu, "all on",  4 * MAX_GENCOORDS);
      glueAddMenuEntryWithValue(model->gencoordmenu, "all off", 4 * MAX_GENCOORDS + 1);
   }
   
   // Make the x-var menu (for plotting).
   if (model->xvarmenu != -1)
      glutDestroyMenu(model->xvarmenu);
   model->xvarmenu = glueCreateMenu("X variables");
   if (model->numgencoords > 0)
   {
      for (i=0; i<model->numgencoords; i++)
          glueAddMenuEntryWithValue(model->xvarmenu,model->gencoord[i]->name,i);

      glueAddMenuEntryWithValue(model->xvarmenu,"---",-1);
   }

   // Make the moment, moment arm, numerical moment arm, and moment@maxforce submenus.
    // Each of these menus is a list of gencoords for which you can find a moment arm
    // (y-var in the plotmaker). It is essentially the same menu as the model's
   // "gencoordmenu" pop-up menu (without the "all on" and "all off" items),
   // but the selections must return different numbers, so you need to make
   // a whole new menu for each y-var
   if (model->momentgencmenu != -1)
      glutDestroyMenu(model->momentgencmenu);
   model->momentgencmenu = glueCreateMenu("Gencoords");
   for (i=0; i<model->numgencoords; i++)
      glueAddMenuEntryWithValue(model->momentgencmenu, model->gencoord[i]->name, i);

   if (model->momentarmgencmenu != -1)
      glutDestroyMenu(model->momentarmgencmenu);
   model->momentarmgencmenu = glueCreateMenu("Gencoords");
   for (i=0; i<model->numgencoords; i++)
      glueAddMenuEntryWithValue(model->momentarmgencmenu, model->gencoord[i]->name, MAX_GENCOORDS + i);

   if (model->momentarmnumgencmenu != -1)
      glutDestroyMenu(model->momentarmnumgencmenu);
   model->momentarmnumgencmenu = glueCreateMenu("Gencoords");
   for (i=0; i<model->numgencoords; i++)
      glueAddMenuEntryWithValue(model->momentarmnumgencmenu, model->gencoord[i]->name, 2 * MAX_GENCOORDS + i);

   if (model->maxmomentgencmenu != -1)
      glutDestroyMenu(model->maxmomentgencmenu);
   model->maxmomentgencmenu = glueCreateMenu("Gencoords");
   for (i=0; i<model->numgencoords; i++)
      glueAddMenuEntryWithValue(model->maxmomentgencmenu, model->gencoord[i]->name, 3 * MAX_GENCOORDS + i);
}


void update_drawmode_menus(ModelStruct* ms)
{
   int i, item_count;

   /* TODO: this function was carved out of make_model_popups because when loading
    * HTR files with analog data, you may load a model and then add forceplates to
    * it (as worldobjects) after the model popups have already been created. In order
    * to get the new forceplates into the drawmode menus, you have to call this
    * function from within the analog processing code. Improvements that can be
    * made: (1) there should be a glueDestroyMenu() so that the menus being updated
    * are not left hanging around, (2) each world object's drawmode menu should be
    * created as the object is created, not in this function (this is done properly
    * for body segments).
    */
   for (i=0; i<ms->numworldobjects; i++)
   {
      if (ms->worldobj[i].drawmodemenu != 0)
         glutDestroyMenu(ms->worldobj[i].drawmodemenu);
      ms->worldobj[i].drawmodemenu = glueCreateMenu("Draw Mode");
      glueAddMenuEntryWithValue(ms->worldobj[i].drawmodemenu,"gouraud shaded", 10*(i+ms->numsegments));
      glueAddMenuEntryWithValue(ms->worldobj[i].drawmodemenu,"flat shaded", 10*(i+ms->numsegments)+1);
      glueAddMenuEntryWithValue(ms->worldobj[i].drawmodemenu,"solid fill", 10*(i+ms->numsegments)+2);
      glueAddMenuEntryWithValue(ms->worldobj[i].drawmodemenu,"wireframe", 10*(i+ms->numsegments)+3);
      glueAddMenuEntryWithValue(ms->worldobj[i].drawmodemenu,"outlined", 10*(i+ms->numsegments)+4);
      glueAddMenuEntryWithValue(ms->worldobj[i].drawmodemenu,"bounding box", 10*(i+ms->numsegments)+5);
      glueAddMenuEntryWithValue(ms->worldobj[i].drawmodemenu,"none", 10*(i+ms->numsegments)+6);
      glueAddMenuEntryWithValue(ms->worldobj[i].drawmodemenu,"---", 10*(i+ms->numsegments)+7);
      glueAddMenuEntryWithValue(ms->worldobj[i].drawmodemenu,"normal vectors", 10*(i+ms->numsegments)+8);
   }

   item_count = ms->numsegments + ms->numworldobjects;

   if (ms->dis.allsegsdrawmodemenu != 0)
      glutDestroyMenu(ms->dis.allsegsdrawmodemenu);
   ms->dis.allsegsdrawmodemenu = glueCreateMenu("Draw Mode");
   glueAddMenuEntryWithValue(ms->dis.allsegsdrawmodemenu,"gouraud shaded", 10*item_count);
   glueAddMenuEntryWithValue(ms->dis.allsegsdrawmodemenu,"flat shaded", 10*item_count+1);
   glueAddMenuEntryWithValue(ms->dis.allsegsdrawmodemenu,"solid fill", 10*item_count+2);
   glueAddMenuEntryWithValue(ms->dis.allsegsdrawmodemenu,"wireframe", 10*item_count+3);
   glueAddMenuEntryWithValue(ms->dis.allsegsdrawmodemenu,"outlined", 10*item_count+4);
   glueAddMenuEntryWithValue(ms->dis.allsegsdrawmodemenu,"bounding box", 10*item_count+5);
   glueAddMenuEntryWithValue(ms->dis.allsegsdrawmodemenu,"none", 10*item_count+6);
   glueAddMenuEntryWithValue(ms->dis.allsegsdrawmodemenu,"---", 10*item_count+7);
   glueAddMenuEntryWithValue(ms->dis.allsegsdrawmodemenu,"normal vectors", 10*item_count+8);

   item_count++;

   if (ms->dis.allligsdrawmodemenu != 0)
      glutDestroyMenu(ms->dis.allligsdrawmodemenu);
   ms->dis.allligsdrawmodemenu = glueCreateMenu("Draw Mode");
   glueAddMenuEntryWithValue(ms->dis.allligsdrawmodemenu,"gouraud shaded", 10*item_count);
   glueAddMenuEntryWithValue(ms->dis.allligsdrawmodemenu,"flat shaded", 10*item_count+1);
   glueAddMenuEntryWithValue(ms->dis.allligsdrawmodemenu,"solid fill", 10*item_count+2);
   glueAddMenuEntryWithValue(ms->dis.allligsdrawmodemenu,"wireframe", 10*item_count+3);
   glueAddMenuEntryWithValue(ms->dis.allligsdrawmodemenu,"outline", 10*item_count+4);
   glueAddMenuEntryWithValue(ms->dis.allligsdrawmodemenu,"bounding box", 10*item_count+5);
   glueAddMenuEntryWithValue(ms->dis.allligsdrawmodemenu,"none", 10*item_count+6);
   glueAddMenuEntryWithValue(ms->dis.allligsdrawmodemenu,"---", 10*item_count+7);
   glueAddMenuEntryWithValue(ms->dis.allligsdrawmodemenu,"normal vectors", 10*item_count+8);

   item_count++;

   if (ms->dis.allworlddrawmodemenu != 0)
      glutDestroyMenu(ms->dis.allworlddrawmodemenu);
   ms->dis.allworlddrawmodemenu = glueCreateMenu("Draw Mode");
   glueAddMenuEntryWithValue(ms->dis.allworlddrawmodemenu,"gouraud shaded", 10*item_count);
   glueAddMenuEntryWithValue(ms->dis.allworlddrawmodemenu,"flat shaded", 10*item_count+1);
   glueAddMenuEntryWithValue(ms->dis.allworlddrawmodemenu,"solid fill", 10*item_count+2);
   glueAddMenuEntryWithValue(ms->dis.allworlddrawmodemenu,"wireframe", 10*item_count+3);
   glueAddMenuEntryWithValue(ms->dis.allworlddrawmodemenu,"outlined", 10*item_count+4);
   glueAddMenuEntryWithValue(ms->dis.allworlddrawmodemenu,"bounding box", 10*item_count+5);
   glueAddMenuEntryWithValue(ms->dis.allworlddrawmodemenu,"none", 10*item_count+6);
   glueAddMenuEntryWithValue(ms->dis.allworlddrawmodemenu,"---", 10*item_count+7);
   glueAddMenuEntryWithValue(ms->dis.allworlddrawmodemenu,"normal vectors", 10*item_count+8);

   item_count++;

   if (ms->dis.alldrawmodemenu != 0)
      glutDestroyMenu(ms->dis.alldrawmodemenu);
   ms->dis.alldrawmodemenu = glueCreateMenu("Draw Mode");
   glueAddMenuEntryWithValue(ms->dis.alldrawmodemenu,"gouraud shaded", 10*item_count);
   glueAddMenuEntryWithValue(ms->dis.alldrawmodemenu,"flat shaded", 10*item_count+1);
   glueAddMenuEntryWithValue(ms->dis.alldrawmodemenu,"solid fill", 10*item_count+2);
   glueAddMenuEntryWithValue(ms->dis.alldrawmodemenu,"wireframe", 10*item_count+3);
   glueAddMenuEntryWithValue(ms->dis.alldrawmodemenu,"outlined", 10*item_count+4);
   glueAddMenuEntryWithValue(ms->dis.alldrawmodemenu,"bounding box", 10*item_count+5);
   glueAddMenuEntryWithValue(ms->dis.alldrawmodemenu,"none", 10*item_count+6);
   glueAddMenuEntryWithValue(ms->dis.alldrawmodemenu,"---", 10*item_count+7);
   glueAddMenuEntryWithValue(ms->dis.alldrawmodemenu,"normal vectors", 10*item_count+8);

   if (ms->dis.maindrawmodemenu != 0)
      glutDestroyMenu(ms->dis.maindrawmodemenu);
   ms->dis.maindrawmodemenu = glueCreateMenu("Object");
   if (ms->numseggroups > 0)
   {
      for (i = 0; i < ms->numseggroups; i++)
      {
         ms->seggroup[i].displaymodemenu = glueCreateMenu(ms->seggroup[i].name);
         
         item_count++;
         
         glueAddMenuEntryWithValue(ms->seggroup[i].displaymodemenu,"gouraud shaded", 10*item_count);
         glueAddMenuEntryWithValue(ms->seggroup[i].displaymodemenu,"flat shaded", 10*item_count+1);
         glueAddMenuEntryWithValue(ms->seggroup[i].displaymodemenu,"solid fill", 10*item_count+2);
         glueAddMenuEntryWithValue(ms->seggroup[i].displaymodemenu,"wireframe", 10*item_count+3);
         glueAddMenuEntryWithValue(ms->seggroup[i].displaymodemenu,"outlined", 10*item_count+4);
         glueAddMenuEntryWithValue(ms->seggroup[i].displaymodemenu,"bounding box", 10*item_count+5);
         glueAddMenuEntryWithValue(ms->seggroup[i].displaymodemenu,"none", 10*item_count+6);
         glueAddMenuEntryWithValue(ms->seggroup[i].displaymodemenu,"---", 10*item_count+7);
         glueAddMenuEntryWithValue(ms->seggroup[i].displaymodemenu,"normal vectors", 10*item_count+8);
         glueAddMenuEntryWithValue(ms->seggroup[i].displaymodemenu,"---", 10*item_count+10);
         glueAddMenuEntryWithValue(ms->seggroup[i].displaymodemenu,"segment axes GRP", 10*item_count+9);
         
         glueAddSubmenuEntry(ms->dis.maindrawmodemenu, ms->seggroup[i].name, ms->seggroup[i].displaymodemenu);
      }

      // The "individual segments" menu is used only if there are segment groups.
      if (ms->dis.eachsegdrawmodemenu != 0)
         glutDestroyMenu(ms->dis.eachsegdrawmodemenu);
      ms->dis.eachsegdrawmodemenu = glueCreateMenu("Segments");

      for (i = 0; i < ms->numsegments; i++)
         glueAddSubmenuEntry(ms->dis.eachsegdrawmodemenu, ms->segment[i].name, ms->segment[i].drawmodemenu);

      glueAddSubmenuEntry(ms->dis.maindrawmodemenu, "individual segments", ms->dis.eachsegdrawmodemenu);
   }
   else
   {
      for (i=0; i<ms->numsegments; i++)
      {
         glueAddSubmenuEntry(ms->dis.maindrawmodemenu,
                             ms->segment[i].name,ms->segment[i].drawmodemenu);
      }
   }

   for (i=0; i<ms->numworldobjects; i++)
   {
      glueAddSubmenuEntry(ms->dis.maindrawmodemenu,
                          ms->worldobj[i].name,ms->worldobj[i].drawmodemenu);
   }

   glueAddSubmenuEntry(ms->dis.maindrawmodemenu,"all body segments", ms->dis.allsegsdrawmodemenu);
   glueAddSubmenuEntry(ms->dis.maindrawmodemenu,"all muscle surfaces", ms->dis.allligsdrawmodemenu);
   glueAddSubmenuEntry(ms->dis.maindrawmodemenu,"all world objects", ms->dis.allworlddrawmodemenu);
   glueAddSubmenuEntry(ms->dis.maindrawmodemenu,"all objects", ms->dis.alldrawmodemenu);

}

void make_function_menu(ModelStruct* ms)
{
   int i;

   if (ms->functionMenu != 0)
      glutDestroyMenu(ms->functionMenu);

   ms->functionMenu = glueCreateMenu("Functions");
   for (i=0; i<ms->func_array_size; i++)
   {
       if (ms->function[i] && ms->function[i]->used == dpYes)
      {
         if (ms->function[i]->source == dpMuscleFile)
            sprintf(buffer, "m%d", ms->function[i]->usernum);
         else
            sprintf(buffer, "%d", ms->function[i]->usernum);
         glueAddMenuEntryWithValue(ms->functionMenu, buffer, FUNCTION_MENU_START + i);
      }
   }
}

void scene_input(WindowParams* win_parameters, WinUnion* win_struct, SimmEvent se)
{
   int i;

   glutSetWindow(win_parameters->id);

   if (se.field1 == window_quit)
   {
      delete_scene(win_struct->scene);
      return;
   }

#if ! CORTEX_PLUGIN
   if ((se.field1 == window_shut) || (((se.field1 == backspace_key) || (se.field1 == delete_key)) && se.field2 == key_pressed))
   {
      confirm_action(win_parameters, model_deletion_text, model_deletion_confirm);
      scene_to_confirm = win_struct->scene;
      return;
   }
#endif

   if (win_struct->scene == NULL)
      return;

   /* Because many tools are interested in mouse buttons pressed in the model window
    * (e.g. to change the model view or pick muscle points), no tool is allowed to
    * attach a function to a solo mouse button press. For example, to pick muscle
    * points in the MuscleEditor, you press the space bar and click the select button,
    * not just click the select button. Thus when a mouse button is pressed, do not
    * generate a SIMM event to notify the tools. It is the responsibility of each
    * tool to wait until it gets a non-mouse-button event, and then enter a loop within
    * its own code which uses getButton() to check if other buttons (like the mouse
    * buttons) have been pressed.
    * Consider the case of selecting muscle points. If the space bar is pressed first,
    * the MuscleEditor tool will get the corresponding SIMM event, and it will loop
    * as long as the space bar is pressed. If the select button (left mouse button)
    * is then pressed, this MuscleEditor loop will use getButton() calls to find out
    * that it is pressed, then let the user select muscle points.
    * If the select button is pressed first, no SIMM event will be generated, so none
    * of the tools will find out about the button press. Control flow will return to
    * glutMainLoop() which waits for and distributes all events. If the space
    * bar is then pressed, the MuscleEditor will pick up the corresponding SIMM event
    * and loop until the button is released. This loop will, as before, use getButton()
    * to see if the select button is pressed, but since the select button was pressed
    * first (and presumably was kept pressed as the space bar was pressed), the
    * MuscleEditor will immediately execute the code to let the user select muscle
    * points.
    */

   /* Hack to support right-clicking on a bone to change its drawmode */
    //TODO5.0: figure out which object was clicked on, and send an event to
    //the appropriate tool's simm event handler.
   if (se.field1 == rightmouse_button && se.field2 == key_pressed)
   {
        handle_right_click(win_struct->scene, se.mouse_x, se.mouse_y);
   }
   else if (se.field1 != leftmouse_button && se.field1 != middlemouse_button && se.field1 != rightmouse_button)
   {
      forward_simm_event(se, SCENE_INPUT_EVENT, (void*)win_struct->scene);
   }
}

static void handle_right_click(Scene* scene, int mouse_x, int mouse_y)
{
   GLubyte pixel[3];
   PickIndex obj_num, object_type, object;
   ModelDrawOptions mdo;

   mdo.mode = GL_SELECT;
   mdo.draw_world_objects = yes;
   mdo.draw_bones = yes;
   mdo.draw_muscles = yes;
   mdo.draw_selection_box = no;
   mdo.skin_mode = draw_skin_and_bones;
   mdo.field1 = mdo.field2 = DRAW_ALL;

   draw_model(scene, scene->model[0], &mdo);

   glutSetWindow(scene->window_glut_id);
   glReadBuffer(GL_BACK);
   glReadPixels(mouse_x, mouse_y, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, pixel);

   unpack_int_from_color(&object, pixel);

   if (object != 0)
   {
      SimmEvent se;

      get_object_type_and_number(object, &object_type, &obj_num);

      memset(&se, 0, sizeof(SimmEvent));
        se.event_code = SCENE_RIGHT_CLICK_EVENT;
        se.struct_ptr1 = (void*)scene;
        se.struct_ptr2 = (void*)scene->model[0];
      se.model_index = scene->model[0]->modelnum;
      se.mouse_x = mouse_x;
      se.mouse_y = mouse_y;
      se.window_id = scene->window_index;
      se.object = object;

      if (object_type == BONE_OBJECT || object_type == WORLD_OBJECT)
      {
            call_simm_event_handler("mv", se);
      }
      else if (object_type == MUSCLE_OBJECT)
      {
            call_simm_event_handler("me", se);
      }
      else if (object_type == MARKER_OBJECT)
      {
            call_simm_event_handler("mk", se);
      }
   }
}

static void call_simm_event_handler(const char tool_id[], SimmEvent se)
{
    int i;

    for (i=0; i<TOOLBUFFER; i++)
    {
        if (tool[i].used == yes && STRINGS_ARE_EQUAL(tool[i].name, tool_id))
        {
            (*tool[i].simm_event_handler)(se);
            return;
        }
    }
}

//TODO_SCENE: delete model or scene???
void model_deletion_confirm(SBoolean answer)
{
   if (answer == yes)
      delete_scene(scene_to_confirm);
}
#endif // OPENSMAC

ReturnCode find_segment_drawing_order(ModelStruct* ms)
{
   int i, index;
   SBoolean* seg_used;

   seg_used = (SBoolean*)simm_malloc(ms->numsegments*sizeof(SBoolean));
   ms->segment_drawing_order = (int*)simm_malloc(ms->numsegments*sizeof(int));

   if (seg_used == NULL || ms->segment_drawing_order == NULL)
      return code_bad;

   for (i=0; i<ms->numsegments; i++)
      seg_used[i] = no;

   ms->segment_drawing_order[0] = ms->ground_segment;
   seg_used[ms->ground_segment] = yes;
   index = 1;

   /* The next 3 lines are a hack until the code at the bottom works better */
   for (i=0; i<ms->numsegments; i++)
      if (seg_used[i] == no)
     ms->segment_drawing_order[index++] = i;

/*
   for (i=ms->numjoints-1; i>=0; i--)
   {
      if (joint_uses_segment(ms,i,ms->ground_segment,&other_seg) == yes &&
      seg_used[other_seg] == no)
      {
     ms->segment_drawing_order[index++] = other_seg;
     seg_used[other_seg] = yes;
      }
   }

   for (i=0; i<ms->numsegments; i++)
      if (seg_used[i] == no)
     ms->segment_drawing_order[index++] = i;
*/

/*
   printf("drawing order:\n");
   for (i=0; i<ms->numsegments; i++)
      printf("%s\n", ms->segment[ms->segment_drawing_order[i]].name);
*/

   free(seg_used);

   return code_fine;

}



static SBoolean joint_uses_segment(ModelStruct* ms, int jointnum, int segmentnum,
                   int* other_segment)
{

   if (ms->joint[jointnum].from == segmentnum)
   {
      *other_segment = ms->joint[jointnum].to;
      return (yes);
   }
   else if (ms->joint[jointnum].to == segmentnum)
   {
      *other_segment = ms->joint[jointnum].from;
      return (yes);
   }

   return (no);

}

//TODO_SCENE: should this be independent of scene?
public void size_model(Scene* scene, ModelStruct* ms, BoundingCube* bc)
{
   int i, j, num_bones = 0;
   double pnt[4];
   double xmin = MAXMDOUBLE, xmax = MINMDOUBLE;
   double ymin = MAXMDOUBLE, ymax = MINMDOUBLE;
   double zmin = MAXMDOUBLE, zmax = MINMDOUBLE;

   // Determine max_dimension of all model segments.
   for (i=0; i<ms->numsegments; i++)
   {
      for (j=0; j<ms->segment[i].numBones; j++, num_bones++)
      {
         if (ms->segment[i].bone[j].num_vertices == 0)
         {
            pnt[0] = pnt[1] = pnt[2] = 0.0;//dkb dec30, 2003 -1.0;
         }
         else
         {
#if NOSCALE
            pnt[0] = ms->segment[i].bone[j].bc.x1 * ms->segment[i].bone_scale[0];
            pnt[1] = ms->segment[i].bone[j].bc.y1 * ms->segment[i].bone_scale[1];
            pnt[2] = ms->segment[i].bone[j].bc.z1 * ms->segment[i].bone_scale[2];
#else
            pnt[0] = ms->segment[i].bone[j].bc.x1;
            pnt[1] = ms->segment[i].bone[j].bc.y1;
            pnt[2] = ms->segment[i].bone[j].bc.z1;
#endif
            convert(ms, pnt, i, ms->ground_segment);
         }
         if (pnt[0] < xmin)
            xmin = pnt[0];
         if (pnt[1] < ymin)
            ymin = pnt[1];
         if (pnt[2] < zmin)
            zmin = pnt[2];

         if (ms->segment[i].bone[j].num_vertices == 0)
         {
            pnt[0] = pnt[1] = pnt[2] = 0.0;//dkb dec 30, 2003  1.0;
         }
         else
         {
#if NOSCALE
            pnt[0] = ms->segment[i].bone[j].bc.x2 * ms->segment[i].bone_scale[0];
            pnt[1] = ms->segment[i].bone[j].bc.y2 * ms->segment[i].bone_scale[1];
            pnt[2] = ms->segment[i].bone[j].bc.z2 * ms->segment[i].bone_scale[2];
#else
            pnt[0] = ms->segment[i].bone[j].bc.x2;
            pnt[1] = ms->segment[i].bone[j].bc.y2;
            pnt[2] = ms->segment[i].bone[j].bc.z2;
#endif
            convert(ms, pnt, i, ms->ground_segment);
         }
         if (pnt[0] > xmax)
            xmax = pnt[0];
         if (pnt[1] > ymax)
            ymax = pnt[1];
         if (pnt[2] > zmax)
            zmax = pnt[2];
      }
   }

   // This code handles the case in which there are no bones in the
   // model. Ideally, you should look at muscle attachment points or
   // joint parameters to determine the best dimensions.
   if (num_bones == 0)
   {
      xmin = ymin = zmin = -1.0;
      xmax = ymax = zmax = 1.0;
   }

   ms->max_dimension = xmax - xmin;
   if (ymax-ymin > ms->max_dimension)
      ms->max_dimension = ymax - ymin;
   if (zmax-zmin > ms->max_dimension)
      ms->max_dimension = zmax - zmin;

   // Determine max_dimension of all model segments and world objects.
   for (i = 0; i < ms->numworldobjects; i++)
   {
      double npnt[4];
      WorldObject* obj = &ms->worldobj[i];

      pnt[0] = obj->wobj->bc.x1;
      pnt[1] = obj->wobj->bc.y1;
      pnt[2] = obj->wobj->bc.z1;
      pnt[3] = 1.0;
      mult_4x4matrix_by_vector(obj->transform, pnt, npnt);

      if (npnt[0] < xmin) xmin = npnt[0];
      if (npnt[1] < ymin) ymin = npnt[1];
      if (npnt[2] < zmin) zmin = npnt[2];

      pnt[0] = obj->wobj->bc.x2;
      pnt[1] = obj->wobj->bc.y2;
      pnt[2] = obj->wobj->bc.z2;
      pnt[3] = 1.0;
      mult_4x4matrix_by_vector(obj->transform, pnt, npnt);

      if (npnt[0] > xmax) xmax = npnt[0];
      if (npnt[1] > ymax) ymax = npnt[1];
      if (npnt[2] > zmax) zmax = npnt[2];
   }
   ms->max_dimension2 = xmax - xmin;
   if (ymax-ymin > ms->max_dimension2)
      ms->max_dimension2 = ymax - ymin;
   if (zmax-zmin > ms->max_dimension2)
      ms->max_dimension2 = zmax - zmin;
   
   // Compute max_diagonal of all model segments and world objects.
   pnt[0] = xmax - xmin;
   pnt[1] = ymax - ymin;
   pnt[2] = zmax - zmin;

   ms->max_diagonal = sqrt(SQR(pnt[0]) + SQR(pnt[1]) + SQR(pnt[2]));
   scene->max_diagonal = ms->max_diagonal; //TODO_SCENE: scene should have its own diagonal calculation

   ms->max_diagonal_needs_recalc = no;

#if 0
   fprintf(stderr, "%s\n", ms->name);
   fprintf(stderr, "  min: %6.2f, %6.2f, %6.2f\n", xmin, ymin, zmin);
   fprintf(stderr, "  max: %6.2f, %6.2f, %6.2f\n", xmax, ymax, zmax);
   fprintf(stderr, "  max_dimension:  %.2f\n", ms->max_dimension);
   fprintf(stderr, "  max_dimension2: %.2f\n", ms->max_dimension2);
   fprintf(stderr, "  max_diagonal:   %.2f\n", ms->max_diagonal);
#endif

   calc_near_and_far_clip_planes(scene, fabs(scene->tz));

   if (bc)
   {
      bc->x1 = xmin;
      bc->x2 = xmax;
      bc->y1 = ymin;
      bc->y2 = ymax;
      bc->z1 = zmin;
      bc->z2 = zmax;
   }
}

//TODO_SCENE: make this work for multiple models
void size_model_display(Scene* scene, ModelStruct* ms, int* suggested_width, int* suggested_height)
{
   int i, j, xpixwidth, sug_height, num_bones=0;
   BoundingCube bc;

   size_model(scene, ms, &bc);

   scene->start_tx = scene->tx = -(bc.x2+bc.x1)/2.0;
   scene->start_ty = scene->ty = -(bc.y2+bc.y1)/2.0;
   scene->start_tz = scene->tz = ms->max_dimension * -1.7;

   if (ms->specified_min_thickness == no)
   {
      if (ms->specified_max_thickness == yes)
         *ms->default_muscle->min_thickness = (*ms->default_muscle->max_thickness)*0.25;
      else
         *ms->default_muscle->min_thickness = ms->max_dimension*0.002;
   }
   if (ms->specified_max_thickness == no)
   {
      if (ms->specified_min_thickness == yes)
         *ms->default_muscle->max_thickness = (*ms->default_muscle->min_thickness)*4.0;
      else
         *ms->default_muscle->max_thickness = ms->max_dimension*0.008;
   }

   ms->dis.muscle_point_radius = ms->max_dimension * 0.0026;
   ms->dis.muscle_point_id = -1;

   scene->model_move_increment = ms->max_dimension * 0.02;

   if (*suggested_width < 0)
      *suggested_width = WINDOW_WIDTH;

   if (*suggested_height < 0)
   {
      xpixwidth = *suggested_width - 2*WINDOW_MARGIN;
      sug_height = xpixwidth*(bc.y2-bc.y1)/(bc.x2-bc.x1) + 2*WINDOW_MARGIN;
      *suggested_height = _MIN(900,sug_height);
   }

   for (i = 0; i < ms->numsegments; i++)
   {
      ms->segment[i].mc_radius = ms->max_dimension * 0.007;

      for (j = 0; j < ms->segment[i].numSpringPoints; j++)
      {
         if (ms->default_muscle->min_thickness && ms->default_muscle->max_thickness)
            ms->segment[i].springPoint[j].radius = *ms->default_muscle->min_thickness;
         else
            ms->segment[i].springPoint[j].radius = ms->max_dimension * 0.002;
      }
   }

   {
      int screen_y = glutGet(GLUT_SCREEN_HEIGHT);

      if (*suggested_height > screen_y - 40)
         *suggested_height = screen_y - 40;
   }
}


public void calc_near_and_far_clip_planes(Scene* scene, double viewVecLen)
{
   if (viewVecLen > scene->max_diagonal * 2.0)
      scene->near_clip_plane = viewVecLen - scene->max_diagonal * 2.0;
   else
      scene->near_clip_plane = viewVecLen / 1000.0;

   scene->far_clip_plane = viewVecLen + scene->max_diagonal * 2.0;

#if 0
   fprintf(stderr, "n: %.2f (%s), v: %.2f, f: %.2f\n",
      ms->dis.near_clip_plane,
      viewVecLen > ms->max_diagonal ? " " : "*",
      viewVecLen,
      ms->dis.far_clip_plane);
#endif
}


#if INCLUDE_MOCAP_MODULE

#if WIN32

/* -------------------------------------------------------------------------
   _read_token - 
---------------------------------------------------------------------------- */
static SBoolean _read_token (FILE* file, char* token)
{
   SBoolean didReadToken = yes;
   
  restart:
   /* ignore any preceeding whitespace:
    */
   token[0] = fgetc(file);
   
   while (isspace(token[0]) && token[0] != (char) EOF)
      token[0] = fgetc(file);
   
   /* check for comments:
    */
   if (token[0] == '#')
   {
      _read_til(file, '\n');
      
      if (feof(file) || ferror(file))
         token[0] = EOF;
      else
         goto restart;
   }
   
   /* read the rest of the token:
    */
   if (token[0] == (char) EOF)
   {
      didReadToken = no;

      token[0] = '\0';
   }
   else
   {
      int i = 0;
      
      while ( ! isspace(token[i]) && token[i] != (char) EOF)
         token[++i] = fgetc(file);
      
      token[i] = '\0';
   }
   
   return didReadToken;
}

#if ! SIMM_VIEWER

STRUCT {
   int numFrames;
   int dataFrameRate;
   int numSegments;
   int gravityAxis;
} HtrHeaderInfo;

/* -------------------------------------------------------------------------
   scan_htr_header - read the header section of the specified HTR file to
      extract a few key bits of information that will help us choose good
      defaults for the motion import options dialog box.
---------------------------------------------------------------------------- */
static void scan_htr_header (const char* htrFile, HtrHeaderInfo* htrInfo)
{
   FILE* file = simm_fopen(htrFile, "r");
   
   if (file)
   {
      while (_read_token(file, buffer))
      {
         if (STRINGS_ARE_EQUAL(buffer, "[SegmentNames&Hierarchy]") ||
             STRINGS_ARE_EQUAL(buffer, "[BasePosition]"))
         {
            break;
         }
         else if (STRINGS_ARE_EQUAL(buffer, "NumFrames"))
         {
            fscanf(file, "%d", &htrInfo->numFrames);
         }
         else if (STRINGS_ARE_EQUAL(buffer, "DataFrameRate"))
         {
            fscanf(file, "%d", &htrInfo->dataFrameRate);
         }
         else if (STRINGS_ARE_EQUAL(buffer, "NumSegments"))
         {
            fscanf(file, "%d", &htrInfo->numSegments);
         }
         else if (STRINGS_ARE_EQUAL(buffer, "GlobalAxisofGravity"))
         {
            if (fscanf(file, "%s", buffer) == 1)
            {
               switch (tolower(buffer[0]))
               {
                  case 'x': htrInfo->gravityAxis = POS_X; break;
                  case 'y': htrInfo->gravityAxis = POS_Y; break;
                  case 'z': htrInfo->gravityAxis = POS_Z; break;
               }
            }
         }
      }
      fclose(file);
   }
}

#if ! OPENSMAC

/* The mocap options are needed by the real-time analog import,
 * so make them global. This assumes that:
 * (1) A post-processed htr and/or analog file was pre-loaded
 * (2) the real-time data matches the format of the *last* time a
 *     post-processed htr and/or analog file was loaded.
 * Not the best way of handling this!!! JPL 10/31/00
 */
glutMocapOptions options;

/* -------------------------------------------------------------------------
   open_motion_analysis_file - Win32-specific routine to display mocap
     options and then load mocap files.
---------------------------------------------------------------------------- */
public ReturnCode open_motion_analysis_file(
   const char htrFile[],
   int        modelIndex,
   int         numAnalogFiles,
   const char* analogFiles[])
{
   HtrHeaderInfo htrInfo;
   char *p, htrPathBuf[1024], *baseName, *extension, *htrPath = htrPathBuf;
   int i;
   SBoolean isHtr2 = no;
   ReturnCode rc = code_fine;

   memset(&options, 0, sizeof(options)); /* build_file_list_from_pattern() depends on this! */

   /* split the input file name into path and base-name componants:
    */
   strcpy(htrPathBuf, htrFile);

   extension = strrchr(htrPathBuf, '.');

   if (extension)
   {
      lowerstr(extension);

      *extension++ = '\0';
      
      isHtr2 = (SBoolean) STRINGS_ARE_EQUAL(extension, "htr2");
   }
   p = strrchr(htrPathBuf, DIR_SEP_CHAR);
   
   if (p)
   {
      baseName = p + 1;
      *p = '\0';
   }
   else
      baseName = htrPathBuf;

   /* extract useful information from htr file header:
    */
   htrInfo.numFrames = 300;
   htrInfo.dataFrameRate = 60;
   htrInfo.numSegments = 0;
   htrInfo.gravityAxis = POS_Y;

   scan_htr_header(htrFile, &htrInfo);

   /* scan the SIMM/Resources/mocap/mappings/ directory for mocap mapping files:
    */
   sprintf(buffer, "%s%s*", get_mocap_folder(), "mappings" DIR_SEP_STRING);

   build_file_list_from_pattern(buffer, &options.mappings, &options.numMappings);
   options.selectedMapping = 0;

   /* look for a mapping file name that contains the number of segments
    * that are specified in the htr file's header:
    */
   if (htrInfo.numSegments > 0 && options.numMappings > 0)
   {
      char numSegs[32];

      sprintf(numSegs, "%d", htrInfo.numSegments);

      for (i = 0; i < options.numMappings; i++)
      {
         const char* p = strstr(options.mappings[i], numSegs);

         if (p && (p == options.mappings[i] || ! isdigit(*(p - 1))))
         {
            options.selectedMapping = i;
            break;
         }
      }
   }

   /* scan the HTR file's directory and SIMM/Resources/mocap/basePositions/
    * for base position files:
    */
   sprintf(buffer, "%s%s*.pose", htrPath, DIR_SEP_STRING);

   build_file_list_from_pattern(buffer, &options.basePositions, &options.numBasePositions);

   sprintf(buffer, "%s%s*", get_mocap_folder(), "basePositions" DIR_SEP_STRING);

   build_file_list_from_pattern(buffer, &options.basePositions, &options.numBasePositions);

   /* look for a base postion file that has the same base name as
    * the motion file:
    */
   options.selectedBasePosition = -1;

   for (i = 0; i < options.numBasePositions; i++)
   {
      /* check if the htr file's base name is part of a base position file:
       */
      if (strstr(options.basePositions[i], baseName))
      {
         options.selectedBasePosition = i;
         break;
      }
   }

   if (options.selectedBasePosition == -1)
   {
      /* check if the a base position file's base name is part of the htr
       * file's base name:
       */
      for (i = 0; i < options.numBasePositions; i++)
      {
         p = strrchr(options.basePositions[i], DIR_SEP_CHAR);

         strcpy(buffer, p ? p+1 : options.basePositions[i]);

         if (strrchr(buffer, '.'))
            *strrchr(buffer, '.') = '\0';

         if (strstr(baseName, buffer))
         {
            options.selectedBasePosition = i;
            break;
         }
      }
   }

   if (options.selectedBasePosition == -1)
   {
      /* otherwise select the appropriate base position as the default:
       */
      for (i = 0; i < options.numBasePositions; i++)
      {
         if (strstr(options.basePositions[i], isHtr2 ? "global" : "init"))
         {
            options.selectedBasePosition = i;
            break;
         }
      }
   }

   if (options.selectedBasePosition == -1)
      options.selectedBasePosition = 0;

   /* initialize the list of analog files to be imported with the motion:
    */
   if (numAnalogFiles > 0 && analogFiles)
   {
      options.numAnalogFiles = numAnalogFiles;
      options.analogFiles    = (char**) analogFiles;
   }
   else
   {
      /* scan the HTR file's directory for analog & xls files with the same
       * base name at the HTR file:
       */
      if (options.numAnalogFiles == 0)
      {
         sprintf(buffer, "%s%s%s.anb", htrPath, DIR_SEP_STRING, baseName);
         build_file_list_from_pattern(buffer, &options.analogFiles, &options.numAnalogFiles);
      }

      if (options.numAnalogFiles == 0)
      {
         sprintf(buffer, "%s%s%s.anc", htrPath, DIR_SEP_STRING, baseName);
         build_file_list_from_pattern(buffer, &options.analogFiles, &options.numAnalogFiles);
      }

      sprintf(buffer, "%s%s%s.xls", htrPath, DIR_SEP_STRING, baseName);
      build_file_list_from_pattern(buffer, &options.analogFiles, &options.numAnalogFiles);
   }

   /* init the remainder of the mocap options record:
    */
   options.inputFile = htrFile;
   options.upDirection = htrInfo.gravityAxis;

   switch (options.upDirection)
   {
     case POS_X: options.faceDirection = POS_Z; break;
     case POS_Y: options.faceDirection = POS_Z; break;
     case POS_Z: options.faceDirection = POS_X; break;
   }
   options.rightLeg = 1;
   options.leftLeg = 1;
   options.upperBody = 1;
   options.muscles = 1;
   options.scaleSegments = 1;
   options.emgSmoothingCycles = 1;
   options.emgWindowSize = 0.032;
   options.showBasePosition = 0;
   options.showZeroFrame = 0;
   options.autocalibrateForceplates = 1;
   options.modelName = baseName;
   options.saveIntermediateFiles = 0;

   sprintf(buffer, "%s.jnt", baseName);
   mstrcpy(&options.jntFile, buffer);

   sprintf(buffer, "%s.msl", baseName);
   mstrcpy(&options.mslFile, buffer);

   sprintf(buffer, "%s.mot", baseName);
   mstrcpy(&options.motFile, buffer);

   /* initialize the realtime import options:
    */
   strcpy(buffer, baseName);
   lowerstr(buffer);

   if (is_module_present(MOTION_REAL) == no)
      options.allowRealtimeImport = -1;
   else
      options.allowRealtimeImport = strstr(buffer, "realtime") ? 1 : 0;
   options.realtimeDuration = (double) htrInfo.numFrames / htrInfo.dataFrameRate;
   options.realtimeFrequency = htrInfo.dataFrameRate;
   options.timeScaleMin = - options.realtimeDuration;
   options.timeScaleMax = 0.0;
   options.slidingTimeScale = 1;

   /* display the mocap options dialog to the user:
    */
   if (glutMocapOptionsBox(&options) != GLUT_MSG_OK)
   {
      rc = code_bad;
      goto cleanup;
   }

   /* translate the specified Motion Analysis HTR file into SIMM joint,
    * muscle, and motion files:
    */
   rc = mocap_to_simm(&options, &modelIndex);
   
   if (rc != code_fine)
      goto cleanup;
   
   //resize_model_display(gModel[modelIndex]); TODO_SCENE: mocap_to_simm should return scene index?

#if 0
   fprintf(stderr, "@@@ %d segments, %d joints, %d gencoords, %d muscles\n",
              model[modelIndex]->numsegments,
              model[modelIndex]->numjoints,
              model[modelIndex]->numgencoords,
              model[modelIndex]->nummuscles);
#endif

  cleanup:
   if (options.mappings)
   {
      for (i = 0; i < options.numMappings; i++)
         FREE_IFNOTNULL(options.mappings[i]);

      free(options.mappings);
   }
   if (options.basePositions)
   {
      for (i = 0; i < options.numBasePositions; i++)
         FREE_IFNOTNULL(options.basePositions[i]);

      free(options.basePositions);
   }
   if (options.analogFiles != analogFiles)
   {
      for (i = 0; i < options.numAnalogFiles; i++)
         FREE_IFNOTNULL(options.analogFiles[i]);

      free(options.analogFiles);
   }
   FREE_IFNOTNULL(options.jntFile);
   FREE_IFNOTNULL(options.mslFile);
   FREE_IFNOTNULL(options.motFile);

   return rc;

} /* open_motion_analysis_file */

#endif // ! OPENSMAC


/* -------------------------------------------------------------------------
   open_tracked_file - Win32-specific routine to display trc import options and
     load a trc file.
---------------------------------------------------------------------------- */
public ReturnCode open_tracked_file(
   const char trcFile[],
   int        modelIndex,
   int         numAnalogFiles,
   const char* analogFiles[],
   SBoolean showDialogBox)
{
   FILE* fp;
   glutTRCOptions options;
   smTRCHeaderInfo trcInfo;
   ReturnCode rc;
   char *p, trcPathBuf[CHARBUFFER], buf[CHARBUFFER], buf2[CHARBUFFER], analogFileBuf[CHARBUFFER];
   char *baseName, *extension, *trcPath = trcPathBuf;
   ModelStruct* ms = NULL;
   smC3DStruct* c3d = NULL;
   MocapInfo mi;
   glutMocapOptions mo;

   if (modelIndex < 0)
      return code_bad;

   ms = gModel[modelIndex];

   init_tracked_file_options(ms, &options, trcFile);
   options.isC3DFile = 0;
   options.markerNameSource = -1;

   /* split the input file name into path and base-name components */
   strcpy(trcPathBuf, trcFile);

   extension = strrchr(trcPathBuf, '.');

   if (extension)
   {
      lowerstr(extension);
      *extension++ = '\0';
   }
   p = strrchr(trcPathBuf, DIR_SEP_CHAR);

   if (p)
   {
      baseName = p + 1;
      *p = '\0';
   }
   else
      baseName = trcPathBuf;

   if (smReadTrackedFileHeader(trcFile, &trcInfo) != smNoError)
   {
      strcpy(errorbuffer, smGetGlobalErrorBuffer());
      error(abort_action, errorbuffer);
      return code_bad;
   }

   if (trcInfo.numFrames <= 0)
   {
      sprintf(buffer, "There are no frames of data in %s", trcFile);
      error(none, buffer);
      return code_bad;
   }

   options.firstFrame = trcInfo.firstFrameNum;
   options.lastFrame = trcInfo.firstFrameNum + trcInfo.numFrames - 1;

   sprintf(buffer, "markers: %d   frames: %d  (%d to %d)", trcInfo.numMarkers, trcInfo.numFrames,
      trcInfo.firstFrameNum, trcInfo.numFrames + trcInfo.firstFrameNum - 1);
   mstrcpy(&options.infoText[0], buffer);
   sprintf(buffer, "data rate: %.1lf Hz   camera rate: %.1lf Hz", trcInfo.dataRate, trcInfo.cameraRate);
   mstrcpy(&options.infoText[1], buffer);

   /* initialize the list of analog files to be imported with the motion */
   if (numAnalogFiles > 0 && analogFiles)
   {
      options.numAnalogFiles = numAnalogFiles;
      options.analogFiles    = (char**) analogFiles;
      /* If the user selected one or more analog files, disable the analog auto-load function */
      options.loadAnalogData = 3; /* auto-load will be inactive, but other analog checkboxes will be active */
   }
   else
   {
      /* Look for the corresponding analog data files in the chosen folder.
       * If one or more exists, set the loadAnalogData option to yes;
       * if none exists, set the option to be grayed out.
       */
      options.loadAnalogData = 1; /* default is active and checked */
      strcpy(analogFileBuf, baseName);
      strcat(analogFileBuf, ".anb");
      if ((fp = fopen(analogFileBuf, "r")) == NULL)
      {
         strcpy(analogFileBuf, baseName);
         strcat(analogFileBuf, ".anc");
         if ((fp = fopen(analogFileBuf, "r")) == NULL)
         {
            strcpy(analogFileBuf, baseName);
            strcat(analogFileBuf, ".xls");
            if ((fp = fopen(analogFileBuf, "r")) == NULL)
            {
               /* no corresponding analog files are in the folder */
               options.loadAnalogData = 2; /* all analog checkboxes will be inactive */
            }
         }
      }
      if (fp != NULL)
         fclose(fp);

      /* scan the HTR file's directory for analog & xls files with the same
       * base name at the HTR file:
       */
      if (options.loadAnalogData == 1)
      {
         if (options.numAnalogFiles == 0)
         {
            sprintf(buffer, "%s%s%s.anb", trcPath, DIR_SEP_STRING, baseName);
            build_file_list_from_pattern(buffer, &options.analogFiles, &options.numAnalogFiles);
         }

         if (options.numAnalogFiles == 0)
         {
            sprintf(buffer, "%s%s%s.anc", trcPath, DIR_SEP_STRING, baseName);
            build_file_list_from_pattern(buffer, &options.analogFiles, &options.numAnalogFiles);
         }

         sprintf(buffer, "%s%s%s.xls", trcPath, DIR_SEP_STRING, baseName);
         build_file_list_from_pattern(buffer, &options.analogFiles, &options.numAnalogFiles);
      }
   }

   if (showDialogBox)
   {
      // Display the trc options dialog to the user.
      if (glutTRCOptionsBox(&options) != GLUT_MSG_OK)
      {
         rc = code_bad;
         goto cleanup;
      }
      else
      {
         // To the user, firstFrame and lastFrame are w.r.t. the user-defined
         // frame numbers in the file, which may not start at 1. Once the user
         // has selected the desired frame sequence, keep them defined this way,
         // but check to make sure the user hasn't changed them to invalid
         // numbers.
         if (options.firstFrame < trcInfo.firstFrameNum)
            options.firstFrame = trcInfo.firstFrameNum;
         else if (options.firstFrame > (trcInfo.firstFrameNum + trcInfo.numFrames - 1))
            options.firstFrame = trcInfo.firstFrameNum + trcInfo.numFrames - 1;

         if (options.lastFrame < trcInfo.firstFrameNum)
            options.lastFrame = trcInfo.firstFrameNum;
         else if (options.lastFrame > (trcInfo.firstFrameNum + trcInfo.numFrames - 1))
            options.lastFrame = trcInfo.firstFrameNum + trcInfo.numFrames - 1;

         if (options.frameIncrement == 0)
            options.frameIncrement = 1;
         else if (options.firstFrame > options.lastFrame && options.frameIncrement > 0)
            options.frameIncrement = -options.frameIncrement;
      }
   }

   // These are the only two fields in MocapInfo that are
   // needed for importing analog data.
   mi.model = ms;
   mi.dataFrameRate = trcInfo.dataRate;

   switch (mi.model->gravity)
   {
      case smNegX:
         mo.upDirection = POS_X;
         break;
      case smX:
         mo.upDirection = NEG_X;
         break;
      case smNegY:
         mo.upDirection = POS_Y;
         break;
      case smY:
         mo.upDirection = NEG_Y;
         break;
      case smNegZ:
         mo.upDirection = POS_Z;
         break;
      case smZ:
         mo.upDirection = NEG_Z;
         break;
      default:
         mo.upDirection = NEG_Z;
         break;
   }

   mo.emgSmoothingCycles = options.emgSmoothingPasses;
   mo.emgWindowSize = options.emgWindowSize;
   mo.muscles = 1;
   mo.autocalibrateForceplates = options.calibrateForcePlates;
   mo.calculateGaitEvents = options.calculateGaitEvents;

   // Load all of the analog data files into the smAnalogStruct struct,
   // then pass the struct to loadTrackedFile() so it can put the data
   // into the motion.
   if (options.loadAnalogData == 1 && options.numAnalogFiles > 0)
      c3d = read_analog_data_files(options.numAnalogFiles, options.analogFiles, &mi, &mo);

   // If there are no analog files or there was an error reading them,
   // allocate space for the C3D struct.
   if (!c3d)
      c3d = (smC3DStruct*) simm_calloc(1, sizeof(smC3DStruct));

   rc = loadTrackedFile(ms, &options, c3d, yes);

   free_analog_data(c3d->analogData);
   free(c3d);

   if (rc != code_fine)
      error(none, "Could not create model.");

cleanup:
   smFreeTRCHeader(&trcInfo);
   FREE_IFNOTNULL(options.inputFile);
   FREE_IFNOTNULL(options.htrFile);
   FREE_IFNOTNULL(options.motFile);

   return rc;
}

/* -------------------------------------------------------------------------
   open_c3d_file - Win32-specific routine to display c3d import options and
     load a c3d file.
---------------------------------------------------------------------------- */
public ReturnCode open_c3d_file(const char c3dFile[], int modelIndex, SBoolean showDialogBox)
{
   int numFrames;
   glutTRCOptions options;
   smC3DHeader head;
   ReturnCode rc;
   char *p, buf[CHARBUFFER], buf2[CHARBUFFER];
   ModelStruct* ms = NULL;
   MocapInfo mi;
   glutMocapOptions mo;

   if (modelIndex < 0)
      return code_bad;

   ms = gModel[modelIndex];

   init_tracked_file_options(ms, &options, c3dFile);
   options.isC3DFile = 1;

   smScanC3DHeader(c3dFile, &head);

   numFrames = head.lastFrame - head.firstFrame + 1;

   if (numFrames <= 0)
   {
      sprintf(buffer, "There are no frames of data in %s", c3dFile);
      error(none, buffer);
      return code_bad;
   }

   options.firstFrame = head.firstFrame;
   options.lastFrame = head.lastFrame;

   sprintf(buffer, "markers: %d   frames: %d", head.num3DPoints, numFrames);
   mstrcpy(&options.infoText[0], buffer);
   sprintf(buffer, "data rate: %.1lf Hz", head.pointFrameRate);
   mstrcpy(&options.infoText[1], buffer);

   if (showDialogBox)
   {
      // Display the trc options dialog to the user.
      if (glutTRCOptionsBox(&options) != GLUT_MSG_OK)
      {
         rc = code_bad;
         goto cleanup;
      }
      else
      {
         // To the user, firstFrame and lastFrame are w.r.t. the user-defined
         // frame numbers in the file, which may not start at 1. Once the user
         // has selected the desired frame sequence, keep them defined this way,
         // but check to make sure the user hasn't changed them to invalid
         // numbers.
         if (options.firstFrame < head.firstFrame)
            options.firstFrame = head.firstFrame;
         else if (options.firstFrame > head.lastFrame)
            options.firstFrame = head.lastFrame;

         if (options.lastFrame < head.firstFrame)
            options.lastFrame = head.firstFrame;
         else if (options.lastFrame > head.lastFrame)
            options.lastFrame = head.lastFrame;

         if (options.frameIncrement == 0)
            options.frameIncrement = 1;
         else if (options.firstFrame > options.lastFrame && options.frameIncrement > 0)
            options.frameIncrement = -options.frameIncrement;
      }
   }

   // These are the only two fields in MocapInfo that are
   // needed for importing analog data.
   mi.model = ms;
   mi.dataFrameRate = head.pointFrameRate;

   switch (mi.model->gravity)
   {
      case smNegX:
         mo.upDirection = POS_X;
         break;
      case smX:
         mo.upDirection = NEG_X;
         break;
      case smNegY:
         mo.upDirection = POS_Y;
         break;
      case smY:
         mo.upDirection = NEG_Y;
         break;
      case smNegZ:
         mo.upDirection = POS_Z;
         break;
      case smZ:
         mo.upDirection = NEG_Z;
         break;
      default:
         mo.upDirection = NEG_Z;
         break;
   }

   mo.emgSmoothingCycles = options.emgSmoothingPasses;
   mo.emgWindowSize = options.emgWindowSize;
   mo.muscles = 1;
   mo.autocalibrateForceplates = options.calibrateForcePlates;

   rc = loadC3DFile(ms, &options, &mi, &mo);

   if (rc != code_fine)
      error(none, "Could not create model.");

cleanup:
   FREE_IFNOTNULL(options.inputFile);
   FREE_IFNOTNULL(options.htrFile);
   FREE_IFNOTNULL(options.motFile);

   return rc;
}

static void init_tracked_file_options(ModelStruct* model, glutTRCOptions* options, const char inputFileName[])
{
   int len;
   const char* pref;

   memset(options, 0, sizeof(glutTRCOptions));

   mstrcpy(&options->inputFile, inputFileName);

   // the maximum length of htrFile and motFile
   len = strlen(inputFileName) + 4;

   options->htrFile = (char*)simm_malloc(len * sizeof(char));
   change_filename_suffix(inputFileName, options->htrFile, "htr", len);
   options->motFile = (char*)simm_malloc(len * sizeof(char));
   change_filename_suffix(inputFileName, options->motFile, "mot", len);

   pref = get_preference("MOCAP_INCREMENT");
   if (pref)
      options->frameIncrement = atoi(pref);
   else
      options->frameIncrement = 1;

   if (model->solver.method == smLevenbergMarquart)
      options->quickSolve = 0;
   else
      options->quickSolve = 1;

   if (is_preference_on(get_preference("MOCAP_CROP_ENDS")) == no)
      options->cropEnds = 0;
   else
      options->cropEnds = 1;

   if (is_preference_on(get_preference("MOCAP_CALC_DERIVATIVES")) == yes)
      options->calculateDerivatives = 1;
   else
      options->calculateDerivatives = 0;

   if (is_preference_on(get_preference("MOCAP_SHOW_MARKERS")) == yes)
      options->showMarkers = 1;
   else
      options->showMarkers = 0;

   if (is_preference_on(get_preference("MOCAP_CALC_GAIT_EVENTS")) == yes)
      options->calculateGaitEvents = 1;
   else
      options->calculateGaitEvents = 0;

   options->xAxisUnits = X_AXIS_TIME;
   pref = get_preference("MOCAP_X_AXIS");
   if (pref)
   {
      if (STRINGS_ARE_EQUAL(pref, "TIME"))
         options->xAxisUnits = X_AXIS_TIME;
      else if (STRINGS_ARE_EQUAL(pref, "FRAME_NUMBER"))
         options->xAxisUnits = X_AXIS_FRAME_NUMBER;
   }

   if (is_preference_on(get_preference("MOCAP_START_AT_ZERO")) == yes)
      options->xAxisStartZero = 1;
   else
      options->xAxisStartZero = 0;

   if (is_preference_on(get_preference("MOCAP_LOAD_ANALOG")) == no)
      options->loadAnalogData = 0;
   else
      options->loadAnalogData = 1; // active and checked

   if (is_preference_on(get_preference("MOCAP_CALIBRATE_FORCES")) == no)
      options->calibrateForcePlates = 0;
   else
      options->calibrateForcePlates = 1;

   options->emgSmoothingPasses = initializeEmgSmoothingPasses();
   options->emgWindowSize = initializeEmgWindowSize();

   options->markerNameSource = POINT_LABELS;
   pref = get_preference("MOCAP_MARKER_NAME_SOURCE");
   if (pref)
   {
      if (STRINGS_ARE_EQUAL(pref, "POINT:LABELS"))
         options->markerNameSource = POINT_LABELS;
      else if (STRINGS_ARE_EQUAL(pref, "POINT:DESCRIPTIONS"))
         options->markerNameSource = POINT_DESCRIPTIONS;
   }

   if (is_preference_on(get_preference("MOCAP_SAVE_HTR_FILE")) == yes)
      options->saveHTRFile = 1;
   else
      options->saveHTRFile = 0;

   if (is_preference_on(get_preference("MOCAP_SAVE_MOTION_FILE")) == yes)
      options->saveMOTFile = 1;
   else
      options->saveMOTFile = 0;
}

#endif /* ! SIMM_VIEWER */

#endif /* WIN32 */

#endif /* INCLUDE_MOCAP_MODULE */

#endif /* ! ENGINE */


void post_process_bones(ModelStruct* ms)
{
   int i, j, k;

   for (i = 0; i < ms->numsegments; i++)
   {
      for (j = 0; j < ms->segment[i].numBones; j++)
      {
         /* If the material and/or drawmode were not specified for the
          * bone, it inherits them from the segment.
          */
         if (ms->segment[i].bone[j].drawmode == -1)
            ms->segment[i].bone[j].drawmode = ms->segment[i].drawmode;
         if (ms->segment[i].bone[j].material == -1)
            ms->segment[i].bone[j].material = ms->segment[i].material;

         /* Multiply each bone vertex by the segment's scale factors. */
         for (k = 0; k < ms->segment[i].bone[j].num_vertices; k++)
         {
            ms->segment[i].bone[j].vertex[k].coord[XX] *= ms->segment[i].bone_scale[XX];
            ms->segment[i].bone[j].vertex[k].coord[YY] *= ms->segment[i].bone_scale[YY];
            ms->segment[i].bone[j].vertex[k].coord[ZZ] *= ms->segment[i].bone_scale[ZZ];
         }

         /* Multiply the bone's bounding box by the segment's scale factors. */
         ms->segment[i].bone[j].bc.x1 *= ms->segment[i].bone_scale[XX];
         ms->segment[i].bone[j].bc.x2 *= ms->segment[i].bone_scale[XX];
         ms->segment[i].bone[j].bc.y1 *= ms->segment[i].bone_scale[YY];
         ms->segment[i].bone[j].bc.y2 *= ms->segment[i].bone_scale[YY];
         ms->segment[i].bone[j].bc.z1 *= ms->segment[i].bone_scale[ZZ];
         ms->segment[i].bone[j].bc.z2 *= ms->segment[i].bone_scale[ZZ];
      }
   }
}


/* CHECK_DEFINITIONS: Checks to make sure that no joints, segments, or
 * functions were referenced but not defined in the joint file.
 */
ReturnCode check_definitions(ModelStruct* ms)
{
   int i, j, k;
   SBoolean any_errors = no;
   char buf[CHARBUFFER];

   if (ms->numsegments == 0)
   {
      error(none,"There were no segments defined in the joint file.");
      return code_bad;
   }

   for (i=0; i<ms->numsegments; i++)
   {
      if (ms->segment[i].defined == no)
      {
         (void)sprintf(errorbuffer,"Segment %s referenced but not defined.", ms->segment[i].name);
         error(none,errorbuffer);
         any_errors = yes;
      }
   }

   for (i=0; i<ms->numgencoords; i++)
   {
      if (ms->gencoord[i]->defined == no)
      {
         (void)sprintf(errorbuffer,"Generalized coordinate %s referenced but not defined.", ms->gencoord[i]->name);
         error(none,errorbuffer);
         any_errors = yes;
      }
   }

   for (i=0; i<ms->func_array_size; i++)
   {
      if (ms->function[i] && ms->function[i]->used == dpYes && ms->function[i]->status == dpFunctionUndefined)
      {
         (void)sprintf(errorbuffer,"Function f%d referenced but not defined.", ms->function[i]->usernum);
         error(none,errorbuffer);
         any_errors = yes;
      }
   }

   if (ms->numsegments > 1)
   {
      for (i=0; i<ms->numsegments; i++)
      {
         for (j=0; j<ms->numjoints; j++)
            if (ms->joint[j].from == i || ms->joint[j].to == i)
               break;
            if (j == ms->numjoints)
            {
               (void)sprintf(errorbuffer,"Segment %s not used in any joint.", ms->segment[i].name);
               error(none,errorbuffer);
               any_errors = yes;
            }
      }
   }

#if NO_LONGER_NECESSARY
   for (i=0; i<ms->numjoints; i++)
   {
      for (j=0; j<6; j++)
      {
         if (ms->joint[i].dofs[j].type == constant_dof)
            continue;
         gc = ms->joint[i].dofs[j].gencoord;
         fn = ms->joint[i].dofs[j].funcnum;
         if (ms->gencoord[gc]->clamped == no || ms->function[fn].defined == no)
            continue;
         if (ms->function[fn].x[0] > ms->gencoord[gc]->range.start ||
            ms->function[fn].x[ms->function[fn].numpoints-1] <
            ms->gencoord[gc]->range.end)
         {
            (void)sprintf(errorbuffer,"Function f%d does not cover full range of gencoord %s",
               ms->function[fn].usernum, ms->gencoord[gc]->name);
            error(none,errorbuffer);
            any_errors = yes;
         }
      }
   }
#endif

   check_gencoord_usage(ms,no);

   for (i=0; i<ms->dis.mat.num_materials; i++)
   {
      if (ms->dis.mat.materials[i].defined_yet == no)
      {
         (void)sprintf(errorbuffer,"Material %s referenced but not defined.", ms->dis.mat.materials[i].name);
         error(none,errorbuffer);
         any_errors = yes;
      }
   }

   /* check spring contact definitions, create list of points for each floor */
#if 0
   /* check that floor for each spring exists */
   for (i = 0; i < ms->numsegments; i++)
   {
      for (j = 0; j < ms->segment[i].numSpringPoints; j++)
      {
         SBoolean floor_defined = no;
         for (k = 0; k < ms->numsegments; k++)
         {
            if (ms->segment[k].springFloor)
            {
               if (STRINGS_ARE_EQUAL(ms->segment[k].springFloor->name, ms->segment[i].springPoint[j].floorName))
               {
                  floor_defined = yes;
               }
            }
         }
         if (!floor_defined)
         {
            (void)sprintf(errorbuffer,"Floor %s referenced but not defined.", ms->segment[i].springPoint[j].floorName);
            error(none,errorbuffer);
            any_errors = yes;
         }
      }
   }
#endif

   if (any_errors == yes)
      return code_bad;

   return code_fine;
}


void check_gencoord_usage(ModelStruct* ms, SBoolean change_visibility)
{
   int i, j, k;
   SBoolean used_in_model;

   ms->numunusedgencoords = 0;
   for (i=0; i<ms->numgencoords; i++)
   {
      used_in_model = no;

      for (j=0; j<ms->numjoints; j++)
      {
          for (k=0; k<6; k++)
         {
             if (ms->joint[j].dofs[k].type == constant_dof)
                continue;
             if (ms->joint[j].dofs[k].gencoord == ms->gencoord[i])
             {
                used_in_model = yes;
                break;
             }
          }
         if (used_in_model == yes)
         {
            /* If the gencoord was previously unused, turn its slider on */
            if (change_visibility == yes && ms->gencoord[i]->used_in_model == no)
            {
               ms->gencform.option[i].active = yes;
               ms->gencslider.sl[i].visible = yes;
               ms->gc_chpanel.checkbox[i].active = yes;
               ms->gc_lockPanel.checkbox[i].active = yes;
               ms->gencform.option[i].visible = yes;
               ms->gencslider.sl[i].visible = yes;
               ms->gc_chpanel.checkbox[i].visible = yes;
               ms->gc_lockPanel.checkbox[i].visible = yes;
#if ! ENGINE
#if ! OPENSMAC
               make_and_queue_simm_event(GENCOORD_CHANGED, ms->modelnum, &ms->gencoord[i], NULL, i, ZERO);
#endif
#endif
            }
            ms->gencoord[i]->used_in_model = yes;
            break;
         }
      }
      if (used_in_model == no)
      {
         ms->numunusedgencoords++;
         if (change_visibility)
         {
            /* If the gencoord was previously used, turn its slider off */
            if (ms->gencoord[i]->used_in_model == yes)
            {
               ms->gencform.option[i].active = no;
               ms->gencslider.sl[i].visible = no;
               ms->gc_chpanel.checkbox[i].active = no;
               ms->gc_lockPanel.checkbox[i].active = no;
               ms->gencform.option[i].visible = no;
               ms->gencslider.sl[i].visible = no;
               ms->gc_chpanel.checkbox[i].visible = no;
               ms->gc_lockPanel.checkbox[i].visible = no;
#if ! ENGINE
#if ! OPENSMAC
               make_and_queue_simm_event(GENCOORD_CHANGED, ms->modelnum, &ms->gencoord[i], NULL, i, ZERO);
#endif
#endif
            }
         }
         else
         {
            (void)sprintf(errorbuffer,"Gencoord %s not used in any joint.", ms->gencoord[i]->name);
            error(none,errorbuffer);
         }
         ms->gencoord[i]->used_in_model = no;
      }
   }
}


void find_ground_joint(ModelStruct* ms)
{
   // Find the number of the ground segment. If there is no ground segment,
   // make the zeroth segment ground.
   ms->ground_segment = enter_segment(ms, "ground", no);

   if (ms->ground_segment == -1)
      ms->ground_segment = 0;

   ms->initial_ground_segment = ms->ground_segment;

   /* Set the initial value of the current frame, which is the ground frame */
   ms->currentframe = ms->ground_segment;
}

SBoolean modelHasMuscles(ModelStruct* ms)
{
   if (ms->nummuscles > 0 || ms->default_muscle->name != NULL)
      return yes;

   return no;
}

static double initializeEmgWindowSize()
{
   const char* value = get_preference("EMG_SMOOTHING_WINDOW");

   if (value)
   {
      double v = atof(value);
      if (v > 0.0001 && v < 10.0)
         return v;
   }

   return 0.032;
}

static int initializeEmgSmoothingPasses()
{
   const char* value = get_preference("EMG_SMOOTHING_PASSES");

   if (value)
   {
      int i = atoi(value);
      if (i >= 0 && i < 50)
         return i;
   }

   return 1;
}

#if ! ENGINE
ReturnCode open_model_archive(char archiveFilename[], int* modelIndex)
{
   char jointFilename[CHARBUFFER], folder[CHARBUFFER], bones_folder[CHARBUFFER], cwdSave[CHARBUFFER], resFolder[CHARBUFFER], *buf = NULL;
   int result;
   ReturnCode rc = code_fine;

   _getcwd(cwdSave, CHARBUFFER);

   mstrcpy(&buf, glutGetTempFileName("smm"));
   strcpy(folder, buf);
   result = makeDir(folder);
   if (result != 0 && errno != EEXIST)
   {
      sprintf(errorbuffer, "Unable to create temporary folder %s\n", folder);
      error(none, errorbuffer);
      rc = code_bad;
      goto cleanup;
   }

   // Temporarily set the current working directory to the tmp
   // folder, so SIMM will find the motion files. After loading
   // the model, it should be set back to whatever it was (which
   // should be the folder containing the archive file.
   chdir(folder);

   sprintf(buffer, "Extracting model from archive %s...", archiveFilename);
   message(buffer, 0, DEFAULT_MESSAGE_X_OFFSET);

   // Make the archive
   strcpy(resFolder, get_preference("SIMM_FOLDER"));
   append_if_necessary(resFolder, DIR_SEP_CHAR);
   sprintf(buffer, "\"%s7za\" x -p%s -o\"%s\" -y \"%s\"", resFolder, archive_password, folder, archiveFilename);
   result = glutSystem(buffer);

   // The .jnt file should always be named model.jnt. Try to open
   // a file by that name and print an error about an invalid
   // archive if it is not found.
   sprintf(jointFilename, "%s\\model.jnt", folder);
   if (file_exists(jointFilename) == no)
   {
      sprintf(buffer, "Error opening model archive %s. Archive is invalid or corrupted.", archiveFilename);
      error(none, buffer);
      rc = code_bad;
      goto cleanup;
   }

   // Load the model
   rc = add_model(jointFilename, NULL, -1, modelIndex, no);
   if (rc == code_bad)
   {
      sprintf(buffer, "Error reading model from archive %s.", archiveFilename);
      error(none, buffer);
      goto cleanup;
   }

   if (STRINGS_ARE_EQUAL(gModel[*modelIndex]->bonepathname, "."))
      strcpy(bones_folder, folder);
   else
      sprintf(bones_folder, "%s\\%s", folder, gModel[*modelIndex]->bonepathname);

   sprintf(buffer, "Extracting model from archive %s... Done.", archiveFilename);
   message(buffer, OVERWRITE_LAST_LINE, DEFAULT_MESSAGE_X_OFFSET);

cleanup:
   FREE_IFNOTNULL(buf);

   // Reset the current working directory.
   chdir(cwdSave);

   // Delete the files in the temporary folder
   if (1)
   {
      OSVERSIONINFO osvi;
      osvi.dwOSVersionInfoSize = sizeof(OSVERSIONINFO);
      GetVersionEx(&osvi);
      if (osvi.dwMajorVersion < 5)                      // Windows 95, 98, Me, NT
         sprintf(buffer, "deltree /y \"%s\"", folder);
      else                                              // Windows 2000, XP, Vista, 7
         sprintf(buffer, "rmdir /s /q \"%s\"", folder);
      result = glutSystem(buffer);
   }

   return rc;
}
#endif
