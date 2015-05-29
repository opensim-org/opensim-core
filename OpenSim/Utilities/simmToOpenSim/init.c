/*******************************************************************************

   INIT.C

   Author: Peter Loan

   Date: 13-APR-89

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Description: This file contains routines that initialize
      structures, set initial program parameter values, or
      reset values of a structure.

   Routines:
      initialize           : called at startup, does basic initialization
      init_model           : sets initial values of some model structures
      init_segment         : initializes a segment structure
      init_model_display   : sets initial values of a model display structure
      initplot             : sets initial values of a plot display structure
      init_muscle          : sets initial values of muscle structure elements
      init_gencoords       : sets initial gencoord structure values
      set_gencoord_info    : records some info about each gencoord
      init_joint            : sets initial values in the joint structure
      make_confirm_menu    : makes the menu used in the Confirm window

*******************************************************************************/

#include <direct.h>

#include "universal.h"
#include "globals.h"
#include "functions.h"
#include "wefunctions.h"


/*************** DEFINES (for this file only) *********************************/
#define EVENT_QUEUE_SIZE 50
#define RAMPSIZE 32
#define INIT_WORLD_ARRAY_SIZE 10

/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/
static void make_confirm_menu(void);
static char* confirmstr[] = {
"yes","cancel"
};


/*************** EXTERNED VARIABLES (declared in another file) ****************/


/*************** GLOBAL VARIABLES (for use in a few files) ****************/

char* expirationMessage = NULL;



/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/


#if ! ENGINE

/* INITIALIZE: the main initializing routine that is called once at startup.
 * It makes the program window, queues the keyboard and mouse buttons, and
 * calls routines to make the color map.
 */

void initialize(void)
{
   int i;
   const char* p;

   add_preprocessor_option(yes, "-I%s", get_preference("RESOURCES_FOLDER"));

   root.numwindows = 0;
   root.messages.line = NULL;

   root.gldesc.max_screen_x = glutGet(GLUT_SCREEN_WIDTH);
   if (root.gldesc.max_screen_x <= 0)
      root.gldesc.max_screen_x = 1024;
   root.gldesc.max_screen_y = glutGet(GLUT_SCREEN_HEIGHT);
   if (root.gldesc.max_screen_y <= 0)
      root.gldesc.max_screen_y = 768;

   root.gldesc.max_screen_x -= 10;
   root.gldesc.max_screen_y -= 30;

   if (is_preference_on(get_preference("MULTIPLE_SCREENS")) == no)
      glueSetConstrainedWindows(yes);
   else
      glueSetConstrainedWindows(no);

   if (is_preference_on(get_preference("MULTIPLE_SCREENS")) == no)
       open_main_window();

   init_global_lighting();

   if (make_selectors() == code_bad)
      error(exit_program,tool_message);

   /* Initialize some root structure variables */
   root.nummodels = 0;
   root.numplots = 0;
   root.numtools = 0;
   root.currentwindow = NO_WINDOW;
   root.modelcount = 0;
   root.confirm_window_open = no;

   root.gfont.defaultfont = SIMM_DEFAULT_FONT;
   root.gfont.largefont   = SIMM_LARGE_FONT;
   root.gfont.smallfont   = SIMM_SMALL_FONT;
   root.gfont.italics     = SIMM_ITALIC_FONT;
   root.gfont.bold        = SIMM_BOLD_FONT;

   /* Set all the plot pointers to NULL */
   for (i=0; i<PLOTBUFFER; i++)
      gPlot[i] = NULL;

   /* Set all the model pointers to NULL */
   for (i=0; i<MODELBUFFER; i++)
      gModel[i] = NULL;

   /* Set all the tool structures to unused */
   for (i=0; i<TOOLBUFFER; i++)
   {
      tool[i].used = no;
      tool[i].name = NULL;
   }

   /* Make the [initially empty] model menu */
   root.modelmenu = -1;
   root.plotmenu = -1;

   root.gravityMenu = glueCreateMenu("Gravity");
   glueAddMenuEntryWithValue(root.gravityMenu, " +X", smX);
   glueAddMenuEntryWithValue(root.gravityMenu, " -X", smNegX);
   glueAddMenuEntryWithValue(root.gravityMenu, " +Y", smY);
   glueAddMenuEntryWithValue(root.gravityMenu, " -Y", smNegY);
   glueAddMenuEntryWithValue(root.gravityMenu, " +Z", smZ);
   glueAddMenuEntryWithValue(root.gravityMenu, " -Z", smNegZ);
   glueAddMenuEntryWithValue(root.gravityMenu, "none", smNoAlign);

   /* Init the command list */
   root.num_commands = 0;
   for (i=0; i<COMMAND_BUFFER; i++)
      root.command[i] = NULL;

   updatemodelmenu();

   /* Call routines to make the color map */
   init_color_database();

   /* Make the confirm-action menu */
   make_confirm_menu();

   /* Malloc the SIMM event queue */
   root.simm_event_queue = (SimmEvent*)simm_malloc(EVENT_QUEUE_SIZE*sizeof(SimmEvent));
   if (root.simm_event_queue == NULL)
      error(exit_program,tool_message);

   root.event_queue_length = EVENT_QUEUE_SIZE;
   root.events_in_queue = 0;
}


void open_main_window()
{
   int xtmp, ytmp;
   WindowParams bwin;
   WinUnion wun;

   init_main_menu();   /* the main menu must be created *before* the SIMM window */

   glueSetWindowPlacement(GLUT_NORMAL_WINDOW, GLUE_CENTER,
              root.gldesc.max_screen_x,
              root.gldesc.max_screen_y, 0, 0, NULL);

   bwin.id = glueOpenWindow("simm",no,GLUE_NO_WINDOW_FLAGS);

   glueSetWindowMinSize(950,700);

#if SIMM_DEMO_VERSION
   sprintf(buffer,"%s - Tryout Version (%s)", program_name, program_version);
#elif SIMM_VIEWER
   sprintf(buffer,"%s (%s)", program_name, program_version);
#elif defined WIN32
   strcpy(buffer, program_with_version);
   
   if (is_in_demo_mode())
      strcat(buffer, " - Demo Mode");
#else
   sprintf(buffer,"%s: %s, %s  %s", program_name, program_full_name,
       program_version, copyright_notice);
#endif

   mstrcpy(&bwin.name,buffer);
   glutSetIconTitle(program_name);
   glutSetWindowTitle(bwin.name);

   wun.tool = NULL;
   root.basewindow = add_window(&bwin,&wun,NOTYPE,-1,no,display_background,
                update_background,background);
   if (root.basewindow == -1)
      error(exit_program,tool_message);

   glutSetWindowData(bwin.id, root.basewindow);
   glueGetWindowOrigin(&root.surgwin.x1,&root.surgwin.y1);
   glueGetWindowSize(&xtmp,&ytmp);
   root.surgwin.x2 = root.surgwin.x1 + xtmp;
   root.surgwin.y2 = root.surgwin.y1 + ytmp;
}

#endif /* ENGINE */


ReturnCode init_scene(Scene* sc)
{
   int i;
   // sc->scene_num is preset; store it here so you can restore it after memset().
   int scene_number = sc->scene_num;
   const char* p = NULL;

   memset(sc, 0, sizeof(Scene));

   sc->scene_num = scene_number;
   sc->trackball = yes;
   sc->camera_segment_model = NULL;
   sc->camera_segment = -1;

   sc->windowX1 = -1;
   sc->windowY1 = -1;
   sc->windowHeight = 400;
   sc->windowWidth = 400;

   reset_4x4matrix(sc->transform_matrix);

   for (i=0; i<3; i++)
   {
      sc->background_color[i] = 0.2f;
      sc->crosshairs_color[i] = 1.0f;
   }

   sc->fov_angle = 40.0;
   sc->y_zoom_constant = tan((double)sc->fov_angle*0.5*DTOR);
   sc->near_clip_plane = 0.0002; /* reset in size_model */
   sc->far_clip_plane = 12.3; /* reset in size_model */

   sc->snapshot_mode           = SNAPSHOT_INACTIVE;
   sc->snapshot_counter        = 1;
   sc->snapshot_file_suffix    = NULL;
   sc->snapshot_include_depth  = no;

   sc->movie_mode = MAKEMOVIE_INACTIVE;
   sc->movie_file = NULL;

   sc->trackball = yes;
   sc->camera_segment = -1;

   sc->windowX1 = -1;
   sc->windowY1 = -1;
   sc->windowHeight = 400;
   sc->windowWidth = 400;

   return code_fine;
}


/* INITMODEL: this guy initializes much of the model structure.
 */
ReturnCode init_model(ModelStruct* ms)
{
   int i;
   const char *p;
    // ms->modelnum is preset; store it here so you can restore it after memset().
   int model_number = ms->modelnum;

   memset(ms, 0, sizeof(ModelStruct));

   ms->modelnum = model_number;
   ms->is_demo_model = no;
   ms->useIK = yes;
   ms->defaultGCApproved = yes;
   ms->defaultLoopsOK = yes;
   ms->defaultConstraintsOK = yes;
   ms->constraintsOK = yes;
   ms->loopsOK = yes;
   ms->muscle_array_size = MUSCLE_ARRAY_INCREMENT;
   ms->ligament_array_size = MUSCLE_ARRAY_INCREMENT;
   ms->muscgroup_array_size = MUSCGROUP_ARRAY_INCREMENT;
   ms->world_array_size = INIT_WORLD_ARRAY_SIZE;
   ms->genc_array_size = GENC_ARRAY_INCREMENT;
   ms->segment_array_size = SEGMENT_ARRAY_INCREMENT;
   ms->joint_array_size = JOINT_ARRAY_INCREMENT;
   ms->func_array_size = FUNC_ARRAY_INCREMENT;
   ms->specified_min_thickness = yes;
   ms->specified_max_thickness = yes;
   ms->dynamics_ready = no;
   ms->max_diagonal_needs_recalc = yes;
   ms->GEFuncOK = no;
   ms->marker_visibility = yes;
   ms->marker_radius = DEFAULT_MARKER_RADIUS;
   ms->loop_tolerance = DEFAULT_LOOP_TOLERANCE;
   ms->loop_weight = DEFAULT_LOOP_WEIGHT;
   ms->solver.accuracy = DEFAULT_SOLVER_ACCURACY;
   ms->solver.method = smLevenbergMarquart;
   ms->solver.max_iterations = 100;
   ms->solver.joint_limits = smYes;
   ms->solver.orient_body = smNo;
   ms->solver.fg_contact = smNo;
   ms->global_show_masscenter = no;
   ms->global_show_inertia = no;

   ms->gravity = smNegY;

   ms->functionMenu = 0;

   ms->motion_array_size = MOTION_ARRAY_INCREMENT;
   ms->motion = (MotionSequence**)simm_malloc(ms->motion_array_size * sizeof(MotionSequence*));
   if (ms->motion == NULL)
   {
      error(none,"Not enough memory to add another model.");
      return code_bad;
   }
   else
   {
      for (i = 0; i < ms->motion_array_size; i++)
         ms->motion[i] = NULL;
   }


   for (i=0; i<100; i++)
      ms->motionfilename[i] = NULL;
   ms->num_motion_files = 0;

   ms->joint = (JointStruct*)simm_malloc(ms->joint_array_size*sizeof(JointStruct));
   ms->segment = (SegmentStruct*)simm_malloc(ms->segment_array_size*sizeof(SegmentStruct));
   ms->muscgroup = (MuscleGroup*)simm_malloc(ms->muscgroup_array_size*sizeof(MuscleGroup));
   ms->seggroup = NULL;
   ms->gencgroup = NULL;
   ms->gencoord = (GeneralizedCoord**)simm_calloc(ms->genc_array_size, sizeof(GeneralizedCoord*));

   ms->muscle = (dpMuscleStruct**)simm_calloc(ms->muscle_array_size, sizeof(dpMuscleStruct*));

   ms->ligament = (LigamentStruct*)simm_malloc(ms->ligament_array_size*sizeof(LigamentStruct));

   ms->function = (dpFunction**)simm_calloc(ms->func_array_size, sizeof(dpFunction*));

   ms->save.function = (dpFunction**)simm_calloc(ms->func_array_size, sizeof(dpFunction*));

   ms->worldobj = (WorldObject*)simm_malloc(ms->world_array_size*sizeof(WorldObject));

   if (ms->joint == NULL || ms->segment == NULL || ms->muscgroup == NULL ||
       ms->gencoord == NULL || ms->function == NULL ||
       ms->save.function == NULL || ms->worldobj == NULL)
   {
      error(none,"Not enough memory to add another model.");
      return code_bad;
   }

   ms->num_wrap_objects = 0;
   ms->wrap_object_array_size = WRAP_OBJECT_ARRAY_INCREMENT;
   ms->wrapobj = (dpWrapObject**)simm_malloc(ms->wrap_object_array_size*sizeof(dpWrapObject*));
   if (ms->wrapobj == NULL)
   {
      error(none,"Not enough memory to add another wrap object.");
      return code_bad;
   }

//   ms->constraint_tolerance = DEFAULT_CONSTRAINT_TOLERANCE;
   ms->num_constraint_objects = 0;
   ms->constraint_object_array_size = CONSTRAINT_OBJECT_ARRAY_INCREMENT;
   ms->constraintobj = (ConstraintObject*)simm_malloc(ms->constraint_object_array_size*sizeof(ConstraintObject));
   if (ms->constraintobj == NULL)
   {
      error(none,"Not enough memory to add another constraint.");
      return code_bad;
   }

   ms->num_deformities = 0;
   ms->deformity_array_size = DEFORMITY_ARRAY_INCREMENT;
   ms->deformity = (Deformity*) simm_malloc(ms->deformity_array_size * sizeof(Deformity));
   if (ms->deformity == NULL)
   {
      error(none,"Not enough memory to add another deformity object.");
      return code_bad;
   }

   init_materials(ms);

   ms->num_motion_objects = 0;
   ms->motion_objects = NULL;

#if ! ENGINE || OPENSMAC
   add_default_motion_objects(ms);
#endif

   /* This part of the display structure must be initialized here because
    * it can be changed when reading in the joints file, which happens before
    * the display structure is initialized.
    */
   ms->dis.num_file_views = 0;
//   ms->dis.muscle_array_size = 0;//dkb = MUSCLE_ARRAY_INCREMENT;
   ms->dis.muscle_array_size = MUSCLE_ARRAY_INCREMENT;
   
   ms->dis.fast_muscle_drawing = no;
   
#if ! ENGINE
   ms->dis.fast_muscle_drawing = is_preference_on(get_preference("FASTER_MUSCLE_DRAWING"));

#if INCLUDE_MSL_LENGTH_COLOR
   ms->dis.muscle_color_factor = 0.0;
#endif
   
   for (i=0; i<MAXSAVEDVIEWS; i++)
   {
      ms->dis.view_used[i] = no;
      ms->dis.view_name[i] = NULL;
   }

   for (i=0; i<3; i++)
   {
      ms->dis.background_color[i] = 0.2f;
      ms->dis.vertex_label_color[i] = 0.0f;
      ms->dis.rotation_axes_color[i] = 1.0f;
      ms->dis.crosshairs_color[i] = 1.0f;
   }
   ms->dis.rotation_axes_color[2] = 0.0f;
   ms->dis.vertex_label_color[0] = 1.0f;

   ms->dis.background_color_spec = no;
   ms->dis.vertex_label_color_spec = no;
   ms->dis.rotation_axes_color_spec = no;
   ms->dis.crosshairs_color_spec = no;

   ms->modelLock = glutNewMutex();
   ms->realtimeState = rtNotConnected;

#endif /* ENGINE */

   ms->gencoordmenu = -1;
   ms->gencoordmenu2 = -1;
   ms->gencoord_group_menu = -1;
   ms->xvarmenu = -1;
   ms->momentgencmenu = -1;
   ms->momentarmgencmenu = -1;
   ms->momentarmnumgencmenu = -1;
   ms->maxmomentgencmenu = -1;

   return code_fine;
}


void init_segment(ModelStruct* ms, SegmentStruct* seg)
{
   int i, j;

   seg->defined = no;
   seg->numBones = 0;
   seg->boneArraySize = 0;
   seg->shadow = no;
   seg->shadow_scale[0] = seg->shadow_scale[1] = seg->shadow_scale[2] = 1.0;
   seg->shadow_trans[0] = seg->shadow_trans[1] = seg->shadow_trans[2] = 0.0;
   seg->shadow_color.rgb[RD] = 0.1f;
   seg->shadow_color.rgb[GR] = 0.1f;
   seg->shadow_color.rgb[BL] = 0.1f;
   seg->shadow_color_spec = no;
   seg->drawmode = gouraud_shading;
   seg->material = ms->dis.mat.default_bone_material;
   seg->mass = 0.0;
   seg->bone = NULL;
   seg->draw_axes = no;
   seg->axis_length = 0.0;

   seg->ground_condition = invalid;

   seg->numMarkers = 0;
   seg->markerArraySize = 0;
   seg->marker = NULL;

#if INCLUDE_BONE_EDITOR_EXTRAS
   seg->pts_file = NULL;
   seg->num_raw = 0;
   seg->raw_vertices = NULL;
#endif

   seg->bone_scale[0] = 1.0;
   seg->bone_scale[1] = 1.0;
   seg->bone_scale[2] = 1.0;

   seg->htr_o[0] = seg->htr_o[1] = seg->htr_o[2] = UNDEFINED_DOUBLE;
   seg->htr_x[0] = seg->htr_x[1] = seg->htr_x[2] = UNDEFINED_DOUBLE;
   seg->htr_y[0] = seg->htr_y[1] = seg->htr_y[2] = UNDEFINED_DOUBLE;

   for (i=0; i<3; i++)
   {
      for (j=0; j<3; j++)
     seg->inertia[i][j] = 0.0;
      seg->masscenter[i] = 0.0;
   }

   seg->mass_specified = no;
   seg->inertia_specified = no;
   seg->masscenter_specified = no;
#if INCLUDE_MOCAP_MODULE
   seg->lengthstartend_specified = no;
#endif
   seg->show_masscenter = ms->global_show_masscenter;//no;
   seg->show_inertia = ms->global_show_inertia;//no;
   
   seg->numgroups = 0;
   seg->group = NULL;

   seg->springFloor = NULL;
   seg->numSpringPoints = 0;
   seg->springPointArraySize = 0;
   seg->springPoint = NULL;

   seg->numContactObjects = 0;
   seg->contactObjectArraySize = 0;
   seg->contactObject = NULL;

   seg->forceMatte = NULL;

   seg->num_deforms = 0;
   seg->deform_obj_array_size = 0;
   seg->deform = NULL;

#if INCLUDE_MOCAP_MODULE
   for (i = 0; i < 3; i++)
   {
      seg->lengthstart[i] = 0.0;
      seg->lengthend[i] = 0.0;
   }
   seg->gait_scale_segment = NULL;
   seg->gait_scale_factor[0] = 1.0;
   seg->gait_scale_factor[1] = 1.0;
   seg->gait_scale_factor[2] = 1.0;
   seg->mocap_segment = NULL;
   seg->mocap_scaling_method = INHERIT_SCALE;
   seg->mocap_scale_chain_end1 = NULL;
   seg->mocap_scale_chain_end2 = NULL;
   identity_matrix(seg->mocap_adjustment_xform);
#endif
}


/* INIT_MODEL_DISPLAY: this routine initializes the model display structure, which
 * holds the viewport and ortho information as well as the current viewing
 * transformations.
 */

ReturnCode init_model_display(ModelStruct* ms)
{
   int i;

   ms->dis.motion_speed = DEFAULT_MOTION_SPEED;
   ms->dis.default_view = 0; // the default camera will be cam0 unless overridden by user
   ms->dis.show_highlighted_polygon = no;
   ms->dis.show_selected_coords = no;
   ms->dis.show_all_muscpts = no;
   ms->dis.show_shadow = no;
   ms->dis.applied_motion = NULL;
   ms->dis.current_motion = NULL;
   ms->dis.hpoly.segmentnum = -1;
   ms->dis.hpoly.bonenum = -1;
   ms->dis.hpoly.polynum = -1;
   ms->dis.continuous_motion = no;
   ms->dis.display_motion_info = yes;

   ms->dis.devs = (int*)simm_malloc(ms->numgencoords*2*sizeof(int));
   ms->dis.dev_values = (int*)simm_malloc(ms->numgencoords*2*sizeof(int));
   if (ms->dis.devs == NULL || ms->dis.dev_values == NULL)
      return code_bad;

   for (i=0; i<ms->numgencoords; i++)
   {
      ms->dis.devs[i*2] = ms->gencoord[i]->keys[0];
      ms->dis.devs[i*2+1] = ms->gencoord[i]->keys[1];
   }
   ms->dis.numdevs = ms->numgencoords*2;

   ms->dis.nummuscleson = ms->nummuscles;
   ms->dis.muscle_array_size = ms->muscle_array_size;
   if (ms->dis.muscle_array_size > 0)
   {
      ms->dis.muscleson = (int*)simm_malloc(ms->dis.muscle_array_size*sizeof(int));
      if (ms->dis.muscleson == NULL)
         return code_bad;
   }

   ms->dis.muscle_cylinder_id = -1;

   for (i=0; i<ms->dis.muscle_array_size; i++)
   {
      if (i < ms->nummuscles)
         ms->dis.muscleson[i] = ms->muscle[i]->display;
      else
         ms->dis.muscleson[i] = 0;
   }
   
   for (i=0; i<COLUMNBUFFER; i++)
      ms->dis.menucolumns[i] = EMPTY;

   for (i=0; i<ms->numgroups; i++)
   {
      ms->dis.mgroup[i].state = off;
      ms->dis.mgroup[i].xo = -1;
      ms->dis.mgroup[i].yo = -1;
   }

   ms->dis.view_menu = 0;
   ms->dis.maindrawmodemenu = 0;
   ms->dis.allsegsdrawmodemenu = 0;
   ms->dis.allligsdrawmodemenu = 0;
   ms->dis.allworlddrawmodemenu = 0;
   ms->dis.alldrawmodemenu = 0;
   ms->dis.eachsegdrawmodemenu = 0;

#if ! ENGINE
   ms->dis.display_motion_info = is_preference_on(get_preference("DISPLAY_MOTION_INFO"));
#endif

   return code_fine;
}

#if ! ENGINE

/* INITPLOT: this guy initializes the plot structure, which contains viewport
 * and ortho values as well as the set of curve structures for that plot.
 * First it calls a routine to set the viewport and window-resize box
 * variables.
 */

ReturnCode initplot(int plotnum)
{
   gPlot[plotnum]->plotnum = plotnum;
   gPlot[plotnum]->numcurves = 0;
   gPlot[plotnum]->zoomed_yet = no;
   gPlot[plotnum]->numpoints = 0;
   gPlot[plotnum]->xname_len = 0;
   gPlot[plotnum]->yname_len = 0;
   gPlot[plotnum]->title_len = 256;
   gPlot[plotnum]->title = (char*)simm_malloc(gPlot[plotnum]->title_len);
   gPlot[plotnum]->xname = NULL;
   gPlot[plotnum]->yname = NULL;
   gPlot[plotnum]->cursor_modelPtr = NULL;
   gPlot[plotnum]->cursor_motion = NULL;
   gPlot[plotnum]->show_cursor = yes;
   gPlot[plotnum]->show_events = yes;
   gPlot[plotnum]->needs_bounding = no;
   gPlot[plotnum]->num_file_events = 0;
   gPlot[plotnum]->file_event = NULL;

   if (gPlot[plotnum]->title == NULL)
      return code_bad;

   NULLIFY_STRING(gPlot[plotnum]->title);

   return code_fine;
}

#endif /* ! ENGINE */

/* INIT_MUSCLE: this routine initializes much of the muscle structure. Before
 * a given muscle is read-in from an input file, each of the pointers to its
 * muscle-specific parameters is set to point into the default-muscle structure.
 * Thus if a certain parameter is not specified for a muscle (e.g. force-length
 * curve), the default-muscle's parameter is used. This cuts down significantly
 * on the amount of information that must be supplied in a muscle file since
 * most muscles share the same active_force_len_func, tendon_force_len_func,
 * passive_force_len_func, and force_vel_func.
 */
ReturnCode init_muscle(ModelStruct* model, dpMuscleStruct* muscle, dpMuscleStruct* default_muscle)
{
   int i;

   memset(muscle, 0, sizeof(dpMuscleStruct));

   muscle->display = default_muscle->display;
   muscle->output = yes;
   muscle->selected = no;
   muscle->numgroups = default_muscle->numgroups;
   muscle->group = default_muscle->group;
   muscle->max_isometric_force = default_muscle->max_isometric_force;
   muscle->pennation_angle = default_muscle->pennation_angle;
   muscle->optimal_fiber_length = default_muscle->optimal_fiber_length;
   muscle->resting_tendon_length = default_muscle->resting_tendon_length;
   muscle->min_thickness = default_muscle->min_thickness;
   muscle->max_thickness = default_muscle->max_thickness;
   muscle->min_material = default_muscle->min_material;
   muscle->max_material = default_muscle->max_material;
   muscle->max_contraction_vel = default_muscle->max_contraction_vel;
   muscle->force_vel_func = default_muscle->force_vel_func;

   if (init_dynamic_param_array(muscle, default_muscle) == code_bad)
      return code_bad;

   muscle->muscle_model_index = default_muscle->muscle_model_index;
   muscle->excitation_func = default_muscle->excitation_func;
   muscle->excitation_abscissa = default_muscle->excitation_abscissa;

   muscle->nummomentarms = model->numgencoords;
   muscle->momentarms = (double *)simm_malloc(muscle->nummomentarms*sizeof(double));
   if (muscle->momentarms == NULL)
      return code_bad;

   for (i=0; i<muscle->nummomentarms; i++)
      muscle->momentarms[i] = 0.0;
   muscle->dynamic_activation = 1.0;
   muscle->tendon_force_len_func = default_muscle->tendon_force_len_func;
   muscle->active_force_len_func = default_muscle->active_force_len_func;
   muscle->passive_force_len_func = default_muscle->passive_force_len_func;

   muscle->wrap_calced = no;

   return code_fine;
}

ReturnCode initMusclePath(dpMusclePathStruct *musclepoints)
{
   musclepoints->num_orig_points = 0;
   musclepoints->mp_orig_array_size = MUSCLEPOINT_ARRAY_INCREMENT;
   musclepoints->mp_orig = (dpMusclePoint *)simm_malloc(musclepoints->mp_orig_array_size * sizeof(dpMusclePoint));
   if (musclepoints->mp_orig == NULL)
      return code_bad;

   musclepoints->num_points = 0;
   musclepoints->mp_array_size = 0;
   musclepoints->mp = NULL;

   return code_fine;
}


ReturnCode init_musclepoint(dpMusclePoint *mp)
{
   int i;

   mp->segment = -1;
   mp->selected = dpNo;
   mp->isMovingPoint = dpNo;
   mp->isVia = dpNo;
   mp->is_auto_wrap_point = dpNo;
   mp->viaRange.gencoord = NULL;
   mp->viaRange.start = UNDEFINED_DOUBLE;
   mp->viaRange.end = UNDEFINED_DOUBLE;
   mp->num_wrap_pts = 0;
   mp->wrap_pts = NULL;
   mp->wrap_distance = 0;
   for (i = 0; i < 3; i++)
   {
      mp->point[i] = ERROR_DOUBLE;
      mp->ground_pt[i] = ERROR_DOUBLE;
      mp->undeformed_point[i] = ERROR_DOUBLE;
      mp->function[i] = NULL;
      mp->gencoord[i] = NULL;
      mp->normal[i] = 0.0;
   }

   return code_fine;
}


/* INIT_GENCOORD: this routine initializes a gencoord structure before a gencoord is
 * read from an input file.
 */
void init_gencoord(GeneralizedCoord* gencoord)
{
   memset(gencoord, 0, sizeof(GeneralizedCoord));

   gencoord->type = rotation_gencoord;
   gencoord->defined = no;
   gencoord->wrap = no;

   gencoord->keys[0] = gencoord->keys[1] = null_key;

   gencoord->default_value_specified = no;
   gencoord->clamped = yes;
   gencoord->clamped_save = gencoord->clamped;
   gencoord->locked = no;
   gencoord->locked_save = gencoord->locked;
   gencoord->range.end = 90.0;
   gencoord->restraintFuncActive = yes;
   gencoord->used_in_model = no;
   gencoord->used_in_loop = no;
   gencoord->used_in_constraint = no;
   gencoord->slider_visible = yes;

   gencoord->restraint_sdcode_num = -1;
   gencoord->min_restraint_sdcode_num = -1;
   gencoord->max_restraint_sdcode_num = -1;

#if INCLUDE_MOCAP_MODULE
   gencoord->mocap_seg_index = -1;
   gencoord->mocap_column = -1;
#endif
}


/* INIT_GENCOORDS: this function sets some of the initial values in the gencoord
 * structs. It must be called after all the joints have been read in because it
 * mallocs an array of size numjoints for each gencoord.
 */
ReturnCode init_gencoords(ModelStruct* ms)
{
   int i;

   for (i=0; i<ms->numgencoords; i++)
   {
      if (ms->gencoord[i]->default_value >= ms->gencoord[i]->range.start &&
         ms->gencoord[i]->default_value <= ms->gencoord[i]->range.end)
         ms->gencoord[i]->value = ms->gencoord[i]->default_value;
      else if (ms->gencoord[i]->range.start > 0.0 ||
         ms->gencoord[i]->range.end < 0.0)
         ms->gencoord[i]->value = ms->gencoord[i]->range.start;
      else
         ms->gencoord[i]->value = 0.0;

      ms->gencoord[i]->velocity = 0.0;

#if ! ENGINE
      storeDoubleInForm(&ms->gencform.option[i], ms->gencoord[i]->value, 3);
#endif

      ms->gencoord[i]->numjoints = 0;
      ms->gencoord[i]->jointnum = (int*)simm_malloc(ms->numjoints*sizeof(int));
      if (ms->gencoord[i]->jointnum == NULL)
         return code_bad;
   }

   return code_fine;
}


/* SET_GENCOORD_INFO: This routine records some information about the
 * gencoords. It first makes a list of the joints that each gencoord
 * appears in, then sets the type (rotation, translation) of the gencoord.
 */
void set_gencoord_info(ModelStruct* ms)
{
   int i, j, k, en;
   DofStruct* adof;

   /* For each joint, go through the six dofs (translations and rotations)
    * and list the gencoords that appear in the functions that define these
    * dofs. Then add this joint number to the joint list for each of these
    * gencoords. What you have at the end is a joint-dependency list for
    * each gencoord. That is, with each gencoord is a list of the joints
    * which have that gencoord in one or more of its dof functions. Thus
    * when you change the value of one gencoord, you know which joint
    * transformations you have to recalculate.
    * The loop over k checks to make sure you don't enter a joint twice in
    * a gencoord's list of joints. If a joint has two or more dofs which use
    * the same gencoord, you need to watch out for this. So as you scan
    * the dofs for each joint and find a gencoord, scan the previous dofs
    * again to see if one of them uses the same gencoord, which means you
    * don't need to [re-]tell the gencoord about the joint.
    */
   for (i=0; i<ms->numgencoords; i++)
      ms->gencoord[i]->numjoints = 0;

   for (i=0; i<ms->numjoints; i++)
   {
      for (j=0; j<6; j++)
      {
         adof = &ms->joint[i].dofs[j];
         if (adof->type == function_dof)
         {
            for (k=0; k<j; k++)
               if (adof->gencoord == ms->joint[i].dofs[k].gencoord)
                  break;
               if (k == j)
               {
                  en = adof->gencoord->numjoints++;
                  adof->gencoord->jointnum[en] = i;
               }
         }
      }
   }

   for (i = 0; i < ms->numgencoords; i++)
      determine_gencoord_type(ms, ms->gencoord[i]);
}


/* This function tries to determine whether each gencoord is primarily rotational
 * or translational. This information is needed for two reasons:
 * 1. to know how big to set the step size for the arrow buttons on the slider for
 *    that gencoord in the Model Viewer (0.001 for translations, 1.0 for rotations).
 * 2. to know how to interpret the moment arm values w.r.t. that gencoord. If the
 *    gencoord maps directly to a rotational dof, then the gencoord's units need to
 *    be treated as degrees, and thus converted to radians when calculating moment arms.
 *    If the gencoord maps directly to a translational dof, then its units are meters
 *    and no conversion is needed. If the gencoord maps directly to a dof of each type,
 *    then it really is an abstract quantity, and interpreting the moment arm in
 *    real-world units is problematic. So just find the first dof that maps directly
 *    to it and go with that one.
 * To determine whether a gencoord is rotational or translational, find the first
 * dof that maps directly to the gencoord (has a function with two points and slope
 * equal to 1.0 or -1.0). If there is no such function, then just look at the dofs
 * that the gencoord is used in. If they are all translational, then the gencoord is
 * translational. If one or more is rotational, then the gencoord is rotational.
 */
void determine_gencoord_type(ModelStruct* model, GeneralizedCoord* gencoord)
{
   int j, k, dofnum, jointnum;
   GencoordType gencoord_type;

   if (mark_unconstrained_dof(model, gencoord, &jointnum, &dofnum, NULL))
   {
      if (dofnum == R1 || dofnum == R2 || dofnum == R3)
         gencoord->type = rotation_gencoord;
      else if (dofnum == TX || dofnum == TY || dofnum == TZ)
         gencoord->type = translation_gencoord;
   }
   else
   {
      gencoord_type = translation_gencoord;
      for (j = 0; j < gencoord->numjoints; j++)
      {
         jointnum = gencoord->jointnum[j];
         for (k = R1; k <= R3; k++)
         {
            if (model->joint[jointnum].dofs[k].type == function_dof &&
                model->joint[jointnum].dofs[k].gencoord == gencoord)
            {
               gencoord_type = rotation_gencoord;
               j = gencoord->numjoints; // to break out of outer loop
               break;
            }
         }
      }
      gencoord->type = gencoord_type;
   }
}


int mark_unconstrained_dof(ModelStruct* ms, GeneralizedCoord* gencoord, int* jnt, int* dof, SBoolean* constrained)
{
   int i, j;
   double slope;
   dpFunction* func;

   *jnt = *dof = -1;

   /* This function looks through all the dofs in all the joints which are a
    * function of the specified gencoord, and tries to find one which should
    * be treated as the unconstrained dof. If the dof has a function with two
    * points, and the slope of the function is 1.0 or -1.0, then it is a good
    * match. The dof is marked as unconstrained, and the function returns.
    * If no such dof is found, the function returns an error. If there are
    * multiple dofs which meet these criteria, the first one is treated as the
    * unconstrained one, and the others will end up constrained (which will
    * create a correct model).
    * JPL 8/2/04: this function has been updated to also check to make sure
    * that the 2-point, slope=1 function also passes through zero.
     * JPL 3/6/06: this function appeared to have a bug which prevented
     * the negative-slope case from getting used, but it turns out that
     * the code for that case is incorrect. So negative-slope functions
     * are not allowed, and will return an error when attempting to save
     * dynamics.
    */
   for (i = 0; i < ms->numjoints; i++)
   {
      for (j = 0; j < 6; j++)
      {
          if (ms->joint[i].dofs[j].type == function_dof &&
              ms->joint[i].dofs[j].gencoord == gencoord)
          {
            func = ms->joint[i].dofs[j].function;
             if (func->numpoints == 2 && EQUAL_WITHIN_ERROR(func->x[0], func->y[0]))
             {
               slope = (func->y[1] - func->y[0]) / (func->x[1] - func->x[0]);
               if (EQUAL_WITHIN_ERROR(slope, 1.0))
               {
                   ms->joint[i].dofs[j].sd.constrained = no;
                    ms->joint[i].dofs[j].sd.conversion_sign = 1.0;
                  *jnt = i;
                  *dof = j;
                        if (constrained)
                            *constrained = no;
                  return 1;
               }
               else if (EQUAL_WITHIN_ERROR(slope,-1.0))
               {
                   ms->joint[i].dofs[j].sd.constrained = no;
                   /* If slope is negative, set the conversion_sign to -1
                     * so the conversion factor (which is set later) will
                     * be negative.
                     */
                    ms->joint[i].dofs[j].sd.conversion_sign = -1.0;
                  *jnt = i;
                  *dof = j;
                        if (constrained)
                            *constrained = no;
                  return 1;
               }
             }
          }
      }
   }

    // This code is specifically for conversion to Simbody-based models, which do
    // not have the restriction that every gencoord must map to a DOF with a "simple"
    // function. If there is such a gencoord, its "primary" DOF is set to the first
    // one that uses it, but it's marked as constrained so that the OpenSim export
    // code knows to add the coordinate to the joint and the kinematic function to
    // the corresponding transform axis.
    if (constrained)
    {
        for (i = 0; i < ms->numjoints; i++)
        {
            for (j = 0; j < 6; j++)
            {
                if (ms->joint[i].dofs[j].type == function_dof &&
                    ms->joint[i].dofs[j].gencoord == gencoord)
                {
                    ms->joint[i].dofs[j].sd.constrained = no;       // so other code will not create a new coordinate
                    ms->joint[i].dofs[j].sd.conversion_sign = 1.0;  // and constrain it to this gencoord
                    *jnt = i;
                    *dof = j;
                    *constrained = yes;
                    return 1;
                }
            }
        }
    }

    return 0;
}


/* INIT_JOINT: this routine initializes a joint structure before a joint is read
 * from an input file.
 */
void init_joint(ModelStruct* ms, JointStruct* jointstr)
{
   int i;

   jointstr->to = -1;
   jointstr->from = -1;
   jointstr->name = NULL;
   jointstr->type = dpUnknownJoint;
   jointstr->defined = no;
   jointstr->solverType = NULL;
   jointstr->loop_joint = no;
   jointstr->user_loop = no;
   jointstr->conversion.condition = invalid;
   jointstr->sd_name = NULL;

   /* In case the transformation order is not specified in the file, assume
    * a default ordering of t, r1, r2, r3.
    */
   for (i=0; i<4; i++)
      jointstr->order[i] = i;

   /* If any dofs are not specified in the file, the error checking will throw out
    * the bad joint (and model), but make all the dofs constants of 0.0 anyway.
    */
   for (i=0; i<6; i++)
   {
      jointstr->dofs[i].type = constant_dof;
      jointstr->dofs[i].value = 0.0;
      jointstr->dofs[i].function = NULL;
      jointstr->dofs[i].gencoord = NULL;
      jointstr->dofs[i].sd.name = NULL;
      jointstr->dofs[i].sd.con_name = NULL;
      jointstr->dofs[i].sd.initial_value = 0.0;
      jointstr->dofs[i].sd.constrained = no;
      jointstr->dofs[i].sd.fixed = no;
      jointstr->dofs[i].sd.state_number = -1;
      jointstr->dofs[i].sd.error_number = -1;
      jointstr->dofs[i].sd.joint = -1;
      jointstr->dofs[i].sd.axis = -1;
      jointstr->dofs[i].sd.conversion = 0.0;
      jointstr->dofs[i].sd.conversion_sign = 1.0;
   }

   jointstr->dofs[0].key = r1;
   jointstr->dofs[1].key = r2;
   jointstr->dofs[2].key = r3;
   jointstr->dofs[3].key = tx;
   jointstr->dofs[4].key = ty;
   jointstr->dofs[5].key = tz;

   jointstr->show_axes[0] = no;
   jointstr->show_axes[1] = no;
   jointstr->show_axes[2] = no;

   jointstr->axis_length[0] = 0.0;
   jointstr->axis_length[1] = 0.0;
   jointstr->axis_length[2] = 0.0;

   jointstr->in_seg_ground_path = NULL;

   jointstr->pretransform_active = no;
   jointstr->pretransform_condition = valid;

   identity_matrix(jointstr->pretransform);
   identity_matrix(jointstr->pretransform_inverse);

#if INCLUDE_MOCAP_MODULE
   jointstr->mocap_segment = NULL;
   jointstr->mocap_seg_index = -1;
#endif
}


void initwrapobject(dpWrapObject* wo)
{
   wo->name = NULL;
   wo->wrap_type = dpWrapSphere;
   wo->wrap_algorithm = WE_HYBRID_ALGORITHM;
   wo->segment = 0;
   wo->display_list = 0;
   wo->display_list_is_stale = no;
   wo->active = yes;               /* set wrap object fields to default values */
   wo->visible = yes;
   wo->show_wrap_pts = no;
   wo->radius[0] = 0.05;
   wo->radius[1] = 0.1;
   wo->radius[2] = 0.05;
   wo->height = 0.1;
   wo->wrap_axis = 0;
   wo->wrap_sign = 0;
   
   wo->rotation_axis.xyz[0] = 1.0;
   wo->rotation_axis.xyz[1] = 0.0;
   wo->rotation_axis.xyz[2] = 0.0;
   wo->rotation_angle = 0.0;
   wo->translation.xyz[0] = 0.0;
   wo->translation.xyz[1] = 0.0;
   wo->translation.xyz[2] = 0.0;
   
   wo->undeformed_translation.xyz[0] = 0.0;
   wo->undeformed_translation.xyz[1] = 0.0;
   wo->undeformed_translation.xyz[2] = 0.0;
   
   identity_matrix(wo->from_local_xform);
   identity_matrix(wo->to_local_xform);
   
   wo->xforms_valid = yes;

#if VISUAL_WRAPPING_DEBUG
   wo->num_debug_glyphs = 0;
#endif
}

void initconstraintobject(ConstraintObject* co)
{
   co->name = NULL;
   co->numPoints = 0;
   co->points = NULL;
   co->cp_array_size = CP_ARRAY_INCREMENT;
   co->constraintType = constraint_sphere;
   co->segment = 0;
   co->display_list = 0;
   co->display_list_is_stale = no;
   co->active = yes;               
   co->visible = yes;
   co->radius.xyz[0] = 0.05;
   co->radius.xyz[1] = 0.1;
   co->radius.xyz[2] = 0.05;
   co->height = 0.1;
   co->constraintAxis = 0;
   co->constraintSign = 0;
   
   co->plane.a = 0.0;
   co->plane.b = 0.0;
   co->plane.c = 1.0;
   co->plane.d = 0.0;

   co->rotationAxis.xyz[0] = 1.0;
   co->rotationAxis.xyz[1] = 0.0;
   co->rotationAxis.xyz[2] = 0.0;
   co->rotationAngle = 0.0;
   co->translation.xyz[0] = 0.0;
   co->translation.xyz[1] = 0.0;
   co->translation.xyz[2] = 0.0;
   
   co->undeformed_translation.xyz[0] = 0.0;
   co->undeformed_translation.xyz[1] = 0.0;
   co->undeformed_translation.xyz[2] = 0.0;
   
   identity_matrix(co->from_local_xform);
   identity_matrix(co->to_local_xform);
   
   co->xforms_valid = yes;

   co->num_qs = 0;
   co->num_jnts = 0;
   co->qs = NULL;
   co->joints = NULL;
}

void initconstraintpoint(ConstraintPoint *pt)
{
   pt->name = NULL;
   pt->weight = 1.0;
   pt->offset[0] = 0.01;
   pt->offset[1] = 0.01;
   pt->offset[2] = 0.01;
   pt->segment = 0;
   pt->tolerance = DEFAULT_CONSTRAINT_TOLERANCE;
//   pt->visible = yes;
//   pt->active = yes;
}

#if ! ENGINE

static void make_confirm_menu(void)
{
   Menu* ms;

   ms = &root.confirm_menu;

   ms->title = NULL;
   ms->numoptions = 2;
   ms->origin.x = 0;
   ms->origin.y = 0;
   ms->type = normal_menu;
   ms->option = (MenuItem*)simm_malloc(ms->numoptions*sizeof(MenuItem));
   if (ms->option == NULL)
      error(exit_program,tool_message);

   ms->option[0].active = yes;
   ms->option[1].active = yes;
   ms->option[0].visible = yes;
   ms->option[1].visible = yes;
   mstrcpy(&ms->option[0].name,confirmstr[0]);
   mstrcpy(&ms->option[1].name,confirmstr[1]);

   SET_BOX(ms->option[0].box,30,100,30,ms->option[0].box.y1+MENU_ITEM_HEIGHT);
   SET_BOX(ms->option[1].box,160,230,30,ms->option[1].box.y1+MENU_ITEM_HEIGHT);
}


void make_message_port(void)
{
   IntBox bbox;
   HelpStruct* hp;
   int windex;
   WindowParams mwin;
   WinUnion wun;
   SBoolean iconified;

   hp = &root.messages;

   hp->lines_per_page = 10;
   hp->window_width = 750;
   hp->window_height = 10*HELP_WINDOW_TEXT_Y_SPACING + 4;

   hp->line = (TextLine*)simm_malloc(100*sizeof(TextLine));
   if (hp->line == NULL)
      error(exit_program,tool_message);

   hp->num_lines_malloced = 100;
   hp->num_lines = 0;
   hp->starting_line = 0;
   hp->background_color = HELP_WINDOW_BACKGROUND;

   bbox.x2 = hp->window_width;
   bbox.x1 = bbox.x2 - 20;
   bbox.y2 = hp->window_height;
   bbox.y1 = 0;

   make_slider(&hp->sl,vertical_slider,bbox,0,(double)(hp->num_lines)*20.0,
      (double)(hp->lines_per_page)*20.0,(double)(hp->num_lines)*20.0,4.0,NULL,NULL);
   glueSetWindowPlacement(GLUT_NORMAL_WINDOW, GLUE_SOUTHWEST,
              hp->window_width, hp->window_height,
              0, 0, NULL);
   
#if defined WIN32 && SIMM_DEMO_VERSION
   iconified = is_in_demo_mode();
#else
   iconified = no;
#endif
   mwin.id = hp->window_id = glueOpenWindow("simmmess",iconified,GLUE_TOOL_WINDOW);

   glueSetWindowMinSize(500,100);
   mstrcpy(&mwin.name,"SIMM Messages");
   glutSetIconTitle("Messages");
   glutSetWindowTitle(mwin.name);

   wun.tool = NULL;

   windex = add_window(&mwin,&wun,NOTYPE,ZERO,no,draw_message_window,
               update_message_window,messages_input);
   if (windex == -1)
      error(exit_program,tool_message);

   glutSetWindowData(mwin.id, windex);
   
   if (expirationMessage)
   {
      error(none, expirationMessage);
      
      FREE_IFNOTNULL(expirationMessage);
   }
}



public void update_message_window(WindowParams* win_parameters, WinUnion* win_struct)
{
   IntBox bbox;
   HelpStruct* hp;

   if (shape_window(win_parameters,no) < CHANGED_SIZE)
      return;

   hp = &root.messages;

   hp->window_width = win_parameters->xsize;
   hp->window_height = win_parameters->ysize;
   hp->lines_per_page = (hp->window_height-4)/20;

   bbox.x2 = hp->window_width;
   bbox.x1 = bbox.x2 - 20;
   bbox.y2 = hp->window_height;
   bbox.y1 = 0;

   make_slider(&hp->sl,vertical_slider,bbox,0,hp->sl.value,
      (double)(hp->lines_per_page)*20.0,(double)(hp->num_lines)*20.0,4.0,NULL,NULL);
   
   if (hp->num_lines <= hp->lines_per_page)
   {
      hp->starting_line = 0;
      hp->sl.value = hp->sl.max_value;
      hp->sl.thumb_thickness = -1;
   }
   else
   {
      if (hp->starting_line + hp->lines_per_page*20 > hp->num_lines*20)
      {
         hp->starting_line = (hp->num_lines - hp->lines_per_page)*20;
         hp->sl.value = hp->sl.max_value - hp->starting_line;
      }
      hp->sl.thumb_thickness = hp->lines_per_page*
         (hp->sl.shaft.y2-hp->sl.shaft.y1)/hp->num_lines;
   }
}

public void draw_message_window(WindowParams* win_parameters, WinUnion* win_struct)
{
   draw_help_window(&root.messages);
}


public void messages_input(WindowParams* win_parameters, WinUnion* win_struct,
               SimmEvent se)
{
   if (se.field1 == window_shut)
   {
      do_tool_window_shut("Messages", win_parameters->id);
      return;
   }

   if (se.field1 != select_button || se.field2 != key_pressed)
      return;

   if (check_slider(&root.messages.sl,se,move_message_text,DUMMY_INT) == yes)
      draw_help_window(&root.messages);
}



public void move_message_text(int dummy_int, double slider_value, double delta)
{
   root.messages.starting_line = root.messages.sl.max_value - slider_value;

   draw_help_window(&root.messages);
}

#endif /* ! ENGINE */


#if ! OPENSIM_BUILD && ! CORTEX_PLUGIN

char* get_mocap_model(void)
{
   char* m;
   const char* p = get_preference("MOCAP_MODEL");

   if (p)
   {
      if (is_absolute_path(p))
      {
         strcpy(buffer, p);
      }
      else
      {
         strcpy(buffer, get_preference("SIMM_FOLDER"));
         append_if_necessary(buffer, DIR_SEP_CHAR);
         strcat(buffer, p);
      }
   }
   else
   {
      strcpy(buffer, get_preference("RESOURCES_FOLDER"));
      append_if_necessary(buffer, DIR_SEP_CHAR);
      strcat(buffer, "mocap\\mocap.jnt");
   }

   mstrcpy(&m, buffer);

   return m;
}
#endif

#if INCLUDE_PERSISTENT_SIMM_STATE

/* ===========================================================================
  PERSISTENT SIMM STATE
------------------------------------------------------------------------------ */
#if 0
   #pragma mark -
   #pragma mark PERSISTENT SIMM STATE (unfinished)
#endif

typedef struct {
   char* name;
   char* value;
} SimmStateVar;

static int           sSimmStateNumVars = 0;
static SimmStateVar* sSimmStateVars = NULL;

/* ---------------------------------------------------------------------------
  _free_simm_state - 
------------------------------------------------------------------------------ */
static void _free_simm_state ()
{
   if (sSimmStateVars)
   {
      int i;
      
      for (i = 0; i < sSimmStateNumVars; i++)
      {
         FREE_IFNOTNULL(sSimmStateVars[i].name);
         FREE_IFNOTNULL(sSimmStateVars[i].value);
      }
      FREE_IFNOTNULL(sSimmStateVars);
      
      sSimmStateNumVars = 0;
   }
}

/* ---------------------------------------------------------------------------
  load_simm_state - 
------------------------------------------------------------------------------ */
public void load_simm_state ()
{
   FILE* file = NULL;

   strcpy(buffer, get_preference("RESOURCES_FOLDER"));
   append_if_necessary(buffer, DIR_SEP_CHAR);
   strcat(buffer, "SimmState.txt");

   file = simm_fopen(buffer, "r");

   if (file)
   {
      int numLines = count_remaining_lines(file, no);

      _free_simm_state();
      fclose(file);
   }
}

/* ---------------------------------------------------------------------------
  save_simm_state - 
------------------------------------------------------------------------------ */
public void save_simm_state ()
{
   
}

/* ---------------------------------------------------------------------------
  get_simm_state_value - 
------------------------------------------------------------------------------ */
public const char* get_simm_state_value (const char* name)
{
   return NULL;
}

/* ---------------------------------------------------------------------------
  set_simm_state_value - 
------------------------------------------------------------------------------ */
public void set_simm_state_value (const char* name, const char* value)
{
   
}

#endif /* INCLUDE_PERSISTENT_SIMM_STATE */
