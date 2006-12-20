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
      get_environment_vars : gets values of some environment variables

*******************************************************************************/

#ifdef __MWERKS__
   #include <unistd.h>
#endif

#ifdef WIN32
#include <direct.h>
#endif

#include "universal.h"
#include "globals.h"
#include "functions.h"

//#include "wrapeditor.h"
#include "wefunctions.h"
//#include "constrainteditor.h"
//#include "cefunctions.h"

/*************** DEFINES (for this file only) *********************************/
#define EVENT_QUEUE_SIZE 50
#define RAMPSIZE 32
#define INIT_WORLD_ARRAY_SIZE 10
#define STARTING_GEAR 0.08 // must be the same as MV_STARTING_GEAR !!

/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/
static void make_confirm_menu(void);
static char* confirmstr[] = {
"yes","cancel"
};


/*************** EXTERNED VARIABLES (declared in another file) ****************/


/*************** GLOBAL VARIABLES (for use in a few files) ****************/

char* expirationMessage = NULL;



/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static void make_mocap_model_filepath(void);


#ifndef ENGINE

/* INITIALIZE: the main initializing routine that is called once at startup.
 * It makes the program window, queues the keyboard and mouse buttons, and
 * calls routines to make the color map.
 */

void initialize(void)
{

   int i;
   int xtmp, ytmp;
   WindowParams bwin;
   WinUnion wun;

   strcpy(buffer,get_simm_directory());
   mstrcpy(&root.simm_base_dir,buffer);

   get_simm_resources_directory();    /* initialize the SIMM resources path */
   
   add_preprocessor_option(yes, "-I%s", get_simm_resources_directory());

   root.numwindows = 0;
   root.messages.line = NULL;

   root.gldesc.max_screen_x = glutGet(GLUT_SCREEN_WIDTH);
   if (root.gldesc.max_screen_x <= 0)
      root.gldesc.max_screen_x = 1024;
   root.gldesc.max_screen_y = glutGet(GLUT_SCREEN_HEIGHT);
   if (root.gldesc.max_screen_y <= 0)
      root.gldesc.max_screen_y = 768;

#ifdef WIN32
   root.gldesc.max_screen_x -= 10;
   root.gldesc.max_screen_y -= 30;

   glueSetWindowPlacement(GLUT_NORMAL_WINDOW, GLUE_CENTER,
			  root.gldesc.max_screen_x,
			  root.gldesc.max_screen_y, 0, 0, NULL);

   glueSetConstrainedWindows(yes);

   init_main_menu();   /* the main menu must be created *before* the SIMM window */
   
   strcpy(root.pref.bonefilepath,   get_simm_resources_directory());
   strcat(root.pref.bonefilepath,   "bones");
   
   strcpy(root.pref.jointfilepath,  ".");
   strcpy(root.pref.plotfilepath,   ".");
   strcpy(root.pref.outputfilepath, ".");
#else
   glueSetPrefWindowPosition(8,root.gldesc.max_screen_x-75,
			     8,root.gldesc.max_screen_y-32);
#endif
   bwin.id = glueOpenWindow("simm",no,GLUE_NO_WINDOW_FLAGS);

   glueSetWindowMinSize(950,700);

#if SIMM_DEMO_VERSION
   sprintf(buffer,"%s - Tryout Version (%s)", program_name, program_version);
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
      plot[i] = NULL;

   /* Set all the model pointers to NULL */
   for (i=0; i<MODELBUFFER; i++)
      model[i] = NULL;

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
   for (i=0; i<100; i++)
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

   make_mocap_model_filepath();

#if ! SIMM_DEMO_VERSION
#ifdef WIN32
   init_gm_help();
#endif
#endif

}

#endif /* ENGINE */

/* INITMODEL: this guy initializes much of the model structure.
 */

ReturnCode init_model(ModelStruct* ms)
{

   int i;

   ms->name = NULL;
   ms->forceUnits = NULL;
   ms->lengthUnits = NULL;
   ms->HTRtranslationUnits = NULL;
   ms->is_demo_model = no;
   ms->useIK = yes;
   ms->defaultGCApproved = yes;
   ms->defaultLoopsOK = yes;
   ms->defaultConstraintsOK = yes;
   ms->constraintsOK = yes;
   ms->loopsOK = yes;
   ms->numjoints = 0;
   ms->numsegments = 0;
   ms->numgroups = 0;
   ms->numseggroups = 0;
   ms->numgencgroups = 0;
   ms->numfunctions = 0;
   ms->numgencoords = 0;
   ms->numunusedgencoords = 0;
   ms->numworldobjects = 0;
   ms->numclosedloops = 0;
   ms->numligaments = 0;
//   ms->dis_muscle_array_size = 0;//dkb = MUSCLE_ARRAY_INCREMENT;
   ms->muscle_array_size = MUSCLE_ARRAY_INCREMENT;//0;//dkbMUSCLE_ARRAY_INCREMENT;
   ms->ligament_array_size = MUSCLE_ARRAY_INCREMENT;
   ms->muscgroup_array_size = MUSCGROUP_ARRAY_INCREMENT;
   ms->seggroup_array_size = 0;
   ms->gencgroup_array_size = 0;
   ms->world_array_size = INIT_WORLD_ARRAY_SIZE;
   ms->genc_array_size = GENC_ARRAY_INCREMENT;
   ms->segment_array_size = SEGMENT_ARRAY_INCREMENT;
   ms->joint_array_size = JOINT_ARRAY_INCREMENT;
   ms->func_array_size = FUNC_ARRAY_INCREMENT;
   ms->specified_min_thickness = yes;
   ms->specified_max_thickness = yes;
   ms->dynamics_ready = no;
   ms->currentframe = 0;
   ms->pushedframe = 0;
   ms->pathptrs = NULL;
   ms->jointfilename = NULL;
   ms->bonepathname = NULL;
   ms->musclefilename = NULL;
   ms->mocap_dir = NULL;
   ms->segment_drawing_order = NULL;
   ms->max_diagonal_needs_recalc = yes;
   ms->GEFuncOK = no;
   ms->marker_visibility = yes;
   ms->marker_radius = DEFAULT_MARKER_RADIUS;
   ms->loop_tolerance = DEFAULT_LOOP_TOLERANCE;
   ms->solver.accuracy = DEFAULT_SOLVER_ACCURACY;
   ms->solver.method = smLevenbergMarquart;
   ms->solver.max_iterations = 100;
   ms->solver.joint_limits = smYes;
   ms->solver.orient_body = smNo;
   ms->global_show_masscenter = no;
   ms->global_show_inertia = no;

   ms->numContactPairs = 0;
   ms->contactPairArraySize = 0;
   ms->contactPair = NULL;
   ms->numContactGroups = 0;
   ms->contactGroupArraySize = 0;
   ms->contactGroup = NULL;

   ms->gravity = smNegY;

   ms->num_motions = 0;
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
   ms->gencoord = (GeneralizedCoord*)simm_malloc(ms->genc_array_size*
      sizeof(GeneralizedCoord));
#if 1
   ms->muscle = (MuscleStruct*)simm_malloc(ms->muscle_array_size*sizeof(MuscleStruct));
   //dkb MUSCLE_ARRAY_SIZE
#else
   ms->muscle = NULL;
#endif
   ms->ligament = (LigamentStruct*)simm_malloc(ms->ligament_array_size*sizeof(LigamentStruct));

   ms->function = (SplineFunction*)simm_malloc(ms->func_array_size*sizeof(SplineFunction));

   ms->save.function = (SplineFunction*)simm_malloc(ms->func_array_size*
      sizeof(SplineFunction));
   ms->worldobj = (WorldObject*)simm_malloc(ms->world_array_size*sizeof(WorldObject));

   if (ms->joint == NULL || ms->segment == NULL || ms->muscgroup == NULL ||
//       ms->gencoord == NULL || ms->muscle == NULL || ms->function == NULL ||
       ms->gencoord == NULL || ms->function == NULL ||
       ms->save.function == NULL || ms->worldobj == NULL)
   {
      error(none,"Not enough memory to add another model.");
      return (code_bad);
   }

   ms->save.num_wrap_objects = 0;
   ms->save.wrapobj = NULL;
   ms->save.wrapobjnames = NULL;
   ms->save.num_muscwrap_associations = 0;
   ms->save.muscwrap_associations = NULL;
   ms->save.num_markers = 0;
   ms->save.marker = NULL;
   ms->save.num_constraint_objects = 0;
   ms->save.constraintobj = NULL;

   ms->num_wrap_objects = 0;
   ms->wrap_object_array_size = WRAP_OBJECT_ARRAY_INCREMENT;
   ms->wrapobj = (WrapObject*)simm_malloc(ms->wrap_object_array_size*sizeof(WrapObject));
   if (ms->wrapobj == NULL)
   {
      error(none,"Not enough memory to add another wrap object.");
      return (code_bad);
   }

//   ms->constraint_tolerance = DEFAULT_CONSTRAINT_TOLERANCE;
   ms->num_constraint_objects = 0;
   ms->constraint_object_array_size = CONSTRAINT_OBJECT_ARRAY_INCREMENT;
   ms->constraintobj = (ConstraintObject*)simm_malloc(ms->constraint_object_array_size*sizeof(ConstraintObject));
   if (ms->constraintobj == NULL)
   {
      error(none,"Not enough memory to add another constraint.");
      return (code_bad);
   }

   ms->save.num_deforms = 0;
   ms->save.deform = NULL;

   ms->num_deformities = 0;
   ms->deformity_array_size = DEFORMITY_ARRAY_INCREMENT;
   ms->deformity = (Deformity*) simm_malloc(ms->deformity_array_size * sizeof(Deformity));
   if (ms->deformity == NULL)
   {
      error(none,"Not enough memory to add another deformity object.");
      return (code_bad);
   }

   for (i=0; i<ms->genc_array_size; i++)
   {
      ms->gencoord[i].defined = no;
      ms->gencoord[i].numgroups = 0;
      ms->gencoord[i].group = NULL;
      ms->gencoord[i].jointnum = NULL;
#if INCLUDE_MOCAP_MODULE
      ms->gencoord[i].mocap_segment   = NULL;
      ms->gencoord[i].mocap_seg_index = -1;
      ms->gencoord[i].mocap_column    = -1;
      ms->gencoord[i].mocap_adjust    = 0.0;
#endif
   }
   for (i=0; i<ms->func_array_size; i++)
   {
      ms->function[i].defined = no;
      ms->function[i].used = no;
      ms->save.function[i].defined = no;
      ms->save.function[i].used = no;
   }

#ifndef ENGINE
   nullify_muscle(&ms->save.default_muscle);
#endif
   ms->save.numsavedmuscs = 0;
   ms->save.numsavedjnts = 0;
   ms->save.numsavedgencs = 0;
   ms->save.numsavedbones = 0;

   init_materials(ms);

   ms->num_motion_objects = 0;
   ms->motion_objects = NULL;

#ifndef ENGINE
   add_default_motion_objects(ms);

   /* This part of the display structure must be initialized here because
    * it can be changed when reading in the joints file, which happens before
    * the display structure is initialized.
    */
#endif

   ms->dis.trackball = yes;
   ms->dis.default_view = 0; // the default camera will be cam0 unless overridden by user
   ms->dis.camera_segment = -1;
   ms->dis.num_file_views = 0;
   ms->dis.current_gear = STARTING_GEAR;
//   ms->dis.muscle_array_size = 0;//dkb = MUSCLE_ARRAY_INCREMENT;
   ms->dis.muscle_array_size = MUSCLE_ARRAY_INCREMENT;
   
   ms->dis.fast_muscle_drawing = no;
   
   ms->dis.windowX1 = -1;
   ms->dis.windowY1 = -1;
   ms->dis.windowHeight = 400;
   ms->dis.windowWidth = 400;

#ifndef ENGINE
   const char* p = getpref("FASTER_MUSCLE_DRAWING");
   
   if (p)
   {
      if (STRINGS_ARE_EQUAL(p, "yes") ||
          STRINGS_ARE_EQUAL(p, "Yes") ||
          STRINGS_ARE_EQUAL(p, "on") ||
          STRINGS_ARE_EQUAL(p, "On") ||
          STRINGS_ARE_EQUAL(p, "true") ||
          STRINGS_ARE_EQUAL(p, "True"))
      {
         ms->dis.fast_muscle_drawing = yes;
      }
   }

#if INCLUDE_MSL_LENGTH_COLOR
   ms->dis.muscle_color_factor = 0.0;
#endif
   
   for (i=0; i<MAXSAVEDVIEWS; i++)
      ms->dis.view_used[i] = no;

   for (i=0; i<3; i++)
   {
      ms->dis.background_color[i] = 0.2;
      ms->dis.vertex_label_color[i] = 0.0;
      ms->dis.rotation_axes_color[i] = 1.0;
      ms->dis.segment_axes_color[i] = 1.0;
      ms->dis.crosshairs_color[i] = 1.0;
   }
   ms->dis.rotation_axes_color[2] = 0.0;
   ms->dis.vertex_label_color[0] = 1.0;

   ms->dis.background_color_spec = no;
   ms->dis.vertex_label_color_spec = no;
   ms->dis.rotation_axes_color_spec = no;
   ms->dis.segment_axes_color_spec = no;
   ms->dis.crosshairs_color_spec = no;

   ms->modelLock = glutNewMutex();
   ms->realtimeState = rtNotConnected;

#endif /* ENGINE */

   return (code_fine);

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
   seg->lengthstartend_specified = no;
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

   ms->dis.rx = 0;
   ms->dis.ry = 0;
   ms->dis.rz = 0;

   reset_4x4matrix(ms->dis.transform_matrix);

   ms->dis.show_highlighted_polygon = no;
   ms->dis.show_selected_coords = no;
   ms->dis.show_crosshairs = no;
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
      ms->dis.devs[i*2] = ms->gencoord[i].keys[0];
      ms->dis.devs[i*2+1] = ms->gencoord[i].keys[1];
   }
   ms->dis.numdevs = ms->numgencoords*2;

   ms->dis.nummuscleson = ms->nummuscles;
   ms->dis.muscle_array_size = ms->muscle_array_size;
   if (ms->dis.muscle_array_size > 0)
   {
      ms->dis.muscleson = (int*)simm_malloc(ms->dis.muscle_array_size*sizeof(int));
      ms->save.disp.muscleson = (int*)simm_malloc(ms->dis.muscle_array_size*sizeof(int));
      if (ms->dis.muscleson == NULL || ms->save.disp.muscleson == NULL)
         return code_bad;
   }

   ms->dis.muscle_cylinder_id = -1;

   for (i=0; i<ms->dis.muscle_array_size; i++)
   {
      if (i < ms->nummuscles)
         ms->dis.muscleson[i] = ms->muscle[i].display;
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

   for (i=0; i<4; i++)
      ms->dis.viewport[i] = 0;

   for (i=0; i<16; i++)
   {
      ms->dis.projection_matrix[i] = 0.0;
      ms->dis.modelview_matrix[i] = 0.0;
   }

   ms->dis.fov_angle = 40.0;
   ms->dis.y_zoom_constant = tan((double)ms->dis.fov_angle*0.5*DTOR);
   ms->dis.near_clip_plane = 0.0002; /* reset in size_model */
   ms->dis.far_clip_plane = 12.3; /* reset in size_model */

#if INCLUDE_SNAPSHOT
   ms->dis.snapshot_mode           = SNAPSHOT_INACTIVE;
   ms->dis.snapshot_counter        = 1;
   ms->dis.snapshot_file_base_name = NULL;
   ms->dis.snapshot_file_suffix    = NULL;
   ms->dis.snapshot_include_depth  = no;

   if (getpref("SNAPSHOT_FILE_NAME"))
      mstrcpy(&ms->dis.snapshot_file_base_name, getpref("SNAPSHOT_FILE_NAME"));
#endif
   
   ms->dis.display_animation_hz = 0x00;

#ifndef ENGINE
   /* initialize motion info display. */
   {
      char* p = (char*)getpref("DISPLAY_MOTION_INFO");
      
      if (p)
      {
         if (STRINGS_ARE_EQUAL(p, "no") ||
             STRINGS_ARE_EQUAL(p, "No") ||
             STRINGS_ARE_EQUAL(p, "NO") ||
             STRINGS_ARE_EQUAL(p, "false") ||
             STRINGS_ARE_EQUAL(p, "False") ||
             STRINGS_ARE_EQUAL(p, "FALSE") ||
             STRINGS_ARE_EQUAL(p, "off") ||
             STRINGS_ARE_EQUAL(p, "Off") ||
             STRINGS_ARE_EQUAL(p, "OFF"))
            ms->dis.display_motion_info = no;
      }
   }

   /* initialize animation frequency display. */
   {
      char* p;
      p = (char*)getpref("DISPLAY_ANIMATION_HZ");

      if (p)
      {
         if (STRINGS_ARE_EQUAL(p, "console"))
         {
            ms->dis.display_animation_hz |= DISPLAY_HZ_ON_CONSOLE;
         }
         else if (STRINGS_ARE_NOT_EQUAL(p, "no") &&
            STRINGS_ARE_NOT_EQUAL(p, "No") &&
            STRINGS_ARE_NOT_EQUAL(p, "NO") &&
            STRINGS_ARE_NOT_EQUAL(p, "false") &&
            STRINGS_ARE_NOT_EQUAL(p, "False") &&
            STRINGS_ARE_NOT_EQUAL(p, "FALSE") &&
            STRINGS_ARE_NOT_EQUAL(p, "off") &&
            STRINGS_ARE_NOT_EQUAL(p, "Off") &&
            STRINGS_ARE_NOT_EQUAL(p, "OFF"))
         {
            ms->dis.display_animation_hz |= DISPLAY_HZ_IN_WINDOW;
         }
      }

      p = (char*)getpref("DISPLAY_POLYGONS_PER_SECOND");

      if (p)
      {
         if (STRINGS_ARE_NOT_EQUAL(p, "no") &&
             STRINGS_ARE_NOT_EQUAL(p, "No") &&
             STRINGS_ARE_NOT_EQUAL(p, "NO") &&
             STRINGS_ARE_NOT_EQUAL(p, "false") &&
             STRINGS_ARE_NOT_EQUAL(p, "False") &&
             STRINGS_ARE_NOT_EQUAL(p, "FALSE") &&
             STRINGS_ARE_NOT_EQUAL(p, "off") &&
             STRINGS_ARE_NOT_EQUAL(p, "Off") &&
             STRINGS_ARE_NOT_EQUAL(p, "OFF"))
         {
            ms->dis.display_animation_hz |= DISPLAY_POLYS_PER_SEC;
         }
      }
   }
#endif

   return code_fine;
}

#ifndef ENGINE

/* INITPLOT: this guy initializes the plot structure, which contains viewport
 * and ortho values as well as the set of curve structures for that plot.
 * First it calls a routine to set the viewport and window-resize box
 * variables.
 */

ReturnCode initplot(int plotnum)
{

   plot[plotnum]->plotnum = plotnum;
   plot[plotnum]->numcurves = 0;
   plot[plotnum]->zoomed_yet = no;
   plot[plotnum]->numpoints = 0;
   plot[plotnum]->xname_len = 0;
   plot[plotnum]->yname_len = 0;
   plot[plotnum]->title_len = 256;
   plot[plotnum]->title = (char*)simm_malloc(plot[plotnum]->title_len);
   plot[plotnum]->xname = NULL;
   plot[plotnum]->yname = NULL;
   plot[plotnum]->cursor_modelPtr = NULL;
   plot[plotnum]->cursor_motion = NULL;
   plot[plotnum]->show_cursor = yes;
   plot[plotnum]->show_events = yes;
   plot[plotnum]->needs_bounding = no;
   plot[plotnum]->numFileEvents = 0;
   plot[plotnum]->fileEvent = NULL;

   if (plot[plotnum]->title == NULL)
      return code_bad;

   NULLIFY_STRING(plot[plotnum]->title);

   return code_fine;

}

#endif /* ! ENGINE */

/* INIT_MUSCLE: this routine initializes much of the muscle structure. Before
 * a given muscle is read-in from an input file, each of the pointers to its
 * muscle-specific parameters is set to point into the default-muscle structure.
 * Thus if a certain parameter is not specified for a muscle (e.g. force-length
 * curve), the default-muscle's parameter is used. This cuts down significantly
 * on the amount of information that must be supplied in a muscle file since
 * most muscles share the same active_force_len_curve, tendon_force_len_curve,
 * passive_force_len_curve, and force_vel_curve.
 */

ReturnCode init_muscle(int mod, int musc)
{

   int i;
   MuscleStruct* muscl;
   MuscleStruct* dm;

   muscl = &model[mod]->muscle[musc];
   dm = &model[mod]->default_muscle;

   muscl->name = dm->name;
   muscl->display = dm->display;
   muscl->output = yes;
   muscl->selected = no;
   muscl->has_wrapping_points = no;
   muscl->has_force_points = no;
   muscl->num_orig_points = dm->num_orig_points;
   muscl->mp_orig_array_size = 0;
   muscl->mp_orig = dm->mp_orig;
   muscl->numgroups = dm->numgroups;
   muscl->group = dm->group;
   muscl->max_isometric_force = dm->max_isometric_force;
   muscl->pennation_angle = dm->pennation_angle;
   muscl->optimal_fiber_length = dm->optimal_fiber_length;
   muscl->resting_tendon_length = dm->resting_tendon_length;
   muscl->min_thickness = dm->min_thickness;
   muscl->max_thickness = dm->max_thickness;
   muscl->min_material = dm->min_material;
   muscl->max_material = dm->max_material;
   muscl->max_contraction_vel = dm->max_contraction_vel;
   muscl->force_vel_curve = dm->force_vel_curve;

   if (init_dynamic_param_array(model[mod],muscl) == code_bad)
      return (code_bad);

   for (i=0; i<muscl->num_dynamic_params; i++)
      muscl->dynamic_params[i] = dm->dynamic_params[i];

   muscl->muscle_model_index = dm->muscle_model_index;
   muscl->excitation = dm->excitation;
   muscl->excitation_format = dm->excitation_format;
   muscl->excitation_index = 0;
   muscl->excitation_abscissa = dm->excitation_abscissa;

   muscl->nummomentarms = model[mod]->numgencoords;
   muscl->momentarms = (double *)simm_malloc(muscl->nummomentarms*sizeof(double));
   if (muscl->momentarms == NULL)
      return (code_bad);

   for (i=0; i<muscl->nummomentarms; i++)
      muscl->momentarms[i] = 0.0;
   muscl->activation = muscl->initial_activation = 1.0;
   muscl->tendon_force_len_curve = dm->tendon_force_len_curve;
   muscl->active_force_len_curve = dm->active_force_len_curve;
   muscl->passive_force_len_curve = dm->passive_force_len_curve;

   muscl->wrap_calced = no;
   muscl->numWrapStructs = 0;
   muscl->wrapStruct = NULL;
   
   muscl->num_points = 0;
   muscl->mp = NULL;
   muscl->mp_array_size = 0;

   return (code_fine);

}


/* INIT_GENCOORD: this routine initializes a gencoord structure before a gencoord is
 * read from an input file.
 */

void init_gencoord(GeneralizedCoord* gc)
{
   gc->name = NULL;
   gc->type = rotation_gencoord;
   gc->defined = no;
   gc->numjoints = 0;
   gc->jointnum = NULL;
   gc->wrap = no;
   gc->numgroups = 0;
   gc->group = NULL;

   gc->keys[0] = gc->keys[1] = null_key;

   gc->tolerance = 0.0;
   gc->default_value = 0.0;
   gc->default_value_specified = no;
   gc->clamped = yes;
   gc->clamped_save = gc->clamped;
   gc->locked = no;
   gc->locked_save = gc->locked;
   gc->range.start = 0.0;
   gc->range.end = 90.0;
   gc->restraintFuncActive = yes;
   gc->used_in_model = no;
   gc->used_in_loop = no;
   gc->used_in_constraint = no;
   gc->slider_visible = yes;
   gc->pd_stiffness = 0.0;

   gc->restraint_user_num = 0;
   gc->restraint_func_num = -1;
   gc->restraint_sdcode_num = -1;
   gc->min_restraint_user_num = 0;
   gc->min_restraint_func_num = -1;
   gc->min_restraint_sdcode_num = -1;
   gc->max_restraint_user_num = 0;
   gc->max_restraint_func_num = -1;
   gc->max_restraint_sdcode_num = -1;

#if INCLUDE_MOCAP_MODULE
   gc->mocap_segment = NULL;
   gc->mocap_seg_index = -1;
   gc->mocap_column = -1;
   gc->mocap_adjust = 0.0;
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
      if (ms->gencoord[i].default_value >= ms->gencoord[i].range.start &&
         ms->gencoord[i].default_value <= ms->gencoord[i].range.end)
         ms->gencoord[i].value = ms->gencoord[i].default_value;
      else if (ms->gencoord[i].range.start > 0.0 ||
         ms->gencoord[i].range.end < 0.0)
         ms->gencoord[i].value = ms->gencoord[i].range.start;
      else
         ms->gencoord[i].value = 0.0;

      ms->gencoord[i].velocity = 0.0;

#ifndef ENGINE
      storeDoubleInForm(&ms->gencform.option[i], ms->gencoord[i].value, 3);
#endif

      ms->gencoord[i].numjoints = 0;
      ms->gencoord[i].jointnum = (int*)simm_malloc(ms->numjoints*sizeof(int));
      if (ms->gencoord[i].jointnum == NULL)
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

   int i, j, k, en, genc;
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
      ms->gencoord[i].numjoints = 0;

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
                  genc = adof->gencoord;
                  en = ms->gencoord[genc].numjoints++;
                  ms->gencoord[genc].jointnum[en] = i;
               }
         }
      }
   }

   for (i = 0; i < ms->numgencoords; i++)
      determine_gencoord_type(ms, i);

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

void determine_gencoord_type(ModelStruct* ms, int gc)
{
   int j, k, dofnum, jointnum;
   GencoordType gencoord_type;

   if (mark_unconstrained_dof(ms, gc, &jointnum, &dofnum))
   {
      if (dofnum == R1 || dofnum == R2 || dofnum == R3)
         ms->gencoord[gc].type = rotation_gencoord;
      else if (dofnum == TX || dofnum == TY || dofnum == TZ)
         ms->gencoord[gc].type = translation_gencoord;
   }
   else
   {
      gencoord_type = translation_gencoord;
      for (j = 0; j < ms->gencoord[gc].numjoints; j++)
      {
         jointnum = ms->gencoord[gc].jointnum[j];
         for (k = R1; k <= R3; k++)
         {
            if (ms->joint[jointnum].dofs[k].type == function_dof &&
                ms->joint[jointnum].dofs[k].gencoord == gc)
            {
               gencoord_type = rotation_gencoord;
               j = ms->gencoord[gc].numjoints; // to break out of outer loop
               break;
            }
         }
      }
      ms->gencoord[gc].type = gencoord_type;
   }
}


int mark_unconstrained_dof(ModelStruct* ms, int gc, int* jnt, int* dof)
{
   int i, j, fnum;
   double slope;
   SplineFunction* func;

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
	          ms->joint[i].dofs[j].gencoord == gc)
	      {
	         fnum = ms->joint[i].dofs[j].funcnum;
            func = &ms->function[fnum];
	         if (func->numpoints == 2 && EQUAL_WITHIN_ERROR(func->x[0], func->y[0]))
	         {
               slope = (func->y[1] - func->y[0]) / (func->x[1] - func->x[0]);
               if (EQUAL_WITHIN_ERROR(slope, 1.0))
               {
	               ms->joint[i].dofs[j].sd.constrained = no;
		            ms->joint[i].dofs[j].sd.conversion_sign = 1.0;
                  *jnt = i;
                  *dof = j;
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
                  return 1;
               }
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
      jointstr->dofs[i].funcnum = -1;
      jointstr->dofs[i].gencoord = -1;
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


void initwrapobject(WrapObject* wo)
{
   wo->name = NULL;
   wo->wrap_type = wrap_sphere;
   wo->wrap_algorithm = WE_HYBRID_ALGORITHM;
   wo->segment = 0;
   wo->display_list = -1;
   wo->display_list_is_stale = no;
   wo->active = yes;               /* set wrap object fields to default values */
   wo->visible = yes;
   wo->show_wrap_pts = no;
   wo->radius.xyz[0] = 0.05;
   wo->radius.xyz[1] = 0.1;
   wo->radius.xyz[2] = 0.05;
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
   co->display_list = -1;
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
   pt->visible = yes;
   pt->active = yes;
}

#ifndef ENGINE

static void make_confirm_menu(void)
{

   Menu* ms;

   ms = &root.confirm_menu;

   ms->title = NULL;
   ms->x_edge = 2;
   ms->y_edge = 2;
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


void get_environment_vars(void)
{

   const char* tmpptr;

#ifdef WIN32
   strcpy(buffer,get_simm_resources_directory());
   mstrcpy(&root.simm_dir,buffer);

   strcpy(buffer,get_simm_resources_directory());
   mstrcpy(&root.color_dir,buffer);

   strcpy(buffer,get_simm_resources_directory());
   strcat(buffer,"help\\");
   mstrcpy(&root.help_dir,buffer);

   strcpy(buffer,get_simm_resources_directory());
   strcat(buffer,"mocap\\");
   mstrcpy(&root.mocap_dir,buffer);
#else
   /* Get environment variables which contain the paths for certain SIMM files */
   char cwd_buf[CHARBUFFER];

   getcwd(cwd_buf, sizeof(cwd_buf));

   if ((tmpptr = getenv("SIMMDIR")) == NULL)
      (void)strcpy(buffer,cwd_buf);
   else
      (void)strcpy(buffer,tmpptr);
   (void)strcat(buffer,"/");
   mstrcpy(&root.simm_dir,buffer);

   if ((tmpptr = getenv("COLORDIR")) == NULL)
      (void)strcpy(buffer,cwd_buf);
   else
      (void)strcpy(buffer,tmpptr);
   (void)strcat(buffer,"/");
   mstrcpy(&root.color_dir,buffer);

   if ((tmpptr = getenv("HELPDIR")) == NULL)
      (void)strcpy(buffer,"help/");
   else
   {
      (void)strcpy(buffer,tmpptr);
      (void)strcat(buffer,"/");
   }
   mstrcpy(&root.help_dir,buffer);

   if ((tmpptr = getenv("MOCAPDIR")) == NULL)
      (void)strcpy(buffer,"mocap/");
   else
   {
      (void)strcpy(buffer,tmpptr);
      (void)strcat(buffer,"/");
   }
   mstrcpy(&root.mocap_dir,buffer);
#endif

   if ((tmpptr = getpref("SET_TOOLS_TO_NEW_MODEL")) == NULL)
      root.pref.set_tools_to_new_model = no;
   else
      root.pref.set_tools_to_new_model = yes;

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


const char* get_simm_directory()
{
   static char  simm_buf[512];
   static char* simm_directory = NULL;

   if (simm_directory == NULL)
   {
      getcwd(simm_buf, sizeof(simm_buf));
      simm_directory = simm_buf;
   }
   return simm_directory;
}

const char* get_simm_resources_directory()
{
   static char  simm_resources_buf[512];
   static char* simm_resources_dir = NULL;

#if !defined(WIN32) && !defined(__linux__)
   return root.simm_dir;
#else

   if (simm_resources_dir == NULL)
   {
      strcpy(simm_resources_buf, get_simm_directory());
      simm_resources_dir = simm_resources_buf;

      strcat(simm_resources_dir, "\\resources\\");
   }
   return simm_resources_dir;
#endif
}

#ifndef ENGINE
static void make_mocap_model_filepath(void)
{
   const char* p = getpref("MOCAP_MODEL");

   if (p)
   {
      if (is_absolute_path(p))
      {
         strcpy(buffer, p);
      }
      else
      {
         strcpy(buffer, root.simm_base_dir);
         append_if_necessary(buffer, DIR_SEP_CHAR);
         strcat(buffer, p);
      }
   }
   else
   {
      strcpy(buffer, root.simm_base_dir);
      append_if_necessary(buffer, DIR_SEP_CHAR);
      strcat(buffer, "resources\\mocap\\mocap.jnt");
   }

   mstrcpy(&root.mocap_model, buffer);
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
   
#ifdef WIN32
   sprintf(buffer, "%sSimmState.txt", get_simm_resources_directory());
#else
   sprintf(buffer, "%s.simmState.txt", get_simm_resources_directory());
#endif
   
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
