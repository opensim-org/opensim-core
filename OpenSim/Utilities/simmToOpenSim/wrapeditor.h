/*******************************************************************************

   WRAPEDITOR.H

   Author: Kenny Smith (based on plotmaker.h by Peter Loan)

   Date: 22-OCT-98

   Copyright (c) 1992-8 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

*******************************************************************************/

#ifndef WRAPEDITOR_H
#define WRAPEDITOR_H

#define WE_WRAP_OBJECT     0
#define WE_CHOOSE_SEGMENT  1
#define WE_MUSCLEGROUPS    2
#define WE_SAVE_ALL    3
#define WE_RESTORE_ALL 4
#define WE_DELETE_OBJECT   5
#define WE_APPLY_POS_XFORM 6
#define WE_APPLY_NEG_XFORM 7
#define WE_CLEAR_XFORM     8
#define WE_IDENTITY_XFORM  9

#define WE_OBJECT_NAME 0
#define WE_RADIUS_X 1
#define WE_RADIUS_Y 2
#define WE_RADIUS_Z 3
#define WE_TRANSLATE_X 4
#define WE_TRANSLATE_Y 5
#define WE_TRANSLATE_Z 6
#define WE_ROTATE_X 7
#define WE_ROTATE_Y 8
#define WE_ROTATE_Z 9

#define WE_SPHERE 0     /* see note below! */
#define WE_CYLINDER 1
#define WE_ELLIPSOID 2
#define WE_TORUS 3
#define WE_NUM_WRAP_TYPE_RADIO_BTNS 4

/* NOTE: the radio-button enum above *must* match the dpWrapObjectType
 *  enum in modelplot.h!!
 */
 
#define WE_X_QUADRANT 0
#define WE_Y_QUADRANT 1
#define WE_Z_QUADRANT 2

#define WE_POSITIVE_QUADRANT 0
#define WE_NEGATIVE_QUADRANT 1

#define WE_ACTIVE_CHBOX      0
#define WE_VISIBLE_CHBOX     1
#define WE_SHOW_PTS_CHBOX    2
#define WE_TRACKBALL_CHBOX   3

#define WE_LOCAL_FRAME_RBTN  0
#define WE_PARENT_FRAME_RBTN 1

#define WE_SLIDER_WIDTH 16
#define WE_MGROUP_YOFFSET 410

#define WE_PICK_WRAP_OBJECT_KEY 0
#define WE_MOVE_WRAP_OBJECT_KEY 1
#define WE_PAN_WRAP_OBJECT_KEY  2
#define WE_ZOOM_WRAP_OBJECT_KEY 3
#define WE_TRACKBALL_KEY        4
#define WE_ROTATE_X_KEY         5
#define WE_ROTATE_Y_KEY         6
#define WE_ROTATE_Z_KEY         7
#define WE_MAX_DEVS             8

enum { WE_LOCAL_FRAME, WE_PARENT_FRAME };

STRUCT {
   int segment;                     /* parent segment of current wrapping object */
   dpWrapObject* wrap_object;       /* index of current wrapping object */
   dpCoord3D translate;
   dpCoord3D rotate;
   int xform_frame;
   SBoolean trackball_rotation;
   MuscleMenu mgroup[GROUPBUFFER];  /* list of muscle groups */
   int menucolumns[COLUMNBUFFER];   /* used for placing menus in the window */
   int* musc;                       /* list of muscle numbers */
   int nummuscles;                  /* number of muscles currently selected */
} WEModelOptions;                   /* user-selectable plotting options */

ENUM {
   WE_TOP_LEVEL,                       /*  */
   WE_ENTER_VALUE                      /*  */
} WEMODE;                              /*  */

STRUCT {
   ModelStruct*  model;                /*  */
   PlotStruct*   plot;                 /*  */
   WEMODE        current_mode;         /*  */
   int           selected_item;        /*  */
   Menu          optionsmenu;          /*  */
   Form          optionsform;          /*  */
   CheckBoxPanel wrap_type_radiopanel;
   CheckBoxPanel wrap_method_radiopanel;
   CheckBoxPanel quadrant_radiopanel;
   CheckBoxPanel quadrant_checkpanel;
   CheckBoxPanel active_visible_checkpanel;
   CheckBoxPanel transform_radiopanel;
   WEModelOptions weop[MODELBUFFER];   /*  */
   HelpStruct    help;                 /*  */
   int           reference_number;     /*  */
   Slider        win_slider;           /*  */
   int           canvas_height;        /*  */
   int           wrap_object_menu;
   int           numdevs;              /* number of device numbers in devs[] */
   int           devs[WE_MAX_DEVS];    /* button numbers to move bone vertices */
} WrapEditorStruct;                    /*  */

STRUCT {
   Scene* scene;
   ModelStruct* model;
   double       wx_old, wy_old, wz_old;
   int          mx_old, my_old;
   double       bpan_wx_old, bpan_wy_old, bpan_wz_old;
   int          bpan_mx_old, bpan_my_old;
   double       span_wx_old, span_wy_old, span_wz_old;
   int          span_mx_old, span_my_old;
   int          zoom_mx_old, zoom_my_old;
   dpCoord3D      zoom_vec;
} WrapEditorTracker;

#endif /*WRAPEDITOR_H*/
