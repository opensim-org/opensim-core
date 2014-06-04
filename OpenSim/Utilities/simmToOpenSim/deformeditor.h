/*******************************************************************************

   DEFORMEDITOR.H

   Author: Kenny Smith

   Date: 19-FEB-99

   Copyright (c) 1992-8 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

*******************************************************************************/

#ifndef DEFORMEDITOR_H
#define DEFORMEDITOR_H

enum {
   DE_CHOOSE_SEGMENT = 0,
   DE_DEFORM_OBJECT,
   DE_SAVE_ALL,
   DE_RESTORE_ALL,
   DE_DELETE_OBJECT,
   DE_APPLY_POS_XFORM,
   DE_APPLY_NEG_XFORM,
   DE_CLEAR_XFORM,
   DE_IDENTITY_XFORM,

   DE_OBJECT_NAME = 0,
   DE_INNER_MIN_X,
   DE_INNER_MIN_Y,
   DE_INNER_MIN_Z,
   DE_INNER_MAX_X,
   DE_INNER_MAX_Y,
   DE_INNER_MAX_Z,
   DE_OUTER_MIN_X,
   DE_OUTER_MIN_Y,
   DE_OUTER_MIN_Z,
   DE_OUTER_MAX_X,
   DE_OUTER_MAX_Y,
   DE_OUTER_MAX_Z,
   DE_TRANSLATE_X,
   DE_TRANSLATE_Y,
   DE_TRANSLATE_Z,
   DE_ROTATE_X,
   DE_ROTATE_Y,
   DE_ROTATE_Z,
   
   DE_POSITION_RBTN = 0,
   DE_DEFORM_START_RBTN,
   DE_DEFORM_END_RBTN,

   DE_ACTIVE_CHBOX = 0,
   DE_VISIBLE_CHBOX,
   DE_AUTORESET_CHBOX,
   DE_TRACKBALL_CHBOX,

   DE_LOCAL_FRAME_RBTN = 0,
   DE_PARENT_FRAME_RBTN,

   DE_SLIDER_WIDTH = 16,
   
   DEFORM_ARRAY_INCREMENT = 2,

   DE_PICK_DEFORM_KEY = 0,
   DE_MOVE_DEFORM_KEY,
   DE_PAN_DEFORM_KEY,
   DE_ZOOM_DEFORM_KEY,
   DE_TRACKBALL_KEY,
   DE_ROTATE_X_KEY,
   DE_ROTATE_Y_KEY,
   DE_ROTATE_Z_KEY,
   DE_MAX_DEVS
};

enum { DE_POSITION_MODE, DE_DEFORM_START_MODE, DE_DEFORM_END_MODE, DE_SLIDER_MODE };

enum { DE_LOCAL_FRAME, DE_PARENT_FRAME };

STRUCT {
   int         segment;                /* current segment */
   int         deform;                 /* index of current wrapping object */
   int         deformMode;             /* mode = position or deform */
   dpCoord3D     translate;
   dpCoord3D     rotate;
   int         xform_frame;
   SBoolean    trackball_rotation;
   Form        deformity_form;
   SliderArray deformity_sliders;
} DEModelOptions;                      /* user-selectable plotting options */

ENUM {
   DE_TOP_LEVEL,                       /*  */
   DE_ENTER_VALUE,                     /* deform object form field mode */
   DE_DEFORMITY_ENTER_VALUE            /* deformity form field mode */
} DEMODE;                              /*  */

STRUCT {
   ModelStruct*  model;                /*  */
   DEMODE        current_mode;         /*  */
   int           selected_item;        /*  */
   Menu          pushbtns;             /*  */
   Form          textfields;           /*  */
   CheckBoxPanel deform_radiopanel;
   CheckBoxPanel active_visible_checkpanel;
   SliderArray   deform_slider;
   CheckBoxPanel transform_radiopanel;
   DEModelOptions deop[MODELBUFFER];   /*  */
   HelpStruct    help;                 /*  */
   int           reference_number;     /*  */
   Slider        win_slider;           /*  */
   int           canvas_height;        /*  */
   int           deform_object_menu;
   int           numdevs;              /* number of device numbers in devs[] */
   int           devs[DE_MAX_DEVS];    /* button numbers to move bone vertices */
} DeformEditorStruct;                  /*  */

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
} DeformEditorTracker;

#endif /* DEFORMEDITOR_H */
