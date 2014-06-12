/*******************************************************************************

   EVENTS.H

   Author: Peter Loan

   Date: 25-JUN-91

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

*******************************************************************************/


/* Base codes (used by tools to form event masks) */

/* OBJECTS */
#define MODEL_OBJECT            0x80000000
#define PLOT_OBJECT             0x40000000
#define TOOL_OBJECT             0x20000000
#define SCENE_OBJECT            0x10000000
#define ALL_OBJECTS             0xf0000000

/* OBJECT ACTIONS */
#define OBJECT_DEACTIVATED      0x0a000000
#define OBJECT_ACTIVATED        0x09000000
#define RIGHT_CLICK_EVENT       0x08000000
#define DRAW_EVENT              0x07000000
#define OBJECT_DESELECTED       0x06000000
#define OBJECT_SELECTED         0x05000000
#define OBJECT_ADDED            0x04000000
#define OBJECT_DELETED          0x03000000
#define OBJECT_CHANGED          0x02000000
#define INPUT_EVENT             0x01000000
#define ALL_ACTIONS             0x0f000000

/* SUB-OBJECTS */
#define NAME_SUB_OBJECT         0x00000001
#define MOTION_SUB_OBJECT       0x00000002
#define MUSCLE_SUB_OBJECT       0x00000004
#define MUSCLE_POINT_SUB_OBJECT 0x00000008
#define MODEL_SUB_OBJECT        0x00800000
#define PLOT_SUB_OBJECT         0x00400000
#define TOOL_SUB_OBJECT         0x00200000
#define SCENE_SUB_OBJECT        0x00100000
#define JOINT_SUB_OBJECT        0x00000010
#define SEGMENT_SUB_OBJECT      0x00000011
#define BONE_SUB_OBJECT         0x00000012
#define WRAP_SUB_OBJECT         0x00000013
#define CONSTRAINT_SUB_OBJECT   0x00000014
#define MARKER_SUB_OBJECT       0x00000015
#define GENCOORD_SUB_OBJECT     0x00000016
#define FUNCTION_SUB_OBJECT     0x00000017
#define ALL_SUB_OBJECTS         0x00ffffff

/* SIMM event codes (as listed in the SIMM event queue) */
#define ALL_INPUT_EVENTS          0xf1000000
#define MODEL_INPUT_EVENT         0x81800000
#define PLOT_INPUT_EVENT          0x41400000
#define TOOL_INPUT_EVENT          0x21200000
#define SCENE_INPUT_EVENT         0x11100000
#define SCENE_RIGHT_CLICK_EVENT   0x18100000
#define MODEL_ADDED               0x84800000
#define MODEL_DELETED             0x83800000
#define MODEL_CHANGED             0x82800000
#define PLOT_ADDED                0x44400000
#define PLOT_DELETED              0x43400000
#define SCENE_ADDED               0x14100000
#define SCENE_DELETED             0x13100000
#define MODEL_NAME_CHANGED        0x82000001
#define PLOT_NAME_CHANGED         0x42000001
#define MOTION_ADDED              0x84000002
#define MOTION_DELETED            0x83000002
#define MOTION_CHANGED            0x82000002
#define MOTION_ACTIVATED          0x89000002
#define MOTION_DEACTIVATED        0x8a000002
#define MUSCLE_DESELECTED         0x86000004
#define MUSCLE_SELECTED           0x85000004
#define MUSCLE_ADDED              0x84000004
#define MUSCLE_DELETED            0x83000004
#define MUSCLE_CHANGED            0x82000004
#define MUSCLE_POINT_DESELECTED   0x86000008
#define MUSCLE_POINT_SELECTED     0x85000008
#define MUSCLE_POINT_ADDED        0x84000008
#define MUSCLE_POINT_DELETED      0x83000008
#define MUSCLE_POINT_CHANGED      0x82000008
#define JOINT_ADDED               0x84000010
#define JOINT_DELETED             0x83000010
#define JOINT_CHANGED             0x82000010
#define SEGMENT_ADDED             0x84000011
#define SEGMENT_DELETED           0x83000011
#define SEGMENT_CHANGED           0x82000011
#define BONE_ADDED                0x84000012
#define BONE_DELETED              0x83000012
#define BONE_CHANGED              0x82000012
#define WRAP_OBJECT_ADDED         0x84000013
#define WRAP_OBJECT_DELETED       0x83000013
#define WRAP_OBJECT_CHANGED       0x82000013
#define CONSTRAINT_OBJECT_ADDED   0x84000014
#define CONSTRAINT_OBJECT_DELETED 0x83000014
#define CONSTRAINT_OBJECT_CHANGED 0x82000014
#define MARKER_ADDED              0x84000015
#define MARKER_DELETED            0x83000015
#define MARKER_CHANGED            0x82000015
#define GENCOORD_ADDED            0x84000016
#define GENCOORD_DELETED          0x83000016
#define GENCOORD_CHANGED          0x82000016
#define FUNCTION_ADDED            0x84000017
#define FUNCTION_DELETED          0x83000017
#define FUNCTION_CHANGED          0x82000017
#define DRAW_MODEL_EVENT          0x87800000
#define DRAW_TOOL_EVENT           0x27200000
