/*******************************************************************************

   DEFUNCTIONS.H

   Author: Kenny Smith

   Date: 22-OCT-98

   Copyright (c) 1998 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

*******************************************************************************/

#ifndef DEFUNCTIONS_H
#define DEFUNCTIONS_H

void make_deformeditor(int rootWindowX, int rootWindowY, SBoolean iconified);
void de_simm_event_handler(SimmEvent);
SBoolean de_query_handler(QueryType, void* data);
void display_deformeditor(WindowParams*, WinUnion*);
void update_deformeditor(WindowParams*, WinUnion*);
void deformeditor(WindowParams*, WinUnion*, SimmEvent);
void slide_de(int arg1, double value, double delta);

void init_deform(DeformObject*);
void init_deformity(Deformity*);

void makedeformeditormenus();
void de_entervalue(SimmEvent);
void de_enter_deformity_value(SimmEvent);
void update_de_forms();

void de_track_cb(void* data, SimmEvent se);
void deformity_slider_cb(int sliderIndex, double value, double delta);

void deform_segment(ModelStruct*, int segment);
static SBoolean deform_joint_pt (ModelStruct* ms, JointStruct* jnt, double* pt);
void deform_vert (DeformObject* dfm, const double undeformed[3], double pt_to_deform[3], SBoolean* didDeform);

void calc_joint_pretransform(ModelStruct*, JointStruct*);

SBoolean is_current_deform(DeformObject*);
void select_deform(int deformobj);

void reset_deform_xform();
void recalc_deform_xforms(SegmentStruct*, DeformObject*);

void apply_xform_to_deform(double factor);
void clear_de_xform_form();

XForm* get_deform_xform(DeformObject*, int deformMode);
XForm* get_deform_xform2(DeformObject*, int deformMode, double factor);
XForm* get_deform_xform3(DeformObject*, int deformMode, double factor);

void save_all_deform_objects(ModelStruct* ms);
void restore_all_deform_objects(ModelStruct* ms);
void delete_deform_object(int deformobj);

void do_de_help();
void draw_de_help_window(WindowParams*, WinUnion*);
void de_help_input(WindowParams*, WinUnion*, SimmEvent);
void move_de_help_text(int dummy_int, double slider_value, double delta);

void draw_deform_objects(ModelStruct* model, SegmentStruct* seg, int segment_index, ModelDrawOptions* mdo);

DeformObject* lookup_deform(ModelStruct*, const char* deformName);


void init_deform_box_verts(DeformObject*);

#define _SET_VERT(XYZ, _X, _Y, _Z) (XYZ[0] = _X, XYZ[1] = _Y, XYZ[2] = _Z)
#define _COPY_VERT(DST, SRC)       ((DST)[0] = (SRC)[0], (DST)[1] = (SRC)[1], (DST)[2] = (SRC)[2])
#define _MAKE_VEC(C, B, A)         ((C)[0] = (B)[0] - (A)[0], (C)[1] = (B)[1] - (A)[1], (C)[2] = (B)[2] - (A)[2])
#define _SCALE_VEC(V, T)           ((V)[0] *= (T), (V)[1] *= (T), (V)[2] *= (T))
#define _ADD_VEC(PT, V)            ((PT)[0] += (V)[0], (PT)[1] += (V)[1], (PT)[2] += (V)[2])


#define SEGMENTS_PER_BOX_EDGE 3   /* number of segments per box edge (was 15) */

#define VERTS_PER_BOX_EDGE (SEGMENTS_PER_BOX_EDGE + 1)
#define VERTS_PER_BOX      (VERTS_PER_BOX_EDGE * 12)

#endif /*DEFUNCTIONS_H*/
