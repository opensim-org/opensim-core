/*******************************************************************************

   WEFUNCTIONS.H - this header declares the Wrap Editor's public functions
      as well as the 3 wrapping algorithm public functions:
      
      calc_line_intersect_ellipsoid
      calc_line_intersect_cylinder
      calc_line_intersect_sphere

   Author: Kenny Smith (based on pmfunctions.h by Peter Loan)

   Date: 22-OCT-98

   Copyright (c) 1998 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

*******************************************************************************/

#ifndef WEFUNCTIONS_H
#define WEFUNCTIONS_H

void   makewrapeditormenus(void);
void   we_entervalue(SimmEvent se);
void   update_we_forms(void);
void   move_we_help_text(int dummy_int, double slider_value, double delta);

void   we_track_cb(void* data, SimmEvent se);

void inval_model_wrapping(ModelStruct* model, dpWrapObject* wrap_object);

int    get_wrap_object_index(ModelStruct* model, dpWrapObject* wrap_object);
int    is_current_wrap_object(ModelStruct*, dpWrapObject*);
void   select_wrapping_object(int wrapobj, SBoolean redisplay);

void   convert_to_wrap_object_frame(dpWrapObject*, double* pt);
void   convert_from_wrap_object_frame(dpWrapObject*, double* pt);

void   reset_wrapobj_xform();
void   recalc_xforms(dpWrapObject*);

void   apply_xform_to_wrapobj(double factor);
void   clear_we_xform_form();

void   save_all_wrap_objects(ModelStruct* ms);

SBoolean query_muscle_wrap_association(dpMuscleStruct* muscle, dpWrapObject* wrap_object);
void     add_muscle_wrap_association(dpMuscleStruct* muscle, MuscWrapAssoc* mwa, dpWrapObject* wrap_object);
void     remove_muscle_wrap_association(dpMuscleStruct* muscle, dpWrapObject* wrap_object);

void   update_we_win_status();

#define VISUAL_WRAPPING_DEBUG      0
#define USER_SPECIFIED_WRAP_METHOD 1

/* wrapping algorithms
 */
#define WE_HYBRID_ALGORITHM    0   /* Frans + fan algorithm */
#define WE_MIDPOINT_ALGORITHM  1   /* midpoint algorithm    */
#define WE_AXIAL_ALGORITHM     2   /* Frans only algorithm  */
#define WE_NUM_WRAP_ALGORITHMS 3

#define WE_FAN_ALGORITHM       4   /* fan only algorithm    */

const char* get_wrap_type_name (int i);
const char* get_wrap_algorithm_name(int wrap_algorithm);

int calc_line_intersect_ellipsoid(double p1[], double p2[],
                                  double m[], double a[],
                                  double *rlen, double r1[], double r2[],
                                  double** wrap_pts, int *num_wrap_pts,
                                  int wrap_axis, int wrap_sign,
                                  dpMuscleWrapStruct*, int* p_flag, dpWrapObject*);

int calc_line_intersect_cylinder(double p1[], double p2[],
                                 double p0[], double dn[], double r, double len,
                                 double *rlen, double r1[], double r2[],
                                 double** wrap_pts, int *num_wrap_pts,
                                 int wrap_axis, int wrap_sign,
                                 dpMuscleWrapStruct*, int* p_flag, dpWrapObject*);

int calc_line_intersect_sphere(double p1[], double p2[],
                               double m[], double r,
                               double *rlen, double r1[], double r2[],
                               double** wrap_pts, int *num_wrap_pts,
                               int wrap_axis, int wrap_sign,
                               dpMuscleWrapStruct*, int* p_flag, dpWrapObject*);

int calc_line_intersect_torus(double p1[], double p2[],
                              double m[], double a[],
                              double *rlen, double r1[], double r2[],
                              double** wrap_pts, int *num_wrap_pts,
                              int wrap_axis, int wrap_sign,
                              dpMuscleWrapStruct*, int* p_flag, dpWrapObject*);

void enable_debug_shapes(SBoolean);

#if VISUAL_WRAPPING_DEBUG

void add_debug_point(
    dpWrapObject* wo, double factor,
    double pt[], float radius, const char* name, const float* color);
    
void add_debug_line(
    dpWrapObject* wo, double factor,
    double pt1[], double pt2[], float lineWidth,
    const char* name1, const char* name2, const float* color);

float* lerp_clr(const float start[3], const float end[3], double t, float color[3]);

#endif /* VISUAL_WRAPPING_DEBUG */

#endif /* WEFUNCTIONS_H */
