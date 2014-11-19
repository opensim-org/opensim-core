/*******************************************************************************

   MATHTOOLS.H

   Author: Peter Loan

   Date: 28-OCT-98

   Copyright (c) 1998 MusculoGraphics, Inc.
   All rights reserved.

*******************************************************************************/

void   append_4x4matrix(double mat1[][4], double mat2[][4]);
void   make_translation_matrix(double mat[][4], int axis, double trans);
void   make_scale_matrix(double mat[][4], int axis, double scale);
int    get_array_length(int list[]);
void   mult_4x4matrix_by_vector(double mat[][4], double vector[], double result[]);
void   mult_4x4matrices(double mat1[][4], double mat2[][4], double result[][4]);
void   transpose_4x4matrix(double mat[][4], double mat_transpose[][4]);
void   cross_vectors(double vector1[], double vector2[], double result[]);
double normalize_vector(double vector[], double norm_vector[]);
double normalize_vector_f(float vector[], float norm_vector[]);
void   format_double(double number, char format[]);
void   reset_4x4matrix(double mat[][4]);
void   make_4x4dircos_matrix(double angle, double axis[], double mat[][4]);
SBoolean vector_intersects_polyhedron(double pt[], double vec[], PolyhedronStruct* ph, double inter[]);
SBoolean intersect_line_plane(double pt1[], double pt2[],
                              double plane[], double d, double inter[], double* t);
SBoolean intersect_line_plane01(double pt1[], double pt2[], 
                double plane[], double d, double inter[], double* t);
SBoolean point_in_polygon3D(double pt[], PolyhedronStruct* ph, int polygon_index);
int    point_in_polygon2D(double point[], double pgon[][2], int numverts);
int    polygon_ray_inter3d(PolyhedronStruct* newph, int poly_index, double pt[3], int axes);
int    polygon_ray_inter_jordanstheorem(double** poly_pts, int numpts,
                    double ptray[3], int axes);
int point_ray_relation(double* pt, double ptray[], int axes);
void   make_rotation_matrix(double** mat, double normal[]);
int    intersect_lines(double p1[], double p2[], double p3[], double p4[],
               double p_int1[], double* t, double p_int2[], double* s);
int    intersect_lines_scaled(double p1[], double p2[], double p3[], double p4[],
                  double p_int1[], double* t, double* mag1,
                  double p_int2[], double* s, double* mag2);
void clear_vector(double a[], int n);
void   mult_3x3matrix_by_vector(double mat[][3], double vector[], double result[]);
void make_3x3_xrot_matrix(double a, double m[][3]);
void make_3x3dircos_matrix(double angle, double axis[], double mat[][3]);
void reset_3x3matrix(double matrix[][3]);
void mult_3x3matrices(double mat1[][3], double mat2[][3], double result[][3]);
void transpose_3x3matrix(double mat[][3], double mat_transpose[][3]);
void copy_3x3matrix(double from[][3], double to[][3]);
void mult_3x3_by_vector(double mat[][3], double vec[]);
int intersect_lines(double p1[], double p2[], double p3[], double p4[],
            double p_int1[], double* t, double p_int2[], double* s);
void find_plane_normal_to_line(PlaneStruct* plane, double pt1[], double pt2[]);
double compute_angle_between_vectors(double vector1[], double vector2[]);
void project_point_onto_plane(double pt[], PlaneStruct* plane, double projpt[]);
double distance_between_vertices(double vertex1[], double vertex2[]);
double distancesqr_between_vertices(double vertex1[], double vertex2[]);
double get_distsqr_point_line(double point[], double pl[], double vl[]);
void get_point_from_point_line(double point[], double pt[], double vec[],
                   double closest_pt[]);
double atan3(double y, double x);
void identity_matrix(double m[][4]);
void x_rotate_matrix_bodyfixed(double m[][4], double radians);  /* body-fixed rotation */
void y_rotate_matrix_bodyfixed(double m[][4], double radians);
void z_rotate_matrix_bodyfixed(double m[][4], double radians);
void x_rotate_matrix_spacefixed(double m[][4], double radians);
void y_rotate_matrix_spacefixed(double m[][4], double radians);
void z_rotate_matrix_spacefixed(double m[][4], double radians);
void rotate_matrix_axis_angle(double m[][4], const double* axis, double angle);
void translate_matrix(double m[][4], const double* delta);
void scale_matrix(double m[][4], const double* scaleBy);
void append_matrix(DMatrix m, const DMatrix b);
void transform_pt(double m[][4], double* pt);
void transform_vec(double m[][4], double* vec);
void extract_rotation(double m[][4], dpCoord3D* axis, double* angle);
void extract_xyz_rot_spacefixed(double m[][4], double xyz_rot[3]);
void extract_xyz_rot_bodyfixed(double m[][4], double xyz_rot[3]);
void lerp_pt(double start[3], double end[3], double t, double result[3]);
void slerp(const dpCoord3D* axisStart, double angleStart,
           const dpCoord3D* axisEnd,   double angleEnd,
           double t, dpCoord3D* axisResult, double* angleResult);

