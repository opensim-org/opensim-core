/*******************************************************************************

   MATHTOOLS.H

   Author: Peter Loan

   Copyright (c) 1998-2004 MusculoGraphics, a division of Motion Analysis Corp.
   All rights reserved.

*******************************************************************************/

int    find_end_of_array(int list[], int* length);
void   mult_4x4matrix_by_vector(double mat[][4], double vector[], double result[]);
void   mult_4x4matrices(double mat1[][4], double mat2[][4], double result[][4]);
void   cross_vectors(double vector1[], double vector2[], double result[]);
double normalize_vector(double vector[], double norm_vector[]);
void   calc_spline_coefficients(dpSplineFunction* f);
double interpolate_spline(double abscissa, dpSplineFunction* func, Derivative deriv,
                          double velocity, double acceleration);
void   format_double(double number, char format[]);
void   reset_4x4matrix(double mat[][4]);
void   make_4x4dircos_matrix(double angle, double axis[], double mat[][4]);
dpBoolean intersect_line_plane01(double pt1[], double pt2[], 
                                 double plane[], double d, double inter[], double* t);
dpBoolean intersect_line_plane(double pt1[], double pt2[], 
                               double plane[], double d, double inter[], double* t);
dpBoolean point_in_polygon3D(double pt[], dpPolyhedronStruct* ph, int polygon_index);
int point_in_polygon2D(double point[], double pgon[][2], int numverts);
void make_rotation_matrix(double** mat, double normal[]);
int intersect_lines(double p1[], double p2[], double p3[], double p4[],
                    double p_int1[], double* t, double p_int2[], double* s);
int intersect_lines_scaled(double p1[], double p2[], double p3[], double p4[],
                           double p_int1[], double* t, double* mag1,
                           double p_int2[], double* s, double* mag2);
void clear_vector(double a[], int n);
void mult_3x3matrix_by_vector(double mat[][3], double vector[], double result[]);
void make_3x3_xrot_matrix(double a, double m[][3]);
void make_3x3dircos_matrix(double angle, double axis[], double mat[][3]);
void reset_3x3matrix(double matrix[][3]);
void mult_3x3matrices(double mat1[][3], double mat2[][3], double result[][3]);
void transpose_3x3matrix(double mat[][3], double mat_transpose[][3]);
void copy_3x3matrix(double from[][3], double to[][3]);
void mult_3x3_by_vector(double mat[][3], double vec[]);
int intersect_lines(double p1[], double p2[], double p3[], double p4[],
                    double p_int1[], double* t, double p_int2[], double* s);
double compute_angle_between_vectors(double vector1[], double vector2[]);
double project_point_onto_plane(double pt[], dpPlaneStruct* plane, double projpt[]);
double distance_between_vertices(double vertex1[], double vertex2[]);
double distancesqr_between_vertices(double vertex1[], double vertex2[]);
double get_distsqr_point_line(double point[], double pl[], double vl[]);
void get_point_from_point_line(double point[], double pt[], double vec[],
                               double closest_pt[]);
void rotate_matrix_axis_angle(double m[][4], const double* axis, double angle);
void translate_matrix(double m[][4], double* delta);
/*void append_matrix(DMatrix m, const DMatrix b);*/
void append_matrix(DMatrix m, DMatrix b);

void transform_pt(double m[][4], double* pt);
void transform_vec(double m[][4], double* vec);
void extract_rotation(double m[][4], Coord3D* axis, double* angle);
void extract_xyz_rot_spacefixed(double m[][4], double xyz_rot[3]);
void extract_xyz_rot_bodyfixed(double m[][4], double xyz_rot[3]);
dpBoolean vect_equal_within_tol(double a[3], double b[3], double tol);
void make_3x3matrix(double m[3][3], double m11, double m12, double m13, 
                    double m21, double m22, double m23, double m31, double m32, double m33);
void make_vector(double vec[3], double x, double y, double z);
dpBoolean zero_3x3matrix(double m[][3]);
