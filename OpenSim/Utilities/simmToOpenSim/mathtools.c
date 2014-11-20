/*******************************************************************************

   MATHTOOLS.C

   Author: Peter Loan

   Date: 8-DEC-88

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Description: This file contains routines that implement various
      mathematical tools and functions. Every one operates exclusively
      on the formal parameter list, and so does not change any external
      variables or other general model or plot structures.

   Routines:
      get_array_length      : gets the length of an integer array
      mult_4x4matrix_by_vector : multiplies a vector by a matrix
      mult_4x4matrices      : multiplies two 4x4 matrices
      transpose_4x4matrix   : transposes a 4x4 matrix
      cross_vectors         : finds cros product of two vectors
      normalize_vector      : normalizes a vector
      calc_function_coefficients : finds coefficients of a function
      interpolate_function    : given an x, interpolates a cubic to get a y value
      format_double         : determines appropriate printf format for a number
      reset_4x4matrix       : sets a 4x4 matrix equal to the identity matrix
      make_4x4dircos_matrix : makes direction cosine matrix given angle & axis

*******************************************************************************/

#include "universal.h"
#include "functions.h"


/*************** DEFINES (for this file only) *********************************/
#define LINE_EPSILON 0.00001


/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/



/*************** EXTERNED VARIABLES (declared in another file) ****************/



/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static void quat_to_axis_angle_rot(const Quat, dpCoord3D* axis, double* angle);
static void matrix_to_quat(double m[][4], Quat);
static void make_quaternion(Quat, const double axis[3], double angle);
static void quat_to_matrix(const Quat q, double m[][4]);
static void rotate_matrix_by_quat(double m[][4], const Quat);


/* GET_ARRAY_LENGTH: get the length of an array of ints. The end is
 * marked by the END_OF_ARRAY constant.
 */
int get_array_length(int list[])
{
   int i = 0;

   while (list[i] != END_OF_ARRAY)
     i++;

   return i;
}


/* MULT_4X4MATRIX_BY_VECTOR: this routine premultiplies a 4x4 matrix by
 * a 1x4 vector. That is, it does vector*mat, not mat*vector.
 */

void mult_4x4matrix_by_vector(double mat[][4], double vector[], double result[])
{

   result[0] = vector[0]*mat[0][0] + vector[1]*mat[1][0] +
               vector[2]*mat[2][0] + vector[3]*mat[3][0];
   result[1] = vector[0]*mat[0][1] + vector[1]*mat[1][1] +
               vector[2]*mat[2][1] + vector[3]*mat[3][1];
   result[2] = vector[0]*mat[0][2] + vector[1]*mat[1][2] +
               vector[2]*mat[2][2] + vector[3]*mat[3][2];
   result[3] = vector[0]*mat[0][3] + vector[1]*mat[1][3] +
               vector[2]*mat[2][3] + vector[3]*mat[3][3];

}



/* MULT_4X4MATRICES: this routine multiplies the two specified 4x4 matrices
 * in the order: mat1*mat2.
 */

void mult_4x4matrices(double mat1[][4], double mat2[][4], double result[][4])
{

   result[0][0] = mat1[0][0]*mat2[0][0] + mat1[0][1]*mat2[1][0] +
      mat1[0][2]*mat2[2][0] + mat1[0][3]*mat2[3][0];

   result[0][1] = mat1[0][0]*mat2[0][1] + mat1[0][1]*mat2[1][1] +
      mat1[0][2]*mat2[2][1] + mat1[0][3]*mat2[3][1];

   result[0][2] = mat1[0][0]*mat2[0][2] + mat1[0][1]*mat2[1][2] +
      mat1[0][2]*mat2[2][2] + mat1[0][3]*mat2[3][2];

   result[0][3] = mat1[0][0]*mat2[0][3] + mat1[0][1]*mat2[1][3] +
      mat1[0][2]*mat2[2][3] + mat1[0][3]*mat2[3][3];


   result[1][0] = mat1[1][0]*mat2[0][0] + mat1[1][1]*mat2[1][0] +
      mat1[1][2]*mat2[2][0] + mat1[1][3]*mat2[3][0];

   result[1][1] = mat1[1][0]*mat2[0][1] + mat1[1][1]*mat2[1][1] +
      mat1[1][2]*mat2[2][1] + mat1[1][3]*mat2[3][1];

   result[1][2] = mat1[1][0]*mat2[0][2] + mat1[1][1]*mat2[1][2] +
      mat1[1][2]*mat2[2][2] + mat1[1][3]*mat2[3][2];

   result[1][3] = mat1[1][0]*mat2[0][3] + mat1[1][1]*mat2[1][3] +
      mat1[1][2]*mat2[2][3] + mat1[1][3]*mat2[3][3];


   result[2][0] = mat1[2][0]*mat2[0][0] + mat1[2][1]*mat2[1][0] +
      mat1[2][2]*mat2[2][0] + mat1[2][3]*mat2[3][0];

   result[2][1] = mat1[2][0]*mat2[0][1] + mat1[2][1]*mat2[1][1] +
      mat1[2][2]*mat2[2][1] + mat1[2][3]*mat2[3][1];

   result[2][2] = mat1[2][0]*mat2[0][2] + mat1[2][1]*mat2[1][2] +
      mat1[2][2]*mat2[2][2] + mat1[2][3]*mat2[3][2];

   result[2][3] = mat1[2][0]*mat2[0][3] + mat1[2][1]*mat2[1][3] +
      mat1[2][2]*mat2[2][3] + mat1[2][3]*mat2[3][3];


   result[3][0] = mat1[3][0]*mat2[0][0] + mat1[3][1]*mat2[1][0] +
      mat1[3][2]*mat2[2][0] + mat1[3][3]*mat2[3][0];

   result[3][1] = mat1[3][0]*mat2[0][1] + mat1[3][1]*mat2[1][1] +
      mat1[3][2]*mat2[2][1] + mat1[3][3]*mat2[3][1];

   result[3][2] = mat1[3][0]*mat2[0][2] + mat1[3][1]*mat2[1][2] +
      mat1[3][2]*mat2[2][2] + mat1[3][3]*mat2[3][2];

   result[3][3] = mat1[3][0]*mat2[0][3] + mat1[3][1]*mat2[1][3] +
      mat1[3][2]*mat2[2][3] + mat1[3][3]*mat2[3][3];

}


/* APPEND_4X4MATRIX: multiplies mat1 by mat2 and stores the
 * result in mat1.
 */

void append_4x4matrix(double mat1[][4], double mat2[][4])
{

   int i, j;
   double mat[4][4];

   mult_4x4matrices(mat1,mat2,mat);

   for (i=0; i<4; i++)
      for (j=0; j<4; j++)
     mat1[i][j] = mat[i][j];

}


/* TRANSPOSE_4X4MATRIX: this routine transposes a 4x4 matrix */

void transpose_4x4matrix(double mat[][4], double mat_transpose[][4])
{

   int i, j;

   for (i=0; i<4; i++)
      for (j=0; j<4; j++)
         mat_transpose[j][i] = mat[i][j];

}


/* MAKE_TRANSLATION_MATRIX: makes a 4x4 matrix with either an X, Y, or Z
 * translation.
 */

void make_translation_matrix(double mat[][4], int axis, double trans)
{

   reset_4x4matrix(mat);

   mat[3][axis] = trans;

}


/* MAKE_SCALE_MATRIX: makes a 4x4 matrix with either an X, Y, or Z
 * scale.
 */

void make_scale_matrix(double mat[][4], int axis, double scale)
{

   reset_4x4matrix(mat);

   mat[axis][axis] = scale;

}


/* CROSS_VECTORS: computes the cross product of two 1x3 vectors */

void cross_vectors(double vector1[], double vector2[], double result[])
{

   result[0] = vector1[1]*vector2[2] - vector1[2]*vector2[1];
   result[1] = vector1[2]*vector2[0] - vector1[0]*vector2[2];
   result[2] = vector1[0]*vector2[1] - vector1[1]*vector2[0];

}


/* NORMALIZE_VECTOR: normalizes a 1x3 vector. */
double normalize_vector(double vector[], double norm_vector[])
{
   double magnitude;

   magnitude = VECTOR_MAGNITUDE(vector);

   if (magnitude < TINY_NUMBER)
   {
      norm_vector[0] = vector[0];
      norm_vector[1] = vector[1];
      norm_vector[2] = vector[2];
   }
   else
   {
      norm_vector[0] = vector[0]/magnitude;
      norm_vector[1] = vector[1]/magnitude;
      norm_vector[2] = vector[2]/magnitude;
   }

   return magnitude;
}


/* NORMALIZE_VECTOR: normalizes a 1x3 vector. */
double normalize_vector_f(float vector[], float norm_vector[])
{
   float magnitude;

   magnitude = VECTOR_MAGNITUDE(vector);

   if (magnitude < TINY_NUMBER)
   {
      norm_vector[0] = vector[0];
      norm_vector[1] = vector[1];
      norm_vector[2] = vector[2];
   }
   else
   {
      norm_vector[0] = vector[0]/magnitude;
      norm_vector[1] = vector[1]/magnitude;
      norm_vector[2] = vector[2]/magnitude;
   }

   return magnitude;
}


/* FORMAT_DOUBLE: this routine finds a good format (printf style) for a given
 * number by checking its magnitude. For example, if the number is greater than
 * 50, you don't really need to show any decimal places, but if the number is
 * less than 0.01, you should show several decimal places.
 */

void format_double(double number, char format[])
{

   if (number > 50.0)
      (void)strcpy(format,"%.3lf");
   else if (number > 10.0)
      (void)strcpy(format,"%.4lf");
   else if (number > 1.0)
      (void)strcpy(format,"%.5lf");
   else if (number > 0.1)
      (void)strcpy(format,"%.6lf");
   else if (number > 0.01)
      (void)strcpy(format,"%.7lf");
   else
      (void)strcpy(format,"%.8lf");

}


/* RESET_4X4MATRIX: this routine sets a 4x4 matrix equal to the identity matrix */

void reset_4x4matrix(double mat[][4])
{

   mat[0][1] = mat[0][2] = mat[0][3] = 0.0;
   mat[1][0] = mat[1][2] = mat[1][3] = 0.0;
   mat[2][0] = mat[2][1] = mat[2][3] = 0.0;
   mat[3][0] = mat[3][1] = mat[3][2] = 0.0;

   mat[0][0] = mat[1][1] = mat[2][2] = mat[3][3] = 1.0;

}


/* MAKE_4X4DIRCOS_MATRIX: this routine creates a 4x4 direction cosine matrix
 * given an arbitrarily oriented axis and a rotation angle in degrees.
 */

void make_4x4dircos_matrix(double angle, double axis[], double mat[][4])
{

   double rad_angle, cl, sl, omc;

   reset_4x4matrix(mat);
   normalize_vector(axis,axis);

   rad_angle = angle*DTOR;

   cl = cos(rad_angle);
   sl = sin(rad_angle);
   omc = 1.0 - cl;

   /* the following matrix is taken from Kane's 'Spacecraft Dynamics,' pp 6-7 */

   mat[0][0] = cl + axis[XX]*axis[XX]*omc;
   mat[1][0] = -axis[ZZ]*sl + axis[XX]*axis[YY]*omc;
   mat[2][0] = axis[YY]*sl + axis[ZZ]*axis[XX]*omc;
   mat[0][1] = axis[ZZ]*sl + axis[XX]*axis[YY]*omc;
   mat[1][1] = cl + axis[YY]*axis[YY]*omc;
   mat[2][1] = -axis[XX]*sl + axis[YY]*axis[ZZ]*omc;
   mat[0][2] = -axis[YY]*sl + axis[ZZ]*axis[XX]*omc;
   mat[1][2] = axis[XX]*sl + axis[YY]*axis[ZZ]*omc;
   mat[2][2] = cl + axis[ZZ]*axis[ZZ]*omc;

}


/* VECTOR_INTERSECTS_POLYHEDRON: This function checks for intersection of a vector
 * and a polyhedron.
 */
SBoolean vector_intersects_polyhedron(double pt[], double vec[], PolyhedronStruct* ph, double inter[])
{
   int i;
   double pt2[3], t;

   pt2[0] = pt[0] + vec[0];
   pt2[1] = pt[1] + vec[1];
   pt2[2] = pt[2] + vec[2];

   /* Check to see if the vector intersects any of the polygons in the polyhedron.
    * intersect_line_plane() is used to get the point of intersection between the
    * vector and the plane of the polygon, and then this point is checked to see if
    * it is inside the boundaries of the polygon. Return the first intersection found.
    */
   for (i = 0; i < ph->num_polygons; i++)
   {
      if (intersect_line_plane(pt, pt2, ph->polygon[i].normal, ph->polygon[i].d, inter, &t) == yes &&
         point_in_polygon3D(inter, ph, i) == yes)
      {
         return yes;
      }
   }

   return no;
}


/* intersection between a line (pt1,pt2) and plane (plane,d)
 * will give the intersection point(inter) and where on the line
 * it is (t). Ratio t should does not have to lie within 0 and 1 
 */
SBoolean intersect_line_plane(double pt1[], double pt2[],
                              double plane[], double d, double inter[], double* t)
{
   double dotprod;
   double vec[3];

   MAKE_3DVECTOR(pt1,pt2,vec);
   dotprod = DOT_VECTORS(vec,plane);

   if (DABS(dotprod) < LINE_EPSILON)
      return no;

   *t = (-d - plane[0]*pt1[0] - plane[1]*pt1[1] - plane[2]*pt1[2])/dotprod;

   inter[0] = pt1[0] + ((*t) * vec[0]);
   inter[1] = pt1[1] + ((*t) * vec[1]);
   inter[2] = pt1[2] + ((*t) * vec[2]);

   return yes;
}

/* intersection between a line (pt1,pt2) and plane (plane,d)
** will give the intersection point (inter) and where on the line it is (t)
** ratio t should lie within 0 and 1 
*/
SBoolean intersect_line_plane01(double pt1[], double pt2[], 
                                double plane[], double d, double inter[], double* t)
{
   double dotprod;
   double vec[3];

   MAKE_3DVECTOR(pt1,pt2,vec);
   dotprod = DOT_VECTORS(vec,plane);

   if (DABS(dotprod) < LINE_EPSILON)
      return no;

   *t = (-d - plane[0]*pt1[0] - plane[1]*pt1[1] - plane[2]*pt1[2])/dotprod;

   if ((*t < -LINE_EPSILON) || (*t > 1.0 + LINE_EPSILON))
      return no;

   inter[0] = pt1[0] + ((*t) * vec[0]);
   inter[1] = pt1[1] + ((*t) * vec[1]);
   inter[2] = pt1[2] + ((*t) * vec[2]);

   return yes;
}

/* POINT_IN_POLYGON3D: determines if a 3D point is inside a 3D polygon. The polygon
 * is the first one in the PolyhedronStruct, and is assumed to be planar.
 */

SBoolean point_in_polygon3D(double pt[], PolyhedronStruct* ph, int polygon_index)
{
   int i, j, index, skip_coord;
   double max_value, polygon[50][2], pt2D[2];

   /* if the polygon has more than 50 vertices, abort. */
   if (ph->polygon[polygon_index].num_vertices > 50)
      return no;

   /* Project the polygon onto one of the major planes by ignoring one of the
    * coordinates. Ignore the one most parallel to the plane's normal so that
    * the area of the projected polygon will be as large as possible.
    */
   skip_coord = XX;
   max_value = DABS(ph->polygon[polygon_index].normal[XX]);
   if (DABS(ph->polygon[polygon_index].normal[YY]) > max_value)
   {
      skip_coord = YY;
      max_value = DABS(ph->polygon[polygon_index].normal[YY]);
   }
   if (DABS(ph->polygon[polygon_index].normal[ZZ]) > max_value)
      skip_coord = ZZ;

   /* Copy the vertices into a 2D array, skipping the skip_coord */
   for (i = 0; i < ph->polygon[polygon_index].num_vertices; i++)
   {
      for (j = 0, index = 0; j < 3; j++)
      {
         if (j != skip_coord)
            polygon[i][index++] = ph->vertex[ph->polygon[polygon_index].vertex_index[i]].coord[j];
      }
   }

   /* Copy the 3D point into a 2D point, skipping the skip_coord */
   for (i = 0, index = 0; i < 3; i++)
   {
      if (i != skip_coord)
         pt2D[index++] = pt[i];
   }

   if (point_in_polygon2D(pt2D, polygon, ph->polygon[polygon_index].num_vertices))
      return yes;

   return no;
}


/* POINT_IN_POLYGON2D: determines if a 2D point is inside a 2D polygon. Uses the
 * "crossings test" taken from:
 * Haines, Eric, "Point in Polygon Strategies," Graphics Gems IV, ed. Paul Heckbert,
 * Academic Press, p. 24-46, 1994.
 * Shoot a test ray along +X axis. The strategy, from MacMartin, is to
 * compare vertex Y values to the testing point's Y and quickly discard
 * edges which are entirely to one side of the test ray.
 *
 * Input 2D polygon _pgon_ with _numverts_ number of vertices and test point
 * _point_, returns 1 if inside, 0 if outside.
 */

int point_in_polygon2D(double point[], double pgon[][2], int numverts)
{
   register int j, yflag0, yflag1, inside_flag, xflag0 ;
   register double ty, tx, *vtx0, *vtx1 ;

   tx = point[XX];
   ty = point[YY];

   vtx0 = pgon[numverts-1];

   /* get test bit for above/below X axis */
   yflag0 = ( vtx0[YY] >= ty );
   vtx1 = pgon[0];

   inside_flag = 0;

   for (j = numverts+1; --j ;)
   {
      yflag1 = ( vtx1[YY] >= ty );
       /* check if endpoints straddle (are on opposite sides) of X axis
        * (i.e. the Y's differ); if so, +X ray could intersect this edge.
        */
      if (yflag0 != yflag1)
      {
         xflag0 = (vtx0[XX] >= tx);
         /* check if endpoints are on same side of the Y axis (i.e. X's
          * are the same); if so, it's easy to test if edge hits or misses.
          */
         if ( xflag0 == ( vtx1[XX] >= tx ) )
         {
            /* if edge's X values both right of the point, must hit */
            if ( xflag0 )
               inside_flag = !inside_flag;
         }
         else
         {
              /* compute intersection of pgon segment with +X ray, note
               * if >= point's X; if so, the ray hits it.
               */
            if ((vtx1[XX] - (vtx1[YY]-ty) * (vtx0[XX]-vtx1[XX])/(vtx0[YY]-vtx1[YY])) >= tx)
            {
               inside_flag = !inside_flag;
            }
         }
      }

      /* move to next pair of vertices, retaining info as possible */
      yflag0 = yflag1;
      vtx0 = vtx1;
      vtx1 += 2;
   }

   return inside_flag;
}

int polygon_ray_inter3d(PolyhedronStruct* newph, int poly_index, double pt[3], int axes)
{

   int i,j, num_inter, numpts;
   double **poly_pts, temppt[3], rotpt[3];
   PolygonStruct* poly;
   VertexStruct* v1;
   double **mat;
   double amat[3][3];

   poly = &newph->polygon[poly_index]; 
   numpts = poly->num_vertices;
   poly_pts = (double**)simm_malloc(numpts*sizeof(double*));
   for (i=0; i<numpts; i++)
      poly_pts[i] = (double*)simm_malloc(3*sizeof(double));

   for (i=0; i<numpts; i++)
   {
      v1 = &newph->vertex[poly->vertex_index[i]];
      poly_pts[i][0] = v1->coord[0];
      poly_pts[i][1] = v1->coord[1];
      poly_pts[i][2] = v1->coord[2];
   }

   mat = (double**)simm_malloc(4*sizeof(double*));
   mat[0] = (double*)simm_malloc(3*sizeof(double));
   mat[1] = (double*)simm_malloc(3*sizeof(double));
   mat[2] = (double*)simm_malloc(3*sizeof(double));
   make_rotation_matrix(mat,poly->normal);

   for (i=0; i<3; i++)
      for (j=0; j<3; j++)
     amat[i][j] = mat[i][j];

   for (i=0; i<numpts; i++)
   {
      COPY_1X3VECTOR(poly_pts[i],temppt);
      mult_3x3matrix_by_vector(amat,temppt,poly_pts[i]); 
   }
   mult_3x3matrix_by_vector(amat,pt,rotpt);

   num_inter = polygon_ray_inter_jordanstheorem(poly_pts,numpts,rotpt,axes);

   for (i=0; i<numpts; i++)
      FREE_IFNOTNULL(poly_pts[i]);
   FREE_IFNOTNULL(poly_pts);

   return (num_inter);

}


int polygon_ray_inter_jordanstheorem(double** poly_pts, int numpts,
                     double ptray[3], int axes)
{

   int i, newstate, algstate, otheraxes, numinter = 0;
   int quit = FALSE;
   double intermin, inter, t;
   int count, start, current, last = -1;

   otheraxes = (axes == 1 ? 0 : 1);

   for (i=0; i<numpts; i++)
   {
      if ((algstate = point_ray_relation(poly_pts[i],ptray,otheraxes)) != ON_RAY)
     break;
   }
  
   if (algstate == ON_RAY)
      return (0);

   /* the vertex is below the ray level */
   count = 0;
   if (i == numpts)
      current = start = 0;
   else
      current =  start = i;

   while (!quit)
   {
      count = 0;
      while (point_ray_relation(poly_pts[current],ptray,otheraxes) == algstate)
      {
     last = current;
     current += 1;
     if (current == numpts)
        current = 0;
     count++; 
     if (count == numpts || current == start)
     {
        quit = TRUE;
        break;
     }
      }
      intermin = 1000000;
      count =0;
      while (point_ray_relation(poly_pts[current],ptray,otheraxes) == ON_RAY)
      {
     intermin = _MIN(intermin,poly_pts[current][axes]);
     last = current;
     current += 1;
     if (current == numpts)
        current = 0;
     count++;
     if (current == start)
     {
        quit = TRUE;
     }
      }
      if ((newstate = point_ray_relation(poly_pts[current],ptray,otheraxes))
      != algstate)
      {
     t = (ptray[otheraxes] - poly_pts[current][otheraxes])/
            (poly_pts[last][otheraxes] - poly_pts[current][otheraxes]);
     inter = poly_pts[last][axes] * t + poly_pts[current][axes] * (1.0-t);
     intermin = _MIN(intermin,inter);
     if (intermin > ptray[axes] && NOT_EQUAL_WITHIN_TOLERANCE(intermin,ptray[axes],LINE_EPSILON))
        numinter++;
      }
      algstate = newstate;
   }

   return (numinter);

}

/* returns the relation between a ray and a given point
** returns 0,1,2
** 0= on the ray;  1= above ray, 2= below ray
** the pt is either on, above or below the ray 
*/
int point_ray_relation(double* pt, double ptray[], int axes)
{

   if (EQUAL_WITHIN_TOLERANCE(ptray[axes],pt[axes],LINE_EPSILON))
      return (ON_RAY);
   else if (ptray[axes] < pt[axes])
      return (ABOVE_RAY);
   else
      return (BELOW_RAY);

}

/*****************************************************************************
** DESCRIPTION:                                                               
**
** prepare a transformation martix to rotate such that Dir is   
** parallel to the Z axes. Used to rotate the     
** polygons to be XY plane parallel.                                          
**
**    Algorithm: form a 4 by 4 matrix from Dir as follows:                    
**                |  tx  ty  tz  0 |   A transformation which takes the coord 
**                |  bx  by  bz  0 |  system into t, b & n as required.       
** [X  Y  Z  1] * |  nx  ny  nz  0 |                                          
**                |  0   0   0   1 |                                          
**   N is exactly Dir, but we got freedom on T & B which must be on           
**   a plane perpendicular to N and perpendicular between them but thats all! 
**   T is therefore selected using this (heuristic ?) algorithm:              
**   Let P be the axis of which the absolute N coefficient is the smallest.   
**   Let B be (N cross P) and T be (B cross N).                               
**                                                                            
** PARAMETERS:                                                                
**   mat:     To place the constructed homogeneous transformation.           
**   dir:     To derive a transformation such that Dir goes to Z axis.        
**                                                                            
*/
void make_rotation_matrix(double** mat, double normal[])
{
  int i,j;
  double r;
  double dir_n[3], t[3],b[3],p[3];

  COPY_1X3VECTOR(normal,dir_n);
  normalize_vector(dir_n,dir_n);
  p[0] = 0.0;
  p[1] = 0.0;
  p[2] = 0.0;

  j=0;
  r = DABS(dir_n[0]);
  for(i=1; i<3; i++)
  {
     if( r > DABS(dir_n[i]))
     {
        r = dir_n[i];
        j = i;
     }
  }
  p[j] = 1.0;   /* set p to the axis with the biggest angle from dirn */

  cross_vectors(dir_n,p,b);  /* calc the bi-normal*/
  normalize_vector(b,b);
  cross_vectors(b,dir_n,t);  /* calc the tangent */
  normalize_vector(t,t);
  
  for(i=0; i<3; i++)
  {
      mat[i][0] = t[i];
      mat[i][1] = b[i];
      mat[i][2] = dir_n[i];
  }

}

int intersect_lines(double p1[], double p2[], double p3[], double p4[],
            double p_int1[], double* t, double p_int2[], double* s)
{

   double mag1, mag2, cross_prod[3], denom, vec1[3], vec2[3], mat[3][3];

   vec1[0] = p2[0] - p1[0];
   vec1[1] = p2[1] - p1[1];
   vec1[2] = p2[2] - p1[2];
   mag1 = normalize_vector(vec1,vec1);

   vec2[0] = p4[0] - p3[0];
   vec2[1] = p4[1] - p3[1];
   vec2[2] = p4[2] - p3[2];
   mag2 = normalize_vector(vec2,vec2);

   cross_vectors(vec1,vec2,cross_prod);

   denom = cross_prod[0]*cross_prod[0] + cross_prod[1]*cross_prod[1]
      + cross_prod[2]*cross_prod[2];

   if (EQUAL_WITHIN_ERROR(denom,0.0))
   {
      *s = *t = MINMDOUBLE;
      return (0);
   }

   mat[0][0] = p3[0] - p1[0];
   mat[0][1] = p3[1] - p1[1];
   mat[0][2] = p3[2] - p1[2];
   mat[1][0] = vec1[0];
   mat[1][1] = vec1[1];
   mat[1][2] = vec1[2];
   mat[2][0] = cross_prod[0];
   mat[2][1] = cross_prod[1];
   mat[2][2] = cross_prod[2];

   *s = CALC_DETERMINANT(mat) / denom;

   p_int2[0] = p3[0] + (*s) * (vec2[0]);
   p_int2[1] = p3[1] + (*s) * (vec2[1]);
   p_int2[2] = p3[2] + (*s) * (vec2[2]);

   mat[1][0] = vec2[0];
   mat[1][1] = vec2[1];
   mat[1][2] = vec2[2];

   *t = CALC_DETERMINANT(mat) / denom;

   p_int1[0] = p1[0] + (*t) * (vec1[0]);
   p_int1[1] = p1[1] + (*t) * (vec1[1]);
   p_int1[2] = p1[2] + (*t) * (vec1[2]);

   (*s) /= mag2;
   (*t) /= mag1;

   return (1);

}

/* Same as intersect_lines(), but this routine does not scale the S and T
 * parameters to be between 0.0 and 1.0. S ranges from 0.0 to mag2,
 * and T ranges from 0.0 to mag1.
 */

int intersect_lines_scaled(double p1[], double p2[], double p3[], double p4[],
               double p_int1[], double* t, double* mag1,
               double p_int2[], double* s, double* mag2)
{

   double cross_prod[3], denom, vec1[3], vec2[3], mat[3][3];

   vec1[0] = p2[0] - p1[0];
   vec1[1] = p2[1] - p1[1];
   vec1[2] = p2[2] - p1[2];
   *mag1 = normalize_vector(vec1,vec1);

   vec2[0] = p4[0] - p3[0];
   vec2[1] = p4[1] - p3[1];
   vec2[2] = p4[2] - p3[2];
   *mag2 = normalize_vector(vec2,vec2);

   cross_vectors(vec1,vec2,cross_prod);

   denom = cross_prod[0]*cross_prod[0] + cross_prod[1]*cross_prod[1]
      + cross_prod[2]*cross_prod[2];

   if (EQUAL_WITHIN_ERROR(denom,0.0))
   {
      *s = *t = MINMDOUBLE;
      return (0);
   }

   mat[0][0] = p3[0] - p1[0];
   mat[0][1] = p3[1] - p1[1];
   mat[0][2] = p3[2] - p1[2];
   mat[1][0] = vec1[0];
   mat[1][1] = vec1[1];
   mat[1][2] = vec1[2];
   mat[2][0] = cross_prod[0];
   mat[2][1] = cross_prod[1];
   mat[2][2] = cross_prod[2];

   *s = CALC_DETERMINANT(mat) / denom;

   p_int2[0] = p3[0] + (*s) * (vec2[0]);
   p_int2[1] = p3[1] + (*s) * (vec2[1]);
   p_int2[2] = p3[2] + (*s) * (vec2[2]);

   mat[1][0] = vec2[0];
   mat[1][1] = vec2[1];
   mat[1][2] = vec2[2];

   *t = CALC_DETERMINANT(mat) / denom;

   p_int1[0] = p1[0] + (*t) * (vec1[0]);
   p_int1[1] = p1[1] + (*t) * (vec1[1]);
   p_int1[2] = p1[2] + (*t) * (vec1[2]);

   return (1);

}


/* MULT_3X3MATRIX_BY_VECTOR: this routine premultiplies a 3x3 matrix by
** a 1x3 vector. That is, it does vector*mat, not mat*vector.
*/
void mult_3x3matrix_by_vector(double mat[][3], double vector[], double result[])
{

   result[0] = vector[0]*mat[0][0] + vector[1]*mat[1][0] +
               vector[2]*mat[2][0];
   result[1] = vector[0]*mat[0][1] + vector[1]*mat[1][1] +
               vector[2]*mat[2][1];
   result[2] = vector[0]*mat[0][2] + vector[1]*mat[1][2] +
               vector[2]*mat[2][2];

}


void make_3x3dircos_matrix(double angle, double axis[], double mat[][3])
{

   double rad_angle, cl, sl, omc;

   reset_3x3matrix(mat);

   rad_angle = angle*DTOR;

   cl = cos(rad_angle);
   sl = sin(rad_angle);
   omc = 1.0 - cl;

   /* the following matrix is taken from Kane's 'Spacecraft Dynamics,' pp 6-7 */

   mat[0][0] = cl + axis[0]*axis[0]*omc;
   mat[1][0] = -axis[2]*sl + axis[0]*axis[1]*omc;
   mat[2][0] = axis[1]*sl + axis[2]*axis[0]*omc;
   mat[0][1] = axis[2]*sl + axis[0]*axis[1]*omc;
   mat[1][1] = cl + axis[1]*axis[1]*omc;
   mat[2][1] = -axis[0]*sl + axis[1]*axis[2]*omc;
   mat[0][2] = -axis[1]*sl + axis[2]*axis[0]*omc;
   mat[1][2] = axis[0]*sl + axis[1]*axis[2]*omc;
   mat[2][2] = cl + axis[2]*axis[2]*omc;

}


void reset_3x3matrix(double matrix[][3])
{

   matrix[0][1] = matrix[0][2] = matrix[1][2] = 0.0;
   matrix[1][0] = matrix[2][0] = matrix[2][1] = 0.0;
   matrix[0][0] = matrix[1][1] = matrix[2][2] = 1.0;

}


void clear_vector(double a[], int n)
{

   int i;

   for (i=0; i<n; i++)
      a[i] = 0.0;

}


void make_3x3_xrot_matrix(double a, double m[][3])
{

   m[0][0] = 1.0;
   m[0][1] = 0.0;
   m[0][2] = 0.0;

   m[1][0] = 0.0;
   m[1][1] = cos(a);
   m[1][2] = sin(a);
   
   m[2][0] = 0.0;
   m[2][1] = -sin(a);
   m[2][2] = cos(a);

}


void mult_3x3matrices(double mat1[][3], double mat2[][3], double result[][3])
{

   result[0][0] = mat1[0][0]*mat2[0][0] + mat1[0][1]*mat2[1][0] +
      mat1[0][2]*mat2[2][0];

   result[0][1] = mat1[0][0]*mat2[0][1] + mat1[0][1]*mat2[1][1] +
      mat1[0][2]*mat2[2][1];

   result[0][2] = mat1[0][0]*mat2[0][2] + mat1[0][1]*mat2[1][2] +
      mat1[0][2]*mat2[2][2];


   result[1][0] = mat1[1][0]*mat2[0][0] + mat1[1][1]*mat2[1][0] +
      mat1[1][2]*mat2[2][0];

   result[1][1] = mat1[1][0]*mat2[0][1] + mat1[1][1]*mat2[1][1] +
      mat1[1][2]*mat2[2][1];

   result[1][2] = mat1[1][0]*mat2[0][2] + mat1[1][1]*mat2[1][2] +
      mat1[1][2]*mat2[2][2];


   result[2][0] = mat1[2][0]*mat2[0][0] + mat1[2][1]*mat2[1][0] +
      mat1[2][2]*mat2[2][0];

   result[2][1] = mat1[2][0]*mat2[0][1] + mat1[2][1]*mat2[1][1] +
      mat1[2][2]*mat2[2][1];

   result[2][2] = mat1[2][0]*mat2[0][2] + mat1[2][1]*mat2[1][2] +
      mat1[2][2]*mat2[2][2];

}



void transpose_3x3matrix(double mat[][3], double mat_transpose[][3])
{

   int i, j;

   for (i=0; i<3; i++)
      for (j=0; j<3; j++)
         mat_transpose[j][i] = mat[i][j];

}


void copy_3x3matrix(double from[][3], double to[][3])
{

   to[0][0] = from[0][0];
   to[0][1] = from[0][1];
   to[0][2] = from[0][2];
   to[1][0] = from[1][0];
   to[1][1] = from[1][1];
   to[1][2] = from[1][2];
   to[2][0] = from[2][0];
   to[2][1] = from[2][1];
   to[2][2] = from[2][2];

}


void mult_3x3_by_vector(double mat[][3], double vec[])
{

   double result[3];

   result[0] = vec[0]*mat[0][0] + vec[1]*mat[1][0] + vec[2]*mat[2][0];
   result[1] = vec[0]*mat[0][1] + vec[1]*mat[1][1] + vec[2]*mat[2][1];
   result[2] = vec[0]*mat[0][2] + vec[1]*mat[1][2] + vec[2]*mat[2][2];

   vec[0] = result[0];
   vec[1] = result[1];
   vec[2] = result[2];

}


void find_plane_normal_to_line(PlaneStruct* plane, double pt1[], double pt2[])
{

   double pt3[3];

   MAKE_3DVECTOR(pt1,pt2,pt3);

   normalize_vector(pt3,pt3);

   plane->a = pt3[XX];
   plane->b = pt3[YY];
   plane->c = pt3[ZZ];

   /* d is the distance along the plane's normal vector from the plane to the
    * origin. Thus it is negative when the plane is above the origin, and
    * positive when the plane is below the origin.
    */

   plane->d = -plane->a*pt1[XX] - plane->b*pt1[YY] - plane->c*pt1[ZZ];

}


double compute_angle_between_vectors(double vector1[], double vector2[])
{

   double normal_vector1[3], normal_vector2[3];

   normalize_vector(vector1,normal_vector1);
   normalize_vector(vector2,normal_vector2);

   return (acos(DOT_VECTORS(normal_vector1,normal_vector2)));

}



void project_point_onto_plane(double pt[], PlaneStruct* plane, double projpt[])
{

   double plane_normal[3];
   double distance_to_plane;

   plane_normal[XX] = plane->a;
   plane_normal[YY] = plane->b;
   plane_normal[ZZ] = plane->c;

   normalize_vector(plane_normal,plane_normal);

   /* distance_to_plane is the distance from pt to the plane along the
    * plane's normal vector. Thus it is negative when pt is below
    * the plane, and positive when pt is above the plane.
    */

   distance_to_plane = DOT_VECTORS(pt,plane_normal) + plane->d;

   projpt[XX] = pt[XX] - distance_to_plane*plane_normal[XX];
   projpt[YY] = pt[YY] - distance_to_plane*plane_normal[YY];
   projpt[ZZ] = pt[ZZ] - distance_to_plane*plane_normal[ZZ];

}



double distancesqr_between_vertices(double vertex1[], double vertex2[])
{

   double vec[3];

   vec[0] = vertex2[0] - vertex1[0];
   vec[1] = vertex2[1] - vertex1[1];
   vec[2] = vertex2[2] - vertex1[2];

   return vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2];

}


double distance_between_vertices(double vertex1[], double vertex2[])
{

   double vec[3];

   vec[0] = vertex2[0] - vertex1[0];
   vec[1] = vertex2[1] - vertex1[1];
   vec[2] = vertex2[2] - vertex1[2];

   return (sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]));

}


/* Calculates the square of the shortest distance from a point (point)
 * to a line (vl, through pl).
 */
double get_distsqr_point_line(double point[], double pl[], double vl[])
{

   double ptemp[3];

   /* find the closest point on line */
   get_point_from_point_line(point,pl,vl,ptemp);

   return distancesqr_between_vertices(point,ptemp);

}


/* to calculate the closest 3d point to given 3d line.
 * the line is defined as vector(vec) and a point(pt)
 * the line has not been normalized to a unit vector
 * the value hypo*(cosalpha) of a rt triangle is found out
 * to get the closest point
 */
void get_point_from_point_line(double point[], double pt[], double vec[],
                   double closest_pt[])
{

   double v1[3], v2[3];
   double t, mag;

   v1[0] = point[0] - pt[0];
   v1[1] = point[1] - pt[1];
   v1[2] = point[2] - pt[2];

   v2[0] = vec[0];
   v2[1] = vec[1];
   v2[2] = vec[2];

   mag = normalize_vector(v1,v1);
   normalize_vector(v2,v2);
   t = DOT_VECTORS(v1,v2) * mag;

   closest_pt[0] = pt[0] + t * v2[0];
   closest_pt[1] = pt[1] + t * v2[1];
   closest_pt[2] = pt[2] + t * v2[2];

}


/* ------------------------------------------------------------------------
   atan3 - like the Standard C Library's atan2(), except returning a result
      between 0 and 2pi instead of -pi and pi.
--------------------------------------------------------------------------- */
double atan3 (double y, double x)
{
   double result = atan2(y, x);

   if (result < 0.0)
      result += 2 * M_PI;

   return result;
}

void identity_matrix(double m[][4])
{
   /* set matrix 'm' to the identity matrix */
   
   m[0][0] = 1.0; m[0][1] = 0.0; m[0][2] = 0.0; m[0][3] = 0.0;
   m[1][0] = 0.0; m[1][1] = 1.0; m[1][2] = 0.0; m[1][3] = 0.0;
   m[2][0] = 0.0; m[2][1] = 0.0; m[2][2] = 1.0; m[2][3] = 0.0;
   m[3][0] = 0.0; m[3][1] = 0.0; m[3][2] = 0.0; m[3][3] = 1.0;
}

void x_rotate_matrix_bodyfixed(double m[][4], double radians)
{
   /* append rotation about local x-axis to matrix 'm' */
   Quat q;
   
   make_quaternion(q, m[0], radians);
   rotate_matrix_by_quat(m, q);
}

void y_rotate_matrix_bodyfixed(double m[][4], double radians)
{
   /* append rotation about local y-axis to matrix 'm' */
   Quat q;
   
   make_quaternion(q, m[1], radians);
   rotate_matrix_by_quat(m, q);
}

void z_rotate_matrix_bodyfixed(double m[][4], double radians)
{
   /* append rotation about local z-axis to matrix 'm' */
   Quat q;
   
   make_quaternion(q, m[2], radians);
   rotate_matrix_by_quat(m, q);
}

void x_rotate_matrix_spacefixed(double m[][4], double radians)
{
   /* append rotation about global x-axis to matrix 'm' */
    double sinTheta = sin(radians);
    double cosTheta = cos(radians);

    double t = m[0][1];
    m[0][1] = t * cosTheta - m[0][2] * sinTheta;
    m[0][2] = t * sinTheta + m[0][2] * cosTheta;

    t = m[1][1];
    m[1][1] = t * cosTheta - m[1][2] * sinTheta;
    m[1][2] = t * sinTheta + m[1][2] * cosTheta;

    t = m[2][1];
    m[2][1] = t * cosTheta - m[2][2] * sinTheta;
    m[2][2] = t * sinTheta + m[2][2] * cosTheta;

    t = m[3][1];
    m[3][1] = t * cosTheta - m[3][2] * sinTheta;
    m[3][2] = t * sinTheta + m[3][2] * cosTheta;
}

void y_rotate_matrix_spacefixed(double m[][4], double radians)
{
   /* append rotation about global y-axis to matrix 'm' */
    double sinTheta = sin(radians);
    double cosTheta = cos(radians);

    double t = m[0][0];
    m[0][0] = t * cosTheta + m[0][2] * sinTheta;
    m[0][2] = m[0][2] * cosTheta - t * sinTheta;

    t = m[1][0];
    m[1][0] = t * cosTheta + m[1][2] * sinTheta;
    m[1][2] = m[1][2] * cosTheta - t * sinTheta;

    t = m[2][0];
    m[2][0] = t * cosTheta + m[2][2] * sinTheta;
    m[2][2] = m[2][2] * cosTheta - t * sinTheta;

    t = m[3][0];
    m[3][0] = t * cosTheta + m[3][2] * sinTheta;
    m[3][2] = m[3][2] * cosTheta - t * sinTheta;
}

void z_rotate_matrix_spacefixed(double m[][4], double radians)
{
   /* append rotation about global z-axis to matrix 'm' */
    double sinTheta = sin(radians);
    double cosTheta = cos(radians);

    double t = m[0][0];
    m[0][0] = t * cosTheta - m[0][1] * sinTheta;
    m[0][1] = t * sinTheta + m[0][1] * cosTheta;

    t = m[1][0];
    m[1][0] = t * cosTheta - m[1][1] * sinTheta;
    m[1][1] = t * sinTheta + m[1][1] * cosTheta;

    t = m[2][0];
    m[2][0] = t * cosTheta - m[2][1] * sinTheta;
    m[2][1] = t * sinTheta + m[2][1] * cosTheta;

    t = m[3][0];
    m[3][0] = t * cosTheta - m[3][1] * sinTheta;
    m[3][1] = t * sinTheta + m[3][1] * cosTheta;
}

void translate_matrix(double m[][4], const double* delta)
{
   /* append translation 'delta' to matrix 'm' */
   
   m[0][0] += m[0][3] * delta[0];   m[0][1] += m[0][3] * delta[1];   m[0][2] += m[0][3] * delta[2];
   m[1][0] += m[1][3] * delta[0];   m[1][1] += m[1][3] * delta[1];   m[1][2] += m[1][3] * delta[2];
   m[2][0] += m[2][3] * delta[0];   m[2][1] += m[2][3] * delta[1];   m[2][2] += m[2][3] * delta[2];
   m[3][0] += m[3][3] * delta[0];   m[3][1] += m[3][3] * delta[1];   m[3][2] += m[3][3] * delta[2];
}

void scale_matrix (double m[][4], const double* scaleBy)
{
   m[0][0] *= scaleBy[XX];   m[0][1] *= scaleBy[YY];   m[0][2] *= scaleBy[ZZ];
   m[1][0] *= scaleBy[XX];   m[1][1] *= scaleBy[YY];   m[1][2] *= scaleBy[ZZ];
   m[2][0] *= scaleBy[XX];   m[2][1] *= scaleBy[YY];   m[2][2] *= scaleBy[ZZ];
   m[3][0] *= scaleBy[XX];   m[3][1] *= scaleBy[YY];   m[3][2] *= scaleBy[ZZ];
}

void append_matrix(DMatrix m, const DMatrix b)
{
   /* append matrix 'b' to matrix 'm' */
   
   double ta[4][4];
   
   memcpy(ta, m, 16 * sizeof(double));

   m[0][0] = ta[0][0] * b[0][0] + ta[0][1] * b[1][0] + ta[0][2] * b[2][0] + ta[0][3] * b[3][0];
   m[0][1] = ta[0][0] * b[0][1] + ta[0][1] * b[1][1] + ta[0][2] * b[2][1] + ta[0][3] * b[3][1];
   m[0][2] = ta[0][0] * b[0][2] + ta[0][1] * b[1][2] + ta[0][2] * b[2][2] + ta[0][3] * b[3][2];
   m[0][3] = ta[0][0] * b[0][3] + ta[0][1] * b[1][3] + ta[0][2] * b[2][3] + ta[0][3] * b[3][3];

   m[1][0] = ta[1][0] * b[0][0] + ta[1][1] * b[1][0] + ta[1][2] * b[2][0] + ta[1][3] * b[3][0];
   m[1][1] = ta[1][0] * b[0][1] + ta[1][1] * b[1][1] + ta[1][2] * b[2][1] + ta[1][3] * b[3][1];
   m[1][2] = ta[1][0] * b[0][2] + ta[1][1] * b[1][2] + ta[1][2] * b[2][2] + ta[1][3] * b[3][2];
   m[1][3] = ta[1][0] * b[0][3] + ta[1][1] * b[1][3] + ta[1][2] * b[2][3] + ta[1][3] * b[3][3];

   m[2][0] = ta[2][0] * b[0][0] + ta[2][1] * b[1][0] + ta[2][2] * b[2][0] + ta[2][3] * b[3][0];
   m[2][1] = ta[2][0] * b[0][1] + ta[2][1] * b[1][1] + ta[2][2] * b[2][1] + ta[2][3] * b[3][1];
   m[2][2] = ta[2][0] * b[0][2] + ta[2][1] * b[1][2] + ta[2][2] * b[2][2] + ta[2][3] * b[3][2];
   m[2][3] = ta[2][0] * b[0][3] + ta[2][1] * b[1][3] + ta[2][2] * b[2][3] + ta[2][3] * b[3][3];

   m[3][0] = ta[3][0] * b[0][0] + ta[3][1] * b[1][0] + ta[3][2] * b[2][0] + ta[3][3] * b[3][0];
   m[3][1] = ta[3][0] * b[0][1] + ta[3][1] * b[1][1] + ta[3][2] * b[2][1] + ta[3][3] * b[3][1];
   m[3][2] = ta[3][0] * b[0][2] + ta[3][1] * b[1][2] + ta[3][2] * b[2][2] + ta[3][3] * b[3][2];
   m[3][3] = ta[3][0] * b[0][3] + ta[3][1] * b[1][3] + ta[3][2] * b[2][3] + ta[3][3] * b[3][3];
}

void transform_pt(double m[][4], double* pt)
{
   /* apply a matrix transformation to a 3d point */
   
   double tx = pt[XX] * m[0][0] + pt[YY] * m[1][0] + pt[ZZ] * m[2][0] + m[3][0];
   double ty = pt[XX] * m[0][1] + pt[YY] * m[1][1] + pt[ZZ] * m[2][1] + m[3][1];

   pt[ZZ] = pt[XX] * m[0][2] + pt[YY] * m[1][2] + pt[ZZ] * m[2][2] + m[3][2];
   pt[XX] = tx;
   pt[YY] = ty;
}

void transform_vec(double m[][4], double* vec)
{
   /* apply a matrix transformation to a 3d vector */
   
   double tx = vec[XX] * m[0][0] + vec[YY] * m[1][0] + vec[ZZ] * m[2][0];
   double ty = vec[XX] * m[0][1] + vec[YY] * m[1][1] + vec[ZZ] * m[2][1];

   vec[ZZ] = vec[XX] * m[0][2] + vec[YY] * m[1][2] + vec[ZZ] * m[2][2];
   vec[XX] = tx;
   vec[YY] = ty;
}

static void quat_to_axis_angle_rot (const Quat q, dpCoord3D* axis, double* angle)
{
   double sin_a2;
      
   axis->xyz[XX] = q[XX];
   axis->xyz[YY] = q[YY];
   axis->xyz[ZZ] = q[ZZ];

   /* |sin a/2|, w = cos a/2 */
   sin_a2 = sqrt(SQR(q[XX]) + SQR(q[YY]) + SQR(q[ZZ]));

   /* 0 <= angle <= PI , because 0 < sin_a2 */
   *angle = 2.0 * atan2(sin_a2, q[WW]);
   
   if (EQUAL_WITHIN_ERROR(0.0,*angle))
   {
      axis->xyz[XX] = 1.0;
      axis->xyz[YY] = 0.0;
      axis->xyz[ZZ] = 0.0;
   }
}

static void matrix_to_quat (double m[][4], Quat q)
{
   double s, trace = m[XX][XX] + m[YY][YY] + m[ZZ][ZZ];

   if (NEAR_GT_OR_EQ(trace, 0.0))
   {
      s    = sqrt(trace + m[WW][WW]);
      q[WW] = s * 0.5;
      s    = 0.5 / s;

      q[XX] = (m[YY][ZZ] - m[ZZ][YY]) * s;
      q[YY] = (m[ZZ][XX] - m[XX][ZZ]) * s;
      q[ZZ] = (m[XX][YY] - m[YY][XX]) * s;
   }
   else
   {
      static const int next[] = { YY, ZZ, XX };

      int i = XX, j, k;

      if (m[YY][YY] > m[XX][XX])
         i = YY;
      if (m[ZZ][ZZ] > m[i][i])
         i = ZZ;

      j = next[i];
      k = next[j];

      s = sqrt((m[i][i] - (m[j][j] + m[k][k])) + m[WW][WW]);

      q[i] = s * 0.5;
      s    = 0.5 / s;

      q[j] = (m[i][j] + m[j][i]) * s;
      q[k] = (m[i][k] + m[k][i]) * s;
      q[WW] = (m[j][k] - m[k][j]) * s;
   }
   if (NOT_EQUAL_WITHIN_ERROR(m[WW][WW], 1.0))
   {
      double scale = 1.0 / sqrt(m[WW][WW]);

      q[XX] *= scale;  q[YY] *= scale;  q[ZZ] *= scale;  q[WW] *= scale;
   }
}

void extract_rotation(double m[][4], dpCoord3D* axis, double* angle)
{
   /* extract matrix rotation in axis-angle format */
   
   Quat q;
   
   matrix_to_quat(m, q);
   
   quat_to_axis_angle_rot(q, axis, angle);
}

void extract_xyz_rot_spacefixed(double m[][4], double xyz_rot[3])
{
   /* NOTE: extracts SPACE-FIXED rotations in x,y,z order.
    *  The matrix may have scale factors in it, so it needs to
    *  be normalized first.
    */
   double mat[4][4];

   copy_4x4matrix(m, mat);
   normalize_vector(mat[0], mat[0]);
   normalize_vector(mat[1], mat[1]);
   normalize_vector(mat[2], mat[2]);

   xyz_rot[YY] = asin(-mat[0][2]);
   
   if (NOT_EQUAL_WITHIN_ERROR(0.0,cos(xyz_rot[YY])))
   {
      xyz_rot[XX] = atan2(mat[1][2], mat[2][2]);
      xyz_rot[ZZ] = atan2(mat[0][1], mat[0][0]);
   }
   else
   {
      xyz_rot[XX] = atan2(mat[1][0], mat[1][1]);
      xyz_rot[ZZ] = 0.0;
   } 
}

void extract_xyz_rot_bodyfixed(double m[][4], double xyz_rot[3])
{
   /* NOTE: extracts BODY-FIXED rotations in x,y,z order, which
    *  is the same as space-fixed rotations in z,y,x order.
    *  The matrix may have scale factors in it, so it needs to
    *  be normalized first.
    */
   double mat1[4][4], mat2[4][4];

#if 0
   // Normalize the columns by transposing so that the
   // vectors are contiguous. Then transpose back.
   transpose_4x4matrix(m, mat1);
   normalize_vector(mat1[0], mat1[0]);
   normalize_vector(mat1[1], mat1[1]);
   normalize_vector(mat1[2], mat1[2]);
   transpose_4x4matrix(mat1, mat2);
#else
   copy_4x4matrix(m, mat2);
   normalize_vector(mat2[0], mat2[0]);
   normalize_vector(mat2[1], mat2[1]);
   normalize_vector(mat2[2], mat2[2]);
#endif

   xyz_rot[YY] = asin(mat2[2][0]);
   
   if (NOT_EQUAL_WITHIN_ERROR(0.0,cos(xyz_rot[YY])))
   {
      xyz_rot[XX] = atan2(-mat2[2][1], mat2[2][2]);
      xyz_rot[ZZ] = atan2(-mat2[1][0], mat2[0][0]);
   }
   else
   {
      xyz_rot[XX] = atan2(mat2[0][1], mat2[1][1]);
      xyz_rot[ZZ] = 0.0;
   } 
   /* NOTE: a body-fixed sequence of rotations is equivalent to
    *  the same sequence of space-fixed rotation in the *opposite*
    *  order!!  (see: "Introduction to Robotics, 2nd Ed. by J. Craig,
    *  page 49)  -- KMS 2/17/99
    */
}

static void make_quaternion(Quat q, const double axis[3], double angle)
{
   /* make a quaternion given an axis-angle rotation
    * (from java-based vecmath package)
    */
   double n, halfAngle;
  
   q[XX] = axis[XX];
   q[YY] = axis[YY];
   q[ZZ] = axis[ZZ];

   n = sqrt(SQR(q[XX]) + SQR(q[YY]) + SQR(q[ZZ]));

   halfAngle = 0.5 * angle;

   if (NOT_EQUAL_WITHIN_ERROR(n,0.0))
   {
      double s = sin(halfAngle) / n;

      q[XX] *= s;  q[YY] *= s;  q[ZZ] *= s;

      q[WW] = cos(halfAngle);
   }
}

static void quat_to_matrix (const Quat q, double m[][4])
{
   /* make a rotation matrix from a quaternion */
   
   double Nq = SQR(q[XX]) + SQR(q[YY]) + SQR(q[ZZ]) + SQR(q[WW]);
   double s = (Nq > 0.0) ? (2.0 / Nq) : 0.0;

   double xs = q[XX] * s,   ys = q[YY] * s,   zs = q[ZZ] * s;
   double wx = q[WW] * xs,  wy = q[WW] * ys,  wz = q[WW] * zs;
   double xx = q[XX] * xs,  xy = q[XX] * ys,  xz = q[XX] * zs;
   double yy = q[YY] * ys,  yz = q[YY] * zs,  zz = q[ZZ] * zs;

   m[XX][XX] = 1.0 - (yy + zz);  m[XX][YY] = xy + wz;          m[XX][ZZ] = xz - wy;
   m[YY][XX] = xy - wz;          m[YY][YY] = 1.0 - (xx + zz);  m[YY][ZZ] = yz + wx;
   m[ZZ][XX] = xz + wy;          m[ZZ][YY] = yz - wx;          m[ZZ][ZZ] = 1.0 - (xx + yy);

   m[XX][WW] = m[YY][WW] = m[ZZ][WW] = m[WW][XX] = m[WW][YY] = m[WW][ZZ] = 0.0;
   m[WW][WW] = 1.0;
}

static void rotate_matrix_by_quat(double m[][4], const Quat q)
{
   /* append a quaternion rotation to a matrix */
   
   double n[4][4];
   
   quat_to_matrix(q, n);
   
   append_matrix(m, n);    /* append matrix 'n' onto matrix 'm' */
}

void rotate_matrix_axis_angle (double m[][4], const double* axis, double angle)
{
    Quat q;

    make_quaternion(q, axis, angle);
    rotate_matrix_by_quat(m, q);
}

void lerp_pt(double start[3], double end[3], double t, double result[3])
{
   result[0] = start[0] + t * (end[0] - start[0]);
   result[1] = start[1] + t * (end[1] - start[1]);
   result[2] = start[2] + t * (end[2] - start[2]);
}

void slerp(const dpCoord3D* axisStart, double angleStart,
           const dpCoord3D* axisEnd,   double angleEnd,
           double t,
           dpCoord3D* axisResult, double* angleResult)
{
   Quat from, to, res;
   double to1[4], omega, cosom, sinom, scale0, scale1;
   
   make_quaternion(from, axisStart->xyz, angleStart);
   make_quaternion(to,   axisEnd->xyz,   angleEnd);

   /* calc cosine */
   cosom = from[XX] * to[XX] + from[YY] * to[YY] + from[ZZ] * to[ZZ] + from[WW] * to[WW];

   /* adjust signs (if necessary) */
   if (cosom < 0.0)
   {
      cosom = -cosom; to1[0] = - to[XX];
      to1[1] = - to[YY];
      to1[2] = - to[ZZ];
      to1[3] = - to[WW];
   }
   else
   {
      to1[0] = to[XX];
      to1[1] = to[YY];
      to1[2] = to[ZZ];
      to1[3] = to[WW];
   }

   /* calculate coefficients */
   if ((1.0 - cosom) > TINY_NUMBER)
   {
      /* standard case (slerp) */
      omega = acos(cosom);
      sinom = sin(omega);
      scale0 = sin((1.0 - t) * omega) / sinom;
      scale1 = sin(t * omega) / sinom;
   }
   else
   {        
      /* "from" and "to" quaternions are very close, do a linear interpolation */
      scale0 = 1.0 - t;
      scale1 = t;
   }
   /* calculate final values */
   res[XX] = scale0 * from[XX] + scale1 * to1[XX];
   res[YY] = scale0 * from[YY] + scale1 * to1[YY];
   res[ZZ] = scale0 * from[ZZ] + scale1 * to1[ZZ];
   res[WW] = scale0 * from[WW] + scale1 * to1[WW];
   
   quat_to_axis_angle_rot(res, axisResult, angleResult);
}

smAxes find_primary_direction(double vec[])
{
   double x_abs = ABS(vec[0]);
   double y_abs = ABS(vec[1]);
   double z_abs = ABS(vec[2]);

   if (x_abs >= y_abs)
   {
      if (x_abs >= z_abs)
      {
         if (vec[0] > 0.0)
            return smX;
         else
            return smNegX;
      }
      else
      {
         if (vec[2] > 0.0)
            return smZ;
         else
            return smNegZ;
      }
   }
   else
   {
      if (y_abs >= z_abs)
      {
         if (vec[1] > 0.0)
            return smY;
         else
            return smNegY;
      }
      else
      {
         if (vec[2] > 0.0)
            return smZ;
         else
            return smNegZ;
      }
   }
}


/* COPY_4X4MATRIX: copies a 4x4 matrix */
void copy_4x4matrix(double from[][4], double to[][4])
{
   to[0][0] = from[0][0];
   to[0][1] = from[0][1];
   to[0][2] = from[0][2];
   to[0][3] = from[0][3];
   to[1][0] = from[1][0];
   to[1][1] = from[1][1];
   to[1][2] = from[1][2];
   to[1][3] = from[1][3];
   to[2][0] = from[2][0];
   to[2][1] = from[2][1];
   to[2][2] = from[2][2];
   to[2][3] = from[2][3];
   to[3][0] = from[3][0];
   to[3][1] = from[3][1];
   to[3][2] = from[3][2];
   to[3][3] = from[3][3];
}
