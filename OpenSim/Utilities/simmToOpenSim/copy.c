/*******************************************************************************

   COPY.C

   Author: Peter Loan

   Date: 17-MAY-89

   Copyright (c) 1992-5 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Description: This file contains routines that copy values
      in some structure or array to another structure or array.

   Routines:
      copy_4x4matrix : copies a 4x4 matrix
      copy_1x4vector : copies a 1x4 array (vector)
      copy_dof       : copies a REFEQ structure which holds a dof
      copy_function  : copies a function's points and spline coefficients
      copy_mps       : copies a MusclePoint array
      copy_nndouble  : mallocs and copies a double if 'from' is not null
      copy_nddouble  : mallocs and copies a double if 'from' is not default
      copy_nnint     : mallocs and copies a int if 'from' is not null
      copy_ndint     : mallocs and copies a int if 'from' is not default

*******************************************************************************/

#include "universal.h"
#include "globals.h"
#include "functions.h"


/*************** DEFINES (for this file only) *********************************/



/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/



/*************** EXTERNED VARIABLES (declared in another file) ****************/



/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/



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



/* COPY_1x4VECTOR: copies a 1x4 vector */

void copy_1x4vector(double from[], double to[])
{

   to[0] = from[0];
   to[1] = from[1];
   to[2] = from[2];
   to[3] = from[3];

}


void copy_point(Coord3D *from, Coord3D *to)
{
   to->xyz[0] = from->xyz[0];
   to->xyz[1] = from->xyz[1];
   to->xyz[2] = from->xyz[2];
}

/* COPY_DOF: copies a dof. Used to save and restore dofs in the joint
 * editor and to copy dofs from the model-file-reading buffer to a
 * model structure.
 */

void copy_dof(DofStruct* from, DofStruct* to)
{

   to->type = from->type;
   to->value = from->value;
   to->funcnum = from->funcnum;
   to->gencoord = from->gencoord;

}


void copy_gencoord_info(GeneralizedCoord *from, SaveGencoords *to)
{
   to->value = from->value;
   to->velocity = from->velocity;
   to->tolerance = from->tolerance;
   to->pd_stiffness = from->pd_stiffness;
   to->default_value = from->default_value;
   to->clamped = from->clamped;
   to->locked = from->locked;
   to->restraintFuncActive = from->restraintFuncActive;
   to->range = from->range;
   to->restraint_func_num = from->restraint_func_num;
   to->restraint_user_num = from->restraint_user_num;
   to->used_in_model = from->used_in_model;
}

/* COPY_Function: copies a function structure. This structure contains
 * the original (x,y) data points and the coefficients (b,c,d) of
 * the natural cubic spline interpolants. It assumes that space
 * for x,y,a,b,c has already been allocated.
 */

void copy_function(SplineFunction* from, SplineFunction* to)
{
   int i;

   to->type = from->type;
   to->numpoints = from->numpoints;
   to->usernum = from->usernum;
   to->defined = from->defined;
   to->used = yes;

   for (i=0; i<to->numpoints; i++)
   {
      to->x[i] = from->x[i];
      to->y[i] = from->y[i];
      to->b[i] = from->b[i];
      to->c[i] = from->c[i];
      to->d[i] = from->d[i];
   }
}



/* COPY_MPS: */

ReturnCode copy_mps(MusclePoint* from, MusclePoint* to)
{
   int i;

   to->segment = from->segment;
   to->refpt = from->refpt;
   to->selected = from->selected;
   to->point[XX] = from->point[XX];
   to->point[YY] = from->point[YY];
   to->point[ZZ] = from->point[ZZ];
   to->ground_pt[XX] = from->ground_pt[XX];
   to->ground_pt[YY] = from->ground_pt[YY];
   to->ground_pt[ZZ] = from->ground_pt[ZZ];
   to->old_seg = from->old_seg;
   to->old_point[XX] = from->old_point[XX];
   to->old_point[YY] = from->old_point[YY];
   to->old_point[ZZ] = from->old_point[ZZ];
   to->numranges = from->numranges;
   if (to->numranges > 0)
   {
      to->ranges = (PointRange*)simm_malloc(to->numranges*sizeof(PointRange));
      if (to->ranges == NULL)
         return code_bad;

      for (i=0; i<to->numranges; i++)
      {
         to->ranges[i].genc = from->ranges[i].genc;
         to->ranges[i].start = from->ranges[i].start;
         to->ranges[i].end = from->ranges[i].end;
      }
   }
   else
      to->ranges = NULL;

   to->is_auto_wrap_point = from->is_auto_wrap_point;
   to->wrap_distance = from->wrap_distance;
   to->num_wrap_pts = from->num_wrap_pts;
   if (to->num_wrap_pts > 0)
   {
      to->wrap_pts = (double*)simm_malloc(to->num_wrap_pts*3*sizeof(double));
      if (to->wrap_pts == NULL)
         return code_bad;
      for (i=0; i<to->num_wrap_pts*3; i++)
         to->wrap_pts[i] = from->wrap_pts[i];
   }
   else
      to->wrap_pts = NULL;

   to->undeformed_point[XX] = from->undeformed_point[XX];
   to->undeformed_point[YY] = from->undeformed_point[YY];
   to->undeformed_point[ZZ] = from->undeformed_point[ZZ];

   return code_fine;

}



/* COPY_NNDOUBLE: */

ReturnCode copy_nndouble(double* from, double** to)
{

   if (from == NULL)
      *to = NULL;
   else
   {
      if ((*to = (double*)simm_malloc(sizeof(double))) == NULL)
	 return (code_bad);
      **to = *from;
   }

   return (code_fine);

}



/* COPY_NDDOUBLE: */

ReturnCode copy_nddouble(double* from, double** to, double* deffrom, double* defto)
{

   if (from == deffrom)
      *to = defto;
   else
   {
      if (from == NULL)
	 *to = NULL;
      else
      {
	 if ((*to = (double*)simm_malloc(sizeof(double))) == NULL)
	    return (code_bad);
	 **to = *from;
      }
   }

   return (code_fine);

}



/* COPY_NNINT: */

ReturnCode copy_nnint(int* from, int** to)
{

   if (from == NULL)
      *to = NULL;
   else
   {
      if ((*to = (int*)simm_malloc(sizeof(int))) == NULL)
	 return (code_bad);
      **to = *from;
   }

   return (code_fine);

}



/* COPY_NDINT: */

ReturnCode copy_ndint(int* from, int** to, int* deffrom, int* defto)
{

   if (from == deffrom)
      *to = defto;
   else
   {
      if (from == NULL)
	 *to = NULL;
      else
      {
	 if ((*to = (int*)simm_malloc(sizeof(int))) == NULL)
	    return (code_bad);
	 **to = *from;
      }
   }

   return (code_fine);

}


