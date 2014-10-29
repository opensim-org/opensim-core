/*******************************************************************************

   INVERT.C

   Author: Peter Loan

   Date: 13-APR-90

   Copyright (c) 1992-3 MusculoGraphics, Inc.
   All rights reserved.
   Portions of this source code are copyrighted by MusculoGraphics, Inc.

   Description: This file ontains routines that invert a matrix. All three
      routines were taken from "Numerical Recipes in C."

   Routines:
      invert_matrix : inverts a square matrix of any size
      ludcmp        : used by invert()
      lubksb        : used by invert()

*******************************************************************************/

#include "universal.h"

#include "globals.h"
#include "functions.h"


/*************** DEFINES (for this file only) *********************************/
#define TINY 1.0e-20


/*************** STATIC GLOBAL VARIABLES (for this file only) *****************/



/*************** EXTERNED VARIABLES (declared in another file) ****************/



/*************** PROTOTYPES for STATIC FUNCTIONS (for this file only) *********/
static void ludcmp(double* a[], int n, int indx[], double* d);
static void lubksb(double* a[], int n, int indx[], double b[]);



void invert_matrix(double* matrix[], double* inverse[], int size)
{

   double d, col[MAX_MATRIX_SIZE];
   int i, j, indx[MAX_MATRIX_SIZE];

   ludcmp(matrix,size,indx,&d);
   for (j=0; j<size; j++)
   {
      for (i=0; i<size; i++)
     col[i] = 0.0;
      col[j] = 1.0;
      lubksb(matrix,size,indx,col);
      for (i=0; i<size; i++)
     inverse[i][j] = col[i];
   }

}



static void ludcmp(double* a[], int n, int indx[], double* d)
{

   int i, imax, j, k;
   double big, dum, sum, temp;
   double vv[MAX_MATRIX_SIZE];

   *d = 1.0;

   for (i=0; i<n; i++)
   {
      big = 0.0;
      for (j=0; j<n; j++)
     if ((temp=fabs(a[i][j])) > big)
        big = temp;
      if (big == 0.0)
      {
     error(recover,"Warning: singular matrix in routine LUDCMP");
     return;
      }
      vv[i] = 1.0/big;
   }

   for (j=0; j<n; j++)
   {
      for (i=0; i<j; i++)
      {
     sum = a[i][j];
     for (k=0; k<i; k++)
        sum -= a[i][k]*a[k][j];
     a[i][j] = sum;
      }
      big = 0.0;
      for (i=j; i<n; i++)
      {
     sum = a[i][j];
     for (k=0; k<j; k++)
        sum -= a[i][k]*a[k][j];
     a[i][j] = sum;
     if ((dum=vv[i]*fabs(sum)) >= big)
     {
        big = dum;
        imax = i;
     }
      }
      if (j != imax)
      {
     for (k=0; k<n; k++)
     {
        dum = a[imax][k];
        a[imax][k] = a[j][k];
        a[j][k] = dum;
     }
     *d = -(*d);
     vv[imax] = vv[j];
      }
      indx[j] = imax;
      if (a[j][j] == 0.0)
     a[j][j] = TINY;
      if (j != n)
      {
     dum = 1.0/(a[j][j]);
     for (i=j+1; i<n; i++)
        a[i][j] *= dum;
      }
   }

}



static void lubksb(double* a[], int n, int indx[], double b[])
{

   int i, j, ip, k= -1;
   double sum;

   for (i=0; i<n; i++)
   {
      ip = indx[i];
      sum = b[ip];
      b[ip] = b[i];
      if (k >= 0)
     for (j=k; j<=i-1; j++)
        sum -= a[i][j]*b[j];
      else if (sum)
     k = i;
      b[i] = sum;
   }
   for (i=n-1; i>=0; i--)
   {
      sum = b[i];
      for (j=i+1; j<n; j++)
     sum -= a[i][j]*b[j];
      b[i] = sum/a[i][i];
   }

}


/* INVERT_4X4MATRIX: this routine is a wrapper for invert_matrix
 * that is designed for 4x4s.
 */

void invert_4x4matrix(double matrix[][4], double inverse[][4])
{

   double *tmp_mat[4];
   double *tmp_inverse[4];

   tmp_mat[0] = &(matrix[0][0]);
   tmp_mat[1] = &(matrix[1][0]);
   tmp_mat[2] = &(matrix[2][0]);
   tmp_mat[3] = &(matrix[3][0]);

   tmp_inverse[0] = &(inverse[0][0]);
   tmp_inverse[1] = &(inverse[1][0]);
   tmp_inverse[2] = &(inverse[2][0]);
   tmp_inverse[3] = &(inverse[3][0]);

   invert_matrix(tmp_mat, tmp_inverse, 4);

}


/* INVERT_3X3MATRIX: this routine is a wrapper for invert_matrix
 * that is designed for 3x3s.
 */

void invert_3x3matrix(double matrix[][3], double inverse[][3])
{

   double *tmp_mat[3];
   double *tmp_inverse[3];

   tmp_mat[0] = &(matrix[0][0]);
   tmp_mat[1] = &(matrix[1][0]);
   tmp_mat[2] = &(matrix[2][0]);

   tmp_inverse[0] = &(inverse[0][0]);
   tmp_inverse[1] = &(inverse[1][0]);
   tmp_inverse[2] = &(inverse[2][0]);

   invert_matrix(tmp_mat, tmp_inverse, 3);

}


/* INVERT_4X4TRANFORM: this routine is just like invert_4x4matrix,
 * except that it assumes that the matrix is a transform matrix
 * with only translations and rotations, so that the inverse is
 * the transpose, with some negative signs for the translations.
 */

void invert_4x4transform(double matrix[][4], double inverse[][4])
{

   double vec1[4], vec2[4];

   inverse[0][0] = matrix[0][0];
   inverse[0][1] = matrix[1][0];
   inverse[0][2] = matrix[2][0];
   inverse[0][3] = 0.0;

   inverse[1][0] = matrix[0][1];
   inverse[1][1] = matrix[1][1];
   inverse[1][2] = matrix[2][1];
   inverse[1][3] = 0.0;

   inverse[2][0] = matrix[0][2];
   inverse[2][1] = matrix[1][2];
   inverse[2][2] = matrix[2][2];
   inverse[2][3] = 0.0;

   inverse[3][0] = 0.0;
   inverse[3][1] = 0.0;
   inverse[3][2] = 0.0;
   inverse[3][3] = 1.0;

   vec1[0] = -matrix[3][0];
   vec1[1] = -matrix[3][1];
   vec1[2] = -matrix[3][2];
   vec1[3] = 1.0;

   mult_4x4matrix_by_vector(inverse,vec1,vec2);

   inverse[3][0] = vec2[0];
   inverse[3][1] = vec2[1];
   inverse[3][2] = vec2[2];
   inverse[3][3] = 1.0;

}

