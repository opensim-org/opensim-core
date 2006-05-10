/* ods_subs.cpp ---------------------------------------------------------------

(c) Copyright 2000, Ncompass Research, Inc.

This file contains two routines for line searching.  The first calculates the
 line search function.  The second performs the line search.

===============================================================================
Revision Record:
2/2/2000 2:35 pm
===============================================================================
===============================================================================


This function calculates a value of the function minimized by the one
 dimensional search routine "ods".
_______________________________________________________________________________

Input:
  P = performance
  C = vector of constraints
  mu = constraint weight factors
  nconstr = total number of constraints
  neqconstr = number of equality constraints
  exponent = this determines how the constraints values will be applied
____________________________________________________________________________ */


#include <stdio.h>
#include <stdlib.h>
#include <math.h>


#define min(A,B) (A < B) ? A : B
#define max(A,B) (A > B) ? A : B


double ods_func(double P, double *C, double *mu, int nconstr, int neqconstr,
                 int exponent)
{
int i;
static double returnval;

returnval = P;

exponent = (max(1,exponent));

for (i=0;i<neqconstr;i++)
  returnval += pow(mu[i]*fabs(C[i]),(double)exponent);

for (i=neqconstr;i<nconstr;i++)
  returnval += pow( mu[i]*(max(0.0,-C[i])) , (double)exponent );

return returnval;

} // End of ods_func ----------------------------------------------------------







/* This function performs a one dimensional search on the function "F" by curve
  fitting it with a quadratic polynomial, and then minimizing this quadratic.
_______________________________________________________________________________

Input -----
      n = number of independent variables
      X = vector of independent variables at the initial point
     dx = direction of descent at the initial point
   epsc = convergence value
 minalf = minimum allowed size of the step length (alpha)
 maxalf = maximum allowed size of the step length (alpha)
fact[0] = multiplication factor used to adjust the size of the interval.
           Try fact[0] = 2.0
fact[1] = multiplication factor used to cut the size of alpha[1] when needed.
           Try fact[1] = 0.1
fact[2] = multiplication factor used to determine the step length to the right
           most point of the interval.  Try fact[2] = 2.0
    mfc = maximum number of function calls (nfc)

Output -----
     Xn = vector of independent variables at the current alpha
    nfc = number of function calls in this line search

Input and Output -----
alpha = vector of step lengths (alpha[3] = minimum)
         An initial value for alpha[1] must be supplied.  Try alpha[1] = 0.2

    F = vector of function values at each alpha step
        F[0] must be calculated before calling this routine
        F[3] = minimum

oflag = a flag which indicates the currect status of the ods algorithm.
        -3 => error condition (alpha is less than minalf)
        -2 => error condition (alpha is greater than maxalf)
        -1 => error condition (too many function calls)
         0 => line search completed (this must also be the initial input)
         1 => calculate first alpha step
         2 => calculate second alpha step
         3 => calculate third alpha step
         4 => calculate the minimum value

Other -----
 F[4] = value of the quadratic function at its minimum


Example -----

int n,m,meq,nfc,mfc,oflag;
double X[25],Xn[25],dx[25],P,C[9],mu[9],F[5],fact[3]
double epsc,minalf,maxalf,alpha[4];
     .
     .
     .

n = 25;
m = 9;
meq = 3;

epsc = 1.0e-06;
minalf = 1.0e-06;
maxalf = 1.0e+06;
fact[0] = 2.0;
fact[1] = 0.2;
fact[2] = 5.0;
mfc = 30;

     .
     .
     .


alpha[1] = 0.2;

performance(X,&P,C);
F[0] = ods_func(P,C,mu,m,meq,2);

oflag = 0;  // Initialize the line search function.

do
{
  ods(n,X,F,dx,epsc,minalf,maxalf,alpha,Xn,fact,&oflag,&nfc,mfc);

  switch(oflag)
  {
    case 0:
      printf("Minimum value = %f\n",F[3]);
      printf("The function is minimized at:\n");
        for (i=0;i<n;i++) printf("Xn[%d] = %f\n",i,Xn[i]);
      printf("with alpha = %f",alpha[3]);
      break;

    case -1:
      printf("\nToo many functions calls.");
      return -1;

    case -2:
      printf("\nalpha too large.");
      return -2;

    case -3:
      printf("\nalpha too small.");
      return -3;

    default;
      performance(Xn,&P,C);
      F[oflag-1] = ods_func(P,C,mu,m,meq,2);
      break;

  } // End of switch -----

}
while(oflag);

     .
     .
     .
____________________________________________________________________________ */


#define DISTL 0.05
#define DISTR 0.95


void ods(int n, double *X, double *F, double *dx, double epsc, double minalf,
          double maxalf, double *alpha, double *Xn, double *fact, int *oflag,
          int *nfc, int mfc)
{
int i;
double aratio,Fe,P,Q,R,temp;


// Initialize various values -----

if (*oflag == 0)
{
  *oflag = 2;

  *nfc = 1;

  alpha[0] = 0.0;
  alpha[3] = 1.0;

  for (i=0;i<n;i++) Xn[i] = X[i] + alpha[1]*dx[i];

  return;
}

//  End of initialization -----



if (*nfc >= mfc)       // Maximum number of function call exceeded.
{
// Redefine the independent variables.  This normally should not be required.
//  However, in some cases the calling routine may change these variables in
//  some unappealing way (e.g., scaling), so they should be reset here.

  for (i=0;i<n;i++)
    Xn[i] = X[i] + alpha[3]*dx[i];

  *oflag = -1;

  return;
}



while(1)
{
  if (*oflag < 3)     // Searching for the 1st or 2nd alpha step.
  {
    if (F[0] > F[1]) // The function at alpha[1] is lower than the function
    {                // at alpha[0].  The step is downhill. Get the next alpha.

      alpha[2] = alpha[0] + fact[2]*(alpha[1] - alpha[0]);

      if (alpha[2] > maxalf) // The required alpha[2] step is too large.
      {                      //  No optimal step length can be found.  Return
        *oflag = -2;         //  a value at a step length equal to alpha[1].

        alpha[3] = alpha[1];
        for (i=0;i<n;i++) Xn[i] = X[i] + alpha[3]*dx[i];

        return;
      }

      *oflag = 3;

//  Find the position of the second alpha step in the interval.

      aratio = (alpha[1] - alpha[0]) / (alpha[2] - alpha[0]);

// If the second alpha step is too close to either end, then adjust the
//  interval size and try again.

      if (aratio < DISTL)
        alpha[2] = alpha[0] +fact[0]*(alpha[1] - alpha[0]);

      else if (aratio > DISTR)
      {
        alpha[0] = alpha[2] - fact[0]*(alpha[2] - alpha[1]);
        *oflag = 1;
      }

      for (i=0;i<n;i++) Xn[i] = X[i] + alpha[(*oflag)-1]*dx[i];
      (*nfc)++;

      return;
    }

    else // The value of the function at alpha[1] is higher than that at
    {    //  alpha[0].  Cut alpha[1] and try again.

      alpha[1] = alpha[1]*fact[1];

      if (alpha[1] < minalf)
      {
// The required alpha[1] step is too small.  No optimal step length can be
//  found.  Return a value at a step length equal to alpha[1] / fact[1].
//  This is the most common failure.  It ususally means the direction of
//  descent is actually uphill.

        *oflag = -3;

        alpha[3] = alpha[1] / fact[1];
        for (i=0;i<n;i++) Xn[i] = X[i] + alpha[3]*dx[i];

        return;
      }


      for (i=0;i<n;i++) Xn[i] = X[i] + alpha[1]*dx[i];
      (*nfc)++;

      *oflag = 2;
      return;
    }
  }


  if (*oflag == 4)  // Find the normalized difference in the quadratic and
  {                 //  the function.

		// NEXT LINE ALTERED BY CLAY ANDERSON
		// Fe = (F[3] - F[4]) / max(fabs(F[3]),epsc); (ORIGINAL)
		Fe = (F[3] - F[4]) / (max(fabs(F[3]),epsc));

    if (fabs(Fe) <= epsc)                      // Check for convergence.
    {
// Redefine the independent variables.  This normally should not be required.
//  However, in some cases the calling routine may change these variables in
//  some unappealing way (e.g., scaling), so they should be reset here.

      for (i=0;i<n;i++) Xn[i] = X[i] + alpha[3]*dx[i];

      *oflag = 0;  // Converged <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
      return;
    }

// If the solution did not converge, then swap points around in some fancy way
//  and try again.

    if (alpha[3] > alpha[1])
    {
      if (F[3] > F[1])
      {
        alpha[2] = alpha[3];
        F[2] = F[3];
      }
      else
      {
        alpha[0] = alpha[1];
        F[0] = F[1];
        alpha[1] = alpha[3];
        F[1] = F[3];
      }
    }

    else
    {
      if (F[3] > F[1])
      {
        alpha[0] = alpha[3];
        F[0] = F[3];
      }
      else
      {
        alpha[2] = alpha[1];
        F[2] = F[1];
        alpha[1] = alpha[3];
        F[1] = F[3];
      }
    }

    *oflag = 3;
  }

// Check to see if the value of the function at alpha[2] is greater than that
//  at alpha[1].  If so, then the quadratic which is to be fit to these points
//  will be concave up.  Otherwise, redefine the points and find a new value
//  for alpha[2].

  if (F[1] < F[2]) break;

  alpha[0] = alpha[1];
  F[0] = F[1];
  alpha[1] = alpha[2];
  F[1] = F[2];

  fact[0] = fact[0] * 1.2;
  fact[2] = fact[2] * 1.2;

  *oflag = 2;

} // End of while loop -----


// Determine the coefficients of the minimizing quadratic function -----

temp =  (alpha[1]-alpha[0])*(alpha[2]-alpha[0])*(alpha[2]-alpha[0]);
temp -= (alpha[2]-alpha[0])*(alpha[1]-alpha[0])*(alpha[1]-alpha[0]);

P = ( (F[2]-F[0])*(alpha[1]-alpha[0])-(F[1]-F[0])*(alpha[2]-alpha[0]) ) / temp;

Q = ( (F[1]-F[0]) - (alpha[1]*alpha[1] - alpha[0]*alpha[0]) * P ) /
       (alpha[1]-alpha[0]);

R = F[0] - alpha[0]*(alpha[0]*P + Q);

alpha[3] = -Q / (P+P);

F[4] = P*alpha[3]*alpha[3] + Q*alpha[3] + R;


// Define new values for the independent variables and get the value of the
//  function at the minimum of the quadratic.

for (i=0;i<n;i++) Xn[i] = X[i] + alpha[3]*dx[i];
(*nfc)++;

*oflag = 4;
return;

} // End of ods ---------------------------------------------------------------

