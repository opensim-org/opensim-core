#ifndef _paramopt_h_
#define _paramopt_h_


/* This function calculates a value of the function minimized by the one
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

double ods_func(double P, double *C, double *mu, int nconstr, int neqconstr,
                 int exponent);

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

void ods(int n, double *X, double *F, double *dx, double epsc, double minalf,
          double maxalf, double *alpha, double *Xn, double *fact, int *oflag,
          int *nfc, int mfc);

/* paramopt.h -----------------------------------------------------------------

(c) Copyright 2001, Ncompass Research, Inc.

No part of this code may be copied or duplicated by any means wthout the
expressed written consent of the author.
_______________________________________________________________________________

This routine performs a parameter optimization on a multi-variable function.
 It returns a direction of descent toward the minimum of the function.
_______________________________________________________________________________

Input:
    n = number of independent variables
    m = total number of constraints
  meq = number of equality constraints (meq <= m)
    G = vector of partial derivatives of the function with respect to the
         controls
    C = vector of constraints
   CN = vector of partial derivatives of the constraints with repect to the
         controls.  This represents a 2 dimensional array which must have
         dimensions consistent with an equivalent array CN[n+1][m].  Thus,
         the vector must have a length of (n+1)*m or greater.
alpha = last line search step length.  This can be undefined for the initial
         call to paramopt.
 epsc = required convergence value

Output:
   DX = direction of descent
   MU = constraint weight factors (for the line search in the direction DX)
ctest = convergence test value (ctest <= epsc at convergence)

Input and Output:
pflag = a flag indicating the status of the parameter optimization routine.
         The calling program must not change this value.

      The possible values for pflag are:
       -3 = no feasible point.
       -2 = the given constraints are inconsistent
       -1 = an artificial bound is active
        0 = convergence not yet achieved (this will be the value until
             convergence or an error occurs)
        1 = convergence achieved (this must also be the initial input)


Work Vectors:
  W = double  [minimum length = 5*n*n + 17*n + 3*m + 12 + max( m , 3*(n+1) )]
 IW = int [minimum length = 4*(n+1) + m + 1]
____________________________________________________________________________ */

void paramopt(int n, int m, int meq, double *G, double *C, double *CN,
               double alpha, double epsc, double *ctest, int *pflag,
               double *DX, double *MU, double *W, int *IW);

#endif
