/* optimize.cpp ---------------------------------------------------------------

(c) Copyright 2001, Ncompass Research, Inc.

 This code may not be copied in whole or in part without the expressed written
 consent of Ncompass Research, inc.
_______________________________________________________________________________

This file contains routines for parameter optimization and line searching.
The main functions are:

o  ods_func     -  calculate the line search function
o  ods          -  line search
o  paramopt     -  parameter optimization

===============================================================================
Revision Record:
6/30/2001 9:00 pm
===============================================================================

============================================================================ */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define DISTL 0.05
#define DISTR 0.95

#define VLARGE 1.0e+06
#define VSMALL 1.0e-06
#define mvTOL 1.0e-08

#define IA(A,D,R,C) A[(R)*(D)+(C)]


// Prototypes

int param_initial(int, int, int, int *, int *, double *, double *, double *,
                   double *, double *, double *);
void param_update(int, int, double, double *, double *, int, double *,
                   double *, double *, double *, double *, double *);
int param_quadprog(int, int, int, double *, double *, double *, int, double,
                    double *, int *, int *, double *, double *, double *,
                    double *, double *, double *, double *, double *, double *,
                    double *, double *);
int param_searchdir(int, int, double *, double *, double *, int, double *,
                     double *, double *, double *, int *, int, double *, int,
                     int *);
int param_feasvert(int, int, double *, int, double *, double *, double *,
                    double *, int *, int, double *, int, int *);
int param_equal(int, int, int, int, int *, double *, double *, double *, int,
                 double *, double *, double *, double *, double *, int);
void param_addcn(int, int, int, int *, int, int *, double, double *, double *,
                  int);
void param_bounder(int, int, int, int *, int *, double *, double *, double *,
                    double *, int);
int param_pconviol(int, int, int, int, int *, double *, int, double *,
                    double *, double *, double *, double *, int);
double param_dotprod(int, double *, int, double *, int);

int param_minvert(double *, int, int);
int param_rowswitch(double *, int *, int, int, int);

// ----------



// This function replaces the standard C function "pow".  Since the exponent
// is always a positive integer, this function will run much faster than "pow".

double ods_pow(double mantissa, int exponent)
{
int n;
static double result;

result = mantissa;

n = 1;

while (n < exponent)
{
  result = result * mantissa;
  n++;
}

return result;

} // End of ods_pow -----------------------------------------------------------





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
                 int exponent)
{
int i;
static double returnval;

returnval = P;

if (exponent < 1) exponent = 1;

for (i=0;i<neqconstr;i++)
  returnval += ods_pow(mu[i]*fabs(C[i]),exponent);

for (i=neqconstr;i<nconstr;i++)
{
  if (C[i] < 0.0) returnval += ods_pow(mu[i]*(-C[i]),exponent);
}

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

    temp = fabs(F[3]);

    if (epsc > temp)
      Fe = (F[3] - F[4]) / epsc;
    else
      Fe = (F[3] - F[4]) / temp;

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





/* paramopt.cpp ---------------------------------------------------------------

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
               double *DX, double *MU, double *W, int *IW)
{
int i,lenIWm1,iMU,iB,iDX,iGLAG,iGAMMA,iBDX,DXlen,iGM,iCM;
int iDXl,iDXu,iH,cCN;


// Define offsets to various sections of the work vectors -----

iMU = m;
iB = iMU + m;
iDX = iB + (n+1)*(n+1);
iGLAG = iDX + n+1;
iGAMMA = iGLAG + n;
iBDX = iGAMMA + n;

DXlen = 4*(n+1);
if (m > (3*(n+1)))
  DXlen += m;
else
  DXlen += 3*(n+1);

iGM = iDX + DXlen;
iCM = iGM + n+1;
iDXl = iCM + m;
iDXu = iDXl + n+1;
iH = iDXu + n+1;

lenIWm1 = 4*(n+1) + m;

cCN = m;           // Column dimension of CN.

if (*pflag == 1)  // Initialize various values
{

  *pflag = param_initial(n,m,meq,IW,&IW[lenIWm1],&W[iMU],&W[iB],&W[iDX],
                          &W[iGM],&W[iDXl],&W[iDXu]);

  if (*pflag < 0) return;

}

else  // Update the variable metric matrix and store some values for the next
{     //  iteration.

  param_update(n,m,alpha,G,CN,cCN,W,&W[iB],&W[iDX],&W[iGLAG],&W[iGAMMA],
                &W[iBDX]);
}


// Call the quadratic programming subroutine -----

*pflag = param_quadprog(n,m,meq,G,C,CN,cCN,epsc,ctest,IW,&IW[lenIWm1],W,
                         &W[iMU],&W[iB],&W[iDX],&W[iGLAG],&W[iGAMMA],&W[iGM],
                         &W[iCM],&W[iDXl],&W[iDXu],&W[iH]);


if (*pflag >= 0) // Update the search direction and constraint weight factors,
{                //  which are passed out to the calling program.
  for (i=0;i<n;i++) DX[i] = W[iDX+i];
  for (i=0;i<m;i++) MU[i] = W[iMU+i];
}


return;

} // End of paramopt ----------------------------------------------------------




// This function sets some initial values -----

int param_initial(int n, int m, int meq, int *LT, int *ma, double *MU,
                   double *B, double *DX, double *GM, double *DXl, double *DXu)
{
int i,j,np2;


// Set the initial values of the variable metric matrix (B), constraint
//  multipliers (MU), search direction (DX), bounds on teh search directions
//  (DXl & DXu), number of active constraints (ma), the negative of G (GM),
//  and the constraint vector flag (LT).


*ma = meq + 1;

// B is extended to n+1 because of the extra variable that is introduced to
//  allow for infeasibility.

for (i=0;i<(n+1);i++)
{
  for (j=0;j<(n+1);j++)
    IA(B,n+1,i,j) = 0.0;

  IA(B,n+1,i,i) = 1.0;
}

IA(B,n+1,n,n) = 0.0;

for (i=0;i<m;i++)
  MU[i] = 0.0;

for (i=0;i<n;i++)
{
  DX[i] = 0.0;
  DXl[i] = -VLARGE;
  DXu[i] =  VLARGE;
}

DXl[n] = 0.0;
DX[n] = 1.0;


// GM is extended to n+1 because of the extra variable that is introduced to
//  allow for infeasibility.

GM[n] = VLARGE;

np2 = 2*(n+1);
for (i=0;i<meq;i++)
  LT[i] = i + np2;

LT[*ma-1] = np2 - 1;

return 0;

} // End of param_initial -----------------------------------------------------




void param_update(int n, int m, double alpha, double *G, double *CN, int cCN,
                   double *LAM, double *B, double *DX, double *GLAG,
                   double *GAMMA, double *BDX)
{
int i,j;
double dg,dbd,theta,thcomp,temp1,temp2;
double PARB=0.2;


// Search direction:
for (i=0;i<n;i++)
  DX[i] = alpha * DX[i];

// Calculate the gradient of the Lagrangian function:
for (i=0;i<n;i++)
  GLAG[i] = G[i];

for (j=0;j<m;j++)
{
  if (LAM[j] != 0.0)
  {
    for (i=0;i<n;i++)
      GLAG[i] -= IA(CN,cCN,i,j)*LAM[j];
  }
}


// Calculate GAMMA and BDX in order to revise B:
dg = 0.0;
dbd = 0.0;

for (i=0;i<n;i++)
{
  GAMMA[i] = GLAG[i] - GAMMA[i];

  BDX[i] = 0.0;
  for (j=0;j<n;j++)
    BDX[i] += IA(B,n+1,i,j)*DX[j];

  dg += DX[i]*GAMMA[i];
  dbd += DX[i]*BDX[i];
}

temp1 = PARB * dbd;


if (dg < temp1)
{
  theta = (dbd - temp1) / (dbd - dg);
  thcomp = 1.0 - theta;

  for (i=0;i<n;i++)
    GAMMA[i] = theta * GAMMA[i] + thcomp * BDX[i];

  dg = temp1;
}


dg = 1.0 / dg;
dbd = 1.0 / dbd;


// Revise the variable metric matrix (B).  This is the BFGS formula for
//  revising the matrix.  See Powell, M.J.D. (1978) A fast algorithm for
//  nonlinearly constrained optimization calculations.  In Matson, G.A.,
//  Numerical Analysis: Lecture Notes in Mathematics 630, eqn. 3.8:

for (i=0;i<n;i++)
{
  temp1 = BDX[i] * dbd;
  temp2 = GAMMA[i] * dg;

  for (j=i;j<n;j++)
  {
    IA(B,n+1,i,j) += temp2 * GAMMA[j] - temp1 * BDX[j];
    IA(B,n+1,j,i) = IA(B,n+1,i,j);
  }
}


return;

} // End of param_update ------------------------------------------------------





// This function calculates the search direction, Lagrange multipliers,
//  constraint weight factors, and check for convergence -----

int param_quadprog(int n, int m, int meq, double *G, double *C, double *CN,
                    int cCN, double epsc, double *ctest, int *LT, int *ma,
                    double *LAM, double *MU, double *B, double *DX,
                    double *GLAG, double *GAMMA, double *GM, double *CM,
                    double *DXl, double *DXu, double *H)
{
int i,j,k,mt,np,np2,feascheck,npj,np6,qp;
double gdx,absLAM,temp,FEASP=0.9;

np = n+1;
np2 = np + np;
np6 = np * 6;
mt = m + np2;


// Set the elements of GM, CM, and CN[n,i]:

for (i=0;i<n;i++)
  GM[i] = -G[i];

for (i=0;i<meq;i++)
{
  CM[i] = 0.0;
  IA(CN,cCN,n,i) = C[i];
  LAM[i] = 0.0;
}

for (i=meq;i<m;i++)
{
  if (C[i] >= 0.0)
  {
    CM[i] = -C[i];
    IA(CN,cCN,n,i) = 0.0;
  }
  else
  {
    CM[i] = 0.0;
    IA(CN,cCN,n,i) = C[i];
  }

  LAM[i] = 0.0;
}

DXu[n] = 1.0;
feascheck = -1;


// Get a new search direction:

do
{
  qp = param_searchdir(np,mt,B,GM,CN,cCN,CM,DXl,DXu,DX,ma,meq,H,np2,LT);

  if (qp < 0) return qp;             // No feasible vertex

  if (DX[n] <= VSMALL) return -2;    // Inconsistent constraints

  for (i=0;i<*ma;i++)
  {
    if (LT[i] < (np2-1))
      return -1;                     // An artificial bound is active

    else if (LT[i] == (np2-1))
      feascheck = 1;
  }


  if (!feascheck)
    return -2;                       // Inconsistent constraints

  else if (feascheck == -1)
  {
    DXu[n] = FEASP * DX[n];
    feascheck = 0;
  }

}
while (!feascheck);



// Calculate the Lagrange multipliers:

for (j=0;j<*ma;j++)
{
  k = LT[j] - np2;
  npj = np + j;

  if (k >= 0)
  {
    for (i=0;i<n;i++)
      LAM[k] += IA(H,np2,npj,i) * DX[np6+i];
  }
}


// Calculate the gradient of the Lagrangian function:

for (i=0;i<n;i++)
  GLAG[i] = G[i];


for (j=0;j<m;j++)
{
  if (LAM[j] != 0.0)
  {
    for (i=0;i<n;i++)
      GLAG[i] -= IA(CN,cCN,i,j) * LAM[j];
  }
}


// Save the gradient of the Lagrangian function for the next iteration, and
//  calculate the scalar product of G and DX:

gdx = 0.0;

for (i=0;i<n;i++)
{
  GAMMA[i] = GLAG[i];
  gdx += G[i] * DX[i];
}


// Revise the vector of constraint weight factors, and sum up the convergence
//  test value:

*ctest = fabs(gdx);

for (i=0;i<m;i++)
{
  absLAM = fabs(LAM[i]);
  temp = 0.5*(absLAM+MU[i]);

  if (absLAM > temp)
    MU[i] = absLAM;
  else
    MU[i] = temp;

  *ctest += fabs(LAM[i]*C[i]);
}


// Check for convergence:

if (*ctest <= epsc)
  return 1;
else
  return 0;

} // End of param_quadprog ----------------------------------------------------




// This function calculates the new search direction -----

int param_searchdir(int np, int mt, double *B, double *GM, double *CN, int cCN,
                     double *CM, double *DXl, double *DXu, double *DX, int *ma,
                     int meq, double *H, int iH, int *LT)
{
int i,j,ki,ia,ib,np2,np3,np4,np5,np6,npma,npi,passive,positive;
double y,z,zdp,alpha,cHc,cac,cc,gHc;

np2 = np + np;
np3 = np2 + np;
np4 = np3 + np;
np5 = np4 + np;
np6 = np5 + np;


Get_feasible_vertex: // ----------------------------------- Get_feasible_vertex

i = param_feasvert(np,mt,CN,cCN,CM,DXl,DXu,DX,ma,meq,H,iH,LT);

if (i < 0) return i;

if (*ma == 0) return 0;

// Initialize operators H = 0 and cstar = CN^-1:

for (i=0;i<np;i++)
{
  for (j=0;j<np;j++)
  {
    IA(H,iH,np+i,j) = IA(H,iH,i,j);
    IA(H,iH,i,j) = 0.0;
  }
}


Calculate_gradient: // ------------------------------------- Calculate_gradient

// Calculate gradient G and lagrange multipliers (-cstar.G), find the largest
//  multiplier, exit if it is not positive.

for (i=0;i<np;i++)
  DX[np6+i] = param_dotprod(np,&IA(B,np,i,0),1,&DX[0],1) - GM[i];


if (*ma == 0) return 0;

z = -1.0e+75;

for (i=0;i<*ma;i++)
{
  if (LT[np2+LT[i]] != -1) //  LT[np4] ... LT[np4+ma-1]
  {
    zdp = -param_dotprod(np,&IA(H,iH,np+i,0),1,&DX[np6],1);

    if (zdp > z)
    {
      z = zdp;
      ki = i;
    }
  }
}


if (z <= 0.0)  // Try again -----
{
  i = param_equal(np,np2,*ma,mt,LT,B,GM,CN,cCN,CM,DXl,DXu,DX,H,iH);

  if (i) goto Get_feasible_vertex;

  for (i=0;i<np;i++)
    DX[np6+i] = param_dotprod(np,&IA(B,np,i,0),1,&DX[0],1) - GM[i];

  z = -1.0e+75;

  for (i=0;i<*ma;i++)
  {
    if (LT[np2+LT[i]] != -1)   // LT[np4] ... LT[np4-ma-1]
    {
      zdp = -param_dotprod(np,&IA(H,iH,np+i,0),1,&DX[np6],1);

      if (zdp > z)
      {
        z = zdp;
        ki = i;
      }
    }
  }
}



if (z <= 0) return 0;  // Search direction found -----



// Set the direction of search as the corresponding row of cstar:

for (i=0;i<np;i++)
  DX[np2+i] = IA(H,iH,np+ki,i);


Set_Pos_Pass: // ------------------------------------------------- Set_Pos_Pass

for (i=0;i<np;i++)
  DX[np+i] = param_dotprod(np,&IA(B,np,i,0),1,&DX[np2],1);

cac = param_dotprod(np,&DX[np2],1,&DX[np],1);

if (cac > 0.0)
{
  positive = 1;
  y = z / cac;
}
else
{
  positive = 0;
  y = 1.0;
}


for (i=0;i<np;i++)
  DX[np5+i] = DX[np2+i] * y;


passive = 1;


Linear_search: // ----------------------------------------------- Linear_search

alpha = 1.0e+75;
npma = np + *ma;


// Perform a linear search along the search direction.  "passive" indicates
//  that a constraint has been removed to get the search direction,
//  "positive" indicates positive curvature long the direction.


for (i=0;i<mt;i++)
{
  if (LT[np2+i] > 0)
  {
    if (i >= np2)
    {
      j = i - np2;
      zdp = param_dotprod(np,&IA(CN,cCN,0,j),cCN,&DX[np5],1);

      if (zdp < 0.0)
      {
        cc = CM[j] - param_dotprod(np,&IA(CN,cCN,0,j),cCN,&DX[0],1);
        cc = cc / zdp;

        if (cc < alpha)
        {
          alpha = cc;
          ia = i;
        }
      }
    }

    else if (i >= np)
    {
      if (DX[np4+i] > 0.0)
      {
        cc = (DXu[i-np] - DX[i-np]) / DX[np4+i];

        if (cc < alpha)
        {
          alpha = cc;
          ia = i;
        }
      }
    }

    else
    {
      if (DX[np5+i] < 0.0)
      {
        cc = (DXl[i] - DX[i]) / DX[np5+i];

        if (cc < alpha)
        {
          alpha = cc;
          ia = i;
        }
      }
    }
  }
}  // End of for loop -----


if (passive) LT[np2+LT[ki]] = 1;

if (!positive || (alpha < 1.0))    // A minimum was not found.
{

// Calculate H.c and cstar.c:

  for (i=0;i<np;i++)
    DX[i] += alpha * DX[np5+i];

  alpha = alpha * y;

  if (*ma == np)
    j = np;
  else
    j = 0;

  if (ia >= np2)
  {
    ib = ia - np2;

    for (i=0;i<np;i++)
      DX[np5+i] = IA(CN,cCN,i,ib);

    for (i=j;i<npma;i++)
      DX[np3+i] = param_dotprod(np,&IA(H,iH,i,0),1,&DX[np5],1);

    if (*ma != np) cHc = param_dotprod(np,&DX[np5],1,&DX[np3],1);

  }

  else if (ia >= np)
  {
    ib = ia - np;

    for (i=j;i<npma;i++)
      DX[np3+i] = -IA(H,iH,i,ib);

    cHc = -DX[np3+ib];
  }

  else
  {
    for (i=j;i<npma;i++)
      DX[np3+i] = IA(H,iH,i,ia);

    cHc = DX[np3+ia];
  }

  LT[np2+ia] = 0;

  if (*ma == np)
  {
    z = 1.0 / DX[np4+ki];

// Apply the simplex formula to exchange constraints:

    for (i=0;i<np;i++)
    {
      npi = np + i;

      if (i == ki)
      {
        for (j=0;j<np;j++)
          IA(H,iH,npi,j) = IA(H,iH,npi,j) * z;
      }
      else
      {
        zdp = z * DX[np4+i];

        for (j=0;j<np;j++)
          IA(H,iH,npi,j) -= zdp * DX[np2+j];
      }
    }

    LT[ki] = ia;

    goto Calculate_gradient;

  }

  if (passive)
  {
    cc = DX[np4+ki];
    y = cHc * cac + cc*cc;
    gHc = param_dotprod(np,&DX[np6],1,&DX[np3],1);

    if ( (alpha*y) < (cHc*(z-alpha*cac)+gHc*cc) )
    {
      param_addcn(np,np3,np4,ma,ia,LT,cHc,DX,H,iH);

// Removal of a constraint has been deferred.  Set up as if the constraint is
//  being removed from augmented basis:

      for (i=0;i<np;i++)
      {
        DX[np6+i] = param_dotprod(np,&IA(B,np,i,0),1,&DX[0],1) - GM[i];
        DX[np2+i] = IA(H,iH,np+ki,i);
      }

      z = -param_dotprod(np,&DX[np6],1,&DX[np2],1);

      if (z == 0.0) goto Update_operators;

      goto Set_Pos_Pass;
    }

    else
    {
// Apply the formula for exchanging a new constraint with a passive constraint:

      for (i=0;i<*ma;i++)
      {
        npi = np + i;
        DX[np5+i] = param_dotprod(np,&IA(H,iH,npi,0),1,&DX[np],1);
      }

      for (i=0;i<np;i++)
      {
        DX[np+i] = ( cHc*DX[np2+i] - cc*DX[np3+i] ) / y;
        DX[np6+i] = ( cac*DX[np3+i] + cc*DX[np2+i] ) / y;
      }

      for (i=0;i<np;i++)
      {
        for (j=0;j<=i;j++)
        {
          IA(H,iH,i,j) += DX[np+i]*DX[np2+j] - DX[np6+i]*DX[np3+j];
          IA(H,iH,j,i) = IA(H,iH,i,j);
        }
      }

      DX[np4+ki] -= 1.0;

      for (i=0;i<*ma;i++)
      {
        npi = np + i;

        for (j=0;j<np;j++)
          IA(H,iH,npi,j) -= DX[np4+i]*DX[np6+j] + DX[np5+i]*DX[np+j];
      }

      LT[ki] = ia;
    }

  }  // End of if passive -----

  else
  {
    param_addcn(np,np3,np4,ma,ia,LT,cHc,DX,H,iH);
  }

  if (*ma == np) goto Calculate_gradient;

// Calculate G, new search direction is -H.G:

  for (i=0;i<np;i++)
    DX[np+i] = param_dotprod(np,&IA(B,np,i,0),1,&DX[0],1) - GM[i];

  z = 0.0;

  for (i=0;i<np;i++)
  {
    DX[np5+i] = -param_dotprod(np,&IA(H,iH,i,0),1,&DX[np],1);
    if (DX[np5+i] != 0.0) z = 1.0;
  }

  passive = 0;

  if (z == 0.0) goto Calculate_gradient;

  positive = 1;

  goto Linear_search;
}

for (i=0;i<np;i++)
  DX[i] += DX[np5+i];

// DX is now the minimum point in the basis.  Update the operator if a
//  constraint had been removed.

if (!passive) goto Calculate_gradient;


Update_operators: // ----------------------------------------- Update_operators


for (i=0;i<np;i++)
{
  alpha = DX[np2+i] / cac;

  for (j=0;j<=i;j++)
  {
    IA(H,iH,i,j) += alpha * DX[np2+j];
    IA(H,iH,j,i) = IA(H,iH,i,j);
  }
}


if (*ma <= 1)                  // No need to continue to the end of the
{                              //  function.  The same end result will occur
  *ma = 0;                     //  anyway.
  goto Calculate_gradient;
}


if (ki != *ma-1)
{
  for (i=0;i<np;i++)
    IA(H,iH,np+ki,i) = IA(H,iH,np+(*ma-1),i);

  LT[ki] = LT[*ma-1];
}

(*ma)--;

for (i=0;i<*ma;i++)
{
  npi = np + i;
  DX[np3+i] = param_dotprod(np,&IA(H,iH,npi,0),1,&DX[np],1);
}

for (i=0;i<*ma;i++)
{
  alpha = DX[np3+i] / cac;
  npi = np + i;

  for (j=0;j<np;j++)
    IA(H,iH,npi,j) -= alpha * DX[np2+j];
}

goto Calculate_gradient;


} // End of param_searchdir ---------------------------------------------------




// This function finds a feasible vertex.  In this routine, the constraints are
//  indexed as: -1 = equality, 0 = active, 1 = inactive, 2 = violated.

int param_feasvert(int np, int mt, double *CN, int cCN, double *CM,
                    double *DXl, double *DXu, double *DX, int *ma, int meq,
                    double *H, int iH, int *LT)
{
int i,j,li,npi,npj,np2,np3;

np2 = np + np;
np3 = np2 + np;


for (i=0;i<mt;i++)
  LT[np2+i] = 1;


if (*ma == 0)
{
// No constraints are designated.  The vertex will be chosen from the upper
//  and lower bounds on the search direction (DX).  This also indicates that
//  the inverse matrix will be trivial.

  for (i=0;i<np;i++)
  {
    for (j=0;j<np;j++)
      IA(H,iH,i,j) = 0.0;

    if ( (DX[i] - DXl[i]) <= (DXu[i] - DX[i]) )
    {
      LT[i] = i;
      IA(H,iH,i,i) = 0.0;
    }
    else
    {
      LT[i] = np+i;
      IA(H,iH,i,i) = -1.0;
    }

    LT[np2+LT[i]] = 0;
  }

  *ma = np;
}

else
{
// Set up the normals (v) of the "ma" designated constraints in basis:

  for (i=0;i<*ma;i++)
  {
    if (i < meq)
      LT[np2+LT[i]] = -1;
    else
      LT[np2+LT[i]] = 0;

    li = LT[i];
    npi = np + i;

    if (li >= np2)
    {
      li -= np2;

      for (j=0;j<np;j++)
        IA(H,iH,j,npi) = IA(CN,cCN,j,li);
    }
    else
    {
      for (j=0;j<np;j++)
        IA(H,iH,j,npi) = 0.0;

      if (li >= np)
        IA(H,iH,li-np,npi) = -1.0;
      else
        IA(H,iH,li,npi) = 1.0;
    }
  }


  if (*ma == np)
  {
    for (j=0;j<np;j++)
    {
      npj = np + j;

      for (i=0;i<np;i++)
        IA(H,iH,i,j) = IA(H,iH,i,npj);
    }

    if (param_minvert(H,np,iH))
    {
      for (i=0;i<np;i++)
      {
        for (j=0;j<np;j++)
          IA(H,iH,i,j) = 0.0;

        IA(H,iH,i,i) = 1.0;
      }
    }
  }

  else
  {
// Form m = (vtranspose.v)^-1:

    for (i=0;i<*ma;i++)
    {
      for (j=i;j<*ma;j++)
      {
        IA(H,iH,i,j) = param_dotprod(np,&IA(H,iH,0,np+i),iH,&IA(H,iH,0,np+j),iH);
        IA(H,iH,j,i) = IA(H,iH,i,j);
      }
    }

    if (param_minvert(H,*ma,iH))
    {
      for (i=0;i<*ma;i++)
      {
        for (j=0;j<*ma;j++)
          IA(H,iH,i,j) = 0.0;

        IA(H,iH,i,i) = 1.0;
      }
    }


// Calculate the generalized inverse of v, vplus = m.vtranspose:

    for (i=0;i<*ma;i++)
    {
      for (j=0;j<*ma;j++)
        DX[np+j] = IA(H,iH,i,j);

      for (j=0;j<np;j++)
        IA(H,iH,i,j) = param_dotprod(*ma,&DX[np],1,&IA(H,iH,j,np),1);
    }


// Set up the diagonal elements of the projection matrix p = v.vplus:

    for (i=0;i<np;i++)
      DX[np+i] = param_dotprod(*ma,&IA(H,iH,0,i),iH,&IA(H,iH,i,np),1);

    for (i=0;i<np;i++)
      LT[np+i] = 0;

// Add bound e[i] correspondingy to the smallest diag(p):

    param_bounder(np,np2,np3,ma,LT,DX,DXl,DXu,H,iH);

  } // End of if (*ma == np)
} // End of if (*ma == 0)



// Set up RHS of constraints in basis:

for (i=0;i<np;i++)
{
  li = LT[i];

  if (li >= np2)
    DX[np+i] = CM[li-np2];
  else if (li >= np)
    DX[np+i] = -DXu[li-np];
  else
    DX[np+i] = DXl[li];
}

// Calculate the position of the vertex:

for (i=0;i<np;i++)
  DX[i] = param_dotprod(np,&IA(H,iH,0,i),iH,&DX[np],1);


// Calculate the constraint residuals, the number of violated constraints,
//  and the sum of their normals:

i = param_pconviol(np,np2,np3,mt,LT,CN,cCN,CM,DX,DXl,DXu,H,iH);

return i;

} // End of param_feasvert ----------------------------------------------------





// This function sets up the matrix and RHS of the equations governing the
//  equality problem -----

int param_equal(int np, int np2, int ma, int mt, int *LT, double *B, double *GM,
                double *CN, int cCN, double *CM, double *DXl, double *DXu,
                double *DX, double *H, int iH)
{
int i,j,li,npma;
double z;


for (i=0;i<np;i++)
{
  DX[np+i] = GM[i];

  for (j=0;j<np;j++)
    IA(H,iH,i,j) = IA(B,np,i,j);
}


for (i=0;i<ma;i++)
{
  li = LT[i];

  if (li >= np2)
  {
    li -= np2;

    for (j=0;j<np;j++)
    {
      IA(H,iH,np+i,j) = IA(CN,cCN,j,li);
      IA(H,iH,j,np+i) = IA(CN,cCN,j,li);
    }
    DX[np2+i] = CM[li];
  }

  else
  {
    for (j=0;j<np;j++)
    {
      IA(H,iH,j,np+i) = 0.0;
      IA(H,iH,np+i,j) = 0.0;
    }

    if (li >= np)
    {
      li -= np;

      IA(H,iH,np+i,li) = -1.0;
      IA(H,iH,li,np+i) = -1.0;
      DX[np2+i] = -DXu[li];
    }

    else
    {
      IA(H,iH,np+i,li) = 1.0;
      IA(H,iH,li,np+i) = 1.0;
      DX[np2+i] = DXl[li];
    }
  } // End of if (li >= np2) -----

  for (j=0;j<ma;j++)
    IA(H,iH,np+i,np+j) = 0.0;

} // End of for (i=0;i<ma;i++) -----


npma = np + ma;

// Invert matrix giving operators H and cstar:

if (param_minvert(H,npma,iH))
{
  for (i=0;i<npma;i++)
  {
    for (j=0;j<npma;j++)
      IA(H,iH,i,j) = 0.0;

    IA(H,iH,i,i) = 1.0;
  }
}

for (i=0;i<np;i++)
  DX[i] = param_dotprod(npma,&IA(H,iH,0,i),iH,&DX[np],1);


// Check feasibility.  If not feasible, then exit and get a new vertex:

for (i=0;i<mt;i++)
{
  if (LT[np2+i] > 0)
  {
    if (i >= np2)
    {
      j = i - np2;
      z = param_dotprod(np,&IA(CN,cCN,0,j),cCN,&DX[0],1) - CM[j];
    }
    else if (i >= np)
      z = DXu[i-np] - DX[i-np];
    else
      z = DX[i] - DXl[i];

    if (z < 0.0) return 1;
  }
}


return 0;

} // End of param_equal -------------------------------------------------------





// This function adds a constraint -----

void param_addcn(int np, int np3, int np4, int *ma, int ia, int *LT, double cHc,
                  double *DX, double *H, int iH)
{
int i,j,npi;
double a;


for (i=0;i<*ma;i++)
{
  a = DX[np4+i] / cHc;
  npi = np + i;

  for (j=0;j<np;j++)
    IA(H,iH,npi,j) -= a * DX[np3+j];
}

LT[*ma] = ia;
(*ma)++;

for (j=0;j<np;j++)
  IA(H,iH,np+(*ma),j) = DX[np3+j] / cHc;


if (*ma < np)
{
  for (i=0;i<np;i++)
  {
    a = DX[np3+i] / cHc;

    for (j=0;j<=i;j++)
    {
      IA(H,iH,i,j) -= a *DX[np3+j];
      IA(H,iH,j,i) = IA(H,iH,i,j);
    }
  }
}
else
{
  for (i=0;i<np;i++)
    for (j=0;j<np;j++)
      IA(H,iH,i,j) = 0.0;
}


return;

} // End of param_addcn -------------------------------------------------------





// This function adds a bound to the search direction (DX) -----

void param_bounder(int np, int np2, int np3, int *ma, int *LT, double *DX,
                    double *DXl, double *DXu, double *H, int iH)
{
int i,j,k,mv;
double a,b,c;

mv = *ma;

do
{
  c = 1.0;

  for (i=0;i<np;i++)
  {
    if ( (LT[np+i] != 1) && (DX[np+i] < c) )
    {
      c = DX[np+i];
      k = i;
    }
  }

  if ( (DX[k] - DXl[k]) > (DXu[k] - DX[k]) )
    b = -1.0;
  else
    b = 1.0;

// Calculate vectors vplus.e[i] and u = e[i] - v.vplus.e[i]:

  for (i=0;i<*ma;i++)
    DX[np2+i] = b * IA(H,iH,i,k);

  for (i=0;i<np;i++)
  {
    if (LT[np+i] != 1)
      DX[np3+i] = -param_dotprod(mv,&IA(H,iH,i,np),1,&DX[np2],1);
  }

  for (i=0;i<np;i++)
    IA(H,iH,i,k) = 0.0;

  LT[np+k] = 1;

  c = 1.0 + DX[np3+k] * b;

// Update vplus and diag(p):

  for (i=0;i<np;i++)
  {
    if (LT[np+i] != 1)
    {
      a = DX[np3+i] / c;
      IA(H,iH,*ma,i) = a;

      for (j=0;j<*ma;j++)
        IA(H,iH,j,i) -= DX[np2+j] * a;

      DX[np+i] += DX[np3+i] * DX[np3+i] / c;
    }
  }

  IA(H,iH,*ma,k) = b;
  (*ma)++;

  if (b < 0.0) k += np;

  LT[np2+k] = 0;
  LT[*ma-1] = k;

}
while (*ma != np);


return;

} // End of param_bounder -----------------------------------------------------





// This function calculates the constraint residuals, the number of violated
//  constraints, and the sum of their normals -----

int param_pconviol(int np, int np2, int np3, int mt, int *LT, double *CN,
                    int cCN, double *CM, double *DX, double *DXl, double *DXu,
                    double *H, int iH)
{
int i,j,k,ki,kj,mv,ib,ia;
double alpha,beta,y,z,zDX;


while(1)
{
  mv = 0;  // Number of violated constraints.

  for (i=0;i<np;i++)
    DX[np+i] = 0.0;

  for (i=0;i<mt;i++)
  {
    if (LT[np2+i] > 0)  // Constraint is inactive or violated.
    {
      if (i >= np2)
      {
        j = i - np2;
        DX[np2+i] = param_dotprod(np,&IA(CN,cCN,0,j),cCN,&DX[0],1) - CM[j];
      }
      else if (i >= np)
        DX[np2+i] = DXu[i-np] - DX[i-np];
      else
        DX[np2+i] = DX[i] - DXl[i];

      if (DX[np2+i] < 0.0)
      {
        mv++;
        LT[np2+i] = 2;

        if (i >= np2)
        {
          for (k=0;k<np;k++)
            DX[np+k] += IA(CN,cCN,k,j);
        }

        else if (i >= np)
          DX[i] -= 1.0;
        else
          DX[np+i] += 1.0;

      }
    }
  }


  if (mv == 0) return 0;  // No more violated constraints -----


// Some constraints are still violated.  The possible directions of search are
//  the rows of H when a constraint is removed.

  z = 0.0;

  for (i=0;i<np;i++)
  {
    if (LT[np2+LT[i]] != -1)  // Not equality constraint.
    {
      y = param_dotprod(np,&IA(H,iH,i,0),1,&DX[np],1);

      if (y > z)
      {
        z = y;
        ki = i;
      }
    }
  }


  if (z <= 0.0) return -3;  // No feasible point -----


// Search for the nearest of the farthest violated constraint and the nearest
//  nonviolated, nonbasic constraint:

  alpha = 1.0e+75;
  beta = 0.0;

  for (i=0;i<np;i++)
    DX[np+i] = IA(H,iH,ki,i);

  for (i=0;i<mt;i++)
  {
    if (LT[np2+i] > 0) // Inactive or violated constraint.
    {
      if (i >= np2)
      {
        kj = i - np2;
        z = -param_dotprod(np,&DX[np],1,&IA(CN,cCN,0,kj),cCN);
      }
      else if (i >= np)
        z = DX[i];
      else
        z = -DX[np+i];

      if (LT[np2+i] != 2)  // Non-violated constraint.
      {
        if (z > 0.0)
        {
          z = DX[np2+i] / z;

          if (z < alpha)
          {
            alpha = z;
            ia = i;
          }
        }
      }

      else
      {
        LT[np2+i] = 1;  // Inactive constraint.

        if (z < 0.0)
        {
          z = DX[np2+i] / z;

          if (z > beta)
          {
            beta = z;
            ib = i;
          }
        }
      }
    } // End of if (LT[np2+i] > 0) -----
  } // End of for (i=0;i<mt;i++) -----


  if (alpha <= beta)
  {
    ib = ia;
    beta = alpha;
  }


// Exchange with the constraint being removed from the basis using the simplex
//  formula for the new H:

  LT[np2+LT[ki]] = 1;
  LT[np2+ib] = 0;
  LT[ki] = ib;


  if (ib >= np2)
  {
    ib -= np2;

    for (i=0;i<np;i++)
      DX[np3+i] = IA(CN,cCN,i,ib);

    for (i=0;i<np;i++)
      DX[np2+i] = param_dotprod(np,&IA(H,iH,i,0),1,&DX[np3],1);
  }

  else if (ib >= np)
  {
    ib -= np;

    for (i=0;i<np;i++)
      DX[np2+i] = -IA(H,iH,i,ib);
  }

  else
  {
    for (i=0;i<np;i++)
      DX[np2+i] = IA(H,iH,i,ib);
  }

  z = 1.0 / DX[np2+ki];

  for (i=0;i<np;i++)
  {
    DX[i] += beta * DX[np+i];

    if (i == ki)
    {
      for (j=0;j<np;j++)
        IA(H,iH,i,j) = IA(H,iH,i,j) * z;
    }
    else
    {
      zDX = z * DX[np2+i];

      for (j=0;j<np;j++)
        IA(H,iH,i,j) -= zDX * DX[np+j];
    }
  }

} // End of while loop -----

} // End of param_pconviol ----------------------------------------------------






// This function calculates the dot product of the vectors sx and sy.  Because
//  these vectors may be columns or rows in the calling function, the
//  increments "incx" and "incy" are included to step to the appropriate
//  elements of the arrays.

double param_dotprod(int n, double *SX, int incx, double *SY, int incy)
{
int i,ix,iy;
static double sum;

ix = 0;
iy = 0;

sum = 0.0;

for (i=0;i<n;i++)
{
  sum += SX[ix] * SY[iy];

  ix += incx;
  iy += incy;
}

return sum;

} // End of param_dotprod -----------------------------------------------------






/* param_minvert.cpp ----------------------------------------------------------

This function calculates the inverse of a matrix using a modification of a
 simple Gaussian elimination scheme.  It uses an "invert in place" algorithm.

 (c) Copyright, Ncompass Research, Inc., 2001

 This code may not be copied in whole or in part without the expressed written
 consent of Ncompass Research, inc.

===============================================================================
Revision Record:
 6/17/2001  8:45 pm
===============================================================================

Input:
     n = size of the matrix to be inverted ( n <= m )
     m = row and column dimension of "mat"

Input and Output:
   mat = matrix (m x m)

Return values:
  0 = successful completion
  1 = n > m
  2 = cannot allocate memory for the switch indexes
  3 = an entire row is less than the specified tolerance

____________________________________________________________________________ */


int param_minvert(double *mat, int n, int m)
{
int i,k,row,col,e1,e2,e3;
int *idx;
double hold;

if (n > m) return 1;                 /* Error checking                    */


/* Allocate memory for the row indexes */
idx = (int *)calloc(n,sizeof(int));

if (!idx) return 2;                      /* Can't allocate memory */

for (row=0;row<n;row++) idx[row] = row;  /* Set the initial row indexes */


/* Reduce "mat" to an upper triangular matrix while setting the diagonal
   elements to unity                                                         */

for (row=0;row<n;row++)
{

/* Perform partial pivoting on the matrix to reduce round-off errors.        */
  if(param_rowswitch(mat,idx,row,n,m))
  {
    free(idx);
    return 3;
  }


/* Scale the elements of this row by the value of the diagonal element.      */
  
  e1 = row*(m+1);                 /* Scale the diagonal element              */
  mat[e1] = 1.0 / mat[e1];

  for (col=0;col<n;col++)
  {
    e2 = row*m + col;
    if (row != col) mat[e2] = mat[e2] * mat[e1];
  }

/* Forward substitution. Reduce the matrix "mat" to an upper triangular one. */
    
  for (i=row+1;i<n;i++) {
    e1 = i*m + row;

    for (col=0;col<n;col++)
    {
      e2 = i*m + col;
      e3 = row*m + col;
      if (col != row) mat[e2] = mat[e2] - mat[e3] * mat[e1];
    }
    mat[e1] =  - mat[e1] * mat[row*(m+1)];

  }

}


/*  Backward substitution.  Reduce "mat" to the identity matrix.             */

for (row=0;row<n-1;row++)
{
  for (i=row+1;i<n;i++)
  {
    e1 = row*m + i;

    for (col=0;col<n;col++)
    {
      e2 = row*m + col;
      e3 = i*m + col;
      if (col != i) mat[e2] = mat[e2] - mat[e3] * mat[e1];
    }
    mat[e1] =  - mat[e1] * mat[i*m+i];

  }
}

/* Reorder the columns of "mat" to undo any changes as a result of row
   switching                                                                 */

for (row=0;row<n;row++)
{
  while (idx[row] != row)
  {
    i = idx[row];

    e1 = row;
    e2 = i;

    for (k=0;k<n;k++)
    {
      hold = mat[e1];
      mat[e1] = mat[e2];
      mat[e2] = hold;

      e1 += m;
      e2 += m;
    }

    k = idx[i];
    idx[i] = i;
    idx[row] = k;

  } /* End of while loop */

}

free(idx);

return 0;

} /* End of param_minvert -------------------------------------------------- */




/* This routine performs partial pivoting on the matrix.  The current row is
   switched with another one depending on the value of the appropriately
   scaled column element below the diagonal.  The scale factor for each row
   is the maximum magnitude of all the elements in the row.  The current row
   is switched with the row that has the greatest ratio of its column element
   below the diagonal to its scale factor.

   smax[i] = max ( abs(mat[i][j]) )  row <= i <= n  and  row < j <= n

   Switch the row with max(abs(mat[i][j])) / smax[i]  with the current row.  */


int param_rowswitch(double *mat, int *idx, int row, int n, int m)
{
int i,e1,e2,e3,col,imax;
double hold,rmax,smax,absmat;

rmax = 0.0;

for (i=row;i<n;i++)
{
  e1 = i*m + row;

  smax = fabs(mat[e1]);

  e2 = i*m + row;
  for (col=row+1;col<n;col++)
  {
    e2++;
    if (fabs(mat[e2]) > smax) smax = fabs(mat[e2]);
  }

/* If the largest element in row  i  is less than or equal to the tolerance
   value, then this matrix is not invertible.                                */

  if (smax <= mvTOL) return 1;

  absmat = fabs(mat[e1]) / smax;

  if (absmat > rmax)
  {
    rmax = absmat;
    imax = i;
  }

}


/* Switch the rows */

if (imax != row)
{
  i = idx[row];
  idx[row] = idx[imax];
  idx[imax] = i;

  for (col=0;col<n;col++) {
    e2 = row*m + col;
    e3 = imax*m + col;
	  
    hold = mat[e2];        
    mat[e2] = mat[e3];
    mat[e3] = hold;
  }
}

return 0;

} /* End of param_rowswitch ------------------------------------------------ */

