#ifndef _nrODS_h_
#define _nrODS_h_
// ods_subs.cpp
//---------------------------------------------------------------
//
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/*  
 * Author:  
 */

//=============================================================================
// Revision Record:
// 2/2/2000 2:35 pm
//=============================================================================
//=============================================================================

double ods_func(double P,double *C,double *mu,int nconstr,int neqconstr,
			int exponent);
void ods(int n,double *X,double *F,double *dx,double epsc,double minalf,
		double maxalf,double *alpha,double *Xn,double *fact,int *oflag,
		int *nfc,int mfc);



//=============================================================================
//=============================================================================
// FOR USAGE, SEE COMMENTS BELOW

/*-----------------------------------------------------------------------------
double ods_func(double P,double *C,double *mu,int nconstr,int neqconstr,
			int exponent);

This function calculates a value of the function minimized by the one
dimensional search routine "ods".  It basically appends constraints
to a performance criterion.
_______________________________________________________________________________

Input:
  P = performance
  C = vector of constraints
  mu = constraint weight factors
  nconstr = total number of constraints
  neqconstr = number of equality constraints
  exponent = this determines how the constraints values will be applied
____________________________________________________________________________ */


//_____________________________________________________________________________
/*
void ods(int n,double *X,double *F,double *dx,double epsc,double minalf,
		double maxalf,double *alpha,double *Xn,double *fact,int *oflag,
		int *nfc,int mfc);

  This function performs a one dimensional search on the function "F" by curve
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
