#ifndef _paramopt_h_
#define _paramopt_h_
// paramopt.h
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


#ifdef __cplusplus
extern "C" {
#endif

void
paramopt_(int *n,int *m,int *meq,double *X,double *F,double *G,
	double *C,double *CN,int *rCN,double *alpha,double *epsc,double *ctest,
	int *pflag,double *W,int *lenW,int *IW,int *lenIW,
	double *DX,double *MU);

#ifdef __cplusplus
}
#endif


//=============================================================================
/**
 * USER NOTES BELOW
 *
 *

c  This routine performs a parameter optimization on a multi-variable function.
c   It returns a direction of descent toward the minimum of the function.

c==============================================================================
c Revision record:
c  9/5/94  1:00 pm
c==============================================================================

c  Input:
c     n = number of independent variables
c     m = total number of constraints
c   meq = number of equality constraints (meq <= m)
c     X = vector of independent variables
c     F = value of the function at X
c     G = vector of partial derivatives of F with respect to X
c     C = vector of constraints
c    CN = array of partial derivatives of the constaints with respect to X
c   rCN = row dimension of CN ( rCN >= n+1 )
c alpha = last line search step length. This can be undefined for the initial
c          call to PARAMOPT.
c  epsc = required convergence value
c  lenW = length of the real work vector.  This value must be at least:
c          4*n*n + 17*n + 3*m + 18 + max0(m,3*(n+1))
c lenIW = length of the integer work vector.  This value must be at least:
c          m + 6*(n+1) + 1
c
c  Output:
c    DX = direction of descent
c    MU = constraint weight factors (for the line search in the direction DX)
c ctest = convergence test value ( ctest <= epsc at convergence)
c
c  Other:
c    W = real work vector.
c   IW = integer work vector.
c
c Input and Output:
c   pflag = a flag indicating the status of the parameter optimization routine.
c            The calling program must not change this value.
c
c         The possible values for pflag are:
c         -6 = no feasible point
c         -5 = the given constraints are inconsistent
c         -4 = an artificial bound is active
c         -3 = the length of the integer work vector is too short
c         -2 = the length of the real work vector is too short
c         -1 = n, m, meq, or rCN are inconsistent
c          0 = convergence not yet achieved (this will be the value until
c               convergence or an error occurs)
c          1 = convergence achieved (this must also be the initial input)
c______________________________________________________________________________

 *
 * END USER NOTES
 */
