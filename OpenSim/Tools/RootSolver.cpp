// RootSolver.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include <iostream>
#include <string>
#include <float.h>
#include "Math.h"
#include "RootSolver.h"




using namespace OpenSim;
using namespace std;


//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
RootSolver::~RootSolver()
{
}

//_____________________________________________________________________________
/**
 * Default constructor.
 */
RootSolver::
RootSolver(VectorFunctionUncoupledNxN *aFunc)
{
	setNull();
	_function = aFunc;
}

//-----------------------------------------------------------------------------
// CONSTRUCTION METHODS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set all member variables to NULL values.
 */
void RootSolver::
setNull()
{
	_function = NULL;
}


//=============================================================================
// SOLVE
//=============================================================================
//_____________________________________________________________________________
/**
 * Solve for the roots.
 *
 * 
 */
Array<double> RootSolver::
solve(const Array<double> &ax,const Array<double> &bx,
		const Array<double> &tol)
{
	int i;
	int N = _function->getNX();

	Array<double> a(0.0,N),b(0.0,N),c(0.0,N);
	Array<double> fa(0.0,N),fb(0.0,N),fc(0.0,N);
	Array<double> prev_step(0.0,N);
	Array<double> tol_act(0.0,N);
	Array<double> p(0.0,N);
	Array<double> q(0.0,N);
	Array<double> new_step(0.0,N);

	bool finished = false;
	Array<int>   converged(0,N);


	// INITIALIZATIONS
	a = ax;
	b = bx;
	_function->evaluate(a,fa);
	_function->evaluate(b,fb);
	c = a;
	fc = fa;


	// ITERATION LOOP
	int iter;
	for(iter=0;!finished;iter++) {

		// ABSCISSAE MANIPULATION LOOP
		for(i=0;i<N;i++) {

			// Continue?
			// If a function is already converged no need to do any manipulation.
			if(converged[i]) continue;
   
			// Make c on opposite side of b.
			// (was down at very bottom)
			 if( (fb[i]>0.0 && fc[i]>0.0) || (fb[i]<0.0 && fc[i]<0.0) ) {
				c[i] = a[i];
				fc[i] = fa[i];
			 }

			// Record previous step
			prev_step[i] = b[i] - a[i];

			// Swap data for b to be the best approximation.
			if( fabs(fc[i]) < fabs(fb[i]) ) {
				a[i] = b[i];  b[i] = c[i];  c[i] = a[i];
				fa[i]= fb[i]; fb[i]= fc[i]; fc[i]= fa[i];
			}
			tol_act[i] = 2.0*DBL_EPSILON*fabs(b[i]) + 0.5*tol[i];
			new_step[i] = 0.5 * (c[i]-b[i]);

			// Converged?
			// Original convergence test:
			if(fabs(new_step[i])<=tol_act[i] || fb[i]==(double)0.0 ) {
				converged[i] = iter;
				continue;
			}

			// Interpolate if prev_step was large enough and in true direction
			if( fabs(prev_step[i])>=tol_act[i] && fabs(fa[i])>fabs(fb[i]) ) {
				register double t1,cb,t2;
				cb = c[i]-b[i];

				// Only two distinct roots, must use linear interpolation.
				if(a[i]==c[i]) {
					t1 = fb[i]/fa[i];
					p[i] = cb*t1;
					q[i] = 1.0 - t1;

				// Quadratic interpolation
				} else {
					q[i] = fa[i]/fc[i];  t1 = fb[i]/fc[i];  t2 = fb[i]/fa[i];
					p[i] = t2 * ( cb*q[i]*(q[i]-t1) - (b[i]-a[i])*(t1-1.0) );
					q[i] = (q[i]-1.0) * (t1-1.0) * (t2-1.0);
				}

				// Change sign of q or p?
				if( p[i]>(double)0.0 ) {
					q[i] = -q[i];
				} else {
					p[i] = -p[i];
				}

				// If the interpolate is bad, use bissection.
				if( p[i]<(0.75*cb*q[i] - 0.5*fabs(tol_act[i]*q[i]))	&& p[i]<fabs(0.5*prev_step[i]*q[i]) )
					new_step[i] = p[i]/q[i];
			}

			// Adjust step to be not less than tolerance.
			if( fabs(new_step[i]) < tol_act[i] )
				if( new_step[i] > (double)0.0 )
					new_step[i] = tol_act[i];
				else
					new_step[i] = -tol_act[i];

			// Save previous approximation.
			a[i] = b[i];  fa[i] = fb[i];


			b[i] += new_step[i];
			 
		} // END ABSCISSAE LOOP
	 

		// NEW FUNCTION EVALUATION
		_function->evaluate(b,fb);


		// FINISHED?
		for(i=0;i<N;i++) {
			finished = true;
			if(!converged[i]) {
				finished = false;
				break;
			}
		}
	}

	// PRINT
	cout<<"\n\nRootSolver:  found solution in "<<iter<<" iterations.\n";
	cout<<"converged array:\n";
	cout<<converged<<endl<<endl;
	//cout<<"roots:\n";
	//cout<<b<<endl<<endl;
	//cout<<"errors:\n";
	//cout<<fb<<endl;
	
	return(b);
}
