#ifndef _RKF_h_
#define _RKF_h_
// RKF.h
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */

// INCLUDES
#include "rdIntegrator.h"
#include "Integrand.h"

#ifdef SWIG
	#ifdef RDSIMULATION_API
		#undef RDSIMULATION_API
		#define RDSIMULATION_API
	#endif
#endif

//=============================================================================
//=============================================================================
/**
 * A class which integrates the equations of motion forward one step in time.
 *
 * The integration method is based on a Runge-Kutta-Feldberg 5-6 Variable-Step
 * Integrator adapted from one given by Atkinson, L.V., et al,
 * "Numerical Methods with Fortran 77.", pp. 310-322, Addison-Wesley
 * Publishing Company, 1989.
 *
 * The user must supply a pointer to an Model on construction.
 */
namespace OpenSim { 

class RDSIMULATION_API RKF
{
//=============================================================================
// DATA
//=============================================================================
public:
	enum status {
		RKF_NORMAL=20,
		RKF_FINE=30,
		RKF_POOR=-40,
		RKF_NAN=-50,
		RKF_ERROR=-60,
		RKF_TOO_MANY_STEPS=-70
	};
protected:
	/** Integrand. */
	Integrand *_integrand;
	/** Integration tolerance. */
	double _tol;
	/** Integration fine tolerance. */
	double _tolFine;
	/** Work arrays for computing the new states. */
	double *_yv,*_ye,*_dy;
	double *_k1,*_k2,*_k3,*_k4,*_k5,*_k6;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	RKF(Integrand *aIntegrand,double aTol=1.0e-4,double aTolFine=-1.0);
	virtual ~RKF();

	//--------------------------------------------------------------------------
	// MEMORY
	//--------------------------------------------------------------------------
private:
	int allocateMemory();
	int freeMemory();

	//--------------------------------------------------------------------------
	//	GET AND SET
	//--------------------------------------------------------------------------
public:
	// MODEL
	Integrand* getIntegrand();
	// TOLERANCE
	void setTolerance(double aTol,double aTolFine=-1.0);
	double getTolerance();
	void setFineTolerance(double aFineTol);
	double getFineTolerance();

	//--------------------------------------------------------------------------
	// INTEGRATION
	//--------------------------------------------------------------------------
	int step(double dt,double t,double *y);
	int stepFixed(double dt,double t,double *y);

//=============================================================================
};	// END class RKF

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __RKF_h__
