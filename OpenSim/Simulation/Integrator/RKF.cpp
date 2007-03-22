// RKF.cpp
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

#include <math.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Exception.h>
#include "RKF.h"



using namespace OpenSim;
using namespace std;

//============================================================================
// INTERNAL TOLERANCE CONSTANTS
//============================================================================
const double TOLMAX = 1.0;
const double TOLMIN = 1.0e-12;
const double TOLFINERATIO = 1.0e-2;

//============================================================================
// INTERMAL RKF CONSTANTS
//============================================================================
const double C21	=  2.500000e-1;
const double C22	=  2.500000e-1;

const double C31	=  3.750000e-1;
const double C32	=  9.375000e-2;
const double C33	=  2.812500e-1;

const double C41	=  9.230769e-1;
const double C42	=  8.793810e-1;
const double C43	= -3.277196;
const double C44	=  3.320892;

const double C51	=  1.000000;
const double C52	=  2.032407;
const double C53	= -8.000000;
const double C54	=  7.173489;
const double C55	= -2.058967e-1;

const double C61	=  5.000000e-1;
const double C62	= -2.962963e-1;
const double C63	=  2.000000;
const double C64	= -1.381676;
const double C65	=  4.529727e-1;
const double C66	= -2.750000e-1;

const double CE1	=  2.777778e-3;
const double CE2	= -2.994152e-2;
const double CE3	= -2.919989e-2;
const double CE4	=  2.000000e-2;
const double CE5	=  3.636364e-2;

const double CY1	=  1.157407e-1;
const double CY2	=  5.489279e-1;
const double CY3	=  5.353314e-1;
const double CY4	= -2.000000e-1;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 *
 * @param aIntegrand Integrand
 * @param aTol Error tolerance.  If the estimated error is greater than
 * aTol, the integration step size is halved.
 * @param aTolFine Fine tolerance.  If the estimated error is less than
 * aFineTol, the integration step size is doubled.
 */
RKF::RKF(Integrand *aIntegrand,double aTol,double aTolFine)
{
	// MODEL
	_integrand = aIntegrand;
	if(_integrand == NULL) {
		string msg = "RKF.RKF: ERR- Null integrand.\n";
		throw Exception(msg,__FILE__,__LINE__);
	}

	// ARRAY POINTERS
	_yv = _ye = _dy = NULL;
	_k1 = _k2 = _k3 = _k4 = _k5 = _k6 = NULL;

	// TOLERANCES
	_tol = aTol;
	if(aTolFine>=0.0) {
		_tolFine = aTolFine;
	} else {
		_tolFine = 1.0e-2 * _tol;
	}
	
	// ALLOCATE MEMORY
	allocateMemory();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
RKF::~RKF()
{
	freeMemory();
}


//-----------------------------------------------------------------------------
// MEMORY
//-----------------------------------------------------------------------------
//____________________________________________________________________________
/**
 * Allocate the memory needed to perform the integration.
 */
int RKF::
allocateMemory()
{
	int size = _integrand->getSize();
	_yv = new double[size];
	_ye = new double[size];
	_dy = new double[size];
	_k1 = new double[size];
	_k2 = new double[size];
	_k3 = new double[size];
	_k4 = new double[size];
	_k5 = new double[size];
	_k6 = new double[size];

	return(0);
}
//____________________________________________________________________________
/**
 * Free the memory needed to perform the integration.
 */
int RKF::
freeMemory()
{
	if(_yv!=NULL) { delete []_yv;  _yv = NULL; }
	if(_ye!=NULL) { delete []_ye;  _ye = NULL; }
	if(_dy!=NULL) { delete []_dy;  _dy = NULL; }
	if(_k1!=NULL) { delete []_k1;  _k1 = NULL; }
	if(_k2!=NULL) { delete []_k2;  _k2 = NULL; }
	if(_k3!=NULL) { delete []_k3;  _k3 = NULL; }
	if(_k4!=NULL) { delete []_k4;  _k4 = NULL; }
	if(_k5!=NULL) { delete []_k5;  _k5 = NULL; }
	if(_k6!=NULL) { delete []_k6;  _k6 = NULL; }

	return(0);
}

//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// MODEL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the model.
 */
Integrand* RKF::
getIntegrand()
{
	return(_integrand);
}


//-----------------------------------------------------------------------------
// TOLERANCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the tolerances of the integrator.
 *
 * The tolerance, aTol, specifies the allowd error of an integration.
 * If the error exceeds the tolerance, the integration step size should
 * be halved.
 *
 * The fine tollerance, aTolFine, specifies the accuracy tolerance of an
 * integration. If the accuracy is better than aTolFine, the integration step
 * size should be doubled.
 */
void RKF::
setTolerance(double aTol,double aTolFine)
{
	// TOLERANCE
	if(aTol<TOLMIN) {
		aTol = TOLMIN;
	} else if(aTol>TOLMAX) {
		aTol = TOLMAX;
	}
	_tol = aTol;
	
	// FINE TOLERANCE
	setFineTolerance(aTolFine);
}
//_____________________________________________________________________________
/**
 * Set the fine tolerance of the integrator.
 *
 * The fine tollerance, aTolFine, specifies the accuracy tolerance of an
 * integration. If the accuracy is better than aTolFine, the integration step
 * size should be doubled.
 */
void RKF::
setFineTolerance(double aTolFine)
{	
	if((aTolFine<=0.0)||(aTolFine>=_tol)) {
		aTolFine = TOLFINERATIO * _tol;
	}
	_tolFine = aTolFine;
}

//_____________________________________________________________________________
/**
 * Get the tolerance of the integrator.
 *
 * The tolerance specifies the allowd error of an integration.
 * If the error exceeds the tolerance, the integration step size is halved.
 */
double RKF::
getTolerance()
{
	return(_tol);
}
//_____________________________________________________________________________
/**
 * Get the fine tolerance of the integrator.
 *
 * The fine tollerance specifies the accuracy tolerance of an integration.
 * If the accuracy is better than aTolFine, the integration step size is
 * doubled.
 */
double RKF::
getFineTolerance()
{
	return(_tolFine);
}


//=============================================================================
// INTEGRATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Step forward in time by dt.
 *
 * @param dt Requested integration time step.
 * @param t Current time.
 * @param y States.
 * @return RKF_NORMAL on a successful step, RKF_NAN if not-a-number is
 * encountered in any of the states, RKF_POOR if the integration error
 * is worse than the specified tolerance, or RKF_FINE if the integration
 * error is better than the specified fine tolerance.
 */
int RKF::
step(double dt,double t,double *y)
{
	int i,offender;
	double t2,yemax;

	// SIZE
	int size = _integrand->getSize();
	//if(size<1) {
	//	printf("rkf.step:  ERROR- the number of states is less than 1.\n");
	//	return(RKF_ERROR);
	//}

	// ORDER 1
	_integrand->compute(t,y,_dy);
	for(i=0;i<size;i++) {
		_k1[i] = dt*_dy[i];
		_yv[i] = y[i] + _k1[i]*C22;
	}
	t2 = t + dt*C21;

	// ORDER 2
	_integrand->compute(t2,_yv,_dy);
	for(i=0;i<size;i++) {
		_k2[i] = dt*_dy[i];
		_yv[i] = y[i] + C32*_k1[i] + C33*_k2[i];
	}
	t2 = t + dt*C31;

	// ORDER 3
	_integrand->compute(t2,_yv,_dy);
	for(i=0;i<size;i++) {
		_k3[i] =  dt*_dy[i];
		_yv[i] = y[i] + C42*_k1[i] + C43*_k2[i] + C44*_k3[i];
	}
	t2 = t + dt*C41;

	// ORDER 4
	_integrand->compute(t2,_yv,_dy);
	for(i=0;i<size;i++) {
		_k4[i] = dt*_dy[i];
		_yv[i] = y[i] + C52*_k1[i] + C53*_k2[i] + C54*_k3[i] + C55*_k4[i];
	}
	t2 = t + dt;

	// ORDER 5
	_integrand->compute(t2,_yv,_dy);
	for(i=0;i<size;i++) {
		_k5[i] = dt*_dy[i];
		_yv[i] = y[i] + C62*_k1[i] + C63*_k2[i] + C64*_k3[i] + C65*_k4[i] + C66*_k5[i];
	}
	t2 = t + dt*C61;

	// ORDER 6
	_integrand->compute(t2,_yv,_dy);
	for(i=0;i<size;i++) {
		_k6[i] = dt*_dy[i];
	}


	// GET MAX ERROR
	offender = -1;
	for(yemax=0.0,i=0;i<size;i++) {

		_ye[i] = CE1*_k1[i] + CE2*_k3[i] + CE3*_k4[i] + CE4*_k5[i] + CE5*_k6[i];

		// GOOD NUMBER
		if((_ye[i]<0.0) || (_ye[i]>=0.0)) {  
			_ye[i] = fabs(_ye[i]);
			if(_ye[i]>yemax) {
				offender = i;
				yemax = _ye[i];
			}

		// NAN
		} else {
			printf("RKF.step: NAN in state %d at time %lf (dt=%lf).\n",
				i,t,dt);
			return(RKF_NAN);
		}
	}

	// CHECK ERROR
	if(yemax > _tol) {
		//printf("rkf: error exceeded tolerance. offender = %d\n",offender);
		return(RKF_POOR);
	}

	// UPDATE STATES
	for(i=0;i<size;i++) {
		y[i] = y[i] + CY1*_k1[i] + CY2*_k3[i] + CY3*_k4[i] + CY4*_k5[i];
	}

	// FINE ACCURACY
	if(yemax < _tolFine) {
		return(RKF_FINE);
	}

	return(RKF_NORMAL);
}

//_____________________________________________________________________________
/**
 * Step forward in time by dt.  This method does not estimate the integration
 * error and does not return any kind of integration status other than
 * RKF_NORMAL or RKF_NAN.
 *
 * @param dt Requested integration step.
 * @param t Current time.
 * @param controlSet Controls.
 * @param y States.
 * @return RKF_NORMAL on a successful step, or RKF_NAN if not-a-number is
 * encountered in any of the states.
 */
int RKF::
stepFixed(double dt,double t,double *y)
{
	int i;
	double t2;

	// SIZE
	int size = _integrand->getSize();

	// ORDER 1
	_integrand->compute(t,y,_dy);
	for(i=0;i<size;i++) {
		_k1[i] = dt*_dy[i];
		_yv[i] = y[i] + _k1[i]*C22;
	}
	t2 = t + dt*C21;

	// ORDER 2
	_integrand->compute(t2,_yv,_dy);
	for(i=0;i<size;i++) {
		_k2[i] = dt*_dy[i];
		_yv[i] = y[i] + C32*_k1[i] + C33*_k2[i];
	}
	t2 = t + dt*C31;

	// ORDER 3
	_integrand->compute(t2,_yv,_dy);
	for(i=0;i<size;i++) {
		_k3[i] =  dt*_dy[i];
		_yv[i] = y[i] + C42*_k1[i] + C43*_k2[i] + C44*_k3[i];
	}
	t2 = t + dt*C41;

	// ORDER 4
	_integrand->compute(t2,_yv,_dy);
	for(i=0;i<size;i++) {
		_k4[i] = dt*_dy[i];
		_yv[i] = y[i] + C52*_k1[i] + C53*_k2[i] + C54*_k3[i] + C55*_k4[i];
	}
	t2 = t + dt;

	// ORDER 5
	_integrand->compute(t2,_yv,_dy);
	for(i=0;i<size;i++) {
		_k5[i] = dt*_dy[i];
		_yv[i] = y[i] + C62*_k1[i] + C63*_k2[i] + C64*_k3[i] + C65*_k4[i] + C66*_k5[i];
	}

	// UPDATE STATES
	for(i=0;i<size;i++) {
		y[i] = y[i] + CY1*_k1[i] + CY2*_k3[i] + CY3*_k4[i] + CY4*_k5[i];

		// CHECK FOR NAN
		if(!((y[i]<0.0)||(y[i]>=0.0))) {
			printf("RKF_fixed.step: NAN in state %d\n",i);
			return(RKF_NAN);
		}
	}

	return(RKF_NORMAL);
}
