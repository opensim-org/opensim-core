// rdCMC_Point.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Copyright (c) 2006 Stanford University and Realistic Dynamics, Inc.
// Contributors: Frank C. Anderson
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
// 
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE AUTHORS,
// CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
// THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include "rdCMC_Point.h"
#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/Mtx.h>

using namespace std;
using namespace OpenSim;


//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
rdCMC_Point::~rdCMC_Point()
{

}
//_____________________________________________________________________________
/**
 * Constructor.
 */
rdCMC_Point::rdCMC_Point(AbstractBody *aBody, double aPoint[3]) :
	rdCMC_Task()
{
	// NULL
	setNull();

	// BODY
	_body = aBody;

	// POINT
	_point[0] = aPoint[0];
	_point[1] = aPoint[1];
	_point[2] = aPoint[2];
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void rdCMC_Point::
setNull()
{
	setType("rdCMC_Point");
	_nTrk = 3;
	_body = NULL;
	_point[0] = _point[1] = _point[2] = 0.0;
}


//=============================================================================
// GET AND SET
//=============================================================================





//=============================================================================
// DESIRED ACCELERATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the desired accelerations.
 */
void rdCMC_Point::
computeDesiredAccelerations(double time,double acc[])
{
	double posErr[3], velErr[3];
	computePositionError(time,posErr);
	computeVelocityError(time,velErr);
	for (int i=0;i<3;i++) {
		_aDes[i] =	_ka[i]*_pTrk[i]->evaluate(2,time) + 
								_kv[i]*velErr[i] + _kp[i]*posErr[i];
	}
}

//_____________________________________________________________________________
/**
 * Compute the current position error
 */
void rdCMC_Point::
computePositionError(double time,double posErr[])
{
	double pos[3];
	_model->getDynamicsEngine().getPosition(*_body,_point,pos);
	for (int i=0;i<3;i++) {
		posErr[i] = _pTrk[i]->evaluate(0,time) - pos[i];
	}
}

//_____________________________________________________________________________
/**
 * Compute the current velocity error
 */
void rdCMC_Point::
computeVelocityError(double time,double velErr[])
{
	double vel[3];
	_model->getDynamicsEngine().getVelocity(*_body,_point,vel);
	for (int i=0;i<3;i++) {
		velErr[i] = _pTrk[i]->evaluate(1,time) - vel[i];
	}
}


//=============================================================================
// OPERATIONAL SPACE
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the Jacobian.
 */
void rdCMC_Point::
computeJacobian()
{
	_model->getDynamicsEngine().formJacobianTranslation(*_body,_point,_j);
}
//_____________________________________________________________________________
/**
 * Compute the effective mass matrix.
 */
void rdCMC_Point::
computeEffectiveMassMatrix()
{
	if(_model==NULL) return;

	// Add an integer pass that informs the routine the size of the x variable
	// HOW MANY POINTS BEING TRACKED?
	int nx = 1;
	int nu = _model->getNumSpeeds();

	// JACOBIAN
	_model->getDynamicsEngine().formJacobianTranslation(*_body,_point,_j);

	// INVERSE MASS MATRIX
	double *I = new double[nu*nu];
	double *Iinv = new double[nu*nu];
	_model->getDynamicsEngine().formMassMatrix(I);
	Mtx::Invert(nu,I,Iinv);

	// EFFECTIVE MASS MATRIX
	// TODO- This method is not available anywhere.
	//AbstractModel::ComputeEffectiveMassMatrix(nx,nu,_j,Iinv,_m);
}
