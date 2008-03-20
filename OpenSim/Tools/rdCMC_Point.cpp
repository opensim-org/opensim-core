// rdCMC_Point.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include "rdCMC_Point.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>

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
rdCMC_Point::rdCMC_Point(AbstractBody *aBody, SimTK::Vec3& aPoint) :
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
computeDesiredAccelerations(double time,SimTK::Vec3& acc)
{
	SimTK::Vec3 posErr, velErr;
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
computePositionError(double time,SimTK::Vec3& posErr)
{
	SimTK::Vec3 pos;
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
computeVelocityError(double time,SimTK::Vec3& velErr)
{
	SimTK::Vec3 vel;
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
	//int nx = 1;
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
	//Model::ComputeEffectiveMassMatrix(nx,nu,_j,Iinv,_m);
}
