// LinearSpring.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson and Saryn Goldberg
// 
//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University, All rights reserved. 
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Common/VectorGCVSplineR1R3.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Model/AbstractBody.h>
#include <OpenSim/Common/VectorFunction.h>
#include "LinearSpring.h"
#include "SimTKcommon.h"

using namespace OpenSim;
using namespace std;
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
LinearSpring::~LinearSpring()
{
}
//_____________________________________________________________________________
/**
 * Construct a derivative callback instance for applying external forces
 * during an integration.
 *
 * @param aModel Model for which external forces are to be applied.
 */
LinearSpring::
LinearSpring(Model *aModel, AbstractBody *aBody) :
	ForceApplier(aModel,aBody)
{
	setNull();
	setType("LinearSpring");
}

//_____________________________________________________________________________
/**
 * Set member variables to approprate NULL values.
 */
void LinearSpring::
setNull()
{
	_targetPosition = NULL;
	_targetVelocity = NULL;
	_scaleFunction = NULL;
	_scaleFactor = 1.0;
	_threshold = 0.0;
	_k[0] = _k[1] = _k[2] = 0.0;
	_b[0] = _b[1] = _b[2] = 0.0;
}

//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// TARGET POSITION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the vector function containing the (t,x,y,z) of the position that the
 * point should be corrected towards, expressed in the global ref frame.
 *
 * @param aTargetPosition Vector function containing the target position
 * expressed in the global reference frame.
 */
void LinearSpring::
setTargetPosition(VectorFunction* aTargetPosition)
{
	_targetPosition = aTargetPosition;
}
//_____________________________________________________________________________
/**
 * Get the vector function containing the (t,x,y,z) of the poisiton that the point
 * should be corrected towards, expressed in the global ref frame.
 *
 * @return Vector function containing the target position
 * expressed in the global reference frame.
 */
VectorFunction* LinearSpring::
getTargetPosition() const
{
	return(_targetPosition);
}

//-----------------------------------------------------------------------------
// TARGET VELOCITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the vector function containing the (t,x,y,z) of the velocity that the
 * point should be corrected towards, expressed in the global ref frame.
 *
 * @param aTargetVelocity Vector function containing the target velocity
 * expressed in the global reference frame.
 */
void LinearSpring::
setTargetVelocity(VectorFunction* aTargetVelocity)
{
	_targetVelocity = aTargetVelocity;
}
//_____________________________________________________________________________
/**
 * Get the vector function containing the (t,x,y,z) of the velocity that the
 * point should be corrected towards, expressed in the global ref frame.
 *
 * @return Vector function containing the target velocity
 * expressed in the global reference frame.
 */
VectorFunction* LinearSpring::
getTargetVelocity() const
{
	return(_targetVelocity);
}

//-----------------------------------------------------------------------------
// K VALUE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the spring constant, k.
 *
 * @param aK Vector of three values of k.
 */
void LinearSpring::
setKValue(const SimTK::Vec3& aK)
{
	_k = aK;
}
//_____________________________________________________________________________
/**
 * Get the spring constant, k.
 *
 * @return aK.
 */
void LinearSpring::
getKValue(SimTK::Vec3& aK) const
{
	memcpy(&aK[0],&_k[0],3*sizeof(double));
}

//-----------------------------------------------------------------------------
// B VALUE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the damping constant, b.
 *
 * @param aK Vector of three values of b.
 */
void LinearSpring::
setBValue(const SimTK::Vec3& aB)
{
	_b = aB;

}
//_____________________________________________________________________________
/**
 * Get the damping constant, b.
 *
 * @return aB.
 */
void LinearSpring::
getBValue(SimTK::Vec3& aB) const
{
	memcpy(&aB[0],&_b[0],3*sizeof(double));
}

//-----------------------------------------------------------------------------
// THRESHOLD
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the magnitude theshold below which no force is applied.
 *
 * @param aThreshold Magnitude threshold.  A negative value or 0 will result
 * in the force always being applied.
 */
void LinearSpring::
setThreshold(double aThreshold)
{
	_threshold = aThreshold;
}
//_____________________________________________________________________________
/**
 * Get the magnitude theshold below which no force is applied.
 *
 * @param aThreshold Magnitude threshold.  A negative value or 0 will result
 * in the force always being applied.
 */
double LinearSpring::
getThreshold() const
{
	return _threshold;
}

//-----------------------------------------------------------------------------
// SCALE FACTOR FUNCTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the function containing the scale factor as a function of time.
 *
 * @param aScaleFunction.
 */
void LinearSpring::
setScaleFunction(Function* aScaleFunction)
{
	_scaleFunction = aScaleFunction;
}
//_____________________________________________________________________________
/**
 * Get the function containing the scale factor as a function of time.
 *
 * @return aScaleFunction.
 */
Function* LinearSpring::
getScaleFunction() const
{
	return(_scaleFunction);
}

//-----------------------------------------------------------------------------
// SCALE FACTOR
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the scale factor that pre-multiplies the applied torque.
 *
 * @param aScaleFactor
 */
void LinearSpring::
setScaleFactor(double aScaleFactor)
{
	_scaleFactor = aScaleFactor;
}
//_____________________________________________________________________________
/**
 * Get the scale factor that pre-multiplies the applied torque.
 *
 * @return aScaleFactor.
 */
double LinearSpring::
getScaleFactor()
{
	return(_scaleFactor);
}


//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the local point on a body and the point's target position and
 * velocity in space that it should track.  A spring force is applied based
 * on the difference between the point's position and velocity and the
 * target position and velocity.  The point is specified in the local frame;
 * the target position and velocity are specified in the global frame.
 *
 * @param aQStore Storage containing the time history of generalized
 * coordinates for the model. Note that all generalized coordinates must
 * be specified and in radians and Euler parameters.
 * @param aUStore Stoarge containing the time history of generalized
 * speeds for the model.  Note that all generalized speeds must
 * be specified and in radians.
 * @param aPStore Storage containing the time history of the position of
 * the point in the global frame.
 */
void LinearSpring::
computePointAndTargetFunctions(
	const Storage &aQStore,const Storage &aUStore,VectorFunction &aPGlobal)
{
	computePointFunction(aQStore, aUStore, aPGlobal);
	computeTargetFunctions(aQStore, aUStore);
}

void LinearSpring::
computeTargetFunctions(const Storage &aQStoreForTarget,const Storage &aUStoreForTarget)
{
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	Array<double> t(0.0,1);
	Array<double> q(0.0,nq),u(0.0,nu);
	Vec3 pGlobal(0.0, 0., 0.);
	Vec3 pLocal(0.0, 0., 0.);
	Vec3 vGlobal(0.0, 0., 0.);
	Vec3 vLocal(0.0, 0., 0.);
	Storage pStore,pGlobalStore,vGlobalStore;

	// CREATE THE TARGET POSITION AND VELOCITY FUNCTIONS
	int size = aQStoreForTarget.getSize();
	for(int i=0;i<size;i++) {
		// Set the model state
		aQStoreForTarget.getTime(i,*(&t[0]));
		aQStoreForTarget.getData(i,nq,&q[0]);
		aUStoreForTarget.getData(i,nu,&u[0]);
		_model->getDynamicsEngine().setConfiguration(&q[0],&u[0]);

		// Get global position and velocity
		_pointFunction->evaluate(&t[0],&pLocal[0]);
		_model->getDynamicsEngine().getPosition(*_body,pLocal,pGlobal);
		_model->getDynamicsEngine().getVelocity(*_body,pLocal,vGlobal);

		// Append to storage
		pGlobalStore.append(t[0],3,&pGlobal[0]);
		vGlobalStore.append(t[0],3,&vGlobal[0]);
	}

	// CREATE TARGET FUNCTIONS
	// Position
	double *time=NULL;
	double *pg0=0,*pg1=0,*pg2=0;
	int padSize = size / 4;
	if(padSize>100) padSize = 100;
	pGlobalStore.pad(padSize);
	size = pGlobalStore.getTimeColumn(time);
	pGlobalStore.getDataColumn(0,pg0);
	pGlobalStore.getDataColumn(1,pg1);
	pGlobalStore.getDataColumn(2,pg2);
	VectorGCVSplineR1R3 *pGlobalFunc = new VectorGCVSplineR1R3(3,size,time,pg0,pg1,pg2);
	setTargetPosition(pGlobalFunc);
	delete[] time;
	delete[] pg0;
	delete[] pg1;
	delete[] pg2;
	// Velocity
	time=NULL;
	double *vg0=0,*vg1=0,*vg2=0;
	vGlobalStore.pad(padSize);
	size = vGlobalStore.getTimeColumn(time);
	vGlobalStore.getDataColumn(0,vg0);
	vGlobalStore.getDataColumn(1,vg1);
	vGlobalStore.getDataColumn(2,vg2);
	VectorGCVSplineR1R3 *vGlobalFunc = new VectorGCVSplineR1R3(3,size,time,vg0,vg1,vg2);
	setTargetVelocity(vGlobalFunc);
	delete[] time;
	delete[] vg0;
	delete[] vg1;
	delete[] vg2;
}


//=============================================================================
// CALLBACKS
//=============================================================================
//_____________________________________________________________________________
/**
 * Callback called right after actuation has been applied by the model.
 *
 * *
 * @param aT Real time.
 * @param aX Controls.
 * @param aY States.
 */
void LinearSpring::
applyActuation(double aT,double *aX,double *aY)
{
	//CALCULATE FORCE AND APPLY
	SimTK::Vec3 dx,dv;
	SimTK::Vec3 force;
	double scaleFactor;

	int i;
	Array<int> derivWRT(0,1);
	Array<double> origin(0.0,3);
	Array<double> vcomGlobal(0.0,3);
	Array<double> vpGlobal(0.0,3),vLocalGlobal(0.0,3);
	Array<double> treal(0.0,1);
	Vec3 pLocal(0.0);
	Array<double> vLocal(0.0,3);
	Array<double> pTarget(0.0,3);
	Array<double> vTarget(0.0,3);
	Vec3 pGlobal(0.0);
	Vec3 vGlobal(0.0);
	
	treal[0] = aT*_model->getTimeNormConstant();
	
	if(_model==NULL) {
		printf("LinearSpring.applyActuation: WARN- no model.\n");
		return;
	}
	if(!getOn()) return;

	if((aT>=getStartTime()) && (aT<getEndTime())){

		if(_pointFunction!=NULL) {
			_pointFunction->evaluate(&treal[0],&pLocal[0]);
			_pointFunction->evaluate(treal,vLocal,derivWRT);
			setPoint(pLocal);
		}

		if(_targetPosition!=NULL) {
			// position
			_targetPosition->evaluate(treal,pTarget);

			// velocity
			if(_targetVelocity!=NULL) {
				_targetVelocity->evaluate(treal,vTarget);
			} else {
				_targetPosition->evaluate(treal,vTarget,derivWRT);
			}
		} else {
			cout<<"\nLinearSpring.applyActuation:  WARN- no target has been set.\n";
		}

		// GET GLOBAL POSITION AND VELOCITY
		_model->getDynamicsEngine().getPosition(*_body,pLocal,pGlobal);
		_model->getDynamicsEngine().getVelocity(*_body,pLocal,vGlobal);
	
		if(_scaleFunction != NULL){
			scaleFactor = _scaleFunction->evaluate(0,aT*_model->getTimeNormConstant());
			setScaleFactor(scaleFactor);
		}
		dx=Vec3::getAs(&pTarget[0])-pGlobal;// Mtx::Subtract(1,3,&pTarget[0],&pGlobal[0],dx);
		dv=Vec3::getAs(&vTarget[0])-vGlobal;//Mtx::Subtract(1,3,&vTarget[0],&vGlobal[0],dv);

		for(i=0;i<3;i++){
			force[i] = _scaleFactor*(_k[i]*dx[i] + _b[i]*dv[i]);
		}
		setForce(force);
		//if(fabs(Mtx::Magnitude(3,force)) >= _threshold) {
		//	cout<<"linear spring ("<<aT<<"):\n";
		//	cout<<"force = "<<force[0]<<", "<<force[1]<<", "<<force[2]<<endl;
		//	cout<<"dx = "<<dx[0]<<", "<<dx[1]<<", "<<dx[2]<<endl;
		//	cout<<"dv = "<<dv[0]<<", "<<dv[1]<<", "<<dv[2]<<endl;
		//}
		if(fabs(Mtx::Magnitude(3,force)) >= _threshold) {
			//cout<<"applying force = "<<force[0]<<", "<<force[1]<<", "<<force[2]<<endl;
			_model->getDynamicsEngine().applyForce(*_body,pLocal,force);
			if(_recordAppliedLoads) _appliedForceStore->append(aT,_force);
		}

	}	
}





