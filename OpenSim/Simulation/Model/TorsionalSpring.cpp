// TorsionalSpring.cpp
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
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Common/VectorGCVSplineR1R3.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Common/FunctionSet.h>
#include "TorsionalSpring.h"


using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________


using namespace OpenSim;
/**
 * Destructor.
 */
TorsionalSpring::~TorsionalSpring()
{
}
//_____________________________________________________________________________
/**
 * Construct a derivative callback instance for applying external torques
 * during an integration.
 *
 * @param aModel Model for which external torques are to be applied.
 * @param aBody Body to which external torques are to be applied.
 */
TorsionalSpring::TorsionalSpring(const Body &aBody, double startTime, double endTime) : CustomForce(),
	_body(aBody), _startTime(startTime), _endTime(endTime)
{
	setNull();
	setType("TorsionalSpring");
	_model = const_cast<OpenSim::Model *>(&aBody.getModel());

}

//_____________________________________________________________________________
/**
 * Set member variables to approprate NULL values.
 */
void TorsionalSpring::
setNull()
{
	_targetPosition = NULL;
	_targetVelocity = NULL;
	_k[0] = _k[1] = _k[2] = 0.0;
	_b[0] = _b[1] = _b[2] = 0.0;
	_threshold = 0.0;
	_scaleFunction = NULL;
	_scaleFactor = 1.0;
}

//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// POSITION VECTOR FUNCTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the vetor function containing the nominal angular postion of the body.
 *
 * @param aPosFunction Vector function containing the nominal angular position of
 * the body in the intertial reference frame.
 */
void TorsionalSpring::
setTargetPosition(VectorFunction* aPosFunction)
{
	_targetPosition = aPosFunction;
}
//_____________________________________________________________________________
/**
 * Get the vector function containing the nominal angular position of the body.
 *
 * @return aPosFunction.
 */
VectorFunction* TorsionalSpring::
getTargetPosition() const
{
	return(_targetPosition);
}

//-----------------------------------------------------------------------------
// VELOCITY VECTOR FUNCTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the vector function containing the nominal angular velocity of the body.
 *
 * @param aPosFunction Vector function containing the nominal angular velocity of
 * the body in the intertial reference frame.
 */
void TorsionalSpring::
setTargetVelocity(VectorFunction* aVelFunction)
{
	_targetVelocity = aVelFunction;
}
//_____________________________________________________________________________
/**
 * Get the vector function containing the nominal angular velocity of the body.
 *
 * @return aVelFunction.
 */
VectorFunction* TorsionalSpring::
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
void TorsionalSpring::
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
void TorsionalSpring::
getKValue(SimTK::Vec3& aK)
{
	aK = _k;
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
void TorsionalSpring::
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
void TorsionalSpring::
getBValue(SimTK::Vec3& aB)
{
	aB = _b;
}

//-----------------------------------------------------------------------------
// THRESHOLD
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the magnitude theshold below which no torque is applied.
 *
 * @param aThreshold Magnitude threshold.  A negative value or 0 will result
 * in the torque always being applied.
 */
void TorsionalSpring::
setThreshold(double aThreshold)
{
	_threshold = aThreshold;
}
//_____________________________________________________________________________
/**
 * Get the magnitude theshold below which no torque is applied.
 *
 * @param aThreshold Magnitude threshold.  A negative value or 0 will result
 * in the torque always being applied.
 */
double TorsionalSpring::
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
void TorsionalSpring::
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
Function* TorsionalSpring::
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
void TorsionalSpring::
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
double TorsionalSpring::
getScaleFactor()
{
	return(_scaleFactor);
}


//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the target orientation and angular velocity of the body.
 * A spring force is applied based on the difference between the body's
 * actual orientation and angular velocities and corresponding target
 * values.  The target orientation and angular velocity are computed
 * with respect to the global frame.
 *
 * @param aQStore Storage containing the time history of generalized
 * coordinates for the model. Note that all generalized coordinates must
 * be specified and in radians and Euler parameters.
 * @param aUStore Stoarge containing the time history of generalized
 * speeds for the model.  Note that all generalized speeds must
 * be specified and in radians.
 */
void TorsionalSpring::computeTargetFunctions(SimTK::State &s, const Storage &aQStore,const Storage &aUStore)
{
	int i;
	int nq = getModel().getNumCoordinates();
	int nu = _model->getNumSpeeds();
	double t;
	Array<double> q(0.0,nq),u(0.0,nu);
	double dirCos[9];
	SimTK::Vec3 ang,angVel;
	Storage angStore,angVelStore;

	// CREATE THE TARGET POSITION AND VELOCITY FUNCTIONS
	int size = aQStore.getSize();
	for(i=0;i<size;i++) {
		// Set the model state
		aQStore.getTime(i,t);
		aQStore.getData(i,nq,&q[0]);
		aUStore.getData(i,nu,&u[0]);

		s.setQ(SimTK::Vector(nq,&q[0]));
		s.setU(SimTK::Vector(nu,&u[0]));

		_model->getSystem().realize(s, SimTK::Stage::Velocity );

		// Get global position and velocity
		_model->getSimbodyEngine().getDirectionCosines(s, _body, dirCos);
		_model->getSimbodyEngine().convertDirectionCosinesToAngles(dirCos,&ang[0],&ang[1],&ang[2]);
		_model->getSimbodyEngine().getAngularVelocity(s, _body, angVel);

		// Append to storage
		angStore.append(t,ang);
		angVelStore.append(t,angVel);
	}

	// CREATE TARGET FUNCTIONS
	// Position
	size = angStore.getSize();
	int padSize = size / 4;
	if(padSize>100) padSize = 100;
	double *time=NULL;
	double *pg0=0,*pg1=0,*pg2=0;
	angStore.pad(padSize);
	size = angStore.getTimeColumn(time);
	angStore.getDataColumn(0,pg0);
	angStore.getDataColumn(1,pg1);
	angStore.getDataColumn(2,pg2);
	VectorGCVSplineR1R3 *angFunc = new VectorGCVSplineR1R3(3,size,time,pg0,pg1,pg2);
	setTargetPosition(angFunc);
	// Velocity
	if(time!=NULL) { delete[] time; time=NULL; }
	delete[] pg0;
	delete[] pg1;
	delete[] pg2;
	double *vg0=0,*vg1=0,*vg2=0;
	angVelStore.pad(padSize);
	size = angVelStore.getTimeColumn(time);
	angVelStore.getDataColumn(0,vg0);
	angVelStore.getDataColumn(1,vg1);
	angVelStore.getDataColumn(2,vg2);
	VectorGCVSplineR1R3 *angVelFunc = new VectorGCVSplineR1R3(3,size,time,vg0,vg1,vg2);
	setTargetVelocity(angVelFunc);
	delete[] vg0;
	delete[] vg1;
	delete[] vg2;
}

//=============================================================================
// CALLBACKS
//=============================================================================
//_____________________________________________________________________________
/**
 * Implement the Force
 *
 * @param s States.
 */
void TorsionalSpring::computeForce(const SimTK::State& s, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const
{
	int i;
	SimTK::Vec3 torque(0,0,0);

	if(_model==NULL) {
		printf("TorsionalSpring.applyActuation: WARN- no model.\n");
		return;
	}

	double time = s.getTime();
	if( !isDisabled(s) && (time >= _startTime) && (time < _endTime)){
		// Calculate Euler angles to rotate from current orientation to nominal orientation,
		// expressed in current body reference frame 
		double nomDirCos[9];
		double curDirCos[9];
		double difDirCos[9];
		double difAng[3];
		double eulerAngle[3];
		_targetPosition->calcValue(&time, eulerAngle, 3);

		_model->getSimbodyEngine().convertAnglesToDirectionCosines(eulerAngle[0], eulerAngle[1], eulerAngle[2], nomDirCos);
		_model->getSimbodyEngine().getDirectionCosines(s, _body, curDirCos);
		Mtx::Transpose(3,3,curDirCos,curDirCos);
		Mtx::Multiply(3,3,3,curDirCos,nomDirCos,difDirCos);
		_model->getSimbodyEngine().convertDirectionCosinesToAngles(difDirCos,&difAng[0],&difAng[1],&difAng[2]);

		// Calculate difference between current angular velocity and nominal ang velocity,
		//  expressed in body-fixed 1-2-3 rotational velocities
		SimTK::Vec3 nomAngVel;
		SimTK::Vec3 curAngVel;
		SimTK::Vec3 difAngVel;
		double difQDot[3];
		double eulerTransform[9];
		_targetVelocity->calcValue(&time, &nomAngVel[0], 3); //NEEDS TO BE IN GLOBAL COORDS
		_model->getSimbodyEngine().getAngularVelocity(s, _body, curAngVel);  //EXPRESSED IN GLOBAL COORDS
		difAngVel=curAngVel-nomAngVel;
//		Mtx::Subtract(3,1,nomAngVel,curAngVel,difAngVel);
		_model->getSimbodyEngine().transform(s,_model->getGroundBody(), difAngVel,_body, difAngVel);
		_model->getSimbodyEngine().formEulerTransform(s, _body, eulerTransform);
		Mtx::Multiply(3,3,1,eulerTransform,&difAngVel[0],difQDot);
		
		// Calculate torque to apply to body, expressed in global reference frame
		double scaleFactor;
		if(_scaleFunction != NULL){
			scaleFactor = _scaleFunction->calcValue(SimTK::Vector(1, s.getTime()));
		}
		for(i=0;i<3;i++)
			torque[i] = scaleFactor*(-_k[i]*difAng[i] - _b[i]*difQDot[i]);
		Mtx::Multiply(3,3,1,eulerTransform,&torque[0],&torque[0]);
		_model->getSimbodyEngine().transform(s, _body,torque,_model->getGroundBody(),torque);

		// Apply torque to body
		//if(fabs(Mtx::Magnitude(3,torque)) >= _threshold) {
		//	cout<<"torsional spring ("<<aT<<"):\n";
		//	cout<<"torque = "<<torque[0]<<", "<<torque[1]<<", "<<torque[2]<<endl;
		//	cout<<"dx = "<<difAng[0]<<", "<<difAng[1]<<", "<<difAng[2]<<endl;
		//	cout<<"dv = "<<difQDot[0]<<", "<<difQDot[1]<<", "<<difQDot[2]<<endl;
		//}
//	 cout<<"torsional spring t=" << s.getTime() << " torque=" << torque << endl;
		if(torque.norm() >= _threshold) {
			//cout<<"applying torque = "<<torque[0]<<", "<<torque[1]<<", "<<torque[2]<<endl;
			applyTorque(s, _body, torque, bodyForces);
			//if(_recordAppliedLoads) _appliedTorqueStore->append(aT,3,&_torque[0]);
		}

	} else {
 //       cout<<"torsional spring t=" << s.getTime() << " OFF"  << endl;
    }	
}
