// TorsionalSpring.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson and Saryn Goldberg
// 
//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/Mtx.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/VectorGCVSplineR1R3.h>
#include <OpenSim/Simulation/Simm/AbstractModel.h>
#include <OpenSim/Simulation/Simm/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Simm/AbstractBody.h>
#include <OpenSim/Tools/FunctionSet.h>
#include "TorsionalSpring.h"

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
TorsionalSpring::
TorsionalSpring(AbstractModel *aModel,AbstractBody *aBody) : 
	TorqueApplier(aModel,aBody)
{
	setNull();
	setType("TorsionalSpring");
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
	_scaleFunction = NULL;
	_scaleFactor = 1.0;
	_storeTorques = false;
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
setKValue(double aK[3])
{
	int i;
	for(i=0;i<3;i++)
		_k[i] = aK[i];
}
//_____________________________________________________________________________
/**
 * Get the spring constant, k.
 *
 * @return aK.
 */
void TorsionalSpring::
getKValue(double aK[3])
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
setBValue(double aB[3])
{
	int i;
	for(i=0;i<3;i++)
		_b[i] = aB[i];

}
//_____________________________________________________________________________
/**
 * Get the damping constant, b.
 *
 * @return aB.
 */
void TorsionalSpring::
getBValue(double aB[3])
{
	aB = _b;
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

//-----------------------------------------------------------------------------
// STORE TORQUES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not to store the applied torques.
 *
 * @param aTrueFalse If true, store the torques.  If false, do not store
 * the torques.
 */
void TorsionalSpring::
setStoreTorques(bool aTrueFalse)
{
	_storeTorques = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Set whether or not to store the applied torques.
 *
 * @return Whether or not the applied torques are being stored.
 */
bool TorsionalSpring::
getStoreTorques()
{
	return(_storeTorques);
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
void TorsionalSpring::
computeTargetFunctions(Storage *aQStore,Storage *aUStore)
{
	int i;
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	double t;
	Array<double> q(0.0,nq),u(0.0,nu);
	double dirCos[9],ang[3],angVel[3];
	Storage angStore,angVelStore;

	// CREATE THE TARGET POSITION AND VELOCITY FUNCTIONS
	int size = aQStore->getSize();
	for(i=0;i<size;i++) {
		// Set the model state
		aQStore->getTime(i,t);
		aQStore->getData(i,nq,&q[0]);
		aUStore->getData(i,nu,&u[0]);
		_model->getDynamicsEngine().setConfiguration(&q[0],&u[0]);

		// Get global position and velocity
		_model->getDynamicsEngine().getDirectionCosines(*_body,dirCos);
		_model->getDynamicsEngine().convertDirectionCosinesToAngles(dirCos,&ang[0],&ang[1],&ang[2]);
		_model->getDynamicsEngine().getAngularVelocity(*_body,angVel);

		// Append to storage
		angStore.append(t,3,ang);
		angVelStore.append(t,3,angVel);
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
	double *vg0=0,*vg1=0,*vg2=0;
	angVelStore.pad(padSize);
	size = angVelStore.getTimeColumn(time);
	angVelStore.getDataColumn(0,vg0);
	angVelStore.getDataColumn(1,vg1);
	angVelStore.getDataColumn(2,vg2);
	VectorGCVSplineR1R3 *angVelFunc = new VectorGCVSplineR1R3(3,size,time,vg0,vg1,vg2);
	setTargetVelocity(angVelFunc);
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
void TorsionalSpring::
applyActuation(double aT,double *aX,double *aY)
{
	int i;
	double torque[3] = {0,0,0};

	if(_model==NULL) {
		printf("TorsionalSpring.applyActuation: WARN- no model.\n");
		return;
	}
	if(!getOn()) return;

	if((aT>=getStartTime()) && (aT<getEndTime())){

		// Calculate Euler angles to rotate from current orientation to nominal orientation,
		//  expressed in current body reference frame
		double time = aT*_model->getTimeNormConstant(); 
		double nomDirCos[9];
		double curDirCos[9];
		double difDirCos[9];
		double difAng[3];
		double eulerAngle[3];
		_targetPosition->evaluate(&time,eulerAngle);

		_model->getDynamicsEngine().convertAnglesToDirectionCosines(eulerAngle[0],eulerAngle[1],
			eulerAngle[2],nomDirCos);
		_model->getDynamicsEngine().getDirectionCosines(*_body,curDirCos);
		Mtx::Transpose(3,3,curDirCos,curDirCos);
		Mtx::Multiply(3,3,3,curDirCos,nomDirCos,difDirCos);
		_model->getDynamicsEngine().convertDirectionCosinesToAngles(difDirCos,&difAng[0],&difAng[1],&difAng[2]);

		// Calculate difference between current angular velocity and nominal ang velocity,
		//  expressed in body-fixed 1-2-3 rotational velocities
		double nomAngVel[3];
		double curAngVel[3];
		double difAngVel[3];
		double difQDot[3];
		double eulerTransform[9];
		_targetVelocity->evaluate(&time,nomAngVel); //NEEDS TO BE IN GLOBAL COORDS
		_model->getDynamicsEngine().getAngularVelocity(*_body,curAngVel);  //EXPRESSED IN GLOBAL COORDS
		Mtx::Subtract(3,1,curAngVel,nomAngVel,difAngVel);
//		Mtx::Subtract(3,1,nomAngVel,curAngVel,difAngVel);
		_model->getDynamicsEngine().transform(_model->getDynamicsEngine().getGroundBody(),difAngVel,*_body,difAngVel);
		_model->getDynamicsEngine().formEulerTransform(*_body, eulerTransform);
		Mtx::Multiply(3,3,1,eulerTransform,difAngVel,difQDot);
		
		// Calculate torque to apply to body, expressed in global reference frame
		double scaleFactor;
		if(_scaleFunction != NULL){
			scaleFactor = _scaleFunction->evaluate(0,aT*_model->getTimeNormConstant());
			setScaleFactor(scaleFactor);
		}
		for(i=0;i<3;i++)
			torque[i] = _scaleFactor*(-_k[i]*difAng[i] - _b[i]*difQDot[i]);
		Mtx::Multiply(3,3,1,eulerTransform,torque,torque);
		_model->getDynamicsEngine().transform(*_body,torque,_model->getDynamicsEngine().getGroundBody(),torque);

		// Apply torque to body
		setTorque(torque);
		_model->getDynamicsEngine().applyTorque(*_body,_torque);
		if(_storeTorques) _appliedTorqueStore->append(aT,3,_torque);

	}	
}
