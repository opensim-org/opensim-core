// Force.cpp
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include "Model.h"
#include "AbstractDynamicsEngine.h"
#include "BodySet.h"
#include "Force.h"




using namespace OpenSim;
using namespace std;


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
Force::~Force()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Force::Force(const string &aBodyAName,const string &aBodyBName) :
	AbstractActuator(),
	_bodyAName(_propBodyAName.getValueStr()),
	_pA(_propPointA.getValueDblArray()),
	_uA(_propUnitVectorA.getValueDblArray()),
	_bodyBName(_propBodyBName.getValueStr()),
	_pB(_propPointB.getValueDblArray()),
	_optimalForce(_propOptimalForce.getValueDbl()),
	_bA(NULL),
	_bB(NULL)

{
	// NULL
	setNull();

	// MEMBER DATA
	_bodyAName = aBodyAName;
	_bodyBName = aBodyBName;

	if(_model) {
		_bA = _model->getDynamicsEngine().getBodySet()->get(_bodyAName);
		_bB = _model->getDynamicsEngine().getBodySet()->get(_bodyBName);
	}
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aForce Force to be copied.
 */
Force::Force(const Force &aForce) :
	AbstractActuator(aForce),
	_bodyAName(_propBodyAName.getValueStr()),
	_pA(_propPointA.getValueDblArray()),
	_uA(_propUnitVectorA.getValueDblArray()),
	_bodyBName(_propBodyBName.getValueStr()),
	_pB(_propPointB.getValueDblArray()),
	_optimalForce(_propOptimalForce.getValueDbl()),
	_bA(NULL),
	_bB(NULL)
{
	setNull();
	*this = aForce;
}
//_____________________________________________________________________________
/**
 * Copy this actuator.  The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this actuator.
 */
Object* Force::
copy() const
{
	Force *act = new Force(*this);
	return(act);
}



//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void Force::
setNull()
{
	setupProperties();

	// TYPE
	setType("Force");

	// APPLIES FORCE
	setAppliesForce(true);

	setNumControls(1); setNumStates(0); setNumPseudoStates(0);
	bindControl(0, _excitation, "excitation");

	// BODY A
	_pA[0] = _pA[1] = _pA[2] = 0.0;
	_uA[0] = _uA[1] = _uA[2] = 0.0;  _uA[0] = 1.0;

	// BODY B
	_pB[0] = _pB[1] = _pB[2] = 0.0;
	_uB[0] = _uB[1] = _uB[2] = 0.0;  _uB[0] = 1.0;

	//POINT FUNCTIONS
	_pAFunction = NULL;
	_pBFunction = NULL;

	//SCALE FACTOR
	_scaleFunction = NULL;
	_scaleFactor = 1.0;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Force::
setupProperties()
{
	double origin[3] = { 0.0, 0.0, 0.0 };
	double x_axis[3] = { 1.0, 0.0, 0.0 };

	_propBodyAName.setName("body_A");
	_propertySet.append( &_propBodyAName );

	_propPointA.setName("point_A");
	_propPointA.setValue(3,origin);
	_propertySet.append( &_propPointA );

	_propUnitVectorA.setName("direction_A");
	_propUnitVectorA.setValue(3,x_axis);
	_propertySet.append( &_propUnitVectorA );

	_propBodyBName.setName("body_B");
	_propertySet.append( &_propBodyBName );

	_propPointB.setName("point_B");
	_propPointB.setValue(3,origin);
	_propertySet.append( &_propPointB );

	_propOptimalForce.setName("optimal_force");
	_propOptimalForce.setValue(1.0);
	_propertySet.append( &_propOptimalForce );

}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
Force& Force::
operator=(const Force &aForce)
{
	// BASE CLASS
	AbstractActuator::operator=(aForce);

	// BODY A
	setBodyA(aForce.getBodyA());

	// POINT A
	aForce.getPointA(&_pA[0]);

	// DIRCTION A
	aForce.getForceDirectionA(&_uA[0]);

	// BODY B
	setBodyB(aForce.getBodyB());

	// POINT B
	aForce.getPointB(&_pB[0]);

	// DIRECTION B
	aForce.getForceDirectionB(_uB);

	// OPTIMAL FORCE
	setOptimalForce(aForce.getOptimalForce());

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// BODY A
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the first body to which the force is applied.
 *
 * @param aBody Pointer to body.
 */
void Force::
setBodyA(AbstractBody* aBody)
{
	_bA = aBody;
	if (aBody)
		_bodyAName = aBody->getName();
}
//_____________________________________________________________________________
/**
 * Get the first body to which the force is applied.
 *
 * @return Pointer to body.
 */
AbstractBody* Force::
getBodyA() const
{
	return(_bA);
}

//-----------------------------------------------------------------------------
// POINT A
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the value of the point on BodyA at which the force is applied.
 *
 * @param aPoint Point x, y, and z values.
 */
void Force::
setPointA(const double aPoint[3])
{
	_pA[0] = aPoint[0];
	_pA[1] = aPoint[1];
	_pA[2] = aPoint[2];
}
//_____________________________________________________________________________
/**
 * Get the value of the point on BodyA at which the force is applied.
 *
 * @param rPoint Point x, y, and z values.
 */
void Force::
getPointA(double rPoint[3]) const
{
	rPoint[0] = _pA[0];
	rPoint[1] = _pA[1];
	rPoint[2] = _pA[2];
}
//_____________________________________________________________________________
/**
 * Set the vector function to read in values for Point A.
 *
 * @param aVectorFunction Vector function containing  x, y, and z values of
 * Point A (expressed in the body local frame) as a function fo time.
 */
void Force::
setPointAFunction(VectorFunction* aVectorFunction)
{
	_pAFunction = aVectorFunction;
}
//_____________________________________________________________________________
/**
 * Get the vector function to read in values for Point A.
 *
 * @param rVectorFunction Vector function containing  x, y, and z values of
 * Point A (expressed in the body local frame) as a function fo time.
 */
const VectorFunction* Force::
getPointAFunction() const
{
	return(_pAFunction);
}
//-----------------------------------------------------------------------------
// FORCE DIRECTION A
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the direction in which a positive actuator force is applied to BodyA.
 *
 * Newton's second law states that the force between
 * two bodies is equal and opposite in direction, so when the direction
 * of force on one body is known, the direction on the other body
 * can be calculated.  Therefore, the direction of force on BodyB is always
 * computed.
 *
 * @param aDirection Direction.  The direction is normalized.  If the
 * magnitude of aDirection is less than rdMath::ZERO, the force direction
 * is set to the zero vector, the effect of which is to prevent any force
 * from being applied.
 */
void Force::
setForceDirectionA(const double aDirection[3])
{
	double mag = Mtx::Normalize(3,aDirection,&_uA[0]);
	if(mag==rdMath::ZERO) {
		printf("Force.setForceDirection: WARN- direction has a magnitude ");
		printf("of less than %lf.\n",rdMath::ZERO);
		printf("\tSetting the direction to (0,0,0).\n");
		_uA[0] = _uA[1] = _uA[2] = 0.0;
	}
}
//_____________________________________________________________________________
/**
 * Set the direction in which a positive actuator force is applied to BodyA.
 *
 * @param rPoint Point x, y, and z values.
 */
void Force::
getForceDirectionA(double rDirection[3]) const
{
	rDirection[0] = _uA[0];
	rDirection[1] = _uA[1];
	rDirection[2] = _uA[2];
}

//-----------------------------------------------------------------------------
// BODY B
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the second body to which the force is applied.
 *
 * @param aBody Pointer to body.
 */
void Force::
setBodyB(AbstractBody* aBody)
{
	_bB = aBody;
	if (aBody)
		_bodyBName = aBody->getName();
}
//_____________________________________________________________________________
/**
 * Get the second body to which the force is applied.
 *
 * @return Pointer to body.
 */
AbstractBody* Force::
getBodyB() const
{
	return(_bB);
}

//-----------------------------------------------------------------------------
// POINT B
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the value of the point on BodyB at which the force is applied.
 *
 * @param aPoint Point x, y, and z values.
 */
void Force::
setPointB(const double aPoint[3])
{
	_pB[0] = aPoint[0];
	_pB[1] = aPoint[1];
	_pB[2] = aPoint[2];
}
//_____________________________________________________________________________
/**
 * Get the value of the point on BodyB at which the force is applied.
 *
 * @param rPoint Point x, y, and z values.
 */
void Force::
getPointB(double rPoint[3]) const
{
	rPoint[0] = _pB[0];
	rPoint[1] = _pB[1];
	rPoint[2] = _pB[2];
}
//_____________________________________________________________________________
/**
 * Set the vector function to read in values for Point B.
 *
 * @param aVectorFunction Vector function containing  x, y, and z values of
 * Point B (expressed in the body local frame) as a function fo time.
 */
void Force::
setPointBFunction(VectorFunction* aVectorFunction)
{
	_pBFunction = aVectorFunction;
}
//_____________________________________________________________________________
/**
 * Get the vector function to read in values for Point B.
 *
 * @param rVectorFunction Vector function containing  x, y, and z values of
 * Point B (expressed in the body local frame) as a function fo time.
 */
const VectorFunction* Force::
getPointBFunction() const
{
	return(_pBFunction);
}

//-----------------------------------------------------------------------------
// FORCE DIRECTION B
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the direction in which a positive actuator force is applied to BodyB.
 *
 * Note that the direction of force application on BodyB cannot be set; it is
 * always computed based on the force direction set for BodyA.
 *
 * @param rPoint Point x, y, and z values.
 */
void Force::
getForceDirectionB(double rDirection[3]) const
{
	rDirection[0] = _uB[0];
	rDirection[1] = _uB[1];
	rDirection[2] = _uB[2];
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
void Force::
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
Function* Force::
getScaleFunction() const
{
	return(_scaleFunction);
}

//-----------------------------------------------------------------------------
// SCALE FACTOR
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the scale factor that pre-multiplies the applied force.
 *
 * @param aScaleFactor
 */
void Force::
setScaleFactor(double aScaleFactor)
{
	_scaleFactor = aScaleFactor;
}
//_____________________________________________________________________________
/**
 * Get the scale factor that pre-multiplies the applied force.
 *
 * @return aScaleFactor.
 */
double Force::
getScaleFactor()
{
	return(_scaleFactor);
}

//-----------------------------------------------------------------------------
// OPTIMAL FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the optimal force of the force.
 *
 * @param aOptimalForce Optimal force.
 */
void Force::
setOptimalForce(double aOptimalForce)
{
	_optimalForce = aOptimalForce;
}
//_____________________________________________________________________________
/**
 * Get the optimal force of the force.
 *
 * @return Optimal force.
 */
double Force::
getOptimalForce() const
{
	return(_optimalForce);
}
//_____________________________________________________________________________
/**
 * Get the stress of the force.
 *
 * @return Stress.
 */
double Force::
getStress() const
{
	return fabs(_force/_optimalForce); 
}


//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the force force to the
 * model.
 */
void Force::
computeActuation()
{
	// DIRECTION
	computeForceDirectionForBodyB();

	// SPEED
	computeSpeed();

	// FORCE
	double force = _excitation * getOptimalForce();
	setForce(force);
}
//_____________________________________________________________________________
/**
 * Compute the force direction for BodyB based on the force direction set
 * for BodyA.
 */
void Force::
computeForceDirectionForBodyB()
{
	_model->getDynamicsEngine().transform(*_bA,&_uA[0],*_bB,_uB);
	Mtx::Multiply(1,3,_uB,-1.0,_uB);
}
//_____________________________________________________________________________
/**
 * Compute the speed of the actuator.
 */
void Force::
computeSpeed()
{
	double vA[3],vB[3],v[3];
	_model->getDynamicsEngine().getVelocity(*_bA,&_pA[0],vA);
	_model->getDynamicsEngine().getVelocity(*_bA,&_pB[0],vB);
	Mtx::Subtract(1,3,vB,vA,v);
	_model->getDynamicsEngine().transform(_model->getDynamicsEngine().getGroundBody(),v,*_bA,v);
	_speed = -Mtx::DotProduct(3,&_uA[0],v);
}
//_____________________________________________________________________________
/**
 * Update the position of PointA.  That is, if it has been specified by a read
 * in function, set the current position to be that in the function.
 */
void Force::
updatePointA()
{
	double time = _model->getTime()*_model->getTimeNormConstant();
	if(_pAFunction !=NULL) _pAFunction->evaluate(&time,&_pA[0]);
}
//_____________________________________________________________________________
/**
 * Update the position of PointB.  That is, if it has been specified by a read
 * in function, set the current position to be that in the function.
 */
void Force::
updatePointB()
{
	double time = _model->getTime()*_model->getTimeNormConstant();
	if(_pBFunction !=NULL) _pBFunction->evaluate(&time,&_pB[0]);
}

//=============================================================================
// APPLICATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Apply the actuator force to BodyA and BodyB.
 */
void Force::
apply()
{
	// SET SCALE FACTOR
	double scaleFactor;
	if(_scaleFunction!=NULL) {
		scaleFactor = _scaleFunction->evaluate(0,_model->getTime()*_model->getTimeNormConstant());
		setScaleFactor(scaleFactor);
	}
	
	// FORCE ON BODY A
	if(_bA!=&_model->getDynamicsEngine().getGroundBody()) {
		double fA[3];
		Mtx::Multiply(1,3,&_uA[0],_force,fA);
		Mtx::Multiply(1,3,fA,_scaleFactor,fA);
		_model->getDynamicsEngine().applyForceBodyLocal(*_bA,&_pA[0],fA);
	}

	// FORCE ON BODY B
	if(_bB!=&_model->getDynamicsEngine().getGroundBody()) {
		double fB[3];
		Mtx::Multiply(1,3,_uB,_force,fB);
		Mtx::Multiply(1,3,fB,_scaleFactor,fB);
		_model->getDynamicsEngine().applyForceBodyLocal(*_bB,&_pB[0],fB);
	}
}


//=============================================================================
// CHECK
//=============================================================================
//_____________________________________________________________________________
/**
 * Check that this force actuator has a valid set of states.
 */
bool Force::
check() const
{
	AbstractActuator::check();

	// BODY A
	if(_model!=NULL) {
		if(getBodyA()==NULL) {
			printf("Force.check: ERROR- %s has invalid body for BodyA (%s).\n",
				getName().c_str(),_bodyAName.c_str());
			return(false);
		}
	}

	// BODY B
	if(_model!=NULL) {
		if(getBodyB()==NULL) {
			printf("Force.check: ERROR- %s has invalid body for BodyB (%s).\n",
				getName().c_str(),_bodyBName.c_str());
			return(false);
		}
	}

	// SAME BODY
	if(getBodyA()==getBodyB()) {
		printf("Force.check: WARN- %s has the same body for BodyA and BodyB\n",
			getName().c_str());
	}

	return(true);
}
//_____________________________________________________________________________
/**
 * setup sets the actual Body references _ba, _bB
 */
void Force::
setup(Model* aModel)
{
	AbstractActuator::setup(aModel);
	if (aModel)
	{
		_bA = aModel->getDynamicsEngine().getBodySet()->get(_bodyAName);
		_bB = aModel->getDynamicsEngine().getBodySet()->get(_bodyBName);
	}
}

//=============================================================================
// UTILITY
//=============================================================================
//-----------------------------------------------------------------------------
// LINE OF ACTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the vector directed from the point of force application on 
 * BodyA to the point of force application on BodyB expressed in the
 * local frame of BodyA.
 *
 * @param rLineOfAction Line of action expressed in the local frame of BodyA.
 * @todo Check that the line of action is expressed in the proper frame.
 */
void Force::
computeLineOfAction(double rLineOfAction[3]) const
{
	if(rLineOfAction==NULL) return;
	if(_model==NULL) {
		printf("Force.computeLineOfAction: ERROR- no model.\n");
		return;
	}

	// GET INERTIAL POSITONS
	double pB[3],pA[3];
	_model->getDynamicsEngine().getPosition(*_bB,&_pB[0],pB);
	_model->getDynamicsEngine().getPosition(*_bA,&_pA[0],pA);

	// SUBTRACT
	Mtx::Subtract(1,3,pB,pA,rLineOfAction);

	// TRANSFORM
	_model->getDynamicsEngine().transform(_model->getDynamicsEngine().getGroundBody(),rLineOfAction,*_bA,rLineOfAction);
}

