// Torque.cpp
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
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/Mtx.h>
#include <OpenSim/Tools/PropertyInt.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include <OpenSim/Tools/PropertyDblArray.h>
#include "Torque.h"




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
 *
 * @todo There appears to be a memory leak when the control names are
 * allocated (_xNames) in Torque.
 */
Torque::~Torque()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Torque::Torque(int aBodyA,int aBodyB,int aNX,int aNY,int aNYP) :
	Actuator(aNX,aNY,aNYP),
	_bA(_propBodyA.getValueInt()),
	_uA(_propUnitVectorA.getValueDblArray()),
	_bB(_propBodyB.getValueInt())
{
	// NULL
	setNull();

	// MEMBER DATA
	_bA = aBodyA;
	_bB = aBodyB;
}
//_____________________________________________________________________________
/**
 * Construct an actuator from file.
 *
 * @param aFileName Name of the file.
 */
Torque::Torque(DOMElement *aElement,int aNX,int aNY,int aNYP) :
	Actuator(aNX,aNY,aNYP,aElement),
	_bA(_propBodyA.getValueInt()),
	_uA(_propUnitVectorA.getValueDblArray()),
	_bB(_propBodyB.getValueInt())
{
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aForce Force to be copied.
 */
Torque::Torque(const Torque &aForce) :
	Actuator(aForce),
	_bA(_propBodyA.getValueInt()),
	_uA(_propUnitVectorA.getValueDblArray()),
	_bB(_propBodyB.getValueInt())
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
Object* Torque::
copy() const
{
	Torque *act = new Torque(*this);
	return(act);
}
//_____________________________________________________________________________
/**
 * Copy this actuator and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * Torque::Torque(DOMElement*,int,int) in order to establish the
 * relationship of the Torque object with the XML node.  Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this Torque object.  Finally, the data members of the copy are
 * updated using Torque::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this actuator.
 */
Object* Torque::
copy(DOMElement *aElement) const
{
	Torque *act = new Torque(aElement,getNX(),getNY());
	*act = *this;
	act->updateFromXMLNode();

	return(act);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void Torque::
setNull()
{
	setupProperties();

	// TYPE
	setType("Torque");

	// APPLIES FORCE
	setAppliesForce(false);

	// BODY A
	_bA = -1;
	_uA[0] = _uA[1] = _uA[2] = 0.0;  _uA[0] = 1.0;

	// BODY B
	_bB = -1;
	_uB[0] = _uB[1] = _uB[2] = 0.0;  _uB[0] = 1.0;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Torque::
setupProperties()
{
	double origin[3] = { 0.0, 0.0, 0.0 };
	double x_axis[3] = { 1.0, 0.0, 0.0 };

	_propBodyA.setName("body_A");
	_propBodyA.setValue(-1);
	_propertySet.append( &_propBodyA );

	_propUnitVectorA.setName("direction_A");
	_propUnitVectorA.setValue(3,x_axis);
	_propertySet.append( &_propUnitVectorA );

	_propBodyB.setName("body_B");
	_propBodyB.setValue(-1);
	_propertySet.append( &_propBodyB );
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
Torque& Torque::
operator=(const Torque &aForce)
{
	// BASE CLASS
	Actuator::operator=(aForce);

	// BODY A
	setBodyA(aForce.getBodyA());

	// DIRCTION A
	aForce.getDirectionA(&_uA[0]);

	// BODY B
	setBodyB(aForce.getBodyB());
	
	// DIRECTION B
	aForce.getDirectionB(_uB);

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
 * Set the id of the first body to which the force is applied.
 *
 * @param aID Body ID.
 */
void Torque::
setBodyA(int aID)
{
	_bA = aID;
}
//_____________________________________________________________________________
/**
 * Get the id of the first body to which the force is applied.
 *
 * @param aID Body ID.
 */
int Torque::
getBodyA() const
{
	return(_bA);
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
void Torque::
setDirectionA(const double aDirection[3])
{
	double mag = Mtx::Normalize(3,aDirection,&_uA[0]);
	if(mag==rdMath::ZERO) {
		printf("Torque.setForceDirection: WARN- direction has a magnitude ");
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
void Torque::
getDirectionA(double rDirection[3]) const
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
 * Set the id of the second body to which the force is applied.
 *
 * @param aID Body ID.
 */
void Torque::
setBodyB(int aID)
{
	_bB = aID;
}
//_____________________________________________________________________________
/**
 * Get the id of the second body to which the force is applied.
 *
 * @param aID Body ID.
 */
int Torque::
getBodyB() const
{
	return(_bB);
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
void Torque::
getDirectionB(double rDirection[3]) const
{
	rDirection[0] = _uB[0];
	rDirection[1] = _uB[1];
	rDirection[2] = _uB[2];
}


//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the actuator force to the
 * model.
 */
void Torque::
computeActuation()
{
	// DIRECTION
	computeDirectionForBodyB();

	// SPEED
	computeSpeed();

	// FORCE
	double torque = getControl(0) * getOptimalForce();
	setForce(torque);
}
//_____________________________________________________________________________
/**
 * Compute the torque direction for BodyB based on the force direction set
 * for BodyA.
 */
void Torque::
computeDirectionForBodyB()
{
	_model->transform(_bA,&_uA[0],_bB,_uB);
	Mtx::Multiply(1,3,_uB,-1.0,_uB);
}
//_____________________________________________________________________________
/**
 * Compute the speed of the actuator.
 * The speed is the angular velocity of the body in the global frame
 * expressed in the body-local frame.
 */
void Torque::
computeSpeed()
{
	double angVel[3];
	_model->getAngularVelocityBodyLocal(_bB,angVel);
	_speed = -Mtx::DotProduct(3,&_uA[0],angVel);
}


//=============================================================================
// APPLICATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Apply the actuator force to BodyA and BodyB.
 */
void Torque::
apply()
{	
	// TORQUE ON BODY A
	if(_bA!=_model->getGroundID()) {
		double fA[3];
		Mtx::Multiply(1,3,&_uA[0],_force,fA);
		_model->applyTorqueBodyLocal(_bA,fA);
	}

	// FORCE ON BODY B
	if(_bB!=_model->getGroundID()) {
		double fB[3];
		Mtx::Multiply(1,3,_uB,_force,fB);
		_model->applyTorqueBodyLocal(_bB,fB);
	}
}


//=============================================================================
// CHECK
//=============================================================================
//_____________________________________________________________________________
/**
 * Check that this force actuator has a valid set of states.
 */
bool Torque::
check() const
{
	Actuator::check();

	// BODY A
	if(_model!=NULL) {
		if((getBodyA()<_model->getGroundID())||(getBodyA()>=_model->getNB())) {
			printf("Torque.check: ERROR- %s has invalid body for BodyA (%d).\n",
				getName().c_str(),getBodyA());
			return(false);
		}
	}

	// BODY B
	if(_model!=NULL) {
		if((getBodyB()<_model->getGroundID())||(getBodyB()>=_model->getNB())) {
			printf("Torque.check: ERROR- %s has invalid body for BodyB (%d).\n",
				getName().c_str(),getBodyB());
			return(false);
		}
	}

	// SAME BODY
	if(getBodyA()==getBodyB()) {
		printf("Torque.check: WARN- %s has the same body for BodyA and BodyB\n",
			getName().c_str());
	}

	return(true);
}


//=============================================================================
// UTILITY
//=============================================================================
