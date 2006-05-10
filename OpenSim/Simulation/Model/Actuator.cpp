// Actuator.cpp
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
#include "Actuator.h"
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/Memory.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/PropertyDbl.h>
//#include <rdToolsTemplates.h>




using namespace OpenSim;
using namespace std;



static const string X_NAME = "excitation";


//=============================================================================
// STATICS
//=============================================================================
const double Actuator::LARGE = 1.0e8;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Actuator::~Actuator()
{
}
//_____________________________________________________________________________
/**
 * Construct an actuator that has a specified number of controls, states,
 * and pseudostates.
 *
 * @param aNX Number of controls.
 * @param aNY Number of states.
 * @param aNYP Number of pseudostates.
 */
Actuator::Actuator(int aNX,int aNY,int aNYP) :
	_area(_propArea.getValueDbl()),
	_minForce(_propMinForce.getValueDbl()),
	_maxForce(_propMaxForce.getValueDbl()),
	_optimalForce(_propOptimalForce.getValueDbl())
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Construct an actuator from an XML Element.
 *
 * @param aNX Number of controls.
 * @param aNY Number of states.
 * @param aNYP Number of pseudo-states.
 * @param aElement XML element.
 */
Actuator::Actuator(int aNX,int aNY,int aNYP,DOMElement *aElement) :
	Object(aElement),
	_area(_propArea.getValueDbl()),
	_minForce(_propMinForce.getValueDbl()),
	_maxForce(_propMaxForce.getValueDbl()),
	_optimalForce(_propOptimalForce.getValueDbl())
{
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aActuator Actuator to copy.
 */
Actuator::Actuator(const Actuator &aAct) :
	Object(aAct),
	_area(_propArea.getValueDbl()),
	_minForce(_propMinForce.getValueDbl()),
	_maxForce(_propMaxForce.getValueDbl()),
	_optimalForce(_propOptimalForce.getValueDbl())
{
	setNull();

	// ASSIGN
	*this = aAct;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void Actuator::
setNull()
{
	setType("Actuator");
	setName(ObjectDEFAULT_NAME);
	setupProperties();
	_model = NULL;

	_appliesForce = true;
	_force = 0.0;
	_speed = 0.0;
	_x = 0.0;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Actuator::
setupProperties()
{
	_propArea.setName("area");
	_propArea.setValue(1.0);
	_propertySet.append( &_propArea );

	_propMinForce.setName("min_force");
	_propMinForce.setValue(-LARGE);
	_propertySet.append( &_propMinForce );

	_propMaxForce.setName("max_force");
	_propMaxForce.setValue(LARGE);
	_propertySet.append( &_propMaxForce );

	_propOptimalForce.setName("optimal_force");
	_propOptimalForce.setValue(1.0);
	_propertySet.append( &_propOptimalForce );
}


//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
Actuator& Actuator::
operator=(const Actuator &aAct)
{
	// BASE CLASS
	Object::operator=(aAct);

	// MODEL
	_model = aAct.getModel();

	// AREA
	_area = aAct.getArea();

	// FORCE
	_appliesForce = aAct.getAppliesForce();
	setForce(aAct.getForce());

	// SPEED
	_speed = aAct.getSpeed();

	// MAX FORCE
	setMaxForce(aAct.getMaxForce());

	// MIN FORCE
	setMinForce(aAct.getMinForce());

	// OPTIMAL FORCE
	setOptimalForce(aAct.getOptimalForce());

	// EXCITATION
	_x = aAct._x;

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// MODEL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the model which this actuator actuates.
 *
 * @param aModel Pointer to a model.
 */
void Actuator::
setModel(Model *aModel)
{
	_model = aModel;
}

//_____________________________________________________________________________
/**
 * Get a pointer to the model on which this analysis is being performed.
 *
 * @return Pointer to the model.
 */
Model* Actuator::
getModel() const
{
	return(_model);
}

//-----------------------------------------------------------------------------
// CONTROLS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the number of controls.
 *
 * @return Number of controls (1 for an Actuator actuator).
 */
int Actuator::
getNX() const
{
	return(1);
}
//_____________________________________________________________________________
/**
 * Get the name of a control.\n
 * Valid indices: 0
 *
 * @param aIndex Index of the control whose name is desired.
 * @throws Exception If aIndex is not valid.
 */
const string Actuator::
getControlName(int aIndex) const
{
	if(aIndex!=0) {
		string msg = "Actuator.setControl: ERR- index out of bounds.\n";
		msg += "Actuator ";
		msg += getName();
		msg += " of type ";
		msg += getType();
		msg += " has only 1 control (only an index of 0 is valid).";
		throw( Exception(msg,__FILE__,__LINE__) );
	}
	string name = getName();
	name += ".";
	name += X_NAME;
	return(name);
}
//_____________________________________________________________________________
/**
 * Get the index of a control of a specified name.\n
 * Valid names: excitation
 *
 * @param aName Name of the control whose index is desired.
 * @return Index of the specified control.
 * @throws Exception If aName is not valid.
 */
int Actuator::
getControlIndex(const string &aName) const
{
	if((aName==X_NAME)||(aName!=getControlName(0))) {
		string msg = "Actuator.getControlIndex: ERR- Actuator ";
		msg += getName();
		msg += " of type ";
		msg += getType();
		msg += " has no control by the name";
		msg += aName;
		msg += ".\nThe only valid control name is ";
		msg += getControlName(0);
		msg += ".";
		throw( Exception(msg,__FILE__,__LINE__) );
	}
	return(0);
}
//_____________________________________________________________________________
/**
 * Set the current values of the controls.  This actuator has one control:
 * the force mangnitude of the actuator.
 *
 * @param aX Array of control values.  aX should be at least a size of 1.
 */
void Actuator::
setControls(const double aX[])
{
		_x = aX[0];
}
//_____________________________________________________________________________
/**
 * Set the value of a control at a specified index.\n
 * Valid indices:  0
 *
 * @param aIndex Index of the control to be set.
 * @param aValue Value to which to set the control.
 * @throws Exception If aIndex is not valid.
 */
void Actuator::
setControl(int aIndex,double aValue)
{
	if(aIndex!=0) {
		string msg = "Actuator.setControl: ERR- index out of bounds.\n";
		msg += "Actuator ";
		msg += getName();
		msg += " of type ";
		msg += getType();
		msg += " has only 1 control (only an index of 0 is valid).";
		throw( Exception(msg,__FILE__,__LINE__) );
	}
	_x = aValue;
}
//_____________________________________________________________________________
/**
 * Set the value of a control of a specified name.\n
 * Valid names: excitation
 *
 * @param aName Name of the control to be set.
 * @param aValue Value to which to set the control.
 * @throws Exception If aName is not valid.
 */
void Actuator::
setControl(const string &aName,double aValue)
{
	int index = getControlIndex(aName);
	setControl(index,aValue);
}
//_____________________________________________________________________________
/**
 * Get the current values of the controls.
 *
 * @param rX Array of control values.  The size of rX should be at least 1.
 */
void Actuator::
getControls(double rX[]) const
{
	rX[0] = _x;
}
//_____________________________________________________________________________
/**
 * Get the value of a control at a specified index.\n
 *	Valid Indices: 0
 *
 * @param aIndex Index of the desired control.
 * @return Value of the desired control.
 * @throws Exception If aIndex is not valid.
 */
double Actuator::
getControl(int aIndex) const
{
	if(aIndex!=0) {
		string msg = "Actuator.setControl: ERR- index out of bounds.\n";
		msg += "Actuator ";
		msg += getName();
		msg += " of type ";
		msg += getType();
		msg += " has only 1 control (only an index of 0 is valid).";
		throw( Exception(msg,__FILE__,__LINE__) );
	}
	return(_x);
}
//_____________________________________________________________________________
/**
 * Get the value of a control of a specified name.\n
 * Valid names: excitation
 *
 * @param aName Name of the desired control.
 * @return Value of the desired control.
 * @throws Exception If aName is not valid.
 */
double Actuator::
getControl(const string &aName) const
{
	int index = getControlIndex(aName);
	return(getControl(index));
}


//-----------------------------------------------------------------------------
// STATES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the number of states.
 *
 * Derived classes should override this method.
 *
 * @return Number of states.
 */
int Actuator::
getNY() const
{
	return(0);
}
//_____________________________________________________________________________
/**
 * Get the name of a state at a specified index.\n
 * Valid indices: none
 *
 * Derived classes should override this method.
 *
 * @param aIndex Index of the state whose name is desired.
 * @throws Exception If aIndex is not valid.
 */
const string& Actuator::
getStateName(int aIndex) const
{
	string msg = "Actuator.getStateName: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no states.";
	throw( Exception(msg,__FILE__,__LINE__) );
}
//_____________________________________________________________________________
/**
 * Get the index of a state of a specified name.\n
 * Valid names: (there are no valid names)
 *
 * Derived classes should override this method.
 *
 * @param aName Name of the state whose index is desired.
 * @return Index of the specified state.
 * @throws Exception If aName is not valid.
 */
int Actuator::
getStateIndex(const string &aName) const
{
	string msg = "Actuator.getStateIndex: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no state by the name";
	msg += aName;
	msg += ".";
	throw( Exception(msg,__FILE__,__LINE__) );
}
//_____________________________________________________________________________
/**
 * Set the current values of the states.
 *
 * @param aY Array of state values.
 * @throws Exception  This actuator has no states.
 */
void Actuator::
setStates(const double aY[])
{
	string msg = "Actuator.setStates: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no states.";
	throw( Exception(msg,__FILE__,__LINE__) );
}
//_____________________________________________________________________________
/**
 * Set the value of a state at a specified index.\n
 * Valid indices: none
 *
 * Derived classes should override this method.
 *
 * @param aIndex Index of the state to be set.
 * @param aValue Value to which to set the state.
 * @throws Exception If aIndex is not valid.
 */
void Actuator::
setState(int aIndex,double aValue)
{
	string msg = "Actuator.setState: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no states.";
	throw( Exception(msg,__FILE__,__LINE__) );
}
//_____________________________________________________________________________
/**
 * Set the value of a state of a specified name.\n
 * Valid names: (there are no valid names)
 *
 * Derived classes should override this method.
 *
 * @param aName Name of the state to be set.
 * @param aValue Value to which to set the state.
 * @throws Exception If aName is not valid.
 */
void Actuator::
setState(const string &aName,double aValue)
{
	string msg = "Actuator.setState: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no states.";
	throw( Exception(msg,__FILE__,__LINE__) );
}
//_____________________________________________________________________________
/**
 * Get the current values of the states.
 *
 * @param rYP Array of state values.
 * @throws Exception This actuator has no states.
 */
void Actuator::
getStates(double rY[]) const
{
	string msg = "Actuator.getStates: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no states.";
	throw( Exception(msg,__FILE__,__LINE__) );
}
//_____________________________________________________________________________
/**
 * Get the value of a state at a specified index.\n
 * Valid indices: none
 *
 * Derived classes should override this method.
 *
 * @param aIndex Index of the desired state.
 * @return Value of the desired state.
 * @throws Exception If aIndex is not valid.
 */
double Actuator::
getState(int aIndex) const
{
	string msg = "Actuator.getState: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no states.";
	throw( Exception(msg,__FILE__,__LINE__) );
}
//_____________________________________________________________________________
/**
 * Get the value of a state of a specified name.\n
 * Valid names: (there are no valid names)
 *
 * Derived classes should override this method.
 *
 * @param aName Name of the desired state.
 * @return Value of the desired state.
 * @throws Exception If aName is not valid
 */
double Actuator::
getState(const string &aName) const
{
	string msg = "Actuator.getState: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no states.";
	throw( Exception(msg,__FILE__,__LINE__) );
}

//-----------------------------------------------------------------------------
// PSEUDOSTATES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the number of pseudostates.
 *
 * Derived classes should override this method.
 *
 * @return Number of pseudostates.
 */
int Actuator::
getNYP() const
{
	return(0);
}
//_____________________________________________________________________________
/**
 * Get the name of a pseudostate at a specified index.\n
 * Valid indices: none
 *
 * Derived classes should override this method.
 *
 * @param aIndex Index of the pseudostate whose name is desired.
 * @throws Exception If aIndex is not valid.
 */
const string& Actuator::
getPseudoStateName(int aIndex) const
{
	string msg = "Actuator.getPseudoStateName: ERR- Actuator ";
	msg += getName();
	msg += " has no pseudostates.";
	throw( Exception(msg,__FILE__,__LINE__) );
}
//_____________________________________________________________________________
/**
 * Get the index of a pseudostate of a specified name.\n
 * Valid names: (there are no valid names)
 *
 * Derived classes should override this method.
 *
 * @param aName Name of the pseudostate whose index is desired.
 * @return Index of the specified pseudostate.
 * @throws Exception If aName is not valid.
 */
int Actuator::
getPseudoStateIndex(const string &aName) const
{
	string msg = "Actuator.getPseudoStateIndex: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no pseudostate by the name";
	msg += aName;
	msg += ".";
	throw Exception(msg,__FILE__,__LINE__);
}
//_____________________________________________________________________________
/**
 * Set the current values of the pseudostates.
 *
 * @param aYP Array of pseudostate values.
 * @throws Exception This actuator has no pseudostates.
 */
void Actuator::
setPseudoStates(const double aYP[])
{
	string msg = "Actuator.setPseudoStates: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no states.";
	throw( Exception(msg,__FILE__,__LINE__) );
}
//_____________________________________________________________________________
/**
 * Set the value of a pseudostate at a specified index.\n
 * Valid indices: none
 *
 * Derived classes should override this method.
 *
 * @param aIndex Index of the pseudostate to be set.
 * @param aValue Value to which to set the pseudostate.
 * @throws Exception If aIndex is not valid.
 */
void Actuator::
setPseudoState(int aIndex,double aValue)
{
	string msg = "Actuator.setPseudoState: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no pseudostates.";
	throw( Exception(msg,__FILE__,__LINE__) );
}
//_____________________________________________________________________________
/**
 * Set the value of a pseudostate of a specified name.\n
 * Valid names: (there are no valid names)
 *
 * Derived classes should override this method.
 *
 * @param aName Name of the pseudostate to be set.
 * @param aValue Value to which to set the pseudostate.
 * @throws Exception If aname is not valid.
 */
void Actuator::
setPseudoState(const string &aName,double aValue)
{
	string msg = "Actuator.setPseudoState: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no pseudostates.";
	throw( Exception(msg,__FILE__,__LINE__) );
}
//_____________________________________________________________________________
/**
 * Get the current values of the pseudostates.
 *
 * @param rYP Array of pseudostate values.
 * @throws Exception This actuator has no pseudostates.
 */
void Actuator::
getPseudoStates(double rYP[]) const
{
	string msg = "Actuator.getPseudoStates: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no pseudostates.";
	throw( Exception(msg,__FILE__,__LINE__) );
}
//_____________________________________________________________________________
/**
 * Get the value of a pseudostate at a specified index.\n
 * Valid indices: none
 *
 * Derived classes should override this method.
 *
 * @param aIndex Index of the desired pseudostate.
 * @return Value of the desired pseudostate.
 * @throws Exception If aIndex is not valid.
 */
double Actuator::
getPseudoState(int aIndex) const
{
	string msg = "Actuator.getPseudoState: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no pseudostates.";
	throw( Exception(msg,__FILE__,__LINE__) );
}
//_____________________________________________________________________________
/**
 * Get the value of a pseudostate of a specified name.\n
 * Valid names: (there are no valid names)
 *
 * Derived classes should override this method.
 *
 * @param aName Name of the desired pseudostate.
 * @return Value of the desired pseudostate.
 * @throws Exception If aName is not valid.
 */
double Actuator::
getPseudoState(const string &aName) const
{
	string msg = "Actuator.getPseudoState: ERR- Actuator ";
	msg += getName();
	msg += " of type ";
	msg += getType();
	msg += " has no pseudostates.";
	throw( Exception(msg,__FILE__,__LINE__) );
}

//-----------------------------------------------------------------------------
// AREA
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the current area of the actuator.
 *
 * @param aArea Area of the actuator.
 */
void Actuator::
setArea(double aArea)
{
	_area = aArea;
	if(_area<rdMath::ZERO) _area = rdMath::ZERO;
}
//_____________________________________________________________________________
/**
 * Get the current area of the actuator.
 *
 * @return Area of the actuator.
 */
double Actuator::
getArea() const
{
	return(_area);
}

//-----------------------------------------------------------------------------
// FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether this actuator applies a force or torque to the model.  This
 * method is protected, so it can only be called by derived classes.
 *
 * @return True if the actuator applies a force, false if the actuator applies
 * a torque.
 */
void Actuator::
setAppliesForce(bool aTrueFalse)
{
	_appliesForce = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Get whether the actuator applies a force or torque to the model.
 *
 * @return True if the actuator applies a force, false if the actuator applies
 * a torque.
 */
bool Actuator::
getAppliesForce() const
{
	return(_appliesForce);
}
//_____________________________________________________________________________
/**
 * Set the current force (or torque) of the actuator.
 *
 * The force (or torque) is represented as a scalar; its vector properties
 * derive from the geometry characterizing how it is applied to the model.
 * The particulars of the geometry is dependent on the type of the actuator.
 *
 * Note that this method clamps the force between the maximum and minimum
 * force allowed for the actuator.
 *
 * @param aForce Force (or torque) of the actuator.
 * @see Actuator::setMaxForce()
 * @see Actuator::setMinForce()
 */
void Actuator::
setForce(double aForce)
{
	_force = aForce;
	if(_force>_maxForce) _force = _maxForce;
	else if(_force<_minForce) _force = _minForce;
}
//_____________________________________________________________________________
/**
 * Get the current force (or torque) of the actuator.
 *
 * The force (or torque) is represented as a scalar; its vector properties
 * derive from the geometry characterizing how it is applied to the model.
 * The particulars of the geometry is dependent on the type of the actuator.
 *
 * @return Force (or torque) of the actuator.
 */
double Actuator::
getForce() const
{
	return(_force);
}
//_____________________________________________________________________________
/**
 * Get the current stress in the actuator (force / area).
 *
 * @return Stress in the actuator.
 */
double Actuator::
getStress() const
{
	return(fabs(_force/_optimalForce));
}

//-----------------------------------------------------------------------------
// SPEED
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the current speed (linear or angular) of the actuator.
 *
 * The speed is the dot product of the direction of force applied by
 * the actuator to Body1 and the velocity of shortening of the actuator:
 *
 *		speed = DotProduct(u1,v), where
 *    u1	=  unit vector describing the direction of force applied to Body1
 *          expressed in the Body0 frame.
 *		v  =  velocity of Point1 in the frame of Body0 expressed in the
 *          Body0 frame.
 *
 * @return Speed (or angular velocity) of the actuator.
 */
double Actuator::
getSpeed() const
{
	return(_speed);
}

//-----------------------------------------------------------------------------
// POWER
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the instantaneous power developed by an actuator.
 * The instantaneous power is the product of the actuator force and the
 * actuator speed.  If the power is positive, the actuator is delivering
 * energy to the model.  If the power is negative, the actuator is absorbing
 * energy from the model.
 *
 * @return Instantaneous power of the actuator.
 */
double Actuator::
getPower() const
{
	return(_force*_speed);
}

//-----------------------------------------------------------------------------
// MAXIMUM FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the maximum force (or torque) that can by applied by the actuator.
 *
 * @param aMax Maximum force or torque.
 */
void Actuator::
setMaxForce(double aMax)
{
	_maxForce = aMax;
	if(_maxForce<_minForce) _minForce = _maxForce;
}
//_____________________________________________________________________________
/**
 * Get the maximum force (or torque) that can by applied by the actuator.
 *
 * @return Maximum force or torque.
 */
double Actuator::
getMaxForce() const
{
	return(_maxForce);
}

//-----------------------------------------------------------------------------
// MINIMUM FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the minimum force (or torque) that can by applied by the actuator.
 *
 * @param aMin Minimum force or torque.
 */
void Actuator::
setMinForce(double aMin)
{
	_minForce = aMin;
	if(_minForce>_maxForce) _maxForce = _minForce;
}
//_____________________________________________________________________________
/**
 * Get the minimum force (or torque) that can by applied by the actuator.
 *
 * @return Minimum force or torque.
 */
double Actuator::
getMinForce() const
{
	return(_minForce);
}

//-----------------------------------------------------------------------------
// OPTIMAL FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the optimal force of the actuator.
 *
 * @param aOptimalForce Optimal force.
 */
void Actuator::
setOptimalForce(double aOptimalForce)
{
	_optimalForce = aOptimalForce;
}
//_____________________________________________________________________________
/**
 * Get the optimal force of the actuator.
 *
 * @return Optimal force.
 */
double Actuator::
getOptimalForce() const
{
	return(_optimalForce);
}


//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Promote a set of controls to state variables.  This method is
 * normally useful when solving for controls that will optimize some
 * aspect of a model performance.  For example, in models in which the
 * controls are neural excitations, but muscle forces are determined by
 * activation level, this method might set the muscle activations equal
 * to the excitations.  Each actuator is responsible for knowing how to
 * promote its own controls to states.
 */
void Actuator::
promoteControlsToStates(const double aX[],double aDT)
{
}

//_____________________________________________________________________________
/**
 * Compute the time derivatives of the states for this actuator.
 *
 * This method should be overridden by derived classes that have actuator
 * states.  In Actuator this method is empty and put here so that
 * derived classes that don't have any actuator states won't have to
 * implement this method.
 *
 * @param rDYDT Time derivatives of the states.
 */
void Actuator::
computeStateDerivatives(double rDYDT[])
{
}
//_____________________________________________________________________________
/**
 * Update actuator parameters so they are intervally consistent.
 * This method is distinguished from computeActuation() in that it is normally
 * called only after an integration time step has been taken.  Updating
 * spring setpoints is an example.
 */
void Actuator::
updatePseudoStates()
{
}


//=============================================================================
// CHECK
//=============================================================================
//_____________________________________________________________________________
/**
 * Check that this actuator is valid.
 */
bool Actuator::
check() const
{
	// MODEL
	if(_model==NULL) {
		printf("Actuator.check: WARN- no model set for %s.\n",
			getName().c_str());
		return(false);
	}

	return(true);
}
