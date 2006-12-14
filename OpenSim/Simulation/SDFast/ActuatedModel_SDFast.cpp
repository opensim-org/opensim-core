// ActuatedModel_SDFast.cpp
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
#include <iostream>
#include <string>
#include <math.h>
#include <float.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/Mtx.h>
#include <OpenSim/Tools/Memory.h>
#include <OpenSim/Simulation/Simm/AbstractBody.h>
#include <OpenSim/Simulation/Simm/AbstractActuator.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Simulation/Model/IntegCallbackSet.h>
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>
#include "ActuatedModel_SDFast.h"
#include "ActuatedModelIntegCallback.h"




using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTANTS
//=============================================================================



//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
ActuatedModel_SDFast::
~ActuatedModel_SDFast()
{
}

//_____________________________________________________________________________
/**
 * Default constructor.
 */
ActuatedModel_SDFast::
ActuatedModel_SDFast(const ActuatorSet *aActuatorSet,
							  const ContactForceSet *aContactSet) :
	_yi(0.0), _ypi(0.0)
{
	// NULL
	setNull();

	// ACTUATOR SET
	if(aActuatorSet!=NULL) {
		_actuatorSet = *aActuatorSet;
	}

	// CONTACT SET
	if(aContactSet!=NULL) {
		_contactSet = *aContactSet;
	}

	// INITIALIZE
	init();
}

//_____________________________________________________________________________
/**
 * Constructor from an XML Document
 */
ActuatedModel_SDFast::ActuatedModel_SDFast(const string &aFileName,
							ActuatorSet *aActuatorSet,
							ContactForceSet *aContactSet) :
	rdSDFast(aFileName), _yi(0.0), _ypi(0.0)
{
	// NULL
	setNull();

	// DESERIALIZE
	updateFromXMLNode();

	// SETS
	_actuatorSet = *aActuatorSet;
	_contactSet = *aContactSet;

	// INITIALIZE
	init();
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all the variable members of this class.
 */
void ActuatedModel_SDFast::
setNull()
{
}

//_____________________________________________________________________________
/**
 * Initialize the actuated model.
 */
void ActuatedModel_SDFast::
init()
{
	// ACTUATOR
	_actuatorSet.setModel(this);
	_actuatorSet.check();

	// CONTACT
	_contactSet.setModel(this);
	_contactSet.check();

	// ADD INTEGRATION STEP CALLBACK
	// This callback is for updating the pseudostates and anything else
	// that needs to be done after each integration step.
	ActuatedModelIntegCallback *integCallback =
		new ActuatedModelIntegCallback(this);
	addIntegCallback(integCallback);

	// INITIAL STATES AND PSEUDO-STATES
	_yi.setSize( ActuatedModel_SDFast::getNumStates() );
	_ypi.setSize( ActuatedModel_SDFast::getNumPseudoStates() );
}


//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the number of controls for this model.
 *
 * @return Number of model controls.
 */
int ActuatedModel_SDFast::
getNumControls() const
{
	return( _actuatorSet.getNumControls() );
}
//_____________________________________________________________________________
/**
 * Get the number of actuators in this model.
 *
 * @return Number of actuators.
 */
int ActuatedModel_SDFast::
getNumActuators() const
{
	return(_actuatorSet.getSize());
}
//_____________________________________________________________________________
/**
 * Get the number of contact points that are present given the current
 * state of the model.
 *
 * This method returns a valid result only after the contact points
 * have been computed.
 *
 * @return Number of contact points.
 * @see getContactPointsBodyLocal()
 */
int ActuatedModel_SDFast::
getNumContacts() const
{
	return(_contactSet.getSize());
}
//_____________________________________________________________________________
/**
 * Get the total number of states.
 *
 * @return Number of model states.
 * @todo Contact sets should be able to support having states.
 */
int ActuatedModel_SDFast::
getNumStates() const
{
	int ny = getNumCoordinates() + getNumSpeeds();
	ny += _actuatorSet.getNumStates();
	//ny += _contactSet.getNumStates();

	return(ny);
}
//_____________________________________________________________________________
/**
 * Get the total number of pseudo-states.
 *
 * @return Number of pseudo-states.
 */
int ActuatedModel_SDFast::
getNumPseudoStates() const
{
	int nyp = _actuatorSet.getNumPseudoStates();
	nyp += _contactSet.getNumPseudoStates();

	return(nyp);
}

//-----------------------------------------------------------------------------
// NAMES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the name of a control.
 *
 * @param aIndex Index of the control whose name is desired.  aIndex should
 * be greater than or equal to 0 and less than the number of controls.
 * @return Control name.
 * @see getNumControls()
 */
string ActuatedModel_SDFast::
getControlName(int aIndex) const
{
	return( _actuatorSet.getControlName(aIndex) );
}
//_____________________________________________________________________________
/**
 * Get the name of an actuator.
 *
 * @param aIndex Index of the actuator whose name is desired.  aIndex should
 * be greater than or equal to 0 and less than the number of actuators.
 * @return Control name.
 * @see getNumActuators()
 */
string ActuatedModel_SDFast::
getActuatorName(int aIndex) const
{
	AbstractActuator *act = _actuatorSet.get(aIndex);
	if(act==NULL) return("");
	return(act->getName());
}
//_____________________________________________________________________________
/**
 * Get the name of a state.
 *
 * @param aIndex Index of the state whose name is desired.  aIndex should
 * be greater than or equal to 0 and less than the number of states.
 * @return State name.  If aIndex is not valid an empty string "" is returned.
 */
string ActuatedModel_SDFast::
getStateName(int aIndex) const
{
	int index;
	string name = "";

	// NEGATIVE
	if(aIndex<0) {
		return(name);

	// COORDINATE
	}else if(aIndex < getNumCoordinates()) {
		name = rdSDFast::getCoordinateName(aIndex);

	// SPEED
	} else if(aIndex < (getNumCoordinates()+getNumSpeeds())) {
		index = aIndex - getNumCoordinates();
		name = rdSDFast::getSpeedName(index);

	// ACTUATOR
	} else if(aIndex < (getNumCoordinates()+getNumSpeeds()+_actuatorSet.getNumStates())) {
		index = aIndex - getNumCoordinates() - getNumSpeeds();
		name = _actuatorSet.getStateName(index);

	// CONTACT
	//} else if(aIndex < (getNumCoordinates()+getNumSpeeds()+_actuatorSet.getNumStates()+_contactSet.getNumStates())) {
	//	index = aIndex - getNumCoordinates() - getNumSpeeds() - _actuatorSet.getNumStates();
	//	name = _contactSet.getStateName(index);
	}

	return(name);
}
//_____________________________________________________________________________
/**
 * Get the name of a pseudo-state.
 *
 * @param aIndex Index of the pseudo-state whose name is desired.  aIndex
 * should be greater than or equal to 0 and less than the number of states.
 * @return Pseudo-state name.
 * @see getNumPseudoStates()
 */
string ActuatedModel_SDFast::
getPseudoStateName(int aIndex) const
{
	int index;
	string name = "";

	// NEGATIVE
	if(aIndex<0) {
		return("");

	// ACTUATOR
	} else if(aIndex < (_actuatorSet.getNumPseudoStates())) {
		name = _actuatorSet.getPseudoStateName(aIndex);

	// CONTACT SET
	} else if(aIndex< (_actuatorSet.getNumPseudoStates()+_contactSet.getNumPseudoStates())) {
		index = aIndex - _actuatorSet.getNumPseudoStates();
		name = _contactSet.getPseudoStateName(index);
	}

	return(name);
}

//-----------------------------------------------------------------------------
// INDICES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the index of an actuator given its name.
 *
 * Note that the actuator names need to be unique for this method
 * to work properly.
 *
 * @param aName Name of an actuator.
 * @return Index of the first actuator with the name aName; -1, if there is no
 * such actuator or if there is an error.
 */
int ActuatedModel_SDFast::
getActuatorIndex(const string &aName) const
{
	return( _actuatorSet.getIndex(aName) );
}
//_____________________________________________________________________________
/**
 * Get the index of a control given its name.
 * The returned indices start at 0: for the first, 0 is returned; for
 * the second, 1; etc.
 *
 * Note that the control names need to be unique for this method
 * to work properly.
 *
 * @param aName Name of a control.
 * @return Index of the first control with the name aName; -1, if there is no
 * such control or if there is an error.  Indicies start at 0.
 */
int ActuatedModel_SDFast::
getControlIndex(const string &aName) const
{
	return( _actuatorSet.getControlIndex(aName) );
}
//_____________________________________________________________________________
/**
 * Get the index of a state given its name.
 *
 * Note that the state names need to be unique for this method
 * to work properly.
 *
 * @param aName Name of a state.
 * @return Index of the first state with the name aName; -1, if there is no
 * such state or if there is an error.
 */
int ActuatedModel_SDFast::
getStateIndex(const string &aName) const
{
	int index;
	int nq = getNumCoordinates();
	int nu = getNumSpeeds();

	// COORDINATES
	index = getCoordinateIndex(aName);
	if(index>=0) {
		return(index);
	}

	// SPEEDS
	index = getSpeedIndex(aName);
	if(index>=0) {
		index += nq;
		return(index);
	}

	// ACTUATORS
	index = _actuatorSet.getStateIndex(aName);
	if(index>=0) {
		index += nq + nu;
		return(index);
	}

	// CONTACTS
	// Contact elements are currently assumed not to have any states.
	//if(index==-1) index = _contactSet.getStateIndex(aName);

	return(index);
}
//_____________________________________________________________________________
/**
 * Get the index of a pseudo-state given its name.
 * The returned indices start at 0: for the first, 0 is returned; for
 * the second, 1; etc.
 *
 * Note that the pseudo-state names need to be unique for this method
 * to work properly.
 *
 * @param aName Name of a pseudo-state.
 * @return Index of the first pseudo-state with the name aName; -1, if there
 * is no such pseudo-state or if there is an error.  Indices start at 0.
 */
int ActuatedModel_SDFast::
getPseudoStateIndex(const string &aName) const
{
	int index;

	index = _actuatorSet.getPseudoStateIndex(aName);
	if(index==-1) index = _contactSet.getPseudoStateIndex(aName);

	return(index);
}

//-----------------------------------------------------------------------------
// ACTUATOR SET
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the actuators for the model.
 *
 * @return Set of actuators.
 */
ActuatorSet* ActuatedModel_SDFast::
getActuatorSet()
{
	return(&_actuatorSet);
}

//-----------------------------------------------------------------------------
// CONTACT SET
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the contact forces for the model.
 *
 * @return Set of contact forces.
 */
ContactForceSet* ActuatedModel_SDFast::
getContactForceSet()
{
	return(&_contactSet);
}

//-----------------------------------------------------------------------------
// CONTROLS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the value of a control by index.
 *
 * @param aIndex Index of the control to be set:  0 <= aIndex < getNumControls().
 * @param aValue Value of the control.
 */
void ActuatedModel_SDFast::
setControl(int aIndex,double aValue)
{
	_actuatorSet.setControl(aIndex,aValue);
}
//_____________________________________________________________________________
/**
 * Set the value of a control by name.
 * Note that this method is slow and should not be used in code where it
 * might be called repeatedly.
 *
 * @param aName Name of the control to be set.
 * @param aValue Value of the control.
 */
void ActuatedModel_SDFast::
setControl(const string &aName,double aValue)
{
	_actuatorSet.setControl(aName,aValue);
}
//_____________________________________________________________________________
/**
 * Get the current value of a control by index.
 *
 * @param aIndex Index of the control:  0 <= aIndex < getNumControls().
 * @return Value of the control.  rdMath::NAN is returned on an error.
 */
double ActuatedModel_SDFast::
getControl(int aIndex) const
{
	double value = rdMath::NAN;
	value = _actuatorSet.getControl(aIndex);

	return(value);
}
//_____________________________________________________________________________
/**
 * Get the value of a control by name.
 * Note that this method is slow and should not be used in code where
 * it might be called repeatedly.
 *
 * @param aName Name of the control.
 * @return Value of the control.  rdMath::NAN is returned on an error.
 * @see getControls(double rX[])
 * @see getControl(int aIndex);
 */
double ActuatedModel_SDFast::
getControl(const std::string &aName) const
{
	double value = rdMath::NAN;
	value = _actuatorSet.getControl(aName);

	return(value);
}

//_____________________________________________________________________________
/**
 * Set the current controls for the model.
 *
 * The controls must be set for the model and the actuators.
 *
 * @param aX Array of controls.  The size of aX should be the value returned
 * by getNumControls().
 * @see ActuatedModel_SDFast::setControls();
 */
void ActuatedModel_SDFast::
setControls(const double aX[])
{
	_actuatorSet.setControls(aX);
}
//_____________________________________________________________________________
/**
 * Get the current controls for the model.
 *
 * The controls must be set for the model and the actuators.
 *
 * @param rX Array of controls.  The size of aX should be the value returned
 * by getNumControls().
 * @see ActuatedModel_SDFast::setControls();
 */
void ActuatedModel_SDFast::
getControls(double rX[]) const
{
	_actuatorSet.getControls(rX);
}

//-----------------------------------------------------------------------------
// STATES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the current states for this model.
 *
 * This method sets the configuration (coordinates and speeds) of the model
 * and also the states of the actuators.
 *
 * @param aY Array of states.  The size of aY should be the value returned by
 * getNumStates().
 * @see ActuatedModel_SDFast::setStates();
 * @see ActuatedModel_SDFast::setConfiguration().
 * @see rdSDFast::setConfiguration().
 */
void ActuatedModel_SDFast::
setStates(const double aY[])
{
	// BASE CLASS
	rdSDFast::setConfiguration(aY);

	// ACTUATORS
	int iAct = getNumCoordinates() + getNumSpeeds();
	if(iAct>=ActuatedModel_SDFast::getNumStates()) return;
	_actuatorSet.setStates(&aY[iAct]);
	//_contactSet.setStates(....);
}
//_____________________________________________________________________________
/**
 * Get the current states.
 *
 * @param aY Array of states.  The size of aYP should be the value
 * returned by getNumStates().
 * @see ActuatedModel_SDFast::setStates();
 */
void ActuatedModel_SDFast::
getStates(double rY[]) const
{
	// GENERALIZED COORDINATES AND SPEEDS
	int nq = getNumCoordinates();
	int nu = getNumSpeeds();
	getCoordinates(&rY[0]);
	getSpeedValues(&rY[nq]);

	// ACTUATORS
	_actuatorSet.getStates(&rY[nq+nu]);
}
//_____________________________________________________________________________
/**
 * Get the a state by index.
 *
 * @param aIndex  Array index of the desired state.
 * @return Value of the state.  On an error, rdMath::NAN is returned.
 */
double ActuatedModel_SDFast::
getState(int aIndex) const
{
	// NUMBERS
	int nyAct = _actuatorSet.getNumStates();
	//int nyCtx = _contactSet.getNumStates();
	double value = rdMath::NAN;

	// NEGATIVE
	if(aIndex<0) {

	// ACTUATOR SET
	} else if((aIndex>=0) && (aIndex<nyAct)) {
		value = _actuatorSet.getState(aIndex);

	// CONTACT SET- currently assumed to have no states.
	//} else if(aIndex < (nyAct+nyCtx)) {
	//	int index = aIndex - nyAct;
	//	value = _contactSet.getState(index);
	}

	return(value);
}
//_____________________________________________________________________________
/**
 * Get the value of a state by name.
 * Note that this method is slow and should not be used in code where
 * it might be called repeatedly.
 *
 * @param aName Name of the state.
 * @return Value of the state.  rdMath::NAN is returned on an error.
 * @see getStates(double rY[])
 * @see getState(int aIndex);
 */
double ActuatedModel_SDFast::
getState(const string &aName) const
{
	int index = getStateIndex(aName);
	if(index<0) {
		cout<<"ActuatedModel_SDFast.getState: ERROR- unknown state "<<
			aName<<"."<<endl;
		return(rdMath::NAN);
	}
	return(getState(index));
}

//-----------------------------------------------------------------------------
// PSEUDOSTATES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the current pseudo-states for this model.
 *
 * @param aYP Array of pseudo-states.  The size of aYP should be the value
 * returned by getNumPseudoStates().
 * @see ActuatedModel_SDFast::setPseudoStates();
 */
void ActuatedModel_SDFast::
setPseudoStates(const double aYP[])
{
	// ACTUATORS
	_actuatorSet.setPseudoStates(aYP);

	// CONTACTS
	_contactSet.setPseudoStates( &aYP[_actuatorSet.getNumPseudoStates()] );
}
//_____________________________________________________________________________
/**
 * Get the current pseudo-states for this model.
 *
 * @param aYP Array of pseudo-states.  The size of aYP should be the value
 * returned by getNumPseudoStates().
 * @see ActuatedModel_SDFast::setPseudoStates();
 */
void ActuatedModel_SDFast::
getPseudoStates(double rYP[]) const
{
	if(getNumPseudoStates()<=0) return;

	// ACTUATORS
	_actuatorSet.getPseudoStates(rYP);

	// CONTACTS
	_contactSet.getPseudoStates( &rYP[_actuatorSet.getNumPseudoStates()] );
}
//_____________________________________________________________________________
/**
 * Get the a pseudo-state by index.
 *
 * @param aIndex  Array index of the desired state.
 * @return Value of the pseudo-state.  On an error, rdMath::NAN is returned.
 */
double ActuatedModel_SDFast::
getPseudoState(int aIndex) const
{
	// NUMBERS
	int nypAct = _actuatorSet.getNumPseudoStates();
	int nypCtx = _contactSet.getNumPseudoStates();
	double value = rdMath::NAN;

	// NEGATIVE
	if(aIndex<0) {

	// ACTUATOR SET
	} else if((aIndex>=0) && (aIndex<nypAct)) {
		value = _actuatorSet.getPseudoState(aIndex);

	// CONTACT SET
	} else if(aIndex < (nypAct+nypCtx)) {
		int index = aIndex - nypAct;
		value = _contactSet.getPseudoState(index);
	}

	return(value);
}
//_____________________________________________________________________________
/**
 * Get the a pseudo-state by name.
 *
 * @param aName  Name of the desired pseudo state.
 * @return Value of the pseudo-state.  On an error, rdMath::NAN is returned.
 */
double ActuatedModel_SDFast::
getPseudoState(const string &aName) const
{
	int index = getPseudoStateIndex(aName);
	
	return(getPseudoState(index));
}

//-----------------------------------------------------------------------------
// INITIAL PSEUDO-STATES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the initial pseudo-states for this model.
 *
 * @param aYPI Array of pseudo-states.  The size of rYP must be at least the
 * number of pseudo-states, which can be found by calling getNumPseudoStates().
 */
void ActuatedModel_SDFast::
setInitialPseudoStates(const double aYPI[])
{
	if(aYPI==NULL) return;

	// SIZE
	int nyp = ActuatedModel_SDFast::getNumPseudoStates();
	_ypi.setSize(nyp);

	// SET
	int i;
	for(i=0;i<nyp;i++) {
		_ypi[i] = aYPI[i];
	}
}
//_____________________________________________________________________________
/**
 * Get the initial pseudo-states currently set for this model.
 *
 * @param rYPI Array to be filled with the initial pseudo-states.  The size of
 * rYP must be at least the number of pseudo-states, which can be found by
 * calling getNumPseudoStates().
 */
void ActuatedModel_SDFast::
getInitialPseudoStates(double rYPI[]) const
{
	if(rYPI==NULL) return;

	// GET
	int i;
	int nyp = ActuatedModel_SDFast::getNumPseudoStates();
	for(i=0;i<nyp;i++) {
		rYPI[i] = _ypi[i];
	}
}
//_____________________________________________________________________________
/**
 * Get the current value of an initial pseudo-state by index.
 *
 * @param aIndex Index of the initial pseudo-state:  0 <= aIndex < getNumPseudoStates().
 * @return Value of the initial pseudo-state.  rdMath::NAN is returned on an
 * error.
 * @see getInitialPseudoStates(double rYP[])
 * @see getInitialPseudoState(const char* aName);
 */
double ActuatedModel_SDFast::
getInitialPseudoState(int aIndex) const
{
	if((aIndex<0)||(aIndex>=ActuatedModel_SDFast::getNumPseudoStates())) {
		printf("ActuatedModel_SDFast.getInitialPseudoState: ERROR- index out of bounds");
		printf(" %d.\n",aIndex);
		return(rdMath::NAN);
	}
	return(_ypi[aIndex]);
}
//_____________________________________________________________________________
/**
 * Get the value of an initial pseudo-state by name.
 * Note that this method is slow and should not be used in code where
 * it might be called repeatedly.
 *
 * @param aName Name of the initial pseudo-state.
 * @return Value of the initial pseudo-state.  rdMath::NAN is returned on
 * an error.
 * @see getInitialPseudoStates(double rYPI[])
 * @see getInitialPseudoState(int aIndex);
 */
double ActuatedModel_SDFast::
getInitialPseudoState(const string &aName) const
{
	int index = getPseudoStateIndex(aName);
	if(index<0) {
		cout<<"ActuatedModel_SDFast.getInitialPseudoState: ERROR- unknown"<<
			" state "<<aName<<"."<<endl;
		return(rdMath::NAN);
	}
	return(getInitialPseudoState(index));
}

//-----------------------------------------------------------------------------
// INITIAL STATES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the initial states for this model.
 *
 * @param aYI Array of states.  The size of rY must be at least the number
 * of states, which can be found by calling getNumStates().
 */
void ActuatedModel_SDFast::
setInitialStates(const double aYI[])
{
	if(aYI==NULL) return;

	// SIZE
	int ny = ActuatedModel_SDFast::getNumStates();
	_yi.setSize(ny);

	// SET
	int i;
	for(i=0;i<ny;i++) {
		_yi[i] = aYI[i];
	}
}
//_____________________________________________________________________________
/**
 * Get the initial states currently set for this model.
 *
 * @param rYI Array to be filled with the initial states.  The size of rY must
 * be at least the number of states, which can be found by calling getNumStates().
 */
void ActuatedModel_SDFast::
getInitialStates(double rYI[]) const
{
	if(rYI==NULL) return;

	int i;
	int ny = ActuatedModel_SDFast::getNumStates();
	for(i=0;i<ny;i++) {
		rYI[i] = _yi[i];
	}
}
//_____________________________________________________________________________
/**
 * Get the current value of an initial state by index.
 *
 * @param aIndex Index of the initial state:  0 <= aIndex < getNumStates().
 * @return Value of the initial state.  rdMath::NAN is returned on an error.
 * @see getInitialStates(double rYI[])
 * @see getInitialState(const char* aName);
 */
double ActuatedModel_SDFast::
getInitialState(int aIndex) const
{
	if((aIndex<0)||(aIndex>=ActuatedModel_SDFast::getNumStates())) {
		printf("ActuatedModel_SDFast.getInitialState: ERROR- index out of bounds %d.\n",
			aIndex);
		return(rdMath::NAN);
	}
	return(_yi[aIndex]);
}
//_____________________________________________________________________________
/**
 * Get the value of an initial state by name.
 * Note that this method is slow and should not be used in code where
 * it might be called repeatedly.
 *
 * @param aName Name of the initial state.
 * @return Value of the initial state.  rdMath::NAN is returned on an error.
 * @see getInitialStates(double rYI[])
 * @see getInitialState(int aIndex);
 */
double ActuatedModel_SDFast::
getInitialState(const string &aName) const
{
	int index = getStateIndex(aName);
	if(index<0) {
		cout<<"ActuatedModel_SDFast.getInitialState: ERROR- unknown state "<<
			 aName<<"."<<endl;
		return(rdMath::NAN);
	}
	return(_yi[index]);
}


//=============================================================================
// ACTUATION
//=============================================================================
//-----------------------------------------------------------------------------
// COMPUTE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute all quantities associated with actuating a model.
 */
void ActuatedModel_SDFast::
computeActuation()
{
	_actuatorSet.computeActuation();
}

//_____________________________________________________________________________
/**
 * Compute time derivatives of all actuator states.
 */
void ActuatedModel_SDFast::
computeActuatorStateDerivatives(double rDY[])
{
	_actuatorSet.computeStateDerivatives(rDY);
}

//-----------------------------------------------------------------------------
// APPLY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Apply an actuator force.
 */
void ActuatedModel_SDFast::
applyActuatorForce(int aID)
{
	AbstractActuator *act = _actuatorSet.get(aID);
	if(act!=NULL) act->apply();
}
//_____________________________________________________________________________
/**
 * Apply actuator forces.
 */
void ActuatedModel_SDFast::
applyActuatorForces()
{
	_actuatorSet.apply();
}

//-----------------------------------------------------------------------------
// SCALARS- FORCE, SPEED, POWER
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the magnitude of force experted by a particular actuator.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNumContacts().
 * @return Magnitude of actuator force.
 */
void ActuatedModel_SDFast::
setActuatorForce(int aID,double aForce)
{
	AbstractActuator *act = _actuatorSet.get(aID);
	if(act==NULL) return;

	act->setForce(aForce);
}
//_____________________________________________________________________________
/**
 * Get the magnitude of force experted by a particular actuator.
 *
 * For the returned information to be valid, computeActuation() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNumContacts().
 * @return Magnitude of actuator force.
 */
double ActuatedModel_SDFast::
getActuatorForce(int aID) const
{
	AbstractActuator *act = _actuatorSet.get(aID);
	if(act==NULL) return(0.0);

	double force = act->getForce();
	return(force);
}
//_____________________________________________________________________________
/**
 * Get the stress in a particular actuator (force / area).
 *
 * For the returned information to be valid, computeActuation() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNumContacts().
 * @return Actuator stress.
 */
double ActuatedModel_SDFast::
getActuatorStress(int aID) const
{
	AbstractActuator *act = _actuatorSet.get(aID);
	if(act==NULL) return(0.0);

	double stress = act->getStress();
	return(stress);
}
//_____________________________________________________________________________
/**
 * Get the speed at which a particular actuator force is applied.
 *
 * For the returned information to be valid, computeActuation() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNumContacts().
 * @return Speed at which actuator force is applied.
 */
double ActuatedModel_SDFast::
getActuatorSpeed(int aID) const
{
	AbstractActuator *act = _actuatorSet.get(aID);
	if(act==NULL) return(0.0);

	double force = act->getSpeed();
	return(force);
}
//_____________________________________________________________________________
/**
 * Get the power delivered or absorbed by a particular actuator.
 * A positive power means the actuator is doing work on the model;
 * negative power means that the actuator is absorbing energy from the
 * model.
 *
 * For the returned information to be valid, computeActuation() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNumContacts().
 * @return Power delivered (positive) or absorbed (negative).
 */
double ActuatedModel_SDFast::
getActuatorPower(int aID) const
{
	AbstractActuator *act = _actuatorSet.get(aID);
	if(act==NULL) return(0.0);

	double power = act->getPower();
	return(power);
}

//=============================================================================
// CONTACT
//=============================================================================
//-----------------------------------------------------------------------------
// COMPUTE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute all quantities associated with simulating contact between bodies.
 * These quantities include at least the contact bodies, contact points, and
 * contact forces.
 *
 * @see getNumContacts()
 */
void ActuatedModel_SDFast::
computeContact()
{
	_contactSet.computeContact();
}

//-----------------------------------------------------------------------------
// APPLY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Apply a contacted force.
 *
 * For the correct contact forces to be applied correctly, computeContact()
 * must be called prior to calling this method.
 */
void ActuatedModel_SDFast::
applyContactForce(int aID)
{
	AbstractActuator *contact = _contactSet.get(aID);
	if(contact!=NULL) contact->apply();
}
//_____________________________________________________________________________
/**
 * Apply the computed contacted forces.
 *
 * For the correct contact forces to be applied correctly, computeContact()
 * must be called prior to calling this method.
 */
void ActuatedModel_SDFast::
applyContactForces()
{
	_contactSet.apply();
}

//-----------------------------------------------------------------------------
// BODY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the body number of BodyA for a particular contact force.
 *
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNumContacts().
 * @return Pointer to BodyA.
 */
AbstractBody* ActuatedModel_SDFast::
getContactBodyA(int aID) const
{
	ContactForce *contact = (ContactForce *)_contactSet.get(aID);
	if (contact == NULL)
		return NULL;

	return contact->getBodyA();
}
//_____________________________________________________________________________
/**
 * Get the body number of BodyB for a particular contact force.
 *
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNumContacts().
 * @return Pointer to BodyB.
 */
AbstractBody* ActuatedModel_SDFast::
getContactBodyB(int aID) const
{
	ContactForce *contact = (ContactForce *)_contactSet.get(aID);
	if (contact == NULL)
		return NULL;

	return contact->getBodyB();
}

//-----------------------------------------------------------------------------
// POINT A
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the contact point on BodyA expressed in the local frame of BodyA
 * for a specified contact.
 *
 * @param aID Index of the desired contact- should be greater than or
 * equal to zero and less than the value returned by getNumContacts().
 * @param aPoint Contact point on BodyA expressed in the local frame of
 * BodyA.
 */
void ActuatedModel_SDFast::
setContactPointA(int aID,const double aPoint[3])
{
	ContactForce *contact = (ContactForce *)_contactSet.get(aID);
	if(contact==NULL) return;
	contact->setPointA(aPoint);
}
//_____________________________________________________________________________
/**
 * Get the contact point on BodyA expressed in the local frame of BodyA
 * for a particular contact.
 * 
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact- should be greater than or
 * equal to zero and less than the value returned by getNumContacts().
 * @param rPoint Contact point on BodyA expressed in the local frame of
 * BodyA.
 */
void ActuatedModel_SDFast::
getContactPointA(int aID,double rPoint[3]) const
{
	ContactForce *contact = (ContactForce *)_contactSet.get(aID);
	if(contact==NULL) return;
	contact->getPointA(rPoint);
}

//-----------------------------------------------------------------------------
// POINT B
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the contact point on BodyB expressed in the local frame of BodyB
 * for a specified contact.
 *
 * @param aID Index of the desired contact- should be greater than or
 * equal to zero and less than the value returned by getNumContacts().
 * @param aPoint Contact point on BodyB expressed in the local frame of
 * BodyA.
 */
void ActuatedModel_SDFast::
setContactPointB(int aID,const double aPoint[3])
{
	ContactForce *contact = (ContactForce *)_contactSet.get(aID);
	if(contact==NULL) return;
	contact->setPointB(aPoint);
}
//_____________________________________________________________________________
/**
 * Get the contact point on BodyB expressed in the local frame of BodyB
 * for a particular contact.
 * 
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNumContacts().
 * @param rPoint Contact point on BodyB expressed in the local frame of
 * BodyB.
 */
void ActuatedModel_SDFast::
getContactPointB(int aID,double rPoint[3]) const
{
	ContactForce *contact = (ContactForce *)_contactSet.get(aID);
	if(contact==NULL) return;
	contact->getPointB(rPoint);
}

//-----------------------------------------------------------------------------
// FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the contact force acting on BodyB expressed in the local frame of
 * BodyA for a particular contact force.
 *
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNumContacts().
 * @param rF Contact force acting on BodyB expressed in the local frame of
 * BodyA.
 */
void ActuatedModel_SDFast::
getContactForce(int aID,double rF[3]) const
{
	ContactForce *contact = (ContactForce *)_contactSet.get(aID);
	if(contact==NULL) return;

	// NORMAL
	double fnp[3],fnv[3],fn[3];
	contact->getNormalForce(fnp,fnv,fn);

	// TANGENTIAL
	double ftp[3],ftv[3],ft[3];
	contact->getTangentialForce(ftp,ftv,ft);

	// RESULTANT
	Mtx::Add(1,3,fn,ft,rF);
}

//_____________________________________________________________________________
/**
 * Get the normal contact force acting on BodyB expressed in the local frame
 * of BodyA for a particular contact force.
 *
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNumContacts().
 * @param rFP Elastic normal contact force NOT corrected to enforce friction
 * constraints.
 * @param rFV Viscous normal contact force NOT corrected to enforce friction
 * constraints.
 * @param rF Total normal contact force acting on BodyB expressed in the local
 * frame of BodyA.  This is the actual normal force applied to BodyB.
 */
void ActuatedModel_SDFast::
getContactNormalForce(int aID,double rFP[3],double rFV[3],double rF[3]) const
{
	ContactForce *contact = (ContactForce *)_contactSet.get(aID);
	if(contact==NULL) return;

	// NORMAL
	contact->getNormalForce(rFP,rFV,rF);
}
//_____________________________________________________________________________
/**
 * Get the tangential contact force acting on BodyB expressed in the local
 * frame of BodyA for a particular contact force.
 *
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNumContacts().
 * @param rFP Elastic tangential contact force NOT corrected to enforce
 * friction constraints.
 * @param rFV Viscous tangential contact force NOT corrected to enforce
 * friction constraints.
 * @param rF Total tangential contact force acting on BodyB expressed in
 * the local frame of BodyA.  This is the actual tangential force applied to
 * BodyB.
 */
void ActuatedModel_SDFast::
getContactTangentForce(int aID,double rFP[3],double rFV[3],double rF[3])
	const
{
	ContactForce *contact = (ContactForce *)_contactSet.get(aID);
	if(contact==NULL) return;

	// NORMAL
	contact->getTangentialForce(rFP,rFV,rF);
}


//-----------------------------------------------------------------------------
// STIFFNESS AND VISCOSITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the instantaneous stiffness (or change in force on BodyB due
 * to a change in position of the BodyB contact point) of a specified
 * contact force.
 *
 * aDF = (stiffness) * aDX;   stiffness == df/dx  (partial of f wrt x)
 *
 * The displacement (aDX) is assumed to be expressed in the local frame of
 * BodyA and specifies a change in position of the BodyB contact point.
 * If the aDX is a unit vector, the returned value is the stiffness of the
 * contact force in the direction aDX.  However, if aDX is not a unit vector,
 * the returned value is the change in force applied to BodyB that would
 * occur for a displacment of aDX. 
 *
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNumContacts().
 * @param aDX Displacement of the BodyB contact point expressed in the
 * local frame of BodyA.
 * @param rDF Change in force applied to BodyB for the given displacement
 * (or stiffness if |aDX| = 1.0) expressed in the local frame of BodyA.
 */
void ActuatedModel_SDFast::
getContactStiffness(int aID,const double aDX[3],double rDF[3]) const
{
	if(aDX==NULL) return;
	if(rDF==NULL) return;
	ContactForce *contact = (ContactForce *)_contactSet.get(aID);
	if(contact==NULL) return;

	// NORMAL
	double k,n[3],dotDX;
	double dxn[3],dfn[3];
	contact->getNormalA(n);
	k = -contact->getInstantaneousNormalStiffness();
	dotDX = Mtx::DotProduct(3,n,aDX);
	Mtx::Multiply(1,3,n,dotDX,dxn);
	Mtx::Multiply(1,3,dxn,k,dfn);

	// TANGENTIAL
	double dxt[3],dft[3];
	Mtx::Subtract(1,3,aDX,dxn,dxt);
	k = -contact->getInstantaneousTangentialStiffness();
	Mtx::Multiply(1,3,dxt,k,dft);

	// RESULTANT
	Mtx::Add(1,3,dfn,dft,rDF);
}
//_____________________________________________________________________________
/**
 * Get the instantaneous viscosity (or change in force on BodyB due
 * to a change in velocity of the BodyB contact point) of a specified
 * contact force.
 *
 * aDF = (viscosity) * aDV;   viscosity == df/dv  (partial of f wrt v)
 *
 * The velocity change (aDV) is assumed to be expressed in the local frame of
 * BodyA and specifies a change in velocity of the BodyB contact point.
 * If the aDV is a unit vector, the returned value is the visocity of the
 * contact force in the direction aDV.  However, if aDV is not a unit vector,
 * the returned value is the change in force applied to BodyB that would
 * occur for a change in velocity of aDV. 
 *
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNumContacts().
 * @param aDV Change in velocity of the BodyB contact point expressed in the
 * local frame of BodyA.
 * @param rDF Change in force applied to BodyB for the given velocity change
 * (or viscosity if |aDX| = 1.0) expressed in the local frame of BodyA.
 */
void ActuatedModel_SDFast::
getContactViscosity(int aID,const double aDV[3],double rDF[3]) const
{
	if(aDV==NULL) return;
	if(rDF==NULL) return;
	ContactForce *contact = (ContactForce *)_contactSet.get(aID);
	if(contact==NULL) return;

	// NORMAL
	double k,n[3],dotDV;
	double dvn[3],dfn[3];
	contact->getNormalA(n);
	k = -contact->getInstantaneousNormalViscosity();
	dotDV = Mtx::DotProduct(3,n,aDV);
	Mtx::Multiply(1,3,n,dotDV,dvn);
	Mtx::Multiply(1,3,dvn,k,dfn);

	// TANGENTIAL
	double dvt[3],dft[3];
	Mtx::Subtract(1,3,aDV,dvn,dvt);
	k = -contact->getInstantaneousTangentialViscosity();
	Mtx::Multiply(1,3,dvt,k,dft);

	// RESULTANT
	Mtx::Add(1,3,dfn,dft,rDF);
}

//-----------------------------------------------------------------------------
// FRICTION CORRECTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the correction that was made to the contact force to enforce friction
 * constraints.  This correction is returned as the change in contact force
 * applied to BodyB expressed in the local frame of BodyA.
 *
 * Note that this correction is NOT intended to express changes in contact
 * force due to damping terms, but rather changes due to the enforcement of
 * constraints on the tangential component of the contact force, such as
 * enforcing a coefficient of friction (e.g., fx <= mu*fy).
 *
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNumContacts().
 * @param rCorrection Change in contact force due to enforcing friction
 * constraints.  This correction is returned as the change in contact force
 * applied to BodyA expressed in the local frame of BodyA.
 */
void ActuatedModel_SDFast::
getContactFrictionCorrection(int aID,double rDFFric[3]) const
{
	ContactForce *contact = (ContactForce *)_contactSet.get(aID);
	if(contact==NULL) return;

	contact->getFrictionCorrection(rDFFric);
}

//-----------------------------------------------------------------------------
// SCALARS- FORCE, SPEED, POWER
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the magnitude of force experted by a particular contact force.
 *
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNumContacts().
 * @return Magnitude of contact force.
 */
double ActuatedModel_SDFast::
getContactForce(int aID) const
{
	ContactForce *contact = (ContactForce *)_contactSet.get(aID);
	if(contact==NULL) return(0.0);

	double power = contact->getForce();
	return(power);
}
//_____________________________________________________________________________
/**
 * Get the speed at which a particular contact force is applied.
 *
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNumContacts().
 * @return Speed at which contact force is applied.
 */
double ActuatedModel_SDFast::
getContactSpeed(int aID) const
{
	ContactForce *contact = (ContactForce *)_contactSet.get(aID);
	if(contact==NULL) return(0.0);

	double power = contact->getSpeed();
	return(power);
}
//_____________________________________________________________________________
/**
 * Get the power delivered or absorbed by a particular contact force.
 * A positive power means the contact force is doing work on the model;
 * negative power means that the contact force is absorbing energy from the
 * model.
 *
 * For the returned information to be valid, computeContact() must be called
 * prior to calling this method.
 *
 * @param aID Index of the desired contact force- should be greater than or
 * equal to zero and less than the value returned by getNumContacts().
 * @return Power delivered (positive) or absorbed (negative).
 */
double ActuatedModel_SDFast::
getContactPower(int aID) const
{
	AbstractActuator *contact = _contactSet.get(aID);
	if(contact==NULL) return(0.0);

	double power = contact->getPower();
	return(power);
}


//=============================================================================
// OPTIMIZATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Promote a set of controls to state variables.  This method is
 * normally useful when solving for controls that will optimize some
 * aspect of a model performance.  For example, in models in which the
 * controls are neural excitations, but muscle forces are determined by
 * activation level, this method might set the muscle activations equal
 * to the excitations.  Each actuator is responsible for knowing how to
 * promote its own controls to states.  For static problems, the time
 * step should be set to zero (i.e., aDT=0.0) for dynamic problems, like
 * during a forward dynamic simulation, aDT should be set to an appropriate
 * time step value, like the integration step size.
 *
 * @param aX Array of controls.
 * @param aDT Time step.
 */
void ActuatedModel_SDFast::
promoteControlsToStates(const double aX[],double aDT)
{
	_actuatorSet.promoteControlsToStates(aX,aDT);
}



//=============================================================================
// DERIVATIVES
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the derivatives of the auxiliary states.  The auxiliary states
 * are any integrated variables that are not the coordinates or speeds.
 *
 * @param dydt Array of the time derivatives of the auxiliary states.  The
 * auxiliary state derivatives should start at the beginning of the dydt
 * array.
 */
void ActuatedModel_SDFast::
computeAuxiliaryDerivatives(double *dydt)
{
	_actuatorSet.computeStateDerivatives(dydt);
}



//=============================================================================
// TYPE REGISTRATION
//=============================================================================
///_____________________________________________________________________________
/**
 * Register the types of objects used by ActuatedModel_SDFast and that need to be registered 
 * for xml serialization purposes
 *
 */
void ActuatedModel_SDFast::
RegisterTypes()
{	
	rdSDFast::RegisterTypes();

	Object::RegisterType(ActuatorSet());
	Object::RegisterType(ContactForceSet());
}
