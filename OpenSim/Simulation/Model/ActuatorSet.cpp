// ActuatorSet.cpp
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
#include "ActuatorSet.h"
#include <OpenSim/Tools/IO.h>
#include <iostream>
#include <OpenSim/Tools/PropertyObjArray.h>
#include <OpenSim/Tools/rdMath.h>
//#include <rdToolsTemplates.h>




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
ActuatorSet::~ActuatorSet()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ActuatorSet::ActuatorSet() :
	_actuatorToControl(-1), _controlToActuator(-1),
	_actuatorToState(-1), _stateToActuator(-1),
	_actuatorToPseudo(-1), _pseudoToActuator(-1)
{
	setNull();

	// INDICES
	constructControlMaps();
	constructStateMaps();
	constructPseudoStateMaps();
}
//_____________________________________________________________________________
/**
 * Construct an actuator set from file.
 *
 * @param aFileName Name of the file.
 */
ActuatorSet::ActuatorSet(const char *aFileName) :
	Set<Actuator>(aFileName),
	_actuatorToControl(-1), _controlToActuator(-1),
	_actuatorToState(-1), _stateToActuator(-1),
	_actuatorToPseudo(-1), _pseudoToActuator(-1)
{
	setNull();
	updateFromXMLNode();
	// removeInvalidObjects();

	// INDICES
	constructControlMaps();
	constructStateMaps();
	constructPseudoStateMaps();
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void ActuatorSet::
setNull()
{
	// TYPE
	setType("ActuatorSet");
	// NAME
	setName("ActuatorSet");

	// PROPERTIES
	setupSerializedMembers();

	// MODEL
	_model = NULL;
}
//_____________________________________________________________________________
/**
 * Set up the serialized member variables.
 */
void ActuatorSet::
setupSerializedMembers()
{
}

//_____________________________________________________________________________
/**
 * Construct the control maps.
 */
void ActuatorSet::
constructControlMaps()
{
	// ACTUATOR TO CONTROL
	int n = getSize();
	_actuatorToControl.setSize(n);
	int i,I,nx;
	Actuator *act;
	for(I=i=0;i<n;i++) {
		act = get(i);
		if(act==NULL) {
			_actuatorToControl[i] = -1;
		} else {
			nx = act->getNX();
			if(nx==0) {
				_actuatorToControl[i] = -1;
			} else {
				_actuatorToControl[i] = I;
				I += nx;
			}
		}
	}

	// DEBUG
	//cout<<"\n\nActuatorSet: ActuatorToControl Map:"<<endl;
	//cout<<_actuatorToControl<<endl;


	// CONTROL TO ACTUATOR
	int a,nxTotal = I;
	_controlToActuator.setSize(nxTotal);
	for(a=i=0,I=-1;i<nxTotal;i++) {
		for(;I<i;a++) {
			act = get(a);
			if(act==NULL) continue;
			I += act->getNX();
		}
		_controlToActuator[i] = a - 1;
	}

	// DEBUG
	//cout<<"\nActuatorSet: ControlToActuator Map:"<<endl;
	//cout<<_controlToActuator<<endl<<endl;
}
//_____________________________________________________________________________
/**
 * Construct the state maps.
 */
void ActuatorSet::
constructStateMaps()
{
	// ACTUATOR TO CONTROL
	int n = getSize();
	_actuatorToState.setSize(n);
	int i,I,ny;
	Actuator *act;
	for(I=i=0;i<n;i++) {
		act = get(i);
		if(act==NULL) {
			_actuatorToState[i] = -1;
		} else {
			ny = act->getNY();
			if(ny==0) {
				_actuatorToState[i] = -1;
			} else {
				_actuatorToState[i] = I;
				I += ny;
			}
		}
	}

	// DEBUG
	//cout<<"\n\nActuatorSet: ActuatorToState Map:"<<endl;
	//cout<<_actuatorToState<<endl;


	// CONTROL TO ACTUATOR
	int a,nyTotal = I;
	_stateToActuator.setSize(nyTotal);
	for(a=i=0,I=-1;i<nyTotal;i++) {
		for(;I<i;a++) {
			act = get(a);
			if(act==NULL) continue;
			I += act->getNY();
		}
		_stateToActuator[i] = a - 1;
	}

	// DEBUG
	//cout<<"\nActuatorSet: StateToActuator Map:"<<endl;
	//cout<<_stateToActuator<<endl<<endl;
}

//_____________________________________________________________________________
/**
 * Construct the pseudo-state maps.
 */
void ActuatorSet::
constructPseudoStateMaps()
{
	// ACTUATOR TO PSEUDO
	int n = getSize();
	_actuatorToPseudo.setSize(n);
	int i,I,nyp;
	Actuator *act;
	for(I=i=0;i<n;i++) {
		act = get(i);
		if(act==NULL) {
			_actuatorToPseudo[i] = -1;
		} else {
			nyp = act->getNYP();
			if(nyp==0) {
				_actuatorToPseudo[i] = -1;
			} else {
				_actuatorToPseudo[i] = I;
				I += nyp;
			}
		}
	}

	// DEBUG
	//cout<<"\n\nActuatorSet: ActuatorToPseudo Map:"<<endl;
	//cout<<_actuatorToPseudo<<endl;

	// PSEUDO TO ACTUATOR
	int a,nypTotal = I;
	_pseudoToActuator.setSize(nypTotal);
	for(a=i=0,I=-1;i<nypTotal;i++) {
		for(;I<i;a++) {
			act = get(a);
			if(act==NULL) continue;
			I += act->getNYP();
		}
		_pseudoToActuator[i] = a - 1;
	}

	// DEBUG
	//cout<<"\nActuatorSet: PseudoToActuator Map:"<<endl;
	//cout<<_pseudoToActuator<<endl<<endl;
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
ActuatorSet& ActuatorSet::
operator=(const ActuatorSet &aActuatorSet)
{
	// BASE CLASS
	Set<Actuator>::operator=(aActuatorSet);

	// MODEL
	_model = aActuatorSet.getModel();

	// MAPS
	_controlToActuator = aActuatorSet._controlToActuator;
	_actuatorToControl = aActuatorSet._actuatorToControl;

	_stateToActuator = aActuatorSet._stateToActuator;
	_actuatorToState = aActuatorSet._actuatorToState;

	_pseudoToActuator = aActuatorSet._pseudoToActuator;
	_actuatorToPseudo = aActuatorSet._actuatorToPseudo;

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
 * Set the model for which actuation is provided.
 *
 * @return Pointer to the model.
 */
void ActuatorSet::
setModel(Model *aModel)
{
	_model = aModel;

	// ACTUATORS
	int i;
	Actuator *act;
	for(i=0;i<getSize();i++) {
		act = get(i);
		if(act==NULL) continue;
		act->setModel(_model);
	}
}

//_____________________________________________________________________________
/**
 * Get a pointer to the model which is actuated.
 *
 * @return Pointer to the model.
 */
Model* ActuatorSet::
getModel() const
{
	return(_model);
}

//-----------------------------------------------------------------------------
// ACTUATOR
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Remove an actuator from the actuator set.
 *
 * @param aIndex Index of the actuator to be removed.
 * @return True if the remove was successful; false otherwise.
 */
bool ActuatorSet::
remove(int aIndex)
{
	bool success = Set<Actuator>::remove(aIndex);

	constructControlMaps();
	constructStateMaps();
	constructPseudoStateMaps();

	return(success);
}
//_____________________________________________________________________________
/**
 * Append an actuator on to the set.  A copy of the specified actuator
 * is not made.
 *
 * This method overrides the method in Set<Actuator> so that several
 * internal variables of the actuator set can be updated.
 *
 * @param aActuator Pointer to the actuator to be appended.
 * @return True if successful; false otherwise.
 */
bool ActuatorSet::
append(Actuator *aActuator)
{
	bool success = Set<Actuator>::append(aActuator);


	if(success) {
		// individual actuators keep pointers to model as well!! 
		aActuator->setModel(_model);

		constructControlMaps();
		constructStateMaps();
		constructPseudoStateMaps();
	}

	return(success);
}
//_____________________________________________________________________________
/**
 * Set the actuator at an index.  A copy of the specified actuator is NOT made.
 * The actuator previously set a the index is removed (and deleted).
 *
 * This method overrides the method in Set<Actuator> so that several
 * internal variables of the actuator set can be updated.
 *
 * @param aIndex Array index where the actuator is to be stored.  aIndex
 * should be in the range 0 <= aIndex <= getNumActuators();
 * @param aActuator Pointer to the actuator to be set.
 * @return True if successful; false otherwise.
 */
bool ActuatorSet::
set(int aIndex,Actuator *aActuator)
{
	bool success = Set<Actuator>::set(aIndex,aActuator);

	if(success) {
		constructControlMaps();
		constructStateMaps();
		constructPseudoStateMaps();
	}

	return(success);
}

//-----------------------------------------------------------------------------
// CONTROLS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the total number of actuator controls.
 *
 *	@return Total number of actuator controls.
 */
int ActuatorSet::
getNX() const
{
	return(_controlToActuator.getSize());
}
//_____________________________________________________________________________
/**
 * Map an actuator index to the index in the global array where that
 * actuator's controls begin.
 *
 * The controls for all the actuators of a model are held in an array
 * of values.  The convention used by ActuatorSet is that the controls for an
 * actuator are held sequentially in the controls array:
 *
 *	ControlsArray	Actuator   Actuator Control
 *		x[0]        0          0
 *		x[1]        0          1
 *		x[2]        0          2
 *		x[3]        1          0
 *		x[4]        3          0
 *		x[5]        3          1
 *		...         ...        ...
 *
 * Notice that not all actuators need have the same number of controls.  In the
 * above example, actuator 0 has 3 controls, actuator 1 has 1 control,
 * actuator 2 has 0 controls, and actuator 3 has 2 controls.
 *
 * This method returns the array index that marks the beginning of the controls
 * for a particular actuator.  In the event the actuator has no controls, -1
 * is returned.  For the above example, the following values of aActuatorIndex
 * would produce the following return values:
 *
 * aActuatorIndex		Return
 *		0					0
 *		1					3
 *		2					-1
 *		3					4
 *
 * @param aActuatorIndex The number (or index) of the actuator whose controls
 * index is desired.
 *	@return Array index that marks the beginning of the controls for the
 * specified actuator.  In the event the actuator has no controls, -1 is
 * returned.
 */
int ActuatorSet::
mapActuatorToControl(int aActuatorIndex) const
{
	if(aActuatorIndex<0) return(-1);
	if(aActuatorIndex>=getSize()) return(-1);
	return(_actuatorToControl[aActuatorIndex]);
}
//_____________________________________________________________________________
/**
 * Map a control, as indexed in the global array, to
 * a particular actuator.  Essentially, get the actuator to which a
 * control belongs.
 *
 * @param aControlIndex Index of a control in the global array.
 *	@return Index of the actuator to which the control belongs; -1 on
 * an error.
 */
int ActuatorSet::
mapControlToActuator(int aControlIndex) const
{
	if(aControlIndex<0) return(-1);
	if(aControlIndex>=getNX()) return(-1);
	return(_controlToActuator[aControlIndex]);
}

//_____________________________________________________________________________
/**
 * Get the index of the first control, as indexed in the global array,
 * that has a specified name.
 *
 * This method is costly to call, as the actuators are searched exhaustively
 * from the beginning of the set.
 *
 * @param aName Name of the desired control
 *	@return Index of the desired control; -1 on an error.
 */
int ActuatorSet::
getControlIndex(const string &aName) const
{
	Actuator *act;
	int i,localIndex,index;
	for(i=0;i<getSize();i++) {

		act = get(i);
		if(act==NULL) continue;

		try {
			localIndex = act->getControlIndex(aName);
			break;
		} catch(Exception x) {
			continue;
		}
	}

	index = mapActuatorToControl(i) + localIndex;

	return(index);
}
//_____________________________________________________________________________
/**
 * Get the name of a control.
 *
 * @param aIndex Index of the control whose name is desired.
 *	@return Name of the control.  If no name is found, an empty string "" is
 * returned.
 */
string ActuatorSet::
getControlName(int aIndex) const
{
	int a = mapControlToActuator(aIndex);
	Actuator *act = get(a);
	if(act==NULL) return("");

	int localIndex = aIndex - a; 
	string name;
	try {
		name = act->getControlName(localIndex);
	} catch(Exception x) {
		name = "";
	}

	return(name);
}

//_____________________________________________________________________________
/**
 * Set the value of a control.
 *
 *	@param aIndex Index of the control.
 * @param aValue Value to which to set the control.
 */
void ActuatorSet::
setControl(int aIndex,double aValue)
{
	int a = mapControlToActuator(aIndex);
	Actuator *act = get(a);
	if(act==NULL) return;

	int localIndex = aIndex - mapActuatorToControl(a);
	act->setControl(localIndex,aValue);
}
//_____________________________________________________________________________
/**
 * Set the value of a control.
 *
 *	@param aName Name of the desired control.
 * @param aValue Value to which to set the control.
 */
void ActuatorSet::
setControl(const string &aName,double aValue)
{
	int index = getControlIndex(aName);
	setControl(index,aValue);
}
//_____________________________________________________________________________
/**
 * Set the controls for all actuators in the model.  See getControlsIndex()
 * for a description of the layout of the controls array.
 *
 *	@param aX Array of controls for all actuators.
 * @see getControlsIndex()
 */
void ActuatorSet::
setControls(const double aX[])
{
	int a,I;
	Actuator *act;
	for(a=0;a<getSize();a++) {
		act = get(a);  if(act==NULL) continue;
		I = mapActuatorToControl(a); if(I<0) continue;
		act->setControls(&aX[I]);
	}
}

//_____________________________________________________________________________
/**
 * Get the value of a control.
 *
 *	@param aIndex Index of the control.
 * @return Value of the control.
 */
double ActuatorSet::
getControl(int aIndex) const
{
	int a = mapControlToActuator(aIndex);
	Actuator *act = get(a);
	if(act==NULL) return(rdMath::NAN);

	int localIndex = aIndex - mapActuatorToControl(a);
	return( act->getControl(localIndex) );
}
//_____________________________________________________________________________
/**
 * Get the value of a control.
 *
 *	@param aName Name of the desired control.
 * @param Value of the control.
 */
double ActuatorSet::
getControl(const string &aName) const
{
	int index = getControlIndex(aName);
	return( getControl(index) );
}
//_____________________________________________________________________________
/**
 * Get the controls for all actuators in the model.  See getControlsIndex()
 * for a description of the assumed layout of the controls array.
 *
 *	@param rX Array of controls for all actuators.
 * @see getControlsIndex()
 */
void ActuatorSet::
getControls(double rX[]) const
{
	int i,I;
	Actuator *act;
	for(i=0;i<getSize();i++) {
		act = get(i);  if(act==NULL) continue;
		I = mapActuatorToControl(i); if(I<0) continue;
		act->getControls(&rX[I]);
	}
}

//-----------------------------------------------------------------------------
// STATES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the total number of actuator states.
 *	@return Total number of actuator states.
 */
int ActuatorSet::
getNY() const
{
	return(_stateToActuator.getSize());
}
//_____________________________________________________________________________
/**
 * Map an actuator index to the index in the global array where that
 * actuator's states begin.
 *
 * The states that characterize all the actuators of a model are held by
 * the model in an array of values.  The convention used by ActuatorSet is that
 * the states for an actuator are held in this array sequentially:
 *
 *	StatesArray    Actuator    Actuator State
 *		y[iAct+0]   0           0
 *		y[iAct+1]   0           1
 *		y[iAct+2]   0           2
 *		y[iAct+3]   1           0
 *		y[iAct+4]   3           0
 *		y[iAct+5]   3           1
 *		...         ...         ...
 *
 * iact is the index in the states array where the states start.
 * 
 * Notice that not all actuators need have the same number of states.  In the
 * above example, actuator 0 has 3 states, actuator 1 has 1 state, actuator 2
 * has 0 states, and actuator 3 has 2 states.
 *
 * Note that it is assumed that the indeces are computed as though
 * the actuator states start at the beginning of the states array.  In actuality
 * the actuator states may start in the middle of some larger states array.
 * The index iAct above indexes where in the state array y, the actuator states
 * begin.  The caller should account for this as necessary.
 *
 * This method returns the array index that marks the beginning of the states
 * for a particular actuator.  In the event the actuator has no states, -1
 * is returned.  For the above example, the following values of aActuatorIndex
 * would produce the following return values:
 *
 * aActuatorIndex		Return
 *		0					0
 *		1					3
 *		2					-1
 *		3					4
 *
 * @param aActuatorIndex The number (or index) of the actuator whose state index
 * is desired.
 *	@return Array index that marks the beginning of the states for the
 * specified actuator.  In the event the actuator has no states, -1 is returned.
 */
int ActuatorSet::
mapActuatorToState(int aActuatorIndex) const
{
	if(aActuatorIndex<0) return(-1);
	if(aActuatorIndex>=getSize()) return(-1);
	return( _actuatorToState[aActuatorIndex] );
}
//_____________________________________________________________________________
/**
 * Map a state, as indexed in the global array, to
 * a particular actuator.  Essentially, get the actuator to which a
 * state belongs.
 *
 * @param aStateIndex Index of a state in the global array.
 *	@return Index of the actuator to which the state belongs; -1 on
 * an error.
 */
int ActuatorSet::
mapStateToActuator(int aStateIndex) const
{
	if(aStateIndex<0) return(-1);
	if(aStateIndex>=getNY()) return(-1);
	return(_stateToActuator[aStateIndex]);
}

//_____________________________________________________________________________
/**
 * Get the index of the first state, as indexed in the global array,
 * that has a specified name.
 *
 * This method is costly to call, as the actuators are searched exhaustively
 * from the beginning of the set.
 *
 * @param aName Name of the desired state
 *	@return Index of the desired state; -1 on an error.
 */
int ActuatorSet::
getStateIndex(const string &aName) const
{
	Actuator *act;
	int i,localIndex,index;
	for(i=0;i<getSize();i++) {

		act = get(i);
		if(act==NULL) continue;

		try {
			localIndex = act->getStateIndex(aName);
			break;
		} catch(Exception x) {
			continue;
		}
	}

	index = mapActuatorToState(i) + localIndex;

	return(index);
}
//_____________________________________________________________________________
/**
 * Get the name of a state.
 *
 * @param aIndex Index of the state whose name is desired.
 *	@return Name of the state.  If no name is found, an empty string "" is
 * returned.
 */
string ActuatorSet::
getStateName(int aIndex) const
{
	int a = mapStateToActuator(aIndex);
	Actuator *act = get(a);
	if(act==NULL) return("");

	int localIndex = aIndex - a; 
	string name;
	try {
		name = act->getStateName(localIndex);
	} catch(Exception x) {
		name = "";
	}

	return(name);
}

//_____________________________________________________________________________
/**
 * Set the value of a state.
 *
 *	@param aIndex Index of the state.
 * @param aValue Value to which to set the state.
 */
void ActuatorSet::
setState(int aIndex,double aValue)
{
	int a = mapStateToActuator(aIndex);
	Actuator *act = get(a);
	if(act==NULL) return;

	int localIndex = aIndex - mapActuatorToState(a);
	act->setState(localIndex,aValue);
}
//_____________________________________________________________________________
/**
 * Set the value of a state.
 *
 *	@param aName Name of the desired state.
 * @param aValue Value to which to set the state.
 */
void ActuatorSet::
setState(const string &aName,double aValue)
{
	int index = getStateIndex(aName);
	setState(index,aValue);
}
//_____________________________________________________________________________
/**
 * Set the states for all actuators in the model.  See getStatesIndex() for a
 * description of the assumed layout of the states array.
 *
 *	@param aY Array of states for all actuators.
 * @see getStatesIndex()
 */
void ActuatorSet::
setStates(const double aY[])
{
	int i,I;
	Actuator *act;
	for(i=0;i<getSize();i++) {
		act = get(i);  if(act==NULL) continue;
		I = mapActuatorToState(i); if(I<0) continue;
		act->setStates(&aY[I]);
	}
}

//_____________________________________________________________________________
/**
 * Get the value of a state.
 *
 *	@param aIndex Index of the state.
 * @return Value of the state.
 */
double ActuatorSet::
getState(int aIndex) const
{
	int a = mapStateToActuator(aIndex);
	Actuator *act = get(a);
	if(act==NULL) return(rdMath::NAN);

	int localIndex = aIndex - mapActuatorToState(a);
	return( act->getState(localIndex) );
}
//_____________________________________________________________________________
/**
 * Get the value of a state.
 *
 *	@param aName Name of the desired state.
 * @param Value of the state.
 */
double ActuatorSet::
getState(const string &aName) const
{
	int index = getStateIndex(aName);
	return( getState(index) );
}
//_____________________________________________________________________________
/**
 * Get the states for all actuators in the model.  See getStatesIndex()
 * for a description of the assumed layout of the states array.
 *
 *	@param rY Array of states for all actuators.
 * @see getStatesIndex()
 */
void ActuatorSet::
getStates(double rY[]) const
{
	int i,I;
	Actuator *act;
	for(i=0;i<getSize();i++) {
		act = get(i);  if(act==NULL) continue;
		I = mapActuatorToState(i); if(I<0) continue;
		act->getStates(&rY[I]);
	}
}

//-----------------------------------------------------------------------------
// PSEUDOSTATES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the total number of actuator pseudostates.
 *	@return Total number of actuator pseudostates.
 */
int ActuatorSet::
getNYP() const
{
	return(_pseudoToActuator.getSize());
}
//_____________________________________________________________________________
/**
 * Map an actuator index to the index in the global array where that
 * actuator's pseudo-states begin.
 *
 * The pseudo-states that characterize all the actuators of a model are held by
 * the model in an array of values.  The convention used by ActuatorSet is that
 * the states for an actuator are held in this array sequentially:
 *
 *	StatesArray    Actuator    Actuator Pseudo-State
 *		yp[iAct+0]   0           0
 *		yp[iAct+1]   0           1
 *		yp[iAct+2]   0           2
 *		yp[iAct+3]   1           0
 *		yp[iAct+4]   3           0
 *		yp[iAct+5]   3           1
 *		...         ...         ...
 *
 * iact is the index in the global array where the pseudo-states
 * start.
 * 
 * Notice that not all actuators need have the same number of states.  In the
 * above example, actuator 0 has 3 states, actuator 1 has 1 state, actuator 2
 * has 0 states, and actuator 3 has 2 states.
 *
 * Note that it is assumed that the indeces are computed as though
 * the actuator states start at the beginning of the states array.  In actuality
 * the actuator states may start in the middle of some larger states array.
 * The index iAct above indexes where in the state array y, the actuator states
 * begin.  The caller should account for this as necessary.
 *
 * This method returns the array index that marks the beginning of the states
 * for a particular actuator.  In the event the actuator has no states, -1
 * is returned.  For the above example, the following values of aActuatorIndex
 * would produce the following return values:
 *
 * aActuatorIndex		Return
 *		0					0
 *		1					3
 *		2					-1
 *		3					4
 *
 * @param aActuatorIndex The number (or index) of the actuator whose state index
 * is desired.
 *	@return Array index that marks the beginning of the states for the
 * specified actuator.  In the event the actuator has no states, -1 is returned.
 */
int ActuatorSet::
mapActuatorToPseudoState(int aActuatorIndex) const
{
	if(aActuatorIndex<0) return(-1);
	if(aActuatorIndex>=getSize()) return(-1);
	return( _actuatorToPseudo[aActuatorIndex] );
}
//_____________________________________________________________________________
/**
 * Map a pseudo-state, as indexed in the global array, to
 * a particular actuator.  Essentially, get the actuator to which a
 * pseudo-state belongs.
 *
 * @param aPseudoStateIndex Index of a pseudo-state in the global array.
 *	@return Index of the actuator to which the pseudo-state belongs; -1 on
 * an error.
 */
int ActuatorSet::
mapPseudoStateToActuator(int aPseudoStateIndex) const
{
	if(aPseudoStateIndex<0) return(-1);
	if(aPseudoStateIndex>=getNYP()) return(-1);
	return(_pseudoToActuator[aPseudoStateIndex]);
}

//_____________________________________________________________________________
/**
 * Get the index of the first pseudo-state, as indexed in the global array,
 * that has a specified name.
 *
 * This method is costly, as the actuators are searched exhaustively
 * from the beginning of the set.
 *
 * @param aName Name of the desired pseudo-state
 *	@return Index of the desired pseudo-state; -1 on an error.
 */
int ActuatorSet::
getPseudoStateIndex(const string &aName) const
{
	Actuator *act;
	int i,localIndex,index;
	for(i=0;i<getSize();i++) {

		act = get(i);
		if(act==NULL) continue;

		try {
			localIndex = act->getPseudoStateIndex(aName);
			break;
		} catch(Exception x) {
			continue;
		}
	}

	index = mapActuatorToPseudoState(i) + localIndex;

	return(index);
}
//_____________________________________________________________________________
/**
 * Get the name of a pseudo-state.
 *
 * @param aIndex Index of the pseudo-state whose name is desired.
 *	@return Name of the pseudo-state.  If no name is found, an empty string ""
 * is returned.
 */
string ActuatorSet::
getPseudoStateName(int aIndex) const
{
	int a = mapPseudoStateToActuator(aIndex);
	Actuator *act = get(a);
	if(act==NULL) return("");

	int baseIndex = mapActuatorToPseudoState(a);

	int localIndex = aIndex - baseIndex; 
	string name;
	try {
		name = act->getPseudoStateName(localIndex);
	} catch(Exception x) {
		name = "";
	}

	return(name);
}

//_____________________________________________________________________________
/**
 * Set the value of a pseudo-state.
 *
 *	@param aIndex Index of the pseudo-state.
 * @param aValue Value to which to set the pseudo-state.
 */
void ActuatorSet::
setPseudoState(int aIndex,double aValue)
{
	int a = mapPseudoStateToActuator(aIndex);
	Actuator *act = get(a);
	if(act==NULL) return;

	int localIndex = aIndex - mapActuatorToPseudoState(a);
	act->setPseudoState(localIndex,aValue);
}
//_____________________________________________________________________________
/**
 * Set the value of a pseudo-state.
 *
 *	@param aName Name of the desired pseudo-state.
 * @param aValue Value to which to set the pseudo-state.
 */
void ActuatorSet::
setPseudoState(const string &aName,double aValue)
{
	int index = getPseudoStateIndex(aName);
	setPseudoState(index,aValue);
}
//_____________________________________________________________________________
/**
 * Set the pseudostates for all actuators in the model.  See
 * getPseudoStatesIndex() for a description of the assumed layout of the
 * pseudostates array.
 *
 *	@param aYP Array of pseudostates for all actuators.  The first element of
 * aYP is assumed to be the beginning of the pseudo states for the actuators.
 * @see getPseudoStatesIndex()
 */
void ActuatorSet::
setPseudoStates(const double aYP[])
{
	int i,I;
	Actuator *act;
	for(i=0;i<getSize();i++) {
		act = get(i);  if(act==NULL) continue;
		I = mapActuatorToPseudoState(i); if(I<0) continue;
		act->setPseudoStates(&aYP[I]);
	}
}

//_____________________________________________________________________________
/**
 * Get the value of a pseudo-state.
 *
 *	@param aIndex Index of the pseudo-state.
 * @return Value of the pseudo-state.
 */
double ActuatorSet::
getPseudoState(int aIndex) const
{
	int a = mapPseudoStateToActuator(aIndex);
	Actuator *act = get(a);
	if(act==NULL) return(rdMath::NAN);

	int localIndex = aIndex - mapActuatorToPseudoState(a);
	return( act->getPseudoState(localIndex) );
}
//_____________________________________________________________________________
/**
 * Get the value of a pseudo-state.
 *
 *	@param aName Name of the desired pseudo-state.
 * @param Value of the pseudo-state.
 */
double ActuatorSet::
getPseudoState(const string &aName) const
{
	int index = getPseudoStateIndex(aName);
	return( getPseudoState(index) );
}
//_____________________________________________________________________________
/**
 * Get the pseudostates for all actuators in the model.  See
 * getPseudoStatesIndex() for a description of the assumed layout of the
 * pseudostates array.
 *
 *	@param rY Array of pseudostates for all actuators.  The first element of
 * aYP is assumed to be the beginning of the pseudo states for the actuators.
 * @see getPseudoStatesIndex()
 */
void ActuatorSet::
getPseudoStates(double rYP[]) const
{
	int i,I;
	Actuator *act;
	for(i=0;i<getSize();i++) {
		act = get(i);  if(act==NULL) continue;
		I = mapActuatorToPseudoState(i); if(I<0) continue;
		act->getPseudoStates(&rYP[I]);
	}
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
void ActuatorSet::
promoteControlsToStates(const double aX[],double aDT)
{
	int i,I;
	Actuator *act;
	for(i=0;i<getSize();i++) {
		act = get(i);  if(act==NULL) continue;
		I = mapActuatorToControl(i); if(I<0) continue;
		act->promoteControlsToStates(&aX[I],aDT);
	}
}

//_____________________________________________________________________________
/**
 * For each actuator, compute all quantities necessary for actuating the model.
 */
void ActuatorSet::
computeActuation()
{
	int i;
	Actuator *act;
	for(i=0;i<getSize();i++) {
		act = get(i);
		if(act!=NULL) act->computeActuation();
	}
}

//_____________________________________________________________________________
/**
 * Compute the time derivatives of the states that characterize the actuators.
 *
 * @param rDY Array of state derivatives.
 */
void ActuatorSet::
computeStateDerivatives(double rDY[])
{
	int i,I;
	Actuator *act;
	for(i=0;i<getSize();i++) {
		act = get(i);
		I = mapActuatorToState(i);
		if(act!=NULL) act->computeStateDerivatives(&rDY[I]);
	}
}

//_____________________________________________________________________________
/**
 * Update the pseudostates of all actuators.  Pseudostates are quantities
 * that are not integrated but that depend on the time history of a
 * simulation (e.g., spring set points).
 */
void ActuatorSet::
updatePseudoStates()
{
	int i;
	Actuator *act;
	for(i=0;i<getSize();i++) {
		act = get(i);
		if(act!=NULL) act->updatePseudoStates();
	}
}


//=============================================================================
// APPLICATION
//=============================================================================
//_____________________________________________________________________________
/**
 * For each actuator, apply the force(s) (or torque(s)) to the model.
 */
void ActuatorSet::
apply()
{
	int i;
	Actuator *act;
	for(i=0;i<getSize();i++) {
		act = get(i);
		if(act!=NULL) act->apply();
	}
}


//=============================================================================
// CHECK
//=============================================================================
//_____________________________________________________________________________
/**
 * Check that all actuators are valid.
 */
bool ActuatorSet::
check() const
{
	bool status=true;

	// LOOP THROUGH ACTUATORS
	int i;
	Actuator *act;
	for(i=0;i<getSize();i++) {
		act = get(i);
		if(act==NULL) continue;
		if(!act->check()) status = false;
	}

	return(status);
}

