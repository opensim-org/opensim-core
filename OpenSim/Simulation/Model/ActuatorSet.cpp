// ActuatorSet.cpp
// Authors: Frank C. Anderson, Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include "ActuatorSet.h"
#include "Model.h"
#include <OpenSim/Common/PropertyObjArray.h>
#include <OpenSim/Common/rdMath.h>

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
ActuatorSet::~ActuatorSet()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ActuatorSet::ActuatorSet() :
	_model(NULL),
	_controlToActuator(-1), _actuatorToControl(-1), 
	_stateToActuator(-1), _actuatorToState(-1), 
	_pseudoToActuator(-1), _actuatorToPseudo(-1)
{
	setNull();

	// INDICES
	constructMaps();
}

//_____________________________________________________________________________
/**
 * Construct an actuator set from file.
 *
 * @param aFileName Name of the file.
 */
ActuatorSet::ActuatorSet(const std::string &aFileName, bool aUpdateFromXMLNode) :
	Set<AbstractActuator>(aFileName, false),
	_model(NULL),
	_controlToActuator(-1), _actuatorToControl(-1),
	_stateToActuator(-1), _actuatorToState(-1),
	_pseudoToActuator(-1), _actuatorToPseudo(-1)
{
	setNull();

	if(aUpdateFromXMLNode) updateFromXMLNode();
	// removeInvalidObjects();

	// INDICES
	constructMaps();
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aActuatorSet ActuatorSet to be copied.
 */
ActuatorSet::ActuatorSet(const ActuatorSet &aActuatorSet) :
	Set<AbstractActuator>(aActuatorSet),
	_model(NULL),
	_controlToActuator(-1), _actuatorToControl(-1),
	_stateToActuator(-1), _actuatorToState(-1),
	_pseudoToActuator(-1), _actuatorToPseudo(-1)
{
	setNull();

	// Class Members
	copyData(aActuatorSet);

	// Indices
	constructMaps();
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this ActuatorSet to their null values.
 */
void ActuatorSet::setNull()
{
	// TYPE
	setType("ActuatorSet");
	// NAME
	//setName("ActuatorSet");

	// PROPERTIES
	setupSerializedMembers();

	// MODEL
	_model = NULL;
	_computeActuationEnabled = true;
}

//_____________________________________________________________________________
/**
 * Copy this ActuatorSet and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this ActuatorSet.
 */
Object* ActuatorSet::copy() const
{
	ActuatorSet *actSet = new ActuatorSet(*this);
	return(actSet);
}

//_____________________________________________________________________________
/**
 * Copy the member variables of the ActuatorSet.
 *
 * @param aAbsActuatorSet actuator set to be copied
 */
void ActuatorSet::copyData(const ActuatorSet &aAbsActuatorSet)
{
	// MODEL
	_model = aAbsActuatorSet.getModel();

	// MAPS
	_controlToActuator = aAbsActuatorSet._controlToActuator;
	_actuatorToControl = aAbsActuatorSet._actuatorToControl;

	_stateToActuator = aAbsActuatorSet._stateToActuator;
	_actuatorToState = aAbsActuatorSet._actuatorToState;

	_pseudoToActuator = aAbsActuatorSet._pseudoToActuator;
	_actuatorToPseudo = aAbsActuatorSet._actuatorToPseudo;
}

//_____________________________________________________________________________
/**
 * Set up the serialized member variables.
 */
void ActuatorSet::
setupSerializedMembers()
{
}

void ActuatorSet::setup(Model* aModel)
{
	// BASE CLASS
	Set<AbstractActuator>::setup();

	_model = aModel;
	// INDICES
	constructMaps();
	for (int i = 0; i < getSize(); i++)
		get(i)->setup(aModel);
}
//_____________________________________________________________________________
/**
 * Update the geometry of the muscles based on coordinate changes
 */
void ActuatorSet::updateGeometry()
{
	int i;
	for (i = 0; i < getSize(); i++) {
		AbstractActuator* act = get(i);
		act->updateGeometry();
	}
}

//_____________________________________________________________________________
/**
 * Construct a generic map.  This is a utility function used by constructMaps to create the maps for
 * controls, states, and pseudo states.  Here "values" refers to controls/states/pseudo states.
 */
void ActuatorSet::
constructMap(const Array<int> &numValuesPerActuator, Array<int> &rActuatorToValue, Array<int> &rValueToActuator)
{
	int numActuators = getSize();
	rActuatorToValue.setSize(numActuators);
	int totalValues = 0;
	for(int i=0;i<numActuators;i++) totalValues += numValuesPerActuator[i];
	rValueToActuator.setSize(totalValues);
	int count = 0;
	for(int i=0;i<numActuators;i++) {
		int n = numValuesPerActuator[i];
		if(n==0) rActuatorToValue[i] = -1;
		else {
			rActuatorToValue[i] = count;
			for(int j=0;j<n;j++) rValueToActuator[count+j] = i;
			count += n;
		}
	}
}

//_____________________________________________________________________________
/**
 * Construct control, state, and pseudo state maps
 */
void ActuatorSet::
constructMaps()
{
	Array<int> numControls(0,getSize());
	Array<int> numStates(0,getSize());
	Array<int> numPseudoStates(0,getSize());

	for(int i=0;i<getSize();i++) {
		numControls[i] = get(i)->getNumControls();
		numStates[i] = get(i)->getNumStates();
		numPseudoStates[i] = get(i)->getNumPseudoStates();
	}

	constructMap(numControls, _actuatorToControl, _controlToActuator);
	constructMap(numStates, _actuatorToState, _stateToActuator);
	constructMap(numPseudoStates, _actuatorToPseudo, _pseudoToActuator);

	// DEBUG
	//cout<<"\n\nActuatorSet: ActuatorToControl Map:\n"<<_actuatorToControl<<endl;
	//cout<<"\nActuatorSet: ControlToActuator Map:\n"<<_controlToActuator<<endl<<endl;
	//cout<<"\n\nActuatorSet: ActuatorToState Map:\n"<<_actuatorToState<<endl;
	//cout<<"\nActuatorSet: StateToActuator Map:\n"<<_stateToActuator<<endl<<endl;
	//cout<<"\n\nActuatorSet: ActuatorToPseudo Map:\n"<<_actuatorToPseudo<<endl;
	//cout<<"\nActuatorSet: PseudoToActuator Map:\n"<<_pseudoToActuator<<endl<<endl;
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
operator=(const ActuatorSet &aAbsActuatorSet)
{
	// BASE CLASS
	Set<AbstractActuator>::operator=(aAbsActuatorSet);

	// Class Members
	copyData(aAbsActuatorSet);

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
	AbstractActuator *act;
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
	bool success = Set<AbstractActuator>::remove(aIndex);

	constructMaps();

	return(success);
}
//_____________________________________________________________________________
/**
 * Append an actuator on to the set.  A copy of the specified actuator
 * is not made.
 *
 * This method overrides the method in Set<AbstractActuator> so that several
 * internal variables of the actuator set can be updated.
 *
 * @param aActuator Pointer to the actuator to be appended.
 * @return True if successful; false otherwise.
 */
bool ActuatorSet::
append(AbstractActuator *aActuator)
{
	bool success = Set<AbstractActuator>::append(aActuator);


	if(success) {
		// individual actuators keep pointers to model as well!! 
		aActuator->setModel(_model);

		constructMaps();
	}

	return(success);
}
//_____________________________________________________________________________
/**
 * Append actuators from an actuator set to this set.  Copies of the actuators are not made.
 *
 * This method overrides the method in Set<AbstractActuator> so that several
 * internal variables of the actuator set can be updated.
 *
 * @param aActuatorSet The set of actuators to be appended.
 * @param aAllowDuplicateNames If true, all actuators will be appended; If false, don't append actuators whose
 * name already exists in this model's actuator set.
 * @return True if successful; false otherwise.
 */
bool ActuatorSet::
append(ActuatorSet &aActuatorSet, bool aAllowDuplicateNames)
{
	bool success = true;
	for(int i=0;i<aActuatorSet.getSize() && success;i++) {
		bool nameExists = false;
		if(!aAllowDuplicateNames) {
			std::string name = aActuatorSet.get(i)->getName();
			for(int j=0;j<getSize();j++) {
				if(get(j)->getName() == name) {
					nameExists = true;
					break;
				}
			}
		}
		if(!nameExists) {
			success = Set<AbstractActuator>::append(aActuatorSet.get(i));
			// individual actuators keep pointers to model as well!! 
			aActuatorSet.get(i)->setModel(_model);
		}
	}

	if(success) {
		constructMaps();
	}

	return(success);
}
//_____________________________________________________________________________
/**
 * Set the actuator at an index.  A copy of the specified actuator is NOT made.
 * The actuator previously set a the index is removed (and deleted).
 *
 * This method overrides the method in Set<AbstractActuator> so that several
 * internal variables of the actuator set can be updated.
 *
 * @param aIndex Array index where the actuator is to be stored.  aIndex
 * should be in the range 0 <= aIndex <= getNumActuators();
 * @param aActuator Pointer to the actuator to be set.
 * @return True if successful; false otherwise.
 */
bool ActuatorSet::
set(int aIndex,AbstractActuator *aActuator)
{
	bool success = Set<AbstractActuator>::set(aIndex,aActuator);

	if(success) {
		constructMaps();
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
getNumControls() const
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
	if(aControlIndex>=getNumControls()) return(-1);
	return(_controlToActuator[aControlIndex]);
}
AbstractActuator *ActuatorSet::
mapControlToActuator(int aControlIndex, int &rLocalIndex) const
{
	int index=mapControlToActuator(aControlIndex);
	AbstractActuator *act=get(index);
	if(act) rLocalIndex = aControlIndex - mapActuatorToControl(index);
	return act;
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
	AbstractActuator *act;
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
	int localIndex;
	AbstractActuator *act = mapControlToActuator(aIndex,localIndex);
	if(!act) return "";
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
	int localIndex;
	AbstractActuator *act = mapControlToActuator(aIndex,localIndex);
	if(act) act->setControl(localIndex,aValue);
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
	AbstractActuator *act;
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
	int localIndex;
	AbstractActuator *act = mapControlToActuator(aIndex,localIndex);
	if(act) return act->getControl(localIndex);
	else return rdMath::NAN;
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
	AbstractActuator *act;
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
getNumStates() const
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
	if(aStateIndex>=getNumStates()) return(-1);
	return(_stateToActuator[aStateIndex]);
}
AbstractActuator *ActuatorSet::
mapStateToActuator(int aStateIndex, int &rLocalIndex) const
{
	int index=mapStateToActuator(aStateIndex);
	AbstractActuator *act=get(index);
	if(act) rLocalIndex = aStateIndex - mapActuatorToState(index);
	return act;
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
	AbstractActuator *act;
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
	int localIndex;
	AbstractActuator *act = mapStateToActuator(aIndex,localIndex);
	if(!act) return "";
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
 * Get the names of the states of the actuators.
 *
 * @param rNames Array of names.
 */
void ActuatorSet::
getStateNames(Array<std::string> &rNames) const
{
	for(int i=0;i<getNumStates();i++) {
		int localIndex;
		AbstractActuator *act = mapStateToActuator(i,localIndex);
		if(act) rNames.append(act->getStateName(localIndex));
	}
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
	int localIndex;
	AbstractActuator *act = mapStateToActuator(aIndex,localIndex);
	if(act) act->setState(localIndex,aValue);
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
	AbstractActuator *act;
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
	int localIndex;
	AbstractActuator *act = mapStateToActuator(aIndex,localIndex);
	if(act) return act->getState(localIndex);
	else return rdMath::NAN;
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
	AbstractActuator *act;
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
getNumPseudoStates() const
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
	if(aPseudoStateIndex>=getNumPseudoStates()) return(-1);
	return(_pseudoToActuator[aPseudoStateIndex]);
}
AbstractActuator *ActuatorSet::
mapPseudoStateToActuator(int aPseudoStateIndex, int &rLocalIndex) const
{
	int index=mapPseudoStateToActuator(aPseudoStateIndex);
	AbstractActuator *act=get(index);
	if(act) rLocalIndex = aPseudoStateIndex - mapActuatorToPseudoState(index);
	return act;
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
	AbstractActuator *act;
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
	int localIndex;
	AbstractActuator *act = mapPseudoStateToActuator(aIndex,localIndex);
	if(!act) return "";
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
	int localIndex;
	AbstractActuator *act = mapPseudoStateToActuator(aIndex,localIndex);
	if(act) act->setPseudoState(localIndex,aValue);
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
	AbstractActuator *act;
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
	int localIndex;
	AbstractActuator *act = mapPseudoStateToActuator(aIndex,localIndex);
	if(act) return act->getPseudoState(localIndex);
	else return rdMath::NAN;
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
	AbstractActuator *act;
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
 * If computeActuation is disabled, computeActuation() does nothing
 * and computeStateDerivatives returns all state derivatives as zero.
 */
void ActuatorSet::
setComputeActuationEnabled(bool aTrueFalse)
{
	_computeActuationEnabled = aTrueFalse;
}
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
	AbstractActuator *act;
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
	if(!_computeActuationEnabled) return;
	int i;
	AbstractActuator *act;
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
	if(!_computeActuationEnabled) {
		memset(rDY,0,getNumStates()*sizeof(double));
	  	return;
	}
	int i,I;
	AbstractActuator *act;
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
	AbstractActuator *act;
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
	AbstractActuator *act;
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
	AbstractActuator *act;
	for(i=0;i<getSize();i++) {
		act = get(i);
		if(act==NULL) continue;
		if(!act->check()) status = false;
	}

	return(status);
}

//=============================================================================
// PETETEST
//=============================================================================
//_____________________________________________________________________________
/**
 * Print the actuatorset for debugging purposes.
 */
#include "AbstractMuscle.h"

void ActuatorSet::peteTest() const
{
	int i;

	if (_objectGroups.getSize() == 0) {
		cout << "The actuatorSet contains no groups." << endl;
	} else {
	   for(i=0;i<_objectGroups.getSize();i++) {
		   _objectGroups.get(i)->peteTest();
	   }
	}

	for(i=0;i<getSize();i++) {
		AbstractMuscle *ms = dynamic_cast<AbstractMuscle*>(get(i));
		if (ms)
			ms->peteTest();
	}
}
