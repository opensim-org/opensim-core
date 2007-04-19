// Model.cpp
// Authors: Frank C. Anderson, Peter Loan, Ayman Habib
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
#include <string>
#include <math.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/IO.h>
#include "Model.h"
#include "AbstractMuscle.h"
#include "CoordinateSet.h"
#include "SpeedSet.h"
#include "BodySet.h"
#include "IntegCallback.h"
#include "IntegCallbackSet.h"
#include "DerivCallback.h"
#include "DerivCallbackSet.h"

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
 * Default constructor.
 */
Model::Model() :
	_fileName("Unassigned"),
	_lengthUnitsStr(_lengthUnitsStrProp.getValueStr()),
	_forceUnitsStr(_forceUnitsStrProp.getValueStr()),
	_dynamicsEngine(_dynamicsEngineProp.getValueObjPtrRef()),
	_actuatorSetProp(PropertyObj("", ActuatorSet())),
	_actuatorSet((ActuatorSet&)_actuatorSetProp.getValueObj()),
	_contactSetProp(PropertyObj("", ContactForceSet())),
	_contactSet((ContactForceSet&)_contactSetProp.getValueObj())
{
	setNull();
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Constructor from an XML file
 */
Model::Model(const string &aFileName) :
	Object(aFileName, false),
	_fileName("Unassigned"),
	_lengthUnitsStr(_lengthUnitsStrProp.getValueStr()),
	_forceUnitsStr(_forceUnitsStrProp.getValueStr()),
	_dynamicsEngine(_dynamicsEngineProp.getValueObjPtrRef()),
	_actuatorSetProp(PropertyObj("", ActuatorSet())),
	_actuatorSet((ActuatorSet&)_actuatorSetProp.getValueObj()),
	_contactSetProp(PropertyObj("", ContactForceSet())),
	_contactSet((ContactForceSet&)_contactSetProp.getValueObj())
{
	setNull();
	setupProperties();

	// Do the maneuver to change then restore working directory 
	// so that the parsing code behaves properly if called from a different directory.
	string saveWorkingDirectory = IO::getCwd();
	string directoryOfSetupFile = IO::getParentDirectory(aFileName);
	IO::chDir(directoryOfSetupFile);
	updateFromXMLNode();
	IO::chDir(saveWorkingDirectory);

	_fileName = aFileName;
	// Throw exception if something wrong happened and we don't have a dynamics engine.
	// likely due to pickng wrong file that has no valid OpenSim model
	if (!hasDynamicsEngine()){
		throw Exception("No dynamics engine in model's XML file "+aFileName+". Model will not be created");
	}
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aModel Model to be copied.
 */

Model::Model(const Model &aModel) :
   Object(aModel),
	_lengthUnitsStr(_lengthUnitsStrProp.getValueStr()),
	_forceUnitsStr(_forceUnitsStrProp.getValueStr()),
	_dynamicsEngine(_dynamicsEngineProp.getValueObjPtrRef()),
	_actuatorSetProp(PropertyObj("", ActuatorSet())),
	_actuatorSet((ActuatorSet&)_actuatorSetProp.getValueObj()),
	_contactSetProp(PropertyObj("", ContactForceSet())),
	_contactSet((ContactForceSet&)_contactSetProp.getValueObj())
{
	setNull();
	setupProperties();
	copyData(aModel);
}
//_____________________________________________________________________________
/**
 * Destructor.
 */
Model::~Model()
{
	if (_analysisSet) {
		delete _analysisSet;
		_analysisSet = NULL;
	}

	if (_integCallbackSet) {
		delete _integCallbackSet;
		_integCallbackSet = NULL;
	}

	if (_derivCallbackSet) {
		delete _derivCallbackSet;
		_derivCallbackSet = NULL;
	}
}
//_____________________________________________________________________________
/**
 * Copy this Model and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this Model.
 */
Object* Model::copy() const
{
	Model *model = new Model(*this);
	return(model);
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy the member variables of the model.
 *
 * @param aModel model to be copied
 */
void Model::copyData(const Model &aModel)
{
	_fileName = aModel._fileName;
	_lengthUnits = aModel._lengthUnits;
	_forceUnits = aModel._forceUnits;
	_lengthUnitsStr = aModel._lengthUnitsStr;
	_forceUnitsStr = aModel._forceUnitsStr;
	_dynamicsEngine = (AbstractDynamicsEngine*)Object::SafeCopy(aModel._dynamicsEngine);
	_actuatorSet = aModel._actuatorSet;
	_muscleGroups = aModel._muscleGroups;
	_contactSet = aModel._contactSet;
	_integCallbackSet = aModel._integCallbackSet;
	_derivCallbackSet = aModel._derivCallbackSet;
	_analysisSet = aModel._analysisSet;
	_time = aModel._time;
	_tNormConst = aModel._tNormConst;
	_yi = aModel._yi;
	_ypi = aModel._ypi;
	_builtOK = aModel._builtOK; //??
}
//_____________________________________________________________________________
/**
 * Set the values of all data members to an appropriate "null" value.
 */
void Model::setNull()
{
	setType("Model");

	_analysisSet = NULL;
	_integCallbackSet = NULL;
	_derivCallbackSet = NULL;
	_time = 0.0;
	_tNormConst = 1.0;
	_builtOK = false;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 *
 */
void Model::setupProperties()
{
	_contactSetProp.setName("ContactForceSet");
	_propertySet.append(&_contactSetProp);

	_dynamicsEngineProp.setName("DynamicsEngine");
	_propertySet.append(&_dynamicsEngineProp);

	_actuatorSetProp.setName("ActuatorSet");
	_propertySet.append(&_actuatorSetProp);

	_lengthUnitsStrProp.setName("length_units");
	_propertySet.append(&_lengthUnitsStrProp);

	_forceUnitsStrProp.setName("force_units");
	_propertySet.append(&_forceUnitsStrProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized. TODO: this method is
 * not yet designed to be called after a model has been
 * copied.
 */
void Model::setup()
{
	// Set the current directory to the directory containing the model
	// file.  This is allow files (i.e. bone files) to be specified using
	// relative paths in the model file.  KMS 4/26/06
	string origDirPath;
	string dirPath = IO::getParentDirectory(_fileName);
	if(!dirPath.empty()) {
		origDirPath = IO::getCwd();
		IO::chDir(dirPath);
	}

	int i;

	// CALLBACK SETS
	_analysisSet = new AnalysisSet(this);
	_integCallbackSet = new IntegCallbackSet(this);
	_derivCallbackSet = new DerivCallbackSet(this);

	/* Initialize the length and force units from the strings specified in the XML file.
	 * If they were not specified, use meters and Newtons.
	 *
	 */
	if (_lengthUnitsStrProp.getUseDefault()){
		_lengthUnits = Units(Units::simmMeters);
		_lengthUnitsStr = _lengthUnits.getLabel();
	}
	else
		_lengthUnits = Units(_lengthUnitsStr);

	if (_forceUnitsStrProp.getUseDefault()){
		_forceUnits = Units(Units::simmNewtons);
		_forceUnitsStr = _forceUnits.getLabel();
	}
	else
		_forceUnits = Units(_forceUnitsStr);

	/* Muscle groups are set up with these steps:
	 *   1. empty groups are created and named.
	 *   2. group->setup() is called so the groups
	 *      can store pointers to their muscles
	 *   3. muscle->setup() is called so the muscles
	 *      can store pointers to the groups they're in.
	 */
	AbstractMuscle *sm;
	for (i = 0; i < _actuatorSet.getSize(); i++)
	{
		if ((sm = dynamic_cast<AbstractMuscle*>(_actuatorSet.get(i))))
		{
			const Array<string>* groupNames = sm->getGroupNames();
			for (int j = 0; j < groupNames->getSize(); j++)
				enterGroup((*groupNames)[j]);
		}
	}

	for (i = 0; i < _muscleGroups.getSize(); i++)
		_muscleGroups[i]->setup(this);

	if(_dynamicsEngine) 
		_dynamicsEngine->setup(this);

	_actuatorSet.setup(this);

	_contactSet.setup(this);

	_yi.setSize(getNumStates());
	_ypi.setSize(getNumPseudoStates());

	// Get reasonable initial state values for actuators by querying them for their current
	// states (it is assumed that by this point an actuator has initialized with reasonable
	// default state values.
	_actuatorSet.getStates(&_yi[getNumCoordinates()+getNumSpeeds()]);
	_contactSet.getStates(&_yi[getNumCoordinates()+getNumSpeeds()+_actuatorSet.getNumStates()]);

	/* The following code should be replaced by a more robust
	 * check for problems while creating the model.
	 */
	if (_dynamicsEngine && _dynamicsEngine->getNumBodies() > 0)
	{
		_builtOK = true;
	}

	// Restore the current directory.
	if(!origDirPath.empty())
		IO::chDir(origDirPath);

	cout << "Created model " << getName() << " from file " << getInputFileName() << endl;
}
//_____________________________________________________________________________
/**
 * Register the types used by this class.
void Model::registerTypes()
{
	// now handled by RegisterTypes_osimSimulation()
}
 */


//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
Model& Model::operator=(const Model &aModel)
{
	// BASE CLASS
	Object::operator=(aModel);

	// Class Members
	copyData(aModel);

	setup();

	return(*this);
}


//=============================================================================
// GRAVITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the gravity vector in the gloabl frame.
 *
 * @param rGrav the XYZ gravity vector in the global frame is returned here.
 */
void Model::getGravity(double rGrav[3]) const
{
	getDynamicsEngine().getGravity(rGrav);
}
//_____________________________________________________________________________
/**
 * Set the gravity vector in the gloabl frame.
 *
 * @param aGrav the XYZ gravity vector
 * @return Whether or not the gravity vector was successfully set.
 */
bool Model::setGravity(double aGrav[3])
{
	return getDynamicsEngine().setGravity(aGrav);
}


//=============================================================================
// NUMBERS
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the number of controls in the model.
 *
 * @return Number of controls.
 */
int Model::getNumControls() const
{
	return _actuatorSet.getNumControls();
}
//_____________________________________________________________________________
/**
 * Get the total number of states in the model.
 *
 * @return Number of states.
 */
int Model::getNumStates() const
{
	int n;
	n = getDynamicsEngine().getNumCoordinates();
	n += getDynamicsEngine().getNumSpeeds();
	n += _actuatorSet.getNumStates();
	n += _contactSet.getNumStates();

	return n;
}
//_____________________________________________________________________________
/**
 * Get the total number of pseudostates in the model.
 *
 * @return Number of pseudostates.
 */
int Model::getNumPseudoStates() const
{
	int n = _actuatorSet.getNumPseudoStates();
	n += _contactSet.getNumPseudoStates();

	return n;
}
//_____________________________________________________________________________
/**
 * Get the total number of bodies in the model.
 *
 * @return Number of bodies.
 */
int Model::getNumBodies() const
{
	return getDynamicsEngine().getNumBodies();
}
//_____________________________________________________________________________
/**
 * Get the total number of joints in the model.
 *
 * @return Number of joints.
 */
int Model::getNumJoints() const
{
	return getDynamicsEngine().getNumJoints();
}
//_____________________________________________________________________________
/**
 * Get the total number of coordinates in the model.
 *
 * @return Number of coordinates.
 */
int Model::getNumCoordinates() const
{
	return getDynamicsEngine().getNumCoordinates();
}
//_____________________________________________________________________________
/**
 * Get the total number of speeds in the model.
 *
 * @return Number of speeds.
 */
int Model::getNumSpeeds() const
{
	return getDynamicsEngine().getNumSpeeds();
}
//_____________________________________________________________________________
/**
 * Get the number of actuators in the model
 *
 * @return The number of actuators
 */
int Model::getNumActuators() const
{
	return _actuatorSet.getSize();
}
//_____________________________________________________________________________
/**
 * Get the number of contacts in the model.
 *
 * @return The number of contacts
 */
int Model::getNumContacts() const
{
	return _contactSet.getSize();
}

//_____________________________________________________________________________
/**
 * Get the number of analyses in the model.
 *
 * @return The number of analyses
 */
int Model::getNumAnalyses() const
{
	if (_analysisSet)
		return _analysisSet->getSize();

	return 0;
}


//=============================================================================
// DYNAMICS ENGINE
//=============================================================================
//_____________________________________________________________________________
/**
 * Returns true if model has a dynamics engine
 */
bool Model::hasDynamicsEngine() const
{
	return _dynamicsEngine != 0;
}
//_____________________________________________________________________________
/**
 * Get the model's dynamics engine
 *
 * @return Reference to the abstract dynamics engine
 */
AbstractDynamicsEngine& Model::getDynamicsEngine() const
{
	assert(hasDynamicsEngine());

	return *_dynamicsEngine;
}
//_____________________________________________________________________________
/**
 * Set the model's dynamics engine
 *
 * @param the abstract dynamics engine to set to
 */
void Model::setDynamicsEngine(AbstractDynamicsEngine& aEngine)
{
	// TODO: make a copy() ?
	_dynamicsEngine = &aEngine;
}


//=============================================================================
// SET CURRENT TIME, CONTROLS, AND STATES
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the model time, controls, and states
 *
 * @param aT Time.
 * @param aX Controls at time aT.
 * @param aY States at time aT.
 */
void Model::set(double aT, const double aX[], const double aY[])
{
	// TIME
	setTime(aT);

	// CONTROLS
	setControls(aX);

	// STATES
	setStates(aY);
}


//=============================================================================
// TIME
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the current time.
 *
 * @param Current time.
 */
void Model::setTime(double aTime)
{
	_time = aTime;
}

//_____________________________________________________________________________
/**
 * Get the current time.
 *
 * @return Current time.
 */
double Model::getTime() const
{
	return _time;
}

//=============================================================================
// TIME NORMALIZATION CONSTANT
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the constant by which time is normalized.
 *
 * The normalization constant must be greater than or equal to the constant
 * rdMath::ZERO.
 *
 * @param Time normalization constant.
 */
void Model::setTimeNormConstant(double aNormConst)
{
	_tNormConst = aNormConst;

	if(_tNormConst < rdMath::ZERO) _tNormConst = rdMath::ZERO; 
}
//_____________________________________________________________________________
/**
 * Get the constant by which time is normalized.
 *
 * By default, the time normalization constant is 1.0.
 *
 * @return Current time normalization constant.
 */
double Model::getTimeNormConstant() const
{
	return _tNormConst;
}


//=============================================================================
// CONTROLS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the controls in the model.
 *
 * @param aX Array of control values
 */
void Model::setControls(const double aX[])
{
	_actuatorSet.setControls(aX);
}
//_____________________________________________________________________________
/**
 * Set a control value, specified by index
 *
 * @param aIndex The index of the control to set
 * @param aValue The control value
 */
void Model::setControl(int aIndex, double aValue)
{
	_actuatorSet.setControl(aIndex, aValue);
}
//_____________________________________________________________________________
/**
 * Set a control value, specified by name
 *
 * @param aName The name of the control to set
 * @param aValue The control value
 */
void Model::setControl(const string &aName, double aValue)
{
	_actuatorSet.setControl(aName, aValue);
}
//_____________________________________________________________________________
/**
 * Get the control values in the model.
 *
 * @param rX The control values are returned here.
 */
void Model::getControls(double rX[]) const
{
	_actuatorSet.getControls(rX);
}
//_____________________________________________________________________________
/**
 * Get a control value, specified by index
 *
 * @param aIndex The index of the control to get
 * @return The control value
 */
double Model::getControl(int aIndex) const
{
	return _actuatorSet.getControl(aIndex);
}
//_____________________________________________________________________________
/**
 * Get a control value, specified by name
 *
 * @param aName The name of the control to get
 * @return The control value
 */
double Model::getControl(const string &aName) const
{
	return _actuatorSet.getControl(aName);
}
//_____________________________________________________________________________
/**
 * Get the name of a control.
 *
 * @param aIndex Index of the control whose name to get.
 * @return Name of the control.
 */
string Model::getControlName(int aIndex) const
{
	return _actuatorSet.getControlName(aIndex);
}
//_____________________________________________________________________________
/**
 * Get the index of a control.
 *
 * @param aName Name of the control whose index to get.
 * @return Index of the control.  -1 is returned if there is no control
 * with the specified name.
int Model::getControlIndex(const string &aName) const
{
	return _actuatorSet.getControlIndex(aName);
}
*/


//=============================================================================
// STATES
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the states for this model.
 *
 * @param aYI Array of states.  The size of rY must be at least the number
 * of states, which can be found by calling getNumStates().
 */
void Model::setStates(const double aY[])
{
	// CONFIGURATION
	getDynamicsEngine().setConfiguration(aY);

	// ACTUATORS
	int index = getNumCoordinates() + getNumSpeeds();
	ActuatorSet *actuatorSet = getActuatorSet();
	actuatorSet->setStates(&aY[index]);

	// CONTACT FORCES
	index += actuatorSet->getNumStates();
	getContactSet()->setStates(&aY[index]);
}
//_____________________________________________________________________________
/**
 * Get the states for this model.
 *
 * @param rYI Array of states.  The size of rY must be at least the number
 * of states, which can be found by calling getNumStates().
 */
void Model::getStates(double rY[]) const
{
	// CONFIGURATION
	getDynamicsEngine().getConfiguration(rY);

	// ACTUATORS
	int index = getNumCoordinates() + getNumSpeeds();
	const ActuatorSet *actuatorSet = getActuatorSet();
	actuatorSet->getStates(&rY[index]);

	// CONTACT FORCES
	index += actuatorSet->getNumStates();
	getContactSet()->getStates(&rY[index]);
}
//_____________________________________________________________________________
/**
 * Get the names of the states.
 *
 * @param rStateNames Array of state names..
 */
void Model::getStateNames(Array<string> &rStateNames) const
{
	getDynamicsEngine().getCoordinateSet()->getNames(rStateNames);
	getDynamicsEngine().getSpeedSet()->getNames(rStateNames);
	getActuatorSet()->getStateNames(rStateNames);
	getContactSet()->getStateNames(rStateNames);
}

//=============================================================================
// INITIAL STATES
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the initial states for this model.
 *
 * @param aYI Array of states.  The size of rY must be at least the number
 * of states, which can be found by calling getNumStates().
 */
void Model::setInitialStates(const double aYI[])
{
	memcpy(&_yi[0],aYI,getNumStates()*sizeof(double));
}
//_____________________________________________________________________________
/**
 * Get the initial states for this model.
 *
 * @param rYI Array of states.  The size of rY must be at least the number
 * of states, which can be found by calling getNumStates().
 */
void Model::getInitialStates(double rYI[]) const
{
	memcpy(rYI,&_yi[0],getNumStates()*sizeof(double));
}


//=============================================================================
// PSEUDO-STATES
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the names of the pseudo-states.
 *
 * @param rStateNames Array of pseudo-state names.
 * @return Number of pseudo-states.
 */
int Model::getPseudoStateNames(Array<string> &rStateNames) const
{
	//TODO_CLAY
	return 0;
}

//_____________________________________________________________________________
/**
 * Set the pseudo-states for this model.  Pseudo-states are quantities that
 * depend on the time history of an integration but are not integrated.
 * Pseudo-states cannot be computed from the states.
 *
 * @param aYP Array of pseudo-states.  The size of rYP must be at least the
 * number of pseudo-states, which can be found by calling getNumPseudoStates().
 */
void Model::setPseudoStates(const double aYP[])
{
	// ACTUATORS
	ActuatorSet *actuatorSet = getActuatorSet();
	actuatorSet->setPseudoStates(&aYP[0]);

	// CONTACT FORCES
	int index = actuatorSet->getNumPseudoStates();
	getContactSet()->setPseudoStates(&aYP[index]);
}
//_____________________________________________________________________________
/**
 * Get the pseudo-states for this model.  Pseudo-states are quantities that
 * depend on the time history of an integration but are not integrated.
 * Pseudo-states cannot be computed from the states.
 *
 * @param rYIP Array of pseudo-states.  The size of rYP must be at least the
 * number of pseudo-states, which can be found by calling getNumPseudoStates().
 */
void Model::getPseudoStates(double rYP[]) const
{
	// ACTUATORS
	const ActuatorSet *actuatorSet = getActuatorSet();
	actuatorSet->getPseudoStates(&rYP[0]);

	// CONTACT FORCES
	int index = actuatorSet->getNumPseudoStates();
	getContactSet()->getPseudoStates(&rYP[index]);
}


//=============================================================================
// INITIAL PSEUDO STATES
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the initial pseudo-states for this model.
 *
 * @param aYPI Array of pseudo-states.  The size of rYP must be at least the
 * number of pseudo-states, which can be found by calling getNumPseudoStates().
 */
void Model::setInitialPseudoStates(const double aYPI[])
{
	memcpy(&_ypi[0],aYPI,getNumPseudoStates()*sizeof(double));
}
//_____________________________________________________________________________
/**
 * Get the initial pseudo-states for this model.
 *
 * @param rYPI Array of pseudo-states.  The size of rYP must be at least the
 * number of pseudo-states, which can be found by calling getNumPseudoStates().
 */
void Model::getInitialPseudoStates(double rYPI[]) const
{
	memcpy(rYPI,&_ypi[0],getNumPseudoStates()*sizeof(double));
}


//=============================================================================
// ACTUATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the actuator set for the model.
 *
 * @return Pointer to the actuator set.
 */
ActuatorSet* Model::getActuatorSet()
{
	return(&_actuatorSet);
}
//_____________________________________________________________________________
/**
 * Get the actuator set for the model.
 *
 * @return Pointer to the actuator set.
 */
const ActuatorSet* Model::getActuatorSet() const
{
	return(&_actuatorSet);
}

//_____________________________________________________________________________
/**
 * Add a muscle group to the model, or return a pointer to it if it's
 * already in the model.
 *
 * @param aName the name of the muscle group
 * @return Pointer to the muscle group
 */
MuscleGroup* Model::enterGroup(const string& aName)
{
	for (int i = 0; i < _muscleGroups.getSize(); i++)
		if (aName == _muscleGroups[i]->getName())
			return _muscleGroups[i];

	MuscleGroup* newGroup = new MuscleGroup();
	newGroup->setName(aName);
	_muscleGroups.append(newGroup);

	return newGroup;
}


//=============================================================================
// CONTACT
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the contact set for the model.
 *
 * @return Pointer to the contact set.
 */
ContactForceSet* Model::getContactSet()
{
	return(&_contactSet);
}
//_____________________________________________________________________________
/**
 * Get the contact set for the model.
 *
 * @return Pointer to the contact set.
 */
const ContactForceSet* Model::getContactSet() const
{
	return(&_contactSet);
}


//=============================================================================
// INTEGRATION CALLBACKS
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the set of integration callbacks.
 *
 * @retun Integration callback set of this model.
 */
IntegCallbackSet* Model::getIntegCallbackSet()
{
	return(_integCallbackSet);
}
//_____________________________________________________________________________
/**
 * Get the set of integration callbacks.
 *
 * @retun Integration callback set of this model.
 */
const IntegCallbackSet* Model::getIntegCallbackSet() const
{
	return(_integCallbackSet);
}
//_____________________________________________________________________________
/**
 * Add an integration callback to the model
 *
 * @param aCallback Pointer to the integration callback to add.
 */
void Model::addIntegCallback(IntegCallback *aCallback)
{
	// CHECK FOR NULL
	if(aCallback==NULL) {
		printf("Model.addIntegCallback:  ERROR- NULL callback.\n");
	}

	// ADD
	aCallback->setModel(this);
	_integCallbackSet->append(aCallback);
}

//_____________________________________________________________________________
/**
 * Remove an integration callback from the model
 *
 * @param aCallback Pointer to the integration callback to remove.
 */
void Model::removeIntegCallback(IntegCallback *aCallback)
{
	// CHECK FOR NULL
	if(aCallback==NULL) {
		printf("Model.removeIntegCallback:  ERROR- NULL callback.\n");
	}

	// ADD
	aCallback->setModel(0);
	_integCallbackSet->remove(aCallback);
}

//=============================================================================
// DERIVATIVE CALLBACKS
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the set of derivative callbacks.
 *
 * @return Derivative callback set of this model.
 */
DerivCallbackSet* Model::getDerivCallbackSet()
{
	return(_derivCallbackSet);
}
//_____________________________________________________________________________
/**
 * Get the set of derivative callbacks.
 *
 * @return Derivative callback set of this model.
 */
const DerivCallbackSet* Model::getDerivCallbackSet() const
{
	return(_derivCallbackSet);
}
//_____________________________________________________________________________
/**
 * Add a derivative callback to the model
 *
 * @param aCallback Pointer to the derivative callback to add.
 */
void Model::addDerivCallback(DerivCallback *aCallback)
{
	// CHECK FOR NULL
	if(aCallback==NULL) {
		printf("Model.addDerivCallback:  ERROR- NULL callback.\n");
	}

	// ADD
	aCallback->setModel(this);
	_derivCallbackSet->append(aCallback);
}


//--------------------------------------------------------------------------
// ANALYSES
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the set of analyses.
 *
 * @return Analysis set of this model.
 */
AnalysisSet* Model::getAnalysisSet()
{
	return _analysisSet;
}
//_____________________________________________________________________________
/**
 * Get the set of analyses.
 *
 * @return Analysis set of this model.
 */
const AnalysisSet* Model::getAnalysisSet() const
{
	return _analysisSet;
}
//_____________________________________________________________________________
/**
 * Add an analysis to the model.
 *
 * @param aAnalysis pointer to the analysis to add
 */
void Model::addAnalysis(Analysis *aAnalysis)
{
	if (aAnalysis && _analysisSet)
	{
		aAnalysis->setModel(this);
		_analysisSet->append(aAnalysis);
	}
}


//==========================================================================
// DERIVATIVES
//==========================================================================
//_____________________________________________________________________________
/**
 * Compute the derivatives of the states of the model.  For this method
 * to return valid derivatives, the following methods should have been
 * called:
 * 1. Model::set()
 * 2. ActuatorSet::computeActuation()
 * 3. ActuatorSet::apply()
 * 4. ContactSet::computeContact()
 * 5. ContactSet::appy()
 *
 * @param rDYDT Derivatives of the states.  These have not been time
 * normalized for integration, but are in un-normalized units.  The
 * length of rDYDT should be at least getNumStates().
 */
void Model::computeDerivatives(double rDYDT[])
{
	// Q's and U's
	int nq = getNumCoordinates();
	int nu = getNumSpeeds();
	getDynamicsEngine().computeDerivatives(&rDYDT[0],&rDYDT[nq]);

	// Actuators
	int iAct = nq + nu;
	_actuatorSet.computeStateDerivatives(&rDYDT[iAct]);

	// Contact Forces
	int iCtx = iAct + _actuatorSet.getNumStates();
	_contactSet.computeStateDerivatives(&rDYDT[iCtx]);
}
//_____________________________________________________________________________
/**
 * Compute the derivatives of the auxiliary states (the actuator and contact
 * states).  The auxiliary states are any integrated variables that are
 * not the coordinates or speeds.
 *
 * @param rDYDT Derivatives of the auxiliary states.  Note that this is a shorter
 * array than the rDYDT used with computeDerivatives (above) as it omits the
 * q and u derivatives. In particular, the length of rDYDT should be at least 
 * _actuatorSet.getNumStates()+_contactSet.getNumStates()
 * These have not been time normalized for integration, but are in un-normalized units.
 */
void Model::computeAuxiliaryDerivatives(double rDYDT[])
{
	// Actuators
	_actuatorSet.computeStateDerivatives(&rDYDT[0]);

	// Contact Forces
	int iCtx = _actuatorSet.getNumStates();
	_contactSet.computeStateDerivatives(&rDYDT[iCtx]);
}


//==========================================================================
// OPERATIONS
//==========================================================================
//--------------------------------------------------------------------------
// SCALE
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Scale the model
 *
 * @param aScaleSet the set of XYZ scale factors for the bodies
 * @param aFinalMass the mass that the scaled model should have
 * @param aPreserveMassDist whether or not the masses of the
 *        individual bodies should be scaled with the body scale factors.
 * @return Whether or not scaling was successful.
 */
bool Model::scale(const ScaleSet& aScaleSet, double aFinalMass, bool aPreserveMassDist)
{
	int i;

	// 1. Save the current pose of the model, then put it in a
	//    default pose, so pre- and post-scale muscle lengths
	//    can be found.
	double *savedConfiguration = new double[getNumConfigurations()];
	getDynamicsEngine().getConfiguration(savedConfiguration);
	getDynamicsEngine().applyDefaultConfiguration();

	// 2. For each Actuator, call its preScale method so it
	//    can calculate and store its pre-scale length in the
	//    current position, and then call its scale method to
	//    scale all of the muscle properties except tendon and
	//    fiber length.
	for (i = 0; i < _actuatorSet.getSize(); i++)
	{
		_actuatorSet.get(i)->preScale(aScaleSet);
		_actuatorSet.get(i)->scale(aScaleSet);
	}

	// 3. Scale the rest of the model
	bool returnVal = getDynamicsEngine().scale(aScaleSet, aFinalMass, aPreserveMassDist);

	// 4. If the dynamics engine was scaled successfully,
	//    call each SimmMuscle's postScale method so it
	//    can calculate its post-scale length in the current
	//    position and then scale the tendon and fiber length
	//    properties.
	if (returnVal)
	{
		for (i = 0; i < _actuatorSet.getSize(); i++)
			_actuatorSet.get(i)->postScale(aScaleSet);
	}

	// 5. Put the model back in whatever pose it was in.
	getDynamicsEngine().setConfiguration(savedConfiguration);
	delete savedConfiguration;

	return returnVal;
}


//=============================================================================
// PRINT
//=============================================================================
//_____________________________________________________________________________
/**
 * Print some basic information about the model.
 *
 * @param aOStream Output stream.
 */
void Model::printBasicInfo(std::ostream &aOStream) const
{
	aOStream<<"             MODEL: "<<getName()<<"\n";
	aOStream<<"         actuators: "<<getNumActuators()<<"\n";
	aOStream<<"          analyses: "<<getNumAnalyses()<<"\n";
	aOStream<<"          contacts: "<<getNumContacts()<<"\n";
}
//_____________________________________________________________________________
/**
 * Print detailed information about the model.
 *
 * @param aOStream Output stream.
 */
void Model::printDetailedInfo(std::ostream &aOStream) const
{
	//int i;

	aOStream << "MODEL: " << getName() << "\n";

	aOStream << "\nANALYSES (" << getNumAnalyses() << ")\n";
	for (int i = 0; i < _analysisSet->getSize(); i++)
		aOStream << "analysis[" << i << "] = " << _analysisSet->get(i)->getName() << "\n";

	aOStream << "\nBODIES (" << getNumBodies() << ")\n";
	BodySet *bodySet = getDynamicsEngine().getBodySet();
	for(int i=0; i < bodySet->getSize(); i++) {
		AbstractBody *body = bodySet->get(i);
		if(body==NULL) continue;
		aOStream << "body[" << i << "] = " << body->getName();
		aOStream << " (mass: "<<body->getMass()<<")";
		double inertia[3][3];
		body->getInertia(inertia);
		aOStream << " (inertia:";
		for(int j=0; j<3; j++) for(int k=0; k<3; k++) aOStream<<" "<<inertia[j][k];
		aOStream << ")"<<endl;
	}

	aOStream << "\nACTUATORS (" << getNumActuators() << ")\n";
	for (int i = 0; i < _actuatorSet.getSize(); i++) {
		aOStream << "actuator[" << i << "] = " << _actuatorSet.get(i)->getName();
		if (_actuatorSet.get(i)->getNumControls()) {
			aOStream << " (controls: ";
			for(int j = 0; j < _actuatorSet.get(i)->getNumControls(); j++) {
				aOStream << _actuatorSet.get(i)->getControlName(j);
				if(j < _actuatorSet.get(i)->getNumControls()-1) aOStream << ", ";
			}
			aOStream << ")";
		}
		aOStream << std::endl;
	}

	aOStream << "\nCONTACTS (" << getNumContacts() << ")\n";

	aOStream << "numControls = " << getNumControls() << std::endl;
	aOStream << "numStates = " << getNumStates() << std::endl;
	aOStream << "numCoordinates = " << getNumCoordinates() << std::endl;
	aOStream << "numSpeeds = " << getNumSpeeds() << std::endl;
	aOStream << "numActuators = " << getNumActuators() << std::endl;
	aOStream << "numBodies = " << getNumBodies() << std::endl;

	int n;

	/*
	aOStream<<"MODEL: "<<getName()<<"\n";

	n = getNumBodies();
	aOStream<<"\nBODIES ("<<n<<")\n";
	for(i=0;i<n;i++) aOStream<<"body["<<i<<"] = "<<getBodyName(i)<<"\n";

	n = getNQ();
	aOStream<<"\nGENERALIZED COORDINATES ("<<n<<")\n";
	for(i=0;i<n;i++) aOStream<<"q["<<i<<"] = "<<getCoordinateName(i)<<"\n";

	n = getNU();
	aOStream<<"\nGENERALIZED SPEEDS ("<<n<<")\n";
	for(i=0;i<n;i++) aOStream<<"u["<<i<<"] = "<<getSpeedName(i)<<"\n";

	n = getNA();
	aOStream<<"\nACTUATORS ("<<n<<")\n";
	for(i=0;i<n;i++) aOStream<<"actuator["<<i<<"] = "<<getActuatorName(i)<<"\n";

	n = getNP();
	aOStream<<"\nCONTACTS ("<<n<<")\n";

	n = getNYP();
	aOStream<<"\nPSEUDO-STATES ("<<n<<")\n";
	for(i=0;i<n;i++) aOStream<<"yp["<<i<<"] = "<<getPseudoStateName(i)<<"\n";
*/
	n = getNumStates();
	Array<string> stateNames("");
	getStateNames(stateNames);
	aOStream<<"\nSTATES ("<<stateNames.getSize()<<")\n";
	for(int i=0;i<n;i++) aOStream<<"y["<<i<<"] = "<<stateNames[i]<<"\n";
}

//=============================================================================
// TEST
//=============================================================================
void Model::peteTest() const
{
	int i;

	cout << "Model " << getName() << endl;

	cout << "   lengthUnits: " << _lengthUnits.getLabel() << endl;
	cout << "   forceUnits: " << _forceUnits.getLabel() << endl;

	for (i = 0; i < _actuatorSet.getSize(); i++)
	{
		AbstractMuscle *ms = dynamic_cast<AbstractMuscle*>(_actuatorSet.get(i));
		if (ms)
			ms->peteTest();
	}

	for (i = 0; i < _muscleGroups.getSize(); i++)
		_muscleGroups[i]->peteTest();

	getDynamicsEngine().peteTest();
}

#if 0
void Model::kinTest()
{
	int m1 = 0, m2 = 1;
	AbstractMuscle* ms1;
	AbstractMuscle* ms2;
	AbstractCoordinate* hip_flex_r;
	AbstractCoordinate* knee_flex_r;

	hip_flex_r = getDynamicsEngine().getCoordinate("hip_flex_r");
	knee_flex_r = getDynamicsEngine().getCoordinate("knee_flex_r");
	ms1 = dynamic_cast<AbstractMuscle*>(_actuatorSet.get(m1));
	ms2 = dynamic_cast<AbstractMuscle*>(_actuatorSet.get(m2));

	if (hip_flex_r && knee_flex_r && ms1 && ms2)
	{
		cout << "kinTest" << endl;

		hip_flex_r->setValue(0.0 * rdMAth::DTR);
		knee_flex_r->setValue(0.0 * rdMAth::DTR);

		cout << hip_flex_r->getName() << " = " << hip_flex_r->getValue() << ", " << knee_flex_r->getName() << " = " << knee_flex_r->getValue() << endl;
		cout << ms1->getName() << ": length = " << ms1->getLength() << ", ma(hip_flex_r) = " << ms1->getMomentArm(*hip_flex_r) << ", ma(knee_flex_r) = " << ms1->getMomentArm(*knee_flex_r) << endl;
		cout << ms2->getName() << ": length = " << ms2->getLength() << ", ma(hip_flex_r) = " << ms2->getMomentArm(*hip_flex_r) << ", ma(knee_flex_r) = " << ms2->getMomentArm(*knee_flex_r) << endl;
		cout << endl;

		hip_flex_r->setValue(30.0 * rdMAth::DTR);
		knee_flex_r->setValue(0.0 * rdMAth::DTR);

		cout << hip_flex_r->getName() << " = " << hip_flex_r->getValue() << ", " << knee_flex_r->getName() << " = " << knee_flex_r->getValue() << endl;
		cout << ms1->getName() << ": length = " << ms1->getLength() << ", ma(hip_flex_r) = " << ms1->getMomentArm(*hip_flex_r) << ", ma(knee_flex_r) = " << ms1->getMomentArm(*knee_flex_r) << endl;
		cout << ms2->getName() << ": length = " << ms2->getLength() << ", ma(hip_flex_r) = " << ms2->getMomentArm(*hip_flex_r) << ", ma(knee_flex_r) = " << ms2->getMomentArm(*knee_flex_r) << endl;
		cout << endl;

		hip_flex_r->setValue(0.0 * rdMAth::DTR);
		knee_flex_r->setValue(30.0 * rdMAth::DTR);

		cout << hip_flex_r->getName() << " = " << hip_flex_r->getValue() << ", " << knee_flex_r->getName() << " = " << knee_flex_r->getValue() << endl;
		cout << ms1->getName() << ": length = " << ms1->getLength() << ", ma(hip_flex_r) = " << ms1->getMomentArm(*hip_flex_r) << ", ma(knee_flex_r) = " << ms1->getMomentArm(*knee_flex_r) << endl;
		cout << ms2->getName() << ": length = " << ms2->getLength() << ", ma(hip_flex_r) = " << ms2->getMomentArm(*hip_flex_r) << ", ma(knee_flex_r) = " << ms2->getMomentArm(*knee_flex_r) << endl;
		cout << endl;

		hip_flex_r->setValue(20.0 * rdMAth::DTR);
		knee_flex_r->setValue(60.0 * rdMAth::DTR);

		cout << hip_flex_r->getName() << " = " << hip_flex_r->getValue() << ", " << knee_flex_r->getName() << " = " << knee_flex_r->getValue() << endl;
		cout << ms1->getName() << ": length = " << ms1->getLength() << ", ma(hip_flex_r) = " << ms1->getMomentArm(*hip_flex_r) << ", ma(knee_flex_r) = " << ms1->getMomentArm(*knee_flex_r) << endl;
		cout << ms2->getName() << ": length = " << ms2->getLength() << ", ma(hip_flex_r) = " << ms2->getMomentArm(*hip_flex_r) << ", ma(knee_flex_r) = " << ms2->getMomentArm(*knee_flex_r) << endl;
		cout << endl;
	}
}
#endif
void Model::kinTest()
{
	AbstractMuscle* ms1;
	AbstractMuscle* ms2;
	AbstractCoordinate* hip_flex_r;
	AbstractCoordinate* knee_flex_r;

	knee_flex_r = getDynamicsEngine().getCoordinateSet()->get("knee_flex_r");
	hip_flex_r = getDynamicsEngine().getCoordinateSet()->get("hip_flex_r");
	ms1 = dynamic_cast<AbstractMuscle*>(_actuatorSet.get("glut_max1_r"));
	ms2 = dynamic_cast<AbstractMuscle*>(_actuatorSet.get("rectus_fem_r"));

	if (hip_flex_r && knee_flex_r && ms2)
	{
		cout << "kinTest" << endl;

		hip_flex_r->setValue(0.0);

		cout << ms2->getName() << endl;
		for (double angle = 0.0 * rdMath::DTR; angle < 121.0 * rdMath::DTR; angle += 30.0 * rdMath::DTR)
		{
			knee_flex_r->setValue(angle);
			cout << "knee_flex_r = " << angle * rdMath::RTD << ", len = " << ms2->getLength() << ", ma = " << ms2->computeMomentArm(*knee_flex_r) << endl;
		}
	}
#if 0
	if (hip_flex_r && knee_flex_r && ms1)
	{
		cout << "kinTest" << endl;

		knee_flex_r->setValue(0.0);

		cout << ms1->getName() << endl;
		for (double angle = 0.0; angle < 91.0 * rdMath::DTR; angle += 5.0 * rdMath::DTR)
		{
			hip_flex_r->setValue(angle);
			cout << "hip_flex_r = " << angle * rdMath::RTD << ", len = " << ms1->getLength() << ", ma = " << ms1->computeMomentArm(*hip_flex_r) << endl;
		}
	}
#endif
}

