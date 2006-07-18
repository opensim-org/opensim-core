// SimmModel.cpp
// Author: Peter Loan
/* Copyright (c) 2005, Stanford University and Peter Loan.
 * 
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
#include "SimmModel.h"
#include "SimmKinematicsEngine.h"
// PATH stuff from Kenny
#ifdef _MSC_VER
	#include <direct.h>
	
	#define PATH_MAX _MAX_PATH
#else
	#include <unistd.h>
#endif

//=============================================================================
// STATICS
//=============================================================================




using namespace OpenSim;
using namespace std;



//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmModel::SimmModel() :
   _muscles((ArrayPtrs<SimmMuscle>&)_musclesProp.getValueObjArray()),
	_kinematicsEngine((ArrayPtrs<AbstractDynamicsEngine>&)_kinematicsEngineProp.getValueObjArray()),
   _gravity(_gravityProp.getValueDblArray()),
	_builtOK(false)
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Constructor from an XML file
 */
SimmModel::SimmModel(const string &aFileName) :
	Model(aFileName),
   _muscles((ArrayPtrs<SimmMuscle>&)_musclesProp.getValueObjArray()),
	_kinematicsEngine((ArrayPtrs<AbstractDynamicsEngine>&)_kinematicsEngineProp.getValueObjArray()),
   _gravity(_gravityProp.getValueDblArray()),
	_builtOK(false)
{
	setNull();
	updateFromXMLNode();

	_fileName = aFileName;
}

SimmModel::SimmModel(DOMElement *aElement) :
   Model(aElement),
   _muscles((ArrayPtrs<SimmMuscle>&)_musclesProp.getValueObjArray()),
	_kinematicsEngine((ArrayPtrs<AbstractDynamicsEngine>&)_kinematicsEngineProp.getValueObjArray()),
   _gravity(_gravityProp.getValueDblArray()),
	_builtOK(false)
{
	setNull();
	updateFromXMLNode();
}


//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmModel::~SimmModel()
{
}


//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aModel SimmModel to be copied.
 */

SimmModel::SimmModel(const SimmModel &aModel) :
   _muscles((ArrayPtrs<SimmMuscle>&)_musclesProp.getValueObjArray()),
	_kinematicsEngine((ArrayPtrs<AbstractDynamicsEngine>&)_kinematicsEngineProp.getValueObjArray()),
   _gravity(_gravityProp.getValueDblArray()),
	_builtOK(false)
{
	setupProperties();
	copyData(aModel);	
}

//_____________________________________________________________________________
/**
 * Copy this SimmModel and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmModel.
 */
Object* SimmModel::copy() const
{
	SimmModel *model = new SimmModel(*this);
	return(model);
}
//_____________________________________________________________________________
/**
 * Copy this SimmModel and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmModel::SimmModel(DOMElement*) in order to establish the
 * relationship of the SimmModel object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmModel object. Finally, the data members of the copy are
 * updated using SimmModel::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmModel.
 */
Object* SimmModel::copy(DOMElement *aElement) const
{
	SimmModel *model = new SimmModel(aElement);
	*model = *this;
	model->updateFromXMLNode();
	return(model);
}

//_____________________________________________________________________________
/**
 * Copy the member variables of the model.
 */
void SimmModel::copyData(const SimmModel &aModel)
{
	_muscles = aModel._muscles;
	_muscleGroups = aModel._muscleGroups;
	_kinematicsEngine = aModel._kinematicsEngine;
	_gravity = aModel._gravity;
}


SimmModel& SimmModel::operator=(const SimmModel &aModel)
{
	// BASE CLASS
	Model::operator=(aModel);

	// Class Members
	copyData(aModel);

	setup();

	return(*this);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the values of all data members to an appropriate "null" value.
 */
void SimmModel::setNull()
{
	setType("SimmModel");
	setupProperties();
}

void SimmModel::setupProperties()
{
	_kinematicsEngineProp.setName("KinematicsEngines");
	ArrayPtrs<Object> kes;
	_kinematicsEngineProp.setValue(kes);
	_propertySet.append(&_kinematicsEngineProp);

	_musclesProp.setName("Muscles");
	ArrayPtrs<Object> musc;
	_musclesProp.setValue(musc);
	_propertySet.append(&_musclesProp);

	const double defaultGravity[] = {0.0, -9.80665, 0.0};
	_gravityProp.setName("gravity");
	_gravityProp.setValue(3, defaultGravity);
	_propertySet.append(&_gravityProp);
}

void SimmModel::registerTypes()
{
	Object::RegisterType(SimmMuscle());
	Object::RegisterType(SimmMuscleGroup());
	SimmMuscle::registerTypes();
}

void SimmModel::setKinematicsEngine(AbstractDynamicsEngine& aKE)
{
	//*(_kinematicsEngine[0]) = aKE; TODO
}

AbstractDynamicsEngine& SimmModel::getKinematicsEngine() const
{
	assert(_kinematicsEngine.getSize() > 0);

	return *(_kinematicsEngine[0]);
}

SimmKinematicsEngine& SimmModel::getSimmKinematicsEngine() const
{
	assert(_kinematicsEngine.getSize() > 0);

	SimmKinematicsEngine* ske = dynamic_cast<SimmKinematicsEngine*>(_kinematicsEngine[0]);
	assert(ske);

	return *ske;
}

/* Return a label describing the gravity vector (used for
 * writing SIMM files). It assumes that gravity is expressed
 * in Nm/sec2, and is aligned with a major axis.
 */
const char* SimmModel::getGravityLabel() const
{
	if (_gravity[0] <= -9.8)
		return "-X";
	if (_gravity[0] >= 9.8)
		return "+X";
	if (_gravity[1] <= -9.8)
		return "-Y";
	if (_gravity[1] >= 9.8)
		return "+Y";
	if (_gravity[2] <= -9.8)
		return "-Z";
	if (_gravity[2] >= 9.8)
		return "+Z";

	return "";
}

bool SimmModel::bodyNeededForDynamics(SimmBody* aBody)
{
	// TODO check for wrap objects, etc.
	aBody->_sdfastInfo.skippable = true;
	return false;
}

SimmMuscleGroup* SimmModel::enterGroup(const string& aName)
{
	for (int i = 0; i < _muscleGroups.getSize(); i++)
		if (aName == _muscleGroups[i]->getName())
			return _muscleGroups[i];

	SimmMuscleGroup* newGroup = new SimmMuscleGroup();
	newGroup->setName(aName);
	_muscleGroups.append(newGroup);

	return newGroup;
}

/* Perform some set up functions that happen after the
 * object has been deserialized or copied. This is basically
 * calling the setup() functions of the member objects.
 */
void SimmModel::setup()
{
	// Set the current directory to the directory containing the model
	// file.  This is allow files (i.e. bone files) to be specified using
	// relative paths in the model file.  KMS 4/26/06
	char origDirPath[PATH_MAX] = "";
	
	string::size_type dirSep = _fileName.rfind('/'); // Unix/Mac dir separator
	
	if (dirSep == string::npos)
		dirSep = _fileName.rfind('\\'); // DOS dir separator
	
	if (dirSep != string::npos) // if '_fileName' contains path information...
	{
		string dirPath(_fileName, 0, dirSep);
		
		if (dirPath.length() > 0)
		{
			getcwd(origDirPath, PATH_MAX);
			chdir(dirPath.c_str());
		}
	}

	int i;

	/* Muscle groups are set up with these steps:
	 *   1. empty groups are created and named.
	 *   2. group->setup() is called so the groups
	 *      can store pointers to their muscles
	 *   3. muscle->setup() is called so the muscles
	 *      can store pointers to the groups they're in.
	 */
	SimmMuscle* sm;
	for (i = 0; i < _muscles.getSize(); i++)
	{
		if (sm = dynamic_cast<SimmMuscle*>(_muscles[i]))
		{
			const Array<string>& groupNames = sm->getGroupNames();
			for (int j = 0; j < groupNames.getSize(); j++)
				enterGroup(groupNames[j]);
		}
	}

	for (i = 0; i < _muscleGroups.getSize(); i++)
		_muscleGroups[i]->setup(this);

	for (i = 0; i < _muscles.getSize(); i++)
	{
		if (sm = dynamic_cast<SimmMuscle*>(_muscles[i]))
			sm->setup(this, &getSimmKinematicsEngine());
	}

	SimmKinematicsEngine* ske;
	for (i = 0; i < _kinematicsEngine.getSize(); i++)
	{
		if (ske = dynamic_cast<SimmKinematicsEngine*>(_kinematicsEngine[i]))
			ske->setup(this);
	}

	/* The following code should be replaced by a more robust
	 * check for problems while creating the model.
	 */
	if (_kinematicsEngine.getSize() > 0 && _kinematicsEngine[0]->getNumBodies() > 0)
	{
		_builtOK = true;

		/* Copy number of bodies to base class. */
		_nb = _kinematicsEngine[0]->getNumBodies();
	}
	
	// Restore the current directory.
	if (origDirPath[0] != '\0')
		chdir(origDirPath);

	cout << "Created model " << getName() << " from file " << _fileName << endl;
}
void SimmModel::moveMarkersToCloud(Storage& aMarkerStorage)
{
	getSimmKinematicsEngine().moveMarkersToCloud(aMarkerStorage);
}

int SimmModel::deleteUnusedMarkers(const Array<string>& aMarkerNames)
{
	return getSimmKinematicsEngine().deleteUnusedMarkers(aMarkerNames);
}

//_____________________________________________________________________________
/**
 * Replace all markers in the model with the ones in the
 * passed-in marker set. Return value is the number of markers
 * in the marker set that were successfully added to the model.
 */
int SimmModel::replaceMarkerSet(SimmMarkerSet& aMarkerSet)
{
	return getSimmKinematicsEngine().replaceMarkerSet(aMarkerSet);
}

//_____________________________________________________________________________
/**
 * Update all markers in the model with the ones in the
 * passed-in marker set. If the marker does not yet exist
 * in the model, it is added.
 */
void SimmModel::updateMarkers(ArrayPtrs<SimmMarker>& aMarkerArray)
{
	getSimmKinematicsEngine().updateMarkers(aMarkerArray);
}

//_____________________________________________________________________________
/**
 * Update all coordinates in the model with the ones in the
 * passed-in coordinate set. If the coordinate does not yet exist
 * in the model, it is not added.
 */
void SimmModel::updateCoordinates(ArrayPtrs<SimmCoordinate>& aCoordinateArray)
{
	getSimmKinematicsEngine().updateCoordinates(aCoordinateArray);
}

double SimmModel::takeMeasurement(const SimmMeasurement& aMeasurement)
{
	return getSimmKinematicsEngine().takeMeasurement(aMeasurement);
}

const SimmUnits& SimmModel::getLengthUnits() const
{
	return getSimmKinematicsEngine().getLengthUnits();
}

const SimmUnits& SimmModel::getForceUnits() const
{
	return getSimmKinematicsEngine().getForceUnits();
}

ArrayPtrs<SimmBody>& SimmModel::getBodies()
{
	return getSimmKinematicsEngine().getBodies();
}

ArrayPtrs<SimmCoordinate>& SimmModel::getCoordinates()
{
	return getSimmKinematicsEngine().getCoordinates();
}

void SimmModel::writeSIMMJointFile(string& aFileName) const
{
	getSimmKinematicsEngine().writeSIMMJointFile(aFileName);
}

void SimmModel::writeSIMMMuscleFile(string& aFileName) const
{
   ofstream out;
	int functionIndex = 1;

   out.open(aFileName.c_str());
   out.setf(ios::fixed);
   out.precision(6);

   if (!out.good())
   {
      cout << "Unable to open output file " << aFileName << endl;
      return;
   }

   out << "/**********************************************************/\n";
   out << "/*            Muscle file created by NMBLTS                */\n";
   if (getInputFileName() != "")
      out << "/* name of original model file: " << getInputFileName() << " */\n";
   out << "/**********************************************************/\n";
   out << "\n";

	for (int i = 0; i < getNumberOfMuscles(); i++)
		_muscles[i]->writeSIMM(out);

   out.close();
	cout << "Wrote SIMM muscle file " << aFileName << " from model " << getName() << endl;
}

/* Write an XML file of all the markers in the model.
 * The contents of this file can be pasted into a
 * MarkerSet definition inside a SimmMarkerPlacementParams
 * or a SimmGenericModelParams.
 */
void SimmModel::writeMarkerFile(string& aFileName) const
{
   getSimmKinematicsEngine().writeMarkerFile(aFileName);
}

void SimmModel::peteTest() const
{
	int i;

	cout << "SimmModel " << getName() << endl;
	cout << "   gravity: " << _gravity << endl;

	for (i = 0; i < getNumberOfMuscles(); i++)
		_muscles[i]->peteTest(&getSimmKinematicsEngine());

	for (i = 0; i < getNumberOfMuscleGroups(); i++)
		_muscleGroups[i]->peteTest();

	getSimmKinematicsEngine().peteTest();
}

//--------------------------------------------------------------------------
// NUMBERS
//--------------------------------------------------------------------------

int SimmModel::getNJ() const
{
	return getKinematicsEngine().getNumJoints();
}

int SimmModel::getNQ() const
{
	return getKinematicsEngine().getNumCoordinates();
}

int SimmModel::getNU() const
{
	return getKinematicsEngine().getNumCoordinates();
}

int SimmModel::getNX() const
{
	return 0;
}

int SimmModel::getNA() const
{
	return _muscles.getSize();
}

int SimmModel::getNP() const
{
	return getKinematicsEngine().getNumContacts();
}

int SimmModel::getNY() const
{
	return getKinematicsEngine().getNumStates();
}

int SimmModel::getNYP() const
{
	return getKinematicsEngine().getNumPseudoStates();
}

//--------------------------------------------------------------------------
// NAMES
//--------------------------------------------------------------------------

void SimmModel::setBodyName(int aIndex, const string &aName)
{
	getKinematicsEngine().setBodyName(aIndex, aName);
}

string SimmModel::getBodyName(int aIndex) const
{
	return getKinematicsEngine().getBodyName(aIndex);
}

string SimmModel::getCoordinateName(int aIndex) const
{
	return getKinematicsEngine().getCoordinateName(aIndex);
}

string SimmModel::getSpeedName(int aIndex) const
{
	return getKinematicsEngine().getSpeedName(aIndex);
}

string SimmModel::getActuatorName(int aIndex) const
{
	if (_muscles.getSize() > aIndex)
		return _muscles[aIndex]->getName();
	else
		return "";
}

string SimmModel::getControlName(int aIndex) const
{
	return getKinematicsEngine().getControlName(aIndex);
}

string SimmModel::getStateName(int aIndex) const
{
	return getKinematicsEngine().getStateName(aIndex);
}

string SimmModel::getPseudoStateName(int aIndex) const
{
	return getKinematicsEngine().getPseudoStateName(aIndex);
}

//--------------------------------------------------------------------------
// INDICES FROM NAMES
//--------------------------------------------------------------------------
int SimmModel::getBodyIndex(const string &aName) const
{
	return getKinematicsEngine().getBodyIndex(aName);
}

int SimmModel::getCoordinateIndex(const string &aName) const
{
	return getKinematicsEngine().getCoordinateIndex(aName);
}

int SimmModel::getSpeedIndex(const string &aName) const
{
	return getKinematicsEngine().getSpeedIndex(aName);
}

int SimmModel::getActuatorIndex(const string &aName) const
{
	for (int i = 0; i < _muscles.getSize(); i++)
		if (_muscles[i]->getName() == aName)
			return i;

	return -1;
}

int SimmModel::getControlIndex(const string &aName) const
{
	return getKinematicsEngine().getControlIndex(aName);
}

int SimmModel::getStateIndex(const string &aName) const
{
	return getKinematicsEngine().getStateIndex(aName);
}

int SimmModel::getPseudoStateIndex(const string &aName) const
{
	return getKinematicsEngine().getPseudoStateIndex(aName);
}

//--------------------------------------------------------------------------
// SET CURRENT TIME, CONTROLS, AND STATES
//--------------------------------------------------------------------------

void SimmModel::set(double aT, const double aX[], const double aY[])
{
	getKinematicsEngine().set(aT, aX, aY);
}

//--------------------------------------------------------------------------
// CONTROLS
//--------------------------------------------------------------------------

void SimmModel::setControls(const double aX[])
{
	// TODO
}

void SimmModel::setControl(int aIndex, double aValue)
{
	// TODO
}

void SimmModel::setControl(const std::string &aName, double aValue)
{
	// TODO
}

void SimmModel::getControls(double rX[]) const
{
	// TODO
}

double SimmModel::getControl(int aIndex) const
{
	// TODO
	return 0.0;
}

double SimmModel::getControl(const std::string &aName) const
{
	// TODO
	return 0.0;
}

//--------------------------------------------------------------------------
// INITIAL STATES
//--------------------------------------------------------------------------

void SimmModel::setInitialStates(const double aYI[])
{
	getKinematicsEngine().setInitialStates(aYI);
}

void SimmModel::getInitialStates(double rYI[]) const
{
	getKinematicsEngine().getInitialStates(rYI);
}

double SimmModel::getInitialState(int aIndex) const
{
	return getKinematicsEngine().getInitialState(aIndex);
}

double SimmModel::getInitialState(const string &aName) const
{
	return getKinematicsEngine().getInitialState(aName);
}

//--------------------------------------------------------------------------
// STATES
//--------------------------------------------------------------------------
void SimmModel::setStates(const double aY[])
{
	getKinematicsEngine().setStates(aY);
}

void SimmModel::getStates(double rY[]) const
{
	getKinematicsEngine().getStates(rY);
}

double SimmModel::getState(int aIndex) const
{
	return getKinematicsEngine().getState(aIndex);
}

double SimmModel::getState(const string &aName) const
{
	return getKinematicsEngine().getState(aName);
}

void SimmModel::applyDefaultPose()
{
	getKinematicsEngine().applyDefaultPose();
}

//--------------------------------------------------------------------------
// INITIAL PSEUDO STATES
//--------------------------------------------------------------------------

void SimmModel::setInitialPseudoStates(const double aYPI[])
{
	getKinematicsEngine().setInitialPseudoStates(aYPI);
}

void SimmModel::getInitialPseudoStates(double rYPI[]) const
{
	getKinematicsEngine().getInitialPseudoStates(rYPI);
}

double SimmModel::getInitialPseudoState(int aIndex) const
{
	return getKinematicsEngine().getInitialPseudoState(aIndex);
}

double SimmModel::getInitialPseudoState(const string &aName) const
{
	return getKinematicsEngine().getInitialPseudoState(aName);
}


//--------------------------------------------------------------------------
// PSEUDO STATES
//--------------------------------------------------------------------------

void SimmModel::setPseudoStates(const double aYP[])
{
	getKinematicsEngine().setPseudoStates(aYP);
}

void SimmModel::getPseudoStates(double rYP[]) const
{
	getKinematicsEngine().getPseudoStates(rYP);
}

double SimmModel::getPseudoState(int aIndex) const
{
	return getKinematicsEngine().getPseudoState(aIndex);
}

//--------------------------------------------------------------------------
// CONFIGURATION
//--------------------------------------------------------------------------

void SimmModel::setConfiguration(const double aY[])
{
	getKinematicsEngine().setConfiguration(aY);
}

void SimmModel::setConfiguration(const double aQ[], const double aU[])
{
	getKinematicsEngine().setConfiguration(aQ, aU);
}

void SimmModel::getCoordinates(double rQ[]) const
{
	getKinematicsEngine().getCoordinateValues(rQ);
}

double SimmModel::getCoordinate(int aIndex) const
{
	return getKinematicsEngine().getCoordinateValue(aIndex);
}

double SimmModel::getCoordinate(const string &aName) const
{
	return getKinematicsEngine().getCoordinateValue(aName);
}

void SimmModel::getSpeeds(double rU[]) const
{
	getKinematicsEngine().getSpeeds(rU);
}

double SimmModel::getSpeed(int aIndex) const
{
	return getKinematicsEngine().getSpeed(aIndex);
}

double SimmModel::getSpeed(const string &aName) const
{
	return getKinematicsEngine().getSpeed(aName);
}

void SimmModel::getAccelerations(double rDUDT[]) const
{
	getKinematicsEngine().getAccelerations(rDUDT);
}

double SimmModel::getAcceleration(int aIndex) const
{
	return getKinematicsEngine().getAcceleration(aIndex);
}

double SimmModel::getAcceleration(const string &aSpeedName) const
{
	return getKinematicsEngine().getAcceleration(aSpeedName);
}

void SimmModel::extractConfiguration(const double aY[], double rQ[], double rU[]) const
{
	getKinematicsEngine().extractConfiguration(aY, rQ, rU);
}

//--------------------------------------------------------------------------
// ASSEMBLING THE MODEL
//--------------------------------------------------------------------------
int SimmModel::assemble(double aTime, double *rState, int *aLock, double aTol, int aMaxevals, int *rFcnt, int *rErr)
{
	return getKinematicsEngine().assemble(aTime, rState, aLock, aTol, aMaxevals, rFcnt, rErr);
}

//--------------------------------------------------------------------------
// SCALE THE MODEL
//--------------------------------------------------------------------------
bool SimmModel::scale(const ScaleSet& aScaleSet)
{
	// 1. Save the current pose of the model, then put it in a
	//    default pose, so pre- and post-scale muscle lengths
	//    can be found.
	double *savedStates = new double[getNQ()];
	getStates(savedStates);
	applyDefaultPose();

	// 2. For each SimmMuscle, call its preScale method so it
	//    can calculate and store its pre-scale length in the
	//    current position, and then call its scale method to
	//    scale all of the muscle properties except tendon and
	//    fiber length.
	int i;
	SimmMuscle* sm;
	for (i = 0; i < _muscles.getSize(); i++)
	{
		if (sm = dynamic_cast<SimmMuscle*>(_muscles[i]))
		{
			sm->preScale(aScaleSet);
			sm->scale(aScaleSet);
		}
	}

	// 3. Scale the rest of the model
	bool returnVal = getKinematicsEngine().scale(aScaleSet);

	// 4. If the dynamics engine was scaled successfully,
	//    call each SimmMuscle's postScale method so it
	//    can calculate its post-scale length in the current
	//    position and then scale the tendon and fiber length
	//    properties.
	if (returnVal)
	{
		for (i = 0; i < _muscles.getSize(); i++)
		{
			if (sm = dynamic_cast<SimmMuscle*>(_muscles[i]))
				sm->postScale(aScaleSet);
		}
	}

	// 5. Put the model back in whatever pose it was in.
	setStates(savedStates);
	delete savedStates;

	return returnVal;
}

bool SimmModel::scale(const ScaleSet& aScaleSet, bool aPreserveMassDist, double aFinalMass)
{
	// Scale the muscles
	SimmMuscle* sm;
	for (int i = 0; i < _muscles.getSize(); i++)
	{
		if (sm = dynamic_cast<SimmMuscle*>(_muscles[i]))
			sm->scale(aScaleSet);
	}

	// Scale the rest of the model
	return getKinematicsEngine().scale(aScaleSet, aPreserveMassDist, aFinalMass);
}

//--------------------------------------------------------------------------
// GRAVITY
//--------------------------------------------------------------------------
void SimmModel::getGravity(double rGrav[3]) const
{
	getKinematicsEngine().getGravity(rGrav);
}

void SimmModel::setGravity(double aGrav[3])
{
	getKinematicsEngine().setGravity(aGrav);
}

//--------------------------------------------------------------------------
// BODY INFORMATION
//--------------------------------------------------------------------------

int SimmModel::getGroundID() const
{
	return getKinematicsEngine().getGroundBodyIndex();
}

void SimmModel::setBodyToJointBodyLocal(int aBody, const double aBTJ[3])
{
	getKinematicsEngine().setBodyToJointBodyLocal(aBody, aBTJ);
}

void SimmModel::getBodyToJointBodyLocal(int aBody, double rBTJ[3]) const
{
	getKinematicsEngine().getBodyToJointBodyLocal(aBody, rBTJ);
}

void SimmModel::setInboardToJointBodyLocal(int aBody, const double aBTJ[3])
{
	getKinematicsEngine().setInboardToJointBodyLocal(aBody, aBTJ);
}

void SimmModel::getInboardToJointBodyLocal(int aBody, double rBTJ[3]) const
{
	getKinematicsEngine().getInboardToJointBodyLocal(aBody, rBTJ);
}

//--------------------------------------------------------------------------
// INERTIA
//--------------------------------------------------------------------------
double SimmModel::getMass() const
{
	return getKinematicsEngine().getMass();
}

double SimmModel::getMass(int aBody) const
{
	return getKinematicsEngine().getMass(aBody);
}

int SimmModel::getInertiaBodyLocal(int aBody, double rI[3][3]) const
{
	return getKinematicsEngine().getInertiaBodyLocal(aBody, rI);
}

int SimmModel::getInertiaBodyLocal(int aBody, double *rI) const
{
	return getKinematicsEngine().getInertiaBodyLocal(aBody, rI);
}

void SimmModel::getSystemInertia(double *rM, double rCOM[3], double rI[3][3]) const
{
	getKinematicsEngine().getSystemInertia(rM, rCOM, rI);
}

void SimmModel::getSystemInertia(double *rM, double *rCOM, double *rI) const
{
	getKinematicsEngine().getSystemInertia(rM, rCOM, rI);
}

//--------------------------------------------------------------------------
// KINEMATICS
//--------------------------------------------------------------------------
void SimmModel::getPosition(int aBody, const double aPoint[3], double rPos[3]) const
{
	getKinematicsEngine().getPosition(aBody, aPoint, rPos);
}

void SimmModel::getVelocity(int aBody, const double aPoint[3], double rVel[3]) const
{
	getKinematicsEngine().getVelocity(aBody, aPoint, rVel);
}

void SimmModel::getAcceleration(int aBody, const double aPoint[3], double rAcc[3]) const
{
	getKinematicsEngine().getAcceleration(aBody, aPoint, rAcc);
}

void SimmModel::getDirectionCosines(int aBody, double rDirCos[3][3]) const
{
	getKinematicsEngine().getDirectionCosines(aBody, rDirCos);
}

void SimmModel::getDirectionCosines(int aBody, double *rDirCos) const
{
	getKinematicsEngine().getDirectionCosines(aBody, rDirCos);
}

void SimmModel::getAngularVelocity(int aBody, double rAngVel[3]) const
{
	getKinematicsEngine().getAngularVelocity(aBody, rAngVel);
}

void SimmModel::getAngularVelocityBodyLocal(int aBody, double rAngVel[3]) const
{
	getKinematicsEngine().getAngularVelocityBodyLocal(aBody, rAngVel);
}

void SimmModel::getAngularAcceleration(int aBody, double rAngAcc[3]) const
{
	getKinematicsEngine().getAngularAcceleration(aBody, rAngAcc);
}

void SimmModel::getAngularAccelerationBodyLocal(int aBody, double rAngAcc[3]) const
{
	getKinematicsEngine().getAngularAccelerationBodyLocal(aBody, rAngAcc);
}

//--------------------------------------------------------------------------
// LOAD APPLICATION
//--------------------------------------------------------------------------

// FORCES EXPRESSED IN INERTIAL FRAME
void SimmModel::applyForce(int aBody, const double aPoint[3], const double aForce[3])
{
	getKinematicsEngine().applyForce(aBody, aPoint, aForce);
}

void SimmModel::applyForces(int aN, const int aBodies[], const double aPoints[][3], const double aForces[][3])
{
	getKinematicsEngine().applyForces(aN, aBodies, aPoints, aForces);
}

void SimmModel::applyForces(int aN, const int aBodies[], const double *aPoints, const double *aForces)
{
	getKinematicsEngine().applyForces(aN, aBodies, aPoints, aForces);
}

// FORCES EXPRESSED IN BODY-LOCAL FRAME
void SimmModel::applyForceBodyLocal(int aBody, const double aPoint[3], const double aForce[3])
{
	getKinematicsEngine().applyForceBodyLocal(aBody, aPoint, aForce);
}

void SimmModel::applyForcesBodyLocal(int aN, const int aBodies[], const double aPoints[][3], const double aForces[][3])
{
	getKinematicsEngine().applyForcesBodyLocal(aN, aBodies, aPoints, aForces);
}

void SimmModel::applyForcesBodyLocal(int aN, const int aBodies[], const double *aPoints, const double *aForces)
{
	getKinematicsEngine().applyForcesBodyLocal(aN, aBodies, aPoints, aForces);
}

// TORQUES EXPRESSED IN INERTIAL FRAME
void SimmModel::applyTorque(int aBody, const double aTorque[3])
{
	getKinematicsEngine().applyTorque(aBody, aTorque);
}

void SimmModel::applyTorques(int aN, const int aBodies[], const double aTorques[][3])
{
	getKinematicsEngine().applyTorques(aN, aBodies, aTorques);
}

void SimmModel::applyTorques(int aN, const int aBodies[], const double *aTorques)
{
	getKinematicsEngine().applyTorques(aN, aBodies, aTorques);
}

// TORQUES EXPRESSED IN BODY-LOCAL FRAME (sdbodyt())
void SimmModel::applyTorqueBodyLocal(int aBody, const double aTorque[3])
{
	getKinematicsEngine().applyTorqueBodyLocal(aBody, aTorque);
}

void SimmModel::applyTorquesBodyLocal(int aN, const int aBodies[], const double aTorques[][3])
{
	getKinematicsEngine().applyTorquesBodyLocal(aN, aBodies, aTorques);
}

void SimmModel::applyTorquesBodyLocal(int aN, const int aBodies[], const double *aTorques)
{
	getKinematicsEngine().applyTorquesBodyLocal(aN, aBodies, aTorques);
}

// GENERALIZED FORCES
void SimmModel::applyGeneralizedForce(int aU, double aF)
{
	getKinematicsEngine().applyGeneralizedForce(aU, aF);
}

void SimmModel::applyGeneralizedForces(const double aF[])
{
	getKinematicsEngine().applyGeneralizedForces(aF);
}

void SimmModel::applyGeneralizedForces(int aN, const int aU[], const double aF[])
{
	getKinematicsEngine().applyGeneralizedForces(aN, aU, aF);
}

//--------------------------------------------------------------------------
// LOAD ACCESS AND COMPUTATION
//--------------------------------------------------------------------------

double SimmModel::getNetAppliedGeneralizedForce(int aU) const
{
	return getKinematicsEngine().getNetAppliedGeneralizedForce(aU);
}

void SimmModel::computeGeneralizedForces(double aDUDT[], double rF[]) const
{
	getKinematicsEngine().computeGeneralizedForces(aDUDT, rF);
}

void SimmModel::computeReactions(double rForces[][3], double rTorques[][3]) const
{
	getKinematicsEngine().computeReactions(rForces, rTorques);
}

//--------------------------------------------------------------------------
// PRESCRIBED MOTION
//--------------------------------------------------------------------------

void SimmModel::prescribeMotion(int aJoint, int aAxis, int aPrescribed)
{
}

//--------------------------------------------------------------------------
// EQUATIONS OF MOTION
//--------------------------------------------------------------------------

void SimmModel::formMassMatrix(double *rI)
{
	getKinematicsEngine().formMassMatrix(rI);
}

void SimmModel::formEulerTransform(int aBody, double *rE) const
{
	getKinematicsEngine().formEulerTransform(aBody, rE);
}

void SimmModel::formJacobianTranslation(int aBody, const double aPoint[3], double *rJ, int aRefBody) const
{
	getKinematicsEngine().formJacobianTranslation(aBody, aPoint, rJ, aRefBody);
}

void SimmModel::formJacobianOrientation(int aBody, double *rJ0, int aRefBody) const
{
	getKinematicsEngine().formJacobianOrientation(aBody, rJ0, aRefBody);
}

void SimmModel::formJacobianEuler(int aBody, double *rJE, int aRefBody) const
{
	getKinematicsEngine().formJacobianEuler(aBody, rJE, aRefBody);
}

//--------------------------------------------------------------------------
// DERIVATIVES
//--------------------------------------------------------------------------

int SimmModel::computeAccelerations(double *dqdt, double *dudt)
{
	return getKinematicsEngine().computeAccelerations(dqdt, dudt);
}

void SimmModel::computeAuxiliaryDerivatives(double *dydt)
{
	getKinematicsEngine().computeAuxiliaryDerivatives(dydt);
}

//--------------------------------------------------------------------------
// UTILITY
//--------------------------------------------------------------------------

void SimmModel::transform(int aBody1, const double aVec1[3], int aBody2, double rVec2[3]) const
{
	getKinematicsEngine().transform(aBody1, aVec1, aBody2, rVec2);
}

void SimmModel::transformPosition(int aBody, const double aPos[3], double rPos[3]) const
{
	getKinematicsEngine().transformPosition(aBody, aPos, rPos);
}

void SimmModel::convertQuaternionsToAngles(double *aQ, double *rQAng) const
{
	getKinematicsEngine().convertQuaternionsToAngles(aQ, rQAng);
}

void SimmModel::convertQuaternionsToAngles(Storage *rQStore) const
{
	getKinematicsEngine().convertQuaternionsToAngles(rQStore);
}

void SimmModel::convertAnglesToQuaternions(double *aQAng, double *rQ) const
{
	getKinematicsEngine().convertAnglesToQuaternions(aQAng, rQ);
}

void SimmModel::convertAnglesToQuaternions(Storage *rQStore) const
{
	getKinematicsEngine().convertAnglesToQuaternions(rQStore);
}

void SimmModel::convertRadiansToDegrees(double *aQRad, double *rQDeg) const
{
	getKinematicsEngine().convertRadiansToDegrees(aQRad, rQDeg);
}

void SimmModel::convertRadiansToDegrees(Storage *rQStore) const
{
	getKinematicsEngine().convertRadiansToDegrees(rQStore);
}

void SimmModel::convertDegreesToRadians(double *aQDeg, double *rQRad) const
{
	getKinematicsEngine().convertDegreesToRadians(aQDeg, rQRad);
}

void SimmModel::convertDegreesToRadians(Storage *rQStore) const
{
	getKinematicsEngine().convertDegreesToRadians(rQStore);
}

void SimmModel::convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double rDirCos[3][3]) const
{
	getKinematicsEngine().convertAnglesToDirectionCosines(aE1, aE2, aE3, rDirCos);
}

void SimmModel::convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double *rDirCos) const
{
	getKinematicsEngine().convertAnglesToDirectionCosines(aE1, aE2, aE3, rDirCos);
}

void SimmModel::convertDirectionCosinesToAngles(double aDirCos[3][3], double *rE1, double *rE2, double *rE3) const
{
	getKinematicsEngine().convertDirectionCosinesToAngles(aDirCos, rE1, rE2, rE3);
}

void SimmModel::convertDirectionCosinesToAngles(double *aDirCos, double *rE1, double *rE2, double *rE3) const
{
	getKinematicsEngine().convertDirectionCosinesToAngles(aDirCos, rE1, rE2, rE3);
}

void SimmModel::convertDirectionCosinesToQuaternions(double aDirCos[3][3], double *rQ1, double *rQ2, double *rQ3, double *rQ4) const
{
	getKinematicsEngine().convertDirectionCosinesToQuaternions(aDirCos, rQ1, rQ2, rQ3, rQ4);
}

void SimmModel::convertDirectionCosinesToQuaternions(double *aDirCos, double *rQ1, double *rQ2, double *rQ3, double *rQ4) const
{
	getKinematicsEngine().convertDirectionCosinesToQuaternions(aDirCos, rQ1, rQ2, rQ3, rQ4);
}

void SimmModel::convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double rDirCos[3][3]) const
{
	getKinematicsEngine().convertQuaternionsToDirectionCosines(aQ1, aQ2, aQ3, aQ4, rDirCos);
}

void SimmModel::convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double *rDirCos) const
{
	getKinematicsEngine().convertQuaternionsToDirectionCosines(aQ1, aQ2, aQ3, aQ4, rDirCos);
}

//--------------------------------------------------------------------------
// ACTUATION
//--------------------------------------------------------------------------
void SimmModel::computeActuation()
{
}

void SimmModel::applyActuatorForce(int aID)
{
}

void SimmModel::applyActuatorForces()
{
}

void SimmModel::setActuatorForce(int aID, double aForce)
{
}

double SimmModel::getActuatorForce(int aID) const
{
	return 0.0;
}

double SimmModel::getActuatorStress(int aID) const
{
	return 0.0;
}

double SimmModel::getActuatorSpeed(int aID) const
{
	return 0.0;
}

double SimmModel::getActuatorPower(int aID) const
{
	return 0.0;
}

//--------------------------------------------------------------------------
// CONTACT
//--------------------------------------------------------------------------
void SimmModel::computeContact()
{
	getKinematicsEngine().computeContact();
}

void SimmModel::applyContactForce(int aID)
{
	getKinematicsEngine().applyContactForce(aID);
}

void SimmModel::applyContactForces()
{
	getKinematicsEngine().applyContactForces();
}

int SimmModel::getContactBodyA(int aID) const
{
	return getKinematicsEngine().getContactBodyA(aID);
}

int SimmModel::getContactBodyB(int aID) const
{
	return getKinematicsEngine().getContactBodyB(aID);
}

void SimmModel::setContactPointA(int aID, const double aPoint[3])
{
	getKinematicsEngine().setContactPointA(aID, aPoint);
}

void SimmModel::getContactPointA(int aID, double rPoint[3]) const
{
	getKinematicsEngine().getContactPointA(aID, rPoint);
}

void SimmModel::setContactPointB(int aID, const double aPoint[3])
{
	getKinematicsEngine().setContactPointB(aID, aPoint);
}

void SimmModel::getContactPointB(int aID, double rPoint[3]) const
{
	getKinematicsEngine().getContactPointB(aID, rPoint);
}

void SimmModel::getContactForce(int aID, double rF[3]) const
{
	getKinematicsEngine().getContactForce(aID, rF);
}

void SimmModel::getContactNormalForce(int aID, double rFP[3], double rFV[3], double rF[3]) const
{
	getKinematicsEngine().getContactNormalForce(aID, rFP, rFV, rF);
}

void SimmModel::getContactTangentForce(int aID, double rFP[3], double rFV[3], double rF[3]) const
{
	getKinematicsEngine().getContactTangentForce(aID, rFP, rFV, rF);
}

void SimmModel::getContactStiffness(int aID, const double aDX[3], double rDF[3]) const
{
	getKinematicsEngine().getContactStiffness(aID, aDX, rDF);
}

void SimmModel::getContactViscosity(int aID, const double aDV[3], double rDF[3]) const
{
	getKinematicsEngine().getContactViscosity(aID, aDV, rDF);
}

void SimmModel::getContactFrictionCorrection(int aID, double aDFFric[3]) const
{
	getKinematicsEngine().getContactFrictionCorrection(aID, aDFFric);
}

double SimmModel::getContactForce(int aID) const
{
	return getKinematicsEngine().getContactForce(aID);
}

double SimmModel::getContactSpeed(int aID) const
{
	return getKinematicsEngine().getContactSpeed(aID);
}

double SimmModel::getContactPower(int aID) const
{
	return getKinematicsEngine().getContactPower(aID);
}
//--------------------------------------------------------------------------
// BODY INFO, to be implemented
//--------------------------------------------------------------------------

void SimmModel::setPin(int aBody,int aPinNumber,const double aPin[3])
{
	//return getKinematicsEngine().setPin(aBody, aPinNumber, aPin);
}

void SimmModel::getPin(int aBody,int aPinNumber,double rPin[3]) const
{
	//return getKinematicsEngine().getPin(aBody, aPinNumber, rPin);
}
void SimmModel::getJointInfo(int aJoint,int rInfo[50],int rSlider[6]) const
{
	// return getKinematicsEngine().getJointInfo(aJoint, rInfo, rSlider);
}
