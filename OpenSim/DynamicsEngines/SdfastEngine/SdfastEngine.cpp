// SdfastEngine.cpp
// Authors: Frank C. Anderson, Ayman Habib, and Peter Loan
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
#include <float.h>
#include <time.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/SQP/rdFSQP.h>
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Common/DebugUtilities.h>
#include "SdfastEngine.h"
#include "sdufuncs.h"
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/SpeedSet.h>
#include <OpenSim/Simulation/Model/JointSet.h>
#include <OpenSim/Simulation/Model/AbstractDof.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractMuscle.h>
#include <OpenSim/Common/Units.h>
#include <cassert>

//=============================================================================
// STATICS
//=============================================================================

using namespace std;
using namespace OpenSim;

const int SdfastEngine::GROUND = -1;

const double SdfastEngine::ASSEMBLY_TOLERANCE = 1e-7;
static char simmGroundName[] = "ground";

//=============================================================================
// SD/Fast function utilities
//=============================================================================

// Declare stub function for each desired SD/Fast function
#define OPENSIM_DECLARE_SDFAST_FUNCTION_PROTOTYPE(name, returntype, args) \
	returntype SDFAST_STUB_##name args { return returntype(); }
OPENSIM_FOR_ALL_SDFAST_FUNCTIONS(OPENSIM_DECLARE_SDFAST_FUNCTION_PROTOTYPE)

// Initalize function pointer to point to the stub function.  Called from setNull().
#define OPENSIM_INITIALIZE_SDFAST_FUNCTION_POINTER(name, returntype, args) \
	_##name = (FUNC_##name)SDFAST_STUB_##name;

// Link function pointer to function in library.  Called from linkToModelLibrary().
#define OPENSIM_LINK_SDFAST_FUNCTION_POINTER(name, returntype, args) \
	_##name = (FUNC_##name)GetProcAddress(modelLibrary, #name); \
	if(!_##name) { throw Exception("SdfastEngine.linkToModelLibrary: ERROR- did not find function "#name" in model library "+_modelLibraryName,__FILE__,__LINE__); }

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SdfastEngine::SdfastEngine() :
	AbstractDynamicsEngine(),
	_modelLibraryName(_modelLibraryNameProp.getValueStr())
{
	setNull();
	setupProperties();
	//init();  don't call init because model has no components???
}

//_____________________________________________________________________________
/**
 * Constructor from an XML Document
 */
SdfastEngine::SdfastEngine(const string &aFileName) :
	AbstractDynamicsEngine(aFileName),
	_modelLibraryName(_modelLibraryNameProp.getValueStr())
{
	setNull();
	setupProperties();
	updateFromXMLNode();
	//init();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */

SdfastEngine::~SdfastEngine()
{
	if (_y)
		delete [] _y;

	if (_dy)
		delete [] _dy;
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 */
SdfastEngine::SdfastEngine(const SdfastEngine& aEngine) :
   AbstractDynamicsEngine(aEngine),
	_modelLibraryName(_modelLibraryNameProp.getValueStr())
{
	// Should the copy constructor ever be called???
	setNull();
	setupProperties();
	copyData(aEngine);
}

//_____________________________________________________________________________
/**
 * Copy this kinematics engine and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SdfastEngine.
 */
Object* SdfastEngine::copy() const
{
	SdfastEngine *object = new SdfastEngine(*this);

	return object;
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Initialize the engine and the SD/FAST model.
 *
 */
void SdfastEngine::init(Model *aModel)
{
	_sdinit();

	// SYSTEM INFORMATION
	constructSystemVariables();

	_init_sdm();
	if(_setCoordinateInitialValues(getCoordinateSet()) < 0)
		throw Exception("SdfastEngine.init: ERR- setCoordinateInitialValues failed",__FILE__,__LINE__);
	if(_setJointConstraintFunctions(getCoordinateSet()) < 0)
		throw Exception("SdfastEngine.init: ERR- setCoordinateInitialValues failed",__FILE__,__LINE__);
	initializeState();
	prescribe();

	assemble();


	// setGravity and adjustJointVectorsForNewMassCenter should be called
	// before setConfiguration because they call sdinit which would clear
	// the state set in setConfiguration...

	setGravity(&_gravity[0]); // force sdgrav to be called

	// Center of mass locations
	// This adjusts the SDFast body-to-joint and inboard-to-joint vectors
	for(int i=0;i<_bodySet.getSize();i++) {
		adjustJointVectorsForNewMassCenter((SdfastBody*)_bodySet.get(i));
	}

	if(_y) setConfiguration(_y);
}

//_____________________________________________________________________________
/**
 * Construct system variables.
 *
 * The following numbers are initialized by making SD/Fast calls:
 *		_nb	Number of bodies.
 *		_nj	Number of joints.
 *		_nu	Number of degrees of freedom.
 *		_nq	Number of generalized coordinates (_nu + number of ball joints).
 *
 * Memory allocations are performed for the following data members:
 *		_q
 *		_u
 *
 * @return -1 on an error, 0 otherwise.
 */
void SdfastEngine::constructSystemVariables()
{
	// GET SYSTEM INFORMATION.
	int sysinfo[50];
	_sdinfo(sysinfo);

	// ASSIGN
	_numBodies = sysinfo[1] + 1; // ground counts as a body because it's in _bodySet
	_numUs = sysinfo[2];
	_numQs = _numUs + sysinfo[7];
	_numJoints = _numBodies + sysinfo[4] - 1; // subtract one because ground is in _numBodies

	if (_numBodies != _bodySet.getSize()) {
		cout << "Error: number of bodies in model file does not match number in SD/FAST code." << endl;
		// should be a fatal error
	}

	if (_numQs != _coordinateSet.getSize()) {
		cout << "Error: number of Qs in model file does not match number in SD/FAST code." << endl;
		// should be a fatal error
	}

	if (_numUs != _speedSet.getSize()) {
		cout << "Error: number of Us in model file does not match number in SD/FAST code." << endl;
		// should be a fatal error
	}

	if (_numJoints != _jointSet.getSize()) {
		cout << "Error: number of joints in model file does not match number in SD/FAST code." << endl;
		// should be a fatal error
	}

	_y = new double[_numQs + _numUs];
	_dy = new double[_numQs + _numUs];
}

//_____________________________________________________________________________
/**
 * Copy data members from one SdfastEngine to another.
 *
 * @param aEngine SdfastEngine to be copied.
 */
void SdfastEngine::copyData(const SdfastEngine &aEngine)
{
	_groundBody = aEngine._groundBody;
	_numBodies = aEngine._numBodies;
	_numQs = aEngine._numQs;
	_numUs = aEngine._numUs;
	_numJoints = aEngine._numJoints;
	if (_numQs > 0 || _numUs > 0) {
		_y = new double[_numQs + _numUs];
		_dy = new double[_numQs + _numUs];
		for (int i = 0; i < _numQs + _numUs; i++) {
			_y[i] = aEngine._y[i];
			_dy[i] = aEngine._dy[i];
		}
	} else {
		_y = _dy = NULL;
	}
}

//_____________________________________________________________________________
/**
 * Set NULL values for all the variable members of this class.
 */
void SdfastEngine::setNull()
{
	setType("SdfastEngine");

	_groundBody = NULL;
	_modelLibraryName = "";
	_numQs = 0;
	_numUs = 0;
	_numBodies = 0;
	_numJoints = 0;
	_y = NULL;
	_dy = NULL;

	OPENSIM_FOR_ALL_SDFAST_FUNCTIONS(OPENSIM_INITIALIZE_SDFAST_FUNCTION_POINTER);
}
//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this SdfastEngine.
 */
void SdfastEngine::linkToModelLibrary()
{
	if(_modelLibraryName == "")
		throw Exception("SdfastEngine.linkToModelLibrary: ERROR- Need to specify a model library name",__FILE__,__LINE__);

	std::cout << "Linking SdfastEngine to model library " << _modelLibraryName << std::endl;

	// Hook up pointers
	OPENSIM_PORTABLE_HINSTANCE modelLibrary = LoadOpenSimLibrary(_modelLibraryName.c_str(), true);
	if(modelLibrary==NULL)
		throw Exception("SdfastEngine.linkToModelLibrary: ERROR- Model library "+_modelLibraryName+" could not be loaded",__FILE__,__LINE__);

	OPENSIM_FOR_ALL_SDFAST_FUNCTIONS(OPENSIM_LINK_SDFAST_FUNCTION_POINTER);

	_setSdfastEngineInstance(this);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this SdfastEngine.
 */
void SdfastEngine::setup(Model* aModel)
{
	linkToModelLibrary();

	// Base class
	AbstractDynamicsEngine::setup(aModel);

	_groundBody = identifyGroundBody();

	init(aModel);
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
SdfastEngine& SdfastEngine::operator=(const SdfastEngine &aEngine)
{
	// BASE CLASS
	AbstractDynamicsEngine::operator=(aEngine);

	copyData(aEngine);

	//setup(aEngine._model);

	return(*this);
}

//=============================================================================
// TYPE REGISTRATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 *
 */
void SdfastEngine::setupProperties()
{
	_modelLibraryNameProp.setComment("Name of the sdfast model library to load.");
	_modelLibraryNameProp.setName("model_library");
	_propertySet.append(&_modelLibraryNameProp);
}

//_____________________________________________________________________________
/**
 * Register the types used by this class.
 */
void SdfastEngine::registerTypes()
{
	Object::RegisterType(SdfastBody());
	Object::RegisterType(SdfastJoint());
	Object::RegisterType(SdfastCoordinate());
	Object::RegisterType(SdfastSpeed());
}


//--------------------------------------------------------------------------
// ADDING COMPONENTS
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Add a body to the engine
 *
 * @param aBody pointer to the body to add
 */
void SdfastEngine::addBody(SdfastBody* aBody)
{
	/* This function assumes that ground is the first body
	 * added, so the indices start at -1.
	 */
	aBody->setSdfastIndex(_bodySet.getSize() - 1);
	_bodySet.append(aBody);
}

//_____________________________________________________________________________
/**
 * Add a joint to the engine
 *
 * @param aJoint pointer to the joint to add
 */
void SdfastEngine::addJoint(SdfastJoint* aJoint)
{
	aJoint->setup(this);
	aJoint->setSdfastIndex(_jointSet.getSize());
	_jointSet.append(aJoint);
}

//_____________________________________________________________________________
/**
 * Add a coordinate to the engine
 *
 * @param aCoord pointer to the coordinate to add
 */
void SdfastEngine::addCoordinate(SdfastCoordinate* aCoord)
{
	aCoord->setup(this);
	aCoord->setSdfastIndex(_coordinateSet.getSize());
	_coordinateSet.append(aCoord);
}

//_____________________________________________________________________________
/**
 * Add a speed to the engine
 *
 * @param aSpeed pointer to the speed to add
 */
void SdfastEngine::addSpeed(SdfastSpeed* aSpeed)
{
	aSpeed->setup(this);
	aSpeed->setSdfastIndex(_speedSet.getSize());
	_speedSet.append(aSpeed);
}

//--------------------------------------------------------------------------
// COORDINATES
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Update all coordinates in the model with the ones in the
 * passed-in coordinate set. If the coordinate does not exist
 * in the model, it is not added.
 *
 * @param aCoordinateSet set of coordinates to be updated/added
 */
void SdfastEngine::updateCoordinateSet(CoordinateSet& aCoordinateSet)
{
	for (int i = 0; i < aCoordinateSet.getSize(); i++) {
		AbstractCoordinate* modelCoordinate = _coordinateSet.get(aCoordinateSet.get(i)->getName());
		if (modelCoordinate)
			modelCoordinate->updateFromCoordinate(*aCoordinateSet.get(i));
	}

	cout << "Updated coordinates in model " << _model->getName() << endl;
}

//_____________________________________________________________________________
/**
 * Get the set of coordinates that are not locked
 *
 * @param rUnlockedCoordinates set of unlocked coordinates is returned here
 */
void SdfastEngine::getUnlockedCoordinates(CoordinateSet& rUnlockedCoordinates) const
{
	rUnlockedCoordinates.setSize(0);
	rUnlockedCoordinates.setMemoryOwner(false);

	for (int i = 0; i < _coordinateSet.getSize(); i++)
		if (!_coordinateSet.get(i)->getLocked())
			rUnlockedCoordinates.append(_coordinateSet.get(i));
}

//--------------------------------------------------------------------------
// SD/FAST FUNCTIONS
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Apply user-supplied forces to the model.
 *
 * This static method is intended to be called from the function
 * sduforce().  sduforce() is a "C" function and, therefore, doesn't know
 * about C++ objects.  Internally, this method uses a static pointer
 * to call the virtual methods Model::applyActuatorForces() and
 * Model::applyContactForces().  This method assumes that
 * computeActuation() and computeContact() have already been called.
 */
void SdfastEngine::sduforce()
{
	// Should not be called anymore
	OPENSIM_FUNCTION_NOT_IMPLEMENTED();
#if 0
	cout << "\n\nSdfastEngine::sduforce: ... SdfastEngine has called sduforce()!\n\n";

	_Instance->getModel()->getActuatorSet()->apply();
	_Instance->getModel()->getContactSet()->apply();

	//TODOAUG the user can implement restraints as actuators (active) or contacts (passive)
	//_Instance->applyRestraintTorques();

	//TODO ever? or are spring forces part of contact forces?
	//_Instance->applySpringForces();
#endif
}

//_____________________________________________________________________________
/**
 * Apply user-supplied motion to the model.
 *
 * This static method is intended to be called from the function
 * sdumotion().  sdumotion() is a "C" function and, therefore, doesn't know
 * about C++ objects.
 */
void SdfastEngine::sdumotion()
{
	OPENSIM_FUNCTION_NOT_IMPLEMENTED();
	cout << "\n\nSdfastEngine::sdumotion: ... SdfastEngine has called sdumotion()!\n\n";
}


//--------------------------------------------------------------------------
// CONFIGURATION
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the configuration (cooridnates and speeds) of the model.
 *
 * @param aY Array of coordinates followed by the speeds.
 */
void SdfastEngine::setConfiguration(const double aY[])
{
	int nq = getNumCoordinates();
	int nu = getNumSpeeds();
	int nqnu = nq + nu;
	memcpy(_y,aY,nqnu*sizeof(double));
	_sdstate(getModel()->getTime(),_y,&_y[nq]);

	// TODO: use Observer mechanism
	int i;
	ActuatorSet* act = getModel()->getActuatorSet();
	for (i = 0; i < act->getSize(); i++) {
		AbstractMuscle* sm = dynamic_cast<AbstractMuscle*>(act->get(i));
		if (sm)
			sm->invalidatePath();
	}
}
//_____________________________________________________________________________
/**
 * Get the configuration (cooridnates and speeds) of the model.
 *
 * @param rY Array of coordinates followed by the speeds.
 */
void SdfastEngine::getConfiguration(double rY[]) const
{
	int nq = getNumCoordinates();
	int nu = getNumSpeeds();
	int nqnu = nq+nu;
	memcpy(rY,_y,nqnu*sizeof(double));
}

//_____________________________________________________________________________
/**
 * Set the configuration (cooridnates and speeds) of the model.
 *
 * @param aQ Array of generalized coordinates.
 * @param aU Array of generalized speeds.
 */
void SdfastEngine::setConfiguration(const double aQ[],const double aU[])
{
	int nq = getNumCoordinates();
	int nu = getNumSpeeds();
	memcpy(_y,aQ,nq*sizeof(double));
	memcpy(&_y[nq],aU,nu*sizeof(double));
	_sdstate(_model->getTime(),_y,&_y[nq]);

	// TODO: use Observer mechanism
	int i;
	ActuatorSet* act = getModel()->getActuatorSet();
	for (i = 0; i < act->getSize(); i++) {
		AbstractMuscle* sm = dynamic_cast<AbstractMuscle*>(act->get(i));
		if (sm)
			sm->invalidatePath();
	}
}
//_____________________________________________________________________________
/**
 * Get the configuration (cooridnates and speeds) of the model.
 *
 * @param rQ Array of generalized coordinates.
 * @param rU Array of generalized speeds.
 */
void SdfastEngine::getConfiguration(double rQ[],double rU[]) const
{
	int nq = getNumCoordinates();
	int nu = getNumSpeeds();
	memcpy(rQ,_y,nq*sizeof(double));
	memcpy(rU,&_y[nq],nu*sizeof(double));
}

//_____________________________________________________________________________
/**
 * Get the values of the generalized coordinates.
 *
 * @param rQ Array of coordinates.
 */
void SdfastEngine::getCoordinates(double rQ[]) const
{
	int nq = getNumCoordinates();
	memcpy(rQ,_y,nq*sizeof(double));
}

//_____________________________________________________________________________
/**
 * Get the values of the generalized speeds.
 *
 * @param rU Array of speeds.
 */
void SdfastEngine::getSpeeds(double rU[]) const
{
	int nq = getNumCoordinates();
	int nu = getNumSpeeds();
	memcpy(rU,&_y[nq],nu*sizeof(double));
}

//_____________________________________________________________________________
/**
 * Get the last-computed values of the accelerations of the generalized
 * coordinates.  For the values to be valid, the method
 * computeAccelerations() must have been called.
 *
 * @param rDUDT Array to be filled with values of the accelerations of the
 * generalized coordinates.  The length of rDUDT should be at least as large
 * as the value returned by getNumSpeeds().
 * @see computeAccelerations()
 * @see getAcceleration(int aIndex)
 * @see getAcceleration(const char* aName);
 */
void SdfastEngine::getAccelerations(double rDUDT[]) const
{
	int nq = getNumCoordinates();
	int nu = getNumSpeeds();
	memcpy(rDUDT,&_dy[nq],nu*sizeof(double));
}
//_____________________________________________________________________________
/**
 * Extract the generalized coordinates and speeds from a combined array of
 * the coordinates and speeds.  This is only a utility method.  The
 * configuration of the model is not changed.
 *
 * @param aY Array of coordinates followed by the speeds.
 * @param rQ Array of coordinates taken from aY. 
 * @param rU Array of speeds taken from aY.
 */
void SdfastEngine::extractConfiguration(const double aY[],double rQ[],double rU[]) const
{
	int nq = getNumCoordinates();
	int nu = getNumSpeeds();
	memcpy(rQ,_y,nq*sizeof(double));
	memcpy(rU,&_y[nq],nu*sizeof(double));
}

//_____________________________________________________________________________
/**
 * applyDefaultConfiguration
 *
 */
void SdfastEngine::applyDefaultConfiguration()
{
	throw Exception("SdfastEngine::applyDefaultConfiguration() not yet implemented.");
}

//--------------------------------------------------------------------------
// ASSEMBLING THE MODEL
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * assemble
 *
 * @param aTime
 * @param rState
 * @param aLock
 * @param aTol
 * @param aMaxevals
 * @param rFcnt
 * @param rErr
 * @return 
 */
int SdfastEngine::assemble(double aTime, double *rState, int *aLock, double aTol, int aMaxevals, int *rFcnt, int *rErr)
{
	// TODOAUG: this method is not needed, assemble(void) takes care of it
	return 0;
}

//_____________________________________________________________________________
/**
 * Initialize the state vector with Q and U values from
 * the coordinate set. Initialize the derivative of the
 * state vector to zeros.
 */
void SdfastEngine::initializeState()
{
   int i;

	for(i=0; i<_coordinateSet.getSize(); i++)
		_y[i] = _coordinateSet[i]->getDefaultValue();

	for(i=0; i<_speedSet.getSize(); i++)
		_y[_numQs + i] = _speedSet[i]->getDefaultValue();

   for( i=0; i<(_numQs+_numUs); i++)
		_dy[i] = 0.0;
}

//_____________________________________________________________________________
/**
 * Set whether the motion of each coordinate is prescribed.
 */
void SdfastEngine::prescribe()
{
	int pres, desiredValue;

	for(int i=0; i<_coordinateSet.getSize(); i++) {
		SdfastCoordinate *coord = (SdfastCoordinate*)_coordinateSet.get(i);
		if((coord->getSdfastQType()==SdfastCoordinate::dpUnconstrained) ||
		(coord->getSdfastQType()==SdfastCoordinate::dpConstrained))
			desiredValue = 0;
		else
			desiredValue = 1;

		// You don't know if the prescribed state is changeable because
		// you don't know if a question mark was used in the SD/FAST
		// input file. So only try to set its value if the desired value
		// is different than the current one.
		_sdgetpres(coord->getJointIndex(),coord->getAxisIndex(),&pres);
		if(desiredValue!=pres)
			_sdpres(coord->getJointIndex(),coord->getAxisIndex(),desiredValue);
		//if (check_for_sderror("SET_PRESCRIBED_MOTION") == 19)
			//fprintf(stderr,"Unable to change prescribed state of %s\n", coord->getName());
	}
}

//_____________________________________________________________________________
/**
 * Assemble the model and make sure the gencoord velocities
 * are properly set.
 */
void SdfastEngine::assemble()
{
   int i, fcnt, err;
	int *lock = new int[_numQs];
	SdfastCoordinate *coord;

   for(i=0; i<_numQs; i++) {
		coord = (SdfastCoordinate*)_coordinateSet.get(i);
		if (coord->getLocked() || coord->getSdfastQType() == SdfastCoordinate::dpFixed)
         lock[i] = 1;
      else
         lock[i] = 0;
   }

   _sdassemble(_model->getTime(), _y, lock, ASSEMBLY_TOLERANCE, 500, &fcnt, &err);
   if (err) {
      cerr << "Assembly failed, err = " << err << endl;
      cerr << "Closest solution:" << endl;
		for (i = 0; i < _numQs; i++) {
			cerr << _coordinateSet.get(i)->getName() << " = " << _y[i] << " (" << lock[i] << ")" << endl;
		}
   }

   _sdinitvel(_model->getTime(), _y, lock, ASSEMBLY_TOLERANCE, 500, &fcnt, &err);
   if (err){
      cerr << "Velocity analysis failed." << endl;
      cerr << "Check that prescribed gencoord velocities are not being set to new values." << endl;
   }

   delete [] lock;
}

//--------------------------------------------------------------------------
// GRAVITY
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the gravity vector in the gloabl frame (and calls down to SD/Fast to
 * update its gravity vector)
 *
 * @param aGrav the XYZ gravity vector
 * @return Whether or not the gravity vector was successfully set.
 */
bool SdfastEngine::setGravity(double aGrav[3])
{
	AbstractDynamicsEngine::setGravity(aGrav);
	_sdgrav(aGrav);
	_sdinit(); // it is imporant to call this after sdgrav
	return true;
}

//--------------------------------------------------------------------------
// BODY INFORMATION
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the body that is being used as ground.
 *
 * @return Pointer to the ground body.
 */
AbstractBody& SdfastEngine::getGroundBody() const
{
	assert(_groundBody);
	return *_groundBody;
}
//_____________________________________________________________________________
/**
 * Get the SD/FAST index of the body that is being used as ground.
 *
 * @return the index
 */
int SdfastEngine::getGroundBodyIndex() const
{
	// Should probably call getGroundBody(), dynamic cast it to an SdfastBody,
	// and then get its index, but we know it should always be -1.
	return GROUND;
}
//_____________________________________________________________________________
/**
 * Determine which body should be treated as the ground body.
 *
 * @return Pointer to the ground body.
 */
AbstractBody* SdfastEngine::identifyGroundBody()
{
	// The ground body is the one that is named simmGroundName.
	for (int i = 0; i < _bodySet.getSize(); i++)
	{
		if (_bodySet.get(i)->getName() == simmGroundName)
			return _bodySet.get(i);
	}

	// If that name is not found, then the first body is selected as ground.
	if (_bodySet.getSize() > 0)
	{
		int j = 0;
		return _bodySet.get(j);
	}

	return NULL;
}
//_____________________________________________________________________________
/**
 * Get tree joint whose child is the given body.
 *
 * @param aBody Pointer to the body.
 */
SdfastJoint *SdfastEngine::getInboardTreeJoint(SdfastBody* aBody) const
{
	for(int i=0; i<_jointSet.getSize(); i++) {
		SdfastJoint *joint = dynamic_cast<SdfastJoint*>(_jointSet[i]);
		SdfastBody *child = joint->getChildBody();
		if((child==aBody) && joint->isTreeJoint())
			return joint;
	}
	return 0;
} 

//_____________________________________________________________________________
/**
 * Adjust to body-to-joint and inboard-to-joint vectors to account for the
 * changed center of mass location of an SD/Fast body
 *
 * @param aBody Pointer to the body.
 * @param aNewMassCenter New mass center location in the SIMM body frame.
 */
bool SdfastEngine::adjustJointVectorsForNewMassCenter(SdfastBody* aBody)
{

	// COMPUTE NEW BODY TO JOINT VECTOR FOR THE BODY
	// While it is true that the joint location is just the negaive of the mass center
	// in all SIMM models, this is not generally true.
	// A body could have a reference frame that does not coincide with its joint.
	// The code below accounts for this possibility.
	double newMassCenter[3],jointLocation[3],btj[3];
	aBody->getMassCenter(newMassCenter);
	SdfastJoint *btjJoint = getInboardTreeJoint(aBody);
	if(btjJoint!=NULL) {
		_sdgetbtj(btjJoint->getSdfastIndex(),btj);
		cout<<"Joint "<<btjJoint->getName()<<": origBTJ="<<btj[0]<<","<<btj[1]<<","<<btj[2];
		btjJoint->getLocationInChild(jointLocation);
		Mtx::Subtract(1,3,jointLocation,newMassCenter,btj);
		_sdbtj(btjJoint->getSdfastIndex(),btj);
		_sdgetbtj(btjJoint->getSdfastIndex(),btj);
		cout<<"  scaledBTJ = "<<btj[0]<<","<<btj[1]<<","<<btj[2]<<endl;
	}

	// COMPUTE NEW INBOARD TO JOINT VECTORS FOR ALL CHILD BODIES
	for (int i=0; i<_jointSet.getSize(); i++) {
		if(aBody != _jointSet.get(i)->getParentBody()) continue;
		double itj[3];
		SdfastJoint *itjJoint = (SdfastJoint*)_jointSet.get(i);
		_sdgetitj(itjJoint->getSdfastIndex(),itj);
		cout<<"Joint "<<itjJoint->getName()<<": origITJ="<<itj[0]<<","<<itj[1]<<","<<itj[2];
		itjJoint->getLocationInParent(jointLocation);
		Mtx::Subtract(1,3,jointLocation,newMassCenter,itj);
		_sditj(itjJoint->getSdfastIndex(),itj);
		_sdgetitj(itjJoint->getSdfastIndex(),itj);
		cout<<"  scaledITJ = "<<itj[0]<<","<<itj[1]<<","<<itj[2]<<endl;
	}

	_sdinit();

#if 0
	// Check for SD/FAST errors. If there are any, then a joint
	// vector must have been specified in the input file without
	// a '?', so the parameter cannot be changed.
	if (SDFAST_ERRORS)
		return false;
	// TODO: what if some vectors were changed and others had errors?
#endif
	return true;
}


//--------------------------------------------------------------------------
// INERTIA
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the total mass of the model
 *
 * @return the mass of the model
 */
double SdfastEngine::getMass() const
{
	double totalMass = 0.0;

	for (int i = 0; i < _bodySet.getSize(); i++)
		totalMass += _bodySet.get(i)->getMass();

	return totalMass;
}

//_____________________________________________________________________________
/**
 * getSystemInertia
 *
 * @param rM
 * @param rCOM
 * @param rI
 */
void SdfastEngine::getSystemInertia(double *rM, double rCOM[3], double rI[3][3]) const
{
	throw Exception("SdfastEngine::getSystemInertia(double *rM, double rCOM[3], double rI[3][3]) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * getSystemInertia
 *
 * @param rM
 * @param rCOM
 * @param rI
 */
void SdfastEngine::getSystemInertia(double *rM, double *rCOM, double *rI) const
{
	throw Exception("SdfastEngine::getSystemInertia(double *rM, double *rCOM, double *rI) not yet implemented.");
}

//--------------------------------------------------------------------------
// KINEMATICS
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the inertial position of a point on a body.
 *
 * Note that the configuration of the model must be set before calling this
 * method.
 *
 * @param aBody Pointer to body.
 * @param aPoint Point on the body expressed in the body-local frame.
 * @param rPos Position of the point in the inertial frame.
 *
 * @see setConfiguration()
 */
void SdfastEngine::getPosition(const AbstractBody &aBody, const double aPoint[3], double rPos[3]) const
{
	const SdfastBody* b = dynamic_cast<const SdfastBody*>(&aBody);

	if (b) {
		double sdpt[3];
		b->transformToSdfastFrame(aPoint, sdpt);
		_sdpos(b->getSdfastIndex(), sdpt, rPos);
	}
}

//_____________________________________________________________________________
/**
 * Get the inertial velocity of a point on a body.
 *
 * Note that the configuration of the model must be set before calling this
 * method.
 *
 * @param aBody Pointer to body.
 * @param aPoint Point on the body expressed in the body-local frame.
 * @param rVel Velocity of the point in the inertial frame.
 *
 * @see setConfiguration()
 */
void SdfastEngine::getVelocity(const AbstractBody &aBody, const double aPoint[3], double rVel[3]) const
{
	const SdfastBody* b = dynamic_cast<const SdfastBody*>(&aBody);

	if (b) {
		double sdpt[3];
		b->transformToSdfastFrame(aPoint, sdpt);
		_sdvel(b->getSdfastIndex(), sdpt, rVel);
	}
}

//_____________________________________________________________________________
/**
 * Get the inertial acceleration of a point on a body.
 *
 * Note that the configuration of the model must be set and accelerations of
 * the generalized coordinates must be computed before calling this method.
 *
 * @param aBody Pointer to body.
 * @param aPoint Point on the body expressed in the body-local frame.
 * @param rAcc Acceleration of the point in the inertial frame.
 *
 * @see set()
 * @see computeAccelerations()
 */
void SdfastEngine::getAcceleration(const AbstractBody &aBody, const double aPoint[3], double rAcc[3]) const
{
	const SdfastBody* b = dynamic_cast<const SdfastBody*>(&aBody);

	if (b) {
		double sdpt[3];
		b->transformToSdfastFrame(aPoint, sdpt);
		_sdacc(b->getSdfastIndex(), sdpt, rAcc);
	}
}

//_____________________________________________________________________________
/**
 * Get the body orientation with respect to the ground.
 *
 * @param aBody Pointer to body.
 * @param rDirCos Orientation of the body with respect to the ground frame.
 */
void SdfastEngine::getDirectionCosines(const AbstractBody &aBody, double rDirCos[3][3]) const
{
	const SdfastBody* b = dynamic_cast<const SdfastBody*>(&aBody);

	if (b)
		_sdorient(b->getSdfastIndex(), rDirCos);
}

//_____________________________________________________________________________
/**
 * Get the body orientation with respect to the ground.
 *
 * @param aBody Pointer to body.
 * @param rDirCos Orientation of the body with respect to the ground frame.
 */
void SdfastEngine::getDirectionCosines(const AbstractBody &aBody, double *rDirCos) const
{
	if (!rDirCos)
		return;

	const SdfastBody* b = dynamic_cast<const SdfastBody*>(&aBody);

	if (b)
	{
		double dirCos[3][3];

		_sdorient(b->getSdfastIndex(), dirCos);

		// Copy the direction cosines to the return vector.
		memcpy(rDirCos, &dirCos[0][0], 9 * sizeof(double));
	}
}

//_____________________________________________________________________________
/**
 * Get the inertial angular velocity of a body in the ground reference frame.
 *
 * @param aBody Pointer to body.
 * @param rAngVel Angular velocity of the body.
 */
void SdfastEngine::getAngularVelocity(const AbstractBody &aBody, double rAngVel[3]) const
{
	const SdfastBody *b = dynamic_cast<const SdfastBody*>(&aBody);

	if (b)
	{
		_sdangvel(b->getSdfastIndex(), rAngVel);
		_sdtrans(b->getSdfastIndex(), rAngVel, getGroundBodyIndex(), rAngVel);
	}
}

//_____________________________________________________________________________
/**
 * Get the inertial angular velocity in the local body reference frame.
 *
 * @param aBody Pointer to body.
 * @param rAngVel Angular velocity of the body.
 */
void SdfastEngine::getAngularVelocityBodyLocal(const AbstractBody &aBody, double rAngVel[3]) const
{
	const SdfastBody *b = dynamic_cast<const SdfastBody*>(&aBody);

	if (b)
		_sdangvel(b->getSdfastIndex(), rAngVel);
}

//_____________________________________________________________________________
/**
 * Get the inertial angular acceleration of a body in the ground reference 
 * frame.
 *
 * @param aBody Pointer to body.
 * @param rAngAcc Angular acceleration of the body.
 */
void SdfastEngine::getAngularAcceleration(const AbstractBody &aBody, double rAngAcc[3]) const
{
	const SdfastBody *b = dynamic_cast<const SdfastBody*>(&aBody);

	if (b)
	{
		_sdangacc(b->getSdfastIndex(), rAngAcc);
		_sdtrans(b->getSdfastIndex(), rAngAcc, getGroundBodyIndex(), rAngAcc);
	}
}

//_____________________________________________________________________________
/**
 * Get the inertial angular acceleration in the local body reference frame.
 *
 * @param aBody Pointer to body.
 * @param rAngAcc Angular acceleration of the body.
 */
void SdfastEngine::getAngularAccelerationBodyLocal(const AbstractBody &aBody, double rAngAcc[3]) const
{
	const SdfastBody *b = dynamic_cast<const SdfastBody*>(&aBody);

	if (b)
		_sdangacc(b->getSdfastIndex(), rAngAcc);
}

//_____________________________________________________________________________
/**
 * get a copy of the transform from the inertial frame to a body
 *
 * @param aBody
 * @return Transform from inertial frame to body
 */
Transform SdfastEngine::getTransform(const AbstractBody &aBody)
{
	Transform t;
	const SdfastBody* sdb = dynamic_cast<const SdfastBody*>(&aBody);

	if (sdb)
	{
		double opos1[3], opos2[3], dirCos[3][3], origin[] = {0.0, 0.0, 0.0};

		// 1. Get the coordinates of aBody's origin in its SD/FAST body's frame.
		sdb->transformToSdfastFrame(origin, opos1);

		// 2. Convert the origin of aBody into the inertial frame.
		_sdpos(sdb->getSdfastIndex(), opos1, opos2);

		// 3. Get the orientation of aBody in the inertial frame.
		_sdorient(sdb->getSdfastIndex(), dirCos);

		// 4. Fill in the transform with the translation and rotation.
		t.setPosition(opos2);
		Mtx::Transpose(3, 3, (double*)dirCos, (double*)dirCos);
		t.setOrientation(dirCos);
	}

	return t;
}

//--------------------------------------------------------------------------
// LOAD APPLICATION
//--------------------------------------------------------------------------
// FORCES EXPRESSED IN INERTIAL FRAME
//_____________________________________________________________________________
/**
 * Apply a force to a body
 *
 * @param aBody Body to apply force to
 * @param aPoint Point on body at which force is applied
 * @param aForce Force vector, expressed in inertial frame
 */
void SdfastEngine::applyForce(const AbstractBody &aBody, const double aPoint[3], const double aForce[3])
{
	int i;
	double p[3], f[3];
	const SdfastBody* sdBody = (const SdfastBody*)&aBody;

	sdBody->transformToSdfastFrame(aPoint,p);
	for (i=0; i<3; i++) {
		f[i] = aForce[i];
	}

	double force[3];
	_sdtrans(GROUND, f, sdBody->getSdfastIndex(), force);
	_sdpointf(sdBody->getSdfastIndex(), p, force);
}

//_____________________________________________________________________________
/**
 * Apply a set of forces to a set of bodies
 *
 * @param aN Number of forces to apply
 * @param aBodies Array of bodies
 * @param aPoints Array of points of application, expressed in body frames
 * @param aForces Array of forces to apply, expressed in the inertial frame
 */
void SdfastEngine::applyForces(int aN, const AbstractBody *aBodies[], const double aPoints[][3], const double aForces[][3])
{
	int i;

	for (i = 0; i < aN; i++)
		applyForce(*aBodies[i], aPoints[i], aForces[i]);
}

//_____________________________________________________________________________
/**
 * Apply a set of forces to a set of bodies
 *
 * @param aN Number of forces to apply
 * @param aBodies Array of bodies
 * @param aPoints Array of points of application, expressed in body frames
 * @param aForces Array of forces to apply, expressed in the inertial frame
 */
void SdfastEngine::applyForces(int aN, const AbstractBody *aBodies[], const double *aPoints, const double *aForces)
{
	// Check that the input vectors have been defined.
	if (!aBodies || !aPoints || !aForces)
		return;

	int i, I;

	for (i = 0; i < aN; i++)
	{
		I = Mtx::ComputeIndex(i, 3, 0);
		applyForce(*aBodies[i], &aPoints[I], &aForces[I]);
	}
}

//_____________________________________________________________________________
/**
 * Apply a force to a body
 *
 * @param aBody Body to apply force to
 * @param aPoint Point on body at which to apply force
 * @param aForce Force to apply, expressed in body frame
 */
void SdfastEngine::applyForceBodyLocal(const AbstractBody &aBody, const double aPoint[3], const double aForce[3])
{
	const SdfastBody* sdBody = dynamic_cast<const SdfastBody*>(&aBody);

	// aBody must be an SdfastBody in order to apply a force to it
	if (sdBody)
	{
		int i;
		double p[3], f[3];

		sdBody->transformToSdfastFrame(aPoint, p);
		for (i = 0; i < 3; i++)
		{
			f[i] = aForce[i];
		}

		_sdpointf(sdBody->getSdfastIndex(), p, f);
	}
}

//_____________________________________________________________________________
/**
 * Apply a set of forces to a set of bodies
 *
 * @param aN Number of forces to apply
 * @param aBodies Array of bodies
 * @param aPoints Array of points of application, expressed in body frames
 * @param aForces Array of forces to apply, expressed in the body frames
 */
void SdfastEngine::applyForcesBodyLocal(int aN, const AbstractBody *aBodies[], const double aPoints[][3], const double aForces[][3])
{
	int i;

	for (i = 0; i < aN; i++)
		applyForce(*aBodies[i], aPoints[i], aForces[i]);
}

//_____________________________________________________________________________
/**
 * Apply a set of forces to a set of bodies
 *
 * @param aN Number of forces to apply
 * @param aBodies Array of bodies
 * @param aPoints Array of points of application, expressed in body frames
 * @param aForces Array of forces to apply, expressed in the body frames
 */
void SdfastEngine::applyForcesBodyLocal(int aN, const AbstractBody *aBodies[], const double *aPoints, const double *aForces)
{
	// Check that the input vectors have been defined.
	if (!aBodies || !aPoints || !aForces)
		return;

	int i;
	double point[3], force[3];

	for (i = 0; i < aN; i++)
	{
		point[0] = *aPoints++;
		point[1] = *aPoints++;
		point[2] = *aPoints;

		force[0] = *aForces++;
		force[1] = *aForces++;
		force[2] = *aForces;

		applyForceBodyLocal(*aBodies[i], point, force);
	}
}

//_____________________________________________________________________________
/**
 * Apply a torque expressed in the inertial frame to a body.
 *
 * @param aBody Pointer to body.
 * @param aTorque Torque expressed in the inertial frame.
 */
void SdfastEngine::applyTorque(const AbstractBody &aBody, const double aTorque[3])
{
	const SdfastBody* b = dynamic_cast<const SdfastBody*>(&aBody);

	if (b)
	{
		int i;
		double torque[3];

		for (i = 0 ; i < 3; i++)
			torque[i] = aTorque[i];

		_sdtrans(getGroundBodyIndex(), torque, b->getSdfastIndex(), torque);
		_sdbodyt(b->getSdfastIndex(), torque);
	}
}

//_____________________________________________________________________________
/**
 * Apply a set of torques expressed in the inertial frame to a set of bodies.
 *
 * @param aN Number of Torques.
 * @param aBody Array of Body pointers.
 * @param aTorques Array of torques applied to the body expressed the inertial 
 * frame.
 */
void SdfastEngine::applyTorques(int aN, const AbstractBody *aBodies[], const double aTorques[][3])
{
	int i;

	for (i = 0; i < aN; i++)
		applyTorque(*aBodies[i], aTorques[i]);
}

//_____________________________________________________________________________
/**
 * Apply a set of torques expressed in the inertial frame to a set of bodies.
 *
 * @param aN Number of Torques.
 * @param aBody Array of Body pointers.
 * @param aTorques Array of torques applied to the body expressed the inertial 
 * frame.
 */
void SdfastEngine::applyTorques(int aN, const AbstractBody *aBodies[], const double *aTorques)
{
	if (aTorques)
	{
		int i;
		double torque[3];

		for (i = 0; i < aN; i++)
		{
			torque[0] = *(aTorques++);
			torque[1] = *(aTorques++);
			torque[2] = *(aTorques++);

			applyTorque(*aBodies[i], torque);
		}
	}
}

// TORQUES EXPRESSED IN BODY-LOCAL FRAME (sdbodyt())
//_____________________________________________________________________________
/**
 * Apply a torque expressed in the body-local frame to a body.
 *
 * @param aBody Pointer to body.
 * @param aTorque Torque expressed in the body-local frame.
 */
void SdfastEngine::applyTorqueBodyLocal(const AbstractBody &aBody, const double aTorque[3])
{
	const SdfastBody *b = dynamic_cast<const SdfastBody*>(&aBody);

	if (b)
	{
		int i;
		double torque[3];

		for (i = 0; i < 3; i++)
			torque[i] = aTorque[i];

		_sdbodyt(b->getSdfastIndex(), torque);
	}
}

//_____________________________________________________________________________
/**
 * Apply a set of torques expressed in the body-local frame to a set of bodies.
 *
 * @param aN Number of Torques.
 * @param aBody Array of Body pointers.
 * @param aTorques Array of torques applied to the body expressed the 
 * body-local frame.
 */
void SdfastEngine::applyTorquesBodyLocal(int aN, const AbstractBody *aBodies[], const double aTorques[][3])
{
	int i;

	for (i = 0; i< aN; i++)
		applyTorqueBodyLocal(*aBodies[i], aTorques[i]);
}

//_____________________________________________________________________________
/**
 * Apply a set of torques expressed in the body-local frame to a set of bodies.
 *
 * @param aN Number of Torques.
 * @param aBody Array of Body pointers.
 * @param aTorques Array of torques applied to the body expressed the 
 * body-local frame.
 */
void SdfastEngine::applyTorquesBodyLocal(int aN, const AbstractBody *aBodies[], const double *aTorques)
{
	if (aTorques)
	{
		int i;
		double torque[3];

		for (i = 0; i < aN; i++)
		{
			torque[0] = *(aTorques++);
			torque[1] = *(aTorques++);
			torque[2] = *(aTorques++);

			applyTorqueBodyLocal(*aBodies[i], torque);
		}
	}
}

// GENERALIZED FORCES
//_____________________________________________________________________________
/**
 * Apply a generalized force to a generalized coordinate.
 * Note that depending on the axis type the generalized force can be a
 * torque or a force.
 * @param aU Generalized coordinate.
 * @param aF Applied force.
 */
void SdfastEngine::applyGeneralizedForce(const AbstractCoordinate &aU, double aF)
{
	const SdfastCoordinate *c = dynamic_cast<const SdfastCoordinate*>(&aU);

	if (c)
		_sdhinget(c->getJointIndex(), c->getAxisIndex(), aF);
}

//_____________________________________________________________________________
/**
 * Apply generalized forces.
 * The dimension of aF is assumed to be the number of generalized coordinates.
 * @param aF Applied force.
 */
void SdfastEngine::applyGeneralizedForces(const double aF[])
{
	int i;

	for (i = 0; i < _coordinateSet.getSize(); i++)
		applyGeneralizedForce(*_coordinateSet.get(i), aF[i]);
}

//_____________________________________________________________________________
/**
 * Apply generalized forces.
 * @param aN Number of generalized forces.
 * @param aU Generalized coordinates.
 * @param aF Applied forces.
 */
void SdfastEngine::applyGeneralizedForces(int aN, const AbstractCoordinate *aU[], const double aF[])
{
	int i;

	for (i = 0; i < aN; i++)
		applyGeneralizedForce(*aU[i], aF[i]);
}

//--------------------------------------------------------------------------
// LOAD ACCESS AND COMPUTATION
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the net applied generalized force.  The returned force is the sum of
 * all applied forces plus any forces needed for any prescribed motions.
 * The methods setState() (or equivalent) and computeAccelerations() must
 * be called prior to calling this method for the returned result to be
 * valid.
 *
 * @param aU Generalized speed (degree of freedom).
 * @return Net applied force/torque at degree of freedom aU.
 */
double SdfastEngine::getNetAppliedGeneralizedForce(const AbstractCoordinate &aU) const
{
	const SdfastCoordinate *c = dynamic_cast<const SdfastCoordinate*>(&aU);

	if (c)
	{
		double f;

		_sdgetht(c->getJointIndex(), c->getAxisIndex(), &f);

		return f;
	}

	return 0.0;
}

//_____________________________________________________________________________
/**
 * Compute the generalized forces necessary to achieve a set of specified
 * accelerations.  If any forces have been applied to the model, the balance
 * of generalized forces needed to achieve the desired accelerations is
 * computed.  Note that constraints are not taken into account by this
 * method.
 *
 * @param aDUDT Array of desired accelerations of the generalized coordinates-
 * should be dimensioned to NU (see getNumSpeeds()).
 * @param rF Array of generalized forces that will achieve aDUDT without
 * enforcing any constraints- should be dimensioned to NU (see getNumSpeeds()).
 */
void SdfastEngine::computeGeneralizedForces(double aDUDT[], double rF[]) const
{
	_sdcomptrq(aDUDT, rF);
}

//_____________________________________________________________________________
/**
 * Compute the reaction forces and torques at all the joints in the model.
 *
 * It is necessary to call computeAccelerations() before this method
 * to get valid results.  This method is expensive to call, beyond the
 * expense of computing the accelerations.  So, this method should be
 * called as infrequently as possible.
 *
 * @param rForces Matrix of reaction forces.  The size should be
 * at least NumberOfJoints x 3.
 * @param rTorques Matrix of reaction torques.  The size should be
 * at least NumberOfJoints x 3.
 */
void SdfastEngine::computeReactions(double rForces[][3], double rTorques[][3]) const
{
	_sdreac(rForces, rTorques);
}


//--------------------------------------------------------------------------
// CONSTRAINTS
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the constrained coordinates and speeds for a multibody system.
 *
 * @param y Array of coordinates followed by the speeds.  Only the first
 * nu+nu elements are accessed, where nu = getNumSpeeds().  The rotational
 * coordinates and speeds are assumed to be in radians and radians/sec.
 * Given the values of the unconstrained coordinates and speeds in y, the
 * constrained coordinates and speeds in y are over-written with their
 * constrained values.
 */
void SdfastEngine::
computeConstrainedCoordinates(double *y) const
{
	_compute_constrained_coords(y);
}


//--------------------------------------------------------------------------
// EQUATIONS OF MOTION
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Form the system mass matrix.
 *
 * @param rI Mass matrix (a square matrix of size NU*NU).
 */
void SdfastEngine::formMassMatrix(double *rI)
{
	_sdmassmat(rI);
}

//_____________________________________________________________________________
/**
 * Form the transformation matrix E[3][3] that can be used to express the
 * angular velocity of a body in terms of the time derivatives of the euler
 * angles.  The Euler angle convention is body-fixed 1-2-3.
 * @param aBody Body ID.
 * @param rE Euler angles.
 */
void SdfastEngine::formEulerTransform(const AbstractBody &aBody, double *rE) const
{
	const SdfastBody *b = dynamic_cast<const SdfastBody*>(&aBody);

	if (b && rE)
	{
		// GET ORIENTATION OF aBody
		double ang[3], dc[3][3];

		_sdorient(b->getSdfastIndex(), dc);	
		_sddc2ang(dc, &ang[0], &ang[1], &ang[2]);

		// ROW 1
		*rE =  cos(ang[2]) / cos(ang[1]);
		rE++;  *rE = -sin(ang[2]) / cos(ang[1]);
		rE++;  *rE = 0.0;

		// ROW 2
		rE++;  *rE = sin(ang[2]);
		rE++;  *rE = cos(ang[2]);
		rE++;  *rE = 0.0;

		// ROW 3
		rE++;  *rE = -cos(ang[2]) * sin(ang[1]) / cos(ang[1]);
		rE++;  *rE =  sin(ang[1]) * sin(ang[2]) / cos(ang[1]);
		rE++;  *rE = 1.0;
	}
}

//_____________________________________________________________________________
/**
 * formJacobianTranslation
 *
 * @param aBody
 * @param aPoint
 * @param rJ
 * @param aRefBody
 */
void SdfastEngine::formJacobianTranslation(const AbstractBody &aBody, const double aPoint[3], double *rJ, const AbstractBody *aRefBody) const
{
	throw Exception("SdfastEngine::formJacobianTranslation(const AbstractBody &aBody, const double aPoint[3], double *rJ, const AbstractBody *aRefBody) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * formJacobianOrientation
 *
 * @param aBody
 * @param rJ0
 * @param aRefBody
 */
void SdfastEngine::formJacobianOrientation(const AbstractBody &aBody, double *rJ0, const AbstractBody *aRefBody) const
{
	throw Exception("SdfastEngine::formJacobianOrientation(const AbstractBody &aBody, double *rJ0, const AbstractBody *aRefBody) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * formJacobianEuler
 *
 * @param aBody
 * @param rJE
 * @param aRefBody
 */
void SdfastEngine::formJacobianEuler(const AbstractBody &aBody, double *rJE, const AbstractBody *aRefBody) const
{
	throw Exception("SdfastEngine::formJacobianEuler(const AbstractBody &aBody, double *rJE, const AbstractBody *aRefBody) not yet implemented.");
}

//--------------------------------------------------------------------------
// DERIVATIVES
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the derivatives of the generalized coordinates and speeds.
 *
 * @param dqdt Derivatives of generalized coordinates.
 * @param dudt Derivatives of generalized speeds.
 */
void SdfastEngine::computeDerivatives(double *dqdt,double *dudt)
{
	_sdderiv(dqdt,dudt);
	int nq = getNumCoordinates();
	int nu = getNumSpeeds();
	memcpy(_dy,dqdt,nq*sizeof(double));
	memcpy(&_dy[nq],dudt,nu*sizeof(double));
}


//--------------------------------------------------------------------------
// UTILITY
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Transform a vector from one body to another
 *
 * @param aBodyFrom the body in which the vector is currently expressed
 * @param aPos the vector to be transformed
 * @param aBodyTo the body the vector will be transformed into
 * @param rPos the vector in the aBodyTo frame is returned here
 */
void SdfastEngine::transform(const AbstractBody &aBodyFrom, const double aVec[3], const AbstractBody &aBodyTo, double rVec[3]) const
{
	int i;

	for(i=0; i<3; i++)
		rVec[i] = aVec[i];

	if(&aBodyFrom == &aBodyTo) return;

	const SdfastBody* b1 = dynamic_cast<const SdfastBody*>(&aBodyFrom);
	const SdfastBody* b2 = dynamic_cast<const SdfastBody*>(&aBodyTo);

	if(b1 && b2)
		_sdtrans(b1->getSdfastIndex(),(double*)aVec,b2->getSdfastIndex(),rVec);
}

//_____________________________________________________________________________
/**
 * Transform a vector from one body to another
 *
 * @param aBodyFrom the body in which the vector is currently expressed
 * @param aPos the vector to be transformed
 * @param aBodyTo the body the vector will be transformed into
 * @param rPos the vector in the aBodyTo frame is returned here
 */
void SdfastEngine::transform(const AbstractBody &aBodyFrom, const Array<double>& aVec, const AbstractBody &aBodyTo, Array<double>& rVec) const
{
	int i;

	for(i=0; i<3; i++)
		rVec[i] = aVec[i];

	if(&aBodyFrom == &aBodyTo)
		return;

	const SdfastBody* b1 = dynamic_cast<const SdfastBody*>(&aBodyFrom);
	const SdfastBody* b2 = dynamic_cast<const SdfastBody*>(&aBodyTo);

	if (b1 && b2) {
		double aVec2[3], rVec2[3];
		aVec2[0] = aVec[0];
		aVec2[1] = aVec[1];
		aVec2[2] = aVec[2];
		_sdtrans(b1->getSdfastIndex(), aVec2, b2->getSdfastIndex(), rVec2);
		rVec[0] = rVec2[0];
		rVec[1] = rVec2[1];
		rVec[2] = rVec2[2];
	}
}

//_____________________________________________________________________________
/**
 * Transform a point from one body to another
 *
 * @param aBodyFrom the body in which the point is currently expressed
 * @param aPos the XYZ coordinates of the point
 * @param aBodyTo the body the point will be transformed into
 * @param rPos the XYZ coordinates of the point in the aBodyTo frame are returned here
 */
void SdfastEngine::transformPosition(const AbstractBody &aBodyFrom, const double aPos[3], const AbstractBody &aBodyTo, double rPos[3]) const
{
	int i;

	for (i = 0; i < 3; i++)
		rPos[i] = aPos[i];

	if (&aBodyFrom == &aBodyTo)
		return;

	const SdfastBody* b1 = dynamic_cast<const SdfastBody*>(&aBodyFrom);
	const SdfastBody* b2 = dynamic_cast<const SdfastBody*>(&aBodyTo);

	if (b1 && b2)
	{
		double sdpt[3], sdpt2[3], dirCos[3][3], origin[] = {0.0, 0.0, 0.0};

		// 1. Convert the point into the SD/FAST frame of aBodyFrom.
		b1->transformToSdfastFrame(aPos, sdpt);

		// 2. Convert the point into the ground frame.
		_sdpos(b1->getSdfastIndex(), sdpt, sdpt2);

		// 3. Subtract the coordinates of aBodyTo's origin. This
		// creates a vector from the origin of aBodyTo to the point,
		// expressed in the ground frame.
		_sdpos(b2->getSdfastIndex(), origin, rPos);
		for (i = 0; i < 3; i++)
			sdpt2[i] -= rPos[i];

		// 4. Get the orientation of aBodyTo in the ground frame.
		_sdorient(b2->getSdfastIndex(), dirCos);

		// 5. Convert the point into the SD/FAST frame of aBodyTo.
		Mtx::Multiply(1, 3, 3, sdpt2, &dirCos[0][0], sdpt);

		// 6. Convert the point into the body frame of aBodyTo.
		b2->transformFromSdfastFrame(sdpt, rPos);
	}
}

//_____________________________________________________________________________
/**
 * Transform a point from one body to another
 *
 * @param aBodyFrom the body in which the point is currently expressed
 * @param aPos the XYZ coordinates of the point
 * @param aBodyTo the body the point will be transformed into
 * @param rPos the XYZ coordinates of the point in the aBodyTo frame are returned here
 */
void SdfastEngine::transformPosition(const AbstractBody &aBodyFrom, const Array<double>& aPos, const AbstractBody &aBodyTo, Array<double>& rPos) const
{
	int i;
	double aPos2[3], rPos2[3];

	for (i = 0; i < 3; i++)
		aPos2[i] = aPos[i];

	transformPosition(aBodyFrom, aPos2, aBodyTo, rPos2);

	for (i = 0; i < 3; i++)
		rPos[i] = rPos2[i];
}

//_____________________________________________________________________________
/**
 * Transform a point from one body to the ground body
 *
 * @param aBodyFrom the body in which the point is currently expressed
 * @param aPos the XYZ coordinates of the point
 * @param rPos the XYZ coordinates of the point in the ground frame are returned here
 */
void SdfastEngine::transformPosition(const AbstractBody &aBodyFrom, const double aPos[3], double rPos[3]) const
{
	int i;
	double pos[3];

	for (i = 0; i < 3; i++)
	{
		pos[i] = aPos[i];
		rPos[i] = aPos[i];
	}

	AbstractBody& ground = getGroundBody();

	if (&aBodyFrom == &ground)
		return;

	const SdfastBody* from = dynamic_cast<const SdfastBody*>(&aBodyFrom);

	if (from)
	{
		double sdPos[3];

		from->transformToSdfastFrame(pos, sdPos);
		_sdpos(from->getSdfastIndex(), sdPos, rPos);
	}
}

//_____________________________________________________________________________
/**
 * Transform a point from one body to the ground body
 *
 * @param aBodyFrom the body in which the point is currently expressed
 * @param aPos the XYZ coordinates of the point
 * @param rPos the XYZ coordinates of the point in the ground frame are returned here
 */
void SdfastEngine::transformPosition(const AbstractBody &aBodyFrom, const Array<double>& aPos, Array<double>& rPos) const
{
	int i;
	double aPos2[3], rPos2[3];

	for (i = 0; i < 3; i++)
		aPos2[i] = aPos[i];

	transformPosition(aBodyFrom, aPos2, rPos2);

	for (i = 0; i < 3; i++)
		rPos[i] = rPos2[i];
}

//_____________________________________________________________________________
/**
 * Calculate the distance between a point on one body and a point on another body
 *
 * @param aBody1 the body that the first point is expressed in
 * @param aPoint1 the XYZ coordinates of the first point
 * @param aBody2 the body that the second point is expressed in
 * @param aPoint2 the XYZ coordinates of the second point
 * @return the distance between aPoint1 and aPoint2
 */
double SdfastEngine::calcDistance(const AbstractBody& aBody1, const Array<double>& aPoint1, const AbstractBody& aBody2, const Array<double>& aPoint2) const
{
	const SdfastBody* b1 = dynamic_cast<const SdfastBody*>(&aBody1);
	const SdfastBody* b2 = dynamic_cast<const SdfastBody*>(&aBody2);

	if (b1 && b2)
	{
		double sdpt1[3], sdpt2[3], pos1[3], pos2[3];

		b1->transformToSdfastFrame(aPoint1, sdpt1);
		b2->transformToSdfastFrame(aPoint2, sdpt2);

		_sdpos(b1->getSdfastIndex(), sdpt1, pos1);
		_sdpos(b2->getSdfastIndex(), sdpt2, pos2);

		return sqrt((pos1[0] - pos2[0])*(pos1[0] - pos2[0]) + (pos1[1] - pos2[1])*(pos1[1] - pos2[1]) +
			         (pos1[2] - pos2[2])*(pos1[2] - pos2[2]));
	}

	return 0.0;
}

//_____________________________________________________________________________
/**
 * Calculate the distance between a point on one body and a point on another body
 *
 * @param aBody1 the body that the first point is expressed in
 * @param aPoint1 the XYZ coordinates of the first point
 * @param aBody2 the body that the second point is expressed in
 * @param aPoint2 the XYZ coordinates of the second point
 * @return the distance between aPoint1 and aPoint2
 */
double SdfastEngine::calcDistance(const AbstractBody& aBody1, const double aPoint1[3], const AbstractBody& aBody2, const double aPoint2[3]) const
{
	const SdfastBody* b1 = dynamic_cast<const SdfastBody*>(&aBody1);
	const SdfastBody* b2 = dynamic_cast<const SdfastBody*>(&aBody2);

	if (b1 && b2)
	{
		double sdpt1[3], sdpt2[3], pos1[3], pos2[3];

		b1->transformToSdfastFrame(aPoint1, sdpt1);
		b2->transformToSdfastFrame(aPoint2, sdpt2);

		_sdpos(b1->getSdfastIndex(), sdpt1, pos1);
		_sdpos(b2->getSdfastIndex(), sdpt2, pos2);

		return sqrt((pos1[0] - pos2[0])*(pos1[0] - pos2[0]) + (pos1[1] - pos2[1])*(pos1[1] - pos2[1]) +
			         (pos1[2] - pos2[2])*(pos1[2] - pos2[2]));
	}

	return 0.0;
}

//_____________________________________________________________________________
/**
 * Convert quaterions to angles.
 *
 * @param aQ Array of generalized coordinates, some of which may be
 * quaternions.  The length of aQ must be at least getNumCoordinates().
 * @param rQAng Array of equivalent angles.
 */
void SdfastEngine::convertQuaternionsToAngles(double *aQ, double *rQAng) const
{
	_sdst2ang(aQ,rQAng);
}

//_____________________________________________________________________________
/**
 * For all the generalized coordinates held in a storage object, convert the
 * generalized coordinates expressed in quaternions to Euler angles.
 *
 * @param rQStore Storage object of generalized coordinates, some of which
 * may be quaternions.  The length of each state-vector in rQStore must be
 * at least getNumCoordinates().
 */
void SdfastEngine::convertQuaternionsToAngles(Storage *rQStore) const
{
	if(rQStore==NULL) return;

	// NUMBER OF Q'S
	int nq = getNumCoordinates();
	int nu = getNumSpeeds();
	int dn = nq - nu;
	if(nq<=0) {
		printf("rdSDFast.convertQuaternionsToAngles: ERROR- models has ");
		printf("no generalized coordinates.\n");
		return;
	}

	// LOOP THROUGH STATE-VECTORS
	int i;
	int size,newSize=0;
	double t,*data,*newData=NULL;
	StateVector *vec;
	for(i=0;i<rQStore->getSize();i++) {

		// GET STATE-VECTOR
		vec = rQStore->getStateVector(i);
		if(vec==NULL) continue;

		// CHECK SIZE
		size = vec->getSize();
		if(size<nq) {
			printf("rdSDFast.convertQuaternionsToAngles: WARN- the size of ");
			printf("a state-vector is less than nq(%d).\n",nq);
			continue;
		}

		// GET DATA
		t = vec->getTime();
		data = vec->getData().get();
		if(data==NULL) continue;

		// ALLOCATE NEW DATA IF NECESSARY
		if(newSize<(size-dn)) {
			if(newData!=NULL) delete[] newData;
			newSize = size-dn;
			newData = new double[newSize];
		}

		// CONVERT QUATERNIONS TO ANGLES
		convertQuaternionsToAngles(data,newData);

		// FILL IN THE REST OF THE DATA
		for(i=nu;i<(size-dn);i++) {
			newData[i] = data[i+dn];
		}

		// CHANGE THE STATE-VECTOR
		vec->setStates(t,newSize,newData);
	}

	// CHANGE THE COLUMN LABELS
	cout<<"rdSDFast.convertQuaternionsToAngles: NOTE- the column labels"<<
		" for "<<rQStore->getName()<<" were not changed."<<endl;

	// CLEANUP
	if(newData!=NULL) delete[] newData;
}

//_____________________________________________________________________________
/**
 * Convert angles to quaterions.
 *
 * @param aQAng Array of generalized coordinates expressed in Euler angles.
 * The length of aQAng must be at least getNumSpeeds().
 * @param rQ Vector of equivalent quaternions.
 */
void SdfastEngine::convertAnglesToQuaternions(double *aQAng, double *rQ) const
{
	_sdang2st(aQAng,rQ);
}

//_____________________________________________________________________________
/**
 * For all the generalized coordinates held in a storage object, convert the
 * generalized coordinates expressed in Euler angles to quaternions when
 * appropriate.
 *
 * @param rQStore Storage object of generalized coordinates that has all
 * angles expressed as Euler angles in radians.  The length of each
 * state-vector in rQStore must be at least getNumSpeeds().
 */
void SdfastEngine::convertAnglesToQuaternions(Storage *rQStore) const
{
	if(rQStore==NULL) return;

	// NUMBER OF Q'S
	int nq = getNumCoordinates();
	int nu = getNumSpeeds();
	int dn = nq - nu;
	if(nu<=0) {
		printf("rdSDFast.convertAnglesToQuaternions: ERROR- models has ");
		printf("no generalized coordinates.\n");
		return;
	}
	if(dn<=0) return;

	// LOOP THROUGH THE STATE-VECTORS
	int i,j;
	int size,newSize=0;
	double t,*data,*newData=NULL;
	StateVector *vec;
	for(i=0;i<rQStore->getSize();i++) {

		// GET STATE-VECTOR
		vec = rQStore->getStateVector(i);
		if(vec==NULL) continue;

		// CHECK SIZE
		size = vec->getSize();
		if(size<nu) {
			printf("rdSDFast.convertAnglesToQuaternions: WARN- the size of ");
			printf("a state-vector is less than nu(%d).\n",nu);
			continue;
		}

		// GET DATA
		t = vec->getTime();
		data = vec->getData().get();
		if(data==NULL) continue;

		// ALLOCATE NEW DATA IF NECESSARY
		if(newSize<(size+dn)) {
			if(newData!=NULL) delete[] newData;
			newSize = size+dn;
			newData = new double[newSize];
		}

		// CONVERT QUATERNIONS TO ANGLES
		convertAnglesToQuaternions(data,newData);

		// FILL IN THE REST OF THE DATA
		for(j=nu;j<size;j++) {
			newData[j+dn] = data[j];
		}

		// CHANGE THE STATE-VECTOR
		vec->setStates(t,newSize,newData);
	}

	// CHANGE THE COLUMN LABELS
	cout<<"rdSDFast.convertAnglesToQuaternions: NOTE- the column labels"<<
		" for "<<rQStore->getName()<<" were not changed."<<endl;

	// CLEANUP
	if(newData!=NULL) delete[] newData;
}

//_____________________________________________________________________________
/**
 * Convert angles to direction cosines.
 * @param aE1 1st Euler angle.
 * @param aE2 2nd Euler angle.
 * @param aE3 3rd Euler angle.
 * @param rDirCos Vector of direction cosines.
 */
void SdfastEngine::convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double rDirCos[3][3]) const
{
	_sdang2dc(aE1,aE2,aE3,rDirCos);
}

//_____________________________________________________________________________
/**
 * Convert angles to direction cosines.
 * @param aE1 1st Euler angle.
 * @param aE2 2nd Euler angle.
 * @param aE3 3rd Euler angle.
 * @param rDirCos Vector of direction cosines.
 */
void SdfastEngine::convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double *rDirCos) const
{
	if (rDirCos==NULL) return;

	double dirCos[3][3];

	_sdang2dc(aE1,aE2,aE3,dirCos);

	// Assign the returned values to the return vector.
	memcpy(rDirCos,&dirCos[0][0],9*sizeof(double));
}

//_____________________________________________________________________________
/**
 * Convert direction cosines to angles.
 * @param aDirCos Vector of direction cosines.
 * @param rE1 1st Euler angle.
 * @param rE2 2nd Euler angle.
 * @param rE3 3rd Euler angle.
 */
void SdfastEngine::convertDirectionCosinesToAngles(double aDirCos[3][3], double *rE1, double *rE2, double *rE3) const
{
	_sddc2ang(aDirCos,rE1,rE2,rE3);
}

//_____________________________________________________________________________
/**
 * Convert direction cosines to angles.
 * @param aDirCos Vector of direction cosines.
 * @param rE1 1st Euler angle.
 * @param rE2 2nd Euler angle.
 * @param rE3 3rd Euler angle.
 */
void SdfastEngine::convertDirectionCosinesToAngles(double *aDirCos, double *rE1, double *rE2, double *rE3) const
{
	if (!aDirCos)
		return;

	double dirCos[3][3];

	// Copy the input vector to the local one.
	memcpy(&dirCos[0][0], aDirCos, 9 * sizeof(double));

	_sddc2ang(dirCos, rE1, rE2, rE3);
}

//_____________________________________________________________________________
/**
 * Convert direction cosines to quaternions.
 * @param aDirCos Vector of direction cosines.
 * @param rQ1 1st Quaternion.
 * @param rQ2 2nd Quaternion.
 * @param rQ3 3rd Quaternion.
 * @param rQ4 4th Quaternion.
 */
void SdfastEngine::convertDirectionCosinesToQuaternions(double aDirCos[3][3], double *rQ1, double *rQ2, double *rQ3, double *rQ4) const
{
	_sddc2quat(aDirCos,rQ1,rQ2,rQ3,rQ4);
}

//_____________________________________________________________________________
/**
 * Convert direction cosines to quaternions.
 * @param aDirCos Vector of direction cosines.
 * @param rQ1 1st Quaternion.
 * @param rQ2 2nd Quaternion.
 * @param rQ3 3rd Quaternion.
 * @param rQ4 4th Quaternion.
 */
void SdfastEngine::convertDirectionCosinesToQuaternions(double *aDirCos, double *rQ1, double *rQ2, double *rQ3, double *rQ4) const
{
	if (aDirCos==NULL) return;

	double dirCos[3][3];

	// Copy the input vector to the local one.
	memcpy(&dirCos[0][0],aDirCos,9*sizeof(double));

	_sddc2quat(dirCos,rQ1,rQ2,rQ3,rQ4);
}

//_____________________________________________________________________________
/**
 * Convert quaternions to direction cosines.
 * @param aQ1 1st Quaternion.
 * @param aQ2 2nd Quaternion.
 * @param aQ3 3rd Quaternion.
 * @param aQ4 4th Quaternion.
 * @param rDirCos Vector of direction cosines.
 */
void SdfastEngine::convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double rDirCos[3][3]) const
{
	_sdquat2dc(aQ1,aQ2,aQ3,aQ4,rDirCos);
}

//_____________________________________________________________________________
/**
 * Convert quaternions to direction cosines.
 * @param aQ1 1st Quaternion.
 * @param aQ2 2nd Quaternion.
 * @param aQ3 3rd Quaternion.
 * @param aQ4 4th Quaternion.
 * @param rDirCos Vector of direction cosines.
 */
void SdfastEngine::convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double *rDirCos) const
{
	if (rDirCos==NULL) return;

	double dirCos[3][3];

	_sdquat2dc(aQ1,aQ2,aQ3,aQ4,dirCos);

	// Assign the returned values to the return vector.
	memcpy(rDirCos,&dirCos[0][0],9*sizeof(double));
}

void SdfastEngine::peteTest() const
{
	int i;

	cout << "Kinematics Engine:" << endl;

	if (_bodySet.getSize() < 1)
	{
		cout << "no bodies" << endl;
	}
	else
	{
		for (i = 0; i < _bodySet.getSize(); i++)
			_bodySet.get(i)->peteTest();
	}

	if (_coordinateSet.getSize() < 1)
	{
		cout << "no coordinates" << endl;
	}
	else
	{
		for (i = 0; i < _coordinateSet.getSize(); i++)
			_coordinateSet.get(i)->peteTest();
	}

	if (_speedSet.getSize() < 1)
	{
		cout << "no speeds" << endl;
	}
	else
	{
		for (i = 0; i < _speedSet.getSize(); i++)
			_speedSet.get(i)->peteTest();
	}

	if (_jointSet.getSize() < 1)
	{
		cout << "no joints" << endl;
	}
	else
	{
		for (i = 0; i < _jointSet.getSize(); i++)
			_jointSet.get(i)->peteTest();
	}

}

//=============================================================================
// STATIC METHOD FOR CREATING THIS MODEL
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
OSIMSDFASTENGINE_API Model*
CreateModel()
{
	Model *model = new Model();
	return(model);
}

//_____________________________________________________________________________
/**
 * Deserialization from file.
 */
OSIMSDFASTENGINE_API Model*
CreateModel_File(const string &aModelFile)
{
	Object::RegisterType(SdfastEngine());
	SdfastEngine::registerTypes();
	Model *model = new Model(aModelFile);
	return(model);
}
