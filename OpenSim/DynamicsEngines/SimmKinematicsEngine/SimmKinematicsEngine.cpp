// SimmKinematicsEngine.cpp
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
#include <OpenSim/Common/Memory.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/SQP/rdFSQP.h>
#include <OpenSim/Common/SimmMacros.h>
#include "SimmKinematicsEngine.h"
#include <OpenSim/Simulation/Model/AbstractModel.h>
#include <OpenSim/Simulation/Model/AbstractBody.h>
#include <OpenSim/Simulation/Model/AbstractCoordinate.h>
#include <OpenSim/Simulation/Model/AbstractJoint.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/Model/Marker.h>
#include <OpenSim/Simulation/Model/AbstractDof.h>
#include "SimmCoordinate.h"
#include "SimmBody.h"
#include "SimmJoint.h"
#include "SimmStep.h"
#include "SimmTranslationDof.h"
#include "SimmRotationDof.h"
#include <OpenSim/Common/Units.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/SpeedSet.h>
#include <OpenSim/Simulation/Model/JointSet.h>

//=============================================================================
// STATICS
//=============================================================================

using namespace std;
using namespace OpenSim;

static char simmGroundName[] = "ground";

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmKinematicsEngine::SimmKinematicsEngine() :
	AbstractDynamicsEngine(),
	_path(0)
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Constructor from an XML Document
 */
SimmKinematicsEngine::SimmKinematicsEngine(const string &aFileName) :
	AbstractDynamicsEngine(aFileName),
	_path(0)
{
	setNull();
	setupProperties();
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */

SimmKinematicsEngine::~SimmKinematicsEngine()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 */
SimmKinematicsEngine::SimmKinematicsEngine(const SimmKinematicsEngine& aEngine) :
   AbstractDynamicsEngine(aEngine),
	_path(0)
{
	setNull();
	setupProperties();
	copyData(aEngine);
}

//_____________________________________________________________________________
/**
 * Copy this kinematics engine and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmKinematicsEngine.
 */
Object* SimmKinematicsEngine::copy() const
{
	SimmKinematicsEngine *object = new SimmKinematicsEngine(*this);

	return object;
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SimmKinematicsEngine to another.
 *
 * @param aEngine SimmKinematicsEngine to be copied.
 */
void SimmKinematicsEngine::copyData(const SimmKinematicsEngine &aEngine)
{
	// TODO- Should we copy _path and _groundBody ?
}

//_____________________________________________________________________________
/**
 * Set NULL values for all the variable members of this class.
 */
void SimmKinematicsEngine::setNull()
{
	setType("SimmKinematicsEngine");

	_path.initTable(0);
	_groundBody = NULL;
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this SimmKinematicsEngine.
 */
void SimmKinematicsEngine::setup(AbstractModel* aModel)
{
	// Base class
	AbstractDynamicsEngine::setup(aModel);

	_groundBody = identifyGroundBody();

	/* Generate paths from every body to every other one. */
	makePaths();

	/* For each coordinate, generate a list of the joints that use it. */
	createCoordinateJointLists();

	/* For each coordinate, generate a list of the paths that use it. */
	createCoordinatePathLists();

	// Now that the coordinate-joint lists have been created, you can
	// use them to determine the type (rotational/translational) of each
	// coordinate.
	int i;
	for (i = 0; i < _coordinateSet.getSize(); i++)
		_coordinateSet[i]->determineType();
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
SimmKinematicsEngine& SimmKinematicsEngine::operator=(const SimmKinematicsEngine &aEngine)
{
	// BASE CLASS
	AbstractDynamicsEngine::operator=(aEngine);

	copyData(aEngine);

	//setup(aEngine._model);

	return(*this);
}

//=============================================================================
// PROPERTIES AND REGISTRATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 *
 */
void SimmKinematicsEngine::setupProperties()
{
}

//_____________________________________________________________________________
/**
 * Register the types used by this class.
 */
void SimmKinematicsEngine::registerTypes()
{
	Object::RegisterType(SimmBody());
	Object::RegisterType(SimmCoordinate());
	Object::RegisterType(SimmJoint());
	Object::RegisterType(NatCubicSpline());
	Object::RegisterType(Constant());
	Object::RegisterType(GCVSpline());
	Object::RegisterType(SimmTranslationDof());
	Object::RegisterType(SimmRotationDof());
	Object::RegisterType(Marker());
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
void SimmKinematicsEngine::updateCoordinateSet(CoordinateSet& aCoordinateSet)
{
	CoordinateSet *coordSet = getCoordinateSet();
	for (int i=0; i<aCoordinateSet.getSize(); i++) {
		AbstractCoordinate* modelCoordinate = coordSet->get(aCoordinateSet.get(i)->getName());
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
void SimmKinematicsEngine::getUnlockedCoordinates(CoordinateSet& rUnlockedCoordinates) const
{
	rUnlockedCoordinates.setSize(0);
	rUnlockedCoordinates.setMemoryOwner(false);

	for (int i = 0; i < _coordinateSet.getSize(); i++)
		if (!_coordinateSet.get(i)->getLocked())
			rUnlockedCoordinates.append(_coordinateSet.get(i));
}


//--------------------------------------------------------------------------
// CONFIGURATION
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the configuration of the model (q's and u's).
 *
 * @param aY Array of generalized coordinates followed by the generalized
 * speeds
 */
void SimmKinematicsEngine::setConfiguration(const double aY[])
{
	int nq = getNumCoordinates();
	setConfiguration(aY,&aY[nq]);
}
//_____________________________________________________________________________
/**
 * Get the configuration of the model (q's and u's).
 *
 * @param rY Array of generalized coordinates and speeds.
 */
void SimmKinematicsEngine::getConfiguration(double rY[]) const
{
	int nq = getNumCoordinates();
	getConfiguration(rY,&rY[nq]);
}

//_____________________________________________________________________________
/**
 * Set the configuration of the model (q's and u's).
 *
 * @param aQ Array of generalized coordinates.
 * @param aU Array of generalized speeds.
 */
void SimmKinematicsEngine::setConfiguration(const double aQ[], const double aU[])
{
	int i;
	int nq = getNumCoordinates();
	int nu = getNumSpeeds();

	// q's
	for (i=0; i<nq; i++) {
		_coordinateSet[i]->setValue(aQ[i]);
	}

	// u's
	for (i=0; i<nu; i++) {
		_speedSet[i]->setValue(aU[i]);
	}
}
//_____________________________________________________________________________
/**
 * Get the configuration of the model (q's and u's).
 *
 * @param rQ Array of generalized coordinates.
 * @param rU Array of generalized speeds.
 */
void SimmKinematicsEngine::getConfiguration(double rQ[],double rU[]) const
{
	int i;
	int nq = getNumCoordinates();
	int nu = getNumSpeeds();

	// q's
	for (i=0; i<nq; i++) {
		rQ[i] = _coordinateSet[i]->getValue();
	}

	// u's
	for (i=0; i<nu; i++) {
		rU[i] = _speedSet[i]->getValue();
	}
}

//_____________________________________________________________________________
/**
 * getCoordinateValues
 *
 * @param rQ Array of coordinate values
 */
void SimmKinematicsEngine::getCoordinates(double rQ[]) const
{
	int nq = getNumCoordinates();
	for (int i=0; i<nq; i++)
		rQ[i] = _coordinateSet[i]->getValue();
}

//_____________________________________________________________________________
/**
 * getSpeedValues
 *
 * @param rU Array of speeds
 */
void SimmKinematicsEngine::getSpeeds(double rU[]) const
{
	int nu = getNumSpeeds();
	for (int i=0; i<nu; i++)
		rU[i] = _speedSet[i]->getValue();
}

//_____________________________________________________________________________
/**
 * getAccelerations
 *
 * @param rDUDT Array of accelerations (should be empty)
 */
void SimmKinematicsEngine::getAccelerations(double rDUDT[]) const
{
	// There are no accelerations in a SimmKinematicsEngine
}

//_____________________________________________________________________________
/**
 * getAcceleration
 *
 * @param aIndex There are no accelerations in a SimmKinematicsEngine
 * @return 
 */
double SimmKinematicsEngine::getAcceleration(int aIndex) const
{
	// There are no accelerations in a SimmKinematicsEngine
	return rdMath::NAN;
}

//_____________________________________________________________________________
/**
 * getAcceleration
 *
 * @param aSpeedName There are no accelerations in a SimmKinematicsEngine
 * @return 
 */
double SimmKinematicsEngine::getAcceleration(const string &aSpeedName) const
{
	// There are no accelerations in a SimmKinematicsEngine
	return rdMath::NAN;
}

//_____________________________________________________________________________
/**
 * Extract the coordinates and speeds from a single array.  The configuration
 * of the model is not changed.
 *
 * @param aY Array of generalized coordinates followed by the generalized
 * speeds.
 * @param rQ Array containing copies of the generalized coodinates in aY.
 * @param rU Array containing copies of the generalized speeds in aY.
 */
void SimmKinematicsEngine::extractConfiguration(const double aY[],double rQ[],double rU[]) const
{
	int nq = getNumCoordinates();
	int nu = getNumSpeeds();
	memcpy(rQ,aY,nq*sizeof(double));
	memcpy(rU,&aY[nq],nu*sizeof(double));
}

//_____________________________________________________________________________
/**
 * applyDefaultConfiguration
 *
 */
void SimmKinematicsEngine::applyDefaultConfiguration()
{
	for (int i = 0; i < _coordinateSet.getSize(); i++)
		_coordinateSet.get(i)->setValue(_coordinateSet.get(i)->getDefaultValue());

	for (int i = 0; i < _speedSet.getSize(); i++)
		_speedSet.get(i)->setValue(_speedSet.get(i)->getDefaultValue());
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
int SimmKinematicsEngine::assemble(double aTime, double *rState, int *aLock, double aTol, int aMaxevals, int *rFcnt, int *rErr)
{
	throw Exception("SimmKinematicsEngine::assemble(double aTime, double *rState, int *aLock, double aTol, int aMaxevals, int *rFcnt, int *rErr) not yet implemented.");
	return 0;
}

//--------------------------------------------------------------------------
// SCALING
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Scale the kinematics engine
 *
 * @param aScaleSet the set of XYZ scale factors for the bodies
 * @param aFinalMass the mass that the scaled model should have
 * @param aPreserveMassDist whether or not the masses of the
 *        individual bodies should be scaled with the body scale factors.
 * @return Whether or not scaling was successful.
 */
bool SimmKinematicsEngine::scale(const ScaleSet& aScaleSet, double aFinalMass, bool aPreserveMassDist)
{
	// Base class
	AbstractDynamicsEngine::scale(aScaleSet, aFinalMass, aPreserveMassDist);

	// Invalidate all of the paths
	_path.invalidate();

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
AbstractBody& SimmKinematicsEngine::getGroundBody() const
{
	assert(_groundBody);
	return *_groundBody;
}

//_____________________________________________________________________________
/**
 * Determine which body should be treated as the ground body.
 *
 * @return Pointer to the ground body.
 */
AbstractBody* SimmKinematicsEngine::identifyGroundBody()
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

//--------------------------------------------------------------------------
// INERTIA
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the total mass of the model
 *
 * @return the mass of the model
 */
double SimmKinematicsEngine::getMass() const
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
void SimmKinematicsEngine::getSystemInertia(double *rM, double rCOM[3], double rI[3][3]) const
{
	throw Exception("SimmKinematicsEngine::getSystemInertia(double *rM, double rCOM[3], double rI[3][3]) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * getSystemInertia
 *
 * @param rM
 * @param rCOM
 * @param rI
 */
void SimmKinematicsEngine::getSystemInertia(double *rM, double *rCOM, double *rI) const
{
	throw Exception("SimmKinematicsEngine::getSystemInertia(double *rM, double *rCOM, double *rI) not yet implemented.");
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
 * @param aBody the body the point is expressed in.
 * @param aPoint Point on the body expressed in the body-local frame.
 * @param rPos Position of the point in the inertial frame.
 *
 * @see setConfiguration()
 */
void SimmKinematicsEngine::getPosition(const AbstractBody& aBody, const double aPoint[3], double rPos[3]) const
{
	rPos[0] = aPoint[0];
	rPos[1] = aPoint[1];
	rPos[2] = aPoint[2];

	AbstractBody& groundBody = getGroundBody();

	if (&aBody == &groundBody)
		return;

	SimmPath* sp = _path.getSimmPath(&aBody, &groundBody);

	if (sp)
	{
		Transform& transform = sp->getInverseTransform();
		transform.transformPoint(rPos);
	}
	else
	{
		string errorMessage = "SimmKinematicsEngine::getPosition: Could not find path from ground to " + aBody.getName();
		throw Exception(errorMessage);
	}
}

//_____________________________________________________________________________
/**
 * getVelocity
 *
 * @param aBody
 * @param aPoint
 * @param rVel
 */
void SimmKinematicsEngine::getVelocity(const AbstractBody &aBody, const double aPoint[3], double rVel[3]) const
{
	throw Exception("SimmKinematicsEngine::getVelocity(const AbstractBody &aBody, const double aPoint[3], double rVel[3]) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * getAcceleration
 *
 * @param aBody
 * @param aPoint
 * @param rAcc
 */
void SimmKinematicsEngine::getAcceleration(const AbstractBody &aBody, const double aPoint[3], double rAcc[3]) const
{
	throw Exception("SimmKinematicsEngine::getAcceleration(const AbstractBody &aBody, const double aPoint[3], double rAcc[3]) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * Get the body orientation with respect to the ground.
 *
 * @param aBody the Body.
 * @param rDirCos Orientation of the body with respect to the ground frame.
 */
void SimmKinematicsEngine::getDirectionCosines(const AbstractBody &aBody, double rDirCos[3][3]) const
{
	AbstractBody& groundBody = getGroundBody();

	if (&aBody == &groundBody)
	{
		rDirCos[0][0] = rDirCos[1][1] = rDirCos[2][2] = 1.0;
		rDirCos[0][1] = rDirCos[0][2] = rDirCos[1][0] = 0.0;
		rDirCos[1][2] = rDirCos[2][0] = rDirCos[2][1] = 0.0;
		return;
	}

	SimmPath* sp = _path.getSimmPath(&groundBody, &aBody);

	if (sp)
	{
		Transform& transform = sp->getForwardTransform();	

		int i, j;
		double *mat = transform.getMatrix();

		for (i = 0; i < 3; i++)
			for (j = 0; j < 3; j++)
				rDirCos[i][j] = mat[i * 4 + j];
	}
	else
	{
		string errorMessage = "SimmKinematicsEngine::getDirectionCosines: Could not find path from ground to " + aBody.getName();
		throw Exception(errorMessage);
	}
}

//_____________________________________________________________________________
/**
 * Get the body orientation with respect to the ground.
 *
 * @param aBody the Body.
 * @param rDirCos Orientation of the body with respect to the ground frame.
 */
void SimmKinematicsEngine::getDirectionCosines(const AbstractBody &aBody, double *rDirCos) const
{
	AbstractBody& groundBody = getGroundBody();

	if (&aBody == &groundBody)
	{
		rDirCos[0] = rDirCos[4] = rDirCos[8] = 1.0;
		rDirCos[1] = rDirCos[2] = rDirCos[3] = 0.0;
		rDirCos[5] = rDirCos[6] = rDirCos[7] = 0.0;
		return;
	}

	SimmPath* sp = _path.getSimmPath(&groundBody, &aBody);

	if (sp)
	{
		Transform& transform = sp->getForwardTransform();	

		double *mat = transform.getMatrix();

		rDirCos[0] = mat[0];
		rDirCos[1] = mat[1];
		rDirCos[2] = mat[2];
		rDirCos[3] = mat[4];
		rDirCos[4] = mat[5];
		rDirCos[5] = mat[6];
		rDirCos[6] = mat[8];
		rDirCos[7] = mat[9];
		rDirCos[8] = mat[10];
	}
	else
	{
		string errorMessage = "SimmKinematicsEngine::getDirectionCosines: Could not find path from ground to " + aBody.getName();
		throw Exception(errorMessage);
	}
}

//_____________________________________________________________________________
/**
 * getAngularVelocity
 *
 * @param aBody
 * @param rAngVel
 */
void SimmKinematicsEngine::getAngularVelocity(const AbstractBody &aBody, double rAngVel[3]) const
{
	throw Exception("SimmKinematicsEngine::getAngularVelocity(const AbstractBody &aBody, double rAngVel[3]) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * getAngularVelocityBodyLocal
 *
 * @param aBody
 * @param rAngVel
 */
void SimmKinematicsEngine::getAngularVelocityBodyLocal(const AbstractBody &aBody, double rAngVel[3]) const
{
	throw Exception("SimmKinematicsEngine::getAngularVelocityBodyLocal(const AbstractBody &aBody, double rAngVel[3]) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * getAngularAcceleration
 *
 * @param aBody
 * @param rAngAcc
 */
void SimmKinematicsEngine::getAngularAcceleration(const AbstractBody &aBody, double rAngAcc[3]) const
{
	throw Exception("SimmKinematicsEngine::getAngularAcceleration(const AbstractBody &aBody, double rAngAcc[3]) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * getAngularAccelerationBodyLocal
 *
 * @param aBody
 * @param rAngAcc
 */
void SimmKinematicsEngine::getAngularAccelerationBodyLocal(const AbstractBody &aBody, double rAngAcc[3]) const
{
	throw Exception("SimmKinematicsEngine::getAngularAccelerationBodyLocal(const AbstractBody &aBody, double rAngAcc[3]) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * get a copy of the transform from the inertial frame to a body
 *
 * @param aBody
 * @return Transform from inertial frame to body
 */
Transform SimmKinematicsEngine::getTransform(const AbstractBody &aBody)
{
	AbstractBody& groundBody = getGroundBody();

	if (&aBody == &groundBody)
		return Transform();  // return a default transform (identity matrix)

	SimmPath* sp = _path.getSimmPath(&groundBody, &aBody);

	if (sp)
	{
		return sp->getForwardTransform();
	}
	else
	{
		string errorMessage = "SimmKinematicsEngine::getTransform: Could not find path from ground to " + aBody.getName();
		throw Exception(errorMessage);
	}
}

//--------------------------------------------------------------------------
// LOAD APPLICATION
//--------------------------------------------------------------------------
// FORCES EXPRESSED IN INERTIAL FRAME
//_____________________________________________________________________________
/**
 * applyForce
 *
 * @param aBody
 * @param aPoint
 * @param aForce
 */
void SimmKinematicsEngine::applyForce(const AbstractBody &aBody, const double aPoint[3], const double aForce[3])
{
	throw Exception("SimmKinematicsEngine::applyForce(const AbstractBody &aBody, const double aPoint[3], const double aForce[3]) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * applyForces
 *
 * @param aN
 * @param aBodies
 * @param aPoints
 * @param aForces
 */
void SimmKinematicsEngine::applyForces(int aN, const AbstractBody *aBodies[], const double aPoints[][3], const double aForces[][3])
{
	throw Exception("SimmKinematicsEngine::applyForces(int aN, const AbstractBody *aBodies[], const double aPoints[][3], const double aForces[][3]) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * applyForces
 *
 * @param aN
 * @param aBodies
 * @param aPoints
 * @param aForces
 */
void SimmKinematicsEngine::applyForces(int aN, const AbstractBody *aBodies[], const double *aPoints, const double *aForces)
{
	throw Exception("SimmKinematicsEngine::applyForces(int aN, const AbstractBody *aBodies[], const double *aPoints, const double *aForces) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * applyForceBodyLocal
 *
 * @param aBody
 * @param aPoint
 * @param aForce
 */
// FORCES EXPRESSED IN BODY-LOCAL FRAME
void SimmKinematicsEngine::applyForceBodyLocal(const AbstractBody &aBody, const double aPoint[3], const double aForce[3])
{
#if 0
	something like this
	// TO ALLOW CONSTANT ARGUMENTS
	int i;
	double p[3],f[3];
	for(i=0;i<3;i++) {
		p[i] = aPoint[i];
		f[i] = aForce[i];
	}

	// APPLY FORCE
	sdpointf(aBody,p,f);
#endif
}

//_____________________________________________________________________________
/**
 * applyForcesBodyLocal
 *
 * @param aN
 * @param aBodies
 * @param aPoints
 * @param aForces
 */
void SimmKinematicsEngine::applyForcesBodyLocal(int aN, const AbstractBody *aBodies[], const double aPoints[][3], const double aForces[][3])
{
	throw Exception("SimmKinematicsEngine::applyForcesBodyLocal(int aN, const AbstractBody *aBodies[], const double aPoints[][3], const double aForces[][3]) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * applyForcesBodyLocal
 *
 * @param aN
 * @param aBodies
 * @param aPoints
 * @param aForces
 */
void SimmKinematicsEngine::applyForcesBodyLocal(int aN, const AbstractBody *aBodies[], const double *aPoints, const double *aForces)
{
	throw Exception("SimmKinematicsEngine::applyForcesBodyLocal(int aN, const AbstractBody *aBodies[], const double *aPoints, const double *aForces) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * applyTorque
 *
 * @param aBody
 * @param aTorque
 */
// TORQUES EXPRESSED IN INERTIAL FRAME
void SimmKinematicsEngine::applyTorque(const AbstractBody &aBody, const double aTorque[3])
{
	throw Exception("SimmKinematicsEngine::applyTorque(const AbstractBody &aBody, const double aTorque[3]) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * applyTorques
 *
 * @param aN
 * @param aBodies
 * @param aTorques
 */
void SimmKinematicsEngine::applyTorques(int aN, const AbstractBody *aBodies[], const double aTorques[][3])
{
	throw Exception("SimmKinematicsEngine::applyTorques(int aN, const AbstractBody *aBodies[], const double aTorques[][3]) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * applyTorques
 *
 * @param aN
 * @param aBodies
 * @param aTorques
 */
void SimmKinematicsEngine::applyTorques(int aN, const AbstractBody *aBodies[], const double *aTorques)
{
	throw Exception("SimmKinematicsEngine::applyTorques(int aN, const AbstractBody *aBodies[], const double *aTorques) not yet implemented.");
}

// TORQUES EXPRESSED IN BODY-LOCAL FRAME (sdbodyt())
//_____________________________________________________________________________
/**
 * applyTorqueBodyLocal
 *
 * @param aBody
 * @param aTorque
 */
void SimmKinematicsEngine::applyTorqueBodyLocal(const AbstractBody &aBody, const double aTorque[3])
{
	throw Exception("SimmKinematicsEngine::applyTorqueBodyLocal(const AbstractBody &aBody, const double aTorque[3]) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * applyTorquesBodyLocal
 *
 * @param aN
 * @param aBodies
 * @param aTorques
 */
void SimmKinematicsEngine::applyTorquesBodyLocal(int aN, const AbstractBody *aBodies[], const double aTorques[][3])
{
	throw Exception("SimmKinematicsEngine::applyTorquesBodyLocal(int aN, const AbstractBody *aBodies[], const double aTorques[][3]) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * applyTorquesBodyLocal
 *
 * @param aN
 * @param aBodies
 * @param aTorques
 */
void SimmKinematicsEngine::applyTorquesBodyLocal(int aN, const AbstractBody *aBodies[], const double *aTorques)
{
	throw Exception("SimmKinematicsEngine::applyTorquesBodyLocal(int aN, const AbstractBody *aBodies[], const double *aTorques) not yet implemented.");
}

// GENERALIZED FORCES
//_____________________________________________________________________________
/**
 * applyGeneralizedForce
 *
 * @param aU
 * @param aF
 */
void SimmKinematicsEngine::applyGeneralizedForce(const AbstractCoordinate &aU, double aF)
{
	throw Exception("SimmKinematicsEngine::applyGeneralizedForce(const AbstractCoordinate &aU, double aF) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * applyGeneralizedForces
 *
 * @param aF
 */
void SimmKinematicsEngine::applyGeneralizedForces(const double aF[])
{
	throw Exception("SimmKinematicsEngine::applyGeneralizedForces(const double aF[]) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * applyGeneralizedForces
 *
 * @param aN
 * @param aU
 * @param aF
 */
void SimmKinematicsEngine::applyGeneralizedForces(int aN, const AbstractCoordinate *aU[], const double aF[])
{
	throw Exception("SimmKinematicsEngine::applyGeneralizedForces(int aN, const AbstractCoordinate *aU[], const double aF[]) not yet implemented.");
}

//--------------------------------------------------------------------------
// LOAD ACCESS AND COMPUTATION
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * getNetAppliedGeneralizedForce
 *
 * @param aU
 * @return 
 */
double SimmKinematicsEngine::getNetAppliedGeneralizedForce(const AbstractCoordinate &aU) const
{
	throw Exception("SimmKinematicsEngine::getNetAppliedGeneralizedForce(const AbstractCoordinate &aU) not yet implemented.");
	return 0.0;
}

//_____________________________________________________________________________
/**
 * computeGeneralizedForces
 *
 * @param aDUDT
 * @param rF
 */
void SimmKinematicsEngine::computeGeneralizedForces(double aDUDT[], double rF[]) const
{
	throw Exception("SimmKinematicsEngine::computeGeneralizedForces(double aDUDT[], double rF[]) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * computeReactions
 *
 * @param rForces
 * @param rTorques
 */
void SimmKinematicsEngine::computeReactions(double rForces[][3], double rTorques[][3]) const
{
	throw Exception("SimmKinematicsEngine::computeReactions(double rForces[][3], double rTorques[][3]) not yet implemented.");
}


//--------------------------------------------------------------------------
// EQUATIONS OF MOTION
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * formMassMatrix
 *
 * @param rI
 */
void SimmKinematicsEngine::formMassMatrix(double *rI)
{
	throw Exception("SimmKinematicsEngine::formMassMatrix(double *rI) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * formEulerTransform
 *
 * @param aBody
 * @param rE
 */
void SimmKinematicsEngine::formEulerTransform(const AbstractBody &aBody, double *rE) const
{
	throw Exception("SimmKinematicsEngine::formEulerTransform(const AbstractBody &aBody, double *rE) not yet implemented.");
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
void SimmKinematicsEngine::formJacobianTranslation(const AbstractBody &aBody, const double aPoint[3], double *rJ, const AbstractBody *aRefBody) const
{
	throw Exception("SimmKinematicsEngine::formJacobianTranslation(const AbstractBody &aBody, const double aPoint[3], double *rJ, const AbstractBody *aRefBody) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * formJacobianOrientation
 *
 * @param aBody
 * @param rJ0
 * @param aRefBody
 */
void SimmKinematicsEngine::formJacobianOrientation(const AbstractBody &aBody, double *rJ0, const AbstractBody *aRefBody) const
{
	throw Exception("SimmKinematicsEngine::formJacobianOrientation(const AbstractBody &aBody, double *rJ0, const AbstractBody *aRefBody) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * formJacobianEuler
 *
 * @param aBody
 * @param rJE
 * @param aRefBody
 */
void SimmKinematicsEngine::formJacobianEuler(const AbstractBody &aBody, double *rJE, const AbstractBody *aRefBody) const
{
	throw Exception("SimmKinematicsEngine::formJacobianEuler(const AbstractBody &aBody, double *rJE, const AbstractBody *aRefBody) not yet implemented.");
}

//--------------------------------------------------------------------------
// DERIVATIVES
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the derivatives of the coordinates and speeds.
 *
 * @param dqdt Time derivatives of the coordinates.
 * @param dudt Time derivatives of the speeds.
 */
void SimmKinematicsEngine::computeDerivatives(double *dqdt, double *dudt)
{
	throw Exception("SimmKinematicsEngine::computeAccelerations(double *dqdt, double *dudt) not yet implemented.");
}


//--------------------------------------------------------------------------
// UTILITY
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Transform a vector from one body to another
 *
 * @param aBodyFrom the body in which the vector is currently expressed
 * @param aVec the vector to be transformed
 * @param aBodyTo the body the vector will be transformed into
 * @param rVec the vector in the aBodyTo frame is returned here
 */
void SimmKinematicsEngine::transform(const AbstractBody &aBodyFrom, const double aVec[3], const AbstractBody &aBodyTo, double rVec[3]) const
{
	int i;

	for (i = 0; i < 3; i++)
		rVec[i] = aVec[i];

	if (&aBodyFrom == &aBodyTo)
		return;

	SimmPath* sp = _path.getSimmPath(&aBodyFrom, &aBodyTo);

	if (sp)
	{
		Transform& transform = sp->getInverseTransform();
		transform.transformVector(rVec);
	}
	else
	{
		string errorMessage = "SimmKinematicsEngine::transform: Could not find path from " + aBodyFrom.getName() + " to " + aBodyTo.getName();
		throw Exception(errorMessage);
	}
}

//_____________________________________________________________________________
/**
 * Transform a vector from one body to another
 *
 * @param aBodyFrom the body in which the vector is currently expressed
 * @param aVec the vector to be transformed
 * @param aBodyTo the body the vector will be transformed into
 * @param rVec the vector in the aBodyTo frame is returned here
 */
void SimmKinematicsEngine::transform(const AbstractBody &aBodyFrom, const Array<double>& aVec, const AbstractBody &aBodyTo, Array<double>& rVec) const
{
	int i;

	for (i = 0; i < 3; i++)
		rVec[i] = aVec[i];

	if (&aBodyFrom == &aBodyTo)
		return;

	SimmPath* sp = _path.getSimmPath(&aBodyFrom, &aBodyTo);

	if (sp)
	{
		Transform& transform = sp->getInverseTransform();
		transform.transformVector(rVec);
	}
	else
	{
		string errorMessage = "SimmKinematicsEngine::transform: Could not find path from " + aBodyFrom.getName() + " to " + aBodyTo.getName();
		throw Exception(errorMessage);
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
void SimmKinematicsEngine::transformPosition(const AbstractBody &aBodyFrom, const double aPos[3], const AbstractBody &aBodyTo, double rPos[3]) const
{
	int i;

	for (i = 0; i < 3; i++)
		rPos[i] = aPos[i];

	if (&aBodyFrom == &aBodyTo)
		return;

	SimmPath* sp = _path.getSimmPath(&aBodyFrom, &aBodyTo);

	if (sp)
	{
		Transform& transform = sp->getInverseTransform();
		transform.transformPoint(rPos);
	}
	else
	{
		string errorMessage = "SimmKinematicsEngine::transformPosition: Could not find path from " + aBodyFrom.getName() + " to " + aBodyTo.getName();
		throw Exception(errorMessage);
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
void SimmKinematicsEngine::transformPosition(const AbstractBody &aBodyFrom, const Array<double>& aPos, const AbstractBody &aBodyTo, Array<double>& rPos) const
{
	int i;

	for (i = 0; i < 3; i++)
		rPos[i] = aPos[i];

	if (&aBodyFrom == &aBodyTo)
		return;

	SimmPath* sp = _path.getSimmPath(&aBodyFrom, &aBodyTo);

	if (sp)
	{
		Transform& transform = sp->getInverseTransform();
		transform.transformPoint(rPos);
	}
	else
	{
		string errorMessage = "SimmKinematicsEngine::transformPosition: Could not find path from " + aBodyFrom.getName() + " to " + aBodyTo.getName();
		throw Exception(errorMessage);
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
void SimmKinematicsEngine::transformPosition(const AbstractBody &aBodyFrom, const double aPos[3], double rPos[3]) const
{
	int i;

	for (i = 0; i < 3; i++)
		rPos[i] = aPos[i];

	AbstractBody& groundBody = getGroundBody();

	if (&aBodyFrom == &groundBody)
		return;

	SimmPath* sp = _path.getSimmPath(&aBodyFrom, &groundBody);

	if (sp)
	{
		Transform& transform = sp->getInverseTransform();
		transform.transformPoint(rPos);
	}
	else
	{
		string errorMessage = "SimmKinematicsEngine::transformPosition: Could not find path from ground to " + aBodyFrom.getName();
		throw Exception(errorMessage);
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
void SimmKinematicsEngine::transformPosition(const AbstractBody &aBodyFrom, const Array<double>& aPos, Array<double>& rPos) const
{
	int i;

	for (i = 0; i < 3; i++)
		rPos[i] = aPos[i];

	AbstractBody& groundBody = getGroundBody();

	if (&aBodyFrom == &groundBody)
		return;

	SimmPath* sp = _path.getSimmPath(&aBodyFrom, &groundBody);

	if (sp)
	{
		Transform& transform = sp->getInverseTransform();
		transform.transformPoint(rPos);
	}
	else
	{
		string errorMessage = "SimmKinematicsEngine::transformPosition: Could not find path from ground to " + aBodyFrom.getName();
		throw Exception(errorMessage);
	}
}

//_____________________________________________________________________________
/**
 * Calculate the distance between a point in one body and a point on another body
 *
 * @param aBody1 the body that the first point is expressed in
 * @param aPoint1 the XYZ coordinates of the first point
 * @param aBody2 the body that the second point is expressed in
 * @param aPoint2 the XYZ coordinates of the second point
 * @return the distance between aPoint1 and aPoint2
 */
double SimmKinematicsEngine::calcDistance(const AbstractBody& aBody1, const Array<double>& aPoint1, const AbstractBody& aBody2, const Array<double>& aPoint2) const
{
	Array<double> pt1copy = aPoint1;

	transformPosition(aBody1, aPoint1, aBody2, pt1copy);

	return sqrt((pt1copy[0] - aPoint2[0])*(pt1copy[0] - aPoint2[0]) + (pt1copy[1] - aPoint2[1])*(pt1copy[1] - aPoint2[1]) +
		         (pt1copy[2] - aPoint2[2])*(pt1copy[2] - aPoint2[2]));
}

//_____________________________________________________________________________
/**
 * Calculate the distance between a point in one body and a point on another body
 *
 * @param aBody1 the body that the first point is expressed in
 * @param aPoint1 the XYZ coordinates of the first point
 * @param aBody2 the body that the second point is expressed in
 * @param aPoint2 the XYZ coordinates of the second point
 * @return the distance between aPoint1 and aPoint2
 */
double SimmKinematicsEngine::calcDistance(const AbstractBody& aBody1, const double aPoint1[3], const AbstractBody& aBody2, const double aPoint2[3]) const
{
	double pt1copy[3];

	transformPosition(aBody1, aPoint1, aBody2, pt1copy);

	return sqrt((pt1copy[0] - aPoint2[0])*(pt1copy[0] - aPoint2[0]) + (pt1copy[1] - aPoint2[1])*(pt1copy[1] - aPoint2[1]) +
		         (pt1copy[2] - aPoint2[2])*(pt1copy[2] - aPoint2[2]));
}

//_____________________________________________________________________________
/**
 * convertQuaternionsToAngles
 *
 * @param aQ
 * @param rQAng
 */
void SimmKinematicsEngine::convertQuaternionsToAngles(double *aQ, double *rQAng) const
{
	throw Exception("SimmKinematicsEngine::convertQuaternionsToAngles(double *aQ, double *rQAng) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * convertQuaternionsToAngles
 *
 * @param rQStore
 */
void SimmKinematicsEngine::convertQuaternionsToAngles(Storage *rQStore) const
{
	throw Exception("SimmKinematicsEngine::convertQuaternionsToAngles(Storage *rQStore) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * convertAnglesToQuaternions
 *
 * @param aQAng
 * @param rQ
 */
void SimmKinematicsEngine::convertAnglesToQuaternions(double *aQAng, double *rQ) const
{
	throw Exception("SimmKinematicsEngine::convertAnglesToQuaternions(double *aQAng, double *rQ) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * convertAnglesToQuaternions
 *
 * @param rQStore
 */
void SimmKinematicsEngine::convertAnglesToQuaternions(Storage *rQStore) const
{
	throw Exception("SimmKinematicsEngine::convertAnglesToQuaternions(Storage *rQStore) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * convertAnglesToDirectionCosines
 *
 * @param aE1
 * @param aE2
 * @param aE3
 * @param rDirCos
 */
void SimmKinematicsEngine::convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double rDirCos[3][3]) const
{
	throw Exception("SimmKinematicsEngine::convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double rDirCos[3][3]) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * convertAnglesToDirectionCosines
 *
 * @param aE1
 * @param aE2
 * @param aE3
 * @param rDirCos
 */
void SimmKinematicsEngine::convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double *rDirCos) const
{
	throw Exception("SimmKinematicsEngine::convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double *rDirCos) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * convertDirectionCosinesToAngles
 *
 * @param aDirCos
 * @param rE1
 * @param rE2
 * @param rE3
 */
void SimmKinematicsEngine::convertDirectionCosinesToAngles(double aDirCos[3][3], double *rE1, double *rE2, double *rE3) const
{
	throw Exception("SimmKinematicsEngine::convertDirectionCosinesToAngles(double aDirCos[3][3], double *rE1, double *rE2, double *rE3) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * convertDirectionCosinesToAngles
 *
 * @param aDirCos
 * @param rE1
 * @param rE2
 * @param rE3
 */
void SimmKinematicsEngine::convertDirectionCosinesToAngles(double *aDirCos, double *rE1, double *rE2, double *rE3) const
{
	throw Exception("SimmKinematicsEngine::convertDirectionCosinesToAngles(double *aDirCos, double *rE1, double *rE2, double *rE3) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * convertDirectionCosinesToQuaternions
 *
 * @param aDirCos
 * @param rQ1
 * @param rQ2
 * @param rQ3
 * @param rQ4
 */
void SimmKinematicsEngine::convertDirectionCosinesToQuaternions(double aDirCos[3][3], double *rQ1, double *rQ2, double *rQ3, double *rQ4) const
{
	throw Exception("SimmKinematicsEngine::convertDirectionCosinesToQuaternions(double aDirCos[3][3], double *rQ1, double *rQ2, double *rQ3, double *rQ4) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * convertDirectionCosinesToQuaternions
 *
 * @param aDirCos
 * @param rQ1
 * @param rQ2
 * @param rQ3
 * @param rQ4
 */
void SimmKinematicsEngine::convertDirectionCosinesToQuaternions(double *aDirCos, double *rQ1, double *rQ2, double *rQ3, double *rQ4) const
{
	throw Exception("SimmKinematicsEngine::convertDirectionCosinesToQuaternions(double *aDirCos, double *rQ1, double *rQ2, double *rQ3, double *rQ4) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * convertQuaternionsToDirectionCosines
 *
 * @param aQ1
 * @param aQ2
 * @param aQ3
 * @param aQ4
 * @param rDirCos
 */
void SimmKinematicsEngine::convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double rDirCos[3][3]) const
{
	throw Exception("SimmKinematicsEngine::convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double rDirCos[3][3]) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * convertQuaternionsToDirectionCosines
 *
 * @param aQ1
 * @param aQ2
 * @param aQ3
 * @param aQ4
 * @param rDirCos
 */
void SimmKinematicsEngine::convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double *rDirCos) const
{
	throw Exception("SimmKinematicsEngine::convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double *rDirCos) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * Compute the constrained coordinates for a multibody system.
 *
 * In the SIMM Kinematics Engine, none of the generalized coordinates are
 * ever constrained.  The constrained degrees of freedom are treated
 * internally as dof's apart from the generalized coordinates.  Currently,
 * there is no convenient means for a user to access these constrained
 * coordinates.
 *
 * @param rQ Array of coordinates with values consistent with any constraints
 * imposed on the coordinates.  Since constrained degrees of freedom are not
 * treated as part of the generalized coordinates, this method does nothing.
 */
void SimmKinematicsEngine::computeConstrainedCoordinates(double *rQ) const
{
	throw Exception("SimmKinematicsEngine::computeConstrainedCoordinates(double *rQ) not yet implemented.");
}

//_____________________________________________________________________________
/**
 * Make a SimmPathMatrix containing a paths from every body to every
 * other body.
 */
void SimmKinematicsEngine::makePaths()
{
   int i, j, numPathsCompleted = 0;
   int numBodies = _bodySet.getSize();
   int numPaths = numBodies*numBodies;

   /* Create space for the paths, and initialize each path to NULL */
	_path.initTable(numBodies);

   /* Count the trivial paths (from a body to itself) as completed */
	numPathsCompleted += numBodies;

   /* Each joint represents two simple paths- from parent to child and from
    * child to parent. Add these paths to the list and count them as completed.
    */
   for (i = 0; i < _jointSet.getSize(); i++)
   {
		JointPath p;
		p.push_back(SimmStep(_jointSet.get(i), SimmStep::forward));
		_path.setPath(_jointSet.get(i)->getParentBody(), _jointSet.get(i)->getChildBody(), p);
      numPathsCompleted++;
		p.clear();
		p.push_back(SimmStep(_jointSet.get(i), SimmStep::inverse));
		_path.setPath(_jointSet.get(i)->getChildBody(), _jointSet.get(i)->getParentBody(), p);
      numPathsCompleted++;
   }

   /* Until you have completed all paths, loop through the joint array and try to
    * fit that joint onto the end of a path in order to make a new path that is not
    * already defined.
    */
   while (numPathsCompleted < numPaths)
   {
      int k, oldCount = numPathsCompleted;

      for (i = 0; i < _jointSet.getSize(); i++)
      {
         const AbstractBody* bodyM = _jointSet.get(i)->getParentBody();
         const AbstractBody* bodyN = _jointSet.get(i)->getChildBody();

         if (bodyM == NULL || bodyN == NULL)
            continue;
         for (j = 0; j < numBodies; j++)
         {
            for (k = 0; k < numBodies; k++)
            {
               if (j == k)
                  continue;
					const AbstractBody* bodyJ = _bodySet.get(j);
					const AbstractBody* bodyK = _bodySet.get(k);
					const JointPath* jp = _path.getPath(bodyJ, bodyK);
					if (jp == NULL)
						continue;

               /* You've just accessed the path from j to k. If the current joint (i)
                * can be tacked onto the end of this path without looping back to j,
                * then create a path from j to n and see if it should be put in path[j][n].
                * Also check to see if the reverse joint can be tacked onto the end to
                * create a path from j to m.
                */
               if (bodyK == bodyM && bodyJ != bodyN)
               {
                  /* If path[j][n] does not yet exist, or if it is longer than the
                   * path you're about to make, replace it with the new path.
                   */
						const JointPath* curPath = _path.getPath(bodyJ, bodyN);
                  if (curPath == NULL || curPath->size() > jp->size() + 1)
                  {
							JointPath p = *jp;
							p.push_back(SimmStep(_jointSet.get(i), SimmStep::forward));
							_path.setPath(_bodySet.get(j), bodyN, p);
                     numPathsCompleted++;
                  }
               }
               else if (bodyK == bodyN && bodyJ != bodyM)
               {
                  /* If path[j][m] does not yet exist, or if it is longer than the
                   * path you're about to make, replace it with the new path.
                   */
						const JointPath* curPath = _path.getPath(bodyJ, bodyM);
                  if (curPath == NULL || curPath->size() > jp->size() + 1)
                  {
							JointPath p = *jp;
							p.push_back(SimmStep(_jointSet.get(i), SimmStep::inverse));
							_path.setPath(bodyJ, bodyM, p);
                     numPathsCompleted++;
						}
               }
            }
         }
      }

      /* If you did not add any paths to the list in this pass, and you still haven't completed
       * them all, there must be something wrong (e.g., a missing joint).
       */
      if (numPathsCompleted == oldCount && numPathsCompleted < numPaths)
      {
         string name1, name2;
			cout << "makePaths : Error - cannot find joint path between the following pairs of bodies: \n";
         for (i = 0; i < numBodies; i++)
         {
            name1 = _bodySet.get(i)->getName();
            for (j = i+1; j < numBodies; j++)
            {
               name2 = _bodySet.get(j)->getName();

					const JointPath* jp = _path.getPath(_bodySet.get(i), _bodySet.get(j));
               if (jp == NULL)
               {
                  cout << "(" << _bodySet.get(i)->getName() << ", " << _bodySet.get(j)->getName() << ")\n";
               }
					else
					{
						if (jp->back().getJoint()->getParentBody() != _bodySet.get(j) &&
							 jp->back().getJoint()->getChildBody() != _bodySet.get(j))
						{
							cout << "(" << _bodySet.get(i)->getName() << ", " << _bodySet.get(j)->getName() << ")\n";
						}
					}
            }
         }
         return;
      }
   }
}

//_____________________________________________________________________________
/**
 * For each coordinate, create a list of the joints that use it. These lists
 * are used to mark joint transforms dirty when a coordinate's value is changed.
 */
void SimmKinematicsEngine::createCoordinateJointLists()
{
   for (int i = 0; i < _coordinateSet.getSize(); i++)
   {
      for (int j = 0; j < _jointSet.getSize(); j++)
      {
         if (_jointSet.get(j)->isCoordinateUsed(_coordinateSet.get(i)))
         {
            _coordinateSet.get(i)->addJointToList(_jointSet.get(j));
         }
      }
   }
}

//_____________________________________________________________________________
/**
 * For each coordinate, create a list of the paths that use it. These lists
 * are used to mark transform paths dirty when a coordinate's value is changed.
 */
void SimmKinematicsEngine::createCoordinatePathLists()
{
	for (int i = 0; i < _bodySet.getSize(); i++)
	{
		for (int j = 0; j < _bodySet.getSize(); j++)
		{
			const JointPath* p = _path.getPath(_bodySet.get(i), _bodySet.get(j));
			if (p && p->size() > 0)
			{
				for (int c = 0; c < _coordinateSet.getSize(); c++)
				{
					for (unsigned int k = 0; k < p->size(); k++)
					{
						AbstractJoint* jnt = (*p)[k].getJoint();
						if (jnt->isCoordinateUsed(_coordinateSet.get(c)))
						{
							SimmPath* sp = _path.getSimmPath(_bodySet.get(i), _bodySet.get(j));
							if (sp)
								_coordinateSet.get(c)->addPathToList(sp);
							break;
						}
					}
				}
			}
		}
   }
}

//_____________________________________________________________________________
/**
 * This function looks through all the dofs in all the joints which are a
 * function of the specified gencoord, and tries to find one which should
 * be treated as the unconstrained dof. If the dof has a function with two
 * points, and the slope of the function is 1.0 or -1.0, and the function
 * passes through zero, then it is a good match. If no such dof is found, the
 * function returns an error. If there are multiple dofs which meet these
 * criteria, the first one is treated as the unconstrained one, and the
 * others will end up constrained.
 */
AbstractDof* SimmKinematicsEngine::findUnconstrainedDof(const AbstractCoordinate& aCoordinate, AbstractJoint*& rJoint)
{
	rJoint = NULL;

   for (int i = 0; i < _jointSet.getSize(); i++)
   {
		DofSet* dofs = _jointSet.get(i)->getDofSet();

      for (int j = 0; j < dofs->getSize(); j++)
      {
			if (dofs->get(j)->getCoordinate() == &aCoordinate)
	      {
				Function* func = dofs->get(j)->getFunction();

				if (func->getNumberOfPoints() == 2)
				{
					double valueAtZero = func->evaluate(0, 0.0, 0.0, 0.0);
					double slopeAtZero = func->evaluate(1, 0.0, 0.0, 0.0);

					if (EQUAL_WITHIN_ERROR(valueAtZero, 0.0))
					{
						if (EQUAL_WITHIN_ERROR(slopeAtZero, 1.0))
						{
							rJoint = _jointSet.get(i);
							return dofs->get(j);
						}
						else if (EQUAL_WITHIN_ERROR(slopeAtZero, -1.0))
						{
							rJoint = _jointSet.get(i);
							return dofs->get(j);
						}
					}
				}
	      }
      }
   }

   return NULL;
}

//_____________________________________________________________________________
/** Given a joint, this function returns which of its bodies, if either,
 * is a leaf node in the kinematic topology.
 */
AbstractBody* SimmKinematicsEngine::getLeafBody(AbstractJoint* aJoint) const
{
	int parentCount = 0, childCount = 0;
	AbstractBody* parentBody = aJoint->getParentBody();
	AbstractBody* childBody = aJoint->getChildBody();

	for (int i = 0; i < _jointSet.getSize(); i++)
	{
		if (parentBody == _jointSet.get(i)->getParentBody() || parentBody == _jointSet.get(i)->getChildBody())
			parentCount++;
		if (childBody == _jointSet.get(i)->getParentBody() || childBody == _jointSet.get(i)->getChildBody())
			childCount++;
	}

	/* If the body is used in exactly one joint, and it is not ground,
	 * then it is a leaf node.
	 */
	if (parentCount == 1 && parentBody != _groundBody)
		return parentBody;
	else if (childCount == 1 && childBody != _groundBody)
		return childBody;
	else
		return NULL;
}

//--------------------------------------------------------------------------
// TESTING
//--------------------------------------------------------------------------
void SimmKinematicsEngine::peteTest() const
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

	_path.peteTest();

}
