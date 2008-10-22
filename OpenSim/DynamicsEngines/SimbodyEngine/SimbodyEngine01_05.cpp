// SimbodyEngine01_05.cpp
// Authors: Frank C. Anderson, Ajay Seth
/*
 * Copyright (c)  2007, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/SpeedSet.h>
#include <OpenSim/Simulation/Model/JointSet.h>
#include <OpenSim/Simulation/Model/AbstractDof01_05.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractMuscle.h>
#include "SimbodyEngine01_05.h"
#include "SimbodyTranslationDof01_05.h"
#include "SimbodyRotationDof01_05.h"


//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using namespace SimTK;

static char simbodyGroundName[] = "ground";


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
SimbodyEngine01_05::~SimbodyEngine01_05()
{
	deleteSimbodyVariables();
}
//_____________________________________________________________________________
/**
 * Default constructor.  This constructor constructs a dynamic model of a
 * simple pendulum.
 */
SimbodyEngine01_05::SimbodyEngine01_05() :
	AbstractDynamicsEngine01_05()
{
	setNull();
	setupProperties();

	// CONSTRUCT SIMPLE PENDULUM
	//constructPendulum();
}

//_____________________________________________________________________________
/**
 * Constructor from an XML Document
 */
SimbodyEngine01_05::SimbodyEngine01_05(const string &aFileName) :
	AbstractDynamicsEngine01_05(aFileName,false)
{
	setNull();
	setupProperties();
	updateFromXMLNode();
	setup(_model);
}


//_____________________________________________________________________________
/**
 * Copy constructor.
 */
SimbodyEngine01_05::SimbodyEngine01_05(const SimbodyEngine01_05& aEngine) :
   AbstractDynamicsEngine01_05(aEngine)
{
	setNull();
	setupProperties();
	copyData(aEngine);
	setup(_model);
}

//_____________________________________________________________________________
/**
 * Copy this engine and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimbodyEngine01_05.
 */
Object* SimbodyEngine01_05::copy() const
{
	SimbodyEngine01_05 *object = new SimbodyEngine01_05(*this);
	return object;
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct the underlying Simbody multibody system for the model.
 */
void SimbodyEngine01_05::constructMultibodySystem()
{

}

//_____________________________________________________________________________
/**
 * Construct the underlying Simbody multibody system for the model.
 */
void SimbodyEngine01_05::addRigidBodies(SimbodyBody01_05 *aBody)
{

}

//_____________________________________________________________________________
/**
 * Find the index of the dof that has the last generalized coordinate
 * in the joint.  It is necessary to know this index because the child body
 * of this coordinate gets non-zero inertial properties.
 *
 * @return Index of the dof that has the last generalized coordinate i
 * the joint.
 */
int SimbodyEngine01_05::
findIndexOfDofThatHasLastGeneralizedCoordinate(DofSet01_05 *aDofSet)
{
	int index = 0;
	int nDof = aDofSet->getSize();
	for(int i=0;i<nDof;i++) {

		AbstractDof01_05 *dof = aDofSet->get(i);
		if(dof==NULL) continue;

		Function *dofFunction = dof->getFunction();

		// NULL
		// If there is no dof function, the assumption is an equality with the
		// generalized coordinate.
		if(dofFunction==NULL) {
			index = i;

		// NOT NULL
		} else {
			string functionType = dofFunction->getType();
			// Only a non-constant function gets a generalized coordinate.
			if(functionType!="Constant") {
				index = i;
			}
		}
	}

	return index;
}
//_____________________________________________________________________________
/**
 * Construct a dynamic model of a simple pendulum.
 */
void SimbodyEngine01_05::constructPendulum()
{

}
//_____________________________________________________________________________
/**
 * Allocate Simbody variables.
 */
void SimbodyEngine01_05::newSimbodyVariables()
{
}
//_____________________________________________________________________________
/**
 * Delete Simbody variables.
 */
void SimbodyEngine01_05::deleteSimbodyVariables()
{
}

//_____________________________________________________________________________
/**
 * Copy data members from one SimbodyEngine01_05 to another.
 *
 * @param aEngine SimbodyEngine01_05 to be copied.
 */
void SimbodyEngine01_05::copyData(const SimbodyEngine01_05 &aEngine)
{

}

//_____________________________________________________________________________
/**
 * Set NULL values for all the variable members of this class.
 */
void SimbodyEngine01_05::setNull()
{
	setType("SimbodyEngine01_05");
	_groundBody = NULL;
	_system = NULL;
	_matter = NULL;
	_s = NULL;
}
//_____________________________________________________________________________
/**
 * Create a ground body if necessary.
 */
void SimbodyEngine01_05::
createGroundBodyIfNecessary()
{
}


//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this SimbodyEngine01_05.
 */
void SimbodyEngine01_05::setup(Model* aModel)
{
	constructMultibodySystem();
	AbstractDynamicsEngine01_05::setup(aModel);
	//applyDefaultConfiguration();
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
SimbodyEngine01_05& SimbodyEngine01_05::operator=(const SimbodyEngine01_05 &aEngine)
{
	AbstractDynamicsEngine01_05::operator=(aEngine);
	copyData(aEngine);
	setup(aEngine._model);
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
void SimbodyEngine01_05::setupProperties()
{
}

//_____________________________________________________________________________
/**
 * Register the types used by this class.
 */
void SimbodyEngine01_05::registerTypes()
{
	Object::RegisterType(SimbodyBody01_05());
	Object::RegisterType(SimbodyJoint01_05());
	Object::RegisterType(SimbodyCoordinate01_05());
	Object::RegisterType(SimbodySpeed01_05());
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
void SimbodyEngine01_05::addBody(SimbodyBody01_05* aBody)
{
	// TODO:  Fill in Simbody stuff
	_bodySet.append(aBody);
}

//_____________________________________________________________________________
/**
 * Add a joint to the engine
 *
 * @param aJoint pointer to the joint to add
 */
void SimbodyEngine01_05::addJoint(SimbodyJoint01_05* aJoint)
{
	// TODO: Fill in Simbody stuff
	_jointSet.append(aJoint);
}

//_____________________________________________________________________________
/**
 * Add a coordinate to the engine
 *
 * @param aCoord pointer to the coordinate to add
 */
void SimbodyEngine01_05::addCoordinate(SimbodyCoordinate01_05* aCoord)
{
	// TODO: Fill in Simbody stuff
	_coordinateSet.append(aCoord);
}

//_____________________________________________________________________________
/**
 * Add a speed to the engine
 *
 * @param aSpeed pointer to the speed to add
 */
void SimbodyEngine01_05::addSpeed(SimbodySpeed01_05* aSpeed)
{
	// TODO: Fill in Simbody stuff
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
void SimbodyEngine01_05::updateCoordinateSet(CoordinateSet& aCoordinateSet)
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
void SimbodyEngine01_05::getUnlockedCoordinates(CoordinateSet& rUnlockedCoordinates) const
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
//done_____________________________________________________________________________
/**
 * Set the configuration (cooridnates and speeds) of the model.
 *
 * @param aY Array of coordinates followed by the speeds.
 */
void SimbodyEngine01_05::setConfiguration(const double aY[])
{
	int nq = getNumCoordinates();
	setConfiguration(aY,&aY[nq]);
}
//done_____________________________________________________________________________
/**
 * Get the configuration (cooridnates and speeds) of the model.
 *
 * @param rY Array of coordinates followed by the speeds.
 */
void SimbodyEngine01_05::getConfiguration(double rY[]) const
{
	int nq = getNumCoordinates();
	getConfiguration(rY,&rY[nq]);
}

//done_____________________________________________________________________________
/**
 * Set the configuration (cooridnates and speeds) of the model.
 *
 * @param aQ Array of generalized coordinates.
 * @param aU Array of generalized speeds.
 */
void SimbodyEngine01_05::setConfiguration(const double aQ[],const double aU[])
{
	// RESET ACCUMULATED FORCES
	resetBodyAndMobilityForceVectors();

	// SET Qs
	int nq = getNumCoordinates();
	Vector q(nq,aQ,true);
	_matter->setQ(*_s,q);
	// SET Us
	int nu = getNumSpeeds();
	Vector u(nu,aU,true);
	_matter->setU(*_s,u);

	// REALIZE AT THE VELOCITY STAGE
	_system->realize(*_s,Stage::Velocity);

	// MARK ACTUATOR PATHS AS INVALID
	// TODO: use Observer mechanism
	// TODO: dynamic cast is slow, make invalidate a general method
	ActuatorSet* act = getModel()->getActuatorSet();
	int size = act->getSize();
	for(int i=0; i<size; i++) {
		AbstractMuscle* m = dynamic_cast<AbstractMuscle*>(act->get(i));
		if(m) m->invalidatePath();
	}
}
//done_____________________________________________________________________________
/**
 * Get the configuration (cooridnates and speeds) of the model.
 *
 * @param rQ Array of generalized coordinates.
 * @param rU Array of generalized speeds.
 */
void SimbodyEngine01_05::getConfiguration(double rQ[],double rU[]) const
{
	getCoordinates(rQ);
	getSpeeds(rU);
}

//done_____________________________________________________________________________
/**
 * Get the values of the generalized coordinates.
 *
 * @param rQ Array of coordinates.
 */
void SimbodyEngine01_05::getCoordinates(double rQ[]) const
{
	int nq = getNumCoordinates();
	Vector q(nq,rQ,true);
	q = _matter->getQ(*_s);
}

//done_____________________________________________________________________________
/**
 * Get the values of the generalized speeds.
 *
 * @param rU Array of speeds.
 */
void SimbodyEngine01_05::getSpeeds(double rU[]) const
{
	int nu = getNumSpeeds();
	Vector u(nu,rU,true);
	u = _matter->getU(*_s);
}

//done_____________________________________________________________________________
/**
 * Get the last-computed values of the accelerations of the generalized
 * coordinates.  For the values to be valid, the method
 * computeDerivatives() must have been called.
 *
 * @param rDUDT Array to be filled with values of the accelerations of the
 * generalized coordinates.  The length of rDUDT should be at least as large
 * as the value returned by getNumSpeeds().
 * @see computeDerivatives()
 * @see getAcceleration(int aIndex)
 * @see getAcceleration(const char* aName);
 */
void SimbodyEngine01_05::getAccelerations(double rDUDT[]) const
{
	int nu = getNumSpeeds();
	Vector dudt(nu,rDUDT,true);
	dudt = _matter->getUDot(*_s);
}
//done_____________________________________________________________________________
/**
 * Extract the generalized coordinates and speeds from a combined array of
 * the coordinates and speeds.  This is only a utility method.  The
 * configuration of the model is not changed.
 *
 * @param aY Array of coordinates followed by the speeds.
 * @param rQ Array of coordinates taken from aY. 
 * @param rU Array of speeds taken from aY.
 */
void SimbodyEngine01_05::extractConfiguration(const double aY[],double rQ[],double rU[]) const
{
	int nq = getNumCoordinates();
	memcpy(rQ,aY,nq*sizeof(double));

	int nu = getNumSpeeds();
	memcpy(rU,&aY[nq],nu*sizeof(double));
}
//_____________________________________________________________________________
/**
 * Apply the default configuration to the model.  This means setting the
 * generalized coordinates and spees to their default values.
 */
void SimbodyEngine01_05::applyDefaultConfiguration()
{
   int i;
	
	// Coordinates
	int nq = _coordinateSet.getSize();
	Array<double> q(0.0,nq);
	for(i=0; i<nq; i++) {
		q[i] = _coordinateSet[i]->getDefaultValue();
	}

	// Speeds
	int nu = _speedSet.getSize();
	Array<double> u(0.0,nu);
	for(i=0; i<nu; i++) {
		u[i] = _speedSet[i]->getDefaultValue();
	}

	setConfiguration(&q[0],&u[0]);
}
//_____________________________________________________________________________
/**
 * Apply the default configuration to the model.  This means setting the
 * generalized coordinates and spees to their default values.
void SimbodyEngine01_05::applyDefaultConfiguration()
{
	for (int i = 0; i < _coordinateSet.getSize(); i++)
		_coordinateSet.get(i)->setValue(_coordinateSet.get(i)->getDefaultValue());

	for (int i = 0; i < _speedSet.getSize(); i++)
		_speedSet.get(i)->setValue(_speedSet.get(i)->getDefaultValue());
}
 */


//--------------------------------------------------------------------------
// GRAVITY
//--------------------------------------------------------------------------
//done_____________________________________________________________________________
/**
 * Set the gravity vector in the gloabl frame.
 *
 * @param aGrav the XYZ gravity vector
 * @return Whether or not the gravity vector was successfully set.
 */
bool SimbodyEngine01_05::setGravity(const SimTK::Vec3& aGrav)
{
	return true;
}
//done_____________________________________________________________________________
/**
 * Get the gravity vector in the gloabl frame.
 *
 * @param aGrav the XYZ gravity vector
 * @return Whether or not the gravity vector was successfully set.
 */
void SimbodyEngine01_05::getGravity(SimTK::Vec3& aGrav) const
{
}


//--------------------------------------------------------------------------
// SCALING
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Scale the Simbody Dynamics engine
 *
 * @param aScaleSet the set of XYZ scale factors for the bodies
 * @param aFinalMass the mass that the scaled model should have
 * @param aPreserveMassDist whether or not the masses of the
 *        individual bodies should be scaled with the body scale factors.
 * @return Whether or not scaling was successful.
 */
bool SimbodyEngine01_05::scale(const ScaleSet& aScaleSet, double aFinalMass, bool aPreserveMassDist)
{
	// Base class
	AbstractDynamicsEngine01_05::scale(aScaleSet, aFinalMass, aPreserveMassDist);

	// Invalidate all of the paths
	//_path.invalidate();

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
AbstractBody& SimbodyEngine01_05::getGroundBody() const
{
	assert(_groundBody);
	return *_groundBody;
}
//_____________________________________________________________________________
/**
 * Get tree joint whose child is the given body.
 *
 * @param aBody Pointer to the body.
 */
SimbodyJoint01_05 *SimbodyEngine01_05::
getInboardTreeJoint(SimbodyBody01_05* aBody) const
{
	int nj = _jointSet.getSize();
	for(int i=0; i<nj; i++) {
		SimbodyJoint01_05 *joint = (SimbodyJoint01_05*)_jointSet[i];
		SimbodyBody01_05 *child = joint->getBody();
		if((child==aBody) && joint->isTreeJoint())  return joint;
	}
	return NULL;
} 
//_____________________________________________________________________________
/**
 * Get the first outboard tree joint for a given body starting at a specified
 * index.  The index is modified by this method and gives the indexed location
 * of the joint in the joint set.  The index can therefore be used to start
 * a search for the next outboard joint in sequence.
 *
 * @param aBody Pointer to the body.
 * @param rIndex Index specifying where to start in the joint set.  Upon
 * return, rIndex is set to the index in the joint set where the joint
 * was found.
 * @return First outboard body.  NULL is returned if there is no
 * such body.
 */
SimbodyJoint01_05 *SimbodyEngine01_05::
getOutboardTreeJoint(SimbodyBody01_05* aBody,int &rIndex) const
{
	int nj = _jointSet.getSize();
	while(rIndex<nj) {
		SimbodyJoint01_05 *joint = (SimbodyJoint01_05*)_jointSet[rIndex];
		string parentName = joint->getParentBodyName();
		if(parentName==aBody->getName())  return joint;
		rIndex++;
	}
	return NULL;
} 
//_____________________________________________________________________________
/**
 * Adjust to body-to-joint and inboard-to-joint vectors to account for the
 * changed center of mass location of an SD/Fast body
 *
 * @param aBody Pointer to the body.
 * @param aNewMassCenter New mass center location in the SIMM body frame.
 */
bool SimbodyEngine01_05::adjustJointVectorsForNewMassCenter(SimbodyBody01_05* aBody)
{

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
double SimbodyEngine01_05::getMass() const
{
	double totalMass = 0.0;

	for (int i=0; i<_bodySet.getSize(); i++)
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
void SimbodyEngine01_05::getSystemInertia(double *rM, SimTK::Vec3& rCOM, double rI[3][3]) const
{
	*rM = _matter->calcSystemMass(*_s);
	rCOM = _matter->calcSystemMassCenterLocationInGround(*_s);
	Mat33::updAs(&rI[0][0]) = _matter->calcSystemCentralInertiaInGround(*_s).toMat33();
	//throw Exception("SimbodyEngine01_05.getSystemInertia: not yet implemented.");
}

//_____________________________________________________________________________
/**
 * getSystemInertia
 *
 * @param rM
 * @param rCOM
 * @param rI
 */
void SimbodyEngine01_05::getSystemInertia(double *rM, double *rCOM, double *rI) const
{
	*rM = _matter->calcSystemMass(*_s);
	Vec3::updAs(rCOM) = _matter->calcSystemMassCenterLocationInGround(*_s);
	Mat33::updAs(rI) = _matter->calcSystemCentralInertiaInGround(*_s).toMat33();
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
void SimbodyEngine01_05::getPosition(const AbstractBody &aBody, const SimTK::Vec3& aPoint, SimTK::Vec3& rPos) const
{
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
void SimbodyEngine01_05::getVelocity(const AbstractBody &aBody, const SimTK::Vec3& aPoint, SimTK::Vec3& rVel) const
{
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
void SimbodyEngine01_05::getAcceleration(const AbstractBody &aBody, const SimTK::Vec3& aPoint, SimTK::Vec3& rAcc) const
{
}

//_____________________________________________________________________________
/**
 * Get the body orientation with respect to the ground.
 *
 * @param aBody Pointer to body.
 * @param rDirCos Orientation of the body with respect to the ground frame.
 */
void SimbodyEngine01_05::getDirectionCosines(const AbstractBody &aBody, double rDirCos[3][3]) const
{
}

//_____________________________________________________________________________
/**
 * Get the body orientation with respect to the ground.
 *
 * @param aBody Pointer to body.
 * @param rDirCos Orientation of the body with respect to the ground frame.
 */
void SimbodyEngine01_05::getDirectionCosines(const AbstractBody &aBody, double *rDirCos) const
{
}

//_____________________________________________________________________________
/**
 * Get the inertial angular velocity of a body in the ground reference frame.
 *
 * @param aBody Pointer to body.
 * @param rAngVel Angular velocity of the body.
 */
void SimbodyEngine01_05::getAngularVelocity(const AbstractBody &aBody, SimTK::Vec3& rAngVel) const
{
}

//_____________________________________________________________________________
/**
 * Get the inertial angular velocity in the local body reference frame.
 *
 * @param aBody Pointer to body.
 * @param rAngVel Angular velocity of the body.
 */
void SimbodyEngine01_05::getAngularVelocityBodyLocal(const AbstractBody &aBody, SimTK::Vec3& rAngVel) const
{
}

//_____________________________________________________________________________
/**
 * Get the inertial angular acceleration of a body in the ground reference 
 * frame.
 *
 * @param aBody Pointer to body.
 * @param rAngAcc Angular acceleration of the body.
 */
void SimbodyEngine01_05::getAngularAcceleration(const AbstractBody &aBody, SimTK::Vec3& rAngAcc) const
{
}

//_____________________________________________________________________________
/**
 * Get the inertial angular acceleration in the local body reference frame.
 *
 * @param aBody Pointer to body.
 * @param rAngAcc Angular acceleration of the body.
 */
void SimbodyEngine01_05::getAngularAccelerationBodyLocal(const AbstractBody &aBody, SimTK::Vec3& rAngAcc) const
{
}

//_____________________________________________________________________________
/**
 * get a copy of the transform from the inertial frame to a body
 *
 * @param aBody
 * @return Transform from inertial frame to body
 */
OpenSim::Transform SimbodyEngine01_05::getTransform(const AbstractBody &aBody)
{
	return (Transform());
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
void SimbodyEngine01_05::applyForce(const AbstractBody &aBody, const SimTK::Vec3& aPoint, const SimTK::Vec3& aForce)
{
	const SimbodyBody01_05* body = (const SimbodyBody01_05*)&aBody;
	Vec3 point,force;
	point = aPoint;
	force = aForce;
	_matter->addInStationForce(*_s,body->_id,point,force,_bodyForces);
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
void SimbodyEngine01_05::applyForces(int aN, const AbstractBody *aBodies[], const double aPoints[][3], const double aForces[][3])
{
	for(int i=0; i<aN; i++) {
		applyForce(*aBodies[i],Vec3::getAs(aPoints[i]),Vec3::getAs(aForces[i]));
	}
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
void SimbodyEngine01_05::applyForces(int aN, const AbstractBody *aBodies[], const double *aPoints, const double *aForces)
{
	int I;
	for(int i=0; i<aN; i++) {
		I = Mtx::ComputeIndex(i, 3, 0);
		applyForce(*aBodies[i], Vec3::getAs(&aPoints[I]), Vec3::getAs(&aForces[I]));
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
void SimbodyEngine01_05::applyForceBodyLocal(const AbstractBody &aBody, const SimTK::Vec3& aPoint, const SimTK::Vec3& aForce)
{
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
void SimbodyEngine01_05::applyForcesBodyLocal(int aN, const AbstractBody *aBodies[], const double aPoints[][3], const double aForces[][3])
{
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
void SimbodyEngine01_05::applyForcesBodyLocal(int aN, const AbstractBody *aBodies[], const double *aPoints, const double *aForces)
{
}

//_____________________________________________________________________________
/**
 * Apply a torque expressed in the inertial frame to a body.
 *
 * @param aBody Pointer to body.
 * @param aTorque Torque expressed in the inertial frame.
 */
void SimbodyEngine01_05::applyTorque(const AbstractBody &aBody, const SimTK::Vec3& aTorque)
{
	const SimbodyBody01_05* b = (SimbodyBody01_05*)&aBody;
	Vec3 torque;
	torque = aTorque;
	_matter->addInBodyTorque(*_s,b->_id,torque,_bodyForces);
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
void SimbodyEngine01_05::applyTorques(int aN, const AbstractBody *aBodies[], const double aTorques[][3])
{
	for (int i=0; i<aN; i++) {
		applyTorque(*aBodies[i], Vec3::getAs(aTorques[i]));
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
void SimbodyEngine01_05::applyTorques(int aN, const AbstractBody *aBodies[], const double *aTorques)
{
	if(aTorques) {
		SimTK::Vec3 torque;

		for (int i=0; i<aN; i++) {
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
void SimbodyEngine01_05::applyTorqueBodyLocal(const AbstractBody &aBody, const SimTK::Vec3& aTorque)
{
	SimTK::Vec3 torqueInG;
	transform(aBody, aTorque, *_groundBody, torqueInG);
	applyTorque(aBody, torqueInG);
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
void SimbodyEngine01_05::applyTorquesBodyLocal(int aN, const AbstractBody *aBodies[], const double aTorques[][3])
{
	for(int i=0; i<aN; i++) {
		applyTorqueBodyLocal(*aBodies[i], Vec3::getAs(aTorques[i]));
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
void SimbodyEngine01_05::applyTorquesBodyLocal(int aN, const AbstractBody *aBodies[], const double *aTorques)
{
	if (aTorques) {
		int i;
		SimTK::Vec3 torque;

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
 *
 * Internally, the force is accumulated in a private vector using the
 * appropriate "addIn" method of Simbody.  The accumulated forces are
 * applied to the matter subsystem through the SimbodyOpenSimUserForces class.
 * This method does not affect the multibody system until the
 * SimbodyUserForce::calc() method is called.  Note that the calc() method
 * is not called by you, but by the underlying Simbody multibody system when
 * it is realized at the Dynamics stage.
 *
 * @param aQ Generalized coordinate.
 * @param aF Applied generalized force.
 */
void SimbodyEngine01_05::applyGeneralizedForce(const AbstractCoordinate &aQ, double aForce)
{
}

//_____________________________________________________________________________
/**
 * Apply generalized forces.
 * The dimension of aF is assumed to be the number of generalized coordinates.
 * @param aF Applied force.
 */
void SimbodyEngine01_05::applyGeneralizedForces(const double aF[])
{
	for(int i=0; i<_coordinateSet.getSize(); i++) {
		applyGeneralizedForce(*_coordinateSet.get(i), aF[i]);
	}
}

//_____________________________________________________________________________
/**
 * Apply generalized forces.
 * @param aN Number of generalized forces.
 * @param aU Generalized coordinates.
 * @param aF Applied forces.
 */
void SimbodyEngine01_05::applyGeneralizedForces(int aN, const AbstractCoordinate *aU[], const double aF[])
{
	for(int i=0; i<aN; i++) {
		applyGeneralizedForce(*aU[i], aF[i]);
	}
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
double SimbodyEngine01_05::getNetAppliedGeneralizedForce(const AbstractCoordinate &aU) const
{
	const SimbodyCoordinate01_05 *c = (SimbodyCoordinate01_05*)(&aU);



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
void SimbodyEngine01_05::computeGeneralizedForces(double aDUDT[], double rF[]) const
{
	//This is an inverse dynamics type routine not currently supported by Simbody
	//TODO: F = M*udot

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
void SimbodyEngine01_05::computeReactions(double rForces[][3], double rTorques[][3]) const
{

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
void SimbodyEngine01_05::computeDerivatives(double *dqdt,double *dudt)
{
	//_s->updTime() = t;

	// COMPUTE ACCELERATIONS
	try {
		_system->realize(*_s,Stage::Acceleration);
	} catch(...) {
		cout<<"SimbodyEngine01_05.computeDerivatives: invalid derivatives.\n\n";
		return;
	}
	Vector qdot = _matter->getQDot(*_s);
	Vector udot = _matter->getUDot(*_s);

	// ASSIGN THEM (MAYBE SLOW BUT CORRECT
	int nq = _s->getNQ();
	for(int i=0;i<nq;i++) dqdt[i] = qdot[i];

	int nu = _s->getNU();
	for(int i=0;i<nu;i++) dudt[i] = udot[i];
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
void SimbodyEngine01_05::transform(const AbstractBody &aBodyFrom, const double aVec[3], const AbstractBody &aBodyTo, double rVec[3]) const
{
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
void SimbodyEngine01_05::transform(const AbstractBody &aBodyFrom, const SimTK::Vec3& aVec, const AbstractBody &aBodyTo, SimTK::Vec3& rVec) const
{
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
void SimbodyEngine01_05::
transformPosition(const AbstractBody &aBodyFrom, const double aPos[3], const AbstractBody &aBodyTo, double rPos[3]) const
{
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
void SimbodyEngine01_05::
transformPosition(const AbstractBody &aBodyFrom, const SimTK::Vec3& aPos,
	const AbstractBody &aBodyTo, SimTK::Vec3& rPos) const
{
}

//_____________________________________________________________________________
/**
 * Transform a point from one body to the ground body
 *
 * @param aBodyFrom the body in which the point is currently expressed
 * @param aPos the XYZ coordinates of the point
 * @param rPos the XYZ coordinates of the point in the ground frame are returned here
 */
void SimbodyEngine01_05::transformPosition(const AbstractBody &aBodyFrom, const double aPos[3], double rPos[3]) const
{
}

//_____________________________________________________________________________
/**
 * Transform a point from one body to the ground body
 *
 * @param aBodyFrom the body in which the point is currently expressed
 * @param aPos the XYZ coordinates of the point
 * @param rPos the XYZ coordinates of the point in the ground frame are returned here
 */
void SimbodyEngine01_05::
transformPosition(const AbstractBody &aBodyFrom,const SimTK::Vec3& aPos,
	SimTK::Vec3& rPos) const
{
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
double SimbodyEngine01_05::
calcDistance(const AbstractBody& aBody1, const SimTK::Vec3& aPoint1,
	const AbstractBody& aBody2, const SimTK::Vec3& aPoint2) const
{
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
double SimbodyEngine01_05::calcDistance(const AbstractBody& aBody1, const double aPoint1[3], const AbstractBody& aBody2, const double aPoint2[3]) const
{
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
void SimbodyEngine01_05::convertQuaternionsToAngles(double *aQ, double *rQAng) const
{
	//Rotation R;
	//R.setToQuaternion(Quaternion(Vec4::getAs(aQ)));
	//Vec3::updAs(rQAng) = R.convertToBodyFixed123();
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
void SimbodyEngine01_05::convertQuaternionsToAngles(Storage *rQStore) const
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
void SimbodyEngine01_05::convertAnglesToQuaternions(double *aQAng, double *rQ) const
{
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
void SimbodyEngine01_05::convertAnglesToQuaternions(Storage *rQStore) const
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
void SimbodyEngine01_05::convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double rDirCos[3][3]) const
{
}

//_____________________________________________________________________________
/**
 * Convert angles to direction cosines.
 * @param aE1 1st Euler angle.
 * @param aE2 2nd Euler angle.
 * @param aE3 3rd Euler angle.
 * @param rDirCos Vector of direction cosines.
 */
void SimbodyEngine01_05::convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double *rDirCos) const
{
}

//_____________________________________________________________________________
/**
 * Convert direction cosines to angles.
 * @param aDirCos Vector of direction cosines.
 * @param rE1 1st Euler angle.
 * @param rE2 2nd Euler angle.
 * @param rE3 3rd Euler angle.
 */
void SimbodyEngine01_05::convertDirectionCosinesToAngles(double aDirCos[3][3], double *rE1, double *rE2, double *rE3) const
{
}

//_____________________________________________________________________________
/**
 * Convert direction cosines to angles.
 * @param aDirCos Vector of direction cosines.
 * @param rE1 1st Euler angle.
 * @param rE2 2nd Euler angle.
 * @param rE3 3rd Euler angle.
 */
void SimbodyEngine01_05::convertDirectionCosinesToAngles(double *aDirCos, double *rE1, double *rE2, double *rE3) const
{
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
void SimbodyEngine01_05::convertDirectionCosinesToQuaternions(double aDirCos[3][3], double *rQ1, double *rQ2, double *rQ3, double *rQ4) const
{
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
void SimbodyEngine01_05::convertDirectionCosinesToQuaternions(double *aDirCos, double *rQ1, double *rQ2, double *rQ3, double *rQ4) const
{
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
void SimbodyEngine01_05::convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double rDirCos[3][3]) const
{
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
void SimbodyEngine01_05::convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double *rDirCos) const
{
}


//--- Private Utility Methods Below Here ---

//=============================================================================
// RESIZE & RESET
//=============================================================================
//_____________________________________________________________________________
/**
 * Finish setting up anything that wasn't possible at the time of
 * construction.
 */
void SimbodyEngine01_05::
resizeBodyAndMobilityForceVectors()
{
	int nb = _matter->getNBodies();
	int nm = _matter->getNMobilities();
	_bodyForces.resize(nb);
	_mobilityForces.resize(nm);
}
//_____________________________________________________________________________
/**
 * Reset the vectors of accumulated forces, torques, and generalized forces,
 * meaning set all of them equal to zero.
 */
void
SimbodyEngine01_05::
resetBodyAndMobilityForceVectors()
{
	_bodyForces.setToZero();
	_mobilityForces = 0.0;
}

void SimbodyEngine01_05::formEulerTransform(const AbstractBody &aBody, double *rE) const
{
	if (&aBody && rE)
	{
		// GET ORIENTATION OF aBody
		double ang[3], dc[3][3];

		getDirectionCosines(aBody, dc);
		convertDirectionCosinesToAngles(dc, &ang[0], &ang[1], &ang[2]);

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
