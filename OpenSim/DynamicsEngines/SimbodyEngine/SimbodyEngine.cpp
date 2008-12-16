// SimbodyEngine.cpp
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
#include <SimTKcommon/internal/Exception.h>
#include <Simmath/internal/Function.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/SpeedSet.h>
#include <OpenSim/Simulation/Model/JointSet.h>
#include <OpenSim/Simulation/Model/DofSet01_05.h>
#include <OpenSim/Simulation/Model/AbstractTransformAxis.h>
#include <OpenSim/Simulation/Model/AbstractDof01_05.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractMuscle.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/Model/Marker.h>
#include <SimTKsimbody.h>

#include "SimbodyEngine.h"
#include "OpenSimUserForces.h"
#include "CustomJoint.h"
#include "TransformAxis.h"
#include "CoordinateCouplerConstraint.h"
#include "SimbodyEngine01_05.h"
#include "SimbodyTranslationDof01_05.h"
#include "SimbodyRotationDof01_05.h"


//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using namespace SimTK;

static std::string SimbodyGroundName = "ground";


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
SimbodyEngine::~SimbodyEngine()
{
	deleteSimbodyVariables();
}
//_____________________________________________________________________________
/**
 * Default constructor.  This constructor constructs a dynamic model of a
 * simple pendulum.
 */
SimbodyEngine::SimbodyEngine() :
	AbstractDynamicsEngine()
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
SimbodyEngine::SimbodyEngine(const string &aFileName) :
	AbstractDynamicsEngine(aFileName,false)
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
SimbodyEngine::SimbodyEngine(const SimbodyEngine& aEngine) :
   AbstractDynamicsEngine(aEngine)
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
 * @return Pointer to a copy of this SimbodyEngine.
 */
Object* SimbodyEngine::copy() const
{
	SimbodyEngine *object = new SimbodyEngine(*this);
	return object;
}

//_____________________________________________________________________________
/**
 * This version of the copy method is used for version migration.
 * The idea is to make a copy of the current object except do so using
 * the properties of the pervious version of the same class.
 *
 * @aPreviousVersion 
 */
void SimbodyEngine::
migrateFromPreviousVersion(const Object *aPreviousVersion)
{
	// Cast to previous version
	const SimbodyEngine01_05 *oldEngine = (const SimbodyEngine01_05*) aPreviousVersion;

	// LOOP THROUGH BODIES
	CoordinateSet tmpGlobalCoordSet;
	tmpGlobalCoordSet.setMemoryOwner(false);
	BodySet *newBodySet = getBodySet();
	const BodySet *oldBodySet = oldEngine->getBodySet();
	int nb = oldBodySet->getSize();
	for(int i=0;i<nb;i++) {

		OpenSim::Body *newBody = new Body();
		SimbodyBody01_05 *oldBody = dynamic_cast<SimbodyBody01_05*>( oldBodySet->get(i) );
		if(oldBody==NULL) continue;
		
		// BASE CLASS ASSIGNMENT OPERATOR
		// We use the assignment operator to get the name, visual properties, etc.
		// Problem is that it overwrites the type as well!	-Ayman 7/08
		std::string saveType = newBody->getType();
		(*newBody).AbstractBody::operator =(*oldBody);
		newBody->setType(saveType);

		// INERTIAL PROPERTIES
		// mass
		newBody->_mass = oldBody->_mass;
		// mass center
		newBody->_massCenter = oldBody->_massCenter;
		// inertia
		newBody->_inertiaXX = oldBody->_inertia[0];
		newBody->_inertiaYY = oldBody->_inertia[4];
		newBody->_inertiaZZ = oldBody->_inertia[8];
		newBody->_inertiaXY = oldBody->_inertia[1];
		newBody->_inertiaXZ = oldBody->_inertia[2];
		newBody->_inertiaYZ = oldBody->_inertia[5];

		// display stuff
		newBody->_displayer = oldBody->_displayer;

		// ADD BODY TO SET
		newBodySet->append(newBody);

		// FIND THE OLD JOINT FOR THIS BODY
		const JointSet *oldJointSet = oldEngine->getJointSet();
		int nj = oldJointSet->getSize();
		SimbodyJoint01_05 *oldJoint = NULL;
		for(int j=0;j<nj;j++) {
			SimbodyJoint01_05 *joint = (SimbodyJoint01_05*) oldJointSet->get(j);
			if(joint==NULL) continue;
			if(newBody->getName() == joint->getBodyName()) {
				oldJoint = joint;
				break;
			}
		}
		if(oldJoint==NULL) continue;

		// CREATE A NEW JOINT
		// All joints in version 1.5 are treated as custom joints.
		CustomJoint *newJoint = new CustomJoint();
		newBody->setJoint(newJoint);  delete newJoint;  // A copy is made, so must delete original.
		newJoint = (CustomJoint*)newBody->getJoint();
		if (newJoint!=NULL)
			_jointSet.append(newJoint);
		// Name
		newJoint->setName(oldJoint->getName());
		// Parent Body Name
		newJoint->_parentName = oldJoint->_bodies[0];
		// Location in Parent
		newJoint->_locationInParent = oldJoint->_locationInParent;
		// Location in Child
		newJoint->_location = oldJoint->_locationInChild;

		// FIND THE OLD DOFs FOR THIS JOINT
		TransformAxisSet *newTransformAxisSet = newJoint->getTransformAxisSet();
		const DofSet01_05 *oldDofSet = oldJoint->getDofSet();
		int nrots = 0;

		int na = oldDofSet->getSize();
		for(int a=0;a<na;a++) {

			AbstractDof01_05 *oldDof = oldDofSet->get(a);
			if(oldDof==NULL) continue;
			Function *function = oldDof->getFunction();
			if(function==NULL) continue;

			// Constant
			if(function->getType() == "Constant") {

				// Add constant translations to the location_in_parent;
				double c = function->evaluate(0,0.0);
				Vec3 shift(0,0,0);
				if(oldDof->getName()=="tx") {
					shift[0] = c;
				} else if(oldDof->getName()=="ty") {
					shift[1] = c;
				} else if(oldDof->getName()=="tz") {
					shift[2] = c;
				}
				newJoint->_locationInParent += shift;

			// Not constant
			} else {

				// Axis
				TransformAxis *newAxis = new TransformAxis();
				newAxis->setName(oldDof->getName());
				Vec3 axis(0,0,0);
				oldDof->getAxis(axis);
				newAxis->_axis = axis;

				// Motion type (rotation or translation)
				string type = oldDof->getType();
				if(type == "SimbodyTranslationDof") {
					newAxis->_isRotation = false;
				} else if(type == "SimbodyRotationDof") {
					newAxis->_isRotation = true;
                    nrots++;
				}

				// Coordinates
				string coordinateName = oldDof->getCoordinateName();
				newAxis->setCoordinateName(coordinateName);
				const CoordinateSet *oldCoordSet = oldEngine->getCoordinateSet();
				const SimbodyCoordinate01_05 *oldCoord = (const SimbodyCoordinate01_05*)oldCoordSet->get(newAxis->getCoordinateName());

				// Does the coordinate alread exist?
				Coordinate *newCoord = (Coordinate*)tmpGlobalCoordSet.get(coordinateName);

				// Add a New Coordinate
				if(newCoord==NULL) {
					newCoord = new Coordinate();
					newCoord->setName(coordinateName);
					newCoord->_joint = newJoint;
					newCoord->_defaultValue = oldCoord->_defaultValue;
					newCoord->_initialValue = oldCoord->_initialValue;
					newCoord->_tolerance = oldCoord->_tolerance;
					newCoord->_stiffness = oldCoord->_stiffness;
					newCoord->_range = oldCoord->_range;
					newCoord->_keys = oldCoord->_keys;
					newCoord->_clamped = oldCoord->_clamped;
					newCoord->_locked = oldCoord->_locked;
					newCoord->_restraintActive = false;
					newCoord->_dynamicsEngine = this;
					if(newAxis->_isRotation) {
						newJoint->getCoordinateSet()->insert(nrots-1, newCoord);
					}
					else
						newJoint->getCoordinateSet()->append(newCoord);
					tmpGlobalCoordSet.append(newCoord);
				}

				// New Coordinate AND New Constraint
				// When the joint is different, this means that the coordinate belongs to a
				// different joint already.  In this case, a new coordinate needs to be made,
				// adding a new degree of freedom, and then adding a constraint.
				// The coordinateby appending "_constrained" to the 
				if(newCoord->_joint != newJoint) {
					
					// New Coordinate
					Coordinate *constrainedCoord = new Coordinate();
					string constrainedCoordName = newBody->getName() + "_" + newAxis->getName() + "_constrained";
					constrainedCoord->setName(constrainedCoordName);
					constrainedCoord->_joint = newJoint;
					newJoint->getCoordinateSet()->append(constrainedCoord);
					// Set the name of the coordinate in the transform axis to the new coordinate.
					newAxis->setCoordinateName(constrainedCoordName);

					// New CoordinateCouplerConstraint
					CoordinateCouplerConstraint *constraint = new CoordinateCouplerConstraint();
					OpenSim::Array<string> indepCoordNames;
					indepCoordNames.append(coordinateName);
					constraint->setIndependentCoordinateNames(indepCoordNames);
					constraint->setDependentCoordinateName(constrainedCoordName);
					constraint->setFunction((Function*)function->copy());
					_constraintSet.append(constraint);

				} else {

					// Add Function to TransformAxis
					// Linear functions with a slope of 1 and intercept of 0 do not get
					// added to the transform axis.
					const double equalityTolerance = 1.0e-6;
					double mx,my,mz;
					function->isLinear(1.0e-3,-1,1,mx,-1,1,my,-1,1,mz);
					double intercept = function->evaluate(0,0.0);
					if(!rdMath::IsEqual(mx,1.0,equalityTolerance) || (!rdMath::IsEqual(intercept,0.0,equalityTolerance))) {
						newAxis->setFunction((Function*)function->copy());
					}
				}

				newTransformAxisSet->append(newAxis);
			}
		}
	}
	// Copy markerSet as well
	_markerSet= *(oldEngine->getMarkerSet());
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Objects in the hierarchy frequently need to access their parent.  For
 * example, each joint is owned by a parent body and needs to have a pointer
 * to that body.  This
 */
void SimbodyEngine::linkObjectsInHierarchy()
{
	// JOINTS
	int nb = _bodySet.getSize();
	for(int i=0;i<nb;i++) {

		// Body
		OpenSim::Body *body = (OpenSim::Body*)_bodySet[i];
		Joint *joint = (Joint*)body->getJoint();
		if(joint==NULL) continue;
		joint->_body = body;

		// Parent Body
		string parentName = joint->getParentName();
		joint->_parentBody = (OpenSim::Body*)_bodySet.get(parentName);

		// Engine
		joint->_dynamicsEngine = this;
	}
}

//_____________________________________________________________________________
/**
 * Construct the underlying Simbody multibody system for the model.
 */
void SimbodyEngine::constructMultibodySystem()
{
	// Get the ground body
	createGroundBodyIfNecessary();
	OpenSim::Body *ground = (OpenSim::Body*)_bodySet.get(SimbodyGroundName);
	if (ground==NULL) return;

	// Construct Simbody subsytems
	_system = new MultibodySystem;
	SimbodyMatterSubsystem *matter = new SimbodyMatterSubsystem(*_system);
	_userForceElements = new GeneralForceSubsystem(*_system);
	_gravitySubsystem = new Force::UniformGravity(*_userForceElements,*matter,Vec3(0,0,-9.8));
	Force::Custom custom(*_userForceElements,new OpenSimUserForces(this));

	// Add rigid bodies
	// This is a recursive method that attaches ALL bodies (children,
	// grand children, etc. to the body sent in as the argument.
	// So, sending in the ground body will construct the entir
	// multibody system.
	_coordinateSet.setMemoryOwner(false);
	_coordinateSet.setSize(0);
	_speedSet.setMemoryOwner(false);
	_speedSet.setSize(0);
	connectBodies(ground);

	// Gravity
	setGravity(_gravity);
}

//_____________________________________________________________________________
/**
 * Connect each body to its parent, constructing the underlying Simbody
 * multibody system.
 */
void SimbodyEngine::
connectBodies(OpenSim::Body *aBody)
{
	_coordinateSet.setSize(0);
	_speedSet.setSize(0);

	// LOOP OVER JOINT AND CONNECT EACH BODY TO ITS PARENT
	Joint *joint;
	MobilizedBodyIndex parentBodyIndex = aBody->_index;
	int iJoint = 0;

	// Need a joint to connect (mobilize) each body except for ground
	int nb = _bodySet.getSize();
	int jointIndex = 0;
	bool* used = new bool[nb];
	memset(used, false, nb*sizeof(bool));

	while(iJoint < nb-1) {
		// Scan for an outboard body of the given body and return the joint that connects them.
		// Returns NULL if no outboard bodies. The array "used" is used to mark which joints
		// have been used already, because you don't necessarily process the joint list in order.
		joint=getOutboardJoint(aBody,jointIndex,used);

		// If a joint exists, connect up the child body, then set aBody so that
		// you look for children of the child body next.
		if(joint){
			joint->connectBody();
			used[jointIndex] = true;
			iJoint++;
			aBody = dynamic_cast<Body *>(joint->getBody());
		}
		// Otherwise, go back to the parent body and look for more children.
		else{ 
			aBody = dynamic_cast<Body *>(aBody->_joint->getParentBody());
		}
	}

	delete [] used;
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
int SimbodyEngine::
findIndexOfDofThatHasLastGeneralizedCoordinate(TransformAxisSet *aTransformAxisSet)
{
	int index = 0;
	int nDof = aTransformAxisSet->getSize();
	for(int i=0;i<nDof;i++) {

		AbstractTransformAxis *dof = aTransformAxisSet->get(i);
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
 * Allocate Simbody variables.
 */
void SimbodyEngine::newSimbodyVariables()
{
	cout<<"SimbodyEngine::newSimbodyVariables:  should no longer call this."<<endl;
	//_system = new MultibodySystem;
	//_matter = new SimbodyMatterSubsystem(*_system);
	//_userForceElements = new GeneralForceSubsystem(*_system);
	//_gravitySubsystem = new Force::UniformGravity(*_userForceElements, *_matter, Vec3(0, 0, -9.8));
	//_s = new State;
}
//_____________________________________________________________________________
/**
 * Delete Simbody variables.
 */
void SimbodyEngine::deleteSimbodyVariables()
{
	delete _gravitySubsystem;  _gravitySubsystem = NULL;
	delete _userForceElements;  _userForceElements = NULL;
	delete _system;  _system = NULL;
}

//_____________________________________________________________________________
/**
 * Copy data members from one SimbodyEngine to another.
 *
 * @param aEngine SimbodyEngine to be copied.
 */
void SimbodyEngine::copyData(const SimbodyEngine &aEngine)
{

}

//_____________________________________________________________________________
/**
 * Set NULL values for all the variable members of this class.
 */
void SimbodyEngine::setNull()
{
	setType("SimbodyEngine");
	_groundBody = NULL;
	_system = NULL;
	_gravitySubsystem = NULL;
	_userForceElements = NULL;
}
//_____________________________________________________________________________
/**
 * Create a ground body if necessary.
 */
void SimbodyEngine::
createGroundBodyIfNecessary()
{
	// See if the ground body already exists.
	// The ground body is assumed to have the name simbodyGroundName.
	int size = _bodySet.getSize();
	if (size==0)	// Don't want to throw an exception for trivial models
		return;
	Body *ground=NULL;
	for(int i=0; i<size; i++) {
		Body *body = (Body*)_bodySet.get(i);
		if(body->getName() == SimbodyGroundName) {
			ground = body;
			break;
		}
	}

	// If the ground body doesn't exist create it
	// and append it to the body set.
	if (ground==NULL){
		string msg = "Model has no ground body. Please rename base segment to 'ground' and retry.";
		throw Exception(msg,__FILE__,__LINE__);
	}
	/* Temporarily comment out pending a robust solution to models without ground
	if(ground==NULL) {
		ground = new Body();
		_bodySet.append(ground);
	}
    */
	// Set member variables
	ground->setName(SimbodyGroundName);
	// Mass
	ground->_mass = 0.0;
	// Mass center
	Vec3 massCenter(0.0);
	ground->_massCenter = massCenter;
	// Simbody id
	ground->_index = SimTK::GroundIndex;
	// Engine
	ground->_dynamicsEngine = this;
	// Set member variable and append to body set.
	_groundBody = ground;
}


//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this SimbodyEngine.
 */
void SimbodyEngine::setup(Model* aModel)
{
	if (!aModel) 
		return;

	// So that everything being setup knows that the engine is not
	// in a valid state
	setInvalid();

	linkObjectsInHierarchy();
	constructMultibodySystem();
	AbstractDynamicsEngine::setup(aModel);
	
	// Set all the states of constraints, coordinates, etc.. down to a new SimTK::state
	// of the underlying Simbody model.
	updateSimbodyModel();

	applyDefaultConfiguration();

	// DynamicsEngine should be ready
	_invalid = false;
}


//void SimbodyEngine::initializeState(SimTK::State& completeState)
//{
//	
//	
//}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
SimbodyEngine& SimbodyEngine::operator=(const SimbodyEngine &aEngine)
{
	AbstractDynamicsEngine::operator=(aEngine);
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
void SimbodyEngine::setupProperties()
{
}

//_____________________________________________________________________________
/**
 * Register the types used by this class.
 */
void SimbodyEngine::registerTypes()
{
	Object::RegisterType(OpenSim::Body());
	Object::RegisterType(CustomJoint());
	Object::RegisterType(Coordinate());
	Object::RegisterType(Speed());
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
void SimbodyEngine::addBody(OpenSim::Body* aBody)
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
void SimbodyEngine::addJoint(Joint* aJoint)
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
void SimbodyEngine::addCoordinate(Coordinate* aCoord)
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
void SimbodyEngine::addSpeed(Speed* aSpeed)
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
void SimbodyEngine::updateCoordinateSet(CoordinateSet& aCoordinateSet)
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
void SimbodyEngine::getUnlockedCoordinates(CoordinateSet& rUnlockedCoordinates) const
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
void SimbodyEngine::setConfiguration(const double aY[])
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
void SimbodyEngine::getConfiguration(double rY[]) const
{
	assert(!_invalid);
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
void SimbodyEngine::setConfiguration(const double aQ[],const double aU[])
{
	// RESET ACCUMULATED FORCES
	resetBodyAndMobilityForceVectors();

	// SET Qs
	int nq = getNumCoordinates();
	Vector q(nq,aQ,true);
	_system->getMatterSubsystem().setQ(_s,q);
	// SET Us
	int nu = getNumSpeeds();
	Vector u(nu,aU,true);
	_system->getMatterSubsystem().setU(_s,u);

	// REALIZE AT THE VELOCITY STAGE
	_system->realize(_s, Stage::Velocity);
	_invalid = false;

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
void SimbodyEngine::getConfiguration(double rQ[],double rU[]) const
{
	getCoordinates(rQ);
	getSpeeds(rU);
}
//done_____________________________________________________________________________


/**
 * Project (change) the current configuration such that it satisfies the constraints acting
 * on the system. This alters the configuration (state) down in Simbody and then returns
 * the projected configuration. This method is intended for corrections to the state during
 * integration and should be called after a successful integrations step.
 * It can also be called to pose the model (i.e. in coord->setValue() after )
 * so that the configuration satisfies the constraints but WARNING this may produce upredictable
 * results when the current configuration is far from satisfying the constraints.
 *
 * returns true iff the engine configuration (state) was changed by the projection
 * @param  uY - Array of generalized coordinates and speeds. (update)
 * @param cTol constraint tolerance. (input)
 * @param  Yerr - Array of state. (update)
 */
bool SimbodyEngine::projectConfigurationToSatisfyConstraints(double uY[], const double cTol, double uYerr[])
{
	// REALIZE AT THE VELOCITY STAGE
	_system->realize(_s, Stage::Velocity);

	// Project configuration on to constraints to make sure we
	// are not violating locking and other constraints
	Vector weights; _system->calcYUnitWeights(_s, weights);
	Vector tols; _system->calcYErrUnitTolerances(_s, tols); 

	// Don't bother attempting to project constraints if there are none.
	if (tols.size() > 0){
		// Size of state errors for multibody system
		int ny = 0; 
	
		// if there is an Yerr to update then make sure we size the vector accordingly
		if(uYerr)
			ny = getNumCoordinates()+getNumSpeeds();
		
		Vector yErrEst(ny,uYerr,true); 

		_system->project(_s, cTol, weights, tols, yErrEst, System::ProjectOptions::All);
		//bool Qchanged = _system->updMatterSubsystem().projectQConstraints(_s, cTol, weights, tols,
		//												errEst, System::ProjectOptions::All);
		//
		//bool Uchanged =  _system->updMatterSubsystem().projectUConstraints(_s, cTol, Vector(ny, 1), Vector(nc, 1),
		//												errEst, System::ProjectOptions::All);
	
		// Get the resulting configuration due to the projection if teh configuration was changed
		getConfiguration(uY);
		return true;
	}
	return false;
}

/**
 * Get the values of the generalized coordinates.
 *
 * @param rQ Array of coordinates.
 */
void SimbodyEngine::getCoordinates(double rQ[]) const
{
	int nq = getNumCoordinates();
	Vector q(nq,rQ,true);
	// REALIZE AT THE POSITION STAGE
	_system->realize(_s, Stage::Position);
	q = _system->getMatterSubsystem().getQ(_s);
}

//done_____________________________________________________________________________
/**
 * Get the values of the generalized speeds.
 *
 * @param rU Array of speeds.
 */
void SimbodyEngine::getSpeeds(double rU[]) const
{
	int nu = getNumSpeeds();
	Vector u(nu,rU,true);
	// REALIZE AT THE VELOCITY STAGE
	_system->realize(_s, Stage::Velocity);
	u = _system->getMatterSubsystem().getU(_s);
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
void SimbodyEngine::getAccelerations(double rDUDT[]) const
{
	int nu = getNumSpeeds();
	Vector dudt(nu,rDUDT,true);
	// REALIZE AT THE ACCELERATION STAGE
	_system->realize(_s, Stage::Acceleration);
	dudt = _system->getMatterSubsystem().getUDot(_s);
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
void SimbodyEngine::extractConfiguration(const double aY[],double rQ[],double rU[]) const
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
void SimbodyEngine::applyDefaultConfiguration()
{
	int i;
	
	// Coordinates
	int nq = _coordinateSet.getSize();
	// Speeds
	int nu = _speedSet.getSize();

	Array<double> coordinatesAndSpeeds(0.0,nq+nu);

	for(i=0; i<nq; i++) {
		coordinatesAndSpeeds[i] = _coordinateSet[i]->getDefaultValue();
	}

	for(i=0; i<nu; i++) {
		coordinatesAndSpeeds[i+nq] = _speedSet[i]->getDefaultValue();
	}

	// Satisfy the coupled coordinate constraints
	computeConstrainedCoordinates(&coordinatesAndSpeeds[0]);

	setConfiguration(&coordinatesAndSpeeds[0]);
	projectConfigurationToSatisfyConstraints(&coordinatesAndSpeeds[0],1e-8);
}
//_____________________________________________________________________________
/**
 * Apply the default configuration to the model.  This means setting the
 * generalized coordinates and spees to their default values.
void SimbodyEngine::applyDefaultConfiguration()
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
bool SimbodyEngine::setGravity(const Vec3& aGrav)
{
	_gravitySubsystem->setGravity(aGrav);
	setInvalid();
	AbstractDynamicsEngine::setGravity(aGrav);
	return true;
}
//done_____________________________________________________________________________
/**
 * Get the gravity vector in the gloabl frame.
 *
 * @param aGrav the XYZ gravity vector
 * @return Whether or not the gravity vector was successfully set.
 */
void SimbodyEngine::getGravity(Vec3& aGrav) const
{
	aGrav = _gravitySubsystem->getGravity();
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
bool SimbodyEngine::scale(const ScaleSet& aScaleSet, double aFinalMass, bool aPreserveMassDist)
{
	// Base class
	AbstractDynamicsEngine::scale(aScaleSet, aFinalMass, aPreserveMassDist);

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
AbstractBody& SimbodyEngine::getGroundBody() const
{
	assert(_groundBody);
	return *_groundBody;
}
//_____________________________________________________________________________
/**
 * Get the first tree joint (that has not already been processed) which uses a
 * given body as its inboard body. The array aUsed says which joints have been
 * processed already. The index of the joint found is returned in rIndex
 * (primarily so that the caller can update the "used" array before calling
 * this function again).
 *
 * @param aBody Pointer to the body.
 * @param rIndex Index of the joint that is returned.
 * @param aUsed Array of booleans indicating which joints to skip over.
 * @return The first unused tree joint. NULL if there is no such body.
 */
Joint *SimbodyEngine::
getOutboardJoint(OpenSim::Body* aBody,int &rIndex,bool aUsed[]) const
{
	int nb = _bodySet.getSize();
	for(int i=0; i<nb; i++) {
		if (!aUsed[i]) {
			OpenSim::Body *body = (Body*)_bodySet[i];
			Joint *joint = (Joint*)body->getJoint();
			if(joint==NULL) continue;
			string parentName = joint->getParentName();
			if(parentName==aBody->getName()) {
				rIndex = i;
				return joint;
			}
		}
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
bool SimbodyEngine::adjustJointVectorsForNewMassCenter(OpenSim::Body* aBody)
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
double SimbodyEngine::getMass() const
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
void SimbodyEngine::getSystemInertia(double *rM, Vec3& rCOM, double rI[3][3]) const
{
	assert(!_invalid);
	*rM = _system->getMatterSubsystem().calcSystemMass(_s);
	rCOM = _system->getMatterSubsystem().calcSystemMassCenterLocationInGround(_s);
	Mat33::updAs(&rI[0][0]) = _system->getMatterSubsystem().calcSystemCentralInertiaInGround(_s).toMat33();
	//throw Exception("SimbodyEngine.getSystemInertia: not yet implemented.");
}

//_____________________________________________________________________________
/**
 * getSystemInertia
 *
 * @param rM
 * @param rCOM
 * @param rI
 */
void SimbodyEngine::getSystemInertia(double *rM, double *rCOM, double *rI) const
{
	assert(!_invalid);
	*rM = _system->getMatterSubsystem().calcSystemMass(_s);
	Vec3::updAs(rCOM) = _system->getMatterSubsystem().calcSystemMassCenterLocationInGround(_s);
	Mat33::updAs(rI) = _system->getMatterSubsystem().calcSystemCentralInertiaInGround(_s).toMat33();
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
void SimbodyEngine::getPosition(const AbstractBody &aBody, const Vec3& aPoint, Vec3& rPos) const
{
	assert(!_invalid);
	const OpenSim::Body* b = (OpenSim::Body*)(&aBody);
	rPos = _system->getMatterSubsystem().getMobilizedBody(b->_index).findStationLocationInGround(_s, aPoint);
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
void SimbodyEngine::getVelocity(const AbstractBody &aBody, const Vec3& aPoint, Vec3& rVel) const
{
	assert(!_invalid);
	const Body* b = (Body*)(&aBody);
	rVel = _system->getMatterSubsystem().getMobilizedBody(b->_index).findStationVelocityInGround(_s, aPoint);
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
void SimbodyEngine::getAcceleration(const AbstractBody &aBody, const Vec3& aPoint, Vec3& rAcc) const
{
	assert(!_invalid);
	const Body* b = (Body*)(&aBody);
	rAcc = _system->getMatterSubsystem().getMobilizedBody(b->_index).findStationAccelerationInGround(_s, aPoint);
}

//_____________________________________________________________________________
/**
 * Get the body orientation with respect to the ground.
 *
 * @param aBody Pointer to body.
 * @param rDirCos Orientation of the body with respect to the ground frame.
 */
void SimbodyEngine::getDirectionCosines(const AbstractBody &aBody, double rDirCos[3][3]) const
{
	assert(!_invalid);
	const Body* b = (Body*)(&aBody);
	Mat33::updAs(&rDirCos[0][0]) = _system->getMatterSubsystem().getMobilizedBody(b->_index).getBodyRotation(_s).asMat33();
}

//_____________________________________________________________________________
/**
 * Get the body orientation with respect to the ground.
 *
 * @param aBody Pointer to body.
 * @param rDirCos Orientation of the body with respect to the ground frame.
 */
void SimbodyEngine::getDirectionCosines(const AbstractBody &aBody, double *rDirCos) const
{
	assert(!_invalid);
	const Body* b = (Body*)(&aBody);
	Mat33::updAs(rDirCos) = _system->getMatterSubsystem().getMobilizedBody(b->_index).getBodyRotation(_s).asMat33();
}

//_____________________________________________________________________________
/**
 * Get the inertial angular velocity of a body in the ground reference frame.
 *
 * @param aBody Pointer to body.
 * @param rAngVel Angular velocity of the body.
 */
void SimbodyEngine::getAngularVelocity(const AbstractBody &aBody, Vec3& rAngVel) const
{
	assert(!_invalid );
	const Body *b = (Body*)(&aBody);
	rAngVel = _system->getMatterSubsystem().getMobilizedBody(b->_index).getBodyAngularVelocity(_s);
}

//_____________________________________________________________________________
/**
 * Get the inertial angular velocity in the local body reference frame.
 *
 * @param aBody Pointer to body.
 * @param rAngVel Angular velocity of the body.
 */
void SimbodyEngine::getAngularVelocityBodyLocal(const AbstractBody &aBody, Vec3& rAngVel) const
{
	assert(!_invalid );
	const Body *b = (Body*)(&aBody);
	rAngVel = _system->getMatterSubsystem().getMobilizedBody(b->_index).getBodyAngularVelocity(_s);
}

//_____________________________________________________________________________
/**
 * Get the inertial angular acceleration of a body in the ground reference 
 * frame.
 *
 * @param aBody Pointer to body.
 * @param rAngAcc Angular acceleration of the body.
 */
void SimbodyEngine::getAngularAcceleration(const AbstractBody &aBody, Vec3& rAngAcc) const
{
	assert(!_invalid );
	const Body *b = (Body*)(&aBody);
	rAngAcc = _system->getMatterSubsystem().getMobilizedBody(b->_index).getBodyAngularAcceleration(_s);
}

//_____________________________________________________________________________
/**
 * Get the inertial angular acceleration in the local body reference frame.
 *
 * @param aBody Pointer to body.
 * @param rAngAcc Angular acceleration of the body.
 */
void SimbodyEngine::getAngularAccelerationBodyLocal(const AbstractBody &aBody, Vec3& rAngAcc) const
{
	const Body *b = (Body*)(&aBody);
	rAngAcc = _system->getMatterSubsystem().getMobilizedBody(b->_index).getBodyAngularAcceleration(_s);
}

//_____________________________________________________________________________
/**
 * get a copy of the transform from the inertial frame to a body
 *
 * @param aBody
 * @return Transform from inertial frame to body
 */
OpenSim::Transform SimbodyEngine::getTransform(const AbstractBody &aBody)
{
	assert(!_invalid );
	double aMat[4][4];
	const Body* b = (Body*)(&aBody);

	Mat44::updAs(&aMat[0][0]) = _system->getMatterSubsystem().getMobilizedBody(b->_index).getBodyTransform(_s).toMat44();
	return Transform(aMat);
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
void SimbodyEngine::applyForce(const AbstractBody &aBody, const Vec3& aPoint, const Vec3& aForce)
{
	assert(!_invalid );
	const Body* body = (const Body*)&aBody;
	Vec3 point,force;
	point = aPoint;
	force = aForce;
	_system->getMatterSubsystem().addInStationForce(_s,body->_index,point,force,_bodyForces);
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
void SimbodyEngine::applyForces(int aN, const AbstractBody *aBodies[], const double aPoints[][3], const double aForces[][3])
{
	assert(!_invalid );
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
void SimbodyEngine::applyForces(int aN, const AbstractBody *aBodies[], const double *aPoints, const double *aForces)
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
void SimbodyEngine::applyForceBodyLocal(const AbstractBody &aBody, const Vec3& aPoint, const Vec3& aForce)
{
	const Body* body = (const Body*)&aBody;

	// Transform force vector into ground frame
	Vec3 forceInB;
	forceInB = aForce;
	Vec3 forceInG;
	forceInG = _system->getMatterSubsystem().getMobilizedBody(body->_index).expressVectorInGroundFrame(_s,forceInB);
	applyForce(aBody,aPoint,forceInG);
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
void SimbodyEngine::applyForcesBodyLocal(int aN, const AbstractBody *aBodies[], const double aPoints[][3], const double aForces[][3])
{
	int i;

	for (i = 0; i < aN; i++)
		applyForce(*aBodies[i], Vec3::getAs(aPoints[i]), Vec3::getAs(aForces[i]));
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
void SimbodyEngine::applyForcesBodyLocal(int aN, const AbstractBody *aBodies[], const double *aPoints, const double *aForces)
{
	// Check that the input vectors have been defined.
	if (!aBodies || !aPoints || !aForces)
		return;

	int i;
	Vec3 point, force;

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
void SimbodyEngine::applyTorque(const AbstractBody &aBody, const Vec3& aTorque)
{
	const Body* b = (Body*)&aBody;
	Vec3 torque;
	torque = aTorque;
	_system->getMatterSubsystem().addInBodyTorque(_s,b->_index,torque,_bodyForces);
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
void SimbodyEngine::applyTorques(int aN, const AbstractBody *aBodies[], const double aTorques[][3])
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
void SimbodyEngine::applyTorques(int aN, const AbstractBody *aBodies[], const double *aTorques)
{
	if(aTorques) {
		Vec3 torque;

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
void SimbodyEngine::applyTorqueBodyLocal(const AbstractBody &aBody, const Vec3& aTorque)
{
	Vec3 torqueInG;
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
void SimbodyEngine::applyTorquesBodyLocal(int aN, const AbstractBody *aBodies[], const double aTorques[][3])
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
void SimbodyEngine::applyTorquesBodyLocal(int aN, const AbstractBody *aBodies[], const double *aTorques)
{
	if (aTorques) {
		int i;
		Vec3 torque;

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
 * applied to the matter subsystem through the OpenSimUserForces class.
 * This method does not affect the multibody system until the
 * SimbodyUserForce::calc() method is called.  Note that the calc() method
 * is not called by you, but by the underlying Simbody multibody system when
 * it is realized at the Dynamics stage.
 *
 * @param aQ Generalized coordinate.
 * @param aF Applied generalized force.
 */
void SimbodyEngine::applyGeneralizedForce(const AbstractCoordinate &aQ, double aForce)
{
	const Coordinate *q = (Coordinate*) &aQ;
	_system->getMatterSubsystem().addInMobilityForce(_s,q->_bodyIndex,MobilizerUIndex(q->_mobilityIndex),aForce,_mobilityForces);
}

//_____________________________________________________________________________
/**
 * Apply generalized forces.
 * The dimension of aF is assumed to be the number of generalized coordinates.
 * @param aF Applied force.
 */
void SimbodyEngine::applyGeneralizedForces(const double aF[])
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
void SimbodyEngine::applyGeneralizedForces(int aN, const AbstractCoordinate *aU[], const double aF[])
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
double SimbodyEngine::getNetAppliedGeneralizedForce(const AbstractCoordinate &aU) const
{
	const Coordinate *c = (Coordinate*)(&aU);



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
void SimbodyEngine::computeGeneralizedForces(double aDUDT[], double rF[]) const
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
void SimbodyEngine::computeReactions(Vector_<Vec3>& rForces, Vector_<Vec3>& rTorques) const
{
	int nb = _system->getMatterSubsystem().getNBodies();
	SimTK::Vector_<SpatialVec> reactionForces(_system->getMatterSubsystem().getNBodies());

	// Systems must be realized to acceleration stage
	_system->realize(_s, Stage::Acceleration);
    _system->getMatterSubsystem().calcMobilizerReactionForces(_s, reactionForces);

	//Separate SimTK SpatialVecs to Forces and Torques
	// SpatialVec = Vec2<Vec3 torque, Vec3 force>
	for(int i=0; i<nb; i++){
		rTorques[i] = reactionForces[i](0);
		rForces[i] = reactionForces[i](1);
	}
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
void SimbodyEngine::computeConstrainedCoordinates(double y[]) const
{
	_system->realize(_s, Stage::Velocity);
	int nq = getNumCoordinates();
	//Cycle through constraints
	for(int i=0; i < _constraintSet.getSize(); i++){
		AbstractConstraint *aConstraint = _constraintSet.get(i);
		//Check that constraint is a coordinate coupler type, then compute dependent coordinate's value
		if(aConstraint->getType() == "CoordinateCouplerConstraint"){
			CoordinateCouplerConstraint *coupler = dynamic_cast<CoordinateCouplerConstraint *>(aConstraint);
			//TODO: In the future a coupled coordinate can be a function of multiple independent coordinates
			int i_ind = _coordinateSet.getIndex(coupler->getIndependentCoordinateNames()[0]);
			int i_dep = _coordinateSet.getIndex(coupler->getDependentCoordinateName());
			// Coordinate value
			y[i_dep] = coupler->getFunction()->evaluate(0, y[i_ind]);
			// Corresponding speed
			y[nq+i_dep] = coupler->getFunction()->evaluateTotalFirstDerivative(y[i_ind], y[i_ind+nq]);
		}
	}
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
void SimbodyEngine::computeDerivatives(double *dqdt,double *dudt)
{
	//_s.updTime() = t;

	Vec3 grav;
	getGravity(grav);

	// COMPUTE ACCELERATIONS
	try {
		_system->realize(_s,Stage::Acceleration);
	} catch(std::exception &x) {
		cout<<x.what()<<endl;
		cout<<"SimbodyEngine.computeDerivatives: invalid derivatives."<<endl;
		return;
	}
	Vector qdot = _system->getMatterSubsystem().getQDot(_s);
	Vector udot = _system->getMatterSubsystem().getUDot(_s);

	// ASSIGN THEM (MAYBE SLOW BUT CORRECT)
	int nq = _s.getNQ();
	for(int i=0;i<nq;i++) dqdt[i] = qdot[i];

	int nu = _s.getNU();
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
void SimbodyEngine::transform(const AbstractBody &aBodyFrom, const double aVec[3], const AbstractBody &aBodyTo, double rVec[3]) const
{
	assert(!_invalid );
	if(&aBodyFrom == &aBodyTo) { for(int i=0; i<3; i++) { rVec[i]=aVec[i]; } return; }
	const Body* bFrom = (const Body*)&aBodyFrom;
	const Body* bTo = (const Body*)&aBodyTo;

	//Get input vector as a Vec3 to make the call down to Simbody and update the output vector 
	Vec3::updAs(rVec) = _system->getMatterSubsystem().getMobilizedBody(bFrom->_index).expressVectorInAnotherBodyFrame(_s, Vec3::getAs(aVec), _system->getMatterSubsystem().getMobilizedBody(bTo->_index));
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
void SimbodyEngine::transform(const AbstractBody &aBodyFrom, const Vec3& aVec, const AbstractBody &aBodyTo, Vec3& rVec) const
{
	assert(!_invalid );
	if(&aBodyFrom == &aBodyTo) { rVec=aVec; return; }
	const Body* bFrom = (const Body*)&aBodyFrom;
	const Body* bTo = (const Body*)&aBodyTo;

	//Get input vector as a Vec3 to make the call down to Simbody and update the output vector 
	rVec = _system->getMatterSubsystem().getMobilizedBody(bFrom->_index).expressVectorInAnotherBodyFrame(_s, aVec, _system->getMatterSubsystem().getMobilizedBody(bTo->_index));
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
void SimbodyEngine::
transformPosition(const AbstractBody &aBodyFrom, const double aPos[3], const AbstractBody &aBodyTo, double rPos[3]) const
{
	assert(!_invalid );
	if(&aBodyFrom == &aBodyTo) {
	   for (int i=0; i<3; i++) rPos[i] = aPos[i];
		return;
	}
	const Body* bFrom = (const Body*)&aBodyFrom;
	const Body* bTo = (const Body*)&aBodyTo;

	//Get input vector as a Vec3 to make the call down to Simbody and update the output vector 
	Vec3::updAs(rPos) = _system->getMatterSubsystem().getMobilizedBody(bFrom->_index).findStationLocationInAnotherBody(_s, Vec3::getAs(aPos), _system->getMatterSubsystem().getMobilizedBody(bTo->_index));
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
void SimbodyEngine::
transformPosition(const AbstractBody &aBodyFrom, const Vec3& aPos,
	const AbstractBody &aBodyTo, Vec3& rPos) const
{
	assert(!_invalid );
	if(&aBodyFrom == &aBodyTo) {
	   for (int i=0; i<3; i++) rPos[i] = aPos[i];
		return;
	}
	const Body* bFrom = (const Body*)&aBodyFrom;
	const Body* bTo = (const Body*)&aBodyTo;

	//Get input vector as a Vec3 to make the call down to Simbody and update the output vector 
	rPos = _system->getMatterSubsystem().getMobilizedBody(bFrom->_index).findStationLocationInAnotherBody(_s, aPos, _system->getMatterSubsystem().getMobilizedBody(bTo->_index));
}

//_____________________________________________________________________________
/**
 * Transform a point from one body to the ground body
 *
 * @param aBodyFrom the body in which the point is currently expressed
 * @param aPos the XYZ coordinates of the point
 * @param rPos the XYZ coordinates of the point in the ground frame are returned here
 */
void SimbodyEngine::transformPosition(const AbstractBody &aBodyFrom, const double aPos[3], double rPos[3]) const
{
	assert(!_invalid );
	const Body* bFrom = (const Body*)&aBodyFrom;

	//Get input vector as a Vec3 to make the call down to Simbody and update the output vector 
	Vec3::updAs(rPos) = _system->getMatterSubsystem().getMobilizedBody(bFrom->_index).findStationLocationInGround(_s, Vec3::getAs(aPos));
}

//_____________________________________________________________________________
/**
 * Transform a point from one body to the ground body
 *
 * @param aBodyFrom the body in which the point is currently expressed
 * @param aPos the XYZ coordinates of the point
 * @param rPos the XYZ coordinates of the point in the ground frame are returned here
 */
void SimbodyEngine::
transformPosition(const AbstractBody &aBodyFrom,const Vec3& aPos,
	Vec3& rPos) const
{
	assert(!_invalid );
	const Body* bFrom = (const Body*)&aBodyFrom;
	rPos = _system->getMatterSubsystem().getMobilizedBody(bFrom->_index).findStationLocationInGround(_s, aPos);
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
double SimbodyEngine::
calcDistance(const AbstractBody& aBody1, const Vec3& aPoint1,
	const AbstractBody& aBody2, const Vec3& aPoint2) const
{
	assert(!_invalid);
	const Body* b1 = (Body*)(&aBody1);
	const Body* b2 = (Body*)(&aBody2);
	_system->realize(_s, SimTK::Stage::Velocity);
	return _system->getMatterSubsystem().getMobilizedBody(b1->_index).calcStationToStationDistance(_s, aPoint1, _system->getMatterSubsystem().getMobilizedBody(b2->_index), aPoint2);
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
double SimbodyEngine::calcDistance(const AbstractBody& aBody1, const double aPoint1[3], const AbstractBody& aBody2, const double aPoint2[3]) const
{
	assert(!_invalid );
	const Body* b1 = (Body*)(&aBody1);
	const Body* b2 = (Body*)(&aBody2);
	return _system->getMatterSubsystem().getMobilizedBody(b1->_index).calcStationToStationDistance(_s, Vec3::getAs(aPoint1), _system->getMatterSubsystem().getMobilizedBody(b2->_index), Vec3::getAs(aPoint2));
}

//_____________________________________________________________________________
/**
 * Convert quaterions to angles.
 *
 * @param aQ Array of generalized coordinates, some of which may be
 * quaternions.  The length of aQ must be at least getNumCoordinates().
 * @param rQAng Array of equivalent angles.
 */
void SimbodyEngine::convertQuaternionsToAngles(double *aQ, double *rQAng) const
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
void SimbodyEngine::convertQuaternionsToAngles(Storage *rQStore) const
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
void SimbodyEngine::convertAnglesToQuaternions(double *aQAng, double *rQ) const
{
	Rotation R;
	R.setRotationToBodyFixedXYZ(Vec3::getAs(aQAng));

	Vec4::updAs(rQ) = R.convertRotationToQuaternion().asVec4();
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
void SimbodyEngine::convertAnglesToQuaternions(Storage *rQStore) const
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
void SimbodyEngine::convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double rDirCos[3][3]) const
{
	Vec3 angs(aE1, aE2, aE3);
	Rotation aRot; 
	aRot.setRotationToBodyFixedXYZ(angs);
	Mat33::updAs(&rDirCos[0][0]) = aRot.asMat33();
}

//_____________________________________________________________________________
/**
 * Convert angles to direction cosines.
 * @param aE1 1st Euler angle.
 * @param aE2 2nd Euler angle.
 * @param aE3 3rd Euler angle.
 * @param rDirCos Vector of direction cosines.
 */
void SimbodyEngine::convertAnglesToDirectionCosines(double aE1, double aE2, double aE3, double *rDirCos) const
{
	if(rDirCos==NULL) return;
	
	Vec3 angs(aE1, aE2, aE3);
	Rotation aRot; 
	aRot.setRotationToBodyFixedXYZ(angs);
	Mat33::updAs(&rDirCos[0]) = aRot.asMat33();
}

//_____________________________________________________________________________
/**
 * Convert direction cosines to angles.
 * @param aDirCos Vector of direction cosines.
 * @param rE1 1st Euler angle.
 * @param rE2 2nd Euler angle.
 * @param rE3 3rd Euler angle.
 */
void SimbodyEngine::convertDirectionCosinesToAngles(double aDirCos[3][3], double *rE1, double *rE2, double *rE3) const
{
	Vec3 ang = Rotation(Rotation::getAs(&aDirCos[0][0])).convertRotationToBodyFixedXYZ();
	*rE1 = ang[0];
	*rE2 = ang[1];
	*rE3 = ang[2];
}

//_____________________________________________________________________________
/**
 * Convert direction cosines to angles.
 * @param aDirCos Vector of direction cosines.
 * @param rE1 1st Euler angle.
 * @param rE2 2nd Euler angle.
 * @param rE3 3rd Euler angle.
 */
void SimbodyEngine::convertDirectionCosinesToAngles(double *aDirCos, double *rE1, double *rE2, double *rE3) const
{
	if(!aDirCos) return;
	Vec3 ang = Rotation(Rotation::getAs(aDirCos)).convertRotationToBodyFixedXYZ();
	*rE1 = ang[0];
	*rE2 = ang[1];
	*rE3 = ang[2];
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
void SimbodyEngine::convertDirectionCosinesToQuaternions(double aDirCos[3][3], double *rQ1, double *rQ2, double *rQ3, double *rQ4) const
{
	Quaternion quat = Rotation(Rotation::getAs(&aDirCos[0][0])).convertRotationToQuaternion();
	*rQ1 = quat[0];
	*rQ2 = quat[1];
	*rQ3 = quat[2];
	*rQ4 = quat[3];
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
void SimbodyEngine::convertDirectionCosinesToQuaternions(double *aDirCos, double *rQ1, double *rQ2, double *rQ3, double *rQ4) const
{
	if(aDirCos==NULL) return;
	Quaternion quat = Rotation(Rotation::getAs(aDirCos)).convertRotationToQuaternion();
	*rQ1 = quat[0];
	*rQ2 = quat[1];
	*rQ3 = quat[2];
	*rQ4 = quat[3];
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
void SimbodyEngine::convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double rDirCos[3][3]) const
{
	Rotation R;
	R.setRotationFromQuaternion(Quaternion(Vec4(aQ1, aQ2, aQ3, aQ4)));

	Mat33::updAs(&rDirCos[0][0]) = R.asMat33();
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
void SimbodyEngine::convertQuaternionsToDirectionCosines(double aQ1, double aQ2, double aQ3, double aQ4, double *rDirCos) const
{
	if(rDirCos==NULL) return;
	Rotation R;
	R.setRotationFromQuaternion(Quaternion(Vec4(aQ1, aQ2, aQ3, aQ4)));

	Mat33::updAs(rDirCos) = R.asMat33();
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
void SimbodyEngine::
resizeBodyAndMobilityForceVectors()
{
	int nb = _system->getMatterSubsystem().getNBodies();
	int nm = _system->getMatterSubsystem().getNMobilities();
	_bodyForces.resize(nb);
	_mobilityForces.resize(nm);
}
//_____________________________________________________________________________
/**
 * Reset the vectors of accumulated forces, torques, and generalized forces,
 * meaning set all of them equal to zero.
 */
void
SimbodyEngine::
resetBodyAndMobilityForceVectors()
{
	_bodyForces.setToZero();
	_mobilityForces = 0.0;
}

void SimbodyEngine::formEulerTransform(const AbstractBody &aBody, double *rE) const
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
//_____________________________________________________________________________
/**
 * Set a flag to indicate that the dynamics need to be reconstructed from scratch. ,
 * should be called whenever a recreation of the MultibodySystem is required 
 * 
 */
void SimbodyEngine::setInvalid() 
{
	_invalid = true;
}
//_____________________________________________________________________________
/**
 * A high level interface to bring SimTK::Simbody engine in sync. with SimbodyEngine
 * The SimbodyEngine knows based on the _invalid and desired stage what needs to be done.
 */
void SimbodyEngine::updateDynamics(SimTK::Stage desiredStage)
{
	if (_invalid){
		//constructMultibodySystem();
		_system->realize(_s, desiredStage); 
		_invalid = false;
	}
}

/**
 * A high level interface to bring SimTK::Simbody engine in sync. with model parameters
 * and toplogy. Requires regenerating the state.
 */
void SimbodyEngine::updateSimbodyModel()
{
	// Once setup is complete, realizing toplogy should return
	// the complete set of states for the system
	_s = _system->realizeTopology();
	_s = _system->getDefaultState();

	// Resize and reset body and mobility force vectors based on topology
	resizeBodyAndMobilityForceVectors();
	resetBodyAndMobilityForceVectors();

	// TODO: This should go into a separate initializeState() method if we decide
	// to separate setup as assempbling the model to initializing which makes
	// sure the state of the model is valid.
	// Go through the coordinate set and make sure the locking flags
	// and coordinate values are in the state
	for(int i = 0; i<_coordinateSet.getSize(); i++){
		Coordinate *aCoord = dynamic_cast<Coordinate *>(_coordinateSet.get(i));
		aCoord->initializeState(_s);
}

	// If a coordinate is locked, or prescribed it's initial velocity should be zero
	for(int i = 0; i<_speedSet.getSize(); i++){
		Speed *aSpeed = dynamic_cast<Speed *>(_speedSet.get(i));
		if(aSpeed->getCoordinate()->getLocked())
			aSpeed->setValue(0.0);
		else
			aSpeed->setValue(aSpeed->getDefaultValue());
	}

	for(int i = 0; i<_constraintSet.getSize(); i++){
		Constraint *aConstraint = dynamic_cast<Constraint *>(_constraintSet.get(i));
		aConstraint->initializeState(_s);
	}
}

//_____________________________________________________________________________
/**
 * Update inertial properties of the SimTK::Body to match those of the passed in 
 * OpenSim::Body. 
 * Engine is marked as invalid as a side effect of this call. 
 * 
 */
void SimbodyEngine::updateBodyInertia(const AbstractBody* aBody)
{
	const Body* body = dynamic_cast<const Body*>(aBody);
	SimTK::MobilizedBody&    mobilizedBody=   _system->updMatterSubsystem().updMobilizedBody(body->_index);

	const SimTK::MassProperties& currentMassProperties = mobilizedBody.getDefaultMassProperties();
	SimTK::Inertia originalInertia = SimTK::Inertia(body->_inertiaXX, body->_inertiaYY, body->_inertiaZZ,
												body->_inertiaXY, body->_inertiaXZ, body->_inertiaYZ);
	SimTK::Inertia shiftedInertia = originalInertia.shiftFromMassCenter(-SimTK::Vec3(body->_massCenter),body->_mass);
	mobilizedBody.setDefaultMassProperties(
			SimTK::MassProperties(body->_mass, body->_massCenter, shiftedInertia));
	
	updateSimbodyModel();
	setInvalid();
}
//_____________________________________________________________________________
/**
 * Get a flat list of Joints contained in the model
 * 
 */
JointSet* SimbodyEngine::getJointSet()
{ 
	_jointSet.setMemoryOwner(false);
	_jointSet.setSize(0);
	for(int i=0; i< getNumBodies(); i++){
		AbstractJoint* nextJoint = ((Body*)_bodySet.get(i))->getJoint();
		if (nextJoint)	// Ground body doesn't have a jnt
			_jointSet.append(nextJoint);
	}
	return &_jointSet; 
}
