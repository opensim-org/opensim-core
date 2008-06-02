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
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/SpeedSet.h>
#include <OpenSim/Simulation/Model/JointSet.h>
#include <OpenSim/Simulation/Model/AbstractDof.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractMuscle.h>
#include "SimbodyEngine.h"
#include "SimbodyOpenSimUserForces.h"
#include "SimbodyTranslationDof.h"
#include "SimbodyRotationDof.h"


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


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct the underlying Simbody multibody system for the model.
 */
void SimbodyEngine::constructMultibodySystem()
{
	deleteSimbodyVariables();
	newSimbodyVariables();

	// Get the ground body
	createGroundBodyIfNecessary();
	SimbodyBody *ground = (SimbodyBody*)_bodySet.get(simbodyGroundName);
	if (ground==NULL) return;
	// Add rigid bodies
	// This is a recursive method that attaches ALL bodies (children,
	// grand children, etc. to the body sent in as the argument.
	// So, sending in the ground body will construct the entir
	// multibody system.
	addRigidBodies(ground);

	// Put subsystems into system
	_system->setMatterSubsystem(*_matter);
	_system->addForceSubsystem(*_gravitySubsystem);
	_system->addForceSubsystem(*_userForceElements);
	SimbodyOpenSimUserForces *osimForces = new SimbodyOpenSimUserForces(this);
	_userForceElements->addUserForce(osimForces);
	
	// Realize the state
	_system->realize(*_s,Stage::Velocity);

	// RESIZE AND RESET BODY AND MOBILITY FORCE VECTORS
	resizeBodyAndMobilityForceVectors();
	resetBodyAndMobilityForceVectors();

	// Gravity
	setGravity(_gravity);
}

//_____________________________________________________________________________
/**
 * Construct the underlying Simbody multibody system for the model.
 */
void SimbodyEngine::addRigidBodies(SimbodyBody *aBody)
{
	// HOOK UP CHILDREN
	int iJoint = 0;
	SimbodyJoint *joint;
	for(joint=getOutboardTreeJoint(aBody,iJoint);joint!=NULL;iJoint++,joint=getOutboardTreeJoint(aBody,iJoint)) {

		// GET CHILD
		string childName = joint->getChildBodyName();
		SimbodyBody *child = (SimbodyBody*)_bodySet.get(childName);
		if(child==NULL) {
			// THROW EXCEPTION
			cerr<<"SimbodyEngine.addRigidBodies: joint "<<joint->getName()<<" has no child."<<endl;
		}

		// SOME VARIABLE DECLARATIONS AND INITIALIZATIONS
		SimTK::BodyId parentId = aBody->_id;
		SimTK::BodyId childId;
		DofSet *dofSet = joint->getDofSet();
		int nDof = dofSet->getSize();
		int dofIndexForLastCoordinate = findIndexOfDofThatHasLastGeneralizedCoordinate(dofSet);
		//cout<<"Dof that has last gen. coord is at index "<<dofIndexForLastCoordinate<<" in DofSet."<<endl;
		bool parentTranslationSet = false;
		Vec3 locationInParent(0,0,0);
		bool custom = false;
		Function *txFunction=NULL,*tyFunction=NULL;

		// CHECK FOR NO DEGREES OF FREEDOM
		int nConstDof = 0;
		bool weld = false;
		for(int i=0;i<nDof;i++) {
			AbstractDof *dof = dofSet->get(i);
			if(dof==NULL) continue;
			string dofName = dof->getName();
			//cout<<"SimbodyEngine.addRigidBodies: creating dof "<<dofName<<" between "<<aBody->getName()<<" and "<<childName<<"."<<endl;
			bool translation = (dofName=="tx")||(dofName=="ty")||(dofName=="tz");
			bool rotation = (dofName=="r1")||(dofName=="r2")||(dofName=="r3");

			Function *dofFunction = dof->getFunction();
			if(dofFunction==NULL) continue;
			string functionType = dofFunction->getType();
			if(functionType=="Constant") nConstDof++;
		}
		// Weld Joint
		// If all degrees of freedom are a constant, then this is a weld joint.
		// Weld joints are not currently supported in Simbody.
		if(nConstDof == nDof) {
			weld = true;
			string msg = "A joint with no degrees of freedom has been detected between ";
			msg += aBody->getName();
			msg += " and ";
			msg += child->getName();
			msg += ".  ";
			msg += "The Simbody dynamics engine does not currently support weld joints.  ";
			msg += "Either add a degree of freedom and lock it, or make an SDFast-based model.";
			throw Exception(msg,__FILE__,__LINE__);
		}

		// DETERMIN JOINT LOCATION IN PARENT
		// This section of code accumulates constant offsets between parent and child
		// in the variable locationInParent.
		for(int i=0;i<nDof;i++) {
			AbstractDof *dof = dofSet->get(i);
			if(dof==NULL) continue;

			Function *dofFunction = dof->getFunction();
			if(dofFunction==NULL) continue;
			string functionType = dofFunction->getType();

			if(functionType=="Constant") {

				string dofName = dof->getName();
				bool translation = (dofName=="tx")||(dofName=="ty")||(dofName=="tz");
				bool rotation = (dofName=="r1")||(dofName=="r2")||(dofName=="r3");

				// Translation
				if(translation) {
					if(dofName=="tx") {
						locationInParent[0] = dofFunction->evaluate(0,0.0);
						//cout<<"Setting a constant tx of "<<locationInParent[0]<<endl;
					} else if(dofName=="ty") {
						locationInParent[1] = dofFunction->evaluate(0,0.0);
						//cout<<"Setting a constant ty of "<<locationInParent[1]<<endl;
					} else if(dofName=="tz") {
						locationInParent[2] = dofFunction->evaluate(0,0.0);
						//cout<<"Setting a constant tz of "<<locationInParent[2]<<endl;
					}

				// Rotation
				} else if(rotation) {
					if(dofFunction->evaluate(0,0.0) != 0.0) {
						cout<<"Found constant rotational offset."<<endl;
						cout<<"Constant rotational offsets are not currently supported."<<endl;
						cout<<"Setting the constant rotational offset to 0.0."<<endl;
					}
				}
			}
		}

		// ADD RIGID BODIES AND JOINTS
		nConstDof=0;
		for(int i=0;i<nDof;i++) {

			// GET DOF
			AbstractDof *dof = dofSet->get(i);
			if(dof==NULL) continue;
			string dofName = dof->getName();
			//cout<<"SimbodyEngine.addRigidBodies: creating dof "<<dofName<<" between "<<aBody->getName()<<" and "<<childName<<"."<<endl;
			bool translation = (dofName=="tx")||(dofName=="ty")||(dofName=="tz");
			bool rotation = (dofName=="r1")||(dofName=="r2")||(dofName=="r3");

			// GET AXIS
			Vec3 axis;
			dof->getAxis(axis);
			UnitVec3 unitVec(axis);

			// TRAP ON TYPE OF DOF
			// A dof can be a constant or a linear or non-linear function of a
			// generalized coordinate.  Constants simply reflect a constant offset
			// between the parent and child frames.  Linear implies a direct
			// correspondence with a generalized coordinate.  Non-linear implies
			// that a complex relationship between the dof and the generalized
			// coordinate, and this generally requires a custom moblizer.
			// The only custom mobilizer that is supported currently is the
			// RotToPlanar mobilizer that represents the kinematics of the human
			// knee or similar joints in which the child body translates wrt
			// the parent body as a function of a joint angle.
			Function *dofFunction = dof->getFunction();
			if(dofFunction!=NULL) {
				string functionType = dofFunction->getType();

				// CONSTANT
				if(functionType=="Constant") {
					continue;
	
				// FUNCTION
				} else {
					double mx,my,mz;
					dofFunction->isLinear(1.0e-3,-1,1,mx,-1,1,my,-1,1,mz);

					// Linear (slider or pin mobilizer)
					if(!rdMath::isNAN(mx)) {
						//cout<<"SimbodyEngine.addRigidBodies: dof depends linearly on its gen coord ";
						//cout<<"with a slope of "<<mx<<"."<<endl;

					// Non-linear (custom mobilizer)
					} else if(rdMath::isNAN(mx)) {
						//cout<<"SimbodyEngine.addRigidBodies: "<<joint->getName()<<" is a custom joint."<<endl;
						custom = true;
						if(dofName=="tx") {
							txFunction = dofFunction;
						} else if(dofName=="ty") {
							tyFunction = dofFunction;
						} else {
							string msg = "SimbodyEngine.addRigidBodies: ERR- only Rot2Planar custom joints are supported.";
							throw Exception(msg,__FILE__,__LINE__);
						}
						continue;
					}
				}
			}	

			// CHILD TRANSFORM
			// Only the last axis gets a potentially non-zero location in the child frame.
			Vec3 childTranslation(0,0,0);
			if(i==dofIndexForLastCoordinate) {
				joint->getLocationInChild(childTranslation);
			}
			Rotation childRotation;
			if(translation) {
				double theta = acos(unitVec[0]);
				Rotation RzTheta = Rotation::aboutZ(theta);
				double phi = acos(unitVec[1]);
				Rotation RxPhi = Rotation::aboutX(phi);
				childRotation = RxPhi * RzTheta;
			} else if(rotation) {
				Rotation R(unitVec);
				childRotation = R;
			}
			SimTK::Transform childTransform(childRotation,childTranslation);

			// PARENT TRANSFORM
			// In addition to translations being specified in tx, ty, and tz constant dofs,
			// the location of the mobilizer in the parent frame can also be explicitly specified.
			// If both mechanisms are used, the two are assumed to add together.
			// Setting the location of the mobilizer frame in the parent should only happen once.
			// The "parentTranslationSet" flag is used to make sure this is the case.
			Vec3 parentTranslation(0,0,0);
			if(!parentTranslationSet) {
				joint->getLocationInParent(parentTranslation);
				parentTranslation += locationInParent;
				parentTranslationSet = true;
			}
			// Note- parent and child mobilizer frame rotations are the same,
			// so we can just use the child rotation.
			//cout<<"parentTranslation = "<<parentTranslation<<endl;
			SimTK::Transform parentTransform(childRotation,parentTranslation);

			// INERTIAL PROPERTIES
			// Only the real child body (i.e., the last body) gets mass
			double mass = 0.0;
			SimTK::Vec3 com(0.0, 0.0, 0.0);
			Mat33 inertia(0.0);
			if(i==dofIndexForLastCoordinate) {
				mass = child->getMass();
				child->getMassCenter(com);
				child->getInertia(inertia);
			}
			Vec3 massCenter(0,0,0);
			massCenter = com;
			Inertia inertiaTensor(inertia[0][0],inertia[1][1],inertia[2][2],inertia[0][1],inertia[0][2],inertia[1][2]);
			Inertia inertiaTensorAboutBodyFrame = inertiaTensor.shiftFromMassCenter(-Vec3(com),mass);
         MassProperties massProps(mass,massCenter,inertiaTensorAboutBodyFrame);
			//cout<<"\n"<<child->getName()<<massProps<<endl;

			// ADD RIGID BODY
			// Custom
			if(custom) {
				if(txFunction==NULL) {
					string msg = "SimbodyEngine.addRigidBodies: ERR- function specifying tx for a Rot2Planar mobilizer is missing.";
					throw Exception(msg,__FILE__,__LINE__);
				}
				if(tyFunction==NULL) {
					string msg = "SimbodyEngine.addRigidBodies: ERR- function specifying ty for a Rot2Planar mobilizer is missing.";
					throw Exception(msg,__FILE__,__LINE__);
				}
				//cout<<"Making a Rot2Planar mobilizer."<<endl;
				childId = _matter->addRigidBody(massProps,childTransform,parentId,parentTransform,Mobilizer::Rot2Planar(txFunction,tyFunction));

			// Translation (Slider)
			} else if(translation) {
				//cout<<"slider axis = "<<unitVec<<endl;
				childId = _matter->addRigidBody(massProps,childTransform,parentId,parentTransform,Mobilizer::Slider());

			// Rotation (Pin)
			} else if(rotation) {
				//cout<<"pin axis = "<<unitVec<<endl;
				childId = _matter->addRigidBody(massProps,childTransform,parentId,parentTransform,Mobilizer::Pin());

			// A Number of Constant DOFs Preceding Child (Weld)
			} else if(weld) {
				cout<<"Last dofs in joint were constant. Adding a weld joint."<<endl;

				/* 
				// The only way to do a weld joint in the current version of Simbody is to use a constraint.
				// This weld joint is done by adding a slider joint and constraining it.
				childId = _matter->addRigidBody(massProps,childTransform,parentId,parentTransform,Mobilizer::Slider());
				Vec3 pParent,pChild;
				pParent = 0.0;
				pChild = 0.0;
				pParent += parentTranslation;
				pChild += childTranslation;
				_matter->addConstantDistanceConstraint(parentId,pParent,childId,pChild,0.0);

				// A coordinate and speed need to be added.
				// Coordinate
				SimbodyCoordinate *qWeld = new SimbodyCoordinate();
				char childIdString[128];
				sprintf(childIdString,"%d",childId);
				string nameOfWeldQ = child->getName() + "_weld";
				nameOfWeldQ += childIdString;
				qWeld->setName(nameOfWeldQ);
				qWeld->_defaultValue = 0.0;
				qWeld->_initialValue = 0.0;
				qWeld->_locked = true;
				qWeld->_motionType = AbstractDof::Translational;
				_coordinateSet.append(qWeld);
				// Speed
				SimbodySpeed *uWeld = new SimbodySpeed();
				string nameOfWeldU = AbstractSpeed::getSpeedName(nameOfWeldQ);
				uWeld->setName(nameOfWeldU);
				uWeld->_coordinateName = nameOfWeldQ;
				uWeld->_defaultValue = 0.0;
				_speedSet.append(uWeld);
				// Assign Dof Name
				dof->setCoordinateName(nameOfWeldQ);
				*/
			}

			// UPDATE BODY ID IN REAL CHILD BODY
			// This is only done for the last axis
			if(i==dofIndexForLastCoordinate) {
				child->_id = childId;
			}

			// SET IDs FOR COORDINATES AND SPEEDS
			// Coordinate
			string qName = dof->getCoordinateName();
			SimbodyCoordinate *q = (SimbodyCoordinate*)_coordinateSet.get(qName);
			if(q==NULL) {
				string msg = "SimbodyEngine.addRigidBodies: ERR- coordinate ";
				msg += qName;
				msg += " not found in coordinate set.";
				throw OpenSim::Exception(msg,__FILE__,__LINE__);
			}
			q->_bodyId = childId;
			q->_mobilityIndex = 0;
			if(rotation){
				q->_motionType = AbstractDof::Rotational;
			} else {
				q->_motionType = AbstractDof::Translational;
			}
			// Speed
			string uName = AbstractSpeed::getSpeedName(qName);
			SimbodySpeed *u = (SimbodySpeed*)_speedSet.get(uName);
			if(u==NULL) {
				string msg = "SimbodyEngine.addRigidBodies: ERR- speed ";
				msg += uName;
				msg += " not found in speed set.";
				throw Exception(msg,__FILE__,__LINE__);
			}
			u->_bodyId = childId;
			u->_mobilityIndex = 0;
			u->setCoordinate(q);

			// CHECK FOR LOCKED COORDINATES
			// If a coordinate is locked, add a constraint.
			if(q->_locked) {
				cout<<"Handling locked coordinate."<<endl;
				Vec3 axis,d,pParent,pChild;
				dof->getAxis(axis);
				if(rotation) {
					Mtx::PerpendicularUnitVector(axis,pParent);
					d = axis % pParent;
				} else {
					pParent = 0.0;
					d = axis;
				}
				d *= 0.01;
				pChild = pParent + d;
				double distance = d.norm();
				pParent += parentTranslation;
				pChild += childTranslation;
				_matter->addConstantDistanceConstraint(parentId,pParent,childId,pChild,distance);
			}

			// UPDATE PARENT ID
			parentId = childId;
		}

		// ADD RIGID BODIES TO THE CHILD BODY
		addRigidBodies(child);
	}
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
findIndexOfDofThatHasLastGeneralizedCoordinate(DofSet *aDofSet)
{
	int index = 0;
	int nDof = aDofSet->getSize();
	for(int i=0;i<nDof;i++) {

		AbstractDof *dof = aDofSet->get(i);
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
void SimbodyEngine::constructPendulum()
{
	deleteSimbodyVariables();
	newSimbodyVariables();

	// Parameters
	double length = 1.0;
	double mass = 2.0;
	SimTK::Vec3 g(0.0, -9.8, 0.0);

	// Add pendulum mass to matter subsystem
	SimTK::Transform GroundFrame;
	const Vec3 massLocation(0, -length/2, 0);
	const Vec3 jointLocation(0, length/2, 0);
	MassProperties massProps(mass, massLocation, mass*Inertia::pointMassAt(massLocation));
	const BodyId bodyId = _matter->addRigidBody(massProps,jointLocation,GroundId,GroundFrame,Mobilizer::Pin());
	
	// Put subsystems into system
	_system->setMatterSubsystem(*_matter);
	_system->addForceSubsystem(*_gravitySubsystem);
	_system->addForceSubsystem(*_userForceElements);
	SimbodyOpenSimUserForces *osimForces = new SimbodyOpenSimUserForces(this);
	_userForceElements->addUserForce(osimForces);

	// Realize the state
	_system->realize(*_s,Stage::Velocity);

	// RESIZE AND RESET BODY AND MOBILITY FORCE VECTORS
	resizeBodyAndMobilityForceVectors();
	resetBodyAndMobilityForceVectors();

	// Gravity
	setGravity(g);

	// CONSTRUCT CORRESPONDING OPENSIM OBJECTS
	// Body
	createGroundBodyIfNecessary();
	SimbodyBody *body = new SimbodyBody();
	body->setName("Pendulum");
	body->_engine = this;
	body->_id = bodyId;
	body->_mass = mass;
	body->_massCenter[0] = massLocation[0];
	body->_massCenter[1] = massLocation[1];
	body->_massCenter[2] = massLocation[2];
	Vec3 moments = massProps.getInertia().getMoments();
	Vec3 products = massProps.getInertia().getProducts();
	body->_inertia[0] = moments[0];
	body->_inertia[4] = moments[1];
	body->_inertia[8] = moments[2];
	body->_inertia[1] = body->_inertia[3] = products[0];
	body->_inertia[2] = body->_inertia[6] = products[1];
	body->_inertia[5] = body->_inertia[7] = products[2];
	body->setup(this);
	_bodySet.append(body);

	// Coordinate
	SimbodyCoordinate *coordinate = new SimbodyCoordinate();
	coordinate->_engine = this;
	coordinate->_bodyId = bodyId;
	coordinate->_mobilityIndex = 0;
	coordinate->setName("PendulumAxis");
	coordinate->setup(this);
	_coordinateSet.append(coordinate);

	// Speed
	SimbodySpeed *speed = new SimbodySpeed();
	speed->_engine = this;
	speed->_bodyId = bodyId;
	speed->_mobilityIndex = 0;
	speed->setName(AbstractSpeed::getSpeedName(coordinate->getName()));
	speed->setCoordinateName(coordinate->getName());
	speed->setup(this);
	_speedSet.append(speed);

	// Joint
	SimbodyJoint *joint = new SimbodyJoint();
	_jointSet.append(joint);
	joint->setName("GroundJoint");
	joint->_engine = this;
	joint->setParentBodyName(simbodyGroundName);
	joint->setChildBodyName("Pendulum");
	Vec3 jointLocationInParent(0.0);
	joint->setLocationInParent(jointLocationInParent);
	joint->setLocationInChild(jointLocation);
	DofSet *dofSet = joint->getDofSet();
	SimbodyRotationDof *dof = new SimbodyRotationDof();
	dof->setName("r1");
	dof->setCoordinateName("PendulumAxis");
	Vec3 zAxis(0.0);  zAxis[2] = 1.0;
	dof->setAxis(zAxis);
	dofSet->append(dof);
}
//_____________________________________________________________________________
/**
 * Allocate Simbody variables.
 */
void SimbodyEngine::newSimbodyVariables()
{
	_system = new MultibodySystem;
	_matter = new SimbodyMatterSubsystem;
	_gravitySubsystem = new UniformGravitySubsystem;
	_userForceElements = new GeneralForceElements;
	_s = new State;
}
//_____________________________________________________________________________
/**
 * Delete Simbody variables.
 */
void SimbodyEngine::deleteSimbodyVariables()
{
	delete _system;  _system = NULL;
	delete _matter;  _matter = NULL;
	delete _gravitySubsystem;  _gravitySubsystem = NULL;
	delete _userForceElements;  _userForceElements = NULL;
	delete _s;  _s = NULL;
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
	_matter = NULL;
	_gravitySubsystem = NULL;
	_userForceElements = NULL;
	_s = NULL;
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
	SimbodyBody *ground=NULL;
	for(int i=0; i<size; i++) {
		SimbodyBody *body = (SimbodyBody*)_bodySet.get(i);
		if(body->getName() == simbodyGroundName) {
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
		ground = new SimbodyBody();
		_bodySet.append(ground);
	}
    */
	// Set member variables
	ground->setName(simbodyGroundName);
	// Mass
	ground->_mass = 0.0;
	// Mass center
	Vec3 massCenter(0.0);
	ground->_massCenter = massCenter;
	// Inertia
	OpenSim::Array<double> inertia(0.0,9);
	ground->_inertia = inertia;
	// Simbody id
	ground->_id = SimTK::GroundId;
	// Engine
	ground->_engine = this;
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
	constructMultibodySystem();
	AbstractDynamicsEngine::setup(aModel);
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
	Object::RegisterType(SimbodyBody());
	Object::RegisterType(SimbodyJoint());
	Object::RegisterType(SimbodyCoordinate());
	Object::RegisterType(SimbodySpeed());
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
void SimbodyEngine::addBody(SimbodyBody* aBody)
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
void SimbodyEngine::addJoint(SimbodyJoint* aJoint)
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
void SimbodyEngine::addCoordinate(SimbodyCoordinate* aCoord)
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
void SimbodyEngine::addSpeed(SimbodySpeed* aSpeed)
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
void SimbodyEngine::getConfiguration(double rQ[],double rU[]) const
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
void SimbodyEngine::getCoordinates(double rQ[]) const
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
void SimbodyEngine::getSpeeds(double rU[]) const
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
void SimbodyEngine::getAccelerations(double rDUDT[]) const
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
bool SimbodyEngine::setGravity(const SimTK::Vec3& aGrav)
{
	_gravitySubsystem->setGravity(*_s,aGrav);
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
void SimbodyEngine::getGravity(SimTK::Vec3& aGrav) const
{
	aGrav = _gravitySubsystem->getGravity(*_s);
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
AbstractBody& SimbodyEngine::getGroundBody() const
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
SimbodyJoint *SimbodyEngine::
getInboardTreeJoint(SimbodyBody* aBody) const
{
	int nj = _jointSet.getSize();
	for(int i=0; i<nj; i++) {
		SimbodyJoint *joint = (SimbodyJoint*)_jointSet[i];
		SimbodyBody *child = joint->getChildBody();
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
SimbodyJoint *SimbodyEngine::
getOutboardTreeJoint(SimbodyBody* aBody,int &rIndex) const
{
	int nj = _jointSet.getSize();
	while(rIndex<nj) {
		SimbodyJoint *joint = (SimbodyJoint*)_jointSet[rIndex];
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
bool SimbodyEngine::adjustJointVectorsForNewMassCenter(SimbodyBody* aBody)
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
void SimbodyEngine::getSystemInertia(double *rM, SimTK::Vec3& rCOM, double rI[3][3]) const
{
	*rM = _matter->calcSystemMass(*_s);
	rCOM = _matter->calcSystemMassCenterLocationInGround(*_s);
	Mat33::updAs(&rI[0][0]) = _matter->calcSystemCentralInertiaInGround(*_s).toMat33();
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
void SimbodyEngine::getPosition(const AbstractBody &aBody, const SimTK::Vec3& aPoint, SimTK::Vec3& rPos) const
{
	const SimbodyBody* b = (SimbodyBody*)(&aBody);
	rPos = _matter->locateBodyPointOnGround(*_s, b->_id, aPoint);
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
void SimbodyEngine::getVelocity(const AbstractBody &aBody, const SimTK::Vec3& aPoint, SimTK::Vec3& rVel) const
{
	const SimbodyBody* b = (SimbodyBody*)(&aBody);
	rVel = _matter->calcBodyFixedPointVelocityInGround(*_s, b->_id, aPoint);
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
void SimbodyEngine::getAcceleration(const AbstractBody &aBody, const SimTK::Vec3& aPoint, SimTK::Vec3& rAcc) const
{
	const SimbodyBody* b = (SimbodyBody*)(&aBody);
	rAcc = _matter->calcBodyFixedPointAccelerationInGround(*_s, b->_id, aPoint);
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
	const SimbodyBody* b = (SimbodyBody*)(&aBody);
	Mat33::updAs(&rDirCos[0][0]) = _matter->getBodyRotation(*_s, b->_id).asMat33();
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
	const SimbodyBody* b = (SimbodyBody*)(&aBody);
	Mat33::updAs(rDirCos) = _matter->getBodyRotation(*_s, b->_id).asMat33();
}

//_____________________________________________________________________________
/**
 * Get the inertial angular velocity of a body in the ground reference frame.
 *
 * @param aBody Pointer to body.
 * @param rAngVel Angular velocity of the body.
 */
void SimbodyEngine::getAngularVelocity(const AbstractBody &aBody, SimTK::Vec3& rAngVel) const
{
	const SimbodyBody *b = (SimbodyBody*)(&aBody);
	rAngVel = _matter->getBodyAngularVelocity(*_s, b->_id);
}

//_____________________________________________________________________________
/**
 * Get the inertial angular velocity in the local body reference frame.
 *
 * @param aBody Pointer to body.
 * @param rAngVel Angular velocity of the body.
 */
void SimbodyEngine::getAngularVelocityBodyLocal(const AbstractBody &aBody, SimTK::Vec3& rAngVel) const
{
	const SimbodyBody *b = (SimbodyBody*)(&aBody);
	rAngVel = _matter->getBodyAngularVelocity(*_s, b->_id);
}

//_____________________________________________________________________________
/**
 * Get the inertial angular acceleration of a body in the ground reference 
 * frame.
 *
 * @param aBody Pointer to body.
 * @param rAngAcc Angular acceleration of the body.
 */
void SimbodyEngine::getAngularAcceleration(const AbstractBody &aBody, SimTK::Vec3& rAngAcc) const
{
	const SimbodyBody *b = (SimbodyBody*)(&aBody);
	rAngAcc = _matter->getBodyAngularAcceleration(*_s, b->_id);
}

//_____________________________________________________________________________
/**
 * Get the inertial angular acceleration in the local body reference frame.
 *
 * @param aBody Pointer to body.
 * @param rAngAcc Angular acceleration of the body.
 */
void SimbodyEngine::getAngularAccelerationBodyLocal(const AbstractBody &aBody, SimTK::Vec3& rAngAcc) const
{
	const SimbodyBody *b = (SimbodyBody*)(&aBody);
	rAngAcc = _matter->getBodyAngularAcceleration(*_s, b->_id);
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
	double aMat[4][4];
	const SimbodyBody* b = (SimbodyBody*)(&aBody);

	Mat44::updAs(&aMat[0][0]) = _matter->getBodyTransform(*_s, b->_id).toMat44();
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
void SimbodyEngine::applyForce(const AbstractBody &aBody, const SimTK::Vec3& aPoint, const SimTK::Vec3& aForce)
{
	const SimbodyBody* body = (const SimbodyBody*)&aBody;
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
void SimbodyEngine::applyForces(int aN, const AbstractBody *aBodies[], const double aPoints[][3], const double aForces[][3])
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
void SimbodyEngine::applyForceBodyLocal(const AbstractBody &aBody, const SimTK::Vec3& aPoint, const SimTK::Vec3& aForce)
{
	const SimbodyBody* body = (const SimbodyBody*)&aBody;

	// Transform force vector into ground frame
	Vec3 forceInB;
	forceInB = aForce;
	Vec3 forceInG;
	forceInG = _matter->expressBodyVectorInGround(*_s,body->_id,forceInB);
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
	SimTK::Vec3 point, force;

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
void SimbodyEngine::applyTorque(const AbstractBody &aBody, const SimTK::Vec3& aTorque)
{
	const SimbodyBody* b = (SimbodyBody*)&aBody;
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
void SimbodyEngine::applyTorqueBodyLocal(const AbstractBody &aBody, const SimTK::Vec3& aTorque)
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
void SimbodyEngine::applyGeneralizedForce(const AbstractCoordinate &aQ, double aForce)
{
	const SimbodyCoordinate *q = (SimbodyCoordinate*) &aQ;
	_matter->addInMobilityForce(*_s,q->_bodyId,q->_mobilityIndex,aForce,_mobilityForces);
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
	const SimbodyCoordinate *c = (SimbodyCoordinate*)(&aU);



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
void SimbodyEngine::computeReactions(double rForces[][3], double rTorques[][3]) const
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
void SimbodyEngine::computeDerivatives(double *dqdt,double *dudt)
{
	//_s->updTime() = t;

	// COMPUTE ACCELERATIONS
	try {
		_system->realize(*_s,Stage::Acceleration);
	} catch(...) {
		cout<<"SimbodyEngine.computeDerivatives: invalid derivatives.\n\n";
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
void SimbodyEngine::transform(const AbstractBody &aBodyFrom, const double aVec[3], const AbstractBody &aBodyTo, double rVec[3]) const
{
	if(&aBodyFrom == &aBodyTo) {
	   for (int i=0; i<3; i++) rVec[i] = aVec[i];
		return;
	}
	const SimbodyBody* bFrom = (const SimbodyBody*)&aBodyFrom;
	const SimbodyBody* bTo = (const SimbodyBody*)&aBodyTo;

	//Get input vector as a Vec3 to make the call down to Simbody and update the output vector 
	Vec3::updAs(rVec) = _matter->expressBodyVectorInBody(*_s, bFrom->_id, Vec3::getAs(aVec), bTo->_id);
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
void SimbodyEngine::transform(const AbstractBody &aBodyFrom, const SimTK::Vec3& aVec, const AbstractBody &aBodyTo, SimTK::Vec3& rVec) const
{
	if(&aBodyFrom == &aBodyTo) {
	   rVec = aVec;
		return;
	}
	const SimbodyBody* bFrom = (const SimbodyBody*)&aBodyFrom;
	const SimbodyBody* bTo = (const SimbodyBody*)&aBodyTo;

	//Get input vector as a Vec3 to make the call down to Simbody and update the output vector 
	rVec = _matter->expressBodyVectorInBody(*_s, bFrom->_id, aVec, bTo->_id);
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
	if(&aBodyFrom == &aBodyTo) {
	   for (int i=0; i<3; i++) rPos[i] = aPos[i];
		return;
	}
	const SimbodyBody* bFrom = (const SimbodyBody*)&aBodyFrom;
	const SimbodyBody* bTo = (const SimbodyBody*)&aBodyTo;

	//Get input vector as a Vec3 to make the call down to Simbody and update the output vector 
	Vec3::updAs(rPos) = _matter->locateBodyPointOnBody(*_s, bFrom->_id, Vec3::getAs(aPos), bTo->_id);
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
transformPosition(const AbstractBody &aBodyFrom, const SimTK::Vec3& aPos,
	const AbstractBody &aBodyTo, SimTK::Vec3& rPos) const
{
	if(&aBodyFrom == &aBodyTo) {
	   for (int i=0; i<3; i++) rPos[i] = aPos[i];
		return;
	}
	const SimbodyBody* bFrom = (const SimbodyBody*)&aBodyFrom;
	const SimbodyBody* bTo = (const SimbodyBody*)&aBodyTo;

	//Get input vector as a Vec3 to make the call down to Simbody and update the output vector 
	rPos = _matter->locateBodyPointOnBody(*_s, bFrom->_id, aPos, bTo->_id);
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
	const SimbodyBody* bFrom = (const SimbodyBody*)&aBodyFrom;

	//Get input vector as a Vec3 to make the call down to Simbody and update the output vector 
	Vec3::updAs(rPos) = _matter->locateBodyPointOnGround(*_s, bFrom->_id, Vec3::getAs(aPos));
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
transformPosition(const AbstractBody &aBodyFrom,const SimTK::Vec3& aPos,
	SimTK::Vec3& rPos) const
{
	const SimbodyBody* bFrom = (const SimbodyBody*)&aBodyFrom;
	rPos = _matter->locateBodyPointOnGround(*_s, bFrom->_id, aPos);
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
calcDistance(const AbstractBody& aBody1, const SimTK::Vec3& aPoint1,
	const AbstractBody& aBody2, const SimTK::Vec3& aPoint2) const
{
	const SimbodyBody* b1 = (SimbodyBody*)(&aBody1);
	const SimbodyBody* b2 = (SimbodyBody*)(&aBody2);
	return _matter->calcPointToPointDistance(*_s, b1->_id, aPoint1, b2->_id, aPoint2);
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
	const SimbodyBody* b1 = (SimbodyBody*)(&aBody1);
	const SimbodyBody* b2 = (SimbodyBody*)(&aBody2);
	return _matter->calcPointToPointDistance(*_s, b1->_id, Vec3::getAs(aPoint1), b2->_id, Vec3::getAs(aPoint2));
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
	R.setToBodyFixed123(Vec3::getAs(aQAng));

	Vec4::updAs(rQ) = R.convertToQuaternion().asVec4();
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
	aRot.setToBodyFixed123(angs);
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
	aRot.setToBodyFixed123(angs);
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
	Vec3 ang = Rotation (Rotation::getAs(&aDirCos[0][0])).convertToBodyFixed123();
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
	Vec3 ang = Rotation (Rotation::getAs(aDirCos)).convertToBodyFixed123();
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
	Quaternion quat = Rotation (Rotation::getAs(&aDirCos[0][0])).convertToQuaternion();
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
	Quaternion quat = Rotation (Rotation::getAs(aDirCos)).convertToQuaternion();
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
	R.setToQuaternion(Quaternion(Vec4(aQ1, aQ2, aQ3, aQ4)));

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
	R.setToQuaternion(Quaternion(Vec4(aQ1, aQ2, aQ3, aQ4)));

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
