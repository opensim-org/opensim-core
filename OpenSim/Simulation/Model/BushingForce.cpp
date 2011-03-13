// BushingForce.cpp
// Author: Ajay Seth
/*
 * Copyright (c)  2010, Stanford University. All rights reserved. 
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
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>

#include "BushingForce.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
BushingForce::~BushingForce()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
BushingForce::BushingForce() :
	Force(),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec()),
	_orientationInBody1(_orientationInBody1Prop.getValueDblVec()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec()),
	_orientationInBody2(_orientationInBody2Prop.getValueDblVec()),
	_rotStiffness(_rotStiffnessProp.getValueDblVec()),
	_transStiffness(_transStiffnessProp.getValueDblVec()),
	_rotDamping(_rotDampingProp.getValueDblVec()),
	_transDamping(_transDampingProp.getValueDblVec())
{
	setNull();
}

/* Convenience constructor */
BushingForce::BushingForce( std::string body1Name, SimTK::Vec3 point1, SimTK::Vec3 orientation1,
		          std::string body2Name, SimTK::Vec3 point2, SimTK::Vec3 orientation2,
				  SimTK::Vec3 transStiffness, SimTK::Vec3 rotStiffness, SimTK::Vec3 transDamping, SimTK::Vec3 rotDamping ):
	Force(),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec()),
	_orientationInBody1(_orientationInBody1Prop.getValueDblVec()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec()),
	_orientationInBody2(_orientationInBody2Prop.getValueDblVec()),
	_rotStiffness(_rotStiffnessProp.getValueDblVec()),
	_transStiffness(_transStiffnessProp.getValueDblVec()),
	_rotDamping(_rotDampingProp.getValueDblVec()),
	_transDamping(_transDampingProp.getValueDblVec())
{
	setNull();
	_body1Name = body1Name;
	_body2Name = body2Name;
	_locationInBody1 = point1;
	_orientationInBody1 = orientation1;
	_locationInBody2 = point2;
	_orientationInBody2 = orientation2;
	_rotStiffness = rotStiffness;
	_transStiffness = transStiffness;
	_rotDamping = rotDamping;
	_transDamping = transDamping;
}


//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aForce BushingForce to be copied.
 */
BushingForce::BushingForce(const BushingForce &aForce) :
   Force(aForce),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec()),
	_orientationInBody1(_orientationInBody1Prop.getValueDblVec()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec()),
	_orientationInBody2(_orientationInBody2Prop.getValueDblVec()),
	_rotStiffness(_rotStiffnessProp.getValueDblVec()),
	_transStiffness(_transStiffnessProp.getValueDblVec()),
	_rotDamping(_rotDampingProp.getValueDblVec()),
	_transDamping(_transDampingProp.getValueDblVec())
{
	setNull();
	copyData(aForce);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy this body and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this OpenSim::Body.
 */
Object* BushingForce::copy() const
{
	BushingForce *Force = new BushingForce(*this);
	return(Force);
}
//_____________________________________________________________________________
/**
 * Copy data members from one BushingForce to another.
 *
 * @param aForce BushingForce to be copied.
 */
void BushingForce::copyData(const BushingForce &aForce)
{
	Force::copyData(aForce);
	_body1Name = aForce._body1Name;
	_body2Name = aForce._body2Name;
	_locationInBody1 = aForce._locationInBody1;
	_orientationInBody1 = aForce._orientationInBody1;
	_locationInBody2 = aForce._locationInBody2;
	_orientationInBody2 = aForce._orientationInBody2;
	_rotStiffness = aForce._rotStiffness;
	_transStiffness = aForce._transStiffness;
	_rotDamping = aForce._rotDamping;
	_transDamping = aForce._transDamping;
}

//_____________________________________________________________________________
/**
 * Set the data members of this BushingForce to their null values.
 */
void BushingForce::setNull()
{
	setType("BushingForce");
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void BushingForce::setupProperties()
{
	// Body 1 name
	_body1NameProp.setName("body_1");
	_propertySet.append(&_body1NameProp);

	// Body 2 name
	_body2NameProp.setName("body_2");
	_propertySet.append(&_body2NameProp);

	//Default location and orientation (rotation sequence)
	SimTK::Vec3 origin(0.0, 0.0, 0.0);

	// Location in Body 1 
	_locationInBody1Prop.setName("location_body_1");
	_locationInBody1Prop.setValue(origin);
	_propertySet.append(&_locationInBody1Prop);

	// Orientation in Body 1 
	_orientationInBody1Prop.setName("orientation_body_1");
	_orientationInBody1Prop.setValue(origin);
	_propertySet.append(&_orientationInBody1Prop);

	// Location in Body 2 
	_locationInBody2Prop.setName("location_body_2");
	_locationInBody2Prop.setValue(origin);
	_propertySet.append(&_locationInBody2Prop);

	// Orientation in Body 2 
	_orientationInBody2Prop.setName("orientation_body_2");
	_orientationInBody2Prop.setValue(origin);
	_propertySet.append(&_orientationInBody2Prop);

	_rotStiffnessProp.setName("rotational_stiffness");
	_rotStiffnessProp.setValue(Vec3(0));
	_propertySet.append(&_rotStiffnessProp);

	_transStiffnessProp.setName("translational_stiffness");
	_transStiffnessProp.setValue(Vec3(0));
	_propertySet.append(&_transStiffnessProp);

	_rotDampingProp.setName("rotational_damping");
	_rotDampingProp.setValue(Vec3(0));
	_propertySet.append(&_rotDampingProp);

	_transDampingProp.setName("translational_damping");
	_transDampingProp.setValue(Vec3(0));
	_propertySet.append(&_transDampingProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this BushingForce.
 */
void BushingForce::setup(Model& aModel)
{
	string errorMessage;

	// Base class
	Force::setup(aModel);

	// Look up the two bodies being connected by bushing by name in the
	// model and might as well keep a pointer to them
	if (!aModel.updBodySet().contains(_body1Name)) {
		errorMessage = "Invalid bushing body1 (" + _body1Name + ") specified in Force " + getName();
		throw (Exception(errorMessage.c_str()));
	}
	if (!aModel.updBodySet().contains(_body2Name)) {
		errorMessage = "Invalid bushing body2 (" + _body2Name + ") specified in Force " + getName();
		throw (Exception(errorMessage.c_str()));
	}
}

void BushingForce::createSystem(SimTK::MultibodySystem& system) const
{
	Body &body1 = _model->updBodySet().get(_body1Name);
	Body &body2 = _model->updBodySet().get(_body2Name);

	// Get underlying mobilized bodies
	SimTK::MobilizedBody b1 = _model->updMatterSubsystem().getMobilizedBody(body1.getIndex());
	SimTK::MobilizedBody b2 = _model->updMatterSubsystem().getMobilizedBody(body2.getIndex());
	// Build the transforms
	SimTK::Rotation r1; r1.setRotationToBodyFixedXYZ(_orientationInBody1);
	SimTK::Rotation r2; r2.setRotationToBodyFixedXYZ(_orientationInBody2);
	SimTK::Transform inb1(r1, _locationInBody1);
	SimTK::Transform inb2(r2, _locationInBody2);

	Vec6 stiffness(_rotStiffness[0], _rotStiffness[1], _rotStiffness[2], _transStiffness[0], _transStiffness[1], _transStiffness[2]);
	Vec6 damping(_rotDamping[0], _rotDamping[1], _rotDamping[2], _transDamping[0], _transDamping[1], _transDamping[2]);

    // Now create a Simbody Force::LinearBushing
    SimTK::Force::LinearBushing simtkForce(_model->updForceSubsystem(), b1, inb1, b2, inb2, stiffness, damping);
    
    // Beyond the const Component get the index so we can access the SimTK::Force later
	BushingForce* mutableThis = const_cast<BushingForce *>(this);
	mutableThis->_index = simtkForce.getForceIndex();
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
BushingForce& BushingForce::operator=(const BushingForce &aForce)
{
	Force::operator=(aForce);
	copyData(aForce);
	return(*this);
}

//=============================================================================
// SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Following methods set attributes of the weld Force */
void BushingForce::setBody1ByName(std::string aBodyName)
{
	_body1Name = aBodyName;
}

void BushingForce::setBody2ByName(std::string aBodyName)
{
	_body2Name = aBodyName;
}

/** Set the location and orientation (optional) for weld on body 1*/
void BushingForce::setBody1BushingLocation(Vec3 location, Vec3 orientation)
{
	_locationInBody1 = location;
	_orientationInBody1 = orientation;
}

/** Set the location and orientation (optional) for weld on body 2*/
void BushingForce::setBody2BushingLocation(Vec3 location, Vec3 orientation)
{
	_locationInBody2 = location;
	_orientationInBody2 = orientation;
}

/* Potential energy is computed by underlying SimTK::Force. */
double BushingForce::computePotentialEnergy(const SimTK::State& s) const
{
	return _model->getForceSubsystem().getForce(_index).calcPotentialEnergyContribution(s);
}

//=============================================================================
// Reporting
//=============================================================================
/** 
 * Provide names of the quantities (column labels) of the force value(s) reported
 * 
 */
OpenSim::Array<std::string> BushingForce::getRecordLabels() const 
{
	OpenSim::Array<std::string> labels("");
	labels.append(getName()+"."+_body1Name+".force.X");
	labels.append(getName()+"."+_body1Name+".force.Y");
	labels.append(getName()+"."+_body1Name+".force.Z");
	labels.append(getName()+"."+_body1Name+".torque.X");
	labels.append(getName()+"."+_body1Name+".torque.Y");
	labels.append(getName()+"."+_body1Name+".torque.Z");
	labels.append(getName()+"."+_body2Name+".force.X");
	labels.append(getName()+"."+_body2Name+".force.Y");
	labels.append(getName()+"."+_body2Name+".force.Z");
	labels.append(getName()+"."+_body2Name+".torque.X");
	labels.append(getName()+"."+_body2Name+".torque.Y");
	labels.append(getName()+"."+_body2Name+".torque.Z");

	return labels;
}
/**
 * Provide the value(s) to be reported that correspond to the labels
 */
OpenSim::Array<double> BushingForce::getRecordValues(const SimTK::State& state) const 
{
	OpenSim::Array<double> values(1);

	const SimTK::Force::LinearBushing &simtkSpring = (SimTK::Force::LinearBushing &)(_model->getForceSubsystem().getForce(_index));

	SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
	SimTK::Vector_<SimTK::Vec3> particleForces(0);
	SimTK::Vector mobilityForces(0);

	//get the net force added to the system contributed by the bushing
	simtkSpring.calcForceContribution(state, bodyForces, particleForces, mobilityForces);
	SimTK::Vec3 forces = bodyForces(_model->getBodySet().get(_body1Name).getIndex())[1];
	SimTK::Vec3 torques = bodyForces(_model->getBodySet().get(_body1Name).getIndex())[0];
	values.append(3, &forces[0]);
	values.append(3, &torques[0]);

	forces = bodyForces(_model->getBodySet().get(_body2Name).getIndex())[1];
	torques = bodyForces(_model->getBodySet().get(_body2Name).getIndex())[0];

	values.append(3, &forces[0]);
	values.append(3, &torques[0]);

	return values;
}
