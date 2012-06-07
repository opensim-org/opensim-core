// BushingForce.cpp
// Author: Ajay Seth
/*
 * Copyright (c)  2010-12, Stanford University. All rights reserved. 
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

using namespace std;
using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, and copy
// assignment operator.

//_____________________________________________________________________________
// Default constructor.
BushingForce::BushingForce()
{
	setNull();
    constructProperties();
}

// Convenience constructor.
BushingForce::BushingForce(const string&    body1Name, 
                           const Vec3&      point1, 
                           const Vec3&      orientation1,
		                   const string&    body2Name, 
                           const Vec3&      point2, 
                           const Vec3&      orientation2,
				           const Vec3&      transStiffness, 
                           const Vec3&      rotStiffness, 
                           const Vec3&      transDamping, 
                           const Vec3&      rotDamping)
{
	setNull();
    constructProperties();

	set_body_1(body1Name);
	set_body_2(body2Name);
	set_location_body_1(point1);
	set_orientation_body_1(orientation1);
	set_location_body_2(point2);
	set_orientation_body_2(orientation2);
	set_rotational_stiffness(rotStiffness);
	set_translational_stiffness(transStiffness);
	set_rotational_damping(rotDamping);
	set_translational_damping(transDamping);
}

//_____________________________________________________________________________
// Set the data members of this BushingForce to their null values.
void BushingForce::setNull()
{
    // no data members
}

//_____________________________________________________________________________
// Allocate and initialize properties.
void BushingForce::constructProperties()
{
	constructProperty_body_1();
	constructProperty_body_2();

	//Default location and orientation (rotation sequence)
	constructProperty_location_body_1(Vec3(0)); // body origin
	constructProperty_orientation_body_1(Vec3(0)); // no rotation
	constructProperty_location_body_2(Vec3(0));
	constructProperty_orientation_body_2(Vec3(0));
	constructProperty_rotational_stiffness(Vec3(0));
	constructProperty_translational_stiffness(Vec3(0));
	constructProperty_rotational_damping(Vec3(0));
	constructProperty_translational_damping(Vec3(0));
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this BushingForce.
 */
void BushingForce::connectToModel(Model& aModel)
{
	Super::connectToModel(aModel); // base class first

	string errorMessage;
	const string& body1Name = get_body_1(); // error if unspecified
	const string& body2Name = get_body_2();


	// Look up the two bodies being connected by bushing by name in the
	// model. TODO: keep a pointer to them?
	if (!aModel.updBodySet().contains(body1Name)) {
		errorMessage = "Invalid bushing body1 (" + body1Name 
                        + ") specified in Force " + getName();
		throw OpenSim::Exception(errorMessage);
	}
	if (!aModel.updBodySet().contains(body2Name)) {
		errorMessage = "Invalid bushing body2 (" + body2Name 
                        + ") specified in Force " + getName();
		throw OpenSim::Exception(errorMessage);
	}
}

void BushingForce::addToSystem(SimTK::MultibodySystem& system) const
{
    Super::addToSystem(system);

	const string&      body1Name            = get_body_1();
	const string&      body2Name            = get_body_2();
	const SimTK::Vec3& locationInBody1      = get_location_body_1();
	const SimTK::Vec3& orientationInBody1   = get_orientation_body_1();
	const SimTK::Vec3& locationInBody2      = get_location_body_2();
	const SimTK::Vec3& orientationInBody2   = get_orientation_body_2();
	const SimTK::Vec3& rotStiffness         = get_rotational_stiffness();
	const SimTK::Vec3& transStiffness       = get_translational_stiffness();
	const SimTK::Vec3& rotDamping           = get_rotational_damping();
	const SimTK::Vec3& transDamping         = get_translational_damping();

	Body& body1 = _model->updBodySet().get(body1Name);
	Body& body2 = _model->updBodySet().get(body2Name);

	// Get underlying mobilized bodies
	SimTK::MobilizedBody b1 = _model->updMatterSubsystem().getMobilizedBody(body1.getIndex());
	SimTK::MobilizedBody b2 = _model->updMatterSubsystem().getMobilizedBody(body2.getIndex());
	// Build the transforms
	SimTK::Rotation r1; r1.setRotationToBodyFixedXYZ(orientationInBody1);
	SimTK::Rotation r2; r2.setRotationToBodyFixedXYZ(orientationInBody2);
	SimTK::Transform inb1(r1, locationInBody1);
	SimTK::Transform inb2(r2, locationInBody2);

	Vec6 stiffness(rotStiffness[0], rotStiffness[1], rotStiffness[2], 
                   transStiffness[0], transStiffness[1], transStiffness[2]);
	Vec6 damping(rotDamping[0], rotDamping[1], rotDamping[2], 
                 transDamping[0], transDamping[1], transDamping[2]);

    // Now create a Simbody Force::LinearBushing
    SimTK::Force::LinearBushing simtkForce
       (_model->updForceSubsystem(), b1, inb1, b2, inb2, stiffness, damping);
    
    // Beyond the const Component get the index so we can access the 
    // SimTK::Force later.
	BushingForce* mutableThis = const_cast<BushingForce *>(this);
	mutableThis->_index = simtkForce.getForceIndex();
}

//=============================================================================
// SET
//=============================================================================
//_____________________________________________________________________________
// The following methods set properties of the bushing Force.
void BushingForce::setBody1ByName(const std::string& aBodyName)
{
	set_body_1(aBodyName);
}

void BushingForce::setBody2ByName(const std::string& aBodyName)
{
	set_body_2(aBodyName);
}

/** Set the location and orientation (optional) for weld on body 1*/
void BushingForce::setBody1BushingLocation(const Vec3& location, 
                                           const Vec3& orientation)
{
	set_location_body_1(location);
	set_orientation_body_1(orientation);
}

/** Set the location and orientation (optional) for weld on body 2*/
void BushingForce::setBody2BushingLocation(const Vec3& location, 
                                           const Vec3& orientation)
{
	set_location_body_2(location);
	set_orientation_body_2(orientation);
}

/* Potential energy is computed by underlying SimTK::Force. */
double BushingForce::computePotentialEnergy(const SimTK::State& s) const
{
	return _model->getForceSubsystem().getForce(_index)
                                      .calcPotentialEnergyContribution(s);
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
	const string& body1Name = get_body_1();
	const string& body2Name = get_body_2();

	OpenSim::Array<std::string> labels("");
	labels.append(getName()+"."+body1Name+".force.X");
	labels.append(getName()+"."+body1Name+".force.Y");
	labels.append(getName()+"."+body1Name+".force.Z");
	labels.append(getName()+"."+body1Name+".torque.X");
	labels.append(getName()+"."+body1Name+".torque.Y");
	labels.append(getName()+"."+body1Name+".torque.Z");
	labels.append(getName()+"."+body2Name+".force.X");
	labels.append(getName()+"."+body2Name+".force.Y");
	labels.append(getName()+"."+body2Name+".force.Z");
	labels.append(getName()+"."+body2Name+".torque.X");
	labels.append(getName()+"."+body2Name+".torque.Y");
	labels.append(getName()+"."+body2Name+".torque.Z");

	return labels;
}
/**
 * Provide the value(s) to be reported that correspond to the labels
 */
OpenSim::Array<double> BushingForce::
getRecordValues(const SimTK::State& state) const 
{
	const string& body1Name = get_body_1();
	const string& body2Name = get_body_2();

	OpenSim::Array<double> values(1);

	const SimTK::Force::LinearBushing &simtkSpring = 
        (SimTK::Force::LinearBushing &)(_model->getForceSubsystem().getForce(_index));

	SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
	SimTK::Vector_<SimTK::Vec3> particleForces(0);
	SimTK::Vector mobilityForces(0);

	//get the net force added to the system contributed by the bushing
	simtkSpring.calcForceContribution(state, bodyForces, particleForces, mobilityForces);
	SimTK::Vec3 forces = bodyForces(_model->getBodySet().get(body1Name).getIndex())[1];
	SimTK::Vec3 torques = bodyForces(_model->getBodySet().get(body1Name).getIndex())[0];
	values.append(3, &forces[0]);
	values.append(3, &torques[0]);

	forces = bodyForces(_model->getBodySet().get(body2Name).getIndex())[1];
	torques = bodyForces(_model->getBodySet().get(body2Name).getIndex())[0];

	values.append(3, &forces[0]);
	values.append(3, &torques[0]);

	return values;
}
