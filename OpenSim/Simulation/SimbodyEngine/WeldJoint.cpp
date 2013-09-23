/* -------------------------------------------------------------------------- *
 *                          OpenSim:  WeldJoint.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <math.h>
#include "WeldJoint.h"
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>

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
WeldJoint::~WeldJoint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
WeldJoint::WeldJoint() :
	Joint()
{
	setAuthors("Ajay Seth");
	constructCoordinates();
}

//_____________________________________________________________________________
/**
 * Convenience Constructor.
 */
	WeldJoint::WeldJoint(const std::string &name, OpenSim::Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
					OpenSim::Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody, bool reverse) :
	Joint(name, parent, locationInParent,orientationInParent,
			body, locationInBody, orientationInBody, reverse)
{
	setAuthors("Ajay Seth");
	constructCoordinates();
	updBody().setJoint(*this);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================


//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
void WeldJoint::addToSystem(SimTK::MultibodySystem& system) const
{

	const SimTK::Vec3& orientation = getProperty_orientation().getValue();
	const SimTK::Vec3& location = getProperty_location().getValue();

	// CHILD TRANSFORM
	Rotation rotation(BodyRotationSequence, orientation[0],XAxis, orientation[1],YAxis, orientation[2],ZAxis);
	SimTK::Transform childTransform(rotation, location);

	const SimTK::Vec3& locationInParent = getProperty_location_in_parent().getValue();
	const SimTK::Vec3& orientationInParent = getProperty_orientation_in_parent().getValue();
	
	// PARENT TRANSFORM
	Rotation parentRotation(BodyRotationSequence, orientationInParent[0],XAxis, orientationInParent[1],YAxis, orientationInParent[2],ZAxis);
	SimTK::Transform parentTransform(parentRotation, locationInParent);

	WeldJoint* mutableThis = const_cast<WeldJoint*>(this);
	mutableThis->createMobilizedBody(parentTransform, childTransform);
    // TODO: Joints require super class to be called last.
    Super::addToSystem(system);
}

void WeldJoint::createMobilizedBody(SimTK::Transform parentTransform, SimTK::Transform childTransform) {

	// CREATE MOBILIZED BODY
	MobilizedBody::Weld
		simtkBody(_model->updMatterSubsystem().updMobilizedBody(getMobilizedBodyIndex(&(updParentBody()))),
			parentTransform,SimTK::Body::Rigid(updBody().getMassProperties()),
			childTransform);
	
	setMobilizedBodyIndex(&(updBody()), simtkBody.getMobilizedBodyIndex());

}