/* -------------------------------------------------------------------------- *
 *                          OpenSim:  BallJoint.cpp                           *
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
#include "BallJoint.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>

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
BallJoint::~BallJoint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
BallJoint::BallJoint() : Joint()
{
	setAuthors("Ajay Seth");
	constructCoordinates();
}
//_____________________________________________________________________________
/**
 * Convenience Constructor.
 */
BallJoint::BallJoint(const std::string &name, OpenSim::Body& parent, 
					 Vec3 locationInParent, Vec3 orientationInParent,
					 OpenSim::Body& body, Vec3 locationInBody, Vec3 orientationInBody, 
					 bool reverse) :
			Joint(name, parent, locationInParent,orientationInParent,
					body, locationInBody, orientationInBody, reverse)
{
	setAuthors("Ajay Seth");
	constructCoordinates();
	updBody().setJoint(*this);
}


//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
void BallJoint::addToSystem(SimTK::MultibodySystem& system) const
{
	const SimTK::Vec3& orientation = get_orientation();
	const SimTK::Vec3& location = get_location();

	// CHILD TRANSFORM
	Rotation rotation(BodyRotationSequence, orientation[0],XAxis, orientation[1],YAxis, orientation[2],ZAxis);
	SimTK::Transform childTransform(rotation,location);

	const SimTK::Vec3& orientationInParent = get_orientation_in_parent();
	const SimTK::Vec3& locationInParent = get_location_in_parent();

	// PARENT TRANSFORM
	Rotation parentRotation(BodyRotationSequence, orientationInParent[0],XAxis, orientationInParent[1],YAxis, orientationInParent[2],ZAxis);
	SimTK::Transform parentTransform(parentRotation, locationInParent);

	// CREATE MOBILIZED BODY
	/*if(_useEulerAngles){
		MobilizedBody::Gimbal
			simtkBody(_model->updMatterSubsystem().updMobilizedBody(getMobilizedBodyIndex(_parentBody)),
				parentTransform,SimTK::Body::Rigid(_body->getMassProperties()),
				childTransform);
		setMobilizedBodyIndex(_body, simtkBody.getMobilizedBodyIndex());
	}
	else{*/
	BallJoint* mutableThis = const_cast<BallJoint*>(this);
	mutableThis->createMobilizedBody(parentTransform, childTransform);
	//}

    // TODO: Joints require super class to be called last.
    Super::addToSystem(system);
}

void BallJoint::initStateFromProperties(SimTK::State& s) const
{
    Super::initStateFromProperties(s);

    const MultibodySystem& system = _model->getMultibodySystem();
    const SimbodyMatterSubsystem& matter = system.getMatterSubsystem();
    if (matter.getUseEulerAngles(s))
        return;

	const CoordinateSet& coordinateSet = get_CoordinateSet();

	double xangle = coordinateSet[0].getDefaultValue();
    double yangle = coordinateSet[1].getDefaultValue();
    double zangle = coordinateSet[2].getDefaultValue();
    Rotation r(BodyRotationSequence, xangle, XAxis, yangle, YAxis, zangle, ZAxis);
	
	BallJoint* mutableThis = const_cast<BallJoint*>(this);
    matter.getMobilizedBody(MobilizedBodyIndex(mutableThis->updBody().getIndex())).setQToFitRotation(s, r);
}

void BallJoint::setPropertiesFromState(const SimTK::State& state)
{
    Super::setPropertiesFromState(state);

    // Override default behavior in case of quaternions.
    const MultibodySystem&        system = _model->getMultibodySystem();
    const SimbodyMatterSubsystem& matter = system.getMatterSubsystem();
    if (!matter.getUseEulerAngles(state)) {
        Rotation r = matter.getMobilizedBody(MobilizedBodyIndex(updBody().getIndex())).getBodyRotation(state);
        Vec3 angles = r.convertRotationToBodyFixedXYZ();
	
		const CoordinateSet& coordinateSet = get_CoordinateSet();

        coordinateSet[0].setDefaultValue(angles[0]);
        coordinateSet[1].setDefaultValue(angles[1]);
        coordinateSet[2].setDefaultValue(angles[2]);
    }
}

void BallJoint::createMobilizedBody(SimTK::Transform parentTransform, SimTK::Transform childTransform) {

	// CREATE MOBILIZED BODY
	MobilizedBody::Ball
		simtkBody(_model->updMatterSubsystem().updMobilizedBody(getMobilizedBodyIndex(&updParentBody())),
			parentTransform,SimTK::Body::Rigid(updBody().getMassProperties()),
			childTransform);

	setMobilizedBodyIndex(&updBody(), simtkBody.getMobilizedBodyIndex());
}