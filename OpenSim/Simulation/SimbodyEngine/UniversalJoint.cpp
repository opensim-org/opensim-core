/* -------------------------------------------------------------------------- *
 *                     OpenSim:  UniversalJoint.cpp                           *
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
#include "UniversalJoint.h"
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
UniversalJoint::~UniversalJoint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
UniversalJoint::UniversalJoint() :
    Joint()
{
    constructCoordinates();

    const CoordinateSet& coordinateSet = get_CoordinateSet();
    coordinateSet[0].setMotionType(Coordinate::Rotational);
    coordinateSet[1].setMotionType(Coordinate::Rotational);
}

//_____________________________________________________________________________
/**
 * Convenience Constructor.
 */
    UniversalJoint::UniversalJoint(const std::string &name, OpenSim::Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
                    OpenSim::Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody, bool reverse) :
    Joint(name, parent, locationInParent,orientationInParent,
            body, locationInBody, orientationInBody, reverse)
{
    constructCoordinates();

    const CoordinateSet& coordinateSet = get_CoordinateSet();
    coordinateSet[0].setMotionType(Coordinate::Rotational);
    coordinateSet[1].setMotionType(Coordinate::Rotational);

    updBody().setJoint(*this);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim  model containing this UniversalJoint.
 */
void UniversalJoint::connectToModel(Model& aModel)
{
    string errorMessage;

    // Base class
    Super::connectToModel(aModel);

    const std::string& parentName = get_parent_body();

    // Look up the parent and child bodies by name in the
    if (!aModel.updBodySet().contains(parentName)) {
        errorMessage += "Invalid parent body (" + parentName + ") specified in joint " + getName();
        throw (Exception(errorMessage.c_str()));
    }
    setParentBody(aModel.updBodySet().get(parentName));
}

//=============================================================================
// GET AND SET
//=============================================================================
//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Scale a joint based on XYZ scale factors for the bodies.
 *
 * @param aScaleSet Set of XYZ scale factors for the bodies.
 * @todo Need to scale transforms appropriately, given an arbitrary axis.
 */
void UniversalJoint::scale(const ScaleSet& aScaleSet)
{
    // Joint knows how to scale locations of the joint in parent and on the body
    Super::scale(aScaleSet);
}

//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
void UniversalJoint::addToSystem(SimTK::MultibodySystem& system) const
{
    const SimTK::Vec3& orientation = getProperty_orientation().getValue();
    const SimTK::Vec3& location = getProperty_location().getValue();

    // CHILD TRANSFORM
    Rotation rotation(BodyRotationSequence, orientation[0],XAxis, orientation[1],YAxis, orientation[2],ZAxis);
    SimTK::Transform childTransform(rotation, location);

    const SimTK::Vec3& orientationInParent = getProperty_orientation_in_parent().getValue();
    const SimTK::Vec3& locationInParent = getProperty_location_in_parent().getValue();

    // PARENT TRANSFORM
    Rotation parentRotation(BodyRotationSequence, orientationInParent[0],XAxis, orientationInParent[1],YAxis, orientationInParent[2],ZAxis);
    SimTK::Transform parentTransform(parentRotation, locationInParent);

    UniversalJoint* mutableThis = const_cast<UniversalJoint*>(this);
    mutableThis->createMobilizedBody(parentTransform, childTransform);

    // TODO: Joints require super class to be called last.
    Super::addToSystem(system);
}

void UniversalJoint::createMobilizedBody(SimTK::Transform parentTransform, SimTK::Transform childTransform) {

    // CREATE MOBILIZED BODY
    MobilizedBody::Universal
        simtkBody(_model->updMatterSubsystem().updMobilizedBody(getMobilizedBodyIndex(&updParentBody())),
            parentTransform,SimTK::Body::Rigid(updBody().getMassProperties()),
            childTransform);

    setMobilizedBodyIndex(&updBody(), simtkBody.getMobilizedBodyIndex());

}