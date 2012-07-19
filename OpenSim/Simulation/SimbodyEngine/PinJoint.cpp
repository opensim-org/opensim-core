// PinJoint.cpp
// Author: Ajay Seth
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
#include <math.h>
#include "PinJoint.h"
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
PinJoint::~PinJoint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
PinJoint::PinJoint() :
	Joint()
{
	constructCoordinates();

	const CoordinateSet& coordinateSet = get_CoordinateSet();
	coordinateSet[0].setMotionType(Coordinate::Rotational);
}

//_____________________________________________________________________________
/**
 * Convenience Constructor.
 */
	PinJoint::PinJoint(const std::string &name, OpenSim::Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
					OpenSim::Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody, bool reverse) :
	Joint(name, parent, locationInParent,orientationInParent,
			body, locationInBody, orientationInBody, reverse)
{
	constructCoordinates();

	const CoordinateSet& coordinateSet = get_CoordinateSet();
	coordinateSet[0].setMotionType(Coordinate::Rotational);

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
 * @param aModel OpenSim  model containing this PinJoint.
 */
void PinJoint::connectToModel(Model& aModel)
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
void PinJoint::scale(const ScaleSet& aScaleSet)
{
	// Joint knows how to scale locations of the joint in parent and on the body
	Super::scale(aScaleSet);
}

//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
void PinJoint::addToSystem(SimTK::MultibodySystem& system) const
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

	PinJoint* mutableThis = const_cast<PinJoint*>(this);
	mutableThis->createMobilizedBody(parentTransform, childTransform);

    // TODO: Joints require super class to be called last.
    Super::addToSystem(system);
}

void PinJoint::createMobilizedBody(SimTK::Transform parentTransform, SimTK::Transform childTransform) {

	// CREATE MOBILIZED BODY
	MobilizedBody::Pin
		simtkBody(_model->updMatterSubsystem().updMobilizedBody(getMobilizedBodyIndex(&updParentBody())),
			parentTransform,SimTK::Body::Rigid(updBody().getMassProperties()),
			childTransform);

	setMobilizedBodyIndex(&updBody(), simtkBody.getMobilizedBodyIndex());

}