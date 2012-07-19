// EllipsoidJoint.cpp
// Author: Ajay Seth
/*
 * Copyright (c)  2008, Stanford University. All rights reserved. 
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
#include "EllipsoidJoint.h"
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
EllipsoidJoint::~EllipsoidJoint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
EllipsoidJoint::EllipsoidJoint() :
	Joint()
{
	constructCoordinates();
	constructProperties();
}
//_____________________________________________________________________________
/**
 * Convenience Constructor.
 */
EllipsoidJoint::EllipsoidJoint(const std::string &name, OpenSim::Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
				OpenSim::Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody,
				SimTK::Vec3 ellipsoidRadii, bool reverse) :
	Joint(name, parent, locationInParent,orientationInParent,
			body, locationInBody, orientationInBody, reverse)
{
	constructCoordinates();
	constructProperties();

	set_radii_x_y_z(ellipsoidRadii);
	updBody().setJoint(*this);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void EllipsoidJoint::constructProperties()
{
	SimTK::Vec3 origin(NaN);
	constructProperty_radii_x_y_z(origin);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this EllipsoidJoint.
 */
void EllipsoidJoint::connectToModel(Model& aModel)
{
	// Base class
	Super::connectToModel(aModel);
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________


//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the EllipsoidJoint's radii. If the the system is created, will attempt
 * to update the the default radii of the underlying MobilizedBody::Ellipsoid
 *
 * @param Vec3 of radii: X, Y, Z in the parent frame.
 */
void EllipsoidJoint::setEllipsoidRadii(Vec3 radii)
{
	set_radii_x_y_z(radii);

	// if the mobilized body index is valid, then attempt to change underlying MobilizedBody::Ellipsoid	
	if(MobilizedBodyIndex::isValid(getMobilizedBodyIndex(&updBody()))){
		MobilizedBody::Ellipsoid &simtkBody = (MobilizedBody::Ellipsoid &)_model->updMatterSubsystem().updMobilizedBody(getMobilizedBodyIndex(&updBody()));
		simtkBody.setDefaultRadii(get_radii_x_y_z());
	}
}

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
void EllipsoidJoint::scale(const ScaleSet& aScaleSet)
{
	Vec3 scaleFactors(1.0);

	// Joint knows how to scale locations of the joint in parent and on the body
	Super::scale(aScaleSet);

	// SCALING TO DO WITH THE PARENT BODY -----
	// Joint kinematics are scaled by the scale factors for the
	// parent body, so get those body's factors
	const string& parentName = getParentBody().getName();
	for (int i=0; i<aScaleSet.getSize(); i++) {
		Scale& scale = aScaleSet.get(i);
		if (scale.getSegmentName()==parentName) {
			scale.getScaleFactors(scaleFactors);
			break;
		}
	}

	SimTK::Vec3& ellipsoidRadii = upd_radii_x_y_z();
	for(int i=0; i<3; i++){ 
		// Scale the size of the mobilizer
		ellipsoidRadii[i] *= scaleFactors[i];
	}
}

//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
void EllipsoidJoint::addToSystem(SimTK::MultibodySystem& system) const
{
	const SimTK::Vec3& orientation = get_orientation();
	const SimTK::Vec3& location = get_location();

    // CHILD TRANSFORM
	Rotation rotation(BodyRotationSequence, orientation[0],XAxis, orientation[1],YAxis, orientation[2],ZAxis);
	SimTK::Transform childTransform(rotation, location);

	const SimTK::Vec3& orientationInParent = get_orientation_in_parent();
	const SimTK::Vec3& locationInParent = get_location_in_parent();

	// PARENT TRANSFORM
	Rotation parentRotation(BodyRotationSequence, orientationInParent[0],XAxis, orientationInParent[1],YAxis, orientationInParent[2],ZAxis);
	SimTK::Transform parentTransform(parentRotation, locationInParent);

	// CREATE MOBILIZED BODY
	EllipsoidJoint* mutableThis = const_cast<EllipsoidJoint*>(this);
	mutableThis->createMobilizedBody(parentTransform, childTransform);
    // TODO: Joints require super class to be called last.
    Super::addToSystem(system);
}

void EllipsoidJoint::initStateFromProperties(SimTK::State& s) const
{
    Super::initStateFromProperties(s);

    const MultibodySystem& system = _model->getMultibodySystem();
    const SimbodyMatterSubsystem& matter = system.getMatterSubsystem();
    if (matter.getUseEulerAngles(s))
        return;
    int zero = 0; // Workaround for really ridiculous Visual Studio 8 bug.

	const CoordinateSet& coordinateSet = get_CoordinateSet();

    double xangle = coordinateSet.get(zero).getDefaultValue();
    double yangle = coordinateSet.get(1).getDefaultValue();
    double zangle = coordinateSet.get(2).getDefaultValue();
    Rotation r(BodyRotationSequence, xangle, XAxis, yangle, YAxis, zangle, ZAxis);
	
	EllipsoidJoint* mutableThis = const_cast<EllipsoidJoint*>(this);
    matter.getMobilizedBody(MobilizedBodyIndex(mutableThis->updBody().getIndex())).setQToFitRotation(s, r);
}

void EllipsoidJoint::setPropertiesFromState(const SimTK::State& state)
{
	Super::setPropertiesFromState(state);

    // Override default in case of quaternions.
    const MultibodySystem& system = _model->getMultibodySystem();
    const SimbodyMatterSubsystem& matter = system.getMatterSubsystem();
    if (!matter.getUseEulerAngles(state)) {
        Rotation r = matter.getMobilizedBody(MobilizedBodyIndex(updBody().getIndex())).getBodyRotation(state);
        Vec3 angles = r.convertRotationToBodyFixedXYZ();
        int zero = 0; // Workaround for really ridiculous Visual Studio 8 bug.

		const CoordinateSet& coordinateSet = get_CoordinateSet();

        coordinateSet.get(zero).setDefaultValue(angles[0]);
        coordinateSet.get(1).setDefaultValue(angles[1]);
        coordinateSet.get(2).setDefaultValue(angles[2]);
    }
}

void EllipsoidJoint::createMobilizedBody(SimTK::Transform parentTransform, SimTK::Transform childTransform) {

	// CREATE MOBILIZED BODY
	MobilizedBody::Ellipsoid
		simtkBody(_model->updMatterSubsystem().updMobilizedBody(getMobilizedBodyIndex(&updParentBody())),
			parentTransform,SimTK::Body::Rigid(updBody().getMassProperties()),
			childTransform, get_radii_x_y_z());

	setMobilizedBodyIndex(&updBody(), simtkBody.getMobilizedBodyIndex());

}