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
	Joint(),
	_ellipsoidRadii(_ellipsoidRadiiProp.getValueDblVec())
{
	setNull();
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Convenience Constructor.
 */
EllipsoidJoint::EllipsoidJoint(const std::string &name, OpenSim::Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
				OpenSim::Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody,
				SimTK::Vec3 ellipsoidRadii, bool reverse) :
	Joint(name, parent, locationInParent,orientationInParent,
			body, locationInBody, orientationInBody, reverse),
	_ellipsoidRadii(_ellipsoidRadiiProp.getValueDblVec())
{
	setNull();
	setupProperties();

	_ellipsoidRadii = ellipsoidRadii;
	_body->setJoint(*this);
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aJoint EllipsoidJoint to be copied.
 */
EllipsoidJoint::EllipsoidJoint(const EllipsoidJoint &aJoint) :
   Joint(aJoint),
	_ellipsoidRadii(_ellipsoidRadiiProp.getValueDblVec())
{
	setNull();
	setupProperties();
	copyData(aJoint);
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
Object* EllipsoidJoint::copy() const
{
	EllipsoidJoint *joint = new EllipsoidJoint(*this);
	return(joint);
}
//_____________________________________________________________________________
/**
 * Copy data members from one EllipsoidJoint to another.
 *
 * @param aJoint EllipsoidJoint to be copied.
 */
void EllipsoidJoint::copyData(const EllipsoidJoint &aJoint)
{
	Joint::copyData(aJoint);
	_ellipsoidRadii = aJoint._ellipsoidRadii;
}

//_____________________________________________________________________________
/**
 * Set the data members of this EllipsoidJoint to their null values.
 */
void EllipsoidJoint::setNull()
{
	setType("EllipsoidJoint");
	constructCoordinates();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void EllipsoidJoint::setupProperties()
{
	_ellipsoidRadiiProp.setName("radii_x_y_z");
	_propertySet.append(&_ellipsoidRadiiProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this EllipsoidJoint.
 */
void EllipsoidJoint::setup(Model& aModel)
{
	// Base class
	Joint::setup(aModel);
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
EllipsoidJoint& EllipsoidJoint::operator=(const EllipsoidJoint &aJoint)
{
	Joint::operator=(aJoint);
	copyData(aJoint);
	return(*this);
}


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
	_ellipsoidRadii = radii;

	// if the mobilized body index is valid, then attempt to change underlying MobilizedBody::Ellipsoid	
	if(MobilizedBodyIndex::isValid(getMobilizedBodyIndex(_body))){
		MobilizedBody::Ellipsoid &simtkBody = (MobilizedBody::Ellipsoid &)_model->updMatterSubsystem().updMobilizedBody(getMobilizedBodyIndex(_body));
		simtkBody.setDefaultRadii(_ellipsoidRadii);
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
	Joint::scale(aScaleSet);

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

	for(int i=0; i<3; i++){ 
		// Scale the size of the mobilizer
		_ellipsoidRadii[i] *= scaleFactors[i];
	}
}

//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
void EllipsoidJoint::createSystem(SimTK::MultibodySystem& system) const
{
	// CHILD TRANSFORM
	Rotation rotation(BodyRotationSequence, _orientation[0],XAxis, _orientation[1],YAxis, _orientation[2],ZAxis);
	SimTK::Transform childTransform(rotation,_location);

	// PARENT TRANSFORM
	Rotation parentRotation(BodyRotationSequence,_orientationInParent[0],XAxis,_orientationInParent[1],YAxis,_orientationInParent[2],ZAxis);
	SimTK::Transform parentTransform(parentRotation, _locationInParent);

	// CREATE MOBILIZED BODY
	MobilizedBody::Ellipsoid
		simtkBody(_model->updMatterSubsystem().updMobilizedBody(getMobilizedBodyIndex(_parentBody)),
			parentTransform,SimTK::Body::Rigid(_body->getMassProperties()),
			childTransform, _ellipsoidRadii);

	setMobilizedBodyIndex(_body, simtkBody.getMobilizedBodyIndex());

    // Let the superclass do its construction.
    Joint::createSystem(system);
}

void EllipsoidJoint::initState(SimTK::State& s) const
{
    Joint::initState(s);
    const MultibodySystem& system = _model->getMultibodySystem();
    const SimbodyMatterSubsystem& matter = system.getMatterSubsystem();
    if (matter.getUseEulerAngles(s))
        return;
    int zero = 0; // Workaround for really ridiculous Visual Studio 8 bug.
    double xangle = _coordinateSet.get(zero).getDefaultValue();
    double yangle = _coordinateSet.get(1).getDefaultValue();
    double zangle = _coordinateSet.get(2).getDefaultValue();
    Rotation r(BodyRotationSequence, xangle, XAxis, yangle, YAxis, zangle, ZAxis);
    matter.getMobilizedBody(MobilizedBodyIndex(_body->getIndex())).setQToFitRotation(s, r);
}

void EllipsoidJoint::setDefaultsFromState(const SimTK::State& state)
{
    const MultibodySystem& system = _model->getMultibodySystem();
    const SimbodyMatterSubsystem& matter = system.getMatterSubsystem();
    if (matter.getUseEulerAngles(state))
        Joint::setDefaultsFromState(state);
    else
    {
        Rotation r = matter.getMobilizedBody(MobilizedBodyIndex(_body->getIndex())).getBodyRotation(state);
        Vec3 angles = r.convertRotationToBodyFixedXYZ();
        int zero = 0; // Workaround for really ridiculous Visual Studio 8 bug.
        _coordinateSet.get(zero).setDefaultValue(angles[0]);
        _coordinateSet.get(1).setDefaultValue(angles[1]);
        _coordinateSet.get(2).setDefaultValue(angles[2]);
    }
}