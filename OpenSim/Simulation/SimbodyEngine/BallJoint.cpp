// BallJoint.cpp
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
#include "BallJoint.h"
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
BallJoint::BallJoint() :
	Joint()
	//_useEulerAngles(_useEulerAnglesProp.getValueBool())
{
	setNull();
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Convenience Constructor.
 */
BallJoint::BallJoint(const std::string &name, OpenSim::Body& parent, Vec3 locationInParent, Vec3 orientationInParent,
					 OpenSim::Body& body, Vec3 locationInBody, Vec3 orientationInBody, /*bool useEulerAngles,*/ bool reverse) :
	Joint(name, parent, locationInParent,orientationInParent,
			body, locationInBody, orientationInBody, reverse)
	//_useEulerAngles(_useEulerAnglesProp.getValueBool())
{
	setNull();
	setupProperties();
	//_useEulerAngles = useEulerAngles;
	_body->setJoint(*this);
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aJoint BallJoint to be copied.
 */
BallJoint::BallJoint(const BallJoint &aJoint) :
   Joint(aJoint)
   //_useEulerAngles(_useEulerAnglesProp.getValueBool())
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
 * Copy data members from one BallJoint to another.
 *
 * @param aJoint BallJoint to be copied.
 */
void BallJoint::copyData(const BallJoint &aJoint)
{
	Joint::copyData(aJoint);
	//_useEulerAngles = aJoint._useEulerAngles;
}

//_____________________________________________________________________________
/**
 * Set the data members of this BallJoint to their null values.
 */
void BallJoint::setNull()
{
	constructCoordinates();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void BallJoint::setupProperties()
{
	//_useEulerAnglesProp.setName("use_euler_angles");
	//_useEulerAnglesProp.setComment("Set flag to true to use Euler angles to parameterize rotations.");
	//_useEulerAnglesProp.setValue(true);
	//_propertySet.append(&_useEulerAnglesProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this BallJoint.
 */
void BallJoint::connectToModel(Model& aModel)
{
	// Base class
	Super::connectToModel(aModel);
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
BallJoint& BallJoint::operator=(const BallJoint &aJoint)
{
	Joint::operator=(aJoint);
	copyData(aJoint);
	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________

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
void BallJoint::scale(const ScaleSet& aScaleSet)
{
}

//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
void BallJoint::addToSystem(SimTK::MultibodySystem& system) const
{
	// CHILD TRANSFORM
	Rotation rotation(BodyRotationSequence, _orientation[0],XAxis, _orientation[1],YAxis, _orientation[2],ZAxis);
	SimTK::Transform childTransform(rotation,_location);

	// PARENT TRANSFORM
	Rotation parentRotation(BodyRotationSequence,_orientationInParent[0],XAxis,_orientationInParent[1],YAxis,_orientationInParent[2],ZAxis);
	SimTK::Transform parentTransform(parentRotation, _locationInParent);

	// CREATE MOBILIZED BODY
	/*if(_useEulerAngles){
		MobilizedBody::Gimbal
			simtkBody(_model->updMatterSubsystem().updMobilizedBody(getMobilizedBodyIndex(_parentBody)),
				parentTransform,SimTK::Body::Rigid(_body->getMassProperties()),
				childTransform);
		setMobilizedBodyIndex(_body, simtkBody.getMobilizedBodyIndex());
	}
	else{*/
		MobilizedBody::Ball
			simtkBody(_model->updMatterSubsystem().updMobilizedBody(getMobilizedBodyIndex(_parentBody)),
				parentTransform,SimTK::Body::Rigid(_body->getMassProperties()),
				childTransform);
		setMobilizedBodyIndex(_body, simtkBody.getMobilizedBodyIndex());
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
    int zero = 0; // Workaround for really ridiculous Visual Studio 8 bug.
    double xangle = _coordinateSet.get(zero).getDefaultValue();
    double yangle = _coordinateSet.get(1).getDefaultValue();
    double zangle = _coordinateSet.get(2).getDefaultValue();
    Rotation r(BodyRotationSequence, xangle, XAxis, yangle, YAxis, zangle, ZAxis);
    matter.getMobilizedBody(MobilizedBodyIndex(_body->getIndex())).setQToFitRotation(s, r);
}

void BallJoint::setPropertiesFromState(const SimTK::State& state)
{
    Super::setPropertiesFromState(state);

    // Override default behavior in case of quaternions.
    const MultibodySystem&        system = _model->getMultibodySystem();
    const SimbodyMatterSubsystem& matter = system.getMatterSubsystem();
    if (!matter.getUseEulerAngles(state)) {
        Rotation r = matter.getMobilizedBody(MobilizedBodyIndex(_body->getIndex())).getBodyRotation(state);
        Vec3 angles = r.convertRotationToBodyFixedXYZ();
        int zero = 0; // Workaround for really ridiculous Visual Studio 8 bug.
        _coordinateSet.get(zero).setDefaultValue(angles[0]);
        _coordinateSet.get(1).setDefaultValue(angles[1]);
        _coordinateSet.get(2).setDefaultValue(angles[2]);
    }
}

