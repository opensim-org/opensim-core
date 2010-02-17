// SliderJoint.cpp
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
#include "SliderJoint.h"
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
SliderJoint::~SliderJoint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SliderJoint::SliderJoint() :
	Joint()
{
	setNull();
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aJoint SliderJoint to be copied.
 */
SliderJoint::SliderJoint(const SliderJoint &aJoint) :
   Joint(aJoint)
{
	setNull();
	setupProperties();
	copyData(aJoint);
}

//_____________________________________________________________________________
/**
 * Convenience Constructor.
 */
	SliderJoint::SliderJoint(const std::string &name, OpenSim::Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
					OpenSim::Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody, bool reverse) :
	Joint(name, parent, locationInParent,orientationInParent,
			body, locationInBody, orientationInBody, reverse)
{
	setNull();
	setupProperties();
	_body->setJoint(*this);
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
Object* SliderJoint::copy() const
{
	SliderJoint *joint = new SliderJoint(*this);
	return(joint);
}
//_____________________________________________________________________________
/**
 * Copy data members from one SliderJoint to another.
 *
 * @param aJoint SliderJoint to be copied.
 */
void SliderJoint::copyData(const SliderJoint &aJoint)
{
	Joint::copyData(aJoint);
}

//_____________________________________________________________________________
/**
 * Set the data members of this SliderJoint to their null values.
 */
void SliderJoint::setNull()
{
	setType("SliderJoint");
	constructCoordinates();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SliderJoint::setupProperties()
{
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim  model containing this SliderJoint.
 */
void SliderJoint::setup(Model& aModel)
{
	string errorMessage;

	// Base class
	Joint::setup(aModel);

	// Look up the parent and child bodies by name in the
	if (!aModel.updBodySet().contains(_parentName)) {
		errorMessage += "Invalid parent body (" + _parentName + ") specified in joint " + getName();
		throw (Exception(errorMessage.c_str()));
	}
	_parentBody = &aModel.updBodySet().get(_parentName);
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
SliderJoint& SliderJoint::operator=(const SliderJoint &aJoint)
{
	Joint::operator=(aJoint);
	copyData(aJoint);
	return(*this);
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
void SliderJoint::scale(const ScaleSet& aScaleSet)
{
	// Joint knows how to scale locations of the joint in parent and on the body
	Joint::scale(aScaleSet);
}

//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
void SliderJoint::createSystem(SimTK::MultibodySystem& system) const
{
	// CHILD TRANSFORM
	Rotation rotation(BodyRotationSequence, _orientation[0],XAxis, _orientation[1],YAxis, _orientation[2],ZAxis);
	SimTK::Transform childTransform(rotation,_location);

	// PARENT TRANSFORM
	Rotation parentRotation(BodyRotationSequence,_orientationInParent[0],XAxis,_orientationInParent[1],YAxis,_orientationInParent[2],ZAxis);
	SimTK::Transform parentTransform(parentRotation, _locationInParent);

	// CREATE MOBILIZED BODY
	MobilizedBody::Slider
		simtkBody(_model->updMatterSubsystem().updMobilizedBody(getMobilizedBodyIndex(_parentBody)),
			parentTransform,SimTK::Body::Rigid(_body->getMassProperties()),
			childTransform);

	setMobilizedBodyIndex(_body, simtkBody.getMobilizedBodyIndex());

    // Let the superclass do its construction.
    Joint::createSystem(system);
}

