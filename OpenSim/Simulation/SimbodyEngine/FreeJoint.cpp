// FreeJoint.cpp
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
#include "FreeJoint.h"
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
FreeJoint::~FreeJoint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
FreeJoint::FreeJoint() :
	Joint(),
	_useEulerAngles(_useEulerAnglesProp.getValueBool())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Convenience Constructor.
 */
	FreeJoint::FreeJoint(const std::string &name, OpenSim::Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
					OpenSim::Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody,
					bool useEulerAngles, bool reverse) :
	Joint(name, parent, locationInParent,orientationInParent,
			body, locationInBody, orientationInBody, reverse),
	_useEulerAngles(_useEulerAnglesProp.getValueBool())
{
	setNull();
	setupProperties();
	_useEulerAngles = useEulerAngles;
	_body->setJoint(*this);
	setName(name);
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aJoint FreeJoint to be copied.
 */
FreeJoint::FreeJoint(const FreeJoint &aJoint) :
   Joint(aJoint),
	_useEulerAngles(_useEulerAnglesProp.getValueBool())
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
Object* FreeJoint::copy() const
{
	FreeJoint *joint = new FreeJoint(*this);
	return(joint);
}
//_____________________________________________________________________________
/**
 * Copy data members from one FreeJoint to another.
 *
 * @param aJoint FreeJoint to be copied.
 */
void FreeJoint::copyData(const FreeJoint &aJoint)
{
	Joint::copyData(aJoint);
	_useEulerAngles = aJoint._useEulerAngles;
}

//_____________________________________________________________________________
/**
 * Set the data members of this FreeJoint to their null values.
 */
void FreeJoint::setNull()
{
	setType("FreeJoint");
	constructCoordinates();
	// We know we have three rotations followed by three translations
	// Replace default names _coord_? with more meaningful names
	string dirStrings[] = {"x", "y", "z"};
	for (int i=0; i< 3; i++){
		string oldName = _coordinateSet.get(i).getName();
		int pos=oldName.find("_coord_"); 
		if (pos != string::npos){
			oldName.replace(pos, 8, ""); 
			_coordinateSet.get(i).setName(oldName+"_"+dirStrings[i]+"Rotation");
			_coordinateSet.get(i).setMotionType(Coordinate::Rotational);
		}
	}
	for (int i=3; i< 6; i++){
		string oldName = _coordinateSet.get(i).getName();
		int pos=oldName.find("_coord_"); 
		if (pos != string::npos){
			oldName.replace(pos, 8, ""); 
			_coordinateSet.get(i).setName(oldName+"_"+dirStrings[i-3]+"Translation");
			_coordinateSet.get(i).setMotionType(Coordinate::Translational);
		}
	}

}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void FreeJoint::setupProperties()
{
	_useEulerAnglesProp.setName("use_euler_angles");
	_useEulerAnglesProp.setComment("Set flag to true to use Euler angles to parameterize rotations.");
	_useEulerAnglesProp.setValue(true);
	_propertySet.append(&_useEulerAnglesProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this FreeJoint.
 */
void FreeJoint::setup(Model& aModel)
{
	string errorMessage;

	// Base class
	Joint::setup(aModel);

	// Look up the parent and child bodies by name in the
	// dynamics engine and store pointers to them.
    try {
    	_parentBody = &aModel.updBodySet().get(_parentName);
    }
	catch (...) {
		errorMessage += "Invalid parent body (" + _parentName + ") specified in joint " + getName();
		throw (Exception(errorMessage.c_str()));
	}
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
FreeJoint& FreeJoint::operator=(const FreeJoint &aJoint)
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
void FreeJoint::scale(const ScaleSet& aScaleSet)
{
}

//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
void FreeJoint::createSystem(SimTK::MultibodySystem& system) const
{
	// CHILD TRANSFORM
	Rotation rotation(BodyRotationSequence, _orientation[0],XAxis, _orientation[1],YAxis, _orientation[2],ZAxis);
	SimTK::Transform childTransform(rotation,_location);

	// PARENT TRANSFORM
	Rotation parentRotation(BodyRotationSequence,_orientationInParent[0],XAxis,_orientationInParent[1],YAxis,_orientationInParent[2],ZAxis);
	SimTK::Transform parentTransform(parentRotation, _locationInParent);

	SimTK::Transform noTransform(Rotation(), Vec3(0));

	// CREATE MOBILIZED BODY
	if(_useEulerAngles){
		MobilizedBody::Gimbal
			simtkMasslessBody(_model->updMatterSubsystem().updMobilizedBody(getMobilizedBodyIndex(_parentBody)),
			parentTransform, SimTK::Body::Massless(), noTransform);
		MobilizedBody::Translation
				simtkBody(simtkMasslessBody, noTransform, SimTK::Body::Rigid(_body->getMassProperties()), childTransform);

		const MobilizedBodyIndex _masslessBodyIndex = simtkMasslessBody.getMobilizedBodyIndex();
		setMobilizedBodyIndex(_body, simtkBody.getMobilizedBodyIndex());

		// SETUP COORDINATES
		// Each coordinate needs to know it's body index and mobility index.
		for(int i =0; i < _numMobilities; i++){
			Coordinate &coord = _coordinateSet.get(i);
			coord.setJoint(*this);
			setCoordinateModel(&coord, _model);
			setCoordinateMobilizedBodyIndex(&coord, ((i < 3) ? (_masslessBodyIndex) : (getMobilizedBodyIndex(_body))));
			// The mobility index is the same as the order in which the coordinate appears in the coordinate set.
			setCoordinateMobilityIndex(&coord, (i < 3 ? i : i-3));
		}
	}
	else {
		MobilizedBody::Free
			simtkBody(_model->updMatterSubsystem().updMobilizedBody(getMobilizedBodyIndex(_parentBody)),
			parentTransform, SimTK::Body::Rigid(_body->getMassProperties()), childTransform);

		setMobilizedBodyIndex(_body, simtkBody.getMobilizedBodyIndex());

		// Let the superclass do its creation of coordinates.
		Joint::createSystem(system);
	}
}

void FreeJoint::initState(SimTK::State& s) const
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
	Vec3 t( _coordinateSet.get(3).getDefaultValue(), 
			_coordinateSet.get(4).getDefaultValue(), 
			_coordinateSet.get(5).getDefaultValue());
    matter.getMobilizedBody(MobilizedBodyIndex(_body->getIndex())).setQToFitTransform(s, Transform(r, t));
}

void FreeJoint::setDefaultsFromState(const SimTK::State& state)
{
    const MultibodySystem& system = _model->getMultibodySystem();
    const SimbodyMatterSubsystem& matter = system.getMatterSubsystem();
    if (matter.getUseEulerAngles(state))
        Joint::setDefaultsFromState(state);
    else
    {
        Rotation r = matter.getMobilizedBody(MobilizedBodyIndex(_body->getIndex())).getMobilizerTransform(state).R();
		Vec3 t = matter.getMobilizedBody(MobilizedBodyIndex(_body->getIndex())).getMobilizerTransform(state).p();
        Vec3 angles = r.convertRotationToBodyFixedXYZ();
        int zero = 0; // Workaround for really ridiculous Visual Studio 8 bug.
        _coordinateSet.get(zero).setDefaultValue(angles[0]);
        _coordinateSet.get(1).setDefaultValue(angles[1]);
        _coordinateSet.get(2).setDefaultValue(angles[2]);
		_coordinateSet.get(3).setDefaultValue(t[0]); 
		_coordinateSet.get(4).setDefaultValue(t[1]); 
		_coordinateSet.get(5).setDefaultValue(t[2]);
    }
}