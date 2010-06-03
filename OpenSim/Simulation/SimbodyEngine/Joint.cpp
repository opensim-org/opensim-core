// Joint.cpp
// Author: Frank C. Anderson, Peter Loan, Ajay Seth
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
#include "Joint.h"
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
//using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Joint::~Joint()
{
	if(_body != NULL)
		_body->_joint = NULL;
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Joint::Joint() :
	ModelComponent(),
	_parentName(_parentNameProp.getValueStr()),
	_locationInParent(_locationInParentProp.getValueDblVec3()),
	_orientationInParent(_orientationInParentProp.getValueDblVec3()),
	_location(_locationProp.getValueDblVec3()),
	_orientation(_orientationProp.getValueDblVec3()),
	_coordinateSetProp(PropertyObj("", CoordinateSet())),
	_coordinateSet((CoordinateSet&)_coordinateSetProp.getValueObj()),
	_reverse(_reverseProp.getValueBool())
{
	setNull();
	setupProperties();
}

/**
 * API constructor.
 */
Joint::Joint(const std::string &name, Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
			 Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody, bool reverse) :
	ModelComponent(),
	_parentName(_parentNameProp.getValueStr()),
	_locationInParent(_locationInParentProp.getValueDblVec3()),
	_orientationInParent(_orientationInParentProp.getValueDblVec3()),
	_location(_locationProp.getValueDblVec3()),
	_orientation(_orientationProp.getValueDblVec3()),
	_coordinateSetProp(PropertyObj("", CoordinateSet())),
	_coordinateSet((CoordinateSet&)_coordinateSetProp.getValueObj()),
	_reverse(_reverseProp.getValueBool())
{
	setNull();
	setupProperties();
	_parentName = parent.getName();
	_parentBody = &parent;
	_locationInParent = locationInParent;
	_orientationInParent = orientationInParent;
	_body = &body;
	_location = locationInBody;
	_orientation = orientationInBody;
	_reverse = reverse;
	updateName(name);
}


//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aJoint Joint to be copied.
 */
Joint::Joint(const Joint &aJoint) :
   ModelComponent(aJoint),
	_parentName(_parentNameProp.getValueStr()),
	_locationInParent(_locationInParentProp.getValueDblVec3()),
	_orientationInParent(_orientationInParentProp.getValueDblVec3()),
	_location(_locationProp.getValueDblVec3()),
	_orientation(_orientationProp.getValueDblVec3()),
	_coordinateSetProp(PropertyObj("", CoordinateSet())),
	_coordinateSet((CoordinateSet&)_coordinateSetProp.getValueObj()),
	_reverse(_reverseProp.getValueBool())
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
 * Copy data members from one Joint to another.
 *
 * @param aJoint Joint to be copied.
 */
void Joint::copyData(const Joint &aJoint)
{
	_parentName = aJoint._parentName;

	_locationInParent = aJoint._locationInParent;
	_location = aJoint._location;
	_orientationInParent = aJoint._orientationInParent;
	_orientation = aJoint._orientation;
	//_body = aJoint._body;
	//_parentBody = aJoint._parentBody;
	_coordinateSet = aJoint._coordinateSet;
	_reverse = aJoint._reverse;
	_model = NULL;
}

//_____________________________________________________________________________
/**
 * Set the data members of this Joint to their null values.
 */
void Joint::setNull()
{
	setType("Joint");
	_parentBody = NULL;
	_body = NULL;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Joint::setupProperties()
{
	// Parent body name
	_parentNameProp.setName("parent_body");
	_propertySet.append(&_parentNameProp);

	// Location in parent
	SimTK::Vec3 origin(0.0, 0.0, 0.0);
	_locationInParentProp.setName("location_in_parent");
	_locationInParentProp.setValue(origin);
	_propertySet.append(&_locationInParentProp);

	// Orientation in parent
	_orientationInParentProp.setName("orientation_in_parent");
	_orientationInParentProp.setValue(origin);
	_propertySet.append(&_orientationInParentProp);

	// Location in child
	_locationProp.setName("location");
	_locationProp.setValue(origin);
	_propertySet.append(&_locationProp);

	// Orientation in child
	_orientationProp.setName("orientation");
	_orientationProp.setValue(origin);
	_propertySet.append(&_orientationProp);

	// Generalized coordinates
	_coordinateSetProp.setComment("Generalized coordinates parameterizing this joint.");
	_propertySet.append(&_coordinateSetProp);

	_reverseProp.setName("reverse");
	_reverseProp.setValue(false);
	_propertySet.append(&_reverseProp);
}

void Joint::updateName(const std::string &aName)
{
	// Prefix coordinate names with joint name
	for(int i = 0; i< _coordinateSet.getSize() ; i++){
		std::string oldName = _coordinateSet.get(i).getName();
		_coordinateSet.get(i).setName(aName +"_"+oldName);
	}
	Object::setName(aName);
}
//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this Joint.
 */
void Joint::setup(Model& aModel)
{
	string errorMessage;

	ModelComponent::setup(aModel);

	// Look up the parent and child bodies by name in the
	// body set
    try {
	    _parentBody = &aModel.updBodySet().get(_parentName);
    }
    catch (...) {
		errorMessage += "Invalid parent body (" + _parentName + ") specified in joint " + getName();
		throw (Exception(errorMessage.c_str()));
	}

	_coordinateSet.setup(aModel);
	for(int i = 0; i< _coordinateSet.getSize(); i++)
		_coordinateSet[i].setJoint(*this);
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
Joint& Joint::operator=(const Joint &aJoint)
{
	Object::operator=(aJoint);
	copyData(aJoint);
	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// BODY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the body to which this joint belongs.
 *
 * @param aBody Body to which this joint should belong.
 */
void Joint::setBody(OpenSim::Body& aBody)
{
	_body = (Body*) &aBody;
}
//_____________________________________________________________________________
/**
 * Get the body to which this joint belongs.
 *
 * @return Body to which this joint belongs.
 */
OpenSim::Body& Joint::getBody() const
{
	return *_body;
}

//-----------------------------------------------------------------------------
// LOCATION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the location of this joint in its body.
 *
 * @param aLocation New location expressed in the local body frame.
 */
void Joint::
setLocation(const SimTK::Vec3& aLocation)
{
    if (_model != NULL)
        _model->invalidateSystem();

	// UPDATE PROPERTY
	_location = aLocation;
}
//_____________________________________________________________________________
/**
 * Get the location of this joint in its child body.
 *
 * @param rLocation Current location expressed in the local body frame.
 */
void Joint::
getLocation(SimTK::Vec3& rLocation) const
{
	rLocation = _location;
}

//-----------------------------------------------------------------------------
// ORIENTATION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the orientation of this joint in its body.
 *
 * @param aOrientation New orientation expressed in the local body frame in
 * body-fixed X-Y-Z Euler angles.
 */
void Joint::
setOrientation(const SimTK::Vec3& aOrientation)
{
    if (_model != NULL)
        _model->invalidateSystem();

	// UPDATE PROPERTY
	_orientation = aOrientation;
}
//_____________________________________________________________________________
/**
 * Get the orientation of this joint in its body.
 *
 * @param rOrientation Current orientation of the joint expressed in the local
 * body frame in body-fixed X-Y-Z Euler angles.
 */
void Joint::
getOrientation(SimTK::Vec3 &rOrientation) const
{
	rOrientation = _orientation;
}

//-----------------------------------------------------------------------------
// PARENT BODY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the name of the joint's parent body.
 *
 * @param Name of the parent body.
 */
void Joint::setParentName(const string& aName)
{
	_parentName = aName;
}
//_____________________________________________________________________________
/**
 * Get the name of the joint's parent body.
 *
 * @return Name of the parent body.
 */
string Joint::getParentName() const
{
	return _parentName;
}
//_____________________________________________________________________________
/**
 * Set the parent body to which this joint attaches.
 *
 * @param aParentBody Parent body to which this joint attaches.
 */
void Joint::
setParentBody(OpenSim::Body& aBody)
{
	_parentBody = (Body*) &aBody;
}
//_____________________________________________________________________________
/**
 * Get the parent body to which this joint attaches.
 *
 * @return Parent body to which this joint attaches.
 */
OpenSim::Body& Joint::
getParentBody() const
{
    if (_parentBody == NULL)
        throw Exception("Joint::getParentBody() : Joint has not been initialized");
	return *_parentBody;
}

//-----------------------------------------------------------------------------
// LOCATION IN PARENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the location of this joint in its parent body.
 *
 * @param aLocation New location expressed in the parent body frame.
 */
void Joint::
setLocationInParent(const SimTK::Vec3& aLocation)
{
    if (_model != NULL)
        _model->invalidateSystem();

	// Update property
	_locationInParent = aLocation;
}
//_____________________________________________________________________________
/**
 * Get the location of this joint in its parent body.
 *
 * @param rLocation Currnt location expressed in the parent body frame.
 */
void Joint::getLocationInParent(SimTK::Vec3& rLocation) const
{
	rLocation=_locationInParent;
}

//-----------------------------------------------------------------------------
// ORIENTATION IN PARENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the orientation of this joint in its parent body.
 *
 * @param aOrientation New orientation expressed in the parent body frame in
 * body-fixed X-Y-Z Euler angles.
 */
void Joint::
setOrientationInParent(const SimTK::Vec3& aOrientation)
{
    if (_model != NULL)
        _model->invalidateSystem();

	// Update property
	_orientationInParent = aOrientation;
}
//_____________________________________________________________________________
/**
 * Get the orientation of this joint in its parent body.
 *
 * @param rOrientation Current orientation expressed in the parent body frame
 * in body-fixed X-Y-Z Euler angles.
 */
void Joint::getOrientationInParent(SimTK::Vec3& rOrientation) const
{
	rOrientation = _orientationInParent;
}

//_____________________________________________________________________________
/**
 * Check if a coordinate is used by the Joint.
 *
 * @param aCoordinate Coordinate to look for in joint.
 * @return True if the coordinate is used.
 */
bool Joint::
isCoordinateUsed(Coordinate& aCoordinate) const
{
	int i, size = _coordinateSet.getSize();
	for(i=0; i<size; i++) {
		if(&_coordinateSet.get(i) == &aCoordinate) return true;
	}

	return false;
}

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________

void Joint::scale(const ScaleSet& aScaleSet)
{
	SimTK::Vec3 parentFactors(1.0);
	SimTK::Vec3 bodyFactors(1.0);

	// SCALING TO DO WITH THE PARENT BODY -----
	// Joint kinematics are scaled by the scale factors for the
	// parent body, so get those body's factors
	const string& parentName = getParentBody().getName();
	const string& bodyName = getBody().getName();
	// Get scale factors
	bool found_p = false;
	bool found_b = false; 
	for (int i=0; i<aScaleSet.getSize(); i++) {
		Scale& scale = aScaleSet.get(i);
		if (!found_p & scale.getSegmentName() == parentName) {
			scale.getScaleFactors(parentFactors);
			found_p = true;
		}
		if (!found_b & scale.getSegmentName() == bodyName) {
			scale.getScaleFactors(bodyFactors);
			found_b = true;
		}
		if(found_p & found_b)
			break;
	}

	for(int i=0; i<3; i++){
		_locationInParent[i]*= parentFactors[i];
		_location[i]*= bodyFactors[i];
	}
    if (_model != NULL)
        _model->invalidateSystem();
}

//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
/** Verify that parent for this Joint is valid */
void Joint::checkParentBody()
{
	if(!(_parentBody) || !SimTK::MobilizedBodyIndex::isValid(getMobilizedBodyIndex(_parentBody))){
		throw(Exception("Joint :: Attempted to connect to a parent body that was itself not connected."));
	}
}

/** Construct coordinates according to the mobilities of the Joint */
void Joint::constructCoordinates()
{
	// Check how many coordinates are already defined if any
	int ncoords = _coordinateSet.getSize();

	for(int i = ncoords; i< numCoordinates() ; i++){
		std::stringstream name;
		name << _name << "_coord_" << i;
		Coordinate *coord = new Coordinate();
		coord->setName(name.str());
		_coordinateSet.append(coord);
	}
}

void Joint::createSystem(SimTK::MultibodySystem& system) const
{
	// Each coordinate needs to know it's body index and mobility index.
	int nq = _coordinateSet.getSize();
	_coordinateSet.setMemoryOwner(false);

	for(int iq=0;iq<nq;iq++) {
		// Coordinate
		Coordinate &q = (Coordinate&)_coordinateSet.get(iq);
		q._bodyIndex = getMobilizedBodyIndex(_body);
		// The mobility index is the same as the order in which the coordinate appears in the coordinate set.
		// The functions (and their dependencies) determine how the coordinates gets used when constructing
		// the transform from parent to child.
		q._mobilityIndex = iq;
	}
}

void Joint::initState(SimTK::State& s) const
{
    for (int i = 0; i < _coordinateSet.getSize(); i++)
        _coordinateSet.get(i).initState(s);
}

void Joint::setDefaultsFromState(const SimTK::State& state)
{
    for (int i = 0; i < _coordinateSet.getSize(); i++)
        _coordinateSet.get(i).setDefaultsFromState(state);
}
