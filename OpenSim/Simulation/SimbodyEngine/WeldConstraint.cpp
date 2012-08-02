/* -------------------------------------------------------------------------- *
 *                        OpenSim:  WeldConstraint.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Peter Loan, Ajay Seth                        *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <math.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>

#include "WeldConstraint.h"
#include "SimbodyEngine.h"

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
WeldConstraint::~WeldConstraint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
WeldConstraint::WeldConstraint() :
	Constraint(),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec()),
	_orientationInBody1(_orientationInBody1Prop.getValueDblVec()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec()),
	_orientationInBody2(_orientationInBody2Prop.getValueDblVec())
{
	setNull();
	setupProperties();
}

WeldConstraint::WeldConstraint(const std::string &name, OpenSim::Body& body1, SimTK::Vec3 locationInBody1, SimTK::Vec3 orientationInBody1,
							   OpenSim::Body& body2, SimTK::Vec3 locationInBody2, SimTK::Vec3 orientationInBody2) : Constraint(),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec()),
	_orientationInBody1(_orientationInBody1Prop.getValueDblVec()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec()),
	_orientationInBody2(_orientationInBody2Prop.getValueDblVec())
{
	setName(name);
	_body1Name = body1.getName();
	_body2Name = body2.getName();
	_locationInBody1 = locationInBody1;
	_orientationInBody1 = orientationInBody1;
	_locationInBody2 = locationInBody2;
	_orientationInBody2 = orientationInBody2;
}

WeldConstraint::WeldConstraint(const std::string &name, OpenSim::Body& body1, SimTK::Transform transformInBody1, 
							   OpenSim::Body& body2, SimTK::Transform transformInBody2) : Constraint(),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec()),
	_orientationInBody1(_orientationInBody1Prop.getValueDblVec()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec()),
	_orientationInBody2(_orientationInBody2Prop.getValueDblVec())
{
	setName(name);
	_body1Name = body1.getName();
	_body2Name = body2.getName();
	_locationInBody1 = transformInBody1.p();
	_orientationInBody1 = transformInBody1.R().convertRotationToBodyFixedXYZ();
	_locationInBody2 = transformInBody2.p();
	_orientationInBody2 = transformInBody2.R().convertRotationToBodyFixedXYZ();
}


//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aConstraint WeldConstraint to be copied.
 */
WeldConstraint::WeldConstraint(const WeldConstraint &aConstraint) :
   Constraint(aConstraint),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec()),
	_orientationInBody1(_orientationInBody1Prop.getValueDblVec()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec()),
	_orientationInBody2(_orientationInBody2Prop.getValueDblVec())
{
	setNull();
	setupProperties();
	copyData(aConstraint);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================

//_____________________________________________________________________________
/**
 * Copy data members from one WeldConstraint to another.
 *
 * @param aConstraint WeldConstraint to be copied.
 */
void WeldConstraint::copyData(const WeldConstraint &aConstraint)
{
	Constraint::copyData(aConstraint);
	_body1Name = aConstraint._body1Name;
	_body2Name = aConstraint._body2Name;
	_locationInBody1 = aConstraint._locationInBody1;
	_orientationInBody1 = aConstraint._orientationInBody1;
	_locationInBody2 = aConstraint._locationInBody2;
	_orientationInBody2 = aConstraint._orientationInBody2;
}

//_____________________________________________________________________________
/**
 * Set the data members of this WeldConstraint to their null values.
 */
void WeldConstraint::setNull()
{
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void WeldConstraint::setupProperties()
{
	// Body 1 name
	_body1NameProp.setName("body_1");
	_propertySet.append(&_body1NameProp);

	// Body 2 name
	_body2NameProp.setName("body_2");
	_propertySet.append(&_body2NameProp);

	//Default location and orientation (rotation sequence)
	SimTK::Vec3 origin(0.0, 0.0, 0.0);

	// Location in Body 1 
	_locationInBody1Prop.setName("location_body_1");
	_locationInBody1Prop.setValue(origin);
	_propertySet.append(&_locationInBody1Prop);

	// Orientation in Body 1 
	_orientationInBody1Prop.setName("orientation_body_1");
	_orientationInBody1Prop.setValue(origin);
	_propertySet.append(&_orientationInBody1Prop);

	// Location in Body 2 
	_locationInBody2Prop.setName("location_body_2");
	_locationInBody2Prop.setValue(origin);
	_propertySet.append(&_locationInBody2Prop);

	// Orientation in Body 2 
	_orientationInBody2Prop.setName("orientation_body_2");
	_orientationInBody2Prop.setValue(origin);
	_propertySet.append(&_orientationInBody2Prop);

}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this WeldConstraint.
 */
void WeldConstraint::connectToModel(Model& aModel)
{
	Super::connectToModel(aModel);
    
    string errorMessage;

	// Look up the two bodies being welded together by name in the
	// model and might as well keep a pointer to them
	if (!aModel.updBodySet().contains(_body1Name)) {
		errorMessage = "Invalid weld body1 (" + _body1Name + ") specified in Constraint " + getName();
		throw (Exception(errorMessage.c_str()));
	}
	if (!aModel.updBodySet().contains(_body2Name)) {
		errorMessage = "Invalid weld body2 (" + _body2Name + ") specified in Constraint " + getName();
		throw (Exception(errorMessage.c_str()));
	}
	_body1 = &aModel.updBodySet().get(_body1Name);
	_body2 = &aModel.updBodySet().get(_body2Name);
}

void WeldConstraint::addToSystem(SimTK::MultibodySystem& system) const
{
    Super::addToSystem(system);

	// Get underlying mobilized bodies
	SimTK::MobilizedBody b1 = _model->updMatterSubsystem().getMobilizedBody(_body1->getIndex());
	SimTK::MobilizedBody b2 = _model->updMatterSubsystem().getMobilizedBody(_body2->getIndex());
	// Build the transforms
	SimTK::Rotation r1; r1.setRotationToBodyFixedXYZ(_orientationInBody1);
	SimTK::Rotation r2; r2.setRotationToBodyFixedXYZ(_orientationInBody2);
	SimTK::Transform inb1(r1, _locationInBody1);
	SimTK::Transform inb2(r2, _locationInBody2);

    // Now create a Simbody Constraint::Weld
    SimTK::Constraint::Weld simtkWeld(b1, inb1, b2, inb2);
    
    // Beyond the const Component get the index so we can access the SimTK::Constraint later
	WeldConstraint* mutableThis = const_cast<WeldConstraint *>(this);
	mutableThis->_index = simtkWeld.getConstraintIndex();
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
WeldConstraint& WeldConstraint::operator=(const WeldConstraint &aConstraint)
{
	Constraint::operator=(aConstraint);
	copyData(aConstraint);
	return(*this);
}

//=============================================================================
// SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Following methods set attributes of the weld constraint */
void WeldConstraint::setBody1ByName(std::string aBodyName)
{
	_body1Name = aBodyName;
}

void WeldConstraint::setBody2ByName(std::string aBodyName)
{
	_body2Name = aBodyName;
}

/** Set the location and orientation (optional) for weld on body 1*/
void WeldConstraint::setBody1WeldLocation(Vec3 location, Vec3 orientation)
{
	_locationInBody1 = location;
	_orientationInBody1 = orientation;

	//if there is a live SimTK::system, we need to push this change down to the underlying constraint.
	if(int(_index) != SimTK::InvalidIndex){
		SimTK::Constraint::Weld &simConstraint = (SimTK::Constraint::Weld &)_model->updMatterSubsystem().updConstraint(_index);
		// Build the transforms
		SimTK::Rotation r1; r1.setRotationToBodyFixedXYZ(_orientationInBody1);
		SimTK::Transform inb1(r1, _locationInBody1);
		simConstraint.setDefaultFrameOnBody1(inb1);
	}
}

/** Set the location and orientation (optional) for weld on body 2*/
void WeldConstraint::setBody2WeldLocation(Vec3 location, Vec3 orientation)
{
	_locationInBody2 = location;
	_orientationInBody2 = orientation;

	//if there is a live SimTK::system, we need to push this change down to the underlying constraint.
	if(int(_index) != SimTK::InvalidIndex){
		SimTK::Constraint::Weld &simConstraint = (SimTK::Constraint::Weld &)_model->updMatterSubsystem().updConstraint(_index);
		// Build the transforms
		SimTK::Rotation r2; r2.setRotationToBodyFixedXYZ(_orientationInBody2);
		SimTK::Transform inb2(r2, _locationInBody2);
		simConstraint.setDefaultFrameOnBody2(inb2);
	}
}

void WeldConstraint::setContactPointForInducedAccelerations(const SimTK::State &s, Vec3 point)
{
	// The contact point coordinates in the surface body frame 
	Vec3 spoint;

	// make sure we are at the position stage
	_model->getMultibodySystem().realize(s, SimTK::Stage::Position);
	
	// For external forces we assume point position vector is defined wrt foot (i.e., _body2)
	// because we are passing it in from a prescribed force.
	// We must also get that point position vector wrt ground (i.e., _body1)
	_model->getSimbodyEngine().transformPosition(s, *_body2, point, *_body1, spoint);
	
	setBody1WeldLocation(spoint, _model->getSimbodyEngine().getTransform(s, *_body1).R().convertRotationToBodyFixedXYZ());
	setBody2WeldLocation(point, _model->getSimbodyEngine().getTransform(s, *_body2).R().convertRotationToBodyFixedXYZ());	
}
