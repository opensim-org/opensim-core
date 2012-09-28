/* -------------------------------------------------------------------------- *
 *                       OpenSim:  PointConstraint.cpp                        *
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
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>

#include "PointConstraint.h"
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
PointConstraint::~PointConstraint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
PointConstraint::PointConstraint() :
	Constraint(),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec())
{
	setNull();
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aConstraint PointConstraint to be copied.
 */
PointConstraint::PointConstraint(const PointConstraint &aConstraint) :
   Constraint(aConstraint),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec())
{
	setNull();
	setupProperties();
	copyData(aConstraint);
}

PointConstraint::PointConstraint(OpenSim::Body& body1, SimTK::Vec3& locationBody1, OpenSim::Body& body2, SimTK::Vec3& locationBody2) :
	Constraint(),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec())
{
	setNull();
	setupProperties();
	_body1Name = body1.getName();
	_locationInBody1 = locationBody1;
	_body2Name = body2.getName();
	_locationInBody2 = locationBody2;

}
//=============================================================================
// CONSTRUCTION
//=============================================================================

//_____________________________________________________________________________
/**
 * Copy data members from one PointConstraint to another.
 *
 * @param aConstraint PointConstraint to be copied.
 */
void PointConstraint::copyData(const PointConstraint &aConstraint)
{
	Constraint::copyData(aConstraint);
	_body1Name = aConstraint._body1Name;
	_body2Name = aConstraint._body2Name;
	_locationInBody1 = aConstraint._locationInBody1;
	_locationInBody2 = aConstraint._locationInBody2;

}

//_____________________________________________________________________________
/**
 * Set the data members of this PointConstraint to their null values.
 */
void PointConstraint::setNull()
{
	setAuthors("Ajay Seth");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PointConstraint::setupProperties()
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

	// Location in Body 2 
	_locationInBody2Prop.setName("location_body_2");
	_locationInBody2Prop.setValue(origin);
	_propertySet.append(&_locationInBody2Prop);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this PointConstraint.
 */
void PointConstraint::connectToModel(Model& aModel) {
	Super::connectToModel(aModel);
    
    string errorMessage;

	// Look up the two bodies being connected together by name in the
	// model and might as well keep a pointer to them
	if (!aModel.updBodySet().contains(_body1Name)) {
		errorMessage = "Invalid point constraint body1 (" + _body1Name + ") specified in Constraint " + getName();
		throw (Exception(errorMessage.c_str()));
	}
	if (!aModel.updBodySet().contains(_body2Name)) {
		errorMessage = "Invalid point constraint body2 (" + _body2Name + ") specified in Constraint " + getName();
		throw (Exception(errorMessage.c_str()));
	}
	_body1 = &aModel.updBodySet().get(_body1Name);
	_body2 = &aModel.updBodySet().get(_body2Name);
}

void PointConstraint::addToSystem(SimTK::MultibodySystem& system) const
{
    Super::addToSystem(system);

	// Get underlying mobilized bodies
	SimTK::MobilizedBody b1 = _model->updMatterSubsystem().getMobilizedBody((MobilizedBodyIndex)_body1->getIndex());
	SimTK::MobilizedBody b2 = _model->updMatterSubsystem().getMobilizedBody((MobilizedBodyIndex)_body2->getIndex());

    // Now create a Simbody Constraint::Point
    SimTK::Constraint::Ball simtkPoint(b1, _locationInBody1, b2, _locationInBody2);
    
    // Beyond the const Component get the index so we can access the SimTK::Constraint later
	PointConstraint* mutableThis = const_cast<PointConstraint *>(this);
	mutableThis->_index  = simtkPoint.getConstraintIndex();
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
PointConstraint& PointConstraint::operator=(const PointConstraint &aConstraint)
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
 * Following methods set attributes of the point constraint */
void PointConstraint::setBody1ByName(std::string aBodyName)
{
	_body1Name = aBodyName;
}

void PointConstraint::setBody2ByName(std::string aBodyName)
{
	_body2Name = aBodyName;
}

/** Set the location for point on body 1*/
void PointConstraint::setBody1PointLocation(Vec3 location)
{
	_locationInBody1 = location;
	//if there is a live SimTK::system, we need to push this change down to the underlying constraint.
	if(_index.isValid()){
		SimTK::Constraint::Ball &simConstraint = (SimTK::Constraint::Ball &)_model->updMatterSubsystem().updConstraint(_index);
		simConstraint.setDefaultPointOnBody1(_locationInBody1);
	}
}

/** Set the location for point on body 2*/
void PointConstraint::setBody2PointLocation(Vec3 location)
{
	_locationInBody2 = location;
	//if there is a live SimTK::system, we need to push this change down to the underlying constraint.
	if(_index.isValid()){
		SimTK::Constraint::Ball &simConstraint = (SimTK::Constraint::Ball &)_model->updMatterSubsystem().updConstraint(_index);
		simConstraint.setDefaultPointOnBody2(_locationInBody2);
	}
}

/** Set the point locations */
void PointConstraint::setContactPointForInducedAccelerations(const SimTK::State &s, Vec3 point)
{
	// The contact point coordinates in the surface body frame 
	Vec3 spoint;

	// make sure we are at the position stage
	_model->getMultibodySystem().realize(s, SimTK::Stage::Position);

	// For external forces we assume point position vector is defined wrt foot (i.e., _body2)
	// because we are passing it in from a prescribed force.
	// We must also get that point position vector wrt ground (i.e., _body1)
	
	cout<<"Body 1 is" << _body1->getName() <<endl;
	cout<<"Body 2 is" << _body2->getName() <<endl;
	_model->getSimbodyEngine().transformPosition(s, *_body2, point, *_body1, spoint);
	setBody1PointLocation(spoint);
	setBody2PointLocation(point);
}

