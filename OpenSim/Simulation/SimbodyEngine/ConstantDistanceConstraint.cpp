/* -------------------------------------------------------------------------- *
 *                  OpenSim:  ConstantDistanceConstraint.cpp                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Matt S. DeMers                                                  *
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

#include "ConstantDistanceConstraint.h"
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
ConstantDistanceConstraint::~ConstantDistanceConstraint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ConstantDistanceConstraint::ConstantDistanceConstraint() :
	Constraint(),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec()),
	_constantDistance(_constantDistanceProp.getValueDbl())
{
	setNull();
	setupProperties();
}


//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aConstraint ConstantDistanceConstraint to be copied.
 */
ConstantDistanceConstraint::ConstantDistanceConstraint(const ConstantDistanceConstraint &aConstraint) :
   Constraint(aConstraint),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec()),
	_constantDistance(_constantDistanceProp.getValueDbl())
{
	setNull();
	setupProperties();
	copyData(aConstraint);
}


//_____________________________________________________________________________
/**
 * Convenience Constructor.
 *
 * @param body1: First body connected with the ConstantDistanceConstraint
 * @param locationBody1: point fixed on body1 where the contraint is applied
 * @param body2: Second body connected with the ConstantDistanceConstraint
 * @param locationBody2: point fixed on body2 where the constraint is applied
 * @param distance: fixed distance that the constraint will maintain between locationBody1 and locationBody2
 */
ConstantDistanceConstraint::ConstantDistanceConstraint(OpenSim::Body& body1, SimTK::Vec3& locationBody1, OpenSim::Body& body2, SimTK::Vec3& locationBody2, double& distance) :
	Constraint(),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec()),
	_constantDistance(_constantDistanceProp.getValueDbl())
{

	setNull();
	setupProperties();
	_body1Name = body1.getName();
	_locationInBody1 = locationBody1;
	_body2Name = body2.getName();
	_locationInBody2 = locationBody2;
	_constantDistance = distance;

}
//=============================================================================
// CONSTRUCTION
//=============================================================================

//_____________________________________________________________________________
/**
 * Copy data members from one ConstantDistanceConstraint to another.
 *
 * @param aConstraint ConstantDistanceConstraint to be copied.
 */
void ConstantDistanceConstraint::copyData(const ConstantDistanceConstraint &aConstraint)
{
	Constraint::copyData(aConstraint);
	_body1Name = aConstraint._body1Name;
	_body2Name = aConstraint._body2Name;
	_locationInBody1 = aConstraint._locationInBody1;
	_locationInBody2 = aConstraint._locationInBody2;
	_constantDistance = aConstraint._constantDistance;

}

//_____________________________________________________________________________
/**
 * Set the data members of this ConstantDistanceConstraint to their null values.
 */
void ConstantDistanceConstraint::setNull()
{
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ConstantDistanceConstraint::setupProperties()
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

	// Constant distance between points
	_constantDistanceProp.setName("constant_distance");
	_constantDistanceProp.setValue(1.0);
	_propertySet.append(&_constantDistanceProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this ConstantDistanceConstraint.
 */
void ConstantDistanceConstraint::connectToModel(Model& aModel)
{
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

void ConstantDistanceConstraint::addToSystem(SimTK::MultibodySystem& system) const
{
    Super::addToSystem(system);

	// Get underlying mobilized bodies
	SimTK::MobilizedBody b1 = _model->updMatterSubsystem().getMobilizedBody((MobilizedBodyIndex)_body1->getIndex());
	SimTK::MobilizedBody b2 = _model->updMatterSubsystem().getMobilizedBody((MobilizedBodyIndex)_body2->getIndex());

    // Now create a Simbody Constraint::Point
    //SimTK::Constraint::Ball simtkPoint(b1, _locationInBody1, b2, _locationInBody2);

	// Now create a Simbody Constraint::Rod
	SimTK::Constraint::Rod simtkRod(b1, _locationInBody1, b2, _locationInBody2, _constantDistance);
    
    // Beyond the const Component get the index so we can access the SimTK::Constraint later
	ConstantDistanceConstraint* mutableThis = const_cast<ConstantDistanceConstraint *>(this);
	mutableThis->_index  = simtkRod.getConstraintIndex();
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
ConstantDistanceConstraint& ConstantDistanceConstraint::operator=(const ConstantDistanceConstraint &aConstraint)
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
void ConstantDistanceConstraint::setBody1ByName(std::string aBodyName)
{
	_body1Name = aBodyName;
}

void ConstantDistanceConstraint::setBody2ByName(std::string aBodyName)
{
	_body2Name = aBodyName;
}

/** Set the location for point on body 1*/
void ConstantDistanceConstraint::setBody1PointLocation(Vec3 location)
{
	_locationInBody1 = location;
	//if there is a live SimTK::system, we need to push this change down to the underlying constraint.
	if(int(_index) != SimTK::InvalidIndex){
		SimTK::Constraint::Rod &simConstraint = (SimTK::Constraint::Rod &)_model->updMatterSubsystem().updConstraint(_index);
		simConstraint.setDefaultPointOnBody1(_locationInBody1);
	}
}

/** Set the location for point on body 2*/
void ConstantDistanceConstraint::setBody2PointLocation(Vec3 location)
{
	_locationInBody2 = location;
	//if there is a live SimTK::system, we need to push this change down to the underlying constraint.
	if(int(_index) != SimTK::InvalidIndex){
		SimTK::Constraint::Rod &simConstraint = (SimTK::Constraint::Rod &)_model->updMatterSubsystem().updConstraint(_index);
		simConstraint.setDefaultPointOnBody2(_locationInBody2);
	}
}

/** Set the constant distance between the two points*/
void ConstantDistanceConstraint::setConstantDistance(double distance)
{

	_constantDistance = distance;
	//if there is a live SimTK::system, we need to push this change down to the underlying constraint.
	if(int(_index) != SimTK::InvalidIndex){
		SimTK::Constraint::Rod &simConstraint = (SimTK::Constraint::Rod &)_model->updMatterSubsystem().updConstraint(_index);
		simConstraint.setDefaultRodLength(_constantDistance);
	}
}

