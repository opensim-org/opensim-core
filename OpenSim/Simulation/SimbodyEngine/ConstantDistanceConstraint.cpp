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
    Constraint()
{
    setNull();
    constructProperties();
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
    Constraint()
{

    setNull();
    constructProperties();
    set_body_1(body1.getName());
    set_location_body_1(locationBody1);
    set_body_2(body2.getName());
    set_location_body_2(locationBody2);
    set_constant_distance(distance);

}
//=============================================================================
// CONSTRUCTION
//=============================================================================


//_____________________________________________________________________________
/**
 * Set the data members of this ConstantDistanceConstraint to their null values.
 */
void ConstantDistanceConstraint::setNull()
{
    setAuthors("Matt S. DeMers");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ConstantDistanceConstraint::constructProperties()
{
    constructProperty_body_1("");
    constructProperty_body_2("");

    //Default location and orientation (rotation sequence)
    SimTK::Vec3 origin(0.0, 0.0, 0.0);

    constructProperty_location_body_1(origin);
    constructProperty_location_body_2(origin);

    // Constant distance between points
    constructProperty_constant_distance(1.0);
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
    std::string body1Name = get_body_1();
    std::string body2Name = get_body_2();
    if (!aModel.updBodySet().contains(body1Name)) {
        errorMessage = "Invalid point constraint body1 (" + body1Name + ") specified in Constraint " + getName();
        throw (Exception(errorMessage.c_str()));
    }
    if (!aModel.updBodySet().contains(body2Name)) {
        errorMessage = "Invalid point constraint body2 (" + body2Name + ") specified in Constraint " + getName();
        throw (Exception(errorMessage.c_str()));
    }
    _body1 = &aModel.updBodySet().get(body1Name);
    _body2 = &aModel.updBodySet().get(body2Name);
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
    SimTK::Constraint::Rod simtkRod(b1, get_location_body_1(), b2, get_location_body_2(), get_constant_distance());

    // Beyond the const Component get the index so we can access the SimTK::Constraint later
    ConstantDistanceConstraint* mutableThis = const_cast<ConstantDistanceConstraint *>(this);
    mutableThis->_index  = simtkRod.getConstraintIndex();
}

//=============================================================================
// SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Following methods set attributes of the point constraint */
void ConstantDistanceConstraint::setBody1ByName(std::string aBodyName)
{
    set_body_1(aBodyName);
}

void ConstantDistanceConstraint::setBody2ByName(std::string aBodyName)
{
    set_body_2(aBodyName);
}

/** Set the location for point on body 1*/
void ConstantDistanceConstraint::setBody1PointLocation(Vec3 location)
{
    set_location_body_1(location);
    //if there is a live SimTK::system, we need to push this change down to the underlying constraint.
    if(_index.isValid()) {
        SimTK::Constraint::Rod &simConstraint = (SimTK::Constraint::Rod &)_model->updMatterSubsystem().updConstraint(_index);
        simConstraint.setDefaultPointOnBody1(get_location_body_1());
    }
}

/** Set the location for point on body 2*/
void ConstantDistanceConstraint::setBody2PointLocation(Vec3 location)
{
    set_location_body_2(location);
    //if there is a live SimTK::system, we need to push this change down to the underlying constraint.
    if(_index.isValid()) {
        SimTK::Constraint::Rod &simConstraint = (SimTK::Constraint::Rod &)_model->updMatterSubsystem().updConstraint(_index);
        simConstraint.setDefaultPointOnBody2(get_location_body_2());
    }
}

/** Set the constant distance between the two points*/
void ConstantDistanceConstraint::setConstantDistance(double distance)
{
    set_constant_distance(distance);
    //if there is a live SimTK::system, we need to push this change down to the underlying constraint.
    if(_index.isValid()) {
        SimTK::Constraint::Rod &simConstraint = (SimTK::Constraint::Rod &)_model->updMatterSubsystem().updConstraint(_index);
        simConstraint.setDefaultRodLength(get_constant_distance());
    }
}

