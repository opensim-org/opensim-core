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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
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
    Constraint()
{
    setNull();
    constructProperties();
}

WeldConstraint::WeldConstraint(const std::string &name, OpenSim::Body& body1, SimTK::Vec3 locationInBody1, SimTK::Vec3 orientationInBody1,
                               OpenSim::Body& body2, SimTK::Vec3 locationInBody2, SimTK::Vec3 orientationInBody2) : Constraint()
{
    constructProperties();
    setName(name);
    _body1 = &body1;
    _body2 = &body2;
    set_body_1(body1.getName());
    set_body_2(body2.getName());
    set_location_body_1(locationInBody1);
    set_orientation_body_1(orientationInBody1);
    set_location_body_2(locationInBody2);
    set_orientation_body_2(orientationInBody2);
}

WeldConstraint::WeldConstraint(const std::string &name, OpenSim::Body& body1, SimTK::Transform transformInBody1, 
                               OpenSim::Body& body2, SimTK::Transform transformInBody2) : Constraint()
{
    constructProperties();
    setName(name);
    _body1 = &body1;
    _body2 = &body2;
    set_body_1(body1.getName());
    set_body_2(body2.getName());
    set_location_body_1(transformInBody1.p());
    set_orientation_body_1(transformInBody1.R().convertRotationToBodyFixedXYZ());
    set_location_body_2(transformInBody2.p());
    set_orientation_body_2(transformInBody2.R().convertRotationToBodyFixedXYZ());
}

//=============================================================================
// CONSTRUCTION
//=============================================================================

//_____________________________________________________________________________
/**
 * Set the data members of this WeldConstraint to their null values.
 */
void WeldConstraint::setNull()
{
    setAuthors("Ajay Seth");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void WeldConstraint::constructProperties()
{
    // Body 1 name
    constructProperty_body_1("");

    // Body 2 name
    constructProperty_body_2("");

    //Default location and orientation (rotation sequence)
    SimTK::Vec3 origin(0.0, 0.0, 0.0);

    // Location in Body 1 
    constructProperty_location_body_1(origin);

    // Orientation in Body 1 
    constructProperty_orientation_body_1(origin);

    // Location in Body 2 
    constructProperty_location_body_2(origin);

    // Orientation in Body 2 
    constructProperty_orientation_body_2(origin);

}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this WeldConstraint.
 */
void WeldConstraint::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);
    
    string errorMessage;

    if (_body1 && _body2){
        return;
    }

    // Look up the two bodies being welded together by name in the
    // model and might as well keep a pointer to them
    std::string body1Name = get_body_1();
    std::string body2Name = get_body_2();
    if (!aModel.updBodySet().contains(body1Name)) {
        errorMessage = "Invalid weld body1 (" + body1Name + ") specified in Constraint " + getName();
        throw (Exception(errorMessage.c_str()));
    }
    if (!aModel.updBodySet().contains(body2Name)) {
        errorMessage = "Invalid weld body2 (" + body2Name + ") specified in Constraint " + getName();
        throw (Exception(errorMessage.c_str()));
    }
    _body1 = &aModel.updBodySet().get(body1Name);
    _body2 = &aModel.updBodySet().get(body2Name);
}

void WeldConstraint::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    // Get underlying mobilized bodies
    SimTK::MobilizedBody b1 = _model->updMatterSubsystem().getMobilizedBody(_body1->getMobilizedBodyIndex());
    SimTK::MobilizedBody b2 = _model->updMatterSubsystem().getMobilizedBody(_body2->getMobilizedBodyIndex());
    // Build the transforms
    SimTK::Rotation r1; r1.setRotationToBodyFixedXYZ(get_orientation_body_1());
    SimTK::Rotation r2; r2.setRotationToBodyFixedXYZ(get_orientation_body_2());
    SimTK::Transform inb1(r1, get_location_body_1());
    SimTK::Transform inb2(r2, get_location_body_2());

    // Now create a Simbody Constraint::Weld
    SimTK::Constraint::Weld simtkWeld(b1, inb1, b2, inb2);
    
    // Beyond the const Component get the index so we can access the SimTK::Constraint later
    WeldConstraint* mutableThis = const_cast<WeldConstraint *>(this);
    mutableThis->_index = simtkWeld.getConstraintIndex();
}

//=============================================================================
// SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Following methods set attributes of the weld constraint */
void WeldConstraint::setBody1ByName(std::string aBodyName)
{
    set_body_1(aBodyName);
}

void WeldConstraint::setBody2ByName(std::string aBodyName)
{
    set_body_2(aBodyName);
}

/** Set the location and orientation (optional) for weld on body 1*/
void WeldConstraint::setBody1WeldLocation(Vec3 location, Vec3 orientation)
{
    set_location_body_1(location);
    set_orientation_body_1(orientation);

    //if there is a live SimTK::system, we need to push this change down to the underlying constraint.
    if(_index.isValid()){
        SimTK::Constraint::Weld &simConstraint = (SimTK::Constraint::Weld &)_model->updMatterSubsystem().updConstraint(_index);
        // Build the transforms
        SimTK::Rotation r1; r1.setRotationToBodyFixedXYZ(get_orientation_body_1());
        SimTK::Transform inb1(r1, get_location_body_1());
        simConstraint.setDefaultFrameOnBody1(inb1);
    }
}

/** Set the location and orientation (optional) for weld on body 2*/
void WeldConstraint::setBody2WeldLocation(Vec3 location, Vec3 orientation)
{
    set_location_body_2(location);
    set_orientation_body_2(orientation);

    //if there is a live SimTK::system, we need to push this change down to the underlying constraint.
    if(_index.isValid()){
        SimTK::Constraint::Weld &simConstraint = (SimTK::Constraint::Weld &)_model->updMatterSubsystem().updConstraint(_index);
        // Build the transforms
        SimTK::Rotation r2; r2.setRotationToBodyFixedXYZ(get_orientation_body_2());
        SimTK::Transform inb2(r2, get_location_body_2());
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
