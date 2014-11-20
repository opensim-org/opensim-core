/* -------------------------------------------------------------------------- *
 *                    OpenSim:  PointOnLineConstraint.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Samuel R. Hamner                                                *
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

#include "PointOnLineConstraint.h"
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
PointOnLineConstraint::~PointOnLineConstraint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
PointOnLineConstraint::PointOnLineConstraint() :
    Constraint()
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
/**
 * Convenience (API) constructor.
 */
   PointOnLineConstraint::PointOnLineConstraint(OpenSim::Body& lineBody, Vec3 lineDirection, Vec3 pointOnLine,
            OpenSim::Body& followerBody, Vec3 followerPoint) :
    Constraint()
{
    setNull();
    constructProperties();
    set_line_body(lineBody.getName());
    set_line_direction_vec(lineDirection);
    set_point_on_line(pointOnLine);
    set_follower_body(followerBody.getName());
    set_point_on_follower(followerPoint);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================

//_____________________________________________________________________________
/**
 * Set the data members of this PointOnLineConstraint to their null values.
 */
void PointOnLineConstraint::setNull()
{
    setAuthors("Samuel R. Hamner ");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PointOnLineConstraint::constructProperties()
{
    // Line Body Name
    constructProperty_line_body("");

    //Default location and orientation (rotation sequence)
    SimTK::Vec3 origin(0.0, 0.0, 0.0);

    // Line Direction
    constructProperty_line_direction_vec(origin);

    // Default Point On Line
    constructProperty_point_on_line(origin);

    // Follower Body
    constructProperty_follower_body("");

    // Point On Follower Body 
    constructProperty_point_on_follower(origin);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this PointOnLineConstraint.
 */
void PointOnLineConstraint::connectToModel(Model& aModel) {
    Super::connectToModel(aModel);

    string errorMessage;
    // Look up the two bodies being constrained together by name in the
    // model and might as well keep a pointer to them
    std::string lineBodyName = get_line_body();
    std::string followerBodyName = get_follower_body();

    if (!aModel.updBodySet().contains(lineBodyName)) {
        errorMessage = "Invalid line body (" + lineBodyName + ") specified in Constraint " + getName();
        throw (Exception(errorMessage.c_str()));
    }
    if (!aModel.updBodySet().contains(followerBodyName)) {
        errorMessage = "Invalid follower body (" + followerBodyName + ") specified in Constraint " + getName();
        throw (Exception(errorMessage.c_str()));
    }
    _lineBody = &aModel.updBodySet().get(lineBodyName);
    _followerBody = &aModel.updBodySet().get(followerBodyName);
}

void PointOnLineConstraint::addToSystem(SimTK::MultibodySystem& system) const
{
    Super::addToSystem(system);

    // Get underlying mobilized bodies
    SimTK::MobilizedBody lb = _model->updMatterSubsystem().getMobilizedBody((MobilizedBodyIndex)_lineBody->getMobilizedBodyIndex());
    SimTK::MobilizedBody fb = _model->updMatterSubsystem().getMobilizedBody((MobilizedBodyIndex)_followerBody->getMobilizedBodyIndex());

    // Normalize Line Direction
    SimTK::UnitVec3 normLineDirection(get_line_direction_vec().normalize());

    // Now create a Simbody Constraint::PointOnLine
    //PointOnLine(MobilizedBody& lineBody_B, const UnitVec3& defaultLineDirection_B, const Vec3& defaultPointOnLine_B,
    //           MobilizedBody& followerBody_F, const Vec3& defaultFollowerPoint_F);
    SimTK::Constraint::PointOnLine simtkPointOnLine(lb, normLineDirection, get_point_on_line(), fb, get_point_on_follower());
    
    // Beyond the const Component get the index so we can access the SimTK::Constraint later
    PointOnLineConstraint* mutableThis = const_cast<PointOnLineConstraint *>(this);
    mutableThis->_index = simtkPointOnLine.getConstraintIndex();
}

//=============================================================================
// SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Following methods set attributes of the point on line constraint */
void PointOnLineConstraint::setLineBodyByName(std::string aBodyName)
{
    set_line_body(aBodyName);
}

void PointOnLineConstraint::setFollowerBodyByName(std::string aBodyName)
{
    set_follower_body(aBodyName);
}

/** Set the line direction for the point on line constraint*/
void PointOnLineConstraint::setLineDirection(Vec3 direction)
{
    set_line_direction_vec(direction);
}

/** Set the location of the point on the line*/
void PointOnLineConstraint::setPointOnLine(Vec3 point)
{
    set_point_on_line(point);
}

/** Set the location of the point on the follower body*/
void PointOnLineConstraint::setPointOnFollower(Vec3 point)
{
    set_point_on_follower(point);
}
