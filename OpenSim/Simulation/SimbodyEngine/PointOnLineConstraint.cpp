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
#include <OpenSim/Simulation/Model/PhysicalFrame.h>
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
    constructInfrastructure();
}

//_____________________________________________________________________________
/**
 * Convenience (API) constructor.
 */
PointOnLineConstraint::PointOnLineConstraint( const PhysicalFrame& lineBody,
    const SimTK::Vec3& lineDirection, SimTK::Vec3 pointOnLine,
    const PhysicalFrame& followerBody, const SimTK::Vec3& followerPoint) :
        Constraint()
{
    setNull();
    constructInfrastructure();

    setLineBodyByName(lineBody.getName());
    setFollowerBodyByName(followerBody.getName());

    set_line_direction_vec(lineDirection);
    set_point_on_line(pointOnLine);

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


void PointOnLineConstraint::constructProperties()
{
    //Default location and orientation (rotation sequence)
    SimTK::Vec3 origin(0.0, 0.0, 0.0);

    // Line Direction
    constructProperty_line_direction_vec(origin);

    // Default Point On Line
    constructProperty_point_on_line(origin);

    // Point On Follower Body 
    constructProperty_point_on_follower(origin);
}

void PointOnLineConstraint::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
    // Get underlying mobilized bodies
    // Get underlying mobilized bodies
    const PhysicalFrame& fl =
        getConnector<PhysicalFrame>("line_body").getConnectee();
    const PhysicalFrame& ff =
        getConnector<PhysicalFrame>("follower_body").getConnectee();

    SimTK::MobilizedBody bl = fl.getMobilizedBody();
    SimTK::MobilizedBody bf = ff.getMobilizedBody();

    // Normalize Line Direction
    SimTK::UnitVec3 normLineDirection(get_line_direction_vec().normalize());

    // Now create a Simbody Constraint::PointOnLine
    SimTK::Constraint::PointOnLine simtkPointOnLine(bl, normLineDirection,
        get_point_on_line(), bf, get_point_on_follower());
    
    // Beyond the const Component get the index so we can access the SimTK::Constraint later
    assignConstraintIndex(simtkPointOnLine.getConstraintIndex());
}

//=============================================================================
// SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Following methods set attributes of the point on line constraint */
void PointOnLineConstraint::setLineBodyByName(const std::string& aBodyName)
{
    updConnector<PhysicalFrame>("line_body").setConnecteeName(aBodyName);
}

void PointOnLineConstraint::setFollowerBodyByName(const std::string& aBodyName)
{
    updConnector<PhysicalFrame>("follower_body").setConnecteeName(aBodyName);

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

void PointOnLineConstraint::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    int documentVersion = versionNumber;
    if (documentVersion < XMLDocument::getLatestVersion()){
        if (documentVersion<30500){
            // replace old properties with latest use of Connectors
            SimTK::Xml::element_iterator body1Element = aNode.element_begin("line_body");
            SimTK::Xml::element_iterator body2Element = aNode.element_begin("follower_body");
            std::string body1_name(""), body2_name("");
            // If default constructed then elements not serialized since they are default
            // values. Check that we have associated elements, then extract their values.
            if (body1Element != aNode.element_end())
                body1Element->getValueAs<std::string>(body1_name);
            if (body2Element != aNode.element_end())
                body2Element->getValueAs<std::string>(body2_name);
            XMLDocument::addConnector(aNode, "Connector_PhysicalFrame_", "line_body", body1_name);
            XMLDocument::addConnector(aNode, "Connector_PhysicalFrame_", "follower_body", body2_name);
        }
    }

    Super::updateFromXMLNode(aNode, versionNumber);
}
