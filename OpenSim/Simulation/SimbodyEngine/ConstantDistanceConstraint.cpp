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

//=============================================================================
// CONSTRUCTION
//=============================================================================
/* Default constructor.
 */
ConstantDistanceConstraint::ConstantDistanceConstraint() :
    Constraint()
{
    setNull();
    constructInfrastructure();
}

/*
 * Convenience Constructor.
*/
ConstantDistanceConstraint::ConstantDistanceConstraint(
    const PhysicalFrame& body1, const SimTK::Vec3& locationBody1,
    const PhysicalFrame& body2, const SimTK::Vec3& locationBody2,
    const double& distance) : Constraint()
{
    setNull();
    constructInfrastructure();

    setBody1ByName(body1.getName());
    setBody2ByName(body2.getName());
    set_location_body_1(locationBody1);

    set_location_body_2(locationBody2);
    set_constant_distance(distance);
}

/* Set the data members of this ConstantDistanceConstraint to their null values.
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
    //Default location and orientation (rotation sequence)
    SimTK::Vec3 origin(0.0, 0.0, 0.0);

    constructProperty_location_body_1(origin);
    constructProperty_location_body_2(origin);

    // Constant distance between points
    constructProperty_constant_distance(SimTK::NaN);
}

void ConstantDistanceConstraint::constructConnectors()
{
    constructConnector<PhysicalFrame>("body_1");
    constructConnector<PhysicalFrame>("body_2");
}


void ConstantDistanceConstraint::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    // Get underlying mobilized bodies
    const PhysicalFrame& f1 =
        getConnector<PhysicalFrame>("body_1").getConnectee();
    const PhysicalFrame& f2 =
        getConnector<PhysicalFrame>("body_2").getConnectee();

    SimTK::MobilizedBody b1 = f1.getMobilizedBody();
    SimTK::MobilizedBody b2 = f2.getMobilizedBody();

    // Now create a Simbody Constraint::Rod
    SimTK::Constraint::Rod simtkRod(b1, get_location_body_1(),
                                    b2, get_location_body_2(),
                                    get_constant_distance() );
    
    // Beyond the const Component get the index so we can access the SimTK::Constraint later
    assignConstraintIndex(simtkRod.getConstraintIndex());
}

//=============================================================================
// GET/SET
//=============================================================================
const PhysicalFrame& ConstantDistanceConstraint::getBody1() const
{
    return getConnector<PhysicalFrame>("body_1").getConnectee();
}

const PhysicalFrame& ConstantDistanceConstraint::getBody2() const
{
    return getConnector<PhysicalFrame>("body_2").getConnectee();
}

/*
* Following methods set attributes of the constraint */
void ConstantDistanceConstraint::setBody1ByName(const std::string& aBodyName)
{
    updConnector<PhysicalFrame>("body_1").set_connected_to_name(aBodyName);
}

void ConstantDistanceConstraint::setBody2ByName(const std::string& aBodyName)
{
    updConnector<PhysicalFrame>("body_2").set_connected_to_name(aBodyName);
}

/** Set the location for point on body 1*/
void ConstantDistanceConstraint::setBody1PointLocation(Vec3 location)
{
    set_location_body_1(location);
}

/** Set the location for point on body 2*/
void ConstantDistanceConstraint::setBody2PointLocation(Vec3 location)
{
    set_location_body_2(location);
}

/** Set the constant distance between the two points*/
void ConstantDistanceConstraint::setConstantDistance(double distance)
{
    set_constant_distance(distance);
}

void ConstantDistanceConstraint::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    int documentVersion = versionNumber;
    if (documentVersion < XMLDocument::getLatestVersion()){
        if (documentVersion<30500){
            // replace old properties with latest use of Connectors
            SimTK::Xml::element_iterator body1Element = aNode.element_begin("body_1");
            SimTK::Xml::element_iterator body2Element = aNode.element_begin("body_2");
            std::string body1_name, body2_name;
            body1Element->getValueAs<std::string>(body1_name);
            body2Element->getValueAs<std::string>(body2_name);
            XMLDocument::addConnector(aNode, "Connector_PhysicalFrame_", "body_1", body1_name);
            XMLDocument::addConnector(aNode, "Connector_PhysicalFrame_", "body_2", body2_name);
        }
    }

    Super::updateFromXMLNode(aNode, versionNumber);
}