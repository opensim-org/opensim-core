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
    Constraint()
{
    setNull();
    constructInfrastructure();
}

PointConstraint::PointConstraint(const PhysicalFrame& body1, 
                                 const SimTK::Vec3& locationBody1,
                                 const PhysicalFrame& body2, 
                                 const SimTK::Vec3& locationBody2) : Constraint()
{
    setNull();
    constructInfrastructure();

    setBody1ByName(body1.getName());
    setBody2ByName(body2.getName());

    set_location_body_1(locationBody1);
    set_location_body_2(locationBody2);
}
//=============================================================================
// CONSTRUCTION
//=============================================================================

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
void PointConstraint::constructProperties()
{
    //Default location and orientation (rotation sequence)
    SimTK::Vec3 origin(0.0, 0.0, 0.0);

    // Location in Body 1 
    constructProperty_location_body_1(origin);

    // Location in Body 2 
    constructProperty_location_body_2(origin);
}

void PointConstraint::constructConnectors()
{
    constructConnector<PhysicalFrame>("body_1");
    constructConnector<PhysicalFrame>("body_2");
}


void PointConstraint::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    // Get underlying mobilized bodies
    // Get underlying mobilized bodies
    const PhysicalFrame& f1 =
        getConnector<PhysicalFrame>("body_1").getConnectee();
    const PhysicalFrame& f2 =
        getConnector<PhysicalFrame>("body_2").getConnectee();

    SimTK::MobilizedBody b1 = f1.getMobilizedBody();
    SimTK::MobilizedBody b2 = f2.getMobilizedBody();

    // Now create a Simbody Constraint::Point
    SimTK::Constraint::Ball simtkPoint(b1, get_location_body_1(), b2, get_location_body_2());
    
    // Beyond the const Component get the index so we can access the SimTK::Constraint later
    assignConstraintIndex(simtkPoint.getConstraintIndex());
}

//=============================================================================
// SET
//=============================================================================
//_____________________________________________________________________________
/*
 * Following methods set attributes of the frames of constraint */
void PointConstraint::setBody1ByName(const std::string& aBodyName)
{
    updConnector<PhysicalFrame>("body_1").set_connected_to_name(aBodyName);
}

void PointConstraint::setBody2ByName(const std::string& aBodyName)
{
    updConnector<PhysicalFrame>("body_2").set_connected_to_name(aBodyName);
}

/** Set the location for point on body 1*/
void PointConstraint::setBody1PointLocation(Vec3 location)
{
    set_location_body_1(location);
}

/** Set the location for point on body 2*/
void PointConstraint::setBody2PointLocation(Vec3 location)
{
    set_location_body_2(location);
}

/** Set the point locations */
void PointConstraint::setContactPointForInducedAccelerations(const SimTK::State &s, Vec3 point)
{
    // make sure we are at the position stage
    getSystem().realize(s, SimTK::Stage::Position);

    const PhysicalFrame& body1 =
        getConnector<PhysicalFrame>("body_1").getConnectee();
    const PhysicalFrame& body2 =
        getConnector<PhysicalFrame>("body_2").getConnectee();

    // For external forces we assume point position vector is defined wrt foot (i.e., _body2)
    // because we are passing it in from a prescribed force.
    // We must also get that point position vector wrt ground (i.e., _body1)
    Vec3 spoint = body2.findLocationInAnotherFrame(s, point, body1);

    setBody1PointLocation(spoint);
    setBody2PointLocation(point);
}

void PointConstraint::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
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
