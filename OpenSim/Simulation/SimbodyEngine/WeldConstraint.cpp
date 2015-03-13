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
#include "WeldConstraint.h"

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
    constructInfrastructure();
}

WeldConstraint::WeldConstraint(const std::string &name,
    const PhysicalFrame& body1, SimTK::Vec3 locationInBody1, SimTK::Vec3 orientationInBody1,
    const PhysicalFrame& body2, SimTK::Vec3 locationInBody2, SimTK::Vec3 orientationInBody2) :
        Constraint()
{
    constructInfrastructure();
    setName(name);
    setBody1ByName(body1.getName());
    setBody2ByName(body2.getName());
    set_location_body_1(locationInBody1);
    set_orientation_body_1(orientationInBody1);
    set_location_body_2(locationInBody2);
    set_orientation_body_2(orientationInBody2);
}

WeldConstraint::WeldConstraint(const std::string &name,
    const PhysicalFrame& body1, SimTK::Transform transformInBody1,
    const PhysicalFrame& body2, SimTK::Transform transformInBody2) :
        Constraint()
{
    constructInfrastructure();
    setName(name);
    setBody1ByName(body1.getName());
    setBody2ByName(body2.getName());
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
/*
 * Construct WeldConstraint's properties
 */
void WeldConstraint::constructProperties()
{

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

void WeldConstraint::constructConnectors()
{
    constructConnector<PhysicalFrame>("body_1");
    constructConnector<PhysicalFrame>("body_2");
}


void WeldConstraint::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    // Get underlying mobilized bodies
    const PhysicalFrame& f1 = 
        getConnector<PhysicalFrame>("body_1").getConnectee();
    const PhysicalFrame& f2 = 
        getConnector<PhysicalFrame>("body_2").getConnectee();

    SimTK::MobilizedBody b1 = f1.getMobilizedBody();
    SimTK::MobilizedBody b2 = f2.getMobilizedBody();
    // Build the transforms
    SimTK::Rotation r1; r1.setRotationToBodyFixedXYZ(get_orientation_body_1());
    SimTK::Rotation r2; r2.setRotationToBodyFixedXYZ(get_orientation_body_2());
    SimTK::Transform inb1(r1, get_location_body_1());
    SimTK::Transform inb2(r2, get_location_body_2());

    // Now create a Simbody Constraint::Weld
    SimTK::Constraint::Weld simtkWeld(b1, inb1, b2, inb2);
    
    // Beyond the const Component get the index so we can access the SimTK::Constraint later
    assignConstraintIndex(simtkWeld.getConstraintIndex());
}

//=============================================================================
// SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Following methods set attributes of the weld constraint */
void WeldConstraint::setBody1ByName(const std::string& aBodyName)
{
    updConnector<PhysicalFrame>("body_1").set_connected_to_name(aBodyName);
}

void WeldConstraint::setBody2ByName(const std::string& aBodyName)
{
    updConnector<PhysicalFrame>("body_2").set_connected_to_name(aBodyName);
}

/** Set the location and orientation (optional) for weld on body 1*/
void WeldConstraint::setBody1WeldLocation(Vec3 location, Vec3 orientation)
{
    set_location_body_1(location);
    set_orientation_body_1(orientation);
}

/** Set the location and orientation (optional) for weld on body 2*/
void WeldConstraint::setBody2WeldLocation(Vec3 location, Vec3 orientation)
{
    set_location_body_2(location);
    set_orientation_body_2(orientation);
}

void WeldConstraint::setContactPointForInducedAccelerations(const SimTK::State &s, Vec3 point)
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
    
    setBody1WeldLocation(spoint, body1.getGroundTransform(s).R().convertRotationToBodyFixedXYZ());
    setBody2WeldLocation(point, body2.getGroundTransform(s).R().convertRotationToBodyFixedXYZ());
}

void WeldConstraint::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
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
