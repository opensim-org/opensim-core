/* -------------------------------------------------------------------------- *
 *                       OpenSim:  ContactGeometry.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
 * Author(s): Peter Eastman                                                   *
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

#include "ContactGeometry.h"
#include "PhysicalOffsetFrame.h"
#include <OpenSim/Common/ScaleSet.h>

using namespace OpenSim;
using SimTK::Vec3;
using SimTK::Rotation;

//=============================================================================
// CONSTRUCTOR
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy 
// assignment operator.

//_____________________________________________________________________________
// Default constructor.
ContactGeometry::ContactGeometry() : ModelComponent()
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
// Convenience constructor.
ContactGeometry::ContactGeometry(const PhysicalFrame& frame) :
    ContactGeometry()
{
    setFrame(frame);
}

ContactGeometry::ContactGeometry(const Vec3& location, const Vec3& orientation, 
    const PhysicalFrame& frame) : ContactGeometry(frame)
{
    set_location(location);
    set_orientation(orientation);
}

void ContactGeometry::setNull()
{
    setAuthors("Peter Eastman");
}

void ContactGeometry::constructProperties()
{
    constructProperty_location(Vec3(0));
    constructProperty_orientation(Vec3(0));
    constructProperty_display_preference(1);

    Array<double> defaultColor(1.0, 3); //color default to 0, 1, 1
    defaultColor[0] = 0.0; 
    constructProperty_color(defaultColor);
}

const Vec3& ContactGeometry::getLocation() const
{ return get_location(); }

void ContactGeometry::setLocation(const Vec3& location)
{ set_location(location); }

const Vec3& ContactGeometry::getOrientation() const
{ return get_orientation(); }

void ContactGeometry::setOrientation(const Vec3& orientation)
{ set_orientation(orientation); }

SimTK::Transform ContactGeometry::getTransform() const
{
    return SimTK::Transform(
            SimTK::Rotation(SimTK::BodyRotationSequence,
                get_orientation()[0], SimTK::XAxis,
                get_orientation()[1], SimTK::YAxis,
                get_orientation()[2], SimTK::ZAxis),
            get_location());
}

const PhysicalFrame& ContactGeometry::getFrame() const
{
    return getConnector<PhysicalFrame>("frame").getConnectee();
}

void ContactGeometry::setFrame(const PhysicalFrame& frame)
{
    updConnector<PhysicalFrame>("frame").connect(frame);
}

const int ContactGeometry::getDisplayPreference()
{ return get_display_preference(); }

void ContactGeometry::setDisplayPreference(const int dispPref)
{ set_display_preference(dispPref); }

const PhysicalFrame& ContactGeometry::getBody() const
{ return getFrame(); }

void ContactGeometry::setBody(const PhysicalFrame& frame)
{ setFrame(frame); }

void ContactGeometry::scale(const ScaleSet& aScaleSet)
{
    throw Exception("ContactGeometry::scale is not implemented");
}

void ContactGeometry::updateFromXMLNode(SimTK::Xml::Element& node,
                                        int versionNumber) {
    if (versionNumber < XMLDocument::getLatestVersion()) {
        if (versionNumber < 30505) {
            SimTK::Xml::element_iterator bodyElement =
                node.element_begin("body_name");
            std::string body_name("");
            // Element may not exist if body_name property had default value.
            if (bodyElement != node.element_end()) {
                bodyElement->getValueAs<std::string>(body_name);
            }
            XMLDocument::addConnector(node, "Connector_PhysicalFrame_",
                    "frame", body_name);
        }
    }
    Super::updateFromXMLNode(node, versionNumber);
}











