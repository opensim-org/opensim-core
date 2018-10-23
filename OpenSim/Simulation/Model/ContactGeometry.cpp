/* -------------------------------------------------------------------------- *
 *                       OpenSim:  ContactGeometry.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
    Appearance defaultAppearance;
    defaultAppearance.set_color(SimTK::Cyan);
    defaultAppearance.set_representation(VisualRepresentation::DrawWireframe);
    constructProperty_Appearance(defaultAppearance);

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
    return getSocket<PhysicalFrame>("frame").getConnectee();
}

void ContactGeometry::setFrame(const PhysicalFrame& frame)
{
    connectSocket_frame(frame);
}

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
            // ContactGeometry in pre-4.0 models are necessarily 1 level deep
            // (model, contact_geom), and Bodies are necessarily 1 level deep.
            // Here we create the correct relative path (accounting for sets
            // being components).
            body_name = XMLDocument::updateConnecteePath30517("bodyset",
                                                              body_name);
            XMLDocument::addConnector(node, "Connector_PhysicalFrame_",
                    "frame", body_name);
        }

        if (versionNumber < 30507) {
            // display_preference and color were replaced with Appearance in PR
            // #1122, at which point the latest XML version number was 30507;
            // however, the corresponding updateFromXMLNode code below was
            // added a while later.
            // https://github.com/opensim-org/opensim-core/pull/1122
            
            SimTK::Xml::Element appearanceNode("Appearance");

            // Move color inside Appearance.
            SimTK::Xml::element_iterator colorIter =
                        node.element_begin("color");
            bool addAppearanceNode = false;
            if (colorIter != node.element_end()) {
                appearanceNode.insertNodeAfter(appearanceNode.element_end(),
                        node.removeNode(colorIter));
                addAppearanceNode = true;
            } else {
                // If the user didn't set a color but set one of the other
                // Appearance properties, then we must set color explicitly so
                // that the Appearance's default of white is not used.
                SimTK::Xml::Element color("color");
                color.setValue("0 1 1"); // SimTK::Cyan
                appearanceNode.insertNodeAfter(appearanceNode.element_end(),
                        color);
            }

            // Move <display_preference> to
            // <Appearance><SurfaceProperties><representation>
            SimTK::Xml::element_iterator reprIter =
                    node.element_begin("display_preference");
            if (reprIter != node.element_end()) {
                if (reprIter->getValue() == "0") {
                    SimTK::Xml::Element visible("visible");
                    visible.setValue("false");
                    appearanceNode.insertNodeAfter(appearanceNode.element_end(),
                            visible);
                } else {
                    reprIter->setElementTag("representation");
                    if (reprIter->getValue() == "4") {
                        // Enum changed to go 0-3 instead of 0-4
                        reprIter->setValue("3");
                    }
                    SimTK::Xml::Element surfProp("SurfaceProperties");
                    surfProp.insertNodeAfter(surfProp.element_end(),
                            node.removeNode(reprIter));
                    appearanceNode.insertNodeAfter(appearanceNode.element_end(),
                            surfProp);
                }
                addAppearanceNode = true;
            }
            if (addAppearanceNode) 
                node.insertNodeAfter(node.element_end(), appearanceNode);
        }
    }
    Super::updateFromXMLNode(node, versionNumber);
}











