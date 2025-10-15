/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Marker.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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
#include "Marker.h"
#include "Model.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/*
 * Default constructor.
 */
Marker::Marker() :
Station()
{
    constructProperties();
}

//_____________________________________________________________________________
/*
 * Convenience constructor.
 */
Marker::Marker(const std::string& name, const PhysicalFrame& frame, 
               const SimTK::Vec3& location) :
Station(frame, location)
{
    constructProperties();
    setName(name);
}

//_____________________________________________________________________________
/*
 * Destructor.
 */
Marker::~Marker()
{
}

//_____________________________________________________________________________
/*
 * Set the data members of this Marker to their null values.
 */
void Marker::setNull()
{

}
//_____________________________________________________________________________
/*
 * Construct properties and initialize their default values.
 */
void Marker::constructProperties()
{
    // Indicate whether the Marker is fixed or not (for MarkerPlacement)
    constructProperty_fixed(false);
}

void Marker::setParentFrameName(const string& name)
{
    updSocket<PhysicalFrame>("parent_frame").setConnecteePath(name);
}

//_____________________________________________________________________________
/*
 * Get the 'frame name' field, which is used when the marker is added to
 * an existing model.
 */

const string& Marker::getParentFrameName() const
{
    return getSocket<PhysicalFrame>("parent_frame").getConnecteePath();
}


void Marker::changeFrame(const PhysicalFrame& parentFrame)
{
    if (&parentFrame == &getParentFrame())
        return;

    setParentFrame(parentFrame);
}

void Marker::changeFramePreserveLocation(const SimTK::State& s, 
                                         const PhysicalFrame& parentFrame)
{

    if (&parentFrame == &getParentFrame())
        return;

    // Preserve location means to switch bodies without changing
    // the location of the marker in the inertial reference frame.
    Vec3 newLocation;
    newLocation = findLocationInFrame(s, parentFrame);
    set_location(newLocation);
    setParentFrame(parentFrame);
}

//_____________________________________________________________________________
/*
 * Override default implementation by object to intercept and fix the XML node
 * underneath the Marker to match current version
 */
/*virtual*/
void Marker::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{

    if (versionNumber < XMLDocument::getLatestVersion()){
        if (versionNumber < 30501) {
            // Parse name of Body under <body>node
            SimTK::Xml::element_iterator bIter = aNode.element_begin("body");
            SimTK::String bName = bIter->getValue();
            // Create nodes for new layout
            SimTK::Xml::Element connectorsElement("connectors");
            SimTK::Xml::Element frameElement("Connector_PhysicalFrame_");
            connectorsElement.insertNodeAfter(connectorsElement.node_end(), frameElement);
            frameElement.setAttributeValue("name", "parent_frame");
            SimTK::Xml::Element connecteeElement("connectee_name");
            // Markers in pre-4.0 models are necessarily 1 level deep
            // (model, markers), and Bodies were necessarily 1 level deep;
            // here we create the correct relative path (accounting for sets
            // being components).
            bName = XMLDocument::updateConnecteePath30517("bodyset", bName);
            connecteeElement.setValue(bName);
            frameElement.insertNodeAfter(frameElement.node_end(), connecteeElement);
            aNode.insertNodeAfter(bIter, connectorsElement);
            aNode.eraseNode(bIter);
        }
    }
    // Call base class now assuming _node has been corrected for current version
    Super::updateFromXMLNode(aNode, versionNumber);
}

void Marker::generateDecorationsImpl(bool fixed, const ModelDisplayHints& hints,
        const SimTK::State& state,
        SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const {
    if (!fixed) return;
    if (!hints.get_show_markers()) return;

    // @TODO default color, size, shape should be obtained from hints
    const Vec3 color = hints.get_marker_color();
    const OpenSim::PhysicalFrame& frame = getParentFrame();
    geometry.push_back(
        SimTK::DecorativeSphere(.01).setBodyId(frame.getMobilizedBodyIndex())
            .setColor(color).setOpacity(1.0)
            .setTransform(frame.findTransformInBaseFrame() * get_location())
            .setScaleFactors(Vec3(1)));
}
