/* -------------------------------------------------------------------------- *
 *                          OpenSim:  PathPoint.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include "PathPoint.h"
#include "BodySet.h"
#include "Model.h"
#include "GeometryPath.h"
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h> 
#include <OpenSim/Simulation/Wrap/WrapObject.h>
#include <OpenSim/Simulation/Model/Geometry.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;
using SimTK::Transform;

void PathPoint::
changeBodyPreserveLocation(const SimTK::State& s, const PhysicalFrame& body)
{
    if (!hasParent()) {
        throw Exception("PathPoint::changeBodyPreserveLocation attempted to "
            " change the body on PathPoint which was not assigned to a body.");
    }
    // if it is already assigned to aBody, do nothing
    const PhysicalFrame& currentFrame = getParentFrame();

    if (currentFrame == body)
        return;

    // Preserve location means to switch bodies without changing
    // the location of the point in the inertial reference frame.
    upd_location() = currentFrame.findLocationInAnotherFrame(s, get_location(), body);

    // now assign this point's body to point to aBody
    setParentFrame(body);
}

void PathPoint::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    int documentVersion = versionNumber;
    if (documentVersion < XMLDocument::getLatestVersion()) {
        if (documentVersion < 30505) {
            // replace old properties with latest use of Connectors
            SimTK::Xml::element_iterator bodyElement = aNode.element_begin("body");
            std::string bodyName("");
            if (bodyElement != aNode.element_end()) {
                bodyElement->getValueAs<std::string>(bodyName);
                XMLDocument::addConnector(aNode, "Connector_PhysicalFrame_",
                    "parent_frame", bodyName);
            }
        }
    }

    Super::updateFromXMLNode(aNode, versionNumber);
}

