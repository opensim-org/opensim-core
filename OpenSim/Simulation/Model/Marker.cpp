/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Marker.cpp                            *
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
#include "Marker.h"
#include "Model.h"
#include <OpenSim/Simulation/Model/Model.h>
#include "BodySet.h"

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
/**
 * Default constructor.
 */
Marker::Marker() :
Station()
{

}

//_____________________________________________________________________________
/**
 * Destructor.
 */
Marker::~Marker()
{
}

//_____________________________________________________________________________
/**
 * Set the data members of this Marker to their null values.
 */
void Marker::setNull()
{

}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Update an existing marker with parameter values from a
 * new one, but only for the parameters that were explicitly
 * specified in the XML node.
 *
 * @param aMarker marker to update from
 */
void Marker::updateFromMarker(const Marker &aMarker)
{
    (*this) == aMarker;
}

//_____________________________________________________________________________
/**
 * Set the 'body name' field, which is used when the marker is added to
 * an existing model.
 *
 * @param aName name of body
 */
void Marker::setBodyName(const string& aName)
{
    setReferenceFrame(getModel().getBodySet().get(aName));

}

//_____________________________________________________________________________
/**
 * Get the 'body name' field, which is used when the marker is added to
 * an existing model.
 *
 * @return Pointer to the body name.
 */
const string& Marker::getBodyName() const
{
	//if (_bodyNameProp.getValueIsDefault())
	//	return NULL;

    return getBody().getName();
}

//_____________________________________________________________________________
/**
 * Change the body that this marker is attached to. It assumes that the body is
 * already set, so that connectMarkerToModel() needs to be called to update 
 * dependent information.
 *
 * @param aBody Reference to the body.
 */
void Marker::changeBody(const OpenSim::Body& aBody)
{

    if (aBody == getBody())
		return;

	setBodyName(aBody.getName());
	connectMarkerToModel(updModel());
}

//_____________________________________________________________________________
/**
 * Change the body that this marker is attached to. It assumes that the body is
 * already set, so that connectMarkerToModel() needs to be called to update 
 * dependent information.
 *
 * @param s State.
 * @param aBody Reference to the body.
 */
void Marker::changeBodyPreserveLocation(const SimTK::State& s, OpenSim::Body& aBody)
{

    if (aBody == getBody())
		return;

	// Preserve location means to switch bodies without changing
	// the location of the marker in the inertial reference frame.
    Vec3 newOffset;
    aBody.getModel().getSimbodyEngine().transformPosition(s, getBody(), getOffset(), aBody, newOffset);
    setOffset(newOffset);
	setBodyName(aBody.getName());
	connectMarkerToModel(aBody.updModel());
}

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Scale the marker.
 *
 * @param aScaleFactors XYZ scale factors.
 */
void Marker::scale(const SimTK::Vec3& aScaleFactors)
{
    upd_location() = get_location().elementwiseMultiply(aScaleFactors);
}


void Marker::connectMarkerToModel(Model& aModel)
{
    Super::connectToModel(aModel);
}

//_____________________________________________________________________________
/**
* Override default implementation by object to intercept and fix the XML node
* underneath the Marker to match current version
*/
/*virtual*/
void Marker::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{

    if (versionNumber < XMLDocument::getLatestVersion()){
        if (versionNumber < 30500) {
            // Parse name of Body under <body>node
            SimTK::Xml::element_iterator bIter = aNode.element_begin("body");
            SimTK::String bName = bIter->getValue();
            // Create nodes for new layout
            SimTK::Xml::Element connectorsElement("connectors");
            SimTK::Xml::Element frameElement("Connector_RigidFrame_");
            connectorsElement.insertNodeAfter(connectorsElement.node_end(), frameElement);
            frameElement.setAttributeValue("name", "reference_frame");
            SimTK::Xml::Element connectedToElement("connected_to_name");
            connectedToElement.setValue(bName);
            frameElement.insertNodeAfter(frameElement.node_end(), connectedToElement);
            aNode.insertNodeAfter(bIter, connectorsElement);
            aNode.eraseNode(bIter);
        }
    }
    // Call base class now assuming _node has been corrected for current version
    Object::updateFromXMLNode(aNode, versionNumber);
}
