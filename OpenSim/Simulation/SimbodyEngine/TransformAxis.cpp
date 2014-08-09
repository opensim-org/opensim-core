/* -------------------------------------------------------------------------- *
 *                        OpenSim:  TransformAxis.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan, Frank C. Anderson, Jeffrey A. Reinbolt, Ajay Seth,  *
 *            Michael Sherman                                                 *
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
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Simulation/Model/Model.h>

#include "TransformAxis.h"
#include "Joint.h"
#include "Coordinate.h"
#include "SimbodyEngine.h"

//=============================================================================
// USING
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;
using SimTK::State;
using SimTK::Vector;
using SimTK::Xml;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
// Default constructor.
TransformAxis::TransformAxis() {
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
// Constructor with coordinate names and axis.
TransformAxis::TransformAxis(const Array<string>& coordNames,
                             const Vec3&          axis) {
    setNull();
    constructProperties();
    setCoordinateNames(coordNames);
    setAxis(axis);
}
// Constructor from XML node.
TransformAxis::TransformAxis(Xml::Element& aNode) {
    setNull();
    constructProperties();
    updateFromXMLNode(aNode);
}

//_____________________________________________________________________________
/**
 * Perform set up functions after model has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this dof.
 * @param aJoint joint containing this dof.
 */
void TransformAxis::connectToJoint(const Joint& aJoint)
{
    string errorMessage;

    _joint = &aJoint;

    // Look up the coordinates by name.
    const Property<string>& coordNames = getProperty_coordinates();
    int nc = coordNames.size();
    const CoordinateSet& coords = _joint->getCoordinateSet();

    // If a Function has been assigned then we have to insist that any
    // specified coordinates actually exist in this joint.
    // TODO: (sherm 20120418) Why do we allow unrecognized coordinate names
    // if there is no Function?

    if (!hasFunction())
        return; // no need to check

    for(int i=0; i< nc; ++i) {
        if (!coords.contains(coordNames[i])) {
            errorMessage += "Invalid coordinate ("
                            + coordNames[i]
                            + ") specified for TransformAxis "
                            + getName() + " in joint " + aJoint.getName();
            throw (Exception(errorMessage));
        }
    }
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================

//_____________________________________________________________________________
/**
 * Set the data members of this TransformAxis to their null values.
 */
void TransformAxis::setNull()
{
    _joint = NULL;
}

//_____________________________________________________________________________
// Define properties.
void TransformAxis::constructProperties() {
    constructProperty_coordinates();
    constructProperty_axis(Vec3(1,0,0)); // use x-axis by default
    constructProperty_function();
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the current value of the transform.
 *
 * @return Current value of the transform.
 */
double TransformAxis::getValue(const State& s )
{
    const Property<string>& coordNames = getCoordinateNames();
    const int nc = coordNames.size();
    const CoordinateSet& coords = _joint->getCoordinateSet();

    Vector workX(nc, 0.0);
    for (int i=0; i < nc; ++i)
        workX[i] = coords.get(coordNames[i]).getValue(s);

    return getFunction().calcValue(workX);
}

//_____________________________________________________________________________
const OpenSim::Function& TransformAxis::getFunction() const
{
    const Property<Function>& function = getProperty_function();
    if (function.empty())
        throw Exception("TransformAxis::getFunction(): no Function is defined");
    return function.getValue();
}

OpenSim::Function& TransformAxis::updFunction()
{
    Property<Function>& function = updProperty_function();
    if (function.empty())
        throw Exception("TransformAxis::getFunction(): no Function is defined");
    return function.updValue();
}

//_____________________________________________________________________________
// This method takes over ownership.
void TransformAxis::setFunction(OpenSim::Function* func)
{
    Property<Function>& prop = updProperty_function();
    prop.clear();
    prop.adoptAndAppendValue(func);
}

// This method makes a copy.
void TransformAxis::setFunction(const OpenSim::Function& func)
{
    updProperty_function() = func;
}


void TransformAxis::updateFromXMLNode
(Xml::Element& node, int versionNumber)
{
    // Version before refactoring spatialTransform.
    // TODO: this is handled in CustomJoint's updateFromXMLNode() method
    // but should be dealt with here. Can't do it both places, though.

    //XMLDocument::renameChildNode(node, "coordinate", "coordinates");

    Super::updateFromXMLNode(node, versionNumber);
}