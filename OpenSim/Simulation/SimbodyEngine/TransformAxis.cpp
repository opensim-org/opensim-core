/* -------------------------------------------------------------------------- *
 *                        OpenSim:  TransformAxis.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
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

#include "TransformAxis.h"
#include "Joint.h"
#include "Coordinate.h"
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Common/Constant.h>

using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
TransformAxis::TransformAxis() {
    setNull();
    constructProperties();
}

TransformAxis::TransformAxis(const Array<std::string>& coordNames,
        const SimTK::Vec3& axis) {
    setNull();
    constructProperties();
    setCoordinateNames(coordNames);
    setAxis(axis);
}

TransformAxis::TransformAxis(SimTK::Xml::Element& aNode) {
    setNull();
    constructProperties();
    updateFromXMLNode(aNode);
}

//=============================================================================
// ACCESSORS
//=============================================================================
void TransformAxis::setCoordinateNames(const Array<std::string>& coordNames) {
    set_coordinates(coordNames);
}

const Property<std::string>& TransformAxis::getCoordinateNames() const {
    return getProperty_coordinates();
}

Array<std::string> TransformAxis::getCoordinateNamesInArray() const {
    Array<std::string> coords;
    for (int i = 0; i < getProperty_coordinates().size(); ++i) {
        coords.append(get_coordinates(i));
    }
    return coords;
}

void TransformAxis::setAxis(const SimTK::Vec3& axis) {
    set_axis(axis);
}

const SimTK::Vec3& TransformAxis::getAxis() const {
    return get_axis();
}

void TransformAxis::getAxis(SimTK::Vec3& axis) const {
    axis = getAxis();
}

double TransformAxis::getAxis(int which) const {
    OPENSIM_ASSERT_FRMOBJ(0 <= which && which <= 2);
    return getAxis()[which];
}

bool TransformAxis::hasFunction() const {
    return !getProperty_function().empty();
}

const Function& TransformAxis::getFunction() const {
    return get_function();
    const Property<Function>& function = getProperty_function();
    if (function.empty())
        throw Exception("TransformAxis::getFunction(): no Function is defined");
    return function.getValue();
}

Function& TransformAxis::updFunction() {
    Property<Function>& function = updProperty_function();
    if (function.empty())
        throw Exception("TransformAxis::getFunction(): no Function is defined");
    return function.updValue();
}

void TransformAxis::setFunction(Function* function) {
    Property<Function>& property = updProperty_function();
    property.clear();
    property.adoptAndAppendValue(function);
}

void TransformAxis::setFunction(const Function& function) {
    updProperty_function() = function;
}

const Joint& TransformAxis::getJoint() const {
    return *_joint;
}

double TransformAxis::getValue(const SimTK::State& s)
{
    const Property<std::string>& coordNames = getCoordinateNames();
    const int nc = coordNames.size();
    const auto& coords = _joint->getProperty_coordinates();

    SimTK::Vector workX(nc, 0.0);
    for (int i=0; i < nc; ++i) {
        const int idx = coords.findIndexForName( coordNames[i] );
        workX[i] = _joint->get_coordinates(idx).getValue(s);
    }

    return getFunction().calcValue(workX);
}

void TransformAxis::connectToJoint(const Joint& aJoint)
{
    std::string errorMessage;

    _joint = &aJoint;

    // Look up the coordinates by name.
    const Property<std::string>& coordNames = getProperty_coordinates();
    int nc = coordNames.size();
    const auto& coords = _joint->getProperty_coordinates();

    if (!hasFunction()) {
        SimTK_ASSERT2_ALWAYS(coordNames.size() == 0,
            "CustomJoint (%s) %s axis has no function but has coordinates.",
            _joint->getName().c_str(), getName().c_str());
        return;
    }

    for(int i=0; i< nc; ++i) {
        if (coords.findIndexForName( coordNames[i] ) < 0) {
            errorMessage += "Invalid coordinate ("
                            + coordNames[i]
                            + ") specified for TransformAxis "
                            + getName() + " in joint " + aJoint.getName();
            throw (Exception(errorMessage));
        }
    }
}

//=============================================================================
// CONVENIENCE METHODS
//=============================================================================
void TransformAxis::setNull() {
    _joint = NULL;
}

void TransformAxis::constructProperties() {
    constructProperty_coordinates();
    constructProperty_axis(SimTK::Vec3(1, 0, 0));
    constructProperty_function(Constant(0));
}

//=============================================================================
// OBJECT INTERFACE
//=============================================================================
void TransformAxis::updateFromXMLNode(SimTK::Xml::Element& node,
        int versionNumber) {
    // Version before refactoring spatialTransform.
    // TODO: this is handled in CustomJoint's updateFromXMLNode() method
    // but should be dealt with here. Can't do it both places, though.

    //XMLDocument::renameChildNode(node, "coordinate", "coordinates");

    Super::updateFromXMLNode(node, versionNumber);
}
