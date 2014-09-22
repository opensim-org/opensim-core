/* -------------------------------------------------------------------------- *
 *                       OpenSim:  ContactGeometry.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include "BodySet.h"
#include "Model.h"
#include <OpenSim/Common/ScaleSet.h>

using SimTK::Vec3;
using SimTK::Rotation;

namespace OpenSim {

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
// Convienience constructor.
ContactGeometry::ContactGeometry(const Vec3& location, const Vec3& orientation, 
    PhysicalFrame& body) : ModelComponent()
{
	setNull();
    constructProperties();

    _body = &body;
    set_body_name(body.getName());
    set_location(location);
    set_orientation(orientation);
}

void ContactGeometry::setNull()
{
	setAuthors("Peter Eastman");
}


//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ContactGeometry::constructProperties()
{
    constructProperty_body_name("Unassigned");
    constructProperty_location(Vec3(0));
    constructProperty_orientation(Vec3(0));
    constructProperty_display_preference(1);

    Array<double> defaultColor(1.0, 3); //color default to 0, 1, 1
	defaultColor[0] = 0.0; 
    constructProperty_color(defaultColor);
}

const Vec3& ContactGeometry::getLocation() const
{
    return get_location();
}

void ContactGeometry::setLocation(const Vec3& location)
{
    set_location(location);
}

const Vec3& ContactGeometry::getOrientation() const
{
    return get_orientation();
}

void ContactGeometry::setOrientation(const Vec3& orientation)
{
    set_orientation(orientation);
}

SimTK::Transform ContactGeometry::getTransform()
{
    return SimTK::Transform(Rotation(SimTK::BodyRotationSequence,
        get_orientation()[0], SimTK::XAxis,
        get_orientation()[1], SimTK::YAxis,
        get_orientation()[2], SimTK::ZAxis), get_location());
}

PhysicalFrame& ContactGeometry::updBody()
{
    return *_body;
}

const PhysicalFrame& ContactGeometry::getBody() const
{
    return *_body;
}

void ContactGeometry::setBody(PhysicalFrame& body)
{
    _body = &body;
    set_body_name(body.getName());
}

const std::string& ContactGeometry::getBodyName()
{
    return get_body_name();
}

void ContactGeometry::setBodyName(const std::string& name)
{
    set_body_name(name);
    _body = nullptr;
}

const int ContactGeometry::getDisplayPreference()
{
    return get_display_preference();
}

void ContactGeometry::setDisplayPreference(const int dispPref)
{
    set_display_preference(dispPref);
}

void ContactGeometry::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);

    //TODO use Connectors!
    try {
        _body =
            static_cast<PhysicalFrame*>(&updModel().updComponent(get_body_name()));
    }
	catch (...)
    {
        std::string errorMessage = "Invalid body (" + get_body_name() + ") specified in contact geometry " + getName();
		throw (Exception(errorMessage.c_str()));
	}
}

void ContactGeometry::scale(const ScaleSet& aScaleSet)
{
    throw Exception("ContactGeometry::Scale is not implemented");
}

} // end of namespace OpenSim
