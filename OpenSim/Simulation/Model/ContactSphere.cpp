/* -------------------------------------------------------------------------- *
 *                        OpenSim:  ContactSphere.cpp                         *
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

#include "ContactSphere.h"
using SimTK::Transform;

namespace OpenSim {

ContactSphere::ContactSphere() :
    ContactGeometry(),
    _radius(_radiusProp.getValueDbl())
{
    setNull();
}

ContactSphere::ContactSphere(double radius, const SimTK::Vec3& location, Body& body) :
    ContactGeometry(location, SimTK::Vec3(0.0), body),
    _radius(_radiusProp.getValueDbl())
{
    setNull();
    setupProperties();
    _radius = radius;
}

ContactSphere::ContactSphere(double radius, const SimTK::Vec3& location, Body& body, const std::string& name) :
    ContactGeometry(location, SimTK::Vec3(0.0), body),
    _radius(_radiusProp.getValueDbl())
{
    setNull();
    setupProperties();
    _radius = radius;
    setName(name);
}

ContactSphere::ContactSphere(const ContactSphere& geom) :
    ContactGeometry(geom),
    _radius(_radiusProp.getValueDbl())
{
    setNull();
    setupProperties();
    _radius = geom._radius;
}

void ContactSphere::setNull()
{
    setAuthors("Peter Eastman");
}

void ContactSphere::setupProperties()
{
    _radiusProp.setName("radius");
    _propertySet.append(&_radiusProp);
}


double ContactSphere::getRadius() const
{
    return _radius;
}

void ContactSphere::setRadius(double radius)
{
    _radius = radius;
}

SimTK::ContactGeometry ContactSphere::createSimTKContactGeometry()
{
    return SimTK::ContactGeometry::Sphere(_radius);
}


//=============================================================================
// VISUALIZER GEOMETRY
//=============================================================================
void ContactSphere::generateDecorations(bool fixed, const ModelDisplayHints& hints, 
    const SimTK::State& s, SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const
{
    Super::generateDecorations(fixed, hints, s, geometry); 

    // There is no fixed geometry to generate here.
    if (fixed) { return; }

    geometry.push_back(SimTK::DecorativeSphere(getRadius())
                           .setTransform(Transform(getLocation()))
                           .setRepresentation(SimTK::DecorativeGeometry::DrawWireframe)
                           .setBodyId(getBody().getMobilizedBodyIndex())
                           .setColor(SimTK::Vec3(0,1,0))
                           .setOpacity(0.5));
}

} // end of namespace OpenSim
