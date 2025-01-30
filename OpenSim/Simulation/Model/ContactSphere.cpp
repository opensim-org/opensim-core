/* -------------------------------------------------------------------------- *
 *                        OpenSim:  ContactSphere.cpp                         *
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
#include "Model.h"
#include "ContactSphere.h"
using SimTK::Transform;

namespace OpenSim {

ContactSphere::ContactSphere() : ContactGeometry()
{
    setNull();
    constructProperties();
}

ContactSphere::ContactSphere(double radius, const SimTK::Vec3& location,
        const PhysicalFrame& frame) :
    ContactGeometry(location, SimTK::Vec3(0.0), frame)
{
    setNull();
    constructProperties();
    set_radius(radius);
}

ContactSphere::ContactSphere(double radius, const SimTK::Vec3& location,
        const PhysicalFrame& frame, const std::string& name) :
    ContactSphere(radius, location, frame)
{
    setName(name);
}

void ContactSphere::setNull()
{
    setAuthors("Peter Eastman");
}

void ContactSphere::constructProperties()
{
    constructProperty_radius(0);
}


double ContactSphere::getRadius() const
{
    return get_radius();
}

void ContactSphere::setRadius(double radius)
{
    set_radius(radius);
}

SimTK::ContactGeometry ContactSphere::createSimTKContactGeometry() const
{
    return SimTK::ContactGeometry::Sphere(get_radius());
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

    // Model-wide hints indicate that contact geometry shouldn't be shown.
    if (!hints.get_show_contact_geometry()) { return; }

    // The decoration has been toggled off by its `Appearance` block.
    if (!get_Appearance().get_visible())  { return; }

    // B: base Frame (Body or Ground)
    // F: PhysicalFrame that this ContactGeometry is connected to
    // P: the frame defined (relative to F) by the location and orientation
    //    properties.
    const auto& X_BF = getFrame().findTransformInBaseFrame();
    const auto& X_FP = getTransform();
    const auto X_BP = X_BF * X_FP;
    geometry.push_back(SimTK::DecorativeSphere(getRadius()).setScale(1)
                           .setTransform(X_BP)
                           .setRepresentation(get_Appearance().get_representation())
                           .setBodyId(getFrame().getMobilizedBodyIndex())
                           .setColor(get_Appearance().get_color())
                           .setOpacity(get_Appearance().get_opacity()));
}

} // end of namespace OpenSim
