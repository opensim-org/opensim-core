/* -------------------------------------------------------------------------- *
 *                       OpenSim:  ContactHalfSpace.cpp                       *
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

#include "ContactHalfSpace.h"
#include "Model.h"

using namespace SimTK;

namespace OpenSim {

ContactHalfSpace::ContactHalfSpace() :
    ContactGeometry()
{
    setNull();
}

ContactHalfSpace::ContactHalfSpace(const SimTK::Vec3& location,
    const SimTK::Vec3& orientation, const PhysicalFrame& frame) :
        ContactGeometry(location, orientation, frame)
{
    setNull();
}

ContactHalfSpace::ContactHalfSpace(const SimTK::Vec3& location, 
    const SimTK::Vec3& orientation, const PhysicalFrame& frame,
    const std::string& name) :
        ContactHalfSpace(location, orientation, frame)
{
    setName(name);
}

void ContactHalfSpace::setNull()
{
    setAuthors("Peter Eastman");
}


SimTK::ContactGeometry ContactHalfSpace::createSimTKContactGeometry() const
{
    return SimTK::ContactGeometry::HalfSpace();
}

void ContactHalfSpace::generateDecorations(bool fixed, const ModelDisplayHints& hints,
    const SimTK::State& s,
    SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const
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
    const double brickHalfThickness = 0.0005;
    geometry.push_back(
        SimTK::DecorativeBrick(Vec3{brickHalfThickness, 0.5, 0.5})
        // The brick is centered on the origin. To ensure the decorative
        // geometry is within the contact geomtry, we must offset by half
        // the thickness of the brick.
        .setTransform(X_BP * Transform(Vec3(brickHalfThickness, 0, 0)))
        .setScale(1)
        .setRepresentation(get_Appearance().get_representation())
        .setBodyId(getFrame().getMobilizedBodyIndex())
        .setColor(get_Appearance().get_color())
        .setOpacity(get_Appearance().get_opacity()));
}


} // end of namespace OpenSim
