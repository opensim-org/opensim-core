/* -------------------------------------------------------------------------- *
 *                       OpenSim:  ContactHalfSpace.cpp                       *
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

#include "ContactHalfSpace.h"

using namespace SimTK;

namespace OpenSim {

ContactHalfSpace::ContactHalfSpace() :
    ContactGeometry()
{
    setNull();
}

ContactHalfSpace::ContactHalfSpace(const SimTK::Vec3& location, const SimTK::Vec3& orientation, Body& body) :
    ContactGeometry(location, orientation, body)
{
    setNull();
}

ContactHalfSpace::ContactHalfSpace(const SimTK::Vec3& location, const SimTK::Vec3& orientation, Body& body, const std::string& name) :
    ContactGeometry(location, orientation, body)
{
    setNull();
    setName(name);
}

ContactHalfSpace::ContactHalfSpace(const ContactHalfSpace& geom) :
    ContactGeometry(geom)
{
    setNull();
}

void ContactHalfSpace::setNull()
{
	setAuthors("Peter Eastman");
}


SimTK::ContactGeometry ContactHalfSpace::createSimTKContactGeometry()
{
    _displayer.freeGeometry();
	_displayer.addGeometry(new PolyhedralGeometry("unit_plane.obj"));
	return SimTK::ContactGeometry::HalfSpace();
}

//=============================================================================
// VISUALIZER GEOMETRY
//=============================================================================
void ContactHalfSpace::generateDecorations(bool fixed, const ModelDisplayHints& hints, 
    const SimTK::State& s, SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const
{
   // There is no fixed geometry to generate here.
    if (!fixed) { return; }

    geometry.push_back(SimTK::DecorativeMeshFile("unit_plane.obj")
                           .setTransform(getTransform())
                           .setRepresentation(SimTK::DecorativeGeometry::DrawSurface)
                           .setBodyId(getBody().getIndex())
                           .setColor(SimTK::Vec3(0,1,0))
                           .setOpacity(0.5)
                           .setUserRef(const_cast<ContactHalfSpace*>(this)));
}

} // end of namespace OpenSim
