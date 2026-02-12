/* -------------------------------------------------------------------------- *
 * OpenSim Moco: StationPlaneContactForce.cpp                                 *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco, Chris Dembia                                   *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "StationPlaneContactForce.h"

using namespace OpenSim;

void StationPlaneContactForce::generateDecorations(
        bool fixed, const ModelDisplayHints& hints,
        const SimTK::State& s,
        SimTK::Array_<SimTK::DecorativeGeometry>& geoms) const {
    Super::generateDecorations(fixed, hints, s, geoms);
    if (!fixed) {
        getModel().realizeVelocity(s);
        // Normalize contact force vector by body weight so that the line
        // is 1 meter long if the contact force magnitude is equal to
        // body weight.
        const double arrowLengthPerForce = 1.0 / 1000.0; // meters / Newton
        //const double mg =
        //        getModel().getTotalMass(s) * getModel().getGravity().norm();
        // TODO avoid recalculating.
        const auto& pt = getConnectee<Station>("station");
        const auto pt1 = pt.getLocationInGround(s);
        const SimTK::Vec3 force = calcContactForceOnStation(s);
        const SimTK::Vec3 pt2 = pt1 + force * arrowLengthPerForce; // mg;
        SimTK::DecorativeLine line(pt1, pt2);
        line.setColor(SimTK::Green);
        line.setLineThickness(0.10);
        geoms.push_back(line);

        // TODO move to fixed.
        SimTK::DecorativeSphere sphere;
        sphere.setColor(SimTK::Green);
        sphere.setRadius(0.01);
        sphere.setBodyId(pt.getParentFrame().getMobilizedBodyIndex());
        sphere.setRepresentation(SimTK::DecorativeGeometry::DrawWireframe);
        sphere.setTransform(SimTK::Transform(pt.get_location()));
        geoms.push_back(sphere);
    }
}
