#ifndef OPENSIM_EXPERIMENTAL_MARKER_H_
#define OPENSIM_EXPERIMENTAL_MARKER_H_
/* -------------------------------------------------------------------------- *
*                        OpenSim:  ExperimentalMarker.h                        *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2018 Stanford University and the Authors                *
* Author(s): Ajay Seth                                                       *
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

#include <OpenSim/Simulation/Model/Point.h>

namespace OpenSim {
//=============================================================================
//                               ExperimentalMarker
//=============================================================================
/**
ExperimentalMarker is a concrete Point that represents the experimental value
of a Marker and can be used to display its location in the OpenSim
Visualizer.

Unlike its model Marker counterpart, an ExperimentalMarker is a measured value
and not physically attached to the musculoskeletal model. It is assumed that
its value is its location with respect to the lab (Ground) frame.

The location in Ground value of an ExperimentalMarker is obtained from its
Input<Vec3>("location_in_ground").

@authors Ajay Seth
**/
class OSIMSIMULATION_API ExperimentalMarker : public Point {
    OpenSim_DECLARE_CONCRETE_OBJECT(ExperimentalMarker, Point);
public:
    ExperimentalMarker() { constructProperties(); }

    OpenSim_DECLARE_PROPERTY(radius, double,
        "The radius of the sphere used to display the ExperimentalMarker.");

    OpenSim_DECLARE_PROPERTY(color, SimTK::Vec3,
        "The color of the sphere used to display the ExperimentalMarker.");

    OpenSim_DECLARE_INPUT(location_in_ground, SimTK::Vec3, SimTK::Stage::Time,
        "Provide ExperimentalMarker location_in_ground the Ground.");

private:
    void constructProperties() {
        constructProperty_radius(0.02);
        constructProperty_color(SimTK::Vec3(0.9, 0.9, 0.2));
    }

    /* Calculate the location with respect to and expressed in Ground */
    SimTK::Vec3 calcLocationInGround(const SimTK::State& s) const override {
        return getInput<SimTK::Vec3>("location_in_ground").getValue(s);
    }

    SimTK::Vec3 calcVelocityInGround(const SimTK::State& s) const override {
        return SimTK::Vec3(SimTK::NaN);
    }

    SimTK::Vec3 calcAccelerationInGround(const SimTK::State& s) const override {
        return SimTK::Vec3(SimTK::NaN);
    }

    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
        const SimTK::State& state,
        SimTK::Array_<SimTK::DecorativeGeometry>& appendToThis) const override
    {
        Super::generateDecorations(fixed, hints, state, appendToThis);
        if (!fixed && hints.get_show_markers()) {
            appendToThis.push_back(
                SimTK::DecorativeSphere(get_radius()).setBodyId(0)
                .setColor(get_color()).setOpacity(0.5)
                .setTransform(getLocationInGround(state)));
            appendToThis.push_back(SimTK::DecorativeText(getName()).setBodyId(0)
                .setTransform(getLocationInGround(state))
                .setScaleFactors(SimTK::Vec3(get_radius()*0.5)));
        }
    }

}; // End of class ExperimentalMarker

} // end of namespace OpenSim

#endif // OPENSIM_EXPERIMENTAL_MARKER_H_