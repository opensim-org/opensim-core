#ifndef OPENSIM_EXPERIMENTAL_FRAME_H_
#define OPENSIM_EXPERIMENTAL_FRAME_H_
/* -------------------------------------------------------------------------- *
*                        OpenSim:  ExperimentalFrame.h                        *
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

#include <OpenSim/Simulation/Model/Frame.h>

namespace OpenSim {
//=============================================================================
//                               ExperimentalFrame
//=============================================================================
/**
ExperimentalFrame is a concrete Frame that represents the experimental value
of a Frame and can be used to display its frame in the OpenSim Visualizer.

Unlike its PhysicalFrame counterpart, an ExperimentalFrame is a measured value
and not physically attached to the musculoskeletal model. It is assumed that
its value is its transform with respect to the lab (Ground) frame.

The transform in Ground value of an ExperimentalFrame is obtained from its
Input<Transform>("transform_in_ground").

@authors Ajay Seth
**/
class OSIMSIMULATION_API ExperimentalFrame : public Frame {
    OpenSim_DECLARE_CONCRETE_OBJECT(ExperimentalFrame, Frame);
public:
    ExperimentalFrame() { constructProperties(); }

    OpenSim_DECLARE_PROPERTY(dimensions, SimTK::Vec3,
        "The block dimensions (m) used to display the ExperimentalFrame.");

    OpenSim_DECLARE_PROPERTY(color, SimTK::Vec3,
        "The color of the block used to display the ExperimentalFrame.");

    OpenSim_DECLARE_PROPERTY(default_transform, SimTK::Transform,
        "The default location of the ExperimentalFrame in Ground.");

    OpenSim_DECLARE_INPUT(transform_in_ground, SimTK::Transform, SimTK::Stage::Time,
        "Provide the ExperimentalFrame's transform in Ground as observed.");

private:
    void constructProperties() {
        constructProperty_default_transform(SimTK::Transform());
        constructProperty_dimensions(SimTK::Vec3(0.04, 0.02, 0.01));
        constructProperty_color(SimTK::Vec3(0.2, 0.9, 0.9));
    }

    const Frame& extendFindBaseFrame() const override { return *this; };

    SimTK::Transform extendFindTransformInBaseFrame() const override {
        return SimTK::Transform();
    }

    /* Calculate the location with respect to and expressed in Ground */
    SimTK::Transform calcTransformInGround(const SimTK::State& s) const override {
        return getInput<SimTK::Vec3>("transform_in_ground").getValue(s);
    }

    SimTK::SpatialVec calcVelocityInGround(const SimTK::State& s) const override {
        return SimTK::SpatialVec(SimTK::Vec3(SimTK::NaN), SimTK::Vec3(SimTK::NaN));
    }

    SimTK::SpatialVec calcAccelerationInGround(const SimTK::State& s) const override {
        return SimTK::SpatialVec(SimTK::Vec3(SimTK::NaN), SimTK::Vec3(SimTK::NaN));
    }

    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
        const SimTK::State& state,
        SimTK::Array_<SimTK::DecorativeGeometry>& appendToThis) const override
    {
        Super::generateDecorations(fixed, hints, state, appendToThis);
        if (!fixed && hints.get_show_markers()) {
            appendToThis.push_back(
                SimTK::DecorativeBrick(get_dimensions()).setBodyId(0)
                .setColor(get_color()).setOpacity(0.5)
                .setTransform(getTransformInGround(state)));
            appendToThis.push_back(SimTK::DecorativeText(getName()).setBodyId(0)
                .setTransform(getTransformInGround(state))
                .setScaleFactors(SimTK::Vec3(get_dimensions().norm()*0.5)));
        }
    }

}; // End of class ExperimentalFrame

}
#endif // OPENSIM_EXPERIMENTAL_FRAME_H_