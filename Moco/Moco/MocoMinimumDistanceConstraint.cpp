/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoMinimumDistanceConstraint.cpp                            *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
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

#include "MocoMinimumDistanceConstraint.h"

using namespace OpenSim;

//=============================================================================
//  MocoMinimumDistanceConstraintPair
//=============================================================================

MocoMinimumDistanceConstraintPair::MocoMinimumDistanceConstraintPair() {
    constructProperties();
}

MocoMinimumDistanceConstraintPair::MocoMinimumDistanceConstraintPair(
        std::string firstFramePath, std::string secondFramePath, 
        double minimum_distance) {
    constructProperties();
    set_first_frame_path(firstFramePath);
    set_second_frame_path(secondFramePath);
    set_minimum_distance(minimum_distance);
}

void MocoMinimumDistanceConstraintPair::constructProperties() {
    constructProperty_first_frame_path("");
    constructProperty_second_frame_path("");
    constructProperty_minimum_distance(0);
}

//=============================================================================
//  MocoMinimumDistanceConstraint
//=============================================================================

MocoMinimumDistanceConstraint::MocoMinimumDistanceConstraint(){ 
    constructProperties(); 
}

void MocoMinimumDistanceConstraint::initializeOnModelImpl(
        const Model& model, const MocoProblemInfo&) const {
    int nFramePairs = getProperty_frame_pairs().size();

    // TODO: setConstraintInfo() is not really intended for use here.
    MocoConstraintInfo info;
    std::vector<MocoBounds> bounds;
    for (int i = 0; i < nFramePairs; ++i) {
        const auto first_frame_path =
                get_frame_pairs(i).get_first_frame_path();
        OPENSIM_THROW_IF(!model.hasComponent<Frame>(first_frame_path),
                Exception,
                format("Could not find frame '%s'.", first_frame_path));
        auto& first_frame = model.getComponent<Frame>(first_frame_path);
        const auto second_frame_path = 
                get_frame_pairs(i).get_second_frame_path();
        OPENSIM_THROW_IF(!model.hasComponent<Frame>(second_frame_path),
                Exception,
                format("Could not find frame '%s'.", second_frame_path));
        auto& second_frame = model.getComponent<Frame>(second_frame_path);

        m_frame_pairs.emplace_back(&first_frame, &second_frame);

        bounds.emplace_back(
                get_frame_pairs(i).get_minimum_distance(), SimTK::Infinity);
    }

    setNumEquations(nFramePairs);
    info.setBounds(bounds);
    const_cast<MocoMinimumDistanceConstraint*>(this)->setConstraintInfo(info);
}

void MocoMinimumDistanceConstraint::calcPathConstraintErrorsImpl(
        const SimTK::State& state, SimTK::Vector& errors) const {
    getModel().realizePosition(state);
    int iconstr = 0;
    SimTK::Vec3 relative_position;
    for (const auto& frame_pair : m_frame_pairs) {
        const auto& first_frame_pos =
                frame_pair.first->getPositionInGround(state);
        const auto& second_frame_pos =
                frame_pair.second->getPositionInGround(state);
        relative_position = second_frame_pos - first_frame_pos;

        errors[iconstr++] = relative_position.norm();
    }
}

void MocoMinimumDistanceConstraint::constructProperties() { 
    constructProperty_frame_pairs(); 
}