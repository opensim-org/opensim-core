/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoFrameDistanceConstraint.cpp                              *
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

#include "MocoFrameDistanceConstraint.h"

using namespace OpenSim;

//=============================================================================
//  MocoFrameDistanceConstraintPair
//=============================================================================

MocoFrameDistanceConstraintPair::MocoFrameDistanceConstraintPair() {
    constructProperties();
}

MocoFrameDistanceConstraintPair::MocoFrameDistanceConstraintPair(
        std::string frame1Path, std::string frame2Path, 
        double minimum_distance, double maximum_distance) {
    constructProperties();
    set_frame1_path(frame1Path);
    set_frame2_path(frame2Path);
    set_minimum_distance(minimum_distance);
    set_maximum_distance(maximum_distance);
}

void MocoFrameDistanceConstraintPair::constructProperties() {
    constructProperty_frame1_path("");
    constructProperty_frame2_path("");
    constructProperty_minimum_distance(0);
    constructProperty_maximum_distance(SimTK::Infinity);
}

//=============================================================================
//  MocoFrameDistanceConstraint
//=============================================================================

MocoFrameDistanceConstraint::MocoFrameDistanceConstraint(){ 
    constructProperties(); 
}

void MocoFrameDistanceConstraint::initializeOnModelImpl(
        const Model& model, const MocoProblemInfo&) const {
    int nFramePairs = getProperty_frame_pairs().size();

    // TODO: setConstraintInfo() is not really intended for use here.
    MocoConstraintInfo info;
    std::vector<MocoBounds> bounds;
    for (int i = 0; i < nFramePairs; ++i) {
        const auto frame1_path = get_frame_pairs(i).get_frame1_path();
        OPENSIM_THROW_IF(!model.hasComponent<Frame>(frame1_path), Exception,
                format("Could not find frame '%s'.", frame1_path));
        auto& frame1 = model.getComponent<Frame>(frame1_path);
        const auto frame2_path = get_frame_pairs(i).get_frame2_path();
        OPENSIM_THROW_IF(!model.hasComponent<Frame>(frame2_path), Exception,
                format("Could not find frame '%s'.", frame2_path));
        auto& frame2 = model.getComponent<Frame>(frame2_path);
        m_frame_pairs.emplace_back(&frame1, &frame2);

        const double& minimum = get_frame_pairs(i).get_minimum_distance();
        const double& maximum = get_frame_pairs(i).get_maximum_distance();
        OPENSIM_THROW_IF(minimum < 0, Exception, 
                format("Expected the minimum distance for this frame pair to " 
                    "non-negative, but it is %d.", minimum));
        OPENSIM_THROW_IF(maximum < 0, Exception,
                format("Expected the maximum distance for this frame pair to "
                       "non-negative, but it is %d.", maximum));
        OPENSIM_THROW_IF(minimum > maximum, Exception,
                format("Expected the minimum distance for this frame pair to "
                       "be less than or equal to the maximum distance, but "
                       "they are %d and %d, respectively.", minimum, maximum));
        bounds.emplace_back(SimTK::square(minimum), SimTK::square(maximum));
    }

    setNumEquations(nFramePairs);
    info.setBounds(bounds);
    const_cast<MocoFrameDistanceConstraint*>(this)->setConstraintInfo(info);
}

void MocoFrameDistanceConstraint::calcPathConstraintErrorsImpl(
        const SimTK::State& state, SimTK::Vector& errors) const {
    getModel().realizePosition(state);
    int iconstr = 0;
    SimTK::Vec3 relative_position;
    for (const auto& frame_pair : m_frame_pairs) {
        const auto& frame1_pos = frame_pair.first->getPositionInGround(state);
        const auto& frame2_pos = frame_pair.second->getPositionInGround(state);
        relative_position = frame2_pos - frame1_pos;
        errors[iconstr++] = relative_position.normSqr();
    }
}

void MocoFrameDistanceConstraint::constructProperties() { 
    constructProperty_frame_pairs(); 
}