/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoFrameOrientationConstraint.cpp                           *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2022 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Size Zheng                                                      *
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

#include "MocoFrameOrientationConstraint.h"

using namespace OpenSim;


MocoFrameOrientationConstraintPair::MocoFrameOrientationConstraintPair() {
    constructProperties();
}

MocoFrameOrientationConstraintPair::MocoFrameOrientationConstraintPair(
    std::string frame1Path, std::string frame2Path,
    double minimum_angle, double maximum_angle) {
    constructProperties();
    set_frame1_path(frame1Path);
    set_frame2_path(frame2Path);
    set_minimum_angle(minimum_angle);
    set_maximum_angle(maximum_angle);
}

void MocoFrameOrientationConstraintPair::constructProperties() {
    constructProperty_frame1_path("");
    constructProperty_frame2_path("");
    constructProperty_minimum_angle(-SimTK::Infinity);
    constructProperty_maximum_angle(SimTK::Infinity);
}

//=============================================================================
//  MocoFrameOrientationConstraint
//=============================================================================

MocoFrameOrientationConstraint::MocoFrameOrientationConstraint() 
{
    constructProperties();
}
void MocoFrameOrientationConstraint::initializeOnModelImpl(const Model& model, const MocoProblemInfo&) const
{

    int nFramePairs = getProperty_frame_pairs().size();
    MocoConstraintInfo info;
    std::vector<MocoBounds> bounds;

    for (int i = 0; i < nFramePairs; ++i) {
        const auto frame1_path = get_frame_pairs(i).get_frame1_path();
        OPENSIM_THROW_IF(!model.hasComponent<Frame>(frame1_path), Exception,
            "Could not find frame '{}'.", frame1_path);
        auto& frame1 = model.getComponent<Frame>(frame1_path);
        const auto frame2_path = get_frame_pairs(i).get_frame2_path();
        OPENSIM_THROW_IF(!model.hasComponent<Frame>(frame2_path), Exception,
            "Could not find frame '{}'.", frame2_path);
        auto& frame2 = model.getComponent<Frame>(frame2_path);
        m_frame_pairs.emplace_back(&frame1, &frame2);

        const double& minimum = get_frame_pairs(i).get_minimum_angle();
        const double& maximum = get_frame_pairs(i).get_maximum_angle();
        
        OPENSIM_THROW_IF(minimum > maximum, Exception,
            "Expected the minimum angle for this frame pair "
            "to be less than or equal to the maximum angle, "
            "but they are {} and {}, respectively.",
            minimum, maximum);
        bounds.emplace_back(minimum, maximum);
    }
  

    setNumEquations(nFramePairs);
    info.setBounds(bounds);
    const_cast<MocoFrameOrientationConstraint*>(this)->setConstraintInfo(info);

}

void MocoFrameOrientationConstraint::calcPathConstraintErrorsImpl(const SimTK::State& state, SimTK::Vector& errors) const
{
    int iconstr = 0;

    getModel().realizePosition(state);

    for (const auto& frame_pair : m_frame_pairs) {
        const auto& frame1_rotation = frame_pair.first->getRotationInGround(state);
        const auto& frame2_rotation = frame_pair.second->getRotationInGround(state);
        auto coordaxis = SimTK::CoordinateAxis(2);											
        auto frame1_angle = frame1_rotation.convertOneAxisRotationToOneAngle(coordaxis);	
        auto frame2_angle = frame2_rotation.convertOneAxisRotationToOneAngle(coordaxis);	
        auto relative_angle = frame2_angle - frame1_angle;
             
        errors[iconstr++] = relative_angle;
    }

}

void MocoFrameOrientationConstraint::constructProperties() {
    constructProperty_frame_pairs();
}
