/* -------------------------------------------------------------------------- *
 * OpenSim: MocoStepLengthAsymmetryGoal.h                                     *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Russell Johnson, Nicholas Bianco                                *
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

#include "MocoStepLengthAsymmetryGoal.h"
#include <OpenSim/Simulation/Model/SmoothSphereHalfSpaceForce.h>

using namespace OpenSim;

void MocoStepLengthAsymmetryGoal::constructProperties() {
    constructProperty_left_foot_frame("");
    constructProperty_right_foot_frame("");
    constructProperty_walking_direction("positive-x");
    constructProperty_smoothing(500);
    constructProperty_target_asymmetry(0.0);
    constructProperty_stride_length(1.0);
}

void MocoStepLengthAsymmetryGoal::initializeOnModelImpl(const Model& model) const {

    m_left_foot_frame = model.getComponent<PhysicalFrame>(get_left_foot_frame());
    m_right_foot_frame = model.getComponent<PhysicalFrame>(get_right_foot_frame());

    // Check that properties contain acceptable values.
    std::set<std::string> directions{"positive-x", "positive-y", "positive-z",
                                     "negative-x", "negative-y", "negative-z"};
    checkPropertyValueIsInSet(getProperty_walking_direction(), directions);
    checkPropertyValueIsInRangeOrSet(getProperty_target_asymmetry(),
                                     -1.0, 1.0, {});
    checkPropertyValueIsPositive(getProperty_smoothing());
    checkPropertyValueIsPositive(getProperty_stride_length());

    // Compute target foot positions based on properties.
    m_right_foot_threshold =
            0.5 * (1 + get_target_asymmetry()) * get_stride_length();
    m_left_foot_threshold =
            0.5 * (1 - get_target_asymmetry()) * get_stride_length();

    // Assign the indices and signs for the contact force direction and walking
    // motion direction.
    auto setIndexAndSign = [](const std::string& direction, int& index,
                              int& sign) {
        if      (direction == "positive-x") { index = 0; sign = 1; }
        else if (direction == "positive-y") { index = 1; sign = 1; }
        else if (direction == "positive-z") { index = 2; sign = 1; }
        else if (direction == "negative-x") { index = 0; sign = -1; }
        else if (direction == "negative-y") { index = 1; sign = -1; }
        else if (direction == "negative-z") { index = 2; sign = -1; }
    };
    setIndexAndSign(get_walking_direction(), m_walking_direction_index,
                    m_walking_direction_sign);

    // Define the smoothing function.
    m_conditional = [](const double& cond, const double& shift,
                       const double& scale, const double& smoothing) {
        return shift + scale * tanh(smoothing * cond);
    };

    // Set the goal requirements.
    setRequirements(1, 1); // TODO: stage dependency velocity issue
}

void MocoStepLengthAsymmetryGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    const auto& state = input.state;
    getModel().realizeVelocity(state);
    integrand = 0;

    const double rightFootPosition =
            m_walking_direction_sign * m_right_foot_frame->
                    getPositionInGround(state)[m_walking_direction_index];
    const double leftFootPosition =
            m_walking_direction_sign * m_left_foot_frame->
                    getPositionInGround(state)[m_walking_direction_index];

    const double rightFootDiff = rightFootPosition - leftFootPosition;
    const double leftFootDiff = leftFootPosition - rightFootPosition;

    const double rightFootAsymmetry = m_conditional(
            m_right_foot_threshold - rightFootDiff, 0.5, -0.5,
            get_smoothing());
    const double leftFootAsymmetry = m_conditional(
            m_left_foot_threshold - leftFootDiff, 0.5, -0.5,
            get_smoothing());

    integrand = rightFootAsymmetry + leftFootAsymmetry;
}

void MocoStepLengthAsymmetryGoal::calcGoalImpl(const GoalInput& input,
        SimTK::Vector& cost) const {
    cost[0] = input.integral;
}