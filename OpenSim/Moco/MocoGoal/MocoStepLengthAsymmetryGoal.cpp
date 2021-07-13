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
    constructProperty_asymmetry_smoothing(5);
    constructProperty_target_asymmetry(0.0);
    constructProperty_stride_length(-1);
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
    checkPropertyValueIsPositive(getProperty_asymmetry_smoothing());
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
    setRequirements(1, 1, SimTK::Stage::Position);
}

void MocoStepLengthAsymmetryGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    const auto& state = input.state;
    getModel().realizePosition(state);
    integrand = 0;

    // Find the position of the right foot frame.
    const double rightFootPosition =
            m_walking_direction_sign * m_right_foot_frame->
                    getPositionInGround(state)[m_walking_direction_index];
    // Find the position of the left foot frame.
    const double leftFootPosition =
            m_walking_direction_sign * m_left_foot_frame->
                    getPositionInGround(state)[m_walking_direction_index];

    // Find the distance between the left foot and the right foot and vice versa
    // to compute the right and left foot step lengths. Since these step length
    // have separate limits when computing the asymmetry below, we compute them
    // separately here.
    const double rightFootStepLength = rightFootPosition - leftFootPosition;
    const double leftFootStepLength = leftFootPosition - rightFootPosition;

    // Compute if the right foot or left foot step lengths exceed their
    // respective thresholds. If so, count this time point towards the
    // asymmetry error (for each foot independently) via the smoothing functions.
    // The user input smoothing value is scaled by 100 here to produce a more
    // noticeable effect on the asymmetry error.
    const double rightFootAsymmetry = m_conditional(
            m_right_foot_threshold - rightFootStepLength, 0.5, -0.5,
            100.0 * get_asymmetry_smoothing());
    const double leftFootAsymmetry = m_conditional(
            m_left_foot_threshold - leftFootStepLength, 0.5, -0.5,
            100.0 * get_asymmetry_smoothing());

    // Compute the total asymmetry error at this time step.
    integrand = rightFootAsymmetry + leftFootAsymmetry;
}

void MocoStepLengthAsymmetryGoal::calcGoalImpl(const GoalInput& input,
        SimTK::Vector& cost) const {
    // Multiplying by 100 scales the cost to more reasonable values. Without it,
    // users would need to provide large cost weights, which may be unintuitive.
    cost[0] = 100.0 * input.integral;
}

void MocoStepLengthAsymmetryGoal::printDescriptionImpl() const {
    log_cout("            target asymmetry: ", get_target_asymmetry());
    log_cout("            left foot frame: ", get_left_foot_frame());
    log_cout("            right foot frame: ", get_right_foot_frame());
}