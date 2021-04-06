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
    constructProperty_foot_velocity_threshold(0.05);
    constructProperty_walking_direction("positive-x");
    constructProperty_smoothing(500);
    constructProperty_target_asymmetry(0.0);
    constructProperty_stride_length(1.0);
    constructProperty_initial_right_foot_position(0.0);
}

void MocoStepLengthAsymmetryGoal::initializeOnModelImpl(const Model& model) const {

    m_left_foot_frame = model.getComponent<PhysicalFrame>(get_left_foot_frame());
    m_right_foot_frame = model.getComponent<PhysicalFrame>(get_right_foot_frame());

    // Check that properties contain acceptable values.
    std::set<std::string> directions{"positive-x", "positive-y", "positive-z",
                                     "negative-x", "negative-y", "negative-z"};
    checkPropertyValueIsInSet(getProperty_walking_direction(), directions);
    checkPropertyValueIsPositive(getProperty_foot_velocity_threshold());
    checkPropertyValueIsInRangeOrSet(getProperty_target_asymmetry(),
                                     -1.0, 1.0, {});
    checkPropertyValueIsPositive(getProperty_smoothing());
    checkPropertyValueIsPositive(getProperty_stride_length());

    // Compute target foot positions based on properties.
    m_left_foot_position =
            0.5 * get_stride_length() * (1.0 - get_target_asymmetry())
                + get_initial_right_foot_position();
    m_right_foot_position = get_initial_right_foot_position();
    m_final_right_foot_position =
            get_stride_length() + get_initial_right_foot_position();

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
    const double rightVelocity =
            m_walking_direction_sign * m_right_foot_frame->
                getVelocityInGround(state)[1][m_walking_direction_index];
    const double leftVelocity =
            m_walking_direction_sign * m_left_foot_frame->
                getVelocityInGround(state)[1][m_walking_direction_index];

    // Right is negative such that shorter right step times give negative
    // asymmetry values.
    const double rightContactDetect = m_conditional(
            rightVelocity - get_foot_velocity_threshold(), 0.5, -0.5,
            get_smoothing());
    const double leftContactDetect = m_conditional(
            leftVelocity - get_foot_velocity_threshold(), 0.5, -0.5,
            get_smoothing());

//    const double rightFootNearTarget = -m_conditional(
//            rightFootPosition - m_left_foot_position, 0.5, 0.5,
//            get_smoothing());
//    const double leftFootNearTarget = -m_conditional(
//            leftFootPosition - m_right_foot_position, 0.5, 0.5,
//            get_smoothing());

    const double rightFootError = m_right_foot_position - rightFootPosition;
    const double leftFootError = m_left_foot_position - leftFootPosition;
    const double rightFootAsymmetry =
            rightContactDetect * rightFootError * rightFootError;
    const double leftFootAsymmetry =
            leftContactDetect * leftFootError * leftFootError;

    integrand = rightFootAsymmetry + leftFootAsymmetry;
}

void MocoStepLengthAsymmetryGoal::calcGoalImpl(const GoalInput& input,
        SimTK::Vector& cost) const {
    const double rightFootFinalPosition =
            m_walking_direction_sign * m_right_foot_frame->
                    getPositionInGround(input.final_state)[
                            m_walking_direction_index];

    const double rightFootError = rightFootFinalPosition -
            m_final_right_foot_position;

    cost[0] = input.integral + rightFootError;
    if (getModeIsCost()) {
        cost[0] *= cost[0];
    }
}
