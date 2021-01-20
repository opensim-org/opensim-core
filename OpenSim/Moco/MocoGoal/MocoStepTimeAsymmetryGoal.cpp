/* -------------------------------------------------------------------------- *
 * OpenSim: MocoStepTimeAsymmetryGoal.h                                       *
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

#include "MocoStepTimeAsymmetryGoal.h"
#include <OpenSim/Simulation/Model/SmoothSphereHalfSpaceForce.h>

using namespace OpenSim;

const std::set<std::string> MocoStepTimeAsymmetryGoal::m_directions{"positive-x",
        "positive-y", "positive-z", "negative-x", "negative-y", "negative-z"};

void MocoStepTimeAsymmetryGoal::constructProperties() {
    constructProperty_left_contact_force_paths();
    constructProperty_right_contact_force_paths();
    constructProperty_contact_force_direction("positive-y");
    constructProperty_walking_direction("positive-x");
    constructProperty_contact_force_threshold(25);
    constructProperty_left_foot_frame("calcn_l");
    constructProperty_right_foot_frame("calcn_r");
    constructProperty_smoothing(10);
    constructProperty_target_asymmetry(0);
}

void MocoStepTimeAsymmetryGoal::initializeOnModelImpl(const Model& model) const {

    // Get references to left and right foot contact spheres.
    for (int ic = 0; ic < getProperty_left_contact_force_paths().size(); ++ic) {
        const auto& path = get_left_contact_force_paths(ic);
        m_left_contacts.emplace_back(
                &model.getComponent<SmoothSphereHalfSpaceForce>(path));
    }
    for (int ic = 0; ic < getProperty_right_contact_force_paths().size(); ++ic) {
        const auto& path = get_right_contact_force_paths(ic);
        m_right_contacts.emplace_back(
                &model.getComponent<SmoothSphereHalfSpaceForce>(path));
    }

    // Get foot frames.
    m_left_frame = &model.getBodySet().get(get_left_foot_frame());
    m_right_frame = &model.getBodySet().get(get_right_foot_frame());

    // Check that properties contain acceptable values.
    checkPropertyValueIsInSet(getProperty_contact_force_direction(),
            m_directions);
    checkPropertyValueIsInSet(getProperty_walking_direction(),
            m_directions);

    auto assign = [](const std::string& direction, int& index, int& sign) {
        if (direction == "positive-x") {
            index = 0;
            sign = 1;
        } else if (direction == "positive-y") {
            index = 1;
            sign = 1;
        } else if (direction == "positive-z") {
            index = 2;
            sign = 1;
        } else if (direction == "negative-x") {
            index = 0;
            sign = -1;
        } else if (direction == "negative-y") {
            index = 1;
            sign = -1;
        } else if (direction == "negative-z") {
            index = 2;
            sign = -1;
        }
    };
    assign(get_contact_force_direction(),
            m_contact_force_index, m_contact_force_sign);
    assign(get_walking_direction(),
           m_walking_direction_index, m_walking_direction_sign);

    checkPropertyValueIsPositive(getProperty_contact_force_threshold());
    checkPropertyValueIsInRangeOrSet(
            getProperty_target_asymmetry(), -1.0, 1.0, {});
    checkPropertyValueIsPositive(getProperty_smoothing());

    // Define smoothing function.
    m_conditional = [](const double& cond, const double& shift,
                       const double& scale, const double& smoothing) {
        return shift + scale * tanh(smoothing * cond);
    };

    setRequirements(1, 1, SimTK::Stage::Velocity);
}

void MocoStepTimeAsymmetryGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    // Need to realize to Velocity for SmoothSphereHalfSpaceForce.
    const auto& state = input.state;
    getModel().realizeVelocity(state);
    integrand = 0;

    // Compute vertical force values from left and right foot contact spheres.
    double leftForce = 0;
    for (const auto& contact : m_left_contacts) {
        Array<double> recordValues = contact->getRecordValues(state);
        leftForce += m_contact_force_sign * recordValues[m_contact_force_index];
    }
    double rightForce = 0;
    for (const auto& contact : m_right_contacts) {
        Array<double> recordValues = contact->getRecordValues(state);
        rightForce += m_contact_force_sign * recordValues[m_contact_force_index];
    }

    // Right is negative such that shorter right step times give negative
    // asymmetry values.
    const double rightContactDetect = -m_conditional(
            rightForce - get_contact_force_threshold(), 0.5, 0.5,
            get_smoothing());
    const double leftContactDetect = m_conditional(
            leftForce - get_contact_force_threshold(), 0.5, 0.5,
            get_smoothing());

    // Now get the locations of each heel contact sphere, and calculate
    // which foot is in front of the other. This is necessary because of the
    // double support phase in walking: when both feet are on the ground, we
    // need to detect which foot is in front so that when the leading foot
    // strikes the ground, it begins a new step.
    double leftPos = m_walking_direction_sign * m_left_frame->
            getPositionInGround(state)[m_walking_direction_index];
    double rightPos = m_walking_direction_sign * m_right_frame->
            getPositionInGround(state)[m_walking_direction_index];

    // This number will be -1 when left foot is in front, the +1 when right
    // foot is in front.
    double frontFoot = m_conditional(rightPos - leftPos, 0, 1, get_smoothing());

    // Use this "tiebreaker" variable to detect which foot is leading when both
    // feet are on the ground.
    // Double Support, left foot in front (left heel strike):
    // tieBreaker = 1 * 1 * -1 = -1
    // Double Support, right foot in front (right heel strike):
    // tieBreaker = 1 * 1 * 1 = 1;
    // During single support, TieBreaker = 0
    // TieBreaker = 1 * 0 * 1 = 0;
    double tieBreaker = leftContactDetect * rightContactDetect * frontFoot;
    double leftStepTime = leftContactDetect + tieBreaker;
    double rightStepTime = rightContactDetect + tieBreaker;

    integrand = m_conditional(leftStepTime + rightStepTime, 0, 1,
                              get_smoothing());
}

void MocoStepTimeAsymmetryGoal::calcGoalImpl(const GoalInput& input,
                                             SimTK::Vector& cost) const {

    // I believe the denominator (=2 here) is dependent on the number of nodes
    // NOTE: we discussed either being able to pass a value in to this function
    // based on the user's identification of their collocation scheme, or that
    // this value =1, and the user needs to by-hand calculate what the target
    // Asymmetry should be given their collocation scheme.
    cost[0] = 0.5 * input.integral - get_target_asymmetry();
}

//void MocoStepTimeAsymmetryGoal::printDescriptionImpl() const {
//    for (int ig = 0; ig < getProperty_contact_groups().size(); ++ig) {
//        const auto& group = get_contact_groups(ig);
//        log_cout("        group {}: ExternalForce: {}",
//                 ig, group.get_external_force_name());
//        log_cout("            forces:");
//        for (int ic = 0; ic < group.getProperty_contact_force_paths().size();
//             ++ic) {
//            log_cout("                {}", group.get_contact_force_paths(ic));
//        }
//    }
//}