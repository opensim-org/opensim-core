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

void MocoStepTimeAsymmetryGoal::constructProperties() {
    constructProperty_left_contact_force_paths();
    constructProperty_right_contact_force_paths();
    constructProperty_vertical_force_index(1);
    constructProperty_forward_direction_index(0);
    constructProperty_foot_strike_threshold(25);
    constructProperty_left_foot_frame("");
    constructProperty_right_foot_frame("");
    constructProperty_smoothing(10);
    constructProperty_target_asymmetry(0);
}

void MocoStepTimeAsymmetryGoal::initializeOnModelImpl(const Model& model) const {

    for (int ic = 0; ic < getProperty_left_contact_force_paths().size(); ++ic) {
        const auto& path = get_left_contact_force_paths(ic);
        const auto& contactForce =
                model.getComponent<SmoothSphereHalfSpaceForce>(path);

        m_left_contacts.emplace_back(&contactForce);
    }

    for (int ic = 0; ic < getProperty_right_contact_force_paths().size(); ++ic) {
        const auto& path = get_right_contact_force_paths(ic);
        const auto& contactForce =
                model.getComponent<SmoothSphereHalfSpaceForce>(path);

        m_right_contacts.emplace_back(&contactForce);
    }

    m_left_frame = &model.getBodySet().get(get_left_foot_frame());
    m_right_frame = &model.getBodySet().get(get_right_foot_frame());

    // Check that properties contain acceptable values.
    checkPropertyValueIsInRangeOrSet(
            getProperty_vertical_force_index(), 0, 2, {});
    checkPropertyValueIsInRangeOrSet(
            getProperty_forward_direction_index(), 0, 2, {});
    checkPropertyValueIsPositive(getProperty_foot_strike_threshold());
    // TODO what should be the range for target asymmetry?
    checkPropertyValueIsInRangeOrSet(
            getProperty_target_asymmetry(), -1.0, 1.0, {});
    checkPropertyValueIsPositive(getProperty_smoothing());

    m_conditional = [](const double& cond, const double& shift,
                       const double& scale, const double& smoothing) {
        return shift + scale * tanh(smoothing * cond);
    };

    setRequirements(1, 1, SimTK::Stage::Velocity);
}

void MocoStepTimeAsymmetryGoal::calcIntegrandImpl(
        const IntegrandInput& input, double& integrand) const {
    const auto& state = input.state;
    const auto& time = state.getTime();
    getModel().realizeVelocity(state);
    SimTK::Vector timeVec(1, time);
    integrand = 0;

    // Compute vertical force values from left and right foot contact spheres.
    double leftForce = 0;
    for (const auto& contact : m_left_contacts) {
        Array<double> recordValues = contact->getRecordValues(state);
        leftForce += recordValues[get_vertical_force_index()];
    }
    double rightForce = 0;
    for (const auto& contact : m_right_contacts) {
        Array<double> recordValues = contact->getRecordValues(state);
        rightForce += recordValues[get_vertical_force_index()];
    }

    // Right is negative such that shorter right step times give negative
    // asymmetry values.
    const double rightContactDetect = -m_conditional(
            rightForce - get_foot_strike_threshold(), 0.5, 0.5, get_smoothing());
    const double leftContactDetect = m_conditional(
            leftForce - get_foot_strike_threshold(), 0.5, 0.5, get_smoothing());

    // Now get the locations of each heel contact sphere, and calculate
    // which foot is in front of the other: This is necessary because of the
    // double support phase in walking: when both feet are on the ground, we
    // need to detect which foot is in front so that when the leading foot
    // strikes the ground, it begins a new step (aka begin a step at foot
    // strike).

    // Similar to above, this relies on the naming convention of the heel
    // contact spheres, maybe specify this in the main Moco script and pass
    // in? Also, I guess, relies on the naming convention of the calcaneous,
    // but maybe that's a safer assumption
    double leftPos = m_left_frame->
            getPositionInGround(state)[get_forward_direction_index()];
    double rightPos = m_right_frame->
            getPositionInGround(state)[get_forward_direction_index()];

    // This number will be -1 when left foot is in front, the +1 when right
    // foot is in front.
    double frontFoot = m_conditional(rightPos - leftPos, 0, 1, get_smoothing());

    // Then, use this "tiebreaker" variable to detect -- WHEN both feet are
    // on the ground, which foot is in front?
    // EXAMPLE: Double Support, left foot in front (left heel strike)
    // TieBreaker = 1 * 1 * -1 = -1;
    // EXAMPLE: Double Support, right foot in front (right heel strike)
    // TieBreaker = 1 * 1 * 1 = 1;
    // During single support, TieBreaker = 0
    // TieBreaker = 1 * 0 * 1 = 0;
    double tieBreaker = leftContactDetect * rightContactDetect * frontFoot;

    double leftStepTime = leftContactDetect + tieBreaker;
    double rightStepTime = rightContactDetect + tieBreaker;

    // Since this is a single term for a single node, (either 1 or -1) there
    // isn't anything to integrate or sum across here.
    integrand = m_conditional(leftStepTime + rightStepTime, 0, 1, get_smoothing());
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