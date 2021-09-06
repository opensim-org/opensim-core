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

MocoStepTimeAsymmetryGoalGroup::MocoStepTimeAsymmetryGoalGroup() {
    constructProperties();
}

MocoStepTimeAsymmetryGoalGroup::MocoStepTimeAsymmetryGoalGroup(
        const std::vector<std::string>& contactForcePaths,
        const std::string& footPositionForcePath) {
    constructProperties();
    for (const auto& path : contactForcePaths) {
        append_contact_force_paths(path);
    }
    set_foot_position_contact_force_path(footPositionForcePath);
}

void MocoStepTimeAsymmetryGoalGroup::constructProperties() {
    constructProperty_contact_force_paths();
    constructProperty_foot_position_contact_force_path("");
}

void MocoStepTimeAsymmetryGoal::constructProperties() {
    constructProperty_left_contact_group(MocoStepTimeAsymmetryGoalGroup());
    constructProperty_right_contact_group(MocoStepTimeAsymmetryGoalGroup());
    constructProperty_target_asymmetry(0);
    constructProperty_contact_force_threshold(25);
    constructProperty_contact_force_direction("positive-y");
    constructProperty_walking_direction("positive-x");
    constructProperty_contact_detection_smoothing(0.25);
    constructProperty_asymmetry_smoothing(10);
}

void MocoStepTimeAsymmetryGoal::initializeOnModelImpl(const Model& model) const {

    // Get references to left and right foot contact spheres.
    const std::string& leftPositionForcePath =
            get_left_contact_group().get_foot_position_contact_force_path();
    const auto& numLeftPaths =
            get_left_contact_group().getProperty_contact_force_paths().size();
    OPENSIM_THROW_IF(leftPositionForcePath.empty(), Exception,
            "Expected a 'foot_position_contact_force_path' for the left contact "
            "group, but it was not provided.");
    OPENSIM_THROW_IF(numLeftPaths == 0, Exception,
            "Expected 'contact_force_paths' to be provided for the left contact "
            "group, but none were found.");
    for (int ic = 0; ic < numLeftPaths; ++ic) {
        const auto& path = get_left_contact_group().get_contact_force_paths(ic);
        const auto& contactForce =
                model.getComponent<SmoothSphereHalfSpaceForce>(path);
        m_left_contacts.emplace_back(&contactForce);

        if (path == leftPositionForcePath) {
            m_left_frame = &contactForce.getConnectee<ContactSphere>("sphere")
                    .getConnectee<PhysicalFrame>("frame").findBaseFrame();
        }
    }
    OPENSIM_THROW_IF(m_left_frame.empty(), Exception,
           "Expected the 'foot_position_contact_force_path' property to match "
           "one of the paths in 'contact_force_paths', but '{}' was not found.",
           leftPositionForcePath);

    const std::string& rightPositionForcePath =
            get_right_contact_group().get_foot_position_contact_force_path();
    const auto& numRightPaths =
            get_right_contact_group().getProperty_contact_force_paths().size();
    OPENSIM_THROW_IF(rightPositionForcePath.empty(), Exception,
            "Expected a 'foot_position_contact_force_path' for the right "
            "contact group, but it not was provided.");
    OPENSIM_THROW_IF(numRightPaths == 0, Exception,
            "Expected 'contact_force_paths' to be provided for the right "
            "contact group, but none were found.");
    for (int ic = 0; ic < numRightPaths; ++ic) {
        const auto& path = get_right_contact_group().get_contact_force_paths(ic);
        const auto& contactForce =
                model.getComponent<SmoothSphereHalfSpaceForce>(path);
        m_right_contacts.emplace_back(&contactForce);
        if (path == rightPositionForcePath) {
            m_right_frame = &contactForce.getConnectee<ContactSphere>("sphere")
                    .getConnectee<PhysicalFrame>("frame").findBaseFrame();
        }
    }
    OPENSIM_THROW_IF(m_right_frame.empty(), Exception,
         "Expected the 'foot_position_contact_force_path' property to match "
         "one of the paths in 'contact_force_paths', but '{}' was not found.",
         rightPositionForcePath);

    // Check that properties contain acceptable values.
    std::set<std::string> directions{"positive-x", "positive-y", "positive-z",
                                     "negative-x", "negative-y", "negative-z"};
    checkPropertyValueIsInSet(getProperty_contact_force_direction(), directions);
    checkPropertyValueIsInSet(getProperty_walking_direction(), directions);
    checkPropertyValueIsPositive(getProperty_contact_force_threshold());
    checkPropertyValueIsInRangeOrSet(getProperty_target_asymmetry(),
            -1.0, 1.0, {});
    checkPropertyValueIsPositive(getProperty_asymmetry_smoothing());
    checkPropertyValueIsPositive(getProperty_contact_detection_smoothing());

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
    setIndexAndSign(get_contact_force_direction(), m_contact_force_index,
           m_contact_force_sign);
    setIndexAndSign(get_walking_direction(), m_walking_direction_index,
           m_walking_direction_sign);

    // Define the smoothing function.
    m_conditional = [](const double& cond, const double& shift,
                       const double& scale, const double& smoothing) {
        return shift + scale * tanh(smoothing * cond);
    };

    // Set the goal requirements.
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
            get_contact_detection_smoothing());
    const double leftContactDetect = m_conditional(
            leftForce - get_contact_force_threshold(), 0.5, 0.5,
            get_contact_detection_smoothing());

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
    double frontFoot = m_conditional(rightPos - leftPos, 0, 1, 100);

    // During double support, leftContactDetect will equal -1 and
    // rightContactDetect will equal 1. To avoid these values canceling each
    // other out when computing the integrand, we use this "tie-breaker" variable
    // to detect which foot is leading during double support. The tie-breaker
    // ensures that only the leading foot will have the current time point
    // counted towards its step time, which is true when calculating step time
    // asymmetry. Examples:
    //
    // Double Support, left foot in front (left heel strike)
    // tieBreaker = 1 * -1 * -1 = 1
    // leftStepTime = 1 + 1 = 2
    // rightStepTime = -1 + 1 = 0
    // leftStepTime + rightStepTime > 0 (count towards left step time)
    //
    // Double Support, right foot in front (right heel strike):
    // tieBreaker = 1 * -1 * 1 = -1
    // leftStepTime = 1 - 1 = 0
    // rightStepTime = -1 - 1 = -2
    // leftStepTime + rightStepTime < 0 (count towards right step time)
    //
    // During single support, tieBreaker = 0
    // tieBreaker = 1 * 0 * 1 = 0
    double tieBreaker = leftContactDetect * rightContactDetect * frontFoot;
    double leftStepTime = leftContactDetect + tieBreaker;
    double rightStepTime = rightContactDetect + tieBreaker;

    integrand = m_conditional(leftStepTime + rightStepTime, 0, 1,
                              get_asymmetry_smoothing());
}

void MocoStepTimeAsymmetryGoal::calcGoalImpl(const GoalInput& input,
                                             SimTK::Vector& cost) const {

    SimTK::Real timeInitial = input.initial_state.getTime();
    SimTK::Real timeFinal = input.final_state.getTime();
    SimTK::Real duration = timeFinal - timeInitial;
    SimTK::Real asymmetry = input.integral / duration;
    double error = asymmetry - get_target_asymmetry();
    // Multiplying by 100 scales the cost to more reasonable values. Without it,
    // users would need to provide large cost weights, which may be unintuitive.
    cost[0] = 100.0 * error * error;
}

void MocoStepTimeAsymmetryGoal::printDescriptionImpl() const {
    log_cout("            target asymmetry: ", get_target_asymmetry());
    const auto& leftGroup = get_left_contact_group();
    log_cout("            left forces:");
    for (int ic = 0; ic < leftGroup.getProperty_contact_force_paths().size();
         ++ic) {
        log_cout("                {}", leftGroup.get_contact_force_paths(ic));
    }
    log_cout("            left contact sphere for position: ",
             leftGroup.get_foot_position_contact_force_path());

    const auto& rightGroup = get_left_contact_group();
    log_cout("            right forces:");
    for (int ic = 0; ic < leftGroup.getProperty_contact_force_paths().size();
         ++ic) {
        log_cout("                {}", rightGroup.get_contact_force_paths(ic));
    }
    log_cout("            right contact sphere for position: ",
             rightGroup.get_foot_position_contact_force_path());
}