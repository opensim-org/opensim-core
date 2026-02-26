/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoGoal.cpp                                                 *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
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

#include "MocoGoal.h"

#include <OpenSim/Moco/Components/ControlDistributor.h>

using namespace OpenSim;

MocoGoal::MocoGoal() {
    constructProperties();
    if (getName().empty()) setName("goal");
}

MocoGoal::MocoGoal(std::string name) {
    setName(std::move(name));
    constructProperties();
}

MocoGoal::MocoGoal(std::string name, double weight)
        : MocoGoal(std::move(name)) {
    set_weight(weight);
}


void MocoGoal::printDescription() const {
    const auto mode = getModeAsString();
    std::string str = fmt::format("  {}. {}, enabled: {}, mode: {}",
            getName(), getConcreteClassName(), get_enabled(), mode);
    if (mode == "cost") {
        str += fmt::format(", weight: {}", get_weight());
    }
    log_info(str);
    printDescriptionImpl();
}

double MocoGoal::calcSystemDisplacement(const GoalInput& input) const {
    // Default behaviour: full 3D Euclidean norm of CoM displacement.
    return calcSystemDisplacement(input, -1);
}

double MocoGoal::calcSystemDisplacement(const GoalInput& input,
        int axisComponent) const {
    const SimTK::Vec3 comInitial =
            getModel().calcMassCenterPosition(input.initial_state);
    const SimTK::Vec3 comFinal =
            getModel().calcMassCenterPosition(input.final_state);
    const SimTK::Vec3 delta = comFinal - comInitial;
    if (axisComponent == -1) { return delta.norm(); }
    return delta[axisComponent];
}

double MocoGoal::calcDuration(const GoalInput& input) const {
    return input.final_time - input.initial_time;
}

double MocoGoal::calcSystemMass(const GoalInput& input) const {
    return getModel().getTotalMass(input.initial_state);
}

std::unordered_map<std::string, int> MocoGoal::getInputControlIndexMap() const {
    OPENSIM_ASSERT(m_control_distributor != nullptr);

    // Get the full Input control index map from the ControlDistributor.
    auto map = m_control_distributor->getControlIndexMap();

    // Get all possible control names from the model.
    auto controlNames = createControlNamesFromModel(getModel());

    // Remove the control names that are associated with the model's
    // ActuatorInputController.
    for (const auto& controlName : controlNames) {
        map.erase(controlName);
    }
    return map;
}

const SimTK::Vector& MocoGoal::getInputControls(
        const SimTK::State& state) const {
    OPENSIM_ASSERT(m_control_distributor != nullptr);
    return m_control_distributor->getControls(state);
}

void MocoGoal::constructProperties() {
    constructProperty_enabled(true);
    constructProperty_weight(1);
    constructProperty_mode();
    constructProperty_MocoConstraintInfo(MocoConstraintInfo());
    constructProperty_scale_factors();
    constructProperty_divide_by_displacement(false);
    constructProperty_divide_by_duration(false);
    constructProperty_divide_by_mass(false);
}
