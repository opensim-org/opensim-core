/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoPeriodicityGoal.cpp                                      *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017-19 Stanford University and the Authors                  *
 *                                                                            *
 * Author(s): Antoine Falisse                                                 *
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

#include "MocoPeriodicityGoal.h"

#include <OpenSim/Simulation/SimulationUtilities.h>
#include <OpenSim/Moco/Components/ActuatorInputController.h>

using namespace OpenSim;

//=============================================================================
//  MocoPeriodicityGoalPair
//=============================================================================

MocoPeriodicityGoalPair::MocoPeriodicityGoalPair() { constructProperties(); }

MocoPeriodicityGoalPair::MocoPeriodicityGoalPair(
        std::string initialVariable, std::string finalVariable) {
    constructProperties();
    set_initial_variable(initialVariable);
    set_final_variable(finalVariable);
}

MocoPeriodicityGoalPair::MocoPeriodicityGoalPair(
        std::string initialVariableIsFinalVariable) {
    constructProperties();
    set_initial_variable(initialVariableIsFinalVariable);
    set_final_variable(initialVariableIsFinalVariable);
}

void MocoPeriodicityGoalPair::constructProperties() {
    constructProperty_initial_variable("");
    constructProperty_final_variable("");
    constructProperty_negate(false);
}

//=============================================================================
//  MocoPeriodicityGoal
//=============================================================================

MocoPeriodicityGoal::MocoPeriodicityGoal() { constructProperties(); }

void MocoPeriodicityGoal::constructProperties() {
    constructProperty_state_pairs();
    constructProperty_control_pairs();
}

void MocoPeriodicityGoal::initializeOnModelImpl(const Model& model) const {

    auto allSysYIndices = createSystemYIndexMap(model);
    int nStatePairs = getProperty_state_pairs().size();

    for (int i = 0; i < nStatePairs; ++i) {
        const auto path1 = get_state_pairs(i).get_initial_variable();
        OPENSIM_THROW_IF(allSysYIndices.count(path1) == 0, Exception,
                "Could not find state '{}'.", path1);
        const auto path2 = get_state_pairs(i).get_final_variable();
        OPENSIM_THROW_IF(allSysYIndices.count(path2) == 0, Exception,
                "Could not find state '{}'.", path2);
        int stateIndex1 = allSysYIndices[path1];
        int stateIndex2 = allSysYIndices[path2];
        m_state_names.emplace_back(path1, path2);
        m_indices_states.emplace_back(stateIndex1, stateIndex2,
                get_state_pairs(i).get_negate() ? -1 : 1);
    }

    auto systemControlIndexMap = createSystemControlIndexMap(model);
    auto inputControlIndexMap = getInputControlIndexMap();

    // Get controls associated with the model's ActuatorInputController.
    auto actuatorInputControls =
            createControlNamesForControllerType<ActuatorInputController>(model);

    int nControlPairs = getProperty_control_pairs().size();
    for (int i = 0; i < nControlPairs; ++i) {
        const auto path1 = get_control_pairs(i).get_initial_variable();
        bool foundControl1 = systemControlIndexMap.count(path1);
        bool foundInputControl1 = inputControlIndexMap.count(path1);
        OPENSIM_THROW_IF(!foundControl1 && !foundInputControl1, Exception,
                "Could not find control or Input control variable "
                "matching name '{}'.", path1);

        const auto path2 = get_control_pairs(i).get_final_variable();
        bool foundControl2 = systemControlIndexMap.count(path2);
        bool foundInputControl2 = inputControlIndexMap.count(path2);
        OPENSIM_THROW_IF(!foundControl2 && !foundInputControl2, Exception,
                "Could not find control or Input control variable "
                "matching name '{}'.", path2);

        int controlIndex1 = foundInputControl1 ? 
                inputControlIndexMap[path1] : systemControlIndexMap[path1];
        int controlIndex2 = foundInputControl2 ? 
                inputControlIndexMap[path2] : systemControlIndexMap[path2];
        m_control_names.emplace_back(path1, path2);
        m_indices_controls.emplace_back(controlIndex1, controlIndex2,
                get_control_pairs(i).get_negate() ? -1 : 1);
        m_isInputControl.emplace_back(
                std::make_pair(foundInputControl1, foundInputControl2));
    }

    setRequirements(
            0, (int)m_indices_states.size() + (int)m_indices_controls.size(),
            SimTK::Stage::Time);
}

void MocoPeriodicityGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& goal) const {

    const auto& initialStates = input.initial_state.getY();
    const auto& finalStates = input.final_state.getY();
    int i = 0;
    for (const auto& index_state : m_indices_states) {
        goal[i] = (initialStates[std::get<0>(index_state)] *
                          std::get<2>(index_state)) -
                  finalStates[std::get<1>(index_state)];
        if (getModeIsCost()) { goal[i] = SimTK::square(goal[i]); }
        ++i;
    }

    const auto& initialControls = input.initial_controls;
    const auto& finalControls = input.final_controls;
    const auto& initialInputControls = getInputControls(input.initial_state);
    const auto& finalInputControls = getInputControls(input.final_state);
    int j = 0;
    for (const auto& index_control : m_indices_controls) {
        int initialIndex = std::get<0>(index_control);
        const auto& initialControl = m_isInputControl[j].first ? 
                initialInputControls[initialIndex] : 
                initialControls[initialIndex];

        int finalIndex = std::get<1>(index_control);
        const auto& finalControl = m_isInputControl[j].second ? 
                finalInputControls[finalIndex] :
                finalControls[finalIndex];

        goal[i + j] = 
                (initialControl * std::get<2>(index_control)) - finalControl;
        if (getModeIsCost()) { goal[i + j] = SimTK::square(goal[i + j]); }
        ++j;
    }
}

void MocoPeriodicityGoal::printDescriptionImpl() const {
    log_info("        state periodicity pairs:");
    for (const auto& pair : m_state_names) {
        log_info("                initial: {}, final: {}", pair.first,
                pair.second);
    }
    log_info("        control periodicity pairs:");
    for (const auto& pair : m_control_names) {
        log_info("                initial: {}, final: {}", pair.first,
                pair.second);
    }
}
