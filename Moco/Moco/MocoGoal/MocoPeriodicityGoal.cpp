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
                format("Could not find state '%s'.", path1));
        const auto path2 = get_state_pairs(i).get_final_variable();
        OPENSIM_THROW_IF(allSysYIndices.count(path2) == 0, Exception,
                format("Could not find state '%s'.", path2));
        int stateIndex1 = allSysYIndices[path1];
        int stateIndex2 = allSysYIndices[path2];
        m_state_names.emplace_back(path1, path2);
        m_indices_states.emplace_back(stateIndex1, stateIndex2,
                get_state_pairs(i).get_negate() ? -1 : 1);
    }

    auto systemControlIndexMap = createSystemControlIndexMap(model);
    int nControlPairs = getProperty_control_pairs().size();

    for (int i = 0; i < nControlPairs; ++i) {
        const auto path1 = get_control_pairs(i).get_initial_variable();
        OPENSIM_THROW_IF(systemControlIndexMap.count(path1) == 0, Exception,
                format("Could not find control '%s'.", path1));
        const auto path2 = get_control_pairs(i).get_final_variable();
        OPENSIM_THROW_IF(systemControlIndexMap.count(path2) == 0, Exception,
                format("Could not find control '%s'.", path2));
        int controlIndex1 = systemControlIndexMap[path1];
        int controlIndex2 = systemControlIndexMap[path2];
        m_control_names.emplace_back(path1, path2);
        m_indices_controls.emplace_back(controlIndex1, controlIndex2,
                get_control_pairs(i).get_negate() ? -1 : 1);
    }

    setNumIntegralsAndOutputs(
            0, (int)m_indices_states.size() + (int)m_indices_controls.size());
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

    const auto& initialControls = getModel().getControls(input.initial_state);
    const auto& finalControls = getModel().getControls(input.final_state);
    int j = 0;
    for (const auto& index_control : m_indices_controls) {
        goal[i + j] = (initialControls[std::get<0>(index_control)] *
                              std::get<2>(index_control)) -
                      finalControls[std::get<1>(index_control)];
        if (getModeIsCost()) { goal[i + j] = SimTK::square(goal[i + j]); }
        ++j;
    }
}

void MocoPeriodicityGoal::printDescriptionImpl(std::ostream& stream) const {
    stream << "        ";
    stream << "state periodicity pairs: " << std::endl;
    for (const auto& pair : m_state_names) {
        stream << "                ";
        stream << "initial: " << pair.first
               << ", final: " << pair.second << std::endl;
    }
    stream << "        ";
    stream << "control periodicity pairs: " << std::endl;
    for (const auto& pair : m_control_names) {
        stream << "                ";
        stream << "initial: " << pair.first
               << ", final: " << pair.second << std::endl;
    }
}
