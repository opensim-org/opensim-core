/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoPeriodicityCost.cpp                                          *
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

#include "MocoPeriodicityCost.h"

#include "../MocoUtilities.h"

using namespace OpenSim;


//=============================================================================
//  MOCO PERIODICITY PAIR
//=============================================================================

MocoPeriodicityCostPair::MocoPeriodicityCostPair() { constructProperties(); }

MocoPeriodicityCostPair::MocoPeriodicityCostPair(std::string name) {
    setName(std::move(name));
    constructProperties();
}

void MocoPeriodicityCostPair::constructProperties() {
    constructProperty_first("");
    constructProperty_second("");
}

//=============================================================================
//  MOCO PERIODICITY COST
//=============================================================================

MocoPeriodicityCost::MocoPeriodicityCost() { constructProperties(); }

void MocoPeriodicityCost::calcCostImpl(
        const CostInput& input, SimTK::Vector& values) const {

    const auto& initialStates = input.initial_state.getY();
    const auto& finalStates = input.final_state.getY();
    int i = 0;
    for (const auto& indices_states : m_indices_states) {
        values[i] = initialStates[indices_states.first] -
                finalStates[indices_states.second];
        ++i;
    }

    const auto& initialControls = getModel().getControls(input.initial_state);
    const auto& finalControls = getModel().getControls(input.final_state);
    int j = 0;
    for (const auto& indices_controls : m_indices_controls) {
        values[i+j] = initialControls[indices_controls.first] -
                finalControls[indices_controls.second];
        ++j;
    }

}

void MocoPeriodicityCost::initializeOnModelImpl(const Model& model) const {

    auto allSysYIndices = createSystemYIndexMap(model);
    int nStatePairs = getProperty_state_pairs().size();
    for (int i = 0; i < nStatePairs; ++i) {

        auto path1 = get_state_pairs(i).get_first();
        auto path2 = get_state_pairs(i).get_second();

        int stateIndex1 = allSysYIndices[path1];
        int stateIndex2 = allSysYIndices[path2];

        m_indices_states.emplace_back(stateIndex1,stateIndex2);
    }


    auto systemControlIndexMap = createSystemControlIndexMap(model);
    int nControlPairs = getProperty_control_pairs().size();
    for (int i = 0; i < nControlPairs; ++i) {

        auto path1 = get_control_pairs(i).get_first();
        auto path2 = get_control_pairs(i).get_second();

        int controlIndex1 = systemControlIndexMap[path1];
        int controlIndex2 = systemControlIndexMap[path2];

        m_indices_controls.emplace_back(controlIndex1,controlIndex2);
    }
}

int MocoPeriodicityCost::getNumOutputsImpl() const {
    return getProperty_state_pairs().size() +
            getProperty_control_pairs().size();
}

void MocoPeriodicityCost::constructProperties() {
    constructProperty_state_pairs();
    constructProperty_control_pairs();
}

