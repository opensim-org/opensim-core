/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoSumSquaredStateGoal.cpp                                  *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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
#include "MocoSumSquaredStateGoal.h"

#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

MocoSumSquaredStateGoal::MocoSumSquaredStateGoal() {
    constructProperties();
}

void MocoSumSquaredStateGoal::initializeOnModelImpl(const Model& model) const {
    // Throw exception if a weight is specified for a nonexistent state.
    auto allSysYIndices = createSystemYIndexMap(model);

    std::regex regex;
    if (getProperty_pattern().size()) { 
        regex = std::regex(get_pattern()); 
    }

    for (int i = 0; i < get_state_weights().getSize(); ++i) {
        const auto& weightName = get_state_weights().get(i).getName();
        if (allSysYIndices.count(weightName) == 0) {
            OPENSIM_THROW_FRMOBJ(Exception, "Weight provided with name '" +
                                                    weightName +
                                                    "' but this is "
                                                    "not a recognized state.");
        }
        if (getProperty_pattern().size() &&
                !std::regex_match(weightName, regex)) {
            OPENSIM_THROW_FRMOBJ(Exception,
                    format("Weight provided with name '%s' but this name does "
                           "not match the pattern '%s'.",
                            weightName, get_pattern()));
        }

        m_sysYIndices.push_back(allSysYIndices[weightName]);

        double weight = 1.0;
        if (get_state_weights().contains(weightName)) {
            weight = get_state_weights().get(weightName).getWeight();
        }
        m_state_weights.push_back(weight);
    }

    setNumIntegralsAndOutputs(1, 1);
}

void MocoSumSquaredStateGoal::calcIntegrandImpl(const SimTK::State& state,
        double& integrand) const {
    if (get_state_weights().getSize() == 0) {
        integrand = state.getY().normSqr();
    } 
    else {
        for (int i = 0; i < get_state_weights().getSize(); ++i) {
            const auto& value = state.getY()[m_sysYIndices[i]];
            integrand += value * value;
        }
    }
}

void MocoSumSquaredStateGoal::printDescriptionImpl(std::ostream& stream) const {
    for (int i = 0; i < (int)m_state_names.size(); i++) {
        stream << "        ";
        stream << "state: " << m_state_names[i] << ", "
               << "weight: " << m_state_weights[i] << std::endl;
    }
}
