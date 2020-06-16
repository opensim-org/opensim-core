/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoUserControlCost.cpp                                      *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Prasanna Sritharan, Christopher Dembia                          *
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


#include <Moco/MocoUtilities.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "MocoUserControlCost.h"

using namespace OpenSim;

MocoUserControlCost::MocoUserControlCost() {
    user_control_cost_fun_ptr = nullptr;
    utility_vector = std::vector<double>();
    constructProperties();
}

void MocoUserControlCost::constructProperties() {
    constructProperty_control_weights(MocoWeightSet());
}

void MocoUserControlCost::setWeight(
        const std::string& controlName, const double& weight) {
    if (get_control_weights().contains(controlName)) {
        upd_control_weights().get(controlName).setWeight(weight);
    } else {
        upd_control_weights().cloneAndAppend({controlName, weight});
    }
}

void MocoUserControlCost::initializeOnModelImpl(const Model& model) const {

    // Get all expected control names.
    auto controlNames = createControlNamesFromModel(model);

    // Check that the model controls are in the correct order.
    checkOrderSystemControls(model);

    auto systemControlIndexMap = createSystemControlIndexMap(model);
    // Make sure there are no weights for nonexistent controls.
    for (int i = 0; i < get_control_weights().getSize(); ++i) {
        const auto& thisName = get_control_weights()[i].getName();
        if (std::find(controlNames.begin(), controlNames.end(), thisName) ==
                controlNames.end()) {
            OPENSIM_THROW_FRMOBJ(
                    Exception, "Unrecognized control '" + thisName + "'.");
        }
    }

    for (const auto& controlName : controlNames) {
        double weight = 1.0;
        if (get_control_weights().contains(controlName)) {
            weight = get_control_weights().get(controlName).getWeight();
        }

        if (weight != 0.0) {
            m_controlIndices.push_back(systemControlIndexMap[controlName]);
            m_weights.push_back(weight);
        }
    }
}


// compute the user-defined integrand
void MocoUserControlCost::calcIntegrandImpl(
        const SimTK::State& state, double& integrand) const {
    integrand = 0;
    if (user_control_cost_fun_ptr) {
        integrand = user_control_cost_fun_ptr(
                state, getModel(), utility_vector, m_weights, m_controlIndices);
    }
}
