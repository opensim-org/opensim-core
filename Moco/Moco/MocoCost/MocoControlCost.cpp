/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoControlCost.cpp                                          *
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

#include "MocoControlCost.h"
#include <OpenSim/Simulation/Model/Model.h>
#include "../MocoUtilities.h"

using namespace OpenSim;

MocoControlCost::MocoControlCost() {
    constructProperties();
}

void MocoControlCost::constructProperties() {
    constructProperty_control_weights(MocoWeightSet());
}

void MocoControlCost::setWeight(
        const std::string& controlName, const double& weight) {
    if (get_control_weights().contains(controlName)) {
        upd_control_weights().get(controlName).setWeight(weight);
    } else {
        upd_control_weights().cloneAndAppend({controlName, weight});
    }
}

void MocoControlCost::initializeOnModelImpl(const Model& model) const {

    // Get all expected control names.
    auto controlNames = createControlNamesFromModel(model);

    // Check that the model controls are in the correct order.
    checkOrderSystemControls(model);
    
    // Make sure there are no weights for nonexistent controls.
    for (int i = 0; i < get_control_weights().getSize(); ++i) {
        const auto& thisName = get_control_weights()[i].getName();
        if (std::find(controlNames.begin(), controlNames.end(), thisName) ==
                controlNames.end()) {
            OPENSIM_THROW_FRMOBJ(Exception,
                    "Unrecognized control '" + thisName + "'.");
        }
    }

    m_weights.resize(model.getNumControls());
    int i = 0;
    for (const auto& controlName : controlNames) {
        double weight = 1.0;
        if (get_control_weights().contains(controlName)) {
            weight = get_control_weights().get(controlName).getWeight();
        }
        m_weights[i] = weight;
        ++i;
    }
}

void MocoControlCost::calcIntegralCostImpl(const SimTK::State& state,
        double& integrand) const {
    getModel().realizeVelocity(state); // TODO would avoid this, ideally.
    const auto& controls = getModel().getControls(state);
    integrand = 0;
    assert((int)m_weights.size() == controls.size());
    for (int i = 0; i < controls.size(); ++i) {
        integrand += m_weights[i] * controls[i] * controls[i];
    }
}
