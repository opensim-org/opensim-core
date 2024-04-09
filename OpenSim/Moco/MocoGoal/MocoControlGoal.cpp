/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoControlGoal.cpp                                          *
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

#include "MocoControlGoal.h"

#include <OpenSim/Moco/Components/ActuatorInputController.h>
#include <OpenSim/Moco/Components/ControlDistributor.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimulationUtilities.h>

using namespace OpenSim;

MocoControlGoal::MocoControlGoal() { constructProperties(); }

void MocoControlGoal::constructProperties() {
    constructProperty_control_weights(MocoWeightSet());
    constructProperty_control_weights_pattern(MocoWeightSet());
    constructProperty_exponent(2);
    constructProperty_ignore_controlled_actuators(false);
}

void MocoControlGoal::setWeightForControl(
        const std::string& controlName, const double& weight) {
    if (get_control_weights().contains(controlName)) {
        upd_control_weights().get(controlName).setWeight(weight);
    } else {
        upd_control_weights().cloneAndAppend({controlName, weight});
    }
}

void MocoControlGoal::setWeightForControlPattern(
        const std::string& pattern, const double& weight) {
    if (get_control_weights_pattern().contains(pattern)) {
        upd_control_weights_pattern().get(pattern).setWeight(weight);
    } else {
        upd_control_weights_pattern().cloneAndAppend({pattern, weight});
    }
}

void MocoControlGoal::initializeOnModelImpl(const Model& model) const {

    // Get all the control names and indices in the model.
    auto controlNames = createControlNamesFromModel(model);
    auto controlIndexMap = createSystemControlIndexMap(model);

    // Get control names associated with the model's ActuatorInputController.
    auto actuatorInputControlNames =
            createControlNamesForControllerType<ActuatorInputController>(model);

    // Get the remaining Input control names from the ControlDistributor.
    auto allControlNames = 
            model.getComponentList<ControlDistributor>().begin()
                    ->getControlNamesInOrder();
    int numInputControls = 
            allControlNames.size() - actuatorInputControlNames.size();
    std::vector<std::string> inputControlNames;
    inputControlNames.reserve(numInputControls);
    for (int i = 0; i < numInputControls; ++i) {
        inputControlNames.push_back(allControlNames[i]);
    }


    // Make sure there are no weights for nonexistent controls.
    for (int i = 0; i < get_control_weights().getSize(); ++i) {
        const auto& thisName = get_control_weights()[i].getName();
        if (std::find(controlNames.begin(), controlNames.end(), thisName) ==
                controlNames.end()) {
            OPENSIM_THROW_FRMOBJ(
                    Exception, "Unrecognized control '" + thisName + "'.");
        }
    }

    // Set the regex pattern controls first.
    std::map<std::string, double> weightsFromPatterns;
    for (int i = 0; i < get_control_weights_pattern().getSize(); ++i) {
        const auto& mocoWeight = get_control_weights_pattern().get(i);
        const auto& pattern = mocoWeight.getName();
        const auto regex = std::regex(pattern);
        for (const auto& controlName : controlNames) {
            if (std::regex_match(controlName, regex)) {
                weightsFromPatterns[controlName] = mocoWeight.getWeight();
            }
        }
    }

    for (const auto& controlName : controlNames) {
        double weight = 1.0;
        if (get_control_weights().contains(controlName)) {
            weight = get_control_weights().get(controlName).getWeight();
        } else if (weightsFromPatterns.count(controlName)) {
            weight = weightsFromPatterns[controlName];
        }

        if (getIgnoreControlledActuators() &&
                !actuatorInputControlNames.count(controlName)) {
            log_info("MocoControlGoal: Control '{}' is associated with a "
                     "user-defined controller and will be ignored, "
                     "as requested.",
                    controlName);
            continue;
        }

        if (weight != 0.0) {
            m_controlIndices.push_back(controlIndexMap[controlName]);
            m_weights.push_back(weight);
            m_controlNames.push_back(controlName);
        } else {
            log_info("MocoControlGoal: Control '{}' has weight 0 and will be "
                     "ignored.", controlName);
        }
    }

    OPENSIM_THROW_IF_FRMOBJ(get_exponent() < 2, Exception,
            "Exponent must be 2 or greater.");
    int exponent = get_exponent();

    // The pow() function gives slightly different results than x * x. On Mac,
    // using x * x requires fewer solver iterations.
    if (exponent == 2) {
        m_power_function = [](const double& x) { return x * x; };
    } else {
        m_power_function = [exponent](const double& x) {
            return pow(std::abs(x), exponent);
        };
    }

    setRequirements(1, 1, SimTK::Stage::Model);
}

void MocoControlGoal::calcIntegrandImpl(
        const IntegrandInput& input, SimTK::Real& integrand) const {
    const auto& controls = input.controls;

    integrand = 0;
    int iweight = 0;
    for (const auto& icontrol : m_controlIndices) {
        const auto& control = controls[icontrol];
        integrand += m_weights[iweight] * m_power_function(control);
        ++iweight;
    }

    // TODO temporary hack for testing.
    // std::cout << "input control: " << input.input_controls[0] << std::endl;
    const auto& input_control = input.input_controls[0];
    integrand += m_power_function(input_control);
}

void MocoControlGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& cost) const {
    cost[0] = input.integral;
}

void MocoControlGoal::printDescriptionImpl() const {
    for (int i = 0; i < (int) m_controlNames.size(); i++) {
        log_cout("        control: {}, weight: {}", m_controlNames[i],
                m_weights[i]);
    }
}
