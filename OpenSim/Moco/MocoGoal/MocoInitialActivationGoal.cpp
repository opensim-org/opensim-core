/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoInitialActivationGoal.cpp                                *
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

#include "MocoInitialActivationGoal.h"

#include <OpenSim/Simulation/SimulationUtilities.h>
#include <OpenSim/Simulation/Control/InputController.h>
#include <OpenSim/Moco/Components/ControlAllocator.h>

using namespace OpenSim;

void MocoInitialActivationGoal::initializeOnModelImpl(
        const Model& model) const {

    // Get a map of all the state indices in the system.
    auto allSysYIndices = createSystemYIndexMap(model);

    // If there are no user-defined controllers, we can use the raw controls.
    // Otherwise, we must compute the controls from the model.
    // TODO move to MocoGoal?
    const auto& controllers = model.getComponentList<Controller>();
    int numControllers =
            (int)std::distance(controllers.begin(), controllers.end());
    if (numControllers > 1) {
        m_computeControlsFromModel = true;
    }

    // Create a map from control names to their indices in the controls vector.
    // If we are using the raw controls, we use the control indices from the
    // model's ControlAllocator. Otherwise, we use the control indices from the
    // model.
    std::unordered_map<std::string, int> controlIndexMap;
    if (m_computeControlsFromModel) {
        controlIndexMap = createSystemControlIndexMap(model);
    } else {
        controlIndexMap = model.getComponentList<ControlAllocator>().begin()
                                  ->getControlIndexMap();
    }

    for (const auto& muscle : model.getComponentList<Muscle>()) {
        if (!muscle.get_ignore_activation_dynamics()) {
            const std::string path = muscle.getAbsolutePathString();
            int excitationIndex = controlIndexMap[path];
            int activationIndex = allSysYIndices[path + "/activation"];
            m_indices.emplace_back(excitationIndex, activationIndex);
        }
    }

    setRequirements(0, (int)m_indices.size(), SimTK::Stage::Time);
}

void MocoInitialActivationGoal::calcGoalImpl(
        const GoalInput& input, SimTK::Vector& goal) const {
    // TODO: compute controls in MocoGoal::calcIntegrand() and pass them to
    // calcIntegrandImpl() by overwriting the IntegrandInput?
    if (m_computeControlsFromModel) {
        getModel().realizeVelocity(input.initial_state);
    }
    const auto& controls = m_computeControlsFromModel ?
            getModel().getControls(input.initial_state) : input.initial_controls;

    const auto& states = input.initial_state.getY();
    int i = 0;
    if (!getModeIsCost()) {
        for (const auto& indices : m_indices) {
            goal[i] = controls[indices.first] - states[indices.second];
            ++i;
        }
    } else {
        for (const auto& indices : m_indices) {
            goal[i] = SimTK::square(controls[indices.first] -
                    states[indices.second]);
            ++i;
        }
    }
}
