/* -------------------------------------------------------------------------- *
 *                       OpenSim:  InputController.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "InputController.h"
#include "OpenSim/Common/Logger.h"
#include <spdlog/fmt/bundled/core.h>

using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
InputController::InputController() : Controller() {}

InputController::~InputController() = default;

InputController::InputController(const InputController& other) = default;

InputController&
InputController::operator=(const InputController& other) = default;

InputController::InputController(InputController&& other) = default;

InputController& InputController::operator=(InputController&& other) = default;

//=============================================================================
// CONTROLLER INTERFACE
//=============================================================================
void InputController::computeControls(const SimTK::State& s,
        SimTK::Vector& controls) const {
    if (m_computeControls) {
        computeControlsImpl(s, controls);
    }
}

//=============================================================================
// METHODS
//=============================================================================
const std::vector<std::string>& InputController::getControlNames() const {
    return m_controlNames;
}

const std::vector<int>& InputController::getControlIndexes() const {
    return m_controlIndexes;
}

//=============================================================================
// MODEL COMPONENT INTERFACE
//=============================================================================
void InputController::extendConnectToModel(Model& model) {
    Super::extendConnectToModel(model);

    // Get a list of the control names and indexes for all actuators in the
    // model. These two vectors serve as a mapping between control names
    // model control cache indexes.
    std::vector<int> modelControlIndexes;
    const std::vector<std::string> modelControlNames =
            createControlNamesFromModel(model, modelControlIndexes);

    // Based on the controller's ActuatorSet, store lists of the control
    // names and their indexes in the model control cache.
    m_controlNames.clear();
    m_controlIndexes.clear();
    const auto& socket = getSocket<Actuator>("actuators");
    for (int i = 0; i < static_cast<int>(socket.getNumConnectees()); ++i) {
        const auto& actu = socket.getConnectee(i);
        if (actu.numControls() > 1) {
            // Non-scalar actuator.
            for (int j = 0; j < actu.numControls(); ++j) {
                // Use the control name format based on
                // SimulationUtilities::createControlNamesFromModel().
                m_controlNames.push_back(
                        fmt::format("{}_{}", actu.getAbsolutePathString(), j));
                auto it = std::find(modelControlNames.begin(),
                        modelControlNames.end(), m_controlNames.back());
                m_controlIndexes.push_back(
                        modelControlIndexes[it - modelControlNames.begin()]);
            }
        } else {
            // Scalar actuator.
            m_controlNames.push_back(actu.getAbsolutePathString());
            auto it = std::find(modelControlNames.begin(),
                    modelControlNames.end(), m_controlNames.back());
            m_controlIndexes.push_back(
                    modelControlIndexes[it - modelControlNames.begin()]);
        }
    }

    setNumControls(static_cast<int>(m_controlNames.size()));

    // Check Input connections.
    const auto& input = getInput<double>("controls");
    int numConnectees = static_cast<int>(input.getNumConnectees());
    if (numConnectees > 0) {
        OPENSIM_THROW_IF_FRMOBJ(numConnectees != getNumInputControls(),
            Exception, "Expected {} Input connectee(s), but received {}.",
            getNumInputControls(), numConnectees);
        m_computeControls = true;
    } else {
        log_info("No Input controls are connected to InputController {}, "
                "therefore it will be ignored when computing model controls.", 
                 getAbsolutePathString());
        m_computeControls = false;
    } 
}
