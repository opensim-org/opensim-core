/* -------------------------------------------------------------------------- *
 * OpenSim Moco: DiscreteController.cpp                                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
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

#include "DiscreteController.h"

#include <OpenSim/Simulation/SimulationUtilities.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

void DiscreteController::setDiscreteControls(SimTK::State& s,
        const SimTK::Vector& controls) const {
    updDiscreteControls(s) = controls;
}

SimTK::Vector& DiscreteController::updDiscreteControls(SimTK::State& s) const {
    const SimTK::Subsystem& subSys = getSystem().getDefaultSubsystem();
    auto& dv = subSys.updDiscreteVariable(s, m_discreteVarIndex);
    auto& discreteControls = SimTK::Value<SimTK::Vector>::updDowncast(dv).upd();
    return discreteControls;
}

const SimTK::Vector& DiscreteController::getDiscreteControls(
        const SimTK::State& s) const {
    const SimTK::Subsystem& subSys = getSystem().getDefaultSubsystem();
    auto& dv = subSys.getDiscreteVariable(s, m_discreteVarIndex);
    auto& discreteControls = SimTK::Value<SimTK::Vector>::downcast(dv).get();
    return discreteControls;
}

void DiscreteController::computeControls(
        const SimTK::State& s, SimTK::Vector& controls) const {
    const SimTK::Subsystem& subSys = getSystem().getDefaultSubsystem();
    const auto& dv = subSys.getDiscreteVariable(s, m_discreteVarIndex) ;
    const auto& discreteControls =
            SimTK::Value<SimTK::Vector>::downcast(dv).get();
    for (int i = 0; i < (int)m_controlIndices.size(); ++i) {
        controls[m_controlIndices[i]] += discreteControls[i];
    }
}

void DiscreteController::extendRealizeTopology(SimTK::State& state) const {
    Super::extendRealizeTopology(state);
    const SimTK::Subsystem& subSys = getSystem().getDefaultSubsystem();

    // Get the control indexes for the actuators that are in the actuator set.
    // This logic stores the indexes in the order of actuators stored in the
    // model, skipping over indexes for any actuators not included in the
    // DiscreteController's actuator set.
    int count = 0;
    for (const auto& actu : getModel().getComponentList<Actuator>()) {
        const int nc = actu.numControls();
        for (int ic = 0; ic < nc; ++ic) {
            const auto& socket = getSocket<Actuator>("actuators");
            for (int i = 0; i < socket.getNumConnectees(); ++i) {
                if (socket.getConnecteePath(i) == actu.getAbsolutePath()) {
                    m_controlIndices.push_back(count);;
                    break;
                }
            }
            count++;
        }
    }

    const SimTK::Vector initControls(
            static_cast<int>(m_controlIndices.size()), 0.0);
    m_discreteVarIndex =
            subSys.allocateDiscreteVariable(state, SimTK::Stage::Dynamics,
                    new SimTK::Value<SimTK::Vector>(initControls));
}
