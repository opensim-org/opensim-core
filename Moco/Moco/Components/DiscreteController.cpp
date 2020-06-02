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
    controls += discreteControls;
}

void DiscreteController::extendRealizeTopology(SimTK::State& state) const {
    Super::extendRealizeTopology(state);
    const SimTK::Subsystem& subSys = getSystem().getDefaultSubsystem();
    m_discreteVarIndex =
            subSys.allocateDiscreteVariable(state, SimTK::Stage::Dynamics,
                    new SimTK::Value<SimTK::Vector>(
                            SimTK::Vector(getModel().getNumControls(), 0.0)));
}
