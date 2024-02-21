/* -------------------------------------------------------------------------- *
*                      OpenSim: ControlDistributor.cpp                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2024 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
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

#include "ControlDistributor.h"

using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
ControlDistributor::ControlDistributor() = default;

ControlDistributor::~ControlDistributor() noexcept = default;

ControlDistributor::ControlDistributor(const ControlDistributor&) = default;

ControlDistributor::ControlDistributor(ControlDistributor&&) = default;

ControlDistributor& ControlDistributor::operator=(const ControlDistributor&) = default;

ControlDistributor& ControlDistributor::operator=(
        ControlDistributor&&) = default;

//=============================================================================
// GET AND SET
//=============================================================================
void ControlDistributor::addControl(const std::string& controlName) {
    const int index = (int)m_controlIndexMap.size();
    m_controlIndexMap[controlName] = index;
}

void ControlDistributor::setControls(SimTK::State& s,
        const SimTK::Vector& controls) const {
    updControls(s) = controls;
}

SimTK::Vector& ControlDistributor::updControls(SimTK::State& s) const {
    const SimTK::Subsystem& subSys = getSystem().getDefaultSubsystem();
    auto& dv = subSys.updDiscreteVariable(s, m_discreteVarIndex);
    auto& discreteControls =
            SimTK::Value<SimTK::Vector>::updDowncast(dv).upd();
    return discreteControls;
}

const SimTK::Vector& ControlDistributor::getControls(
        const SimTK::State& s) const {
    const SimTK::Subsystem& subSys = getSystem().getDefaultSubsystem();
    auto& dv = subSys.getDiscreteVariable(s, m_discreteVarIndex);
    auto& discreteControls = SimTK::Value<SimTK::Vector>::downcast(dv).get();
    return discreteControls;
}

double ControlDistributor::getControlForOutputChannel(const SimTK::State& s,
        const std::string& channel) const {
    return getControls(s)[m_controlIndexMap.at(channel)];
}

std::vector<std::string> ControlDistributor::getControlNamesInOrder() const {
    // Return the control names in ascending order of their indices.
    std::vector<std::string> names;
    names.reserve(m_controlIndexMap.size());
    for (const auto& kv : m_controlIndexMap) {
        names.push_back(kv.first);
    }
    std::sort(names.begin(), names.end(),
            [&](const std::string& a, const std::string& b) {
                return m_controlIndexMap.at(a) < m_controlIndexMap.at(b);
            });
    return names;
}

//=============================================================================
// MODEL COMPONENT INTERFACE
//=============================================================================
void ControlDistributor::extendRealizeTopology(SimTK::State& state) const {
    Super::extendRealizeTopology(state);
    const SimTK::Subsystem& subSys = getSystem().getDefaultSubsystem();
    const SimTK::Vector initControls(
            static_cast<int>(m_controlIndexMap.size()), 0.0);
    m_discreteVarIndex =
            subSys.allocateDiscreteVariable(state, SimTK::Stage::Dynamics,
                    new SimTK::Value<SimTK::Vector>(initControls));
}

void ControlDistributor::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();
    for (const auto& kv : m_controlIndexMap) {
        updOutput("controls").addChannel(kv.first);
    }
}
