/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoProblem.cpp                                          *
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

#include "MucoProblem.h"

using namespace OpenSim;


// ============================================================================
// MucoVariableInfo
// ============================================================================

MucoVariableInfo::MucoVariableInfo() {
    constructProperties();
}

MucoVariableInfo::MucoVariableInfo(const std::string& name,
        const MucoBounds& bounds, const MucoInitialBounds& initial,
        const MucoFinalBounds& final) : MucoVariableInfo() {
    setName(name);
    set_bounds(bounds.getAsArray());
    set_initial_bounds(initial.getAsArray());
    set_final_bounds(final.getAsArray());
}

void MucoVariableInfo::constructProperties() {
    constructProperty_bounds();
    constructProperty_initial_bounds();
    constructProperty_final_bounds();
}




// ============================================================================
// MucoPhase
// ============================================================================
MucoPhase::MucoPhase() {
    constructProperties();
}
void MucoPhase::constructProperties() {
    constructProperty_model(Model());
    constructProperty_time_initial_bounds();
    constructProperty_time_final_bounds();
    constructProperty_state_infos();
    constructProperty_control_infos();
    constructProperty_costs();
}
void MucoPhase::setModel(const Model& model) {
    set_model(model);
}
void MucoPhase::setTimeBounds(const MucoInitialBounds& initial,
        const MucoFinalBounds& final) {
    set_time_initial_bounds(initial.getAsArray());
    set_time_final_bounds(final.getAsArray());
}
void MucoPhase::setStateInfo(const std::string& name, const MucoBounds& bounds,
        const MucoInitialBounds& initial, const MucoFinalBounds& final) {
    int idx = getProperty_state_infos().findIndexForName(name);

    MucoVariableInfo info(name, bounds, initial, final);
    if (idx == -1) append_state_infos(info);
    else           upd_state_infos(idx) = info;
}
void MucoPhase::setControlInfo(const std::string& name,
        const MucoBounds& bounds,
        const MucoInitialBounds& initial, const MucoFinalBounds& final) {
    int idx = getProperty_control_infos().findIndexForName(name);

    MucoVariableInfo info(name, bounds, initial, final);
    if (idx == -1) append_control_infos(info);
    else           upd_control_infos(idx) = info;
}
void MucoPhase::addCost(const MucoCost& cost) {
    OPENSIM_THROW_IF_FRMOBJ(cost.getName().empty(), Exception,
        "Cannot add a cost if it does not have a name (use setName()).");
    int idx = getProperty_costs().findIndexForName(cost.getName());
    OPENSIM_THROW_IF_FRMOBJ(idx != -1, Exception,
        "A cost with name '" + cost.getName() + "' already exists.");
    append_costs(cost);
}
MucoInitialBounds MucoPhase::getTimeInitialBounds() const {
    return MucoInitialBounds(getProperty_time_initial_bounds());
}
MucoFinalBounds MucoPhase::getTimeFinalBounds() const {
    return MucoFinalBounds(getProperty_time_final_bounds());
}
std::vector<std::string> MucoPhase::createStateInfoNames() const {
    std::vector<std::string> names(getProperty_state_infos().size());
    for (int i = 0; i < getProperty_state_infos().size(); ++i) {
        names[i] = get_state_infos(i).getName();
    }
    return names;
}
std::vector<std::string> MucoPhase::createControlInfoNames() const {
    std::vector<std::string> names(getProperty_control_infos().size());
    for (int i = 0; i < getProperty_control_infos().size(); ++i) {
        names[i] = get_control_infos(i).getName();
    }
    return names;
}
const MucoVariableInfo& MucoPhase::getStateInfo(
        const std::string& name) const {
    int idx = getProperty_state_infos().findIndexForName(name);
    OPENSIM_THROW_IF_FRMOBJ(idx == -1, Exception,
            "No info available for state '" + name + "'.");
    return get_state_infos(idx);
}
const MucoVariableInfo& MucoPhase::getControlInfo(
        const std::string& name) const {

    int idx = getProperty_control_infos().findIndexForName(name);
    OPENSIM_THROW_IF_FRMOBJ(idx == -1, Exception,
            "No info provided for control for '" + name + "'.");
    return get_control_infos(idx);
}
void MucoPhase::initialize(const Model& model) const {
    /// Must use the model provided in this function, *not* the one stored as
    /// a property in this class.
    const auto stateNames = model.getStateVariableNames();
    for (int i = 0; i < getProperty_state_infos().size(); ++i) {
        const auto& name = get_state_infos(i).getName();
        OPENSIM_THROW_IF(stateNames.findIndex(name) == -1, Exception,
                "State info provided for nonexistant state '" + name + "'.");
    }
    OpenSim::Array<std::string> actuNames;
    const auto modelPath = model.getAbsolutePath();
    for (const auto& actu : model.getComponentList<Actuator>()) {
        actuNames.append(
                actu.getAbsolutePath().formRelativePath(modelPath).toString());
    }
    // TODO can only handle ScalarActuators?
    for (int i = 0; i < getProperty_control_infos().size(); ++i) {
        const auto& name = get_control_infos(i).getName();
        OPENSIM_THROW_IF(actuNames.findIndex(name) == -1, Exception,
                "Control info provided for nonexistant actuator '"
                        + name + "'.");
    }

    for (int i = 0; i < getProperty_costs().size(); ++i) {
        const_cast<MucoCost&>(get_costs(i)).initialize(model);
    }
}




// ============================================================================
// MucoProblem
// ============================================================================
MucoProblem::MucoProblem() {
    constructProperties();
}

void MucoProblem::setModel(const Model& model) {
    upd_phases(0).setModel(model);
}
void MucoProblem::setTimeBounds(const MucoInitialBounds& initial,
        const MucoFinalBounds& final) {
    upd_phases(0).setTimeBounds(initial, final);
}
void MucoProblem::setStateInfo(const std::string& name,
        const MucoBounds& bounds,
        const MucoInitialBounds& initial, const MucoFinalBounds& final) {
    upd_phases(0).setStateInfo(name, bounds, initial, final);
}
void MucoProblem::setControlInfo(const std::string& name,
        const MucoBounds& bounds,
        const MucoInitialBounds& initial, const MucoFinalBounds& final) {
    upd_phases(0).setControlInfo(name, bounds, initial, final);
}
void MucoProblem::addCost(const MucoCost& cost) {
    upd_phases(0).addCost(cost);
}
void MucoProblem::initialize(const Model& model) const {
    for (int i = 0; i < getProperty_phases().size(); ++i)
        get_phases(i).initialize(model);
}
void MucoProblem::constructProperties() {
    constructProperty_phases(Array<MucoPhase>(MucoPhase(), 1));
}
