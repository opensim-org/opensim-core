/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoProblem.cpp                                          *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia, Nicholas Bianco                             *
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

#include <simbody/internal/Constraint.h>

using namespace OpenSim;


// ============================================================================
// MucoPhase
// ============================================================================
MucoPhase::MucoPhase() {
    constructProperties();
}
void MucoPhase::constructProperties() {
    constructProperty_model(Model());
    constructProperty_time_initial_bounds(MucoInitialBounds());
    constructProperty_time_final_bounds(MucoFinalBounds());
    constructProperty_default_speed_bounds(MucoBounds(-50, 50));
    constructProperty_state_infos();
    constructProperty_control_infos();
    constructProperty_parameters();
    constructProperty_costs();
    constructProperty_path_constraints();
    constructProperty_multibody_constraint_bounds(MucoBounds(0));
    constructProperty_multiplier_bounds(MucoBounds(-1000.0, 1000.0));
}
void MucoPhase::setModel(Model model) {
    set_model(std::move(model));
}
void MucoPhase::setTimeBounds(const MucoInitialBounds& initial,
        const MucoFinalBounds& final) {
    set_time_initial_bounds(initial);
    set_time_final_bounds(final);
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
void MucoPhase::addParameter(const MucoParameter& parameter) {
    OPENSIM_THROW_IF_FRMOBJ(parameter.getName().empty(), Exception,
        "Cannot add a parameter if it does not have a name (use setName()).");
    int idx = getProperty_parameters().findIndexForName(parameter.getName());
    OPENSIM_THROW_IF_FRMOBJ(idx != -1, Exception,
        "A parameter with name '" + parameter.getName() + "' already exists.");
    append_parameters(parameter);
}
void MucoPhase::addCost(const MucoCost& cost) {
    OPENSIM_THROW_IF_FRMOBJ(cost.getName().empty(), Exception,
        "Cannot add a cost if it does not have a name (use setName()).");
    int idx = getProperty_costs().findIndexForName(cost.getName());
    OPENSIM_THROW_IF_FRMOBJ(idx != -1, Exception,
        "A cost with name '" + cost.getName() + "' already exists.");
    append_costs(cost);
}
void MucoPhase::addPathConstraint(const MucoPathConstraint& constraint) {
    OPENSIM_THROW_IF_FRMOBJ(constraint.getName().empty(), Exception,
        "Cannot add a constraint if it does not have a name (use setName()).");
    int idx = 
        getProperty_path_constraints().findIndexForName(constraint.getName());
    OPENSIM_THROW_IF_FRMOBJ(idx != -1, Exception,
        "A constraint with name '" + constraint.getName() + "' already "
        "exists.");
    append_path_constraints(constraint);
}
MucoInitialBounds MucoPhase::getTimeInitialBounds() const {
    return get_time_initial_bounds();
}
MucoFinalBounds MucoPhase::getTimeFinalBounds() const {
    return get_time_final_bounds();
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
            "No info available for control '" + name + "'.");
    return get_control_infos(idx);
}
const MucoParameter& MucoPhase::getParameter(
        const std::string& name) const {

    int idx = getProperty_parameters().findIndexForName(name);
    OPENSIM_THROW_IF_FRMOBJ(idx == -1, Exception,
            "No parameter with name '" + name + "' found.");
    return get_parameters(idx);
}
MucoParameter& MucoPhase::updParameter(
    const std::string& name) {

    int idx = getProperty_parameters().findIndexForName(name);
    OPENSIM_THROW_IF_FRMOBJ(idx == -1, Exception,
        "No parameter with name '" + name + "' found.");
    return upd_parameters(idx);
}
const MucoPathConstraint& MucoPhase::getPathConstraint(
        const std::string& name) const {

    int idx = getProperty_path_constraints().findIndexForName(name);
    OPENSIM_THROW_IF_FRMOBJ(idx == -1, Exception,
            "No path constraint with name '" + name + "' found.");
    return get_path_constraints(idx);
}


// ============================================================================
// MucoProblem
// ============================================================================
MucoProblem::MucoProblem() {
    constructProperties();
}

void MucoProblem::setModel(Model model) {
    upd_phases(0).setModel(std::move(model));
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
void MucoProblem::setMultibodyConstraintBounds(const MucoBounds& bounds) {
    upd_phases(0).setMultibodyConstraintBounds(bounds);
}
void MucoProblem::setMultiplierBounds(const MucoBounds& bounds) {
    upd_phases(0).setMultiplierBounds(bounds);
}
void MucoProblem::addParameter(const MucoParameter& parameter) {
    upd_phases(0).addParameter(parameter);
}
void MucoProblem::addCost(const MucoCost& cost) {
    upd_phases(0).addCost(cost);
}
void MucoProblem::addPathConstraint(const MucoPathConstraint& cost) {
    upd_phases(0).addPathConstraint(cost);
}
void MucoProblem::constructProperties() {
    constructProperty_phases(Array<MucoPhase>(MucoPhase(), 1));
}
