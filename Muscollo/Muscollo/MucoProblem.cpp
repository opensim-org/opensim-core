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
    constructProperty_kinematic_constraint_bounds(MucoBounds(0));
    constructProperty_multiplier_bounds(MucoBounds(-1000.0, 1000.0));
}
Model* MucoPhase::setModel(std::unique_ptr<Model> model) {
    // Write the connectee paths to properties.
    model->finalizeConnections();
    updProperty_model().clear();
    updProperty_model().adoptAndAppendValue(model.release());
    return &upd_model();
}
Model* MucoPhase::setModelCopy(Model model) {
    set_model(std::move(model));
    return &upd_model();
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
const MucoCost& MucoPhase::getCost(const std::string& name) const {

    int idx = getProperty_costs().findIndexForName(name);
    OPENSIM_THROW_IF_FRMOBJ(idx == -1, Exception,
            "No cost with name '" + name + "' found.");
    return get_costs(idx);
}
MucoCost& MucoPhase::updCost(const std::string& name) {

    int idx = updProperty_costs().findIndexForName(name);
    OPENSIM_THROW_IF_FRMOBJ(idx == -1, Exception,
            "No cost with name '" + name + "' found.");
    return upd_costs(idx);
}
const MucoPathConstraint& MucoPhase::getPathConstraint(
        const std::string& name) const {

    int idx = getProperty_path_constraints().findIndexForName(name);
    OPENSIM_THROW_IF_FRMOBJ(idx == -1, Exception,
            "No path constraint with name '" + name + "' found.");
    return get_path_constraints(idx);
}
MucoPathConstraint& MucoPhase::updPathConstraint(
        const std::string& name) {

    int idx = updProperty_path_constraints().findIndexForName(name);
    OPENSIM_THROW_IF_FRMOBJ(idx == -1, Exception,
            "No path constraint with name '" + name + "' found.");
    return upd_path_constraints(idx);
}


// ============================================================================
// MucoProblem
// ============================================================================
MucoProblem::MucoProblem() {
    constructProperties();
}

Model* MucoProblem::setModel(std::unique_ptr<Model> model) {
    return upd_phases(0).setModel(std::move(model));
}
Model* MucoProblem::setModelCopy(Model model) {
    return upd_phases(0).setModelCopy(std::move(model));
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
void MucoProblem::setKinematicConstraintBounds(const MucoBounds& bounds) {
    upd_phases(0).setKinematicConstraintBounds(bounds);
}
void MucoProblem::setMultiplierBounds(const MucoBounds& bounds) {
    upd_phases(0).setMultiplierBounds(bounds);
}
void MucoProblem::constructProperties() {
    constructProperty_phases(Array<MucoPhase>(MucoPhase(), 1));
}
