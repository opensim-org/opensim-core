/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoProblem.cpp                                              *
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

#include "MocoProblem.h"

#include <simbody/internal/Constraint.h>

using namespace OpenSim;

// ============================================================================
// MocoPhase
// ============================================================================
MocoPhase::MocoPhase() { constructProperties(); }
void MocoPhase::constructProperties() {
    constructProperty_model(ModelProcessor(Model{}));
    constructProperty_time_initial_bounds(MocoInitialBounds());
    constructProperty_time_final_bounds(MocoFinalBounds());
    constructProperty_default_speed_bounds(MocoBounds(-50, 50));
    constructProperty_bound_activation_from_excitation(true);
    constructProperty_state_infos();
    constructProperty_state_infos_pattern();
    constructProperty_control_infos();
    constructProperty_control_infos_pattern();
    constructProperty_parameters();
    constructProperty_costs();
    constructProperty_path_constraints();
    constructProperty_kinematic_constraint_bounds(MocoBounds(0));
    constructProperty_multiplier_bounds(MocoBounds(-1000.0, 1000.0));
}
Model* MocoPhase::setModel(std::unique_ptr<Model> model) {
    // Write the connectee paths to properties.
    model->finalizeConnections();
    return upd_model().setModel(std::move(model));
}
Model* MocoPhase::setModelCopy(Model model) {
    set_model(ModelProcessor(std::move(model)));
    return &upd_model().updModel();
}
void MocoPhase::setModelProcessor(ModelProcessor model) {
    set_model(std::move(model));
}
ModelProcessor& MocoPhase::updModelProcessor() {
    return upd_model();
}
void MocoPhase::setTimeBounds(
        const MocoInitialBounds& initial, const MocoFinalBounds& final) {
    set_time_initial_bounds(initial);
    set_time_final_bounds(final);
}
void MocoPhase::setStateInfo(const std::string& name, const MocoBounds& bounds,
        const MocoInitialBounds& initial, const MocoFinalBounds& final) {
    int idx = getProperty_state_infos().findIndexForName(name);

    MocoVariableInfo info(name, bounds, initial, final);
    if (idx == -1)
        append_state_infos(info);
    else
        upd_state_infos(idx) = info;
}
void MocoPhase::setStateInfoPattern(const std::string& pattern,
        const MocoBounds& bounds, const MocoInitialBounds& initial,
        const MocoFinalBounds& final) {
    int idx = getProperty_state_infos_pattern().findIndexForName(pattern);

    MocoVariableInfo info(pattern, bounds, initial, final);
    if (idx == -1)
        append_state_infos_pattern(info);
    else
        upd_state_infos_pattern(idx) = info;
}
void MocoPhase::setControlInfo(const std::string& name,
        const MocoBounds& bounds, const MocoInitialBounds& initial,
        const MocoFinalBounds& final) {
    int idx = getProperty_control_infos().findIndexForName(name);

    MocoVariableInfo info(name, bounds, initial, final);
    if (idx == -1)
        append_control_infos(info);
    else
        upd_control_infos(idx) = info;
}
void MocoPhase::setControlInfoPattern(const std::string& pattern,
        const MocoBounds& bounds, const MocoInitialBounds& initial,
        const MocoFinalBounds& final) {
    int idx = getProperty_control_infos_pattern().findIndexForName(pattern);

    MocoVariableInfo info(pattern, bounds, initial, final);
    if (idx == -1)
        append_control_infos_pattern(info);
    else
        upd_control_infos_pattern(idx) = info;
}
MocoInitialBounds MocoPhase::getTimeInitialBounds() const {
    return get_time_initial_bounds();
}
MocoFinalBounds MocoPhase::getTimeFinalBounds() const {
    return get_time_final_bounds();
}
const MocoVariableInfo& MocoPhase::getStateInfo(const std::string& name) const {

    int idx = getProperty_state_infos().findIndexForName(name);
    OPENSIM_THROW_IF_FRMOBJ(idx == -1, Exception,
            format("No info available for state '%s'.", name));
    return get_state_infos(idx);
}
const MocoVariableInfo& MocoPhase::getControlInfo(
        const std::string& name) const {

    int idx = getProperty_control_infos().findIndexForName(name);
    OPENSIM_THROW_IF_FRMOBJ(idx == -1, Exception,
            format("No info available for control '%s'.", name));
    return get_control_infos(idx);
}
const MocoParameter& MocoPhase::getParameter(const std::string& name) const {

    int idx = getProperty_parameters().findIndexForName(name);
    OPENSIM_THROW_IF_FRMOBJ(idx == -1, Exception,
            format("No parameter with name '%s' found.", name));
    return get_parameters(idx);
}
MocoParameter& MocoPhase::updParameter(const std::string& name) {

    int idx = getProperty_parameters().findIndexForName(name);
    OPENSIM_THROW_IF_FRMOBJ(idx == -1, Exception,
            format("No parameter with name '%s' found.", name));
    return upd_parameters(idx);
}
const MocoCost& MocoPhase::getCost(const std::string& name) const {

    int idx = getProperty_costs().findIndexForName(name);
    OPENSIM_THROW_IF_FRMOBJ(idx == -1, Exception,
            format("No cost with name '%s' found.", name));
    return get_costs(idx);
}
MocoCost& MocoPhase::updCost(const std::string& name) {

    int idx = updProperty_costs().findIndexForName(name);
    OPENSIM_THROW_IF_FRMOBJ(idx == -1, Exception,
            format("No cost with name '%s' found.", name));
    return upd_costs(idx);
}
const MocoPathConstraint& MocoPhase::getPathConstraint(
        const std::string& name) const {

    int idx = getProperty_path_constraints().findIndexForName(name);
    OPENSIM_THROW_IF_FRMOBJ(idx == -1, Exception,
            format("No path constraint with name '%s' found.", name));
    return get_path_constraints(idx);
}
MocoPathConstraint& MocoPhase::updPathConstraint(const std::string& name) {

    int idx = updProperty_path_constraints().findIndexForName(name);
    OPENSIM_THROW_IF_FRMOBJ(idx == -1, Exception,
            format("No path constraint with name '%s' found.", name));
    return upd_path_constraints(idx);
}

// ============================================================================
// MocoProblem
// ============================================================================
MocoProblem::MocoProblem() { constructProperties(); }

Model* MocoProblem::setModel(std::unique_ptr<Model> model) {
    return upd_phases(0).setModel(std::move(model));
}
Model* MocoProblem::setModelCopy(Model model) {
    return upd_phases(0).setModelCopy(std::move(model));
}
void MocoProblem::setModelProcessor(ModelProcessor model) {
    upd_phases(0).setModelProcessor(std::move(model));
}
void MocoProblem::setTimeBounds(
        const MocoInitialBounds& initial, const MocoFinalBounds& final) {
    upd_phases(0).setTimeBounds(initial, final);
}
void MocoProblem::setStateInfo(const std::string& name,
        const MocoBounds& bounds, const MocoInitialBounds& initial,
        const MocoFinalBounds& final) {
    upd_phases(0).setStateInfo(name, bounds, initial, final);
}
void MocoProblem::setControlInfo(const std::string& name,
        const MocoBounds& bounds, const MocoInitialBounds& initial,
        const MocoFinalBounds& final) {
    upd_phases(0).setControlInfo(name, bounds, initial, final);
}
void MocoProblem::setKinematicConstraintBounds(const MocoBounds& bounds) {
    upd_phases(0).setKinematicConstraintBounds(bounds);
}
void MocoProblem::setMultiplierBounds(const MocoBounds& bounds) {
    upd_phases(0).setMultiplierBounds(bounds);
}
MocoCost& MocoProblem::updCost(const std::string& name) {
    return upd_phases(0).updCost(name);
}
void MocoProblem::constructProperties() {
    constructProperty_phases(Array<MocoPhase>(MocoPhase(), 1));
}
void MocoProblem::setStateInfoPattern(const std::string& pattern,
        const MocoBounds& bounds, const MocoInitialBounds& initial,
        const MocoFinalBounds& final) {
    upd_phases(0).setStateInfoPattern(pattern, bounds, initial, final);
}

void MocoProblem::setControlInfoPattern(const std::string& pattern,
        const MocoBounds& bounds, const MocoInitialBounds& initial,
        const MocoFinalBounds& final) {
    upd_phases(0).setControlInfoPattern(pattern, bounds, initial, final);
}
