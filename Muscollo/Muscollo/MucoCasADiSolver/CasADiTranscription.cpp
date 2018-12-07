/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: CasADiTranscription.cpp                                  *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2018 Stanford University and the Authors                     *
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

#include "CasADiTranscription.h"

using casadi::MX;
using casadi::DM;
using casadi::Sparsity;
using casadi::Slice;
using casadi::Callback;
using casadi::Dict;

std::vector<DM> EndpointCostFunction::eval(const std::vector<DM>& arg) const {
    m_transcrip.applyParametersToModel(arg.at(2));
    auto& state = m_transcrip.m_state;
    double time = double(arg.at(0));
    state.setTime(time);
    for (int i = 0; i < state.getNY(); ++i) {
        state.updY()[i] = double(arg.at(1)(i));
    }
    DM cost = p.calcEndpointCost(state);
    return {cost};
}

// TODO: Move to a better place.
void convertToSimTKState(const DM& time, const DM& states, const DM& controls,
        const Model& model, SimTK::State& simtkState) {
    simtkState.setTime(double(time));
    simtkState.setY(SimTK::Vector(simtkState.getNY(), states.ptr(), true));
    auto& simtkControls = model.updControls(simtkState);
    simtkControls = SimTK::Vector(simtkControls.size(), controls.ptr(), true);
    model.realizeVelocity(simtkState);
    model.setControls(simtkState, simtkControls);
}

std::vector<DM> IntegrandCostFunction::eval(const std::vector<DM>& arg) const {
    m_transcrip.applyParametersToModel(arg.at(3));
    auto& state = m_transcrip.m_state;
    convertToSimTKState(arg.at(0), arg.at(1), arg.at(2), p.getModel(), state);
    // TODO don't necessarily need to realize to Velocity just to penalize
    // controls.
    return {p.calcIntegralCost(state)};
}

casadi::Sparsity DynamicsFunction::get_sparsity_out(casadi_int i) {
    if (i == 0) {
        int numRows = m_transcrip.m_state.getNU() + m_transcrip.m_state.getNZ();
        return casadi::Sparsity::dense(numRows, 1);
    }
    else return casadi::Sparsity(0, 0);
}
std::vector<DM> DynamicsFunction::eval(const std::vector<DM>& arg) const {
    m_transcrip.applyParametersToModel(arg.at(3));
    auto& state = m_transcrip.m_state;
    convertToSimTKState(arg.at(0), arg.at(1), arg.at(2), p.getModel(), state);
    p.getModel().realizeAcceleration(state);

    int numRowsOut = state.getNU() + state.getNZ();
    DM deriv(Sparsity::dense(numRowsOut, 1));
    // Modify the internal vector of nonzeros because it's more efficient.
    deriv.nonzeros().resize(numRowsOut);
    for (int i = 0; i < deriv.rows(); ++i) {
        // Skip over qdot.
        deriv.nonzeros()[i] = state.getYDot()[i + state.getNQ()];
    }
    return {deriv};
}
