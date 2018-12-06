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
    // TODO do not make a copy!
    auto state = p.getModel().getWorkingState();
    double time = double(arg.at(0));
    state.setTime(time);
    for (int i = 0; i < state.getNY(); ++i) {
        state.updY()[i] = double(arg.at(1)(i));
    }
    DM cost = p.calcEndpointCost(state);
    return {cost};
}

std::vector<DM> IntegrandCostFunction::eval(const std::vector<DM>& arg) const {
    m_transcrip.applyParametersToModel(arg.at(3));
    auto& state = m_transcrip.m_state;
    double time = double(arg.at(0));
    state.setTime(time);
    for (int i = 0; i < state.getNY(); ++i) {
        state.updY()[i] = double(arg.at(1)(i));
    }
    auto& controls = p.getModel().updControls(state);
    for (int i = 0; i < controls.size(); ++i) {
        controls[i] = double(arg.at(2)(i));
    }
    p.getModel().realizeVelocity(state);
    p.getModel().setControls(state, controls);
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
    // TODO move copying over the state to a separate function.
    double time = double(arg.at(0));
    state.setTime(time);
    for (int i = 0; i < state.getNY(); ++i) {
        state.updY()[i] = double(arg.at(1)(i));
    }
    auto& controls = p.getModel().updControls(state);
    for (int i = 0; i < controls.size(); ++i) {
        controls[i] = double(arg.at(2)(i));
    }
    p.getModel().realizeVelocity(state);
    p.getModel().setControls(state, controls);
    p.getModel().realizeAcceleration(state);
    DM deriv(state.getNU() + state.getNZ(), 1);
    for (int i = 0; i < deriv.rows(); ++i) {
        // Skip over qdot.
        deriv(i, 0) = state.getYDot()[i + state.getNQ()];
    }
    return {deriv};
}
