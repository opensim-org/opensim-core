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

int EndpointCostFunction::eval(const double** inputs, double** outputs,
        casadi_int*, double*, void*) const {
    m_transcrip.applyParametersToModel(
            SimTK::Vector(p.getNumParameters(), inputs[2], true));
    auto& state = m_transcrip.m_state;
    state.setTime(inputs[0][0]);
    std::copy_n(inputs[1], state.getNY(),
            state.updY().updContiguousScalarData());
    // TODO set controls to NaN.
    outputs[0][0] = p.calcEndpointCost(state);
    return 0;
}

// TODO: Move to a better place.
void convertToSimTKState(
        const double* time, const double* states, const double* controls,
        const Model& model, SimTK::State& simtkState) {
    simtkState.setTime(time[0]);
    simtkState.setY(SimTK::Vector(simtkState.getNY(), states, true));
    std::copy_n(states, simtkState.getNY(),
            simtkState.updY().updContiguousScalarData());
    auto& simtkControls = model.updControls(simtkState);
    std::copy_n(controls, simtkControls.size(),
            simtkControls.updContiguousScalarData());
    model.realizeVelocity(simtkState);
    model.setControls(simtkState, simtkControls);
}

int IntegrandCostFunction::eval(const double** inputs, double** outputs,
        casadi_int*, double*, void*) const {
    m_transcrip.applyParametersToModel(
            SimTK::Vector(p.getNumParameters(), inputs[3], true));
    auto& state = m_transcrip.m_state;
    // TODO don't necessarily need to realize to Velocity just to penalize
    // controls.
    convertToSimTKState(inputs[0], inputs[1], inputs[2], p.getModel(), state);
    outputs[0][0] = p.calcIntegralCost(state);
    return 0;
}

casadi::Sparsity DynamicsFunction::get_sparsity_out(casadi_int i) {
    if (i == 0) {
        int numRows = m_transcrip.m_state.getNU() + m_transcrip.m_state.getNZ();
        return casadi::Sparsity::dense(numRows, 1);
    }
    else return casadi::Sparsity(0, 0);
}
int DynamicsFunction::eval(const double** inputs, double** outputs,
        casadi_int*, double*, void*) const {
    m_transcrip.applyParametersToModel(
            SimTK::Vector(p.getNumParameters(), inputs[3], true));
    auto& state = m_transcrip.m_state;
    convertToSimTKState(inputs[0], inputs[1], inputs[2], p.getModel(), state);
    p.getModel().realizeAcceleration(state);

    // TODO create member variable for numRowsOut.
    int numRowsOut = state.getNU() + state.getNZ();
    std::copy_n(state.getYDot().getContiguousScalarData() + state.getNQ(),
            numRowsOut, outputs[0]);
    return 0;
}
