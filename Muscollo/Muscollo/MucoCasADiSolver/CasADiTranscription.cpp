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
    OPENSIM_THROW_IF(
            m_transcrip.dynamicsModeIsImplicit() &&
                    state.getSystemStage() >= SimTK::Stage::Acceleration,
            Exception,
            "Cannot realize to Acceleration in implicit dynamics mode.");
    return 0;
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
    OPENSIM_THROW_IF(
            m_transcrip.dynamicsModeIsImplicit() &&
                    state.getSystemStage() >= SimTK::Stage::Acceleration,
            Exception,
            "Cannot realize to Acceleration in implicit dynamics mode.");
    return 0;
}

int PathConstraintFunction::
eval(const double** inputs, double** outputs, casadi_int*, double*, void*)
        const {
    m_transcrip.applyParametersToModel(
            SimTK::Vector(p.getNumParameters(), inputs[3], true));
    auto& state = m_transcrip.m_state;
    // TODO don't necessarily need to realize to Velocity.
    convertToSimTKState(inputs[0], inputs[1], inputs[2], p.getModel(), state);
    SimTK::Vector errors(m_pathCon.getConstraintInfo().getNumEquations());
    m_pathCon.calcPathConstraintErrors(state, errors);
    OPENSIM_THROW_IF(
            m_transcrip.dynamicsModeIsImplicit() &&
                    state.getSystemStage() >= SimTK::Stage::Acceleration,
            Exception,
            "Cannot realize to Acceleration in implicit dynamics mode.");
    std::copy_n(errors.getContiguousScalarData(), errors.size(), outputs[0]);
    return 0;
}
