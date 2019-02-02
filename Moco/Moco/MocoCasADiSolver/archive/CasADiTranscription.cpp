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

class AccelerationStageNotAllowed : public OpenSim::Exception {
public:
    using Exception::Exception;
};

void EndpointCostFunction::init() {
    // Ensure that evaluating the cost functions does not throw
    // any exceptions.
    const auto& lowerBounds = m_transcrip.getVariablesLowerBounds();
    std::unique_ptr<const double*> inputArray(new const double*[3]);
    inputArray.get()[0] = lowerBounds.at(Var::final_time).ptr();
    inputArray.get()[1] = lowerBounds.at(Var::states)(Slice(), 0).ptr();
    inputArray.get()[2] = lowerBounds.at(Var::parameters).ptr();
    std::unique_ptr<double*> outputArray(new double*[1]);
    SimTK::Vector output(1);
    outputArray.get()[0] = output.updContiguousScalarData();
    try {
        eval(inputArray.get(), outputArray.get(), nullptr, nullptr, nullptr);
    } catch (const AccelerationStageNotAllowed&) {
        throw;
    } catch (...) {}
}

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
            AccelerationStageNotAllowed,
            "Cannot realize to Acceleration in implicit dynamics mode.");
    return 0;
}

void IntegrandCostFunction::init() {
    // Ensure that evaluating the cost functions does not throw
    // any exceptions.
    const auto& lowerBounds = m_transcrip.getVariablesLowerBounds();
    std::unique_ptr<const double*> inputArray(new const double*[4]);
    inputArray.get()[0] = lowerBounds.at(Var::initial_time).ptr();
    inputArray.get()[1] = lowerBounds.at(Var::states)(Slice(), 0).ptr();
    inputArray.get()[2] = lowerBounds.at(Var::controls)(Slice(), 0).ptr();
    inputArray.get()[3] = lowerBounds.at(Var::parameters).ptr();
    std::unique_ptr<double*> outputArray(new double*[1]);
    SimTK::Vector output(1);
    outputArray.get()[0] = output.updContiguousScalarData();
    try {
        eval(inputArray.get(), outputArray.get(), nullptr, nullptr, nullptr);
    } catch (const AccelerationStageNotAllowed&) {
        throw;
    } catch (...) {}
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
            AccelerationStageNotAllowed,
            "Cannot realize to Acceleration in implicit dynamics mode.");
    return 0;
}

void PathConstraintFunction::init() {
    // Ensure that evaluating the path constraint does not throw
    // any exceptions.
    const auto& lowerBounds = m_transcrip.getVariablesLowerBounds();
    std::unique_ptr<const double*> inputArray(new const double*[4]);
    inputArray.get()[0] = lowerBounds.at(Var::initial_time).ptr();
    inputArray.get()[1] = lowerBounds.at(Var::states)(Slice(), 0).ptr();
    inputArray.get()[2] = lowerBounds.at(Var::controls)(Slice(), 0).ptr();
    inputArray.get()[3] = lowerBounds.at(Var::parameters).ptr();
    std::unique_ptr<double*> outputArray(new double*[1]);
    int numEquations = m_pathCon.getConstraintInfo().getNumEquations();
    SimTK::Vector output(numEquations);
    outputArray.get()[0] = output.updContiguousScalarData();
    try {
        eval(inputArray.get(), outputArray.get(), nullptr, nullptr, nullptr);
    } catch (const AccelerationStageNotAllowed&) {
        throw;
    } catch (...) {}
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
            AccelerationStageNotAllowed,
            "Cannot realize to Acceleration in implicit dynamics mode.");
    std::copy_n(errors.getContiguousScalarData(), errors.size(), outputs[0]);
    return 0;
}

