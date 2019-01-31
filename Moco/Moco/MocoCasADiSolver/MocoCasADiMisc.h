#ifndef MOCO_MOCOCASADIMISC_H
#define MOCO_MOCOCASADIMISC_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoCasADiMisc.h                                             *
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

#include "../MocoBounds.h"
#include "CasOCProblem.h"

namespace OpenSim {

inline CasOC::Bounds convertBounds(const MocoBounds& mb) {
    return {mb.getLower(), mb.getUpper()};
}
inline CasOC::Bounds convertBounds(const MocoInitialBounds& mb) {
    return {mb.getLower(), mb.getUpper()};
}
inline CasOC::Bounds convertBounds(const MocoFinalBounds& mb) {
    return {mb.getLower(), mb.getUpper()};
}

template <typename VectorType = SimTK::Vector>
VectorType convertToSimTKVector(const casadi::DM& casVector) {
    assert(casVector.columns() == 1);
    VectorType simtkVector((int)casVector.rows());
    for (int i = 0; i < casVector.rows(); ++i) {
        simtkVector[i] = double(casVector(i));
    }
    return simtkVector;
}

/// This converts a casadi::DM matrix to a
/// SimTK::Matrix, transposing the data in the process.
SimTK::Matrix convertToSimTKMatrix(const casadi::DM& casMatrix) {
    SimTK::Matrix simtkMatrix((int)casMatrix.columns(), (int)casMatrix.rows());
    for (int irow = 0; irow < casMatrix.rows(); ++irow) {
        for (int icol = 0; icol < casMatrix.columns(); ++icol) {
            simtkMatrix(icol, irow) = double(casMatrix(irow, icol));
        }
    }
    return simtkMatrix;
}

template <typename TOut = MocoIterate>
TOut convertToMocoIterate(const CasOC::Iterate& casIt) {
    SimTK::Matrix simtkStates;
    const auto& casVars = casIt.variables;
    using CasOC::Var;
    if (!casIt.state_names.empty()) {
        simtkStates = convertToSimTKMatrix(casVars.at(Var::states));
    }
    SimTK::Matrix simtkControls;
    if (!casIt.control_names.empty()) {
        simtkControls = convertToSimTKMatrix(casVars.at(Var::controls));
    }
    SimTK::RowVector simtkParameters;
    if (!casIt.parameter_names.empty()) {
        const auto paramsValue = casVars.at(Var::parameters);
        simtkParameters = convertToSimTKVector<SimTK::RowVector>(paramsValue);
    }
    SimTK::Matrix simtkMultipliers;
    if (!casIt.multiplier_names.empty()) {
        const auto multsValue = casVars.at(Var::multipliers);
        simtkMultipliers = convertToSimTKMatrix(multsValue);
    }
    SimTK::Matrix simtkDerivatives;
    if (casVars.count(Var::derivatives)) {
        const auto derivsValue = casVars.at(Var::derivatives);
        simtkDerivatives = convertToSimTKMatrix(derivsValue);
    }

    SimTK::Vector simtkTimes = convertToSimTKVector(casIt.times);

    TOut mucoIterate(simtkTimes, casIt.state_names, casIt.control_names,
            casIt.multiplier_names, casIt.derivative_names,
            casIt.parameter_names, simtkStates, simtkControls, simtkMultipliers,
            simtkDerivatives, simtkParameters);
    return mucoIterate;
}

void applyParametersToModel(
        const SimTK::Vector& parameters, const MocoProblemRep& mocoProblemRep) {
    if (parameters.size()) mocoProblemRep.applyParametersToModel(parameters);
    const_cast<Model&>(mocoProblemRep.getModel()).initSystem();
}

void convertToSimTKState(const double* time, const double* states,
        const double* controls, const Model& model, SimTK::State& simtkState) {
    OPENSIM_THROW_IF(simtkState.getNQ() != simtkState.getNU(),
            OpenSim::Exception, "NQ != NU, copying state is incorrect.");
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

class MocoCasADiIntegralCostIntegrand : public CasOC::IntegralCostIntegrand {

public:
    MocoCasADiIntegralCostIntegrand(const OpenSim::MocoProblemRep& problem)
            : m_mocoProblemRep(problem), m_model(problem.getModel()),
              m_simtkState(m_model.getWorkingState()) {}
    // Use the more efficient virtual function (not the eval() that uses DMs)
    // to avoid any overhead.
    int eval(const double** inputs, double** outputs, casadi_int*, double*,
            void*) const override {
        applyParametersToModel(SimTK::Vector(m_casProblem->getNumParameters(),
                                       inputs[3], true),
                m_mocoProblemRep);
        convertToSimTKState(
                inputs[0], inputs[1], inputs[2], m_model, m_simtkState);
        // TODO: Create separate functions for each cost term.
        outputs[0][0] = m_mocoProblemRep.calcIntegralCost(m_simtkState);
        // TODO: Check if implicit mode and realizing to Acceleration.
        return 0;
    }

private:
    const OpenSim::MocoProblemRep& m_mocoProblemRep;
    const OpenSim::Model& m_model;
    mutable SimTK::State m_simtkState;
};

class MocoCasADiEndpointCost : public CasOC::EndpointCost {
public:
    MocoCasADiEndpointCost(const OpenSim::MocoProblemRep& problem)
            : m_mocoProblemRep(problem), m_model(problem.getModel()),
              m_simtkState(m_model.getWorkingState()) {}
    int eval(const double** inputs, double** outputs, casadi_int*, double*,
            void*) const override {
        applyParametersToModel(SimTK::Vector(m_casProblem->getNumParameters(),
                                       inputs[3], true),
                m_mocoProblemRep);
        m_simtkState.setTime(inputs[0][0]);
        // TODO: Use convertToSimTKState().
        std::copy_n(inputs[1], m_simtkState.getNY(),
                m_simtkState.updY().updContiguousScalarData());
        m_mocoProblemRep.getModel().updControls(m_simtkState).setToNaN();
        outputs[0][0] = m_mocoProblemRep.calcEndpointCost(m_simtkState);
        return 0;
    }

private:
    const OpenSim::MocoProblemRep& m_mocoProblemRep;
    const OpenSim::Model& m_model;
    mutable SimTK::State m_simtkState;
};

class MocoCasADiMultibodySystem : public CasOC::MultibodySystem {
public:
    MocoCasADiMultibodySystem(const OpenSim::MocoProblemRep& problem)
            : m_mocoProblemRep(problem), m_model(problem.getModel()),
              m_simtkState(m_model.getWorkingState()) {}
    int eval(const double** inputs, double** outputs, casadi_int*, double*,
            void*) const override {
        const double* time = inputs[0];
        const double* states = inputs[1];
        const double* controls = inputs[2];
        // TODO const double* multipliers = inputs[3];
        const double* parameters = inputs[4];
        double* out_multibody_derivatives = outputs[0];
        double* out_auxiliary_derivatives = outputs[1];
        // TODO double* out_kinematic_constraint_errors = outputs[2];
        applyParametersToModel(SimTK::Vector(m_casProblem->getNumParameters(),
                                       parameters, true),
                m_mocoProblemRep);
        convertToSimTKState(time, states, controls, m_model, m_simtkState);
        // If enabled constraints exist in the model, compute accelerations
        // based on Lagrange multipliers.

        m_mocoProblemRep.getModel().realizeAcceleration(m_simtkState);

        std::copy_n(m_simtkState.getUDot().getContiguousScalarData(),
                m_simtkState.getNU(), out_multibody_derivatives);
        std::copy_n(m_simtkState.getZDot().getContiguousScalarData(),
                m_simtkState.getNZ(), out_auxiliary_derivatives);
        return 0;
    }

private:
    const OpenSim::MocoProblemRep& m_mocoProblemRep;
    const OpenSim::Model& m_model;
    mutable SimTK::State m_simtkState;
};

} // namespace OpenSim

#endif // MOCO_MOCOCASADIMISC_H
