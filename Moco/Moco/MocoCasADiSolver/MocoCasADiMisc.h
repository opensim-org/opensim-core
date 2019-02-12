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
#include "../MocoProblemRep.h"
#include "CasOCProblem.h"

namespace OpenSim {

using VectorDM = std::vector<casadi::DM>;

inline CasOC::Bounds convertBounds(const MocoBounds& mb) {
    return {mb.getLower(), mb.getUpper()};
}
inline CasOC::Bounds convertBounds(const MocoInitialBounds& mb) {
    return {mb.getLower(), mb.getUpper()};
}
inline CasOC::Bounds convertBounds(const MocoFinalBounds& mb) {
    return {mb.getLower(), mb.getUpper()};
}

/// This converts a SimTK::Matrix to a casadi::DM matrix, transposing the
/// data in the process.
inline casadi::DM convertToCasADiDMTranspose(const SimTK::Matrix& simtkMatrix) {
    casadi::DM out(simtkMatrix.ncol(), simtkMatrix.nrow());
    for (int irow = 0; irow < simtkMatrix.nrow(); ++irow) {
        for (int icol = 0; icol < simtkMatrix.ncol(); ++icol) {
            out(icol, irow) = simtkMatrix(irow, icol);
        }
    }
    return out;
}
/// This converts a SimTK::RowVector to a casadi::DM column vector.
inline casadi::DM convertToCasADiDMTranspose(const SimTK::RowVector& simtkRV) {
    casadi::DM out(simtkRV.size(), 1);
    for (int i = 0; i < simtkRV.size(); ++i) { out(i) = simtkRV[i]; }
    return out;
}
/// This converts a SimTK::Vector to a casadi::DM column vector.
inline casadi::DM convertToCasADiDM(const SimTK::Vector& simtkRV) {
    casadi::DM out(simtkRV.size(), 1);
    for (int i = 0; i < simtkRV.size(); ++i) { out(i) = simtkRV[i]; }
    return out;
}

/// This resamples the iterate to obtain values that lie on the mesh.
inline CasOC::Iterate convertToCasOCIterate(const MocoIterate& mocoIt) {
    CasOC::Iterate casIt;
    CasOC::VariablesDM& casVars = casIt.variables;
    using CasOC::Var;
    casVars[Var::initial_time] = mocoIt.getInitialTime();
    casVars[Var::final_time] = mocoIt.getFinalTime();
    casVars[Var::states] =
            convertToCasADiDMTranspose(mocoIt.getStatesTrajectory());
    casVars[Var::controls] =
            convertToCasADiDMTranspose(mocoIt.getControlsTrajectory());
    casVars[Var::multipliers] =
            convertToCasADiDMTranspose(mocoIt.getMultipliersTrajectory());
    if (!mocoIt.getDerivativeNames().empty()) {
        casVars[Var::derivatives] =
                convertToCasADiDMTranspose(mocoIt.getDerivativesTrajectory());
    }
    casVars[Var::parameters] =
            convertToCasADiDMTranspose(mocoIt.getParameters());
    casIt.times = convertToCasADiDMTranspose(mocoIt.getTime());
    casIt.state_names = mocoIt.getStateNames();
    casIt.control_names = mocoIt.getControlNames();
    casIt.multiplier_names = mocoIt.getMultiplierNames();
    casIt.derivative_names = mocoIt.getDerivativeNames();
    casIt.parameter_names = mocoIt.getParameterNames();
    return casIt;
}

template <typename VectorType = SimTK::Vector>
VectorType convertToSimTKVector(const casadi::DM& casVector) {
    OPENSIM_THROW_IF(casVector.columns() != 1 && casVector.rows() != 1,
            Exception,
            format("casVector should be 1-dimensional, but has size %i x "
                   "%i.",
                    casVector.rows(), casVector.columns()));
    VectorType simtkVector((int)casVector.numel());
    for (int i = 0; i < casVector.numel(); ++i) {
        simtkVector[i] = double(casVector(i));
    }
    return simtkVector;
}

/// This converts a casadi::DM matrix to a
/// SimTK::Matrix, transposing the data in the process.
inline SimTK::Matrix convertToSimTKMatrix(const casadi::DM& casMatrix) {
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
    SimTK::RowVector simtkParameters;
    if (!casIt.parameter_names.empty()) {
        const auto paramsValue = casVars.at(Var::parameters);
        simtkParameters = convertToSimTKVector<SimTK::RowVector>(paramsValue);
    }

    SimTK::Vector simtkTimes = convertToSimTKVector(casIt.times);

    TOut mocoIterate(simtkTimes, casIt.state_names, casIt.control_names,
            casIt.multiplier_names, casIt.derivative_names,
            casIt.parameter_names, simtkStates, simtkControls, simtkMultipliers,
            simtkDerivatives, simtkParameters);
    return mocoIterate;
}

inline void applyParametersToModel(
        const SimTK::Vector& parameters, const MocoProblemRep& mocoProblemRep) {
    if (parameters.size()) {
        mocoProblemRep.applyParametersToModel(parameters);
        const_cast<Model&>(mocoProblemRep.getModel()).initSystem();
    }
}

inline void convertToSimTKState(const double& time, const casadi::DM& states,
        const Model& model, SimTK::State& simtkState,
        bool setControlsToNaN = true) {
    OPENSIM_THROW_IF(simtkState.getNQ() != simtkState.getNU(),
            OpenSim::Exception, "NQ != NU, copying state is incorrect.");
    simtkState.setTime(time);
    std::copy_n(states.ptr(), simtkState.getNY(),
            simtkState.updY().updContiguousScalarData());
    if (setControlsToNaN) model.updControls(simtkState).setToNaN();
}

inline void convertToSimTKState(const double& time, const casadi::DM& states,
        const casadi::DM& controls, const Model& model,
        SimTK::State& simtkState) {
    OPENSIM_THROW_IF(simtkState.getNQ() != simtkState.getNU(),
            OpenSim::Exception, "NQ != NU, copying state is incorrect.");
    convertToSimTKState(time, states, model, simtkState, false);
    auto& simtkControls = model.updControls(simtkState);
    std::copy_n(controls.ptr(), simtkControls.size(),
            simtkControls.updContiguousScalarData());
    model.realizeVelocity(simtkState);
    model.setControls(simtkState, simtkControls);
}

class AccelerationStageNotAllowed : public OpenSim::Exception {
public:
    using Exception::Exception;
};

class MocoCasADiPathConstraint : public CasOC::PathConstraint {
public:
    MocoCasADiPathConstraint(const OpenSim::MocoProblemRep& problem,
            const OpenSim::MocoPathConstraint& mocoPathConstraint)
            : m_mocoProblemRep(problem), m_model(problem.getModel()),
              m_simtkState(m_model.getWorkingState()),
              m_mocoPathCon(mocoPathConstraint) {}

    VectorDM eval(const VectorDM& args) const override {
        const double& time = args.at(0).scalar();
        const casadi::DM& states = args.at(1);
        const casadi::DM& controls = args.at(2);
        const casadi::DM& parameters = args.at(3);
        applyParametersToModel(SimTK::Vector(m_casProblem->getNumParameters(),
                                       parameters.ptr(), true),
                m_mocoProblemRep);
        // TODO: Don't necessarily need to realize to Velocity.
        convertToSimTKState(time, states, controls, m_model, m_simtkState);
        m_errors.resize(m_numEquations);
        m_mocoPathCon.calcPathConstraintErrors(m_simtkState, m_errors);
        return {convertToCasADiDM(m_errors)};
    }

private:
    const OpenSim::MocoProblemRep& m_mocoProblemRep;
    const OpenSim::Model& m_model;
    mutable SimTK::State m_simtkState;
    const MocoPathConstraint& m_mocoPathCon;
    mutable SimTK::Vector m_errors;
};

class MocoCasADiIntegralCostIntegrand : public CasOC::IntegralCostIntegrand {
public:
    MocoCasADiIntegralCostIntegrand(const OpenSim::MocoProblemRep& problem)
            : m_mocoProblemRep(problem), m_model(problem.getModel()),
              m_simtkState(m_model.getWorkingState()) {}
    VectorDM eval(const VectorDM& args) const override {
        const double& time = args.at(0).scalar();
        const casadi::DM& states = args.at(1);
        const casadi::DM& controls = args.at(2);
        const casadi::DM& parameters = args.at(3);
        applyParametersToModel(SimTK::Vector(m_casProblem->getNumParameters(),
                                       parameters.ptr(), true),
                m_mocoProblemRep);
        convertToSimTKState(time, states, controls, m_model, m_simtkState);
        // TODO: Create separate functions for each cost term.
        casadi::DM output(1, 1);
        output(0, 0) = m_mocoProblemRep.calcIntegralCost(m_simtkState);
        // TODO: Check if implicit mode and realizing to Acceleration.
        return {output};
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
    VectorDM eval(const VectorDM& args) const override {
        const double& time = args.at(0).scalar();
        const casadi::DM& states = args.at(1);
        const casadi::DM& params = args.at(2);
        applyParametersToModel(SimTK::Vector(m_casProblem->getNumParameters(),
                                       params.ptr(), true),
                m_mocoProblemRep);
        convertToSimTKState(time, states, m_model, m_simtkState, true);
        casadi::DM output(1, 1);
        output(0, 0) = m_mocoProblemRep.calcEndpointCost(m_simtkState);
        return {output};
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
    VectorDM eval(const VectorDM& args) const override {
        const double& time = args.at(0).scalar();
        const casadi::DM& states = args.at(1);
        const casadi::DM& controls = args.at(2);
        // TODO const double* multipliers = args.at(3);
        const casadi::DM& parameters = args.at(4);
        applyParametersToModel(SimTK::Vector(m_casProblem->getNumParameters(),
                                       parameters.ptr(), true),
                m_mocoProblemRep);
        convertToSimTKState(time, states, controls, m_model, m_simtkState);
        // If enabled constraints exist in the model, compute accelerations
        // based on Lagrange multipliers.

        m_mocoProblemRep.getModel().realizeAcceleration(m_simtkState);

        const auto out_multibody_derivatives =
                convertToCasADiDM(m_simtkState.getUDot());
        const auto out_auxiliary_derivatives =
                convertToCasADiDM(m_simtkState.getZDot());

        return {out_multibody_derivatives, out_auxiliary_derivatives,
                casadi::DM(0, 1)};
    }

private:
    const OpenSim::MocoProblemRep& m_mocoProblemRep;
    const OpenSim::Model& m_model;
    mutable SimTK::State m_simtkState;
};

} // namespace OpenSim

#endif // MOCO_MOCOCASADIMISC_H
