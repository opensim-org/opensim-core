#ifndef MOCO_MOCOCASADIBRIDGE_H
#define MOCO_MOCOCASADIBRIDGE_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoCasADiBridge.h                                           *
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

#include "../Components/AccelerationMotion.h"
#include "../Components/DiscreteForces.h"
#include "../MocoBounds.h"
#include "../MocoProblemRep.h"
#include "CasOCProblem.h"
#include "MocoCasADiSolver.h"

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

template <typename T>
casadi::DM convertToCasADiDMTemplate(const T& simtk) {
    casadi::DM out(casadi::Sparsity::dense(simtk.size(), 1));
    std::copy_n(simtk.getContiguousScalarData(), simtk.size(), out.ptr());
    return out;
}
/// This converts a SimTK::RowVector to a casadi::DM column vector.
inline casadi::DM convertToCasADiDMTranspose(const SimTK::RowVector& simtkRV) {
    return convertToCasADiDMTemplate(simtkRV);
}
/// This converts a SimTK::Vector to a casadi::DM column vector.
inline casadi::DM convertToCasADiDM(const SimTK::Vector& simtkVec) {
    return convertToCasADiDMTemplate(simtkVec);
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
    if (!mocoIt.getSlackNames().empty()) {
        casVars[Var::slacks] =
                convertToCasADiDMTranspose(mocoIt.getSlacksTrajectory());
    }
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
    casIt.slack_names = mocoIt.getSlackNames();
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
    SimTK::Matrix simtkSlacks;
    if (!casIt.slack_names.empty()) {
        const auto slacksValue = casVars.at(Var::slacks);
        simtkSlacks = convertToSimTKMatrix(slacksValue);
    }
    SimTK::Matrix simtkDerivatives;
    auto derivativeNames = casIt.derivative_names;
    if (casVars.count(Var::derivatives)) {
        const auto derivsValue = casVars.at(Var::derivatives);
        simtkDerivatives = convertToSimTKMatrix(derivsValue);
    } else {
        derivativeNames.clear();
    }
    SimTK::RowVector simtkParameters;
    if (!casIt.parameter_names.empty()) {
        const auto paramsValue = casVars.at(Var::parameters);
        simtkParameters = convertToSimTKVector<SimTK::RowVector>(paramsValue);
    }
    SimTK::Vector simtkTimes = convertToSimTKVector(casIt.times);

    TOut mocoIterate(simtkTimes, casIt.state_names, casIt.control_names,
            casIt.multiplier_names, derivativeNames, casIt.parameter_names,
            simtkStates, simtkControls, simtkMultipliers, simtkDerivatives,
            simtkParameters);

    // Append slack variables. MocoIterate requires the slack variables to be
    // the same length as its time vector, but it will not be if the
    // CasOC::Iterate was generated from a CasOC::Transcription object.
    // Therefore, slack variables are interpolated as necessary.
    if (!casIt.slack_names.empty()) {
        int simtkSlacksLength = simtkSlacks.nrow();
        SimTK::Vector slackTime = createVectorLinspace(simtkSlacksLength,
                simtkTimes[0], simtkTimes[simtkTimes.size() - 1]);
        for (int i = 0; i < (int)casIt.slack_names.size(); ++i) {
            if (simtkSlacksLength != simtkTimes.size()) {
                mocoIterate.appendSlack(casIt.slack_names[i],
                        interpolate(slackTime, simtkSlacks.col(i), simtkTimes));
            } else {
                mocoIterate.appendSlack(
                        casIt.slack_names[i], simtkSlacks.col(i));
            }
        }
    }
    return mocoIterate;
}

/// Apply parameters to properties in the models returned by
/// `mocoProblemRep.getModelBase()` and
/// `mocoProblemRep.getModelDisabledConstraints()`.
inline void applyParametersToModelProperties(
        const SimTK::Vector& parameters, const MocoProblemRep& mocoProblemRep) {
    if (parameters.size()) {
        mocoProblemRep.applyParametersToModelProperties(parameters, true);
    }
}

/// Copy values from `states` into `simtkState.updY()`, accounting for empty
/// slots in Simbody's Y vector.
/// It's fine for the size of `states` to be less than the size of Y; only the
/// first states.size1() values are copied.
inline void convertToSimTKState(const double& time, const casadi::DM& states,
        const Model& model, const std::unordered_map<int, int>& yIndexMap,
        SimTK::State& simtkState, bool setControlsToNaN = true) {
    simtkState.setTime(time);
    for (int isv = 0; isv < states.size1(); ++isv) {
        simtkState.updY()[yIndexMap.at(isv)] = *(states.ptr() + isv);
    }
    if (setControlsToNaN) model.updControls(simtkState).setToNaN();
}

inline void convertToSimTKState(const double& time, const casadi::DM& states,
        const casadi::DM& controls, const Model& model,
        const std::unordered_map<int, int>& yIndexMap,
        SimTK::State& simtkState) {
    convertToSimTKState(time, states, model, yIndexMap, simtkState, false);
    auto& simtkControls = model.updControls(simtkState);
    std::copy_n(controls.ptr(), simtkControls.size(),
            simtkControls.updContiguousScalarData());
    model.realizeVelocity(simtkState);
    model.setControls(simtkState, simtkControls);
}

// TODO: make this part of a class so we can reduce the number of arguments.
inline void calcKinematicConstraintForces(const casadi::DM& multipliers,
        const SimTK::State& stateBase, const Model& modelBase,
        const DiscreteForces& constraintForces,
        SimTK::Vector_<SimTK::SpatialVec>& constraintBodyForces,
        SimTK::Vector& constraintMobilityForces,
        SimTK::State& stateDisabledConstraints) {
    // Calculate the constraint forces using the original model and the
    // solver-provided Lagrange multipliers.
    modelBase.realizeVelocity(stateBase);
    const auto& matterBase = modelBase.getMatterSubsystem();
    SimTK::Vector simtkMultipliers(
            (int)multipliers.size1(), multipliers.ptr(), true);
    // Multipliers are negated so constraint forces can be used like
    // applied forces.
    matterBase.calcConstraintForcesFromMultipliers(stateBase, -simtkMultipliers,
            constraintBodyForces, constraintMobilityForces);

    // Apply the constraint forces on the model with disabled constraints.
    constraintForces.setAllForces(stateDisabledConstraints,
            constraintMobilityForces, constraintBodyForces);
}

// TODO: make this part of a class so we can reduce the number of arguments.
inline void calcKinematicConstraintErrors(const Model& modelBase,
        const SimTK::State& stateBase, const SimTK::Vector& udot,
        const CasOC::Problem* casProblem,
        const bool& enforceConstraintDerivatives, SimTK::Vector& pvaerr,
        VectorDM& out) {
    // The total number of scalar holonomic, non-holonomic, and acceleration
    // constraint equations enabled in the model. This does not count
    // equations for derivatives of holonomic and non-holonomic constraints.
    const int total_mp = casProblem->getNumHolonomicConstraintEquations();
    const int total_mv = casProblem->getNumNonHolonomicConstraintEquations();
    const int total_ma = casProblem->getNumAccelerationConstraintEquations();

    // Position-level errors.
    const auto& qerr = stateBase.getQErr();

    if (enforceConstraintDerivatives || total_ma) {
        // Calculuate udoterr. We cannot use State::getUDotErr()
        // because that uses Simbody's multiplilers and UDot,
        // whereas we have our own multipliers and UDot. Here, we use
        // the udot computed from the base model (with enabled constraints)
        // since we cannot use (nor do we have availabe) udot computed
        // from the original model.
        const auto& matter = modelBase.getMatterSubsystem();
        matter.calcConstraintAccelerationErrors(stateBase, udot, pvaerr);
    } else {
        pvaerr = SimTK::NaN;
    }

    const auto& uerr = stateBase.getUErr();
    int uerrOffset;
    int uerrSize;
    const auto& udoterr = pvaerr;
    int udoterrOffset;
    int udoterrSize;
    if (enforceConstraintDerivatives) {
        // Velocity-level errors.
        uerrOffset = 0;
        uerrSize = uerr.size();
        // Acceleration-level errors.
        udoterrOffset = 0;
        udoterrSize = udoterr.size();
    } else {
        // Velocity-level errors. Skip derivatives of position-level
        // constraint equations.
        uerrOffset = total_mp;
        uerrSize = total_mv;
        // Acceleration-level errors. Skip derivatives of velocity-
        // and position-level constraint equations.
        udoterrOffset = total_mp + total_mv;
        udoterrSize = total_ma;
    }

    // This way of copying the data avoids a threadsafety issue in
    // CasADi related to cached Sparsity objects.
    casadi::DM out_kinematic_constraint_errors = casadi::DM(
            casadi::Sparsity::dense(qerr.size() + uerrSize + udoterrSize, 1));
    std::copy_n(qerr.getContiguousScalarData(), qerr.size(),
            out_kinematic_constraint_errors.ptr());
    std::copy_n(uerr.getContiguousScalarData() + uerrOffset, uerrSize,
            out_kinematic_constraint_errors.ptr() + qerr.size());
    std::copy_n(udoterr.getContiguousScalarData() + udoterrOffset, udoterrSize,
            out_kinematic_constraint_errors.ptr() + qerr.size() + uerrSize);

    out.push_back(out_kinematic_constraint_errors);
}

class MocoCasADiPathConstraint : public CasOC::PathConstraint {
public:
    MocoCasADiPathConstraint(ThreadsafeJar<const MocoProblemRep>& jar,
            std::unordered_map<int, int> yIndexMap,
            const std::string& mocoPathConName)
            : m_jar(jar), m_yIndexMap(std::move(yIndexMap)),
              m_mocoPathConName(mocoPathConName) {}

    VectorDM eval(const VectorDM& args) const override {
        const double& time = args.at(0).scalar();
        const casadi::DM& states = args.at(1);
        const casadi::DM& controls = args.at(2);
        const casadi::DM& parameters = args.at(3);
        auto mocoProblemRep = m_jar.take();

        // Model with disabled constriants and its associated state. This model
        // is passed to MocoPathConstraints, so we must use it here.
        // TODO: deal with constness better.
        auto& modelDisabledConstraints = const_cast<Model&>(
                mocoProblemRep->getModelDisabledConstraints());
        auto& simTKStateDisabledConstraints =
                modelDisabledConstraints.updWorkingState();

        // Update the model and state.
        // TODO: This applies parameters to both models, rename for clarity.
        applyParametersToModelProperties(
                SimTK::Vector((int)parameters.size1(), parameters.ptr(), true),
                *mocoProblemRep);
        // TODO: Don't necessarily need to realize to Velocity.
        convertToSimTKState(time, states, controls, modelDisabledConstraints,
                m_yIndexMap, simTKStateDisabledConstraints);

        // Compute path constraint errors.
        // TODO avoid this lookup-by-name.
        const auto& mocoPathCon =
                mocoProblemRep->getPathConstraint(m_mocoPathConName);
        m_errors.resize(m_numEquations);
        mocoPathCon.calcPathConstraintErrors(
                simTKStateDisabledConstraints, m_errors);

        m_jar.leave(std::move(mocoProblemRep));
        return {convertToCasADiDM(m_errors)};
    }

private:
    ThreadsafeJar<const MocoProblemRep>& m_jar;
    std::unordered_map<int, int> m_yIndexMap;
    std::string m_mocoPathConName;
    static thread_local SimTK::Vector m_errors;
};

class MocoCasADiIntegralCostIntegrand : public CasOC::IntegralCostIntegrand {
public:
    MocoCasADiIntegralCostIntegrand(ThreadsafeJar<const MocoProblemRep>& jar,
            std::unordered_map<int, int> yIndexMap)
            : m_jar(jar), m_yIndexMap(std::move(yIndexMap)) {}
    VectorDM eval(const VectorDM& args) const override {
        const double& time = args.at(0).scalar();
        const casadi::DM& states = args.at(1);
        const casadi::DM& controls = args.at(2);
        const casadi::DM& parameters = args.at(3);
        auto mocoProblemRep = m_jar.take();

        // Model with disabled constriants and its associated state. This model
        // is passed to MocoCost terms, so we must use it here.
        // TODO: deal with constness better.
        auto& modelDisabledConstraints = const_cast<Model&>(
                mocoProblemRep->getModelDisabledConstraints());
        auto& simTKStateDisabledConstraints =
                modelDisabledConstraints.updWorkingState();

        // Update the model and state.
        // TODO: This applies parameters to both models, rename for clarity.
        applyParametersToModelProperties(
                SimTK::Vector((int)parameters.size1(), parameters.ptr(), true),
                *mocoProblemRep);
        convertToSimTKState(time, states, controls, modelDisabledConstraints,
                m_yIndexMap, simTKStateDisabledConstraints);

        // Compute the integrand for all MocoCosts.
        // TODO: Create separate functions for each cost term.
        casadi::DM output(1, 1);
        output(0, 0) =
                mocoProblemRep->calcIntegralCost(simTKStateDisabledConstraints);

        m_jar.leave(std::move(mocoProblemRep));
        // TODO: Check if implicit mode and realizing to Acceleration.
        return {output};
    }

private:
    ThreadsafeJar<const MocoProblemRep>& m_jar;
    std::unordered_map<int, int> m_yIndexMap;
};

class MocoCasADiEndpointCost : public CasOC::EndpointCost {
public:
    MocoCasADiEndpointCost(ThreadsafeJar<const MocoProblemRep>& jar,
            std::unordered_map<int, int> yIndexMap)
            : m_jar(jar), m_yIndexMap(std::move(yIndexMap)) {}
    VectorDM eval(const VectorDM& args) const override {
        const double& time = args.at(0).scalar();
        const casadi::DM& states = args.at(1);
        const casadi::DM& parameters = args.at(2);
        auto mocoProblemRep = m_jar.take();

        // Model with disabled constriants and its associated state. This model
        // is passed to MocoCost terms, so we must use it here.
        // TODO: deal with constness better.
        auto& modelDisabledConstraints = const_cast<Model&>(
                mocoProblemRep->getModelDisabledConstraints());
        auto& simTKStateDisabledConstraints =
                modelDisabledConstraints.updWorkingState();

        // Update the model and state.
        // TODO: This applies parameters to both models, rename for clarity.
        applyParametersToModelProperties(
                SimTK::Vector((int)parameters.size1(), parameters.ptr(), true),
                *mocoProblemRep);
        convertToSimTKState(time, states, modelDisabledConstraints, m_yIndexMap,
                simTKStateDisabledConstraints, true);

        // Compute the endpoint cost for all MocoCosts.
        casadi::DM output(1, 1);
        output(0, 0) =
                mocoProblemRep->calcEndpointCost(simTKStateDisabledConstraints);

        m_jar.leave(std::move(mocoProblemRep));
        return {output};
    }

private:
    ThreadsafeJar<const MocoProblemRep>& m_jar;
    std::unordered_map<int, int> m_yIndexMap;
};

template <bool CalcKCErrors>
class MocoCasADiMultibodySystem : public CasOC::MultibodySystem<CalcKCErrors> {
public:
    MocoCasADiMultibodySystem(ThreadsafeJar<const MocoProblemRep>& jar,
            const OpenSim::MocoCasADiSolver& solver,
            std::unordered_map<int, int> yIndexMap)
            : m_jar(jar), m_mocoCasADiSolver(solver),
              m_yIndexMap(std::move(yIndexMap)) {}

    VectorDM eval(const VectorDM& args) const override {
        const double& time = args.at(0).scalar();
        const casadi::DM& states = args.at(1);
        const casadi::DM& controls = args.at(2);
        const casadi::DM& multipliers = args.at(3);
        const casadi::DM& parameters = args.at(4);
        VectorDM out(2);
        auto mocoProblemRep = m_jar.take();

        // Original model and its associated state. These are used to calculate
        // kinematic constraint forces and errors.
        const auto& modelBase = mocoProblemRep->getModelBase();
        auto& simtkStateBase = mocoProblemRep->updStateBase();

        // Model with disabled constraints and its associated state. These are
        // used to compute the accelerations.
        const auto& modelDisabledConstraints =
                mocoProblemRep->getModelDisabledConstraints();
        auto& simtkStateDisabledConstraints =
                mocoProblemRep->updStateDisabledConstraints();

        // Update the model and state.
        applyParametersToModelProperties(
                SimTK::Vector((int)parameters.size1(), parameters.ptr(), true),
                *mocoProblemRep);
        convertToSimTKState(
                time, states, controls, modelBase, m_yIndexMap, simtkStateBase);
        convertToSimTKState(time, states, controls, modelDisabledConstraints,
                m_yIndexMap, simtkStateDisabledConstraints);
        // If enabled constraints exist in the model, compute constraint forces
        // based on Lagrange multipliers. This also updates the associated
        // discrete variables in the state.
        const int numMultipliers = this->m_casProblem->getNumMultipliers();
        if (numMultipliers) {
            calcKinematicConstraintForces(multipliers, simtkStateBase,
                    modelBase,
                    mocoProblemRep->getConstraintForces(),
                    m_constraintBodyForces, m_constraintMobilityForces,
                    simtkStateDisabledConstraints);
        }

        // Compute the accelerations.
        modelDisabledConstraints.realizeAcceleration(
                simtkStateDisabledConstraints);

        // Compute kinematic constraint errors if they exist.
        if (numMultipliers && CalcKCErrors) {
            calcKinematicConstraintErrors(modelBase, simtkStateBase,
                    simtkStateDisabledConstraints.getUDot(), this->m_casProblem,
                    m_mocoCasADiSolver.get_enforce_constraint_derivatives(),
                    m_pvaerr, out);
        }

        // Copy state derivative values to output.
        out[0] = convertToCasADiDM(simtkStateDisabledConstraints.getUDot());
        out[1] = convertToCasADiDM(simtkStateDisabledConstraints.getZDot());

        // This path should never be reached during an optimization, but
        // CasADi will throw an error (likely while constructing the
        // expression graph) if this path doesn't provide the correct
        // size output.
        if (!numMultipliers && CalcKCErrors) {
            // Add an empty kinematic constraint error vector.
            out.emplace_back(0, 1);
        }

        m_jar.leave(std::move(mocoProblemRep));

        return out;
    }

private:
    ThreadsafeJar<const MocoProblemRep>& m_jar;
    const OpenSim::MocoCasADiSolver& m_mocoCasADiSolver;
    std::unordered_map<int, int> m_yIndexMap;
    // Local memory to hold constraint forces.
    static thread_local SimTK::Vector_<SimTK::SpatialVec>
        m_constraintBodyForces;
    static thread_local SimTK::Vector m_constraintMobilityForces;
    // This is the output argument of
    // SimbodyMatterSubsystem::calcConstraintAccelerationErrors(), and includes
    // the acceleration-level holonomic, non-holonomic constraint errors and the
    // acceleration-only constraint errors.
    static thread_local SimTK::Vector m_pvaerr;
};

class MocoCasADiVelocityCorrection : public CasOC::VelocityCorrection {
public:
    MocoCasADiVelocityCorrection(ThreadsafeJar<const MocoProblemRep>& jar,
            std::unordered_map<int, int> yIndexMap)
            : m_jar(jar), m_yIndexMap(std::move(yIndexMap)) {}
    VectorDM eval(const VectorDM& args) const override {
        const double& time = args.at(0).scalar();
        const casadi::DM& multibody_states = args.at(1);
        const casadi::DM& slacks = args.at(2);
        const casadi::DM& parameters = args.at(3);
        auto mocoProblemRep = m_jar.take();

        const auto& modelBase = mocoProblemRep->getModelBase();
        auto& simtkStateBase = mocoProblemRep->updStateBase();

        // Update the model and state.
        applyParametersToModelProperties(
                SimTK::Vector((int)parameters.size1(), parameters.ptr(), true),
                *mocoProblemRep);
        convertToSimTKState(time, multibody_states, modelBase, m_yIndexMap,
                simtkStateBase, false);
        modelBase.realizeVelocity(simtkStateBase);

        // Apply velocity correction to qdot if at a mesh interval midpoint.
        // This correction modifies the dynamics to enable a projection of
        // the model coordinates back onto the constraint manifold whenever
        // they deviate.
        // Posa, Kuindersma, Tedrake, 2016. "Optimization and stabilization
        // of trajectories for constrained dynamical systems"
        // Note: Only supported for the Hermite-Simpson transcription
        // scheme.
        const SimTK::SimbodyMatterSubsystem& matterBase =
                modelBase.getMatterSubsystem();

        SimTK::Vector gamma(
                this->m_casProblem->getNumSlacks(), slacks.ptr(), true);
        matterBase.multiplyByGTranspose(simtkStateBase, gamma, m_qdotCorr);
        m_jar.leave(std::move(mocoProblemRep));

        casadi::DM velocity_correction;
        velocity_correction = convertToCasADiDM(SimTK::Vector(
                m_qdotCorr.size(), m_qdotCorr.getContiguousScalarData(), true));

        return {velocity_correction};
    }

private:
    ThreadsafeJar<const MocoProblemRep>& m_jar;
    std::unordered_map<int, int> m_yIndexMap;
    static thread_local SimTK::Vector m_qdotCorr;
};

template <bool CalcKCErrors>
class MocoCasADiMultibodySystemImplicit
        : public CasOC::MultibodySystemImplicit<CalcKCErrors> {
public:
    MocoCasADiMultibodySystemImplicit(ThreadsafeJar<const MocoProblemRep>& jar,
            const OpenSim::MocoCasADiSolver& solver,
            std::unordered_map<int, int> yIndexMap)
            : m_jar(jar), m_mocoCasADiSolver(solver),
              m_yIndexMap(std::move(yIndexMap)) {}
    VectorDM eval(const VectorDM& args) const override {
        const double& time = args.at(0).scalar();
        const casadi::DM& states = args.at(1);
        const casadi::DM& controls = args.at(2);
        const casadi::DM& multipliers = args.at(3);
        const casadi::DM& derivatives = args.at(4);
        const casadi::DM& parameters = args.at(5);
        VectorDM out(2);
        auto mocoProblemRep = m_jar.take();

        // Original model and its associated state. These are used to calculate
        // kinematic constraint forces and errors.
        // TODO: deal with constness better.
        const auto& modelBase = mocoProblemRep->getModelBase();
        auto& simtkStateBase = mocoProblemRep->updStateBase();

        // Model with disabled constriants and its associated state. These are
        // used to compute the accelerations.
        const auto& modelDisabledConstraints =
                mocoProblemRep->getModelDisabledConstraints();
        auto& simtkStateDisabledConstraints =
                mocoProblemRep->updStateDisabledConstraints();
        auto& accel = mocoProblemRep->getAccelerationMotion();
        accel.setEnabled(simtkStateDisabledConstraints, true);

        // Update the model and state.
        applyParametersToModelProperties(
                SimTK::Vector((int)parameters.size1(), parameters.ptr(), true),
                *mocoProblemRep);
        convertToSimTKState(
                time, states, controls, modelBase, m_yIndexMap, simtkStateBase);
        convertToSimTKState(time, states, controls, modelDisabledConstraints,
                m_yIndexMap, simtkStateDisabledConstraints);
        // If enabled constraints exist in the model, compute constraint forces
        // based on Lagrange multipliers. This also updates the associated
        // discrete variables in the state.
        const int numMultipliers = this->m_casProblem->getNumMultipliers();
        if (numMultipliers) {
            calcKinematicConstraintForces(multipliers, simtkStateBase,
                    modelBase,
                    mocoProblemRep->getConstraintForces(),
                    m_constraintBodyForces, m_constraintMobilityForces,
                    simtkStateDisabledConstraints);
        }

        SimTK::Vector udot((int)derivatives.size1(), derivatives.ptr(), true);
        accel.setUDot(simtkStateDisabledConstraints, udot);

        // Compute kinematic constraint errors if they exist.
        if (numMultipliers && CalcKCErrors) {
            calcKinematicConstraintErrors(modelBase, simtkStateBase, udot,
                    this->m_casProblem,
                    m_mocoCasADiSolver.get_enforce_constraint_derivatives(),
                    m_pvaerr, out);
        }

        const SimTK::SimbodyMatterSubsystem& matterDisabledConstraints =
                modelDisabledConstraints.getMatterSubsystem();
        modelDisabledConstraints.realizeAcceleration(
                simtkStateDisabledConstraints);
        out[0] = casadi::DM(casadi::Sparsity::dense(udot.size(), 1));
        SimTK::Vector simtkResidual(udot.size(), out[0].ptr(), true);
        matterDisabledConstraints.findMotionForces(
                simtkStateDisabledConstraints, simtkResidual);

        // Copy auxiliary dynamics to output.
        out[1] = convertToCasADiDM(simtkStateDisabledConstraints.getZDot());

        // This path should never be reached during an optimization, but
        // CasADi will throw an error (likely while constructing the
        // expression graph) if this path doesn't provide the correct
        // size output.
        if (!numMultipliers && CalcKCErrors) {
            // Add an empty kinematic constraint error vector.
            out.emplace_back(0, 1);
        }

        m_jar.leave(std::move(mocoProblemRep));

        return out;
    }

private:
    ThreadsafeJar<const MocoProblemRep>& m_jar;
    const OpenSim::MocoCasADiSolver& m_mocoCasADiSolver;
    std::unordered_map<int, int> m_yIndexMap;
    static thread_local SimTK::Vector m_residual;
    // Local memory to hold constraint forces.
    static thread_local SimTK::Vector_<SimTK::SpatialVec>
       m_constraintBodyForces;
    static thread_local SimTK::Vector m_constraintMobilityForces;
    // This is the output argument of
    // SimbodyMatterSubsystem::calcConstraintAccelerationErrors(), and includes
    // the acceleration-level holonomic, non-holonomic constraint errors and the
    // acceleration-only constraint errors.
    static thread_local SimTK::Vector m_pvaerr;
};

} // namespace OpenSim

#endif // MOCO_MOCOCASADIBRIDGE_H
