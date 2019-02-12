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
#include "../MocoCasADiSolver/MocoCasADiSolver.h"
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

/// This converts a SimTK::Matrix to a casadi::DM matrix, transposing the
/// data in the process.
inline casadi::DM convertToCasADiDM(const SimTK::Matrix& simtkMatrix) {
    casadi::DM out(simtkMatrix.ncol(), simtkMatrix.nrow());
    for (int irow = 0; irow < simtkMatrix.nrow(); ++irow) {
        for (int icol = 0; icol < simtkMatrix.ncol(); ++icol) {
            out(icol, irow) = simtkMatrix(irow, icol);
        }
    }
    return out;
}
/// This converts a SimTK::RowVector to a casadi::DM column vector.
inline casadi::DM convertToCasADiDM(const SimTK::RowVector& simtkRV) {
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
    casVars[Var::states] = convertToCasADiDM(mocoIt.getStatesTrajectory());
    casVars[Var::controls] = convertToCasADiDM(mocoIt.getControlsTrajectory());
    casVars[Var::multipliers] =
            convertToCasADiDM(mocoIt.getMultipliersTrajectory());
    if (!mocoIt.getSlackNames().empty()) {
        casVars[Var::slacks] = convertToCasADiDM(mocoIt.getSlacksTrajectory());
    }
    if (!mocoIt.getDerivativeNames().empty()) {
        casVars[Var::derivatives] =
                convertToCasADiDM(mocoIt.getDerivativesTrajectory());
    }
    casVars[Var::parameters] = convertToCasADiDM(mocoIt.getParameters());
    casIt.times = convertToCasADiDM(mocoIt.getTime());
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
    // Append slack variables.
    for (int i = 0; i < casIt.slack_names.size(); ++i) {
        SimTK::Vector slackTime = createVectorLinspace(
            simtkSlacks.col(i).size(), simtkTimes[0], 
            simtkTimes[simtkTimes.size()-1]);
        std::cout << "currSlack: " << simtkSlacks.col(i) << std::endl;

        SimTK::Vector currSlackInterp = interpolate(slackTime, simtkSlacks.col(i), simtkTimes);
        std::cout << "currSlackInterp: " << currSlackInterp << std::endl;

        mocoIterate.appendSlack(casIt.slack_names[i], currSlackInterp);
    }
    return mocoIterate;
}

inline void applyParametersToModel(
        const SimTK::Vector& parameters, const MocoProblemRep& mocoProblemRep) {
    if (parameters.size()) {
        mocoProblemRep.applyParametersToModel(parameters);
        const_cast<Model&>(mocoProblemRep.getModel()).initSystem();
    }
}

inline void convertToSimTKState(const double* time, const double* states,
        const Model& model, SimTK::State& simtkState,
        bool setControlsToNaN = true) {
    OPENSIM_THROW_IF(simtkState.getNQ() != simtkState.getNU(),
            OpenSim::Exception, "NQ != NU, copying state is incorrect.");
    simtkState.setTime(time[0]);
    std::copy_n(states, simtkState.getNY(),
            simtkState.updY().updContiguousScalarData());
    if (setControlsToNaN) model.updControls(simtkState).setToNaN();
}

inline void convertToSimTKState(const double* time, const double* states,
        const double* controls, const Model& model, SimTK::State& simtkState) {
    OPENSIM_THROW_IF(simtkState.getNQ() != simtkState.getNU(),
            OpenSim::Exception, "NQ != NU, copying state is incorrect.");
    convertToSimTKState(time, states, model, simtkState, false);
    auto& simtkControls = model.updControls(simtkState);
    std::copy_n(controls, simtkControls.size(),
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

    int eval(const double** inputs, double** outputs, casadi_int*, double*,
            void*) const {
        applyParametersToModel(SimTK::Vector(m_casProblem->getNumParameters(),
                                       inputs[3], true),
                m_mocoProblemRep);
        // TODO: Don't necessarily need to realize to Velocity.
        convertToSimTKState(
                inputs[0], inputs[1], inputs[2], m_model, m_simtkState);
        errors.resize(m_numEquations);
        m_mocoPathCon.calcPathConstraintErrors(m_simtkState, errors);
        std::copy_n(
                errors.getContiguousScalarData(), errors.size(), outputs[0]);
        return 0;
    }

private:
    const OpenSim::MocoProblemRep& m_mocoProblemRep;
    const OpenSim::Model& m_model;
    mutable SimTK::State m_simtkState;
    const MocoPathConstraint& m_mocoPathCon;
    mutable SimTK::Vector errors;
};

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
                                       inputs[2], true),
                m_mocoProblemRep);
        convertToSimTKState(inputs[0], inputs[1], m_model, m_simtkState, true);
        outputs[0][0] = m_mocoProblemRep.calcEndpointCost(m_simtkState);
        return 0;
    }

private:
    const OpenSim::MocoProblemRep& m_mocoProblemRep;
    const OpenSim::Model& m_model;
    mutable SimTK::State m_simtkState;
};

template <bool CalcKinConErrors>
class MocoCasADiMultibodySystem : public CasOC::MultibodySystem<CalcKinConErrors> {
public:
    MocoCasADiMultibodySystem(const OpenSim::MocoProblemRep& problem,
        const OpenSim::MocoCasADiSolver& solver)
            : m_mocoProblemRep(problem), m_mocoCasADiSolver(solver),
              m_model(problem.getModel()),
              m_simtkState(m_model.getWorkingState()) {}
    int eval(const double** inputs, double** outputs, casadi_int*, double*,
            void*) const override {
        const double* time = inputs[0];
        const double* states = inputs[1];
        const double* controls = inputs[2];
        const double* multipliers = inputs[3];
        const double* parameters = inputs[4];
        double* out_multibody_derivatives = outputs[0];
        double* out_auxiliary_derivatives = outputs[1];
        applyParametersToModel(SimTK::Vector(m_casProblem->getNumParameters(),
                                       parameters, true),
                m_mocoProblemRep);
        //convertToSimTKState(time, states, controls, m_model, m_simtkState);

        m_simtkState.setTime(time[0]);
        std::copy_n(states, m_simtkState.getNY(),
            m_simtkState.updY().updContiguousScalarData());
        auto& simtkControls = m_model.updControls(m_simtkState);
        std::copy_n(controls, simtkControls.size(),
            simtkControls.updContiguousScalarData());
        m_model.realizeVelocity(m_simtkState);
        m_model.setControls(m_simtkState, simtkControls);

        // If enabled constraints exist in the model, compute accelerations
        // based on Lagrange multipliers.
        m_total_mp = m_casProblem->getNumHolonomicConstraintEquations();
        m_total_mv = m_casProblem->getNumNonHolonomicConstraintEquations();
        m_total_ma = m_casProblem->getNumAccelerationConstraintEquations();
        m_numMultipliers = m_casProblem->getNumMultipliers();
        if (m_numMultipliers) {
            const auto& enforceConstraintDerivatives =
                m_mocoCasADiSolver.get_enforce_constraint_derivatives();
            
            m_mocoProblemRep.getModel().realizeDynamics(m_simtkState);

            const SimTK::MultibodySystem& multibody =
                m_model.getMultibodySystem();
            const SimTK::Vector_<SimTK::SpatialVec>& appliedBodyForces =
                multibody.getRigidBodyForces(m_simtkState,
                    SimTK::Stage::Dynamics);
            const SimTK::Vector& appliedMobilityForces =
                multibody.getMobilityForces(m_simtkState,
                    SimTK::Stage::Dynamics);

            const SimTK::SimbodyMatterSubsystem& matter =
                m_model.getMatterSubsystem();

            // Multipliers are negated so constraint forces can be used like
            // applied forces.
            SimTK::Vector simtkMultipliers(m_numMultipliers, multipliers, true);
            matter.calcConstraintForcesFromMultipliers(m_simtkState, 
                -simtkMultipliers, constraintBodyForces,
                constraintMobilityForces);

            matter.calcAccelerationIgnoringConstraints(m_simtkState,
                appliedMobilityForces + constraintMobilityForces,
                appliedBodyForces + constraintBodyForces,
                udot, A_GB);

            // Constraint errors.
            // TODO double-check that disabled constraints don't show up in
            // state
            if (CalcKinConErrors) {
                double* out_kinematic_constraint_errors = outputs[2];
                // Position-level errors.
                std::copy_n(m_simtkState.getQErr().getContiguousScalarData(),
                    m_total_mp, out_kinematic_constraint_errors);

                if (enforceConstraintDerivatives || m_total_ma) {
                    // Calculuate udoterr. We cannot use State::getUDotErr()
                    // because that uses Simbody's multiplilers and UDot,
                    // whereas we have our own multipliers and UDot.
                    matter.calcConstraintAccelerationErrors(m_simtkState,
                        udot, m_pvaerr);
                } else {
                    m_pvaerr = SimTK::NaN;
                }

                if (enforceConstraintDerivatives) {
                    // Velocity-level errors.
                    std::copy_n(
                        m_simtkState.getUErr().getContiguousScalarData(),
                        m_total_mp + m_total_mv,
                        out_kinematic_constraint_errors + m_total_mp);
                    // Acceleration-level errors.
                    std::copy_n(
                        m_pvaerr.getContiguousScalarData(),
                        m_total_mp + m_total_mv + m_total_ma,
                        out_kinematic_constraint_errors + 2*m_total_mp 
                            + m_total_mv);
                } else {
                    // Velocity-level errors. Skip derivatives of position-level
                    // constraint equations.
                    std::copy_n(
                        m_simtkState.getUErr().getContiguousScalarData()
                            + m_total_mp, m_total_mv,
                            out_kinematic_constraint_errors + m_total_mp);
                    // Acceleration-level errors. Skip derivatives of velocity-
                    // and position-level constraint equations.
                    std::copy_n(
                        m_pvaerr.getContiguousScalarData() +
                        m_total_mp + m_total_mv, m_total_ma,
                        out_kinematic_constraint_errors + m_total_mp + 
                            m_total_mv);
                }
            }
            

            // Copy state derivative values to output struct. We cannot simply
            // use getYDot() because that requires realizing to Acceleration.
            const int nq = m_simtkState.getQ().size();
            const int nu = udot.size();
            const int nz = m_simtkState.getZ().size();
            std::copy_n(udot.getContiguousScalarData(),
                udot.size(), out_multibody_derivatives + nq);
            std::copy_n(m_simtkState.getZDot().getContiguousScalarData(), nz,
                out_auxiliary_derivatives + nq + nu);
        } else {
            m_mocoProblemRep.getModel().realizeAcceleration(m_simtkState);

            std::copy_n(m_simtkState.getUDot().getContiguousScalarData(),
                m_simtkState.getNU(), out_multibody_derivatives);
            std::copy_n(m_simtkState.getZDot().getContiguousScalarData(),
                m_simtkState.getNZ(), out_auxiliary_derivatives);
        }


        return 0;
    }

private:
    const OpenSim::MocoProblemRep& m_mocoProblemRep;
    const OpenSim::MocoCasADiSolver& m_mocoCasADiSolver;
    const OpenSim::Model& m_model;
    mutable SimTK::State m_simtkState;
    // This member variable avoids unnecessary extra allocation of memory for
    // spatial accelerations, which are incidental to the computation of
    // generalized accelerations when specifying the dynamics with model
    // constraints present.
    mutable SimTK::Vector_<SimTK::SpatialVec> constraintBodyForces;
    mutable SimTK::Vector constraintMobilityForces;
    mutable SimTK::Vector udot;
    // The total number of scalar holonomic, non-holonomic, and acceleration 
    // constraint equations enabled in the model. This does not count equations                                                                     
    // for derivatives of holonomic and non-holonomic constraints. 
    mutable int m_total_mp = 0;
    mutable int m_total_mv = 0;
    mutable int m_total_ma = 0;
    // This is the sum of m_total_m(p|v|a).
    mutable int m_numMultipliers = 0;
    mutable SimTK::Vector_<SimTK::SpatialVec> A_GB;
    // This is the output argument of
    // SimbodyMatterSubsystem::calcConstraintAccelerationErrors(), and includes
    // the acceleration-level holonomic, non-holonomic constraint errors and the
    // acceleration-only constraint errors.
    mutable SimTK::Vector m_pvaerr;
};

class MocoCasADiVelocityCorrection : public CasOC::VelocityCorrection {
public:
    MocoCasADiVelocityCorrection(const OpenSim::MocoProblemRep& problem)
        : m_mocoProblemRep(problem), m_model(problem.getModel()),
        m_simtkState(m_model.getWorkingState()) {}
    int eval(const double** inputs, double** outputs, casadi_int*, double*,
        void*) const override {
        
        // TODO: would the velocity correction ever be parameter dependent?
        const double* time = inputs[0];
        const double* states = inputs[1];
        const double* slacks = inputs[2];
        double* velocity_correction = outputs[0];

        m_simtkState.setTime(time[0]);
        std::copy_n(states, m_simtkState.getNY(),
            m_simtkState.updY().updContiguousScalarData());
        m_model.realizeVelocity(m_simtkState);

        // Apply velocity correction to qdot if at a mesh interval midpoint.
        // This correction modifies the dynamics to enable a projection of
        // the model coordinates back onto the constraint manifold whenever
        // they deviate.
        // Posa, Kuindersma, Tedrake, 2016. "Optimization and stabilization
        // of trajectories for constrained dynamical systems"
        // Note: Only supported for the Hermite-Simpson transcription 
        // scheme.
        // TODO: This assumes that velocity correction variables are applied
        // at all collocation points on the mesh interval interior. We know
        // this because kinematic constraint errors are currently only
        // computed at mesh points.
        const SimTK::SimbodyMatterSubsystem& matter =
            m_model.getMatterSubsystem();

        SimTK::Vector gamma(m_casProblem->getNumSlacks(), slacks);
        matter.multiplyByGTranspose(m_simtkState, gamma, qdotCorr);

        std::copy_n(qdotCorr.getContiguousScalarData(),
            m_simtkState.getNQ(), velocity_correction);
        return 0;
    }

private:
    const OpenSim::MocoProblemRep& m_mocoProblemRep;
    const OpenSim::Model& m_model;
    mutable SimTK::State m_simtkState;
    mutable SimTK::Vector qdotCorr;

};

} // namespace OpenSim

#endif // MOCO_MOCOCASADIMISC_H
