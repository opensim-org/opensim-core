#ifndef OPENSIM_TROPTERPROBLEM_H
#define OPENSIM_TROPTERPROBLEM_H
/* -------------------------------------------------------------------------- *
 * OpenSim: TropterProblem.h                                                  *
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

#include <simbody/internal/Constraint.h>

#include <OpenSim/Moco/Components/AccelerationMotion.h>
#include <OpenSim/Moco/Components/DiscreteController.h>
#include <OpenSim/Moco/Components/DiscreteForces.h>
#include <OpenSim/Moco/MocoBounds.h>
#include <OpenSim/Moco/MocoTropterSolver.h>
#include <OpenSim/Moco/MocoUtilities.h>
#include <OpenSim/Simulation/InverseDynamicsSolver.h>
#include <OpenSim/Simulation/SimulationUtilities.h>

#include <tropter/tropter.h>

namespace OpenSim {

inline tropter::Bounds convertBounds(const MocoBounds& mb) {
    return {mb.getLower(), mb.getUpper()};
}
inline tropter::InitialBounds convertBounds(const MocoInitialBounds& mb) {
    return {mb.getLower(), mb.getUpper()};
}
inline tropter::FinalBounds convertBounds(const MocoFinalBounds& mb) {
    return {mb.getLower(), mb.getUpper()};
}

template <typename T>
class MocoTropterSolver::TropterProblemBase : public tropter::Problem<T> {
protected:
    TropterProblemBase(const MocoTropterSolver& solver, bool implicit = false)
            : tropter::Problem<T>(solver.getProblemRep().getName()),
              m_mocoTropterSolver(solver),
              m_mocoProbRep(solver.getProblemRep()),
              m_modelBase(m_mocoProbRep.getModelBase()),
              m_stateBase(m_mocoProbRep.updStateBase()),
              m_modelDisabledConstraints(
                      m_mocoProbRep.getModelDisabledConstraints()),
              m_stateDisabledConstraints(
                      m_mocoProbRep.updStateDisabledConstraints()),
              m_implicit(implicit) {

        // It is sufficient to perform this check only on the original model.
        OPENSIM_THROW_IF(!m_modelBase.getMatterSubsystem().getUseEulerAngles(
                                 m_stateBase),
                Exception, "Quaternions are not supported.");

        // Ensure the model does not have user-provided controllers.
        int numControllers = 0;
        for (const auto& controller :
                m_modelBase.template getComponentList<Controller>()) {
            // Avoid unused variable warning.
            (void)&controller;
            ++numControllers;
        }
        // The model has a DiscreteController added by MocoProblemRep; any other
        // controllers were added by the user.
        OPENSIM_THROW_IF(numControllers > 1, Exception,
                "MocoCasADiSolver does not support models with Controllers.");

        // It is sufficient to create these containers from the original model
        // since the discrete variables added to the model with disabled
        // constraints wouldn't show up anyway.
        m_svNamesInSysOrder =
                m_mocoProbRep.createStateVariableNamesInSystemOrder(
                        m_yIndexMap);

        addStateVariables();
        addControlVariables();
        addParameters();
        addCosts();
        addKinematicConstraints();
        addGenericPathConstraints();

        std::string formattedTimeString(getFormattedDateTime(true));
        m_fileDeletionThrower = OpenSim::make_unique<FileDeletionThrower>(
                fmt::format("delete_this_to_stop_optimization_{}_{}.txt",
                        m_mocoProbRep.getName(), formattedTimeString));
    }

    void addStateVariables() {
        this->set_time(convertBounds(m_mocoProbRep.getTimeInitialBounds()),
                convertBounds(m_mocoProbRep.getTimeFinalBounds()));
        for (const auto& svName : m_svNamesInSysOrder) {
            const auto& info = m_mocoProbRep.getStateInfo(svName);
            this->add_state(svName, convertBounds(info.getBounds()),
                    convertBounds(info.getInitialBounds()),
                    convertBounds(info.getFinalBounds()));
        }
    }

    void addControlVariables() {
        auto controlNames =
                createControlNamesFromModel(m_modelBase, m_modelControlIndices);
        for (const auto& controlName : controlNames) {
            const auto& info = m_mocoProbRep.getControlInfo(controlName);
            this->add_control(controlName, convertBounds(info.getBounds()),
                    convertBounds(info.getInitialBounds()),
                    convertBounds(info.getFinalBounds()));
        }
    }

    void addCosts() {
        const auto costNames = m_mocoProbRep.createCostNames();
        for (const auto& name : costNames) {
            const auto& cost = m_mocoProbRep.getCost(name);
            this->add_cost(name, cost.getNumIntegrals());
        }
        OPENSIM_THROW_IF(m_mocoProbRep.getNumEndpointConstraints(), Exception,
                "MocoTropterSolver does not support endpoint constraints.");
        if (m_mocoTropterSolver.get_minimize_lagrange_multipliers()) {
            m_multiplierCostIndex = this->add_cost("multipliers", 1);
        }
    }

    void addKinematicConstraints() {
        // Add any scalar constraints associated with kinematic constraints in
        // the model as path constraints in the problem.
        // Whether or not enabled kinematic constraints exist in the model,
        // check that optional solver properties related to constraints are
        // set properly.
        const std::vector<std::string> kcNames =
                m_mocoProbRep.createKinematicConstraintNames();
        if (kcNames.empty()) {
            OPENSIM_THROW_IF(
                    m_mocoTropterSolver.get_minimize_lagrange_multipliers(),
                    Exception,
                    "Solver property 'minimize_lagrange_multipliers' "
                    "was enabled but no enabled kinematic constraints exist in "
                    "the "
                    "model.");
            // Do not add kinematic constraints, so we can return.
            return;
        }

        int cid, mp, mv, ma;
        int numEquationsThisConstraint;
        int multIndexThisConstraint;
        std::vector<MocoBounds> bounds;
        std::vector<std::string> labels;
        std::vector<KinematicLevel> kinLevels;
        const bool enforceConstraintDerivs =
                m_mocoTropterSolver.get_enforce_constraint_derivatives();
        for (const auto& kcName : kcNames) {
            const auto& kc = m_mocoProbRep.getKinematicConstraint(kcName);
            const auto& multInfos = m_mocoProbRep.getMultiplierInfos(kcName);
            cid = kc.getSimbodyConstraintIndex();
            mp = kc.getNumPositionEquations();
            mv = kc.getNumVelocityEquations();
            ma = kc.getNumAccelerationEquations();
            bounds = kc.getConstraintInfo().getBounds();
            labels = kc.getConstraintInfo().getConstraintLabels();
            kinLevels = kc.getKinematicLevels();

            // TODO only add velocity correction variables for holonomic
            // constraint derivatives? For now, disallow enforcing derivatives
            // if non-holonomic or acceleration constraints present.
            OPENSIM_THROW_IF(enforceConstraintDerivs && mv != 0, Exception,
                    "Enforcing constraint derivatives is supported "
                    "only for holonomic (position-level) constraints. There "
                    "are {} velocity-level scalar constraints associated with "
                    "the model Constraint at ConstraintIndex {}.",
                    mv, cid);
            OPENSIM_THROW_IF(enforceConstraintDerivs && ma != 0, Exception,
                    "Enforcing constraint derivatives is supported only for "
                    "holonomic (position-level) constraints. There are {} "
                    "acceleration-level scalar constraints associated with the "
                    "model Constraint at ConstraintIndex {}.",
                    ma, cid);

            m_total_mp += mp;
            m_total_mv += mv;
            m_total_ma += ma;

            // Loop through all scalar constraints associated with the model
            // constraint and corresponding path constraints to the optimal
            // control problem.
            //
            // We need a different index for the Lagrange multipliers since
            // they are only added if the current constraint equation is not a
            // derivative of a position- or velocity-level equation.
            multIndexThisConstraint = 0;
            numEquationsThisConstraint = 0;
            for (int i = 0; i < kc.getConstraintInfo().getNumEquations(); ++i) {

                // If the index for this path constraint represents an
                // a non-derivative scalar constraint equation, also add a
                // Lagrange multiplier to the problem.
                if (kinLevels[i] == KinematicLevel::Position ||
                        kinLevels[i] == KinematicLevel::Velocity ||
                        kinLevels[i] == KinematicLevel::Acceleration) {

                    // TODO name constraints based on model constraint names
                    // or coordinate names if a locked or prescribed coordinate
                    this->add_path_constraint(
                            labels[i], convertBounds(bounds[i]));

                    const auto& multInfo = multInfos[multIndexThisConstraint];
                    this->add_adjunct(multInfo.getName(),
                            convertBounds(multInfo.getBounds()),
                            convertBounds(multInfo.getInitialBounds()),
                            convertBounds(multInfo.getFinalBounds()));
                    // Add velocity correction variables if enforcing
                    // constraint equation derivatives.
                    if (enforceConstraintDerivs) {
                        // TODO this naming convention assumes that the
                        // associated Lagrange multiplier name begins with
                        // "lambda", which may change in the future.
                        OPENSIM_THROW_IF(
                                multInfo.getName().substr(0, 6) != "lambda",
                                Exception,
                                "Expected the multiplier name for "
                                "this constraint to begin with "
                                "'lambda' but it begins with '{}'.",
                                multInfo.getName().substr(0, 6));
                        this->add_diffuse(std::string(multInfo.getName())
                                                  .replace(0, 6, "gamma"),
                                convertBounds(
                                        m_mocoTropterSolver
                                                .get_velocity_correction_bounds()));
                    }
                    ++multIndexThisConstraint;
                    ++numEquationsThisConstraint;

                    // If enforcing constraint derivatives, also add path
                    // constraints for kinematic constraint equation
                    // derivatives.
                } else if (enforceConstraintDerivs) {
                    this->add_path_constraint(
                            labels[i], convertBounds(bounds[i]));
                    ++numEquationsThisConstraint;
                }
            }

            m_numKinematicConstraintEquations += numEquationsThisConstraint;
        }
        m_numMultipliers = m_total_mp + m_total_mv + m_total_ma;
    }

    /// Add any generic path constraints included in the problem.
    void addGenericPathConstraints() {
        for (std::string pcName : m_mocoProbRep.createPathConstraintNames()) {
            const MocoPathConstraint& constraint =
                    m_mocoProbRep.getPathConstraint(pcName);
            auto pcInfo = constraint.getConstraintInfo();
            auto labels = pcInfo.getConstraintLabels();
            auto bounds = pcInfo.getBounds();
            for (int i = 0; i < pcInfo.getNumEquations(); ++i) {
                this->add_path_constraint(labels[i], convertBounds(bounds[i]));
            }
        }
        m_numPathConstraintEquations =
                m_mocoProbRep.getNumPathConstraintEquations();
    }

    void addParameters() {
        for (std::string name : m_mocoProbRep.createParameterNames()) {
            const MocoParameter& parameter = m_mocoProbRep.getParameter(name);
            this->add_parameter(name, convertBounds(parameter.getBounds()));
        }
    }

    void initialize_on_iterate(
            const Eigen::VectorXd& parameters) const override final {
        m_fileDeletionThrower->throwIfDeleted();
        // If they exist, apply parameter values to the model.
        this->applyParametersToModelProperties(parameters);
    }

    void setSimTKTimeAndStates(const T& time,
            Eigen::Ref<const tropter::VectorX<T>> states,
            SimTK::State& simTKState) const {
        simTKState.setTime(time);
        // We must skip over unused slots in the SimTK::State that are reserved
        // for quaternions.
        for (int isv = 0; isv < states.size(); ++isv) {
            simTKState.updY()[m_yIndexMap.at(isv)] = states[isv];
        }
    }

    void setSimTKState(const tropter::Input<T>& in) const {
        setSimTKState(in.time, in.states, in.controls, in.adjuncts, 0);
    }
    void setSimTKStateForCostInitial(
            const tropter::CostInput<T>& in) const {
        setSimTKState(in.initial_time, in.initial_states, in.initial_controls,
                in.initial_adjuncts, 0);
    }
    void setSimTKStateForCostFinal(
            const tropter::CostInput<T>& in) const {
        setSimTKState(in.final_time, in.final_states, in.final_controls,
                in.final_adjuncts, 1);
    }
    /// Use `stateDisConIndex` to specify which of the two
    /// stateDisabledConstraints from MocoProblemRep to update.
    void setSimTKState(const T& time,
            const Eigen::Ref<const tropter::VectorX<T>>& states,
            const Eigen::Ref<const tropter::VectorX<T>>& controls,
            const Eigen::Ref<const tropter::VectorX<T>>& adjuncts,
            int stateDisConIndex = 0) const {

        auto& simTKStateBase = this->m_stateBase;
        auto& simTKStateDisabledConstraints =
                m_mocoProbRep.updStateDisabledConstraints(stateDisConIndex);
        auto& modelDisabledConstraints = this->m_modelDisabledConstraints;

        if (m_implicit && !m_mocoProbRep.isPrescribedKinematics()) {
            const auto& accel = this->m_mocoProbRep.getAccelerationMotion();
            const int NU = simTKStateDisabledConstraints.getNU();
            const auto& w = adjuncts.segment(
                    this->m_numKinematicConstraintEquations, NU);
            SimTK::Vector udot((int)w.size(), w.data(), true);
            accel.setUDot(simTKStateDisabledConstraints, udot);
        }

        this->setSimTKTimeAndStates(
                time, states, simTKStateDisabledConstraints);

        if (modelDisabledConstraints.getNumControls()) {
            // Set the controls for actuators in the OpenSim model with disabled
            // constraints. The base model never gets realized past
            // Stage::Velocity, so we don't ever need to set its controls.
            auto& osimControls =
                    m_mocoProbRep.getDiscreteControllerDisabledConstraints()
                            .updDiscreteControls(simTKStateDisabledConstraints);
            for (int ic = 0; ic < controls.size(); ++ic) {
                osimControls[m_modelControlIndices[ic]] = controls[ic];
            }
        }

        // If enabled constraints exist in the model, compute constraint forces
        // based on Lagrange multipliers, update the associated
        // discrete variables in the state.
        if (this->m_numKinematicConstraintEquations) {
            this->setSimTKTimeAndStates(time, states, simTKStateBase);
            this->calcAndApplyKinematicConstraintForces(
                    adjuncts, simTKStateBase, simTKStateDisabledConstraints);
        }
    }

    void calc_cost_integrand(int cost_index, const tropter::Input<T>& in,
            T& integrand) const override {
        if (cost_index == m_multiplierCostIndex) {
            // Unpack variables.
            const auto& adjuncts = in.adjuncts;
            // If specified, add squared multiplers cost to the integrand.
            const auto& multiplierWeight =
                    m_mocoTropterSolver.get_lagrange_multiplier_weight();
            for (int i = 0; i < m_numMultipliers; ++i) {
                integrand += multiplierWeight * adjuncts[i] * adjuncts[i];
            }
            return;
        }

        // Update the state.
        // TODO would it make sense to a vector of States, one for each mesh
        // point, so that each can preserve their cache?
        this->setSimTKState(in);

        const auto& discreteController =
                m_mocoProbRep.getDiscreteControllerDisabledConstraints();
        const auto& rawControls = discreteController.getDiscreteControls(
                this->m_stateDisabledConstraints);

        // Compute the integrand for this cost term.
        const auto& cost = m_mocoProbRep.getCostByIndex(cost_index);
        integrand = cost.calcIntegrand(
                {in.time, this->m_stateDisabledConstraints, rawControls});
    }

    void calc_cost(int cost_index, const tropter::CostInput<T>& in,
            T& cost_value) const override {
        if (cost_index == m_multiplierCostIndex) {
            cost_value = in.integral;
            return;
        }

        // Update the state.
        this->setSimTKStateForCostInitial(in);
        this->setSimTKStateForCostFinal(in);

        const auto& initialState = m_mocoProbRep.updStateDisabledConstraints(0);
        const auto& finalState = m_mocoProbRep.updStateDisabledConstraints(1);

        const auto& discreteController =
                m_mocoProbRep.getDiscreteControllerDisabledConstraints();
        const auto& initialRawControls = discreteController.getDiscreteControls(
                initialState);
        const auto& finalRawControls = discreteController.getDiscreteControls(
                finalState);

        // Compute the cost for this cost term.
        const auto& cost = m_mocoProbRep.getCostByIndex(cost_index);
        SimTK::Vector costVector(cost.getNumOutputs());
        cost.calcGoal({in.initial_time, initialState, initialRawControls,
                              in.final_time, finalState, finalRawControls,
                              in.integral},
                costVector);
        cost_value = costVector.sum();
    }

    const MocoTropterSolver& m_mocoTropterSolver;
    const MocoProblemRep& m_mocoProbRep;
    const Model& m_modelBase;
    SimTK::State& m_stateBase;
    const Model& m_modelDisabledConstraints;
    SimTK::State& m_stateDisabledConstraints;
    const bool m_implicit;
    int m_multiplierCostIndex = -1;

    std::unique_ptr<FileDeletionThrower> m_fileDeletionThrower;

    std::vector<std::string> m_svNamesInSysOrder;
    std::unordered_map<int, int> m_yIndexMap;
    std::vector<int> m_modelControlIndices;
    mutable SimTK::Vector_<SimTK::SpatialVec> m_constraintBodyForces;
    mutable SimTK::Vector m_constraintMobilityForces;
    mutable SimTK::Vector qdot;
    mutable SimTK::Vector qdotCorr;
    // The total number of scalar holonomic, non-holonomic, and acceleration
    // constraint equations enabled in the model. This does not count equations
    // for derivatives of holonomic and non-holonomic constraints.
    mutable int m_total_mp = 0;
    mutable int m_total_mv = 0;
    mutable int m_total_ma = 0;
    // This is the sum of m_total_m(p|v|a).
    mutable int m_numMultipliers = 0;
    // This is the output argument of
    // SimbodyMatterSubsystem::calcConstraintAccelerationErrors(), and includes
    // the acceleration-level holonomic, non-holonomic constraint errors and the
    // acceleration-only constraint errors.
    mutable SimTK::Vector m_pvaerr;
    // The total number of scalar constraint equations associated with model
    // kinematic constraints that the solver is responsible for enforcing. This
    // number does include equations for constraint derivatives.
    mutable int m_numKinematicConstraintEquations = 0;
    // The total number of scalar constraint equations associated with
    // MocoPathConstraints added to the MocoProblem.
    mutable int m_numPathConstraintEquations = 0;

    /// Apply parameters to properties in `m_modelBase` and
    /// `m_modelDisabledConstraints`.
    void applyParametersToModelProperties(
            const tropter::VectorX<T>& parameters) const {
        if (parameters.size()) {
            // Warning: memory borrowed, not copied (when third argument to
            // SimTK::Vector constructor is true)
            SimTK::Vector mocoParams(
                    (int)parameters.size(), parameters.data(), true);

            m_mocoProbRep.applyParametersToModelProperties(mocoParams, true);
        }
    }

    void calcAndApplyKinematicConstraintForces(
            const tropter::VectorX<T>& adjuncts, const SimTK::State& stateBase,
            SimTK::State& stateDisabledConstraints) const {
        // Calculate the constraint forces using the original model and the
        // solver-provided Lagrange multipliers.
        m_modelBase.realizeVelocity(stateBase);
        const auto& matter = m_modelBase.getMatterSubsystem();
        // Multipliers are negated so constraint forces can be used like
        // applied forces.
        SimTK::Vector multipliers(m_numMultipliers, adjuncts.data(), true);
        matter.calcConstraintForcesFromMultipliers(stateBase, -multipliers,
                m_constraintBodyForces, m_constraintMobilityForces);
        // Apply the constraint forces on the model with disabled constraints.
        const auto& constraintForces = m_mocoProbRep.getConstraintForces();
        constraintForces.setAllForces(stateDisabledConstraints,
                m_constraintMobilityForces, m_constraintBodyForces);
    }

    void calcKinematicConstraintErrors(
            const SimTK::Vector& udot, tropter::Output<T>& out) const {
        // Only compute constraint errors if we're at a time point where path
        // constraints in the optimal control problem are enforced.
        if (out.path.size() != 0 && this->m_numKinematicConstraintEquations) {
            auto& stateBase = this->m_stateBase;

            // Position-level errors.
            std::copy_n(stateBase.getQErr().getContiguousScalarData(),
                    m_total_mp, out.path.data());

            const auto& enforceConstraintDerivatives =
                    m_mocoTropterSolver.get_enforce_constraint_derivatives();
            if (enforceConstraintDerivatives || m_total_ma) {
                // Calculate udoterr. We cannot use State::getUDotErr()
                // because that uses Simbody's multipliers and UDot,
                // whereas we have our own multipliers and UDot. Here, we use
                // the udot computed from the model with disabled constraints
                // since we cannot use (nor do we have available) udot computed
                // from the original model.
                const auto& matterBase = m_modelBase.getMatterSubsystem();
                matterBase.calcConstraintAccelerationErrors(
                        stateBase, udot, m_pvaerr);
            } else {
                m_pvaerr = SimTK::NaN;
            }

            if (enforceConstraintDerivatives) {
                // Velocity-level errors.
                std::copy_n(stateBase.getUErr().getContiguousScalarData(),
                        m_total_mp + m_total_mv, out.path.data() + m_total_mp);
                // Acceleration-level errors.
                std::copy_n(m_pvaerr.getContiguousScalarData(),
                        m_total_mp + m_total_mv + m_total_ma,
                        out.path.data() + 2 * m_total_mp + m_total_mv);
            } else {
                // Velocity-level errors. Skip derivatives of position-level
                // constraint equations.
                std::copy_n(stateBase.getUErr().getContiguousScalarData() +
                                    m_total_mp,
                        m_total_mv, out.path.data() + m_total_mp);
                // Acceleration-level errors. Skip derivatives of velocity-
                // and position-level constraint equations.
                std::copy_n(m_pvaerr.getContiguousScalarData() + m_total_mp +
                                    m_total_mv,
                        m_total_ma, out.path.data() + m_total_mp + m_total_mv);
            }
        }
    }

    void calcPathConstraintErrors(
            const SimTK::State& state, tropter::Output<T>& out) const {
        if (out.path.size() != 0) {
            // Copy errors from generic path constraints into output struct.
            SimTK::Vector pathConstraintErrors(
                    this->m_numPathConstraintEquations,
                    out.path.data() + m_numKinematicConstraintEquations, true);
            m_mocoProbRep.calcPathConstraintErrors(state, pathConstraintErrors);
        }
    }

public:
    template <typename MocoTrajectoryType, typename tropIterateType>
    MocoTrajectoryType convertIterateTropterToMoco(
            const tropIterateType& tropSol) const;

    MocoTrajectory convertToMocoTrajectory(
            const tropter::Iterate& tropSol) const;

    MocoSolution convertToMocoSolution(const tropter::Solution& tropSol) const;

    tropter::Iterate convertToTropterIterate(
            const MocoTrajectory& mocoIter) const;
};

template <typename T>
class MocoTropterSolver::ExplicitTropterProblem
        : public MocoTropterSolver::TropterProblemBase<T> {
public:
    ExplicitTropterProblem(const MocoTropterSolver& solver)
            : MocoTropterSolver::TropterProblemBase<T>(solver) {}
    void initialize_on_mesh(const Eigen::VectorXd&) const override {}
    void calc_differential_algebraic_equations(const tropter::Input<T>& in,
            tropter::Output<T> out) const override {
        // Unpack variables.
        const auto& diffuses = in.diffuses;

        // Original model and its associated state. These are used to calculate
        // kinematic constraint forces and errors.
        const auto& modelBase = this->m_modelBase;
        auto& simTKStateBase = this->m_stateBase;

        // Model with disabled constraints and its associated state. These are
        // used to compute the accelerations.
        const auto& modelDisabledConstraints = this->m_modelDisabledConstraints;
        auto& simTKStateDisabledConstraints = this->m_stateDisabledConstraints;

        // Update the state.
        this->setSimTKState(in);

        // Compute the accelerations.
        // TODO Antoine and Gil said realizing Dynamics is a lot costlier
        // than realizing to Velocity and computing forces manually.
        modelDisabledConstraints.realizeAcceleration(
                simTKStateDisabledConstraints);

        // Compute kinematic constraint errors if they exist.
        this->calcKinematicConstraintErrors(
                simTKStateDisabledConstraints.getUDot(), out);

        // Apply velocity correction to qdot if at a mesh interval midpoint.
        // This correction modifies the dynamics to enable a projection of
        // the model coordinates back onto the constraint manifold whenever
        // they deviate.
        // Posa, Kuindersma, Tedrake, 2016. "Optimization and stabilization
        // of trajectories for constrained dynamical systems"
        // Note: Only supported for the Hermite-Simpson transcription scheme.
        if (diffuses.size() != 0) {
            SimTK::Vector gamma((int)diffuses.size(), diffuses.data());
            const auto& matter = modelBase.getMatterSubsystem();
            matter.multiplyByGTranspose(simTKStateBase, gamma, this->qdotCorr);
            // It doesn't matter what state we use for U since it's U is the
            // same in both states.
            this->qdot = simTKStateDisabledConstraints.getU() + this->qdotCorr;
        } else {
            this->qdot = simTKStateDisabledConstraints.getU();
        }

        // Copy state derivative values to output struct. We cannot simply
        // use getYDot() because we may have applied a velocity correction to
        // qdot.
        const int nq = this->qdot.size();
        const auto& udot = simTKStateDisabledConstraints.getUDot();
        const auto& zdot = simTKStateDisabledConstraints.getZDot();
        const int nu = udot.size();
        const int nz = zdot.size();
        std::copy_n(
                this->qdot.getContiguousScalarData(), nq, out.dynamics.data());
        std::copy_n(
                udot.getContiguousScalarData(), nu, out.dynamics.data() + nq);
        std::copy_n(zdot.getContiguousScalarData(), nz,
                out.dynamics.data() + nq + nu);

        // Path constraint errors.
        this->calcPathConstraintErrors(simTKStateDisabledConstraints, out);
    }
};

template <typename T>
class MocoTropterSolver::ImplicitTropterProblem
        : public MocoTropterSolver::TropterProblemBase<T> {
public:
    ImplicitTropterProblem(const MocoTropterSolver& solver)
            : TropterProblemBase<T>(solver, true) {
        OPENSIM_THROW_IF(this->m_numKinematicConstraintEquations, Exception,
                "Cannot use implicit dynamics mode with kinematic "
                "constraints.");

        auto& simTKStateDisabledConstraints = this->m_stateDisabledConstraints;
        if (!this->m_mocoProbRep.isPrescribedKinematics()) {
            const auto& accel = this->m_mocoProbRep.getAccelerationMotion();
            accel.setEnabled(simTKStateDisabledConstraints, true);
        }

        // Add adjuncts for udot, which we call "w".
        int NU = simTKStateDisabledConstraints.getNU();
        for (int iudot = 0; iudot < NU; ++iudot) {
            auto name = this->m_svNamesInSysOrder[iudot];
            auto leafpos = name.find("value");
            OPENSIM_THROW_IF(
                    leafpos == std::string::npos, Exception, "Internal error.");
            name.replace(leafpos, name.size(), "accel");
            this->add_adjunct(name,
                convertBounds(
                    solver.get_implicit_multibody_acceleration_bounds()));
            this->add_path_constraint(name.substr(0, leafpos) + "residual", 0);
        }
    }
    void calc_differential_algebraic_equations(const tropter::Input<T>& in,
            tropter::Output<T> out) const override {

        const auto& states = in.states;
        const auto& adjuncts = in.adjuncts;

        const auto& modelDisabledConstraints = this->m_modelDisabledConstraints;
        auto& simTKStateDisabledConstraints = this->m_stateDisabledConstraints;

        const int numEmptySlots =
                simTKStateDisabledConstraints.getNY() - (int)states.size();
        const auto NQ = simTKStateDisabledConstraints.getNQ() - numEmptySlots;
        const auto NU = simTKStateDisabledConstraints.getNU();
        const auto NZ = simTKStateDisabledConstraints.getNZ();

        const auto& u = states.segment(NQ, NU);
        const auto& w =
                adjuncts.segment(this->m_numKinematicConstraintEquations, NU);

        // Kinematic differential equations
        // --------------------------------
        // qdot = u
        // TODO does not work for quaternions!
        out.dynamics.head(NQ) = u;

        // Multibody dynamics: differential equations
        // ------------------------------------------
        // udot = w
        out.dynamics.segment(NQ, NU) = w;

        // Multibody dynamics: "F - ma = 0"
        // --------------------------------
        this->setSimTKState(in);

        // TODO: Update to support kinematic constraints, using
        // this->calcKinematicConstraintForces()
        this->calcPathConstraintErrors(simTKStateDisabledConstraints, out);

        if (NZ || out.path.size()) {
            modelDisabledConstraints.realizeAcceleration(
                    simTKStateDisabledConstraints);
        }

        if (NZ) {
            const auto& zdot = simTKStateDisabledConstraints.getZDot();
            std::copy_n(zdot.getContiguousScalarData(), zdot.size(),
                    out.dynamics.data() + NQ + NU);
        }

        if (out.path.size() != 0) {
            const auto& matter =
                    this->m_modelDisabledConstraints.getMatterSubsystem();
            matter.findMotionForces(simTKStateDisabledConstraints, m_residual);

            double* residualBegin = out.path.data() +
                                    this->m_numKinematicConstraintEquations +
                                    this->m_numPathConstraintEquations;
            std::copy_n(m_residual.getContiguousScalarData(), m_residual.size(),
                    residualBegin);
        }
    }

private:
    mutable SimTK::Vector m_residual;
};

} // namespace OpenSim

#endif // OPENSIM_TROPTERPROBLEM_H
