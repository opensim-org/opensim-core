#ifndef MUSCOLLO_TROPTERPROBLEM_H
#define MUSCOLLO_TROPTERPROBLEM_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: TropterProblem.h                                         *
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

#include "../MucoBounds.h"
#include "../MucoTropterSolver.h"
#include "../MuscolloUtilities.h"

#include <simbody/internal/Constraint.h>
#include <OpenSim/Simulation/InverseDynamicsSolver.h>

#include <tropter/tropter.h>

namespace OpenSim {

inline tropter::Bounds convertBounds(const MucoBounds& mb) {
    return {mb.getLower(), mb.getUpper()};
}
inline tropter::InitialBounds convertBounds(const MucoInitialBounds& mb) {
    return {mb.getLower(), mb.getUpper()};
}
inline tropter::FinalBounds convertBounds(const MucoFinalBounds& mb) {
    return {mb.getLower(), mb.getUpper()};
}

template <typename T>
class MucoTropterSolver::TropterProblemBase
        : public tropter::Problem<T> {
protected:
    TropterProblemBase(const MucoTropterSolver& solver)
            : tropter::Problem<T>(solver.getProblemRep().getName()),
            m_mucoTropterSolver(solver),
            m_mucoProbRep(solver.getProblemRep()),
            m_model(m_mucoProbRep.getModel()) {
        // TODO set name properly.
        // Disable all controllers.
        // TODO temporary; don't want to actually do this.
        auto controllers = m_model.getComponentList<Controller>();
        for (auto& controller : controllers) {
            OPENSIM_THROW_IF(controller.get_enabled(), Exception,
                    "MucoTropterSolver does not support OpenSim Controllers. "
                    "Disable all controllers in the model.");
        }
        m_state = m_model.getWorkingState();
        m_svNamesInSysOrder = createStateVariableNamesInSystemOrder(m_model);

        addStateVariables();
        addControlVariables();
        addMultibodyConstraints();
        addGenericPathConstraints();
        addParameters();
    }

    void addStateVariables() {
        this->set_time(convertBounds(m_mucoProbRep.getTimeInitialBounds()),
                convertBounds(m_mucoProbRep.getTimeFinalBounds()));
        for (const auto& svName : m_svNamesInSysOrder) {
            const auto& info = m_mucoProbRep.getStateInfo(svName);
            this->add_state(svName, convertBounds(info.getBounds()),
                    convertBounds(info.getInitialBounds()),
                    convertBounds(info.getFinalBounds()));
        }
    }

    void addControlVariables() {
        for (const auto& actu : m_model.getComponentList<Actuator>()) {
            // TODO handle a variable number of control signals.
            const auto& actuName = actu.getAbsolutePathString();
            const auto& info = m_mucoProbRep.getControlInfo(actuName);
            this->add_control(actuName, convertBounds(info.getBounds()),
                    convertBounds(info.getInitialBounds()),
                    convertBounds(info.getFinalBounds()));
        }
    }

    void addMultibodyConstraints() {
        // Add any scalar constraints associated with multibody constraints in
        // the model as path constraints in the problem.
        int cid, mp, mv, ma, numEquationsEnforced, multIndexThisConstraint;
        std::vector<MucoBounds> bounds;
        std::vector<std::string> labels;
        std::vector<KinematicLevel> kinLevels;
        std::vector<std::string> mcNames =
                m_mucoProbRep.createMultibodyConstraintNames();
        for (const auto& mcName : mcNames) {
            const auto& mc = m_mucoProbRep.getMultibodyConstraint(mcName);
            const auto& multInfos = m_mucoProbRep.getMultiplierInfos(mcName);
            cid = mc.getSimbodyConstraintIndex();
            mp = mc.getNumPositionEquations();
            mv = mc.getNumVelocityEquations();
            ma = mc.getNumAccelerationEquations();
            bounds = mc.getConstraintInfo().getBounds();
            labels = mc.getConstraintInfo().getConstraintLabels();
            kinLevels = mc.getKinematicLevels();

            m_mpSum += mp;
            // Only considering holonomic constraints for now.
            // TODO if (get_enforce_holonomic_constraints_only()) {
            OPENSIM_THROW_IF(mv != 0, Exception,
                    "Only holonomic "
                    "(position-level) constraints are currently supported. "
                    "There are " + std::to_string(mv) + " velocity-level "
                    "scalar constraints associated with the model Constraint "
                    "at ConstraintIndex " + std::to_string(cid) + ".");
            OPENSIM_THROW_IF(ma != 0, Exception,
                    "Only holonomic "
                    "(position-level) constraints are currently supported. "
                    "There are " + std::to_string(ma) + " acceleration-level "
                    "scalar constraints associated with the model Constraint "
                    "at ConstraintIndex " + std::to_string(cid) + ".");
            // } else {
            //   m_mvSum += mv;
            //   m_maSum += ma;
            // }

            numEquationsEnforced = mp;
            //TODO numEquationsEnforced =
            //    get_enforce_holonomic_constraints_only()
            //        ? mp : mcInfo.getNumEquations();

            // Loop through all scalar constraints associated with the model
            // constraint and corresponding path constraints to the optimal
            // control problem.
            //
            // We need a different index for the Lagrange multipliers since
            // they are only added if the current constraint equation is not a
            // derivative of a position- or velocity-level equation.
            multIndexThisConstraint = 0;
            for (int i = 0; i < numEquationsEnforced; ++i) {

                // TODO name constraints based on model constraint names
                // or coordinate names if a locked or prescribed coordinate
                this->add_path_constraint(labels[i], convertBounds(bounds[i]));

                // If the index for this path constraint represents an
                // a non-derivative scalar constraint equation, also add a
                // Lagrange multplier to the problem.
                if (kinLevels[i] == KinematicLevel::Position ||
                        kinLevels[i] == KinematicLevel::Velocity ||
                        kinLevels[i] == KinematicLevel::Acceleration) {

                    const auto& multInfo = multInfos[multIndexThisConstraint];
                    this->add_adjunct(multInfo.getName(),
                            convertBounds(multInfo.getBounds()),
                            convertBounds(multInfo.getInitialBounds()),
                            convertBounds(multInfo.getFinalBounds()));
                    ++multIndexThisConstraint;
                }
            }

            m_numMultibodyConstraintEqs += numEquationsEnforced;
        }

    }

    /// Add any generic path constraints included in the problem.
    void addGenericPathConstraints() {
        for (std::string pcName : m_mucoProbRep.createPathConstraintNames()) {
            const MucoPathConstraint& constraint =
                    m_mucoProbRep.getPathConstraint(pcName);
            auto pcInfo = constraint.getConstraintInfo();
            auto labels = pcInfo.getConstraintLabels();
            auto bounds = pcInfo.getBounds();
            for (int i = 0; i < pcInfo.getNumEquations(); ++i) {
                this->add_path_constraint(labels[i], convertBounds(bounds[i]));
            }
        }
        m_numPathConstraintEqs = m_mucoProbRep.getNumPathConstraintEquations();
    }

    void addParameters() {
        for (std::string name : m_mucoProbRep.createParameterNames()) {
            const MucoParameter& parameter = m_mucoProbRep.getParameter(name);
            this->add_parameter(name, convertBounds(parameter.getBounds()));
        }
    }

    void initialize_on_iterate(const Eigen::VectorXd& parameters)
            const override final {
        // If they exist, apply parameter values to the model.
        this->applyParametersToModel(parameters);
    }

    void calc_integral_cost(const tropter::Input<T>& in,
            T& integrand) const override {
        // Unpack variables.
        const auto& time = in.time;
        const auto& states = in.states;
        const auto& controls = in.controls;
        const auto& adjuncts = in.adjuncts;

        // TODO would it make sense to a vector of States, one for each mesh
        // point, so that each can preserve their cache?
        m_state.setTime(time);
        std::copy_n(states.data(), states.size(),
                m_state.updY().updContiguousScalarData());

        // Set the controls for actuators in the OpenSim model.
        if (m_model.getNumControls()) {
            auto& osimControls = m_model.updControls(m_state);
            std::copy_n(controls.data(), controls.size(),
                    osimControls.updContiguousScalarData());
            m_model.realizePosition(m_state);
            m_model.setControls(m_state, osimControls);
        } else {
            m_model.realizePosition(m_state);
        }

        integrand = m_mucoProbRep.calcIntegralCost(m_state);

        // TODO if (get_enforce_holonomic_constraints_only()) {
        // Add squared multiplers cost to integrand. Since we currently don't
        // include the derivatives of the holonomic contraint equations as path
        // constraints in the OCP, this term exists in order to impose
        // uniqueness in the Lagrange multipliers.
        for (int i = 0; i < m_numMultibodyConstraintEqs; ++i) {
            // The first m_numMultibodyConstraintEqs adjuncts are the
            // multibody constraint multipliers.
            integrand += m_mucoTropterSolver.get_multiplier_weight()
                    * adjuncts[i] * adjuncts[i];
        }

    }

    void calc_endpoint_cost(const T& final_time,
            const tropter::VectorX<T>& states,
            const tropter::VectorX<T>& /*parameters*/,
            T& cost) const override {
        // TODO avoid all of this if there are no endpoint costs.
        m_state.setTime(final_time);
        std::copy(states.data(), states.data() + states.size(),
                m_state.updY().updContiguousScalarData());
        // TODO cannot use control signals...
        m_model.updControls(m_state).setToNaN();
        cost = m_mucoProbRep.calcEndpointCost(m_state);
    }

    const MucoTropterSolver& m_mucoTropterSolver;
    const MucoProblemRep& m_mucoProbRep;
    const Model& m_model;
    mutable SimTK::State m_state;

    std::vector<std::string> m_svNamesInSysOrder;

    // The number of scalar holonomic constraint equations enabled in the model.
    // This does not count equations for derivatives of scalar holonomic
    // constraints.
    mutable int m_mpSum = 0;
    // TODO mutable int m_mvSum = 0; (nonholonomic constraints)
    // TODO mutable int m_maSum = 0; (acceleration-only constraints)

    // The total number of scalar constraint equations associated with model
    // multibody constraints that the solver is responsible for enforcing.
    mutable int m_numMultibodyConstraintEqs = 0;
    // The total number of scalar constraint equations associated with
    // MucoPathConstraints added to the MucoProblem.
    mutable int m_numPathConstraintEqs = 0;

    void applyParametersToModel(const tropter::VectorX<T>& parameters) const
    {
        if (parameters.size()) {
            // Warning: memory borrowed, not copied (when third argument to
            // SimTK::Vector constructor is true)
            SimTK::Vector mucoParams(
                    (int)parameters.size(), parameters.data(), true);


            m_mucoProbRep.applyParametersToModel(mucoParams);
            // TODO: Avoid this const_cast.
            const_cast<Model&>(m_model).initSystem();
        }
    }

    void calcMultibodyConstraintForces(const tropter::Input<T>& in,
            const SimTK::State& state,
            SimTK::Vector_<SimTK::SpatialVec>& constraintBodyForces,
            SimTK::Vector& constraintMobilityForces) const {
        // If enabled constraints exist in the model, compute accelerations
        // based on Lagrange multipliers.
        const auto& matter = m_model.getMatterSubsystem();

        // Multipliers are negated so constraint forces can be used like
        // applied forces.
        SimTK::Vector multipliers(m_numMultibodyConstraintEqs,
                in.adjuncts.data(), true);
        matter.calcConstraintForcesFromMultipliers(state, -multipliers,
                constraintBodyForces, constraintMobilityForces);
    }

    void calcPathConstraintErrors(const SimTK::State& state,
            double* errorsBegin) const {
        // Copy errors from generic path constraints into output struct.
        SimTK::Vector pathConstraintErrors(
                this->m_numPathConstraintEqs, errorsBegin, true);
        m_mucoProbRep.calcPathConstraintErrors(state, pathConstraintErrors);
    }

public:
    template <typename MucoIterateType, typename tropIterateType>
    MucoIterateType
    convertIterateTropterToMuco(const tropIterateType& tropSol) const;

    MucoIterate
    convertToMucoIterate(const tropter::Iterate& tropSol) const;

    MucoSolution
    convertToMucoSolution(const tropter::Solution& tropSol) const;

    tropter::Iterate
    convertToTropterIterate(const MucoIterate& mucoIter) const;
};


template <typename T>
class MucoTropterSolver::ExplicitTropterProblem
        : public MucoTropterSolver::TropterProblemBase<T> {
public:
    ExplicitTropterProblem(const MucoTropterSolver& solver)
            : MucoTropterSolver::TropterProblemBase<T>(solver) {
    }
    void initialize_on_mesh(const Eigen::VectorXd&) const override {
    }
    void calc_differential_algebraic_equations(
            const tropter::Input<T>& in,
            tropter::Output<T> out) const override {

        // TODO convert to implicit formulation.

        const auto& states = in.states;
        const auto& controls = in.controls;

        auto& model = this->m_model;
        auto& simTKState = this->m_state;

        simTKState.setTime(in.time);
        std::copy_n(states.data(), states.size(),
                simTKState.updY().updContiguousScalarData());
        //
        // TODO do not copy? I think this will still make a copy:
        // TODO use m_state.updY() = SimTK::Vector(states.size(), states.data(), true);
        //m_state.setY(SimTK::Vector(states.size(), states.data(), true));

        // Set the controls for actuators in the OpenSim model.
        if (model.getNumControls()) {
            auto& osimControls = model.updControls(simTKState);
            std::copy_n(controls.data(), controls.size(),
                    osimControls.updContiguousScalarData());
            model.realizeVelocity(simTKState);
            model.setControls(simTKState, osimControls);
        }

        // If enabled constraints exist in the model, compute accelerations
        // based on Lagrange multipliers.
        if (this->m_numMultibodyConstraintEqs) {
            // TODO Antoine and Gil said realizing Dynamics is a lot costlier than
            // realizing to Velocity and computing forces manually.
            model.realizeDynamics(simTKState);

            const SimTK::MultibodySystem& multibody =
                    model.getMultibodySystem();
            const SimTK::Vector_<SimTK::SpatialVec>& appliedBodyForces =
                    multibody.getRigidBodyForces(simTKState,
                            SimTK::Stage::Dynamics);
            const SimTK::Vector& appliedMobilityForces =
                    multibody.getMobilityForces(simTKState,
                            SimTK::Stage::Dynamics);

            const SimTK::SimbodyMatterSubsystem& matter =
                    model.getMatterSubsystem();

            // TODO: Use working memory.
            SimTK::Vector_<SimTK::SpatialVec> constraintBodyForces;
            SimTK::Vector constraintMobilityForces;
            this->calcMultibodyConstraintForces(in, simTKState,
                    constraintBodyForces, constraintMobilityForces);

            SimTK::Vector udot;
            matter.calcAccelerationIgnoringConstraints(simTKState,
                    appliedMobilityForces + constraintMobilityForces,
                    appliedBodyForces + constraintBodyForces, udot, A_GB);

            // Constraint errors.
            // TODO double-check that disabled constraints don't show up in
            // state
            std::copy_n(simTKState.getQErr().getContiguousScalarData(),
                    this->m_mpSum, out.path.data());
            // TODO if (!get_enforce_holonomic_constraints_only()) {
            //    std::copy_n(simTKState.getUErr().getContiguousScalarData(),
            //          m_mpSum + m_mvSum, out.path.data() + m_mpSum);
            //    std::copy_n(simTKState.getUDotErr().getContiguousScalarData(),
            //        m_mpSum + m_mvSum + m_maSum,
            //        out.path.data() + 2*m_mpSum + m_mvSum);
            //}

            // Copy state derivative values to output struct. We cannot simply
            // use getYDot() because that requires realizing to Acceleration.
            const int nq = simTKState.getQ().size();
            const int nu = udot.size();
            const int nz = simTKState.getZ().size();
            std::copy_n(simTKState.getQDot().getContiguousScalarData(),
                    nq, out.dynamics.data());
            std::copy_n(udot.getContiguousScalarData(),
                    udot.size(), out.dynamics.data() + nq);
            std::copy_n(simTKState.getZDot().getContiguousScalarData(), nz,
                    out.dynamics.data() + nq + nu);

        } else {
            // TODO Antoine and Gil said realizing Dynamics is a lot costlier than
            // realizing to Velocity and computing forces manually.
            model.realizeAcceleration(simTKState);

            // Copy state derivative values to output struct.
            std::copy_n(simTKState.getYDot().getContiguousScalarData(),
                    states.size(), out.dynamics.data());
        }

        this->calcPathConstraintErrors(simTKState,
                out.path.data() + this->m_numMultibodyConstraintEqs);
    }
private:
    // This member variable avoids unnecessary extra allocation of memory for
    // spatial accelerations, which are incidental to the computation of
    // generalized accelerations when specifying the dynamics with model
    // constraints present.
    mutable SimTK::Vector_<SimTK::SpatialVec> A_GB;
};

template <typename T>
class MucoTropterSolver::ImplicitTropterProblem :
        public MucoTropterSolver::TropterProblemBase<T> {
public:
    ImplicitTropterProblem(const MucoTropterSolver& solver)
            : TropterProblemBase<T>(solver) {
        OPENSIM_THROW_IF(this->m_state.getNZ(), Exception,
                "Cannot use implicit dynamics mode if the system has auxiliary "
                "states.");
        OPENSIM_THROW_IF(this->m_numMultibodyConstraintEqs, Exception,
                "Cannot use implicit dynamics mode with multibody "
                "constraints.");
        // Add adjuncts for udot, which we call "w".
        int NU = this->m_state.getNU();
        OPENSIM_THROW_IF(NU != this->m_state.getNQ(), Exception,
                "Quaternions are not supported.");
        for (int iudot = 0; iudot < NU; ++iudot) {
            auto name = this->m_svNamesInSysOrder[iudot];
            auto leafpos = name.find("value");
            OPENSIM_THROW_IF(leafpos == std::string::npos, Exception,
                    "Internal error.");
            name.replace(leafpos, name.size(), "accel");
            // TODO: How to choose bounds on udot?
            this->add_adjunct(name, {-1000, 1000});
            this->add_path_constraint(name.substr(0, leafpos) + "residual", 0);
        }
    }
    void calc_differential_algebraic_equations(
            const tropter::Input<T>& in,
            tropter::Output<T> out) const override {

        const auto& states = in.states;
        const auto& controls = in.controls;
        const auto& adjuncts = in.adjuncts;

        auto& model = this->m_model;
        auto& simTKState = this->m_state;

        simTKState.setTime(in.time);
        const auto NQ = simTKState.getNQ(); // TODO we assume NQ = NU

        const auto& u = states.segment(NQ, NQ);
        const auto& w = adjuncts.segment(this->m_numMultibodyConstraintEqs, NQ);

        // Kinematic differential equations
        // --------------------------------
        // qdot = u
        // TODO does not work for quaternions!
        out.dynamics.head(NQ) = u;

        // Multibody dynamics: differential equations
        // ------------------------------------------
        // udot = w
        out.dynamics.segment(NQ, NQ) = w;


        // Multibody dynamics: "F - ma = 0"
        // --------------------------------
        std::copy_n(states.data(), states.size(),
                simTKState.updY().updContiguousScalarData());

        // TODO do not copy? I think this will still make a copy:
        // TODO use m_state.updY() = SimTK::Vector(states.size(), states.data(), true);
        //m_state.setY(SimTK::Vector(states.size(), states.data(), true));

        if (model.getNumControls()) {
            auto& osimControls = model.updControls(simTKState);
            std::copy_n(controls.data(), controls.size(),
                    osimControls.updContiguousScalarData());

            model.realizeVelocity(simTKState);
            model.setControls(simTKState, osimControls);
        }

        // TODO: Update to support multibody constraints, using
        // this->calcMultibodyConstraintForces()

        double* pathConstraintErrorBegin =
                out.path.data() + this->m_numMultibodyConstraintEqs;
        this->calcPathConstraintErrors(simTKState, pathConstraintErrorBegin);
        OPENSIM_THROW_IF(
                simTKState.getSystemStage() >= SimTK::Stage::Acceleration,
                Exception,
                "Cannot realize to Acceleration in implicit dynamics mode.");

        InverseDynamicsSolver id(model);
        SimTK::Vector udot((int)w.size(), w.data(), true);
        SimTK::Vector residual = id.solve(simTKState, udot);

        double* residualBegin =
                pathConstraintErrorBegin + this->m_numPathConstraintEqs;
        std::copy_n(residual.getContiguousScalarData(), residual.size(),
                residualBegin);

        // TODO Antoine and Gil said realizing Dynamics is a lot costlier than
        // realizing to Velocity and computing forces manually.

        /*
        if (SimTK::isNaN(in.adjuncts(0))) {
            std::cout << "DEBUG " << in.adjuncts << std::endl;
        }
        std::cout << "DEBUG dynamics\n" << out.dynamics << "\npath\n" << out.path << std::endl;
        std::cout << simTKState.getY() << std::endl;
        std::cout << residual << std::endl;
        std::cout << udot << std::endl;
        std::cout << "adjuncts " << adjuncts << std::endl;
        std::cout << "num multibody " << this->m_numMultibodyConstraintEqs;
         */
    }
    void calc_integral_cost(const tropter::Input<T>& in,
            T& integrand) const override final {
        TropterProblemBase<T>::calc_integral_cost(in, integrand);
        OPENSIM_THROW_IF(
                this->m_state.getSystemStage() >= SimTK::Stage::Acceleration,
                Exception,
                "Cannot realize to Acceleration in implicit dynamics mode.");
    }
    void calc_endpoint_cost(const T& final_time,
            const tropter::VectorX<T>& states,
            const tropter::VectorX<T>& parameters,
            T& cost) const override final {
        TropterProblemBase<T>::calc_endpoint_cost(final_time, states,
                parameters, cost);
        OPENSIM_THROW_IF(
                this->m_state.getSystemStage() >= SimTK::Stage::Acceleration,
                Exception,
                "Cannot realize to Acceleration in implicit dynamics mode.");
    }
};

} // namespace OpenSim


#endif // MUSCOLLO_TROPTERPROBLEM_H
