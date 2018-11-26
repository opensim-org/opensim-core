#ifndef MUSCOLLO_TROPTERPROBLEM_H
#define MUSCOLLO_TROPTERPROBLEM_H
// TODO license block

// TODO create a MucoTropterSolver folder.

#include <tropter/tropter.h>

template <typename MucoIterateType, typename tropIterateType>
MucoIterateType convert(const tropIterateType& tropSol) {
    const auto& tropTime = tropSol.time;
    SimTK::Vector time((int)tropTime.size(), tropTime.data());
    const auto& state_names = tropSol.state_names;
    const auto& control_names = tropSol.control_names;
    const auto& multiplier_names = tropSol.adjunct_names;
    const auto& parameter_names = tropSol.parameter_names;

    int numTimes = (int)time.size();
    int numStates = (int)state_names.size();
    int numControls = (int)control_names.size();
    int numMultipliers = (int)multiplier_names.size();
    int numParameters = (int)parameter_names.size();
    // Create and populate states matrix.
    SimTK::Matrix states(numTimes, numStates);
    for (int itime = 0; itime < numTimes; ++itime) {
        for (int istate = 0; istate < numStates; ++istate) {
            states(itime, istate) = tropSol.states(istate, itime);
        }
    }
    // Instantiating a SimTK::Matrix with a zero row or column does not create
    // an empty matrix. For example,
    //      SimTK::Matrix controls(5, 0);
    // will create a matrix with five empty rows. So, for variables other than
    // states, only allocate memory if necessary. Otherwise, return an empty
    // matrix. This will prevent weird comparison errors between two iterates
    // that should be equal but have slightly different "empty" values.
    SimTK::Matrix controls;
    if (numControls) {
        controls.resize(numTimes, numControls);
        for (int itime = 0; itime < numTimes; ++itime) {
            for (int icontrol = 0; icontrol < numControls; ++icontrol) {
                controls(itime, icontrol) = tropSol.controls(icontrol, itime);
            }
        }
    }
    SimTK::Matrix multipliers;
    if (numMultipliers) {
        multipliers.resize(numTimes, numMultipliers);
        for (int itime = 0; itime < numTimes; ++itime) {
            for (int imultiplier = 0; imultiplier < numMultipliers;
                 ++imultiplier) {
                multipliers(itime, imultiplier) = tropSol.adjuncts(imultiplier,
                        itime);
            }
        }
    }
    // This produces an empty RowVector if numParameters is zero.
    SimTK::RowVector parameters(numParameters, tropSol.parameters.data());
    return {time, state_names, control_names, multiplier_names, parameter_names,
            states, controls, multipliers, parameters};
}

OpenSim::MucoSolution convert(const tropter::Solution& tropSol) {
    // TODO enhance when solution contains more info than iterate.
    return convert<OpenSim::MucoSolution, tropter::Solution>(tropSol);
}

tropter::Iterate convert(const OpenSim::MucoIterate& mucoIter) {
    tropter::Iterate tropIter;
    if (mucoIter.empty()) return tropIter;

    using Eigen::Map;
    using Eigen::RowVectorXd;
    using Eigen::MatrixXd;
    using Eigen::VectorXd;

    const auto& time = mucoIter.getTime();
    tropIter.time = Map<const RowVectorXd>(&time[0], time.size());

    tropIter.state_names = mucoIter.getStateNames();
    tropIter.control_names = mucoIter.getControlNames();
    tropIter.adjunct_names = mucoIter.getMultiplierNames();
    tropIter.parameter_names = mucoIter.getParameterNames();

    int numTimes = (int)time.size();
    int numStates = (int)tropIter.state_names.size();
    int numControls = (int)tropIter.control_names.size();
    int numMultipliers = (int)tropIter.adjunct_names.size();
    int numParameters = (int)tropIter.parameter_names.size();
    const auto& states = mucoIter.getStatesTrajectory();
    const auto& controls = mucoIter.getControlsTrajectory();
    const auto& multipliers = mucoIter.getMultipliersTrajectory();
    const auto& parameters = mucoIter.getParameters();
    // Muscollo's matrix is numTimes x numStates;
    // tropter's is numStates x numTimes.
    tropIter.states = Map<const MatrixXd>(
            &states(0, 0), numTimes, numStates).transpose();
    if (numControls) {
        tropIter.controls = Map<const MatrixXd>(
                &controls(0, 0), numTimes, numControls).transpose();
    } else {
        tropIter.controls.resize(numControls, numTimes);
    }
    if (numMultipliers) {
        tropIter.adjuncts = Map<const MatrixXd>(
                &multipliers(0, 0), numTimes, numMultipliers).transpose();
    } else {
        tropIter.adjuncts.resize(numMultipliers, numTimes);
    }
    if (numParameters) {
        tropIter.parameters = Map<const VectorXd>(
                &parameters(0), numParameters);
    } else {
        tropIter.parameters.resize(numParameters);
    }
    return tropIter;
}

tropter::Bounds convert(const OpenSim::MucoBounds& mb) {
    return {mb.getLower(), mb.getUpper()};
}
tropter::InitialBounds convert(const OpenSim::MucoInitialBounds& mb) {
    return {mb.getLower(), mb.getUpper()};
}
tropter::FinalBounds convert(const OpenSim::MucoFinalBounds& mb) {
    return {mb.getLower(), mb.getUpper()};
}

template <typename T>
class OpenSim::MucoTropterSolver::TropterProblemBase
        : public tropter::Problem<T> {
protected:
    TropterProblemBase(const OpenSim::MucoTropterSolver& solver)
            : tropter::Problem<T>(solver.getProblemRep().getName()),
            m_mucoTropterSolver(solver),
            m_mucoProb(solver.getProblemRep()),
            m_model(m_mucoProb.getModel()) {
        // TODO set name properly.
        // Disable all controllers.
        // TODO temporary; don't want to actually do this.
        auto controllers = m_model.getComponentList<OpenSim::Controller>();
        for (auto& controller : controllers) {
            OPENSIM_THROW_IF(controller.get_enabled(), OpenSim::Exception,
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
        this->set_time(convert(m_mucoProb.getTimeInitialBounds()),
                convert(m_mucoProb.getTimeFinalBounds()));
        for (const auto& svName : m_svNamesInSysOrder) {
            const auto& info = m_mucoProb.getStateInfo(svName);
            this->add_state(svName, convert(info.getBounds()),
                    convert(info.getInitialBounds()),
                    convert(info.getFinalBounds()));
        }
    }

    void addControlVariables() {
        for (const auto& actu : m_model.getComponentList<Actuator>()) {
            // TODO handle a variable number of control signals.
            const auto& actuName = actu.getAbsolutePathString();
            const auto& info = m_mucoProb.getControlInfo(actuName);
            this->add_control(actuName, convert(info.getBounds()),
                    convert(info.getInitialBounds()),
                    convert(info.getFinalBounds()));
        }
    }

    void addMultibodyConstraints() {
        // Add any scalar constraints associated with multibody constraints in
        // the model as path constraints in the problem.
        int cid, mp, mv, ma, numEquationsEnforced, multIndexThisConstraint;
        std::vector<OpenSim::MucoBounds> bounds;
        std::vector<std::string> labels;
        std::vector<OpenSim::KinematicLevel> kinLevels;
        std::vector<std::string> mcNames =
                m_mucoProb.createMultibodyConstraintNames();
        for (const auto& mcName : mcNames) {
            const auto& mc = m_mucoProb.getMultibodyConstraint(mcName);
            const auto& multInfos = m_mucoProb.getMultiplierInfos(mcName);
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
            OPENSIM_THROW_IF(mv != 0, OpenSim::Exception,
                    "Only holonomic "
                    "(position-level) constraints are currently supported. "
                    "There are " + std::to_string(mv) + " velocity-level "
                    "scalar constraints associated with the model Constraint "
                    "at ConstraintIndex " + std::to_string(cid) + ".");
            OPENSIM_THROW_IF(ma != 0, OpenSim::Exception,
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
                this->add_path_constraint(labels[i], convert(bounds[i]));

                // If the index for this path constraint represents an
                // a non-derivative scalar constraint equation, also add a
                // Lagrange multplier to the problem.
                if (kinLevels[i] == KinematicLevel::Position ||
                        kinLevels[i] == KinematicLevel::Velocity ||
                        kinLevels[i] == KinematicLevel::Acceleration) {

                    const auto& multInfo = multInfos[multIndexThisConstraint];
                    this->add_adjunct(multInfo.getName(),
                            convert(multInfo.getBounds()),
                            convert(multInfo.getInitialBounds()),
                            convert(multInfo.getFinalBounds()));
                    ++multIndexThisConstraint;
                }
            }

            m_numMultibodyConstraintEqs += numEquationsEnforced;
        }

    }

    /// Add any generic path constraints included in the problem.
    void addGenericPathConstraints() {
        for (std::string pcName : m_mucoProb.createPathConstraintNames()) {
            const MucoPathConstraint& constraint =
                    m_mucoProb.getPathConstraint(pcName);
            auto pcInfo = constraint.getConstraintInfo();
            auto labels = pcInfo.getConstraintLabels();
            auto bounds = pcInfo.getBounds();
            for (int i = 0; i < pcInfo.getNumEquations(); ++i) {
                this->add_path_constraint(labels[i], convert(bounds[i]));
            }
        }
        m_numPathConstraintEqs = m_mucoProb.getNumPathConstraintEquations();
    }

    void addParameters() {
        for (std::string name : m_mucoProb.createParameterNames()) {
            const MucoParameter& parameter = m_mucoProb.getParameter(name);
            this->add_parameter(name, convert(parameter.getBounds()));
        }
    }

    void initialize_on_iterate(const Eigen::VectorXd& parameters)
            const override final {
        // If they exist, apply parameter values to the model.
        this->applyParametersToModel(parameters);
    }

    void calc_integral_cost(const tropter::Input<T>& in,
            T& integrand) const override final {
        // Unpack variables.
        const auto& time = in.time;
        const auto& states = in.states;
        const auto& controls = in.controls;
        const auto& adjuncts = in.adjuncts;

        // TODO would it make sense to a vector of States, one for each mesh
        // point, so that each can preserve their cache?
        m_state.setTime(time);
        std::copy(states.data(), states.data() + states.size(),
                &m_state.updY()[0]);

        // Set the controls for actuators in the OpenSim model.
        if (m_model.getNumControls()) {
            auto& osimControls = m_model.updControls(m_state);
            std::copy(controls.data(), controls.data() + controls.size(),
                    &osimControls[0]);
            m_model.realizePosition(m_state);
            m_model.setControls(m_state, osimControls);
        } else {
            m_model.realizePosition(m_state);
        }

        integrand = m_mucoProb.calcIntegralCost(m_state);

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
        // }

    }

    void calc_endpoint_cost(const T& final_time,
            const tropter::VectorX<T>& states,
            const tropter::VectorX<T>& /*parameters*/,
            T& cost) const override {
        // TODO avoid all of this if there are no endpoint costs.
        m_state.setTime(final_time);
        std::copy(states.data(), states.data() + states.size(),
                &m_state.updY()[0]);
        // TODO cannot use control signals...
        m_model.updControls(m_state).setToNaN();
        cost = m_mucoProb.calcEndpointCost(m_state);
    }

    const MucoTropterSolver& m_mucoTropterSolver;
    const MucoProblemRep& m_mucoProb;
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
                    (int)m_mucoProb.createParameterNames().size(),
                    parameters.data(), true);

            m_mucoProb.applyParametersToModel(mucoParams);
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
        m_mucoProb.calcPathConstraintErrors(state, pathConstraintErrors);
    }
};


template <typename T>
class OpenSim::MucoTropterSolver::ExplicitTropterProblem
        : public OpenSim::MucoTropterSolver::TropterProblemBase<T> {
public:
    ExplicitTropterProblem(const OpenSim::MucoTropterSolver& solver)
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
        const auto& adjuncts = in.adjuncts;

        auto& model = this->m_model;
        auto& simTKState = this->m_state;

        simTKState.setTime(in.time);
        std::copy_n(states.data(), states.size(), &simTKState.updY()[0]);
        //
        // TODO do not copy? I think this will still make a copy:
        // TODO use m_state.updY() = SimTK::Vector(states.size(), states.data(), true);
        //m_state.setY(SimTK::Vector(states.size(), states.data(), true));

        // Set the controls for actuators in the OpenSim model.
        if (model.getNumControls()) {
            auto& osimControls = model.updControls(simTKState);
            std::copy_n(controls.data(), controls.size(), &osimControls[0]);
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
            std::copy_n(&simTKState.getQErr()[0], this->m_mpSum,
                    out.path.data());
            // TODO if (!get_enforce_holonomic_constraints_only()) {
            //    std::copy_n(&simTKState.getUErr()[0], m_mpSum + m_mvSum,
            //        out.path.data() + m_mpSum);
            //    std::copy_n(&simTKState.getUDotErr()[0],
            //        m_mpSum + m_mvSum + m_maSum,
            //        out.path.data() + 2*m_mpSum + m_mvSum);
            //}

            // Copy state derivative values to output struct. We cannot simply
            // use getYDot() because that requires realizing to Acceleration.
            const int nq = simTKState.getQ().size();
            const int nu = udot.size();
            const int nz = simTKState.getZ().size();
            std::copy_n(&simTKState.getQDot()[0], nq, out.dynamics.data());
            std::copy_n(&udot[0], udot.size(), out.dynamics.data() + nq);
            if (nz) {
                std::copy_n(&simTKState.getZDot()[0], nz,
                        out.dynamics.data() + nq + nu);
            }

        } else {
            // TODO Antoine and Gil said realizing Dynamics is a lot costlier than
            // realizing to Velocity and computing forces manually.
            model.realizeAcceleration(simTKState);

            // Copy state derivative values to output struct.
            std::copy_n(&simTKState.getYDot()[0], states.size(),
                    out.dynamics.data());
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
class OpenSim::MucoTropterSolver::ImplicitTropterProblem :
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

        // Kinematic differential equations
        // --------------------------------
        // qdot = u
        // TODO does not work for quaternions!
        const auto NQ = simTKState.getNQ(); // TODO we assume NQ = NU
        const auto& u = states.segment(NQ, NQ);
        out.dynamics.head(NQ) = u;

        // Multibody dynamics: differential equations
        // ------------------------------------------
        // udot = w
        out.dynamics.segment(NQ, NQ) = controls.head(NQ);


        // Multibody dynamics: "F - ma = 0"
        // --------------------------------
        std::copy(states.data(), states.data() + states.size(),
                &this->m_state.updY()[0]);

        // TODO do not copy? I think this will still make a copy:
        // TODO use m_state.updY() = SimTK::Vector(states.size(), states.data(), true);
        //m_state.setY(SimTK::Vector(states.size(), states.data(), true));

        if (model.getNumControls()) {
            auto& osimControls = model.updControls(simTKState);
            std::copy(controls.data() + NQ,
                    controls.data() + controls.size(),
                    &osimControls[0]);

            model.realizeVelocity(simTKState);
            model.setControls(this->m_state, osimControls);
        }

        // TODO: Update to support multibody constraints, using
        // this->calcMultibodyConstraintForces()

        InverseDynamicsSolver id(model);
        const double* udotBegin =
                adjuncts.data() + this->m_numMultibodyConstraintEqs;
        SimTK::Vector udot(NQ, udotBegin, true);
        SimTK::Vector residual = id.solve(simTKState, udot);

        std::copy_n(&residual[0], residual.size(),
                out.path.data());

        // TODO Antoine and Gil said realizing Dynamics is a lot costlier than
        // realizing to Velocity and computing forces manually.

        this->calcPathConstraintErrors(simTKState,
                out.path.data() +
                residual.size() + this->m_numMultibodyConstraintEqs);
    }
};

#endif // MUSCOLLO_TROPTERPROBLEM_H
