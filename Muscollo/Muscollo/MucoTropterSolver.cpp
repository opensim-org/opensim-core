/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoTropterSolver.cpp                                               *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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
#include "MucoTropterSolver.h"
#include "MucoProblem.h"
#include "MuscolloUtilities.h"

#include <OpenSim/Simulation/Manager/Manager.h>
#include <simbody/internal/Constraint.h>

#include <tropter/tropter.h>

using namespace OpenSim;

using tropter::VectorX;

/// The map provides the index of each state variable in
/// SimTK::State::getY() from its each state variable path string.
std::vector<std::string> createStateVariableNamesInSystemOrder(
        const Model& model) {
    std::vector<std::string> svNamesInSysOrder;
    auto s = model.getWorkingState();
    const auto svNames = model.getStateVariableNames();
    s.updY() = 0;
    for (int iy = 0; iy < s.getNY(); ++iy) {
        s.updY()[iy] = SimTK::NaN;
        const auto svValues = model.getStateVariableValues(s);
        for (int isv = 0; isv < svNames.size(); ++isv) {
            if (SimTK::isNaN(svValues[isv])) {
                svNamesInSysOrder.push_back(svNames[isv]);
                s.updY()[iy] = 0;
                break;
            }
        }
    }
    SimTK_ASSERT2_ALWAYS((size_t)svNames.size() == svNamesInSysOrder.size(),
            "Expected to get %i state names but found %i.", svNames.size(),
            svNamesInSysOrder.size());
    return svNamesInSysOrder;
}

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

MucoSolution convert(const tropter::Solution& tropSol) {
    // TODO enhance when solution contains more info than iterate.
    return convert<MucoSolution, tropter::Solution>(tropSol);
}

tropter::Iterate convert(const MucoIterate& mucoIter) {
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

tropter::Bounds convert(const MucoBounds& mb) {
    return {mb.getLower(), mb.getUpper()};
}
tropter::InitialBounds convert(const MucoInitialBounds& mb) {
    return {mb.getLower(), mb.getUpper()};
}
tropter::FinalBounds convert(const MucoFinalBounds& mb) {
    return {mb.getLower(), mb.getUpper()};
}

template <typename T>
class MucoTropterSolver::OCProblem : public tropter::Problem<T> {
public:
    OCProblem(const MucoTropterSolver& solver)
            // TODO set name properly.
            : tropter::Problem<T>(solver.getProblem().getName()),
              m_mucoTropterSolver(solver),
              m_mucoProb(solver.getProblem()),
              m_phase0(m_mucoProb.getPhase(0)) {
        m_model = m_phase0.getModel();
        // Disable all controllers.
        // TODO temporary; don't want to actually do this.
        m_model.finalizeFromProperties();
        auto controllers = m_model.updComponentList<Controller>();
        for (auto& controller : controllers) {
            controller.set_enabled(false);
        }
        m_state = m_model.initSystem();
        // TODO avoid multiple calls to initialize
        m_mucoProb.initialize(m_model);

        this->set_time(convert(m_phase0.getTimeInitialBounds()),
                convert(m_phase0.getTimeFinalBounds()));
        auto svNamesInSysOrder = createStateVariableNamesInSystemOrder(m_model);
        for (const auto& svName : svNamesInSysOrder) {
            const auto& info = m_phase0.getStateInfo(svName);
            this->add_state(svName, convert(info.getBounds()),
                    convert(info.getInitialBounds()),
                    convert(info.getFinalBounds()));
        }

        // Add any scalar constraints associated with multibody constraints in 
        // the model as path constraints in the problem.
        int cid, mp, mv, ma, numEquationsEnforced, multIndexThisConstraint;
        std::vector<MucoBounds> bounds;
        std::vector<std::string> labels;
        std::vector<KinematicLevel> kinLevels;
        std::vector<std::string> mcNames = 
            m_phase0.createMultibodyConstraintNames();
        for (const auto& mcName : mcNames) {
            const auto& mc = m_phase0.getMultibodyConstraint(mcName);
            const auto& multInfos = m_phase0.getMultiplierInfos(mcName);
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
            OPENSIM_THROW_IF(mv != 0, Exception, "Only holonomic "
                "(position-level) constraints are currently supported. "
                "There are " + std::to_string(mv) + " velocity-level "
                "scalar constraints associated with the model Constraint "
                "at ConstraintIndex " + std::to_string(cid) + ".");
            OPENSIM_THROW_IF(ma != 0, Exception, "Only holonomic "
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
        
        // Add any generic path constraints included in the problem. 
        for (std::string pcName : m_phase0.createPathConstraintNames()) {
            const MucoPathConstraint& constraint = 
                m_phase0.getPathConstraint(pcName);
            auto pcInfo = constraint.getConstraintInfo();
            auto labels = pcInfo.getConstraintLabels();
            auto bounds = pcInfo.getBounds();
            for (int i = 0; i < pcInfo.getNumEquations(); ++i) {
                this->add_path_constraint(labels[i], convert(bounds[i])); 
            }
        }
        m_numPathConstraintEqs = m_phase0.getNumPathConstraintEquations();
        // Allocate path constraint error memory.
        m_pathConstraintErrors.resize(m_numPathConstraintEqs);
        m_pathConstraintErrors.setToZero();

        for (const auto& actu : m_model.getComponentList<Actuator>()) {
            // TODO handle a variable number of control signals.
            const auto& actuName = actu.getAbsolutePathString();
            const auto& info = m_phase0.getControlInfo(actuName);
            this->add_control(actuName, convert(info.getBounds()),
                    convert(info.getInitialBounds()),
                    convert(info.getFinalBounds()));
        }
        for (std::string name : m_phase0.createParameterNames()) {
            const MucoParameter& parameter = m_phase0.getParameter(name);
            this->add_parameter(name, convert(parameter.getBounds()));
        }
    }
    void initialize_on_mesh(const Eigen::VectorXd&) const override {
        m_mucoProb.initialize(m_model);
    }
    void initialize_on_iterate(const Eigen::VectorXd& parameters)
            const override {
        // If they exist, apply parameter values to the model.
        this->applyParametersToModel(parameters);
    }
    void calc_differential_algebraic_equations(
            const tropter::Input<T>& in,
            tropter::Output<T> out) const override {

        // TODO convert to implicit formulation.

        const auto& states = in.states;
        const auto& controls = in.controls;
        const auto& adjuncts = in.adjuncts;

        m_state.setTime(in.time);
        std::copy(states.data(), states.data() + states.size(),
                &m_state.updY()[0]);
        //
        // TODO do not copy? I think this will still make a copy:
        // TODO use m_state.updY() = SimTK::Vector(states.size(), states.data(), true);
        //m_state.setY(SimTK::Vector(states.size(), states.data(), true));

        // Set the controls for actuators in the OpenSim model.
        if (m_model.getNumControls()) {
            auto& osimControls = m_model.updControls(m_state);
            std::copy(controls.data(), controls.data() + controls.size(),
                &osimControls[0]);
            m_model.realizeVelocity(m_state);
            m_model.setControls(m_state, osimControls);
        }

        // If enabled constraints exist in the model, compute accelerations
        // based on Lagrange multipliers.
        if (m_numMultibodyConstraintEqs) {
            // TODO Antoine and Gil said realizing Dynamics is a lot costlier than
            // realizing to Velocity and computing forces manually.
            m_model.realizeDynamics(m_state);

            const SimTK::MultibodySystem& multibody = 
                m_model.getMultibodySystem();
            const SimTK::Vector_<SimTK::SpatialVec>& appliedBodyForces =
                multibody.getRigidBodyForces(m_state, SimTK::Stage::Dynamics);
            const SimTK::Vector& appliedMobilityForces = 
                multibody.getMobilityForces(m_state, SimTK::Stage::Dynamics);

            const SimTK::SimbodyMatterSubsystem& matter = 
                m_model.getMatterSubsystem();
            SimTK::Vector_<SimTK::SpatialVec> constraintBodyForces;
            SimTK::Vector constraintMobilityForces;
            // Multipliers are negated so constraint forces can be used like 
            // applied forces.
            SimTK::Vector multipliers(m_numMultibodyConstraintEqs, 
                adjuncts.data());
            matter.calcConstraintForcesFromMultipliers(m_state, -multipliers,
                constraintBodyForces, constraintMobilityForces);

            SimTK::Vector& udot = m_state.updUDot();
            matter.calcAccelerationIgnoringConstraints(m_state,
                appliedMobilityForces + constraintMobilityForces,
                appliedBodyForces + constraintBodyForces, udot, A_GB);
           
            // Constraint errors.
            // TODO double-check that disable constraints don't show up in 
            // state
            std::copy(&m_state.getQErr()[0], 
                &m_state.getQErr()[0] + m_mpSum,
                out.path.data());
            // TODO if (!get_enforce_holonomic_constraints_only()) {
            //    std::copy(&m_state.getUErr()[0],
            //        &m_state.getUErr()[0] + m_mpSum + m_mvSum,
            //        out.path.data() + m_mpSum);
            //    std::copy(&m_state.getUDotErr()[0],
            //        &m_state.getUDotErr()[0] + m_mpSum + m_mvSum + m_maSum,
            //        out.path.data() + 2*m_mpSum + m_mvSum);
            //}

        } else {
            // TODO Antoine and Gil said realizing Dynamics is a lot costlier than
            // realizing to Velocity and computing forces manually.
            m_model.realizeAcceleration(m_state);
        }

        // Copy errors from generic path constraints into output struct.
        m_phase0.calcPathConstraintErrors(m_state, m_pathConstraintErrors);
        std::copy(m_pathConstraintErrors.begin(),
                  m_pathConstraintErrors.end(),
                  out.path.data() + m_numMultibodyConstraintEqs);

        // Copy state derivative values to output struct.
        std::copy(&m_state.getYDot()[0], &m_state.getYDot()[0] + states.size(),
                  out.dynamics.data());   
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

        integrand = m_phase0.calcIntegralCost(m_state);

        // TODO if (get_enforce_holonomic_constraints_only()) {
        // Add squared multiplers cost to integrand. Since we currently don't
        // include the derivatives of the holonomic contraint equations as path
        // constraints in the OCP, this term exists in order to impose
        // uniqueness in the Lagrange multipliers. 
        for (int i = 0; i < m_numMultibodyConstraintEqs; ++i) {
            integrand += m_mucoTropterSolver.get_multiplier_weight() 
                * adjuncts[i] * adjuncts[i];
        }
        // }
        
    }
    void calc_endpoint_cost(const T& final_time, const VectorX<T>& states,
            const VectorX<T>& /*parameters*/, T& cost) const override {
        // TODO avoid all of this if there are no endpoint costs.
        m_state.setTime(final_time);
        std::copy(states.data(), states.data() + states.size(),
                &m_state.updY()[0]);
        // TODO cannot use control signals...
        m_model.updControls(m_state).setToNaN();
        cost = m_phase0.calcEndpointCost(m_state);
    }

private:
    const MucoTropterSolver& m_mucoTropterSolver;
    const MucoProblem& m_mucoProb;
    const MucoPhase& m_phase0;
    mutable Model m_model;
    mutable SimTK::State m_state;
    // This member variable avoids unnecessary extra allocation of memory for
    // spatial accelerations, which are incidental to the computation of
    // generalized accelerations when specifying the dynamics with model 
    // constraints present.
    mutable SimTK::Vector_<SimTK::SpatialVec> A_GB;
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
    // Cached path constraint errors.
    mutable SimTK::Vector m_pathConstraintErrors;

    void applyParametersToModel(const VectorX<T>& parameters) const
    {
        if (parameters.size()) {
            // Warning: memory borrowed, not copied (when third argument to
            // SimTK::Vector constructor is true)
            SimTK::Vector mucoParams(
                (int)m_phase0.createParameterNames().size(),
                parameters.data(), true);

            m_phase0.applyParametersToModel(mucoParams);
            m_model.initSystem();
        }
    }
};

MucoTropterSolver::MucoTropterSolver() {
    constructProperties();
}

void MucoTropterSolver::constructProperties() {
    constructProperty_num_mesh_points(100);
    constructProperty_verbosity(2);
    constructProperty_optim_solver("ipopt");
    constructProperty_optim_max_iterations(-1);
    constructProperty_optim_convergence_tolerance(-1);
    constructProperty_optim_constraint_tolerance(-1);
    constructProperty_optim_hessian_approximation("limited-memory");
    constructProperty_optim_sparsity_detection("random");
    constructProperty_optim_ipopt_print_level(-1);
    constructProperty_multiplier_weight(100.0);
    // TODO constructProperty_enforce_holonomic_constraints_only(true);

    constructProperty_guess_file("");
}

std::shared_ptr<const tropter::Problem<double>>
MucoTropterSolver::getTropterProblem() const {
    if (!m_tropProblem) {
        m_tropProblem = std::make_shared<OCProblem<double>>(*this);
    }
    return m_tropProblem;
}

void MucoTropterSolver::clearProblemImpl() {
    clearGuess();
}

void MucoTropterSolver::setProblemImpl(const MucoProblem& /*problem*/) {
    clearProblemImpl();
}

MucoIterate MucoTropterSolver::createGuess(const std::string& type) const {
    OPENSIM_THROW_IF_FRMOBJ(
               type != "bounds"
            && type != "random"
            && type != "time-stepping",
            Exception,
            "Unexpected guess type '" + type +
            "'; supported types are 'bounds', 'random', and "
            "'time-stepping'.");
    auto ocp = getTropterProblem();

    if (type == "time-stepping") {
        return createGuessTimeStepping();
    }

    // TODO avoid performing error checks multiple times; use
    // isObjectUpToDateWithProperties();
    checkPropertyIsPositive(*this, getProperty_num_mesh_points());
    int N = get_num_mesh_points();

    checkPropertyInSet(*this, getProperty_optim_solver(), {"ipopt", "snopt"});
    tropter::DirectCollocationSolver<double> dircol(ocp, "trapezoidal",
            get_optim_solver(), N);

    tropter::Iterate tropIter;
    if (type == "bounds") {
        tropIter = dircol.make_initial_guess_from_bounds();
    } else if (type == "random") {
        tropIter = dircol.make_random_iterate_within_bounds();
    }
    return convert<MucoIterate, tropter::Iterate>(tropIter);
}

MucoIterate MucoTropterSolver::createGuessTimeStepping() const {
    const auto& problem = getProblem();
    const auto& phase = problem.getPhase();
    const auto& initialTime = phase.getTimeInitialBounds().getUpper();
    const auto& finalTime = phase.getTimeFinalBounds().getLower();
    OPENSIM_THROW_IF_FRMOBJ(finalTime <= initialTime, Exception,
        "Expected lower bound on final time to be greater than "
        "upper bound on initial time, but "
        "final_time.lower: " + std::to_string(finalTime) + "; " +
        "initial_time.upper: " + std::to_string(initialTime) + ".");
    Model model(phase.getModel());

    // Disable all controllers?
    SimTK::State state = model.initSystem();

    // Modify initial state values as necessary.
    Array<std::string> svNames = model.getStateVariableNames();
    for (int isv = 0; isv < svNames.getSize(); ++isv) {
        const auto& svName = svNames[isv];
        const auto& initBounds = phase.getStateInfo(svName).getInitialBounds();
        const auto defaultValue = model.getStateVariableValue(state, svName);
        SimTK::Real valueToUse = defaultValue;
        if (initBounds.isEquality()) {
            valueToUse = initBounds.getLower();
        } else if (!initBounds.isWithinBounds(defaultValue)) {
            valueToUse = 0.5 * (initBounds.getLower() + initBounds.getUpper());
        }
        if (valueToUse != defaultValue) {
            model.setStateVariableValue(state, svName, valueToUse);
        }
    }

    // TODO Equilibrate fiber length?

    state.setTime(initialTime);
    Manager manager(model, state);
    manager.integrate(finalTime);

    const auto& statesTable = manager.getStatesTable();
    auto controlsTable = model.getControlsTable();

    // Fix column labels.
    auto labels = controlsTable.getColumnLabels();
    for (auto& label : labels) { label = "/forceset/" + label; }
    controlsTable.setColumnLabels(labels);

    // TODO handle parameters.
    return MucoIterate::createFromStatesControlsTables(
            problem, statesTable, controlsTable);
}

void MucoTropterSolver::setGuess(MucoIterate guess) {
    // Ensure the guess is compatible with this solver/problem.
    // Make sure to initialize the problem. TODO put in a better place.
    getTropterProblem();
    guess.isCompatible(getProblem(), true);
    clearGuess();
    m_guessFromAPI = std::move(guess);
}
void MucoTropterSolver::setGuessFile(const std::string& file) {
    clearGuess();
    set_guess_file(file);
}
void MucoTropterSolver::clearGuess() {
    m_guessFromAPI = MucoIterate();
    m_guessFromFile = MucoIterate();
    set_guess_file("");
    m_guessToUse.reset();
}
const MucoIterate& MucoTropterSolver::getGuess() const {
    if (!m_guessToUse) {
        if (get_guess_file() != "" && m_guessFromFile.empty()) {
            // The API should make it impossible for both guessFromFile and
            // guessFromAPI to be non-empty.
            assert(m_guessFromAPI.empty());
            // No need to load from file again if we've already loaded it.
            MucoIterate guessFromFile(get_guess_file());
            guessFromFile.isCompatible(getProblem(), true);
            m_guessFromFile = guessFromFile;
            m_guessToUse.reset(&m_guessFromFile);
        } else {
            // This will either be a guess specified via the API, or empty to
            // signal that tropter should use the default guess.
            m_guessToUse.reset(&m_guessFromAPI);
        }
    }
    return m_guessToUse.getRef();
}

void MucoTropterSolver::printOptimizationSolverOptions(std::string solver) {
    if (solver == "ipopt") {
        tropter::optimization::IPOPTSolver::print_available_options();
    } else {
        std::cout << "No info available for " << solver << " options." <<
                std::endl;
    }
}

MucoSolution MucoTropterSolver::solveImpl() const {
    const Stopwatch stopwatch;

    auto ocp = getTropterProblem();

    checkPropertyInSet(*this, getProperty_verbosity(), {0, 1, 2});

    if (get_verbosity()) {
        std::cout << std::string(79, '=') << "\n";
        std::cout << "MucoTropterSolver starting.\n";
        std::cout << std::string(79, '-') << std::endl;
        getProblem().printDescription();
        // We can provide more detail about our problem than tropter can.
        // ocp->print_description();
    }

    // Apply settings/options.
    // -----------------------
    checkPropertyIsPositive(*this, getProperty_num_mesh_points());
    int N = get_num_mesh_points();

    checkPropertyInSet(*this, getProperty_optim_solver(), {"ipopt", "snopt"});

    tropter::DirectCollocationSolver<double> dircol(ocp, "trapezoidal",
            get_optim_solver(), N);

    dircol.set_verbosity(get_verbosity() >= 1);

    auto& optsolver = dircol.get_opt_solver();

    checkPropertyInRangeOrSet(*this, getProperty_optim_max_iterations(),
            0, std::numeric_limits<int>::max(), {-1});
    if (get_optim_max_iterations() != -1)
        optsolver.set_max_iterations(get_optim_max_iterations());

    checkPropertyInRangeOrSet(*this,
            getProperty_optim_convergence_tolerance(),
            0.0, SimTK::NTraits<double>::getInfinity(), {-1.0});
    if (get_optim_convergence_tolerance() != -1)
        optsolver.set_convergence_tolerance(get_optim_convergence_tolerance());
    checkPropertyInRangeOrSet(*this,
            getProperty_optim_constraint_tolerance(),
            0.0, SimTK::NTraits<double>::getInfinity(), {-1.0});
    if (get_optim_constraint_tolerance() != -1)
        optsolver.set_constraint_tolerance(get_optim_constraint_tolerance());

    optsolver.set_hessian_approximation(get_optim_hessian_approximation());

    if (get_optim_solver() == "ipopt") {
        checkPropertyInRangeOrSet(*this, getProperty_optim_ipopt_print_level(),
                0, 12, {-1});
        if (get_verbosity() < 2) {
            optsolver.set_advanced_option_int("print_level", 0);
        } else {
            if (get_optim_ipopt_print_level() != -1) {
                optsolver.set_advanced_option_int("print_level",
                        get_optim_ipopt_print_level());
            }
        }
    }

    checkPropertyInSet(*this, getProperty_optim_sparsity_detection(),
            {"random", "initial-guess"});
    optsolver.set_sparsity_detection(get_optim_sparsity_detection());

    // Set advanced settings.
    //for (int i = 0; i < getProperty_optim_solver_options(); ++i) {
    //    optsolver.set_advanced_option(TODO);
    //}

    tropter::Iterate tropIterate = convert(getGuess());
    tropter::Solution tropSolution = dircol.solve(tropIterate);

    if (get_verbosity()) {
        dircol.print_constraint_values(tropSolution);
    }

    MucoSolution mucoSolution = convert(tropSolution);

    // TODO move this to convert():
    MucoSolver::setSolutionStats(mucoSolution, tropSolution.success,
            tropSolution.status, tropSolution.num_iterations);

    if (get_verbosity()) {
        std::cout << std::string(79, '-') << "\n";
        std::cout << "Elapsed real time: "
                << stopwatch.getElapsedTimeFormatted() << ".\n";
        if (mucoSolution) {
            std::cout << "MucoTropterSolver succeeded!\n";
        } else {
            // TODO cout or cerr?
            std::cout << "MucoTropterSolver did NOT succeed:\n";
            std::cout << "  " << mucoSolution.getStatus() << "\n";
        }
        std::cout << std::string(79, '=') << std::endl;
    }

    return mucoSolution;
}
