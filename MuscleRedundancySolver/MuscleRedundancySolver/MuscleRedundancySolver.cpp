
#include "MuscleRedundancySolver.h"
#include "DeGroote2016Muscle.h"
#include "InverseMuscleSolverMotionData.h"
#include "GlobalStaticOptimizationSolver.h"

#include <mesh.h>

#include <algorithm>

// TODO to get a good initial guess: solve without pennation! avoid all those
//         sqrts!


using namespace OpenSim;

void MuscleRedundancySolver::Solution::write(const std::string& prefix) const
{
    auto write = [&](const TimeSeriesTable& table, const std::string& suffix)
    {
        if (table.getNumRows()) {
            STOFileAdapter_<double>::write(table,
                                           prefix + "_" + suffix + ".sto");
        }
    };
    write(excitation, "excitation");
    write(activation, "activation");
    write(norm_fiber_length, "norm_fiber_length");
    write(norm_fiber_velocity, "norm_fiber_velocity");
    write(other_controls, "other_controls");
    write(tendon_force, "tendon_force");
}

/// "Separate" denotes that the dynamics are not coming from OpenSim, but
/// rather are coded separately.
template<typename T>
class MRSProblemSeparate : public mesh::OptimalControlProblemNamed<T> {
public:
    MRSProblemSeparate(const MuscleRedundancySolver& mrs,
                       const Model& model,
                       const InverseMuscleSolverMotionData& motionData)
            : mesh::OptimalControlProblemNamed<T>("MRS"),
              _mrs(mrs), _model(model), _motionData(motionData) {
        SimTK::State state = _model.initSystem();

        // Set the time bounds.
        _initialTime = motionData.getInitialTime();
        _finalTime = motionData.getFinalTime();
        this->set_time({_initialTime}, {_finalTime});

        // States and controls for actuators.
        // ----------------------------------

        // TODO handle different actuator types more systematically.
        auto actuators = _model.getComponentList<ScalarActuator>();
        for (const auto& actuator : actuators) {
            if (actuator.get_appliesForce() &&
                    !dynamic_cast<const Muscle*>(&actuator) &&
                    !dynamic_cast<const CoordinateActuator*>(&actuator)) {
                throw std::runtime_error("[MRS] Only Muscles and "
                        "CoordinateActuators are currently supported but the"
                        " model contains an enabled "
                        + actuator.getConcreteClassName() + ". Either set "
                        "appliesForce=false for this actuator, or remove it "
                        "from the model.");
            }
        }

        // CoordinateActuators.
        // --------------------
        _numCoordActuators = 0;
        for (const auto& actuator :
                _model.getComponentList<CoordinateActuator>()) {
            if (!actuator.get_appliesForce()) continue;

            const auto& actuPath = actuator.getAbsolutePathName();
            this->add_control(actuPath + "_control",
                              {actuator.get_min_control(),
                               actuator.get_max_control()});

            _otherControlsLabels.push_back(actuPath);
            _numCoordActuators++;
        }
        const auto coordsOrdered = _model.getCoordinatesInMultibodyTreeOrder();
        _optimalForce.resize(_numCoordActuators);
        _coordActuatorDOFs.resize(_numCoordActuators);
        int i_act = 0;
        for (const auto& actuator :
                _model.getComponentList<CoordinateActuator>()) {
            if (!actuator.get_appliesForce()) continue;
            _optimalForce[i_act] = actuator.getOptimalForce();
            // Figure out which DOF each coordinate actuator is actuating.
            const auto* coord = actuator.getCoordinate();
            size_t i_coord = 0;
            while (i_coord < coordsOrdered.size() &&
                    coordsOrdered[i_coord].get() != coord) {
                ++i_coord;
            }
            if (i_coord == coordsOrdered.size()) {
                throw std::runtime_error("[MRS] Could not find Coordinate '" +
                        coord->getAbsolutePathName() + "' used in "
                        "CoordinateActuator '" + actuator.getAbsolutePathName()
                                                 + "'.");
            }
            _coordActuatorDOFs[i_act] = i_coord;
            ++i_act;
        }

        // Muscles.
        // --------
        _numMuscles = 0;
        for (const auto& actuator : _model.getComponentList<Muscle>()) {
            // TODO conslidate with MotionData.
            if (!actuator.get_appliesForce()) continue;

            const auto& actuPath = actuator.getAbsolutePathName();
            // Activation dynamics.
            this->add_control(actuPath + "_excitation",
                              {actuator.get_min_control(),
                               actuator.get_max_control()});
            // TODO use activation bounds, not excitation bounds (then
            // also update the property comment for zero_initial_activation).
            const double initialActiv = mrs.get_zero_initial_activation() ?
                    std::max(actuator.get_min_control(), 0.0) : SimTK::NaN;
            this->add_state(actuPath + "_activation",
                            {actuator.get_min_control(),
                             actuator.get_max_control()},
                            initialActiv);

            // Fiber dynamics.
            // TODO initial value should be 0? That is what CMC does (via
            // equilibrateMuscles()).
            this->add_control(actuPath + "_norm_fiber_velocity", {-1, 1});
            // These bounds come from simtk.org/projects/optcntrlmuscle.
            this->add_state(actuPath + "_norm_fiber_length", {0.2, 1.8});
            this->add_path_constraint(
                    actuator.getAbsolutePathName() + "_equilibrium", 0);

            _muscleLabels.push_back(actuPath);
            _numMuscles++;
        }

        // Create De Groote muscles.
        _muscles.resize(_numMuscles);
        int i_mus = 0;
        for (const auto& osimMus : _model.getComponentList<Muscle>()) {
            if (!osimMus.get_appliesForce()) continue;

            _muscles[i_mus] = DeGroote2016Muscle<T>(
                    osimMus.get_max_isometric_force(),
                    osimMus.get_optimal_fiber_length(),
                    osimMus.get_tendon_slack_length(),
                    osimMus.get_pennation_angle_at_optimal(),
                    osimMus.get_max_contraction_velocity());

            i_mus++;
        }

        // Add a constraint for each joint moment.
        _numDOFs = 0;
        for (int i = 0; i < state.getNU(); ++i) {
            this->add_path_constraint("joint_moment_" + std::to_string(i), 0);
            _numDOFs++;
        }

    }

    void initialize_on_mesh(const Eigen::VectorXd& mesh) const override {

        // For caching desired joint moments.
        auto* mutableThis = const_cast<MRSProblemSeparate<T>*>(this);

        Eigen::VectorXd times = (_finalTime - _initialTime) * mesh;
        _motionData.interpolateNetMoments(times, mutableThis->_desiredMoments);
        if (_numMuscles) {
            _motionData.interpolateMuscleTendonLengths(times,
                     mutableThis->_muscleTendonLengths);
            _motionData.interpolateMomentArms(times,
                     mutableThis->_momentArms);
        }
    }

    void dynamics(const mesh::VectorX<T>& states,
                  const mesh::VectorX<T>& controls,
                  Eigen::Ref<mesh::VectorX<T>> derivatives) const override {
        // Actuator dynamics.
        // ==================
        for (Eigen::Index i_act = 0; i_act < _numMuscles; ++i_act) {

            // Unpack variables.
            const T& excitation = controls[_numCoordActuators + 2 * i_act];
            const T& activation = states[2 * i_act];
            const T& normFibVel = controls[_numCoordActuators + 2 * i_act + 1];

            // Activation dynamics.
            _muscles[i_act].calcActivationDynamics(excitation, activation,
                                                   derivatives[2 * i_act]);

            // Fiber dynamics.
            derivatives[2 * i_act + 1] =
                    _muscles[i_act].get_max_contraction_velocity() * normFibVel;
        }

        // TODO std::cout << "DEBUG dynamics " << derivatives << std::endl;
    }
    void path_constraints(unsigned i_mesh,
                          const T& /*time*/,
                          const mesh::VectorX<T>& states,
                          const mesh::VectorX<T>& controls,
                          Eigen::Ref<mesh::VectorX<T>> constraints)
            const override {
        // Actuator equilibrium.
        // =====================

        // Assemble generalized forces to apply to the joints.
        mesh::VectorX<T> genForce(_numDOFs);
        genForce.setZero();

        // CoordinateActuators.
        // --------------------
        for (Eigen::Index i_act = 0; i_act < _numCoordActuators; ++i_act) {
            genForce[_coordActuatorDOFs[i_act]]
                    += _optimalForce[i_act] * controls[i_act];
        }

        // Muscles.
        // --------
        if (_numMuscles) {
            mesh::VectorX<T> tendonForces(_numMuscles);
            for (Eigen::Index i_act = 0; i_act < _numMuscles; ++i_act) {
                // Unpack variables.
                const T& activation = states[2 * i_act];
                const T& normFibVel = controls[_numCoordActuators+2*i_act+1];
                const T& normFibLen = states[2 * i_act + 1];

                // Get the total muscle-tendon length from the data.
                const T& musTenLen = _muscleTendonLengths(i_act, i_mesh);

                T normTenForce;
                _muscles[i_act].calcEquilibriumResidual(activation, musTenLen,
                                                        normFibLen, normFibVel,
                                                        constraints[i_act],
                                                        normTenForce);

                tendonForces[i_act] = _muscles[i_act].get_max_isometric_force()
                        * normTenForce;
            }

            // Compute generalized forces from muscles.
            const auto& momArms = _momentArms[i_mesh];
            genForce += momArms.template cast<adouble>() * tendonForces;
        }


        // Achieve the motion.
        // ===================
        constraints.segment(_numMuscles, _numDOFs)
                = _desiredMoments.col(i_mesh).template cast<adouble>()
                - genForce;
    }
    void integral_cost(const T& /*time*/,
                       const mesh::VectorX<T>& /*states*/,
                       const mesh::VectorX<T>& controls,
                       T& integrand) const override {
        // Use a map to skip over fiber velocities.
        using ExcitationsVector = Eigen::Map<const mesh::VectorX<T>,
                /* pointer alignment: Unaligned */   0,
                /* pointer increment btn elements */ Eigen::InnerStride<2>>;
        ExcitationsVector muscleExcit(controls.data() + _numCoordActuators,
                                      _numMuscles);

        // We attempted to minimize activations to prevent the initial
        // activation from being incorrectly large (b/c no penalty), but this
        // did not end up being a solution to the problem.
        // TODO could also just minimize initial activation.
        // Use a map to skip over fiber lengths.
        // using ActivationsVector = Eigen::Map<const mesh::VectorX<T>,
        //         /* pointer alignment: Unaligned */   0,
        //         /* pointer increment btn elements */ Eigen::InnerStride<2>>;
        // ActivationsVector muscleActiv(states.data() + _numCoordActuators,
        //                               _numMuscles);

        integrand = controls.head(_numCoordActuators).squaredNorm()
                  + muscleExcit.squaredNorm();
    }
    /// If mrsVars seems to be empty (no rows in either the
    /// mrsVars.activation or mrsVars.other_controls tables), then this returns
    /// an empty iterate.
    // TODO should take a "Iterate" instead.
    mesh::OptimalControlIterate construct_iterate(
            const MuscleRedundancySolver::Solution& mrsVars) const {
        using Eigen::Map;
        using Eigen::VectorXd;
        using Eigen::MatrixXd;

        mesh::OptimalControlIterate vars;
        // The mrsVars has time as the row dimension, but mrsVars has time as
        // the column dimension. As a result, some quantities must be
        // transposed.
        size_t numTimes;
        if (mrsVars.activation.getNumRows() != 0) {
            numTimes = mrsVars.activation.getNumRows();
            vars.time = Map<const VectorXd>(
                    mrsVars.activation.getIndependentColumn().data(), numTimes);
        } else if (mrsVars.other_controls.getNumRows() != 0) {
            numTimes = mrsVars.other_controls.getNumRows();
            vars.time = Map<const VectorXd>(
                    mrsVars.other_controls.getIndependentColumn().data(),
                    numTimes);
        } else {
            // mrsVars seems to be empty (no muscles or other controls);
            // return an empty OptimalControlIterate.
            return vars;
        }

        // Each muscle has excitation and norm. fiber velocity controls.
        vars.controls.resize(_numCoordActuators + 2 * _numMuscles, numTimes);
        // Each muscle has activation and fiber length states.
        vars.states.resize(2 * _numMuscles, numTimes);

        if (_numCoordActuators) {
            vars.controls.topRows(_numCoordActuators) = Map<const MatrixXd>(
                            &mrsVars.other_controls.getMatrix()(0, 0),
                            numTimes, _numCoordActuators).transpose();
        }

        if (_numMuscles) {
            // SkipRowView is a view onto a Matrix wherein every other row of
            // the original matrix is skipped. We use this to edit every
            // other row using a single assignment. Eigen's inner stride (the
            // second template argument below) is the pointer increment between
            // two consecutive entries; so a stride of 2 skips 1 row.
            // The first argument is the outer stride (increment between
            // columns), which we only know during run-time (e.g., Dynamic).
            using SkipRowStride = Eigen::Stride<Eigen::Dynamic, 2>;
            // These tell Map to interpret the passed-in data as a matrix;
            // the outer stride is just the number of rows (in this case).
            SkipRowStride controlsStride(vars.controls.outerStride(), 2);
            SkipRowStride statesStride(vars.states.outerStride(), 2);
            using SkipRowView = Map<MatrixXd, Eigen::Unaligned, SkipRowStride>;

            SkipRowView excitationView(
                    // Skip over coordinate actuator controls.
                    vars.controls.bottomRows(2 *_numMuscles).data(),
                    _numMuscles, numTimes, controlsStride);
            excitationView = Map<const MatrixXd>(
                    &mrsVars.excitation.getMatrix()(0, 0),
                    numTimes, _numMuscles).transpose();

            SkipRowView activationView(
                    vars.states.data(), _numMuscles, numTimes, statesStride);
            activationView = Map<const MatrixXd>(
                    &mrsVars.activation.getMatrix()(0, 0),
                    numTimes, _numMuscles).transpose();

            SkipRowView fiberLengthView(
                    // Skip the first state row, which has activations
                    // for the first muscle.
                    vars.states.bottomRows(2 * _numMuscles - 1).data(),
                    _numMuscles, numTimes, statesStride);
            fiberLengthView = Map<const MatrixXd>(
                    &mrsVars.norm_fiber_length.getMatrix()(0, 0),
                    numTimes, _numMuscles).transpose();

            SkipRowView fiberVelocityView(
                    // Skip the coordinate actuator rows and the excitation
                    // for the first muscle.
                    vars.controls.bottomRows(2 * _numMuscles - 1).data(),
                    _numMuscles, numTimes, controlsStride);
            fiberVelocityView = Map<const MatrixXd>(
                    &mrsVars.norm_fiber_velocity.getMatrix()(0, 0),
                    numTimes, _numMuscles).transpose();
        }
        return vars;
    }
    /// The mesh points in the iterate must match those that this solver is
    /// expecting (for the current mesh refinement iteration); this is
    /// because computing tendon length requires on the cached muscle-tendon
    /// lengths.
    MuscleRedundancySolver::Solution deconstruct_iterate(
            const mesh::OptimalControlIterate& ocpVars) const {

        MuscleRedundancySolver::Solution sol;
        if (_numCoordActuators) {
            sol.other_controls.setColumnLabels(_otherControlsLabels);
        }
        if (_numMuscles) {
            sol.excitation.setColumnLabels(_muscleLabels);
            sol.activation.setColumnLabels(_muscleLabels);
            sol.norm_fiber_length.setColumnLabels(_muscleLabels);
            sol.norm_fiber_velocity.setColumnLabels(_muscleLabels);
            sol.tendon_force.setColumnLabels(_muscleLabels);
        }

        for (int i_time = 0; i_time < ocpVars.time.cols(); ++i_time) {
            const auto& time = ocpVars.time[i_time];
            const auto& controls = ocpVars.controls.col(i_time);
            const auto& states = ocpVars.states.col(i_time);

            // Other controls.
            // ---------------
            // The first _numCoordActuators rows of the controls matrix
            // are for the CoordinateActuators.
            if (_numCoordActuators) {
                SimTK::RowVector other_controls(_numCoordActuators,
                                                controls.data(),
                                                true /* <- this is a view */);
                sol.other_controls.appendRow(time, other_controls);
            }

            // Muscle-related quantities.
            // --------------------------
            if (_numMuscles == 0) continue;
            SimTK::RowVector excitation(_numMuscles,
                                        2 /* stride: skip over fiber vel. */,
                                        controls.data() + _numCoordActuators,
                                        true /* makes this a view */);
            sol.excitation.appendRow(time, excitation);

            SimTK::RowVector activation(_numMuscles,
                                        2 /* stride: skip over fiber length */,
                                        states.data(),
                                        true /* makes this a view */);
            sol.activation.appendRow(time, activation);

            SimTK::RowVector fiber_length(_numMuscles, /*TODO*/
                                          2 /* stride: skip over activation */,
                                          states.data() + 1,
                                          true /* makes this a view */);
            sol.norm_fiber_length.appendRow(time, fiber_length);

            SimTK::RowVector fiber_velocity(_numMuscles,
                                       2 /* stride: skip over excit. */,
                                       controls.data() + _numCoordActuators + 1,
                                       true /* makes this a view */);
            sol.norm_fiber_velocity.appendRow(time, fiber_velocity);

            // Compute other helpful quantities from the states.
            SimTK::RowVector tendon_force(_numMuscles);
            for (int i_musc = 0; i_musc < _numMuscles; ++i_musc) {
                const auto muscle =
                        _muscles[i_musc].convert_scalartype_double();
                const auto& musTenLen = _muscleTendonLengths(i_musc, i_time);
                const auto& normFibLen = states[2 * i_musc + 1];

                muscle.calcTendonForce(musTenLen, normFibLen,
                                       tendon_force[i_musc]);
            }
            sol.tendon_force.appendRow(time, tendon_force);
        }
        return sol;
    }
private:
    const MuscleRedundancySolver& _mrs;
    Model _model;
    const InverseMuscleSolverMotionData& _motionData;
    double _initialTime = SimTK::NaN;
    double _finalTime = SimTK::NaN;

    // Bookkeeping.
    int _numDOFs;
    int _numCoordActuators;
    int _numMuscles;
    std::vector<std::string> _muscleLabels;
    std::vector<std::string> _otherControlsLabels;
    // The index of the DOF that is actuated by each CoordinateActuator.
    std::vector<Eigen::Index> _coordActuatorDOFs;

    // Motion data to use during the optimization.
    Eigen::MatrixXd _desiredMoments;
    Eigen::MatrixXd _muscleTendonLengths;
    // TODO use Eigen::SparseMatrixXd
    std::vector<Eigen::MatrixXd> _momentArms;

    // CoordinateActuator optimal forces.
    Eigen::VectorXd _optimalForce;

    // De Groote muscles.
    std::vector<DeGroote2016Muscle<T>> _muscles;
};

MuscleRedundancySolver::MuscleRedundancySolver() {
    constructProperties();
}

MuscleRedundancySolver::MuscleRedundancySolver(
        const std::string& setupFilePath) : InverseMuscleSolver(setupFilePath) {
    constructProperties();
    updateFromXMLDocument();
}

void MuscleRedundancySolver::constructProperties() {
    constructProperty_initial_guess("static_optimization");
    constructProperty_zero_initial_activation(false);
}

MuscleRedundancySolver::Solution MuscleRedundancySolver::solve() {

    // TODO make sure experimental data is available for all unconstrained
    // coordinates; throw exception otherwise!

    // Load model and kinematics files.
    // --------------------------------
    Model origModel; // without reserve actuators.
    TimeSeriesTable kinematics;
    loadModelAndKinematicsData(origModel, kinematics);

    // Create reserve actuators.
    // -------------------------
    // TODO move to InverseMuscleSolver
    Model model(origModel);
    SimTK::State state = model.initSystem();
    if (get_create_reserve_actuators() != -1) {
        const auto& optimalForce = get_create_reserve_actuators();
        OPENSIM_THROW_IF(optimalForce <= 0, Exception,
            "Invalid value (" + std::to_string(optimalForce)
            + ") for create_reserve_actuators; should be -1 or positive.");

        std::cout << "Adding reserve actuators with an optimal force of "
                  << optimalForce << "..." << std::endl;

        std::vector<std::string> coordNames;
        // Borrowed from CoordinateActuator::CreateForceSetOfCoordinateAct...
        for (auto& coord : model.getCoordinatesInMultibodyTreeOrder()) {
            if (!coord->isConstrained(state)) {
                auto* actu = new CoordinateActuator();
                actu->setCoordinate(const_cast<Coordinate*>(coord.get()));
                auto path = coord->getAbsolutePathName();
                coordNames.push_back(path);
                // Get rid of model name.
                path = ComponentPath(path).formRelativePath(
                        model.getAbsolutePathName()).toString();
                // Get rid of slashes in the path; slashes not allowed in names.
                std::replace(path.begin(), path.end(), '/', '_');
                actu->setName("reserve_" + path);
                actu->setOptimalForce(optimalForce);
                model.addComponent(actu);
            }
        }
        // Re-make the system, since there are new actuators.
        model.initSystem();
        std::cout << "Added " << coordNames.size() << " reserve actuator(s), "
                "for each of the following coordinates:" << std::endl;
        for (const auto& name : coordNames) {
            std::cout << "    " << name << std::endl;
        }
    }

    // Process experimental data.
    // --------------------------
    // TODO move to InverseMuscleSolver
    OPENSIM_THROW_IF(get_lowpass_cutoff_frequency_for_joint_moments() <= 0 &&
            get_lowpass_cutoff_frequency_for_joint_moments() != -1, Exception,
                     "Invalid value for cutoff frequency for joint moments.");
    InverseMuscleSolverMotionData motionData(model, kinematics,
                          get_lowpass_cutoff_frequency_for_joint_moments());

    // Create the optimal control problem.
    // -----------------------------------
    auto ocp = std::make_shared<MRSProblemSeparate<adouble>>(*this,
                                                             model, motionData);

    // Solve for an initial guess with static optimization.
    // ----------------------------------------------------
    OPENSIM_THROW_IF(get_initial_guess() != "bounds" &&
                     get_initial_guess() != "static_optimization", Exception,
            "Invalid value (" + get_initial_guess() + " for "
            "initial_guess; should be 'static_optimization' or 'bounds'.");
    const int num_mesh_points = 100;
    mesh::OptimalControlIterate guess;
    if (get_initial_guess() == "static_optimization") {
        std::cout << std::string(79, '=') << std::endl;
        std::cout << "Computing an initial guess with static optimization."
                  << std::endl;
        std::cout << std::string(79, '-') << std::endl;
        GlobalStaticOptimizationSolver gso;
        gso.setModel(origModel);
        gso.setKinematicsData(kinematics);
        gso.set_lowpass_cutoff_frequency_for_joint_moments(
                get_lowpass_cutoff_frequency_for_joint_moments());
        gso.set_create_reserve_actuators(get_create_reserve_actuators());
        // Convert the static optimization solution into a guess.
        GlobalStaticOptimizationSolver::Solution gsoSolution = gso.solve();
        // gsoSolution.write("DEBUG_MuscleRedundancySolver_GSO_solution");
        MuscleRedundancySolver::Solution mrsGuess;
        mrsGuess.excitation = gsoSolution.activation;
        mrsGuess.activation = gsoSolution.activation;
        mrsGuess.norm_fiber_length = gsoSolution.norm_fiber_length;
        mrsGuess.norm_fiber_velocity = gsoSolution.norm_fiber_velocity;
        mrsGuess.other_controls = gsoSolution.other_controls;
        guess = ocp->construct_iterate(mrsGuess);
        // guess.write("DEBUG_MuscleRedundancySolver_guess.csv");
    } else {
        // This is the behavior if no guess is provided.
        std::cout << std::string(79, '=') << std::endl;
        std::cout << "Using variable bounds to form initial guess."
                  << std::endl;
        std::cout << std::string(79, '-') << std::endl;
    }

    // TODO perhaps solve without pennation initially? Pennation slows down
    // convergence.

    // Solve the optimal control problem.
    // ----------------------------------
    ocp->print_description();
    mesh::DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal", "ipopt",
                                                  num_mesh_points);
    std::cout << std::string(79, '=') << std::endl;
    std::cout << "Running the Muscle Redundancy Solver." << std::endl;
    std::cout << std::string(79, '-') << std::endl;
    mesh::OptimalControlSolution ocp_solution;
    if (get_initial_guess() == "static_optimization") {
        ocp_solution = dircol.solve(guess);
    } else {
        ocp_solution = dircol.solve();
    }

    // Return the solution.
    // --------------------
    // TODO remove:
    ocp_solution.write("MuscleRedundancySolver_OCP_solution.csv");
    // dircol.print_constraint_values(ocp_solution);
    return ocp->deconstruct_iterate(ocp_solution);
}
