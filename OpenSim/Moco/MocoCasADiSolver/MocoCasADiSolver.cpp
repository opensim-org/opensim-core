/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoCasADiSolver.cpp                                         *
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

#include "MocoCasADiSolver.h"

#include <OpenSim/Common/Assertion.h>
#include <OpenSim/Moco/MocoUtilities.h>

#ifdef OPENSIM_WITH_CASADI
    #include "CasOCSolver.h"
    #include "MocoCasOCProblem.h"
    #include <casadi/casadi.hpp>

    #include <OpenSim/Common/Stopwatch.h>

    using casadi::Callback;
    using casadi::Dict;
    using casadi::DM;
    using casadi::MX;
    using casadi::Slice;
    using casadi::Sparsity;
#endif

using namespace OpenSim;

MocoCasADiSolver::MocoCasADiSolver() { constructProperties(); }

void MocoCasADiSolver::constructProperties() {
    constructProperty_scale_variables_using_bounds(false);
    constructProperty_parameters_require_initsystem(true);
    constructProperty_optim_sparsity_detection("none");
    constructProperty_optim_write_sparsity("");
    constructProperty_optim_finite_difference_scheme("central");
    constructProperty_parallel();
    constructProperty_output_interval(0);

    constructProperty_minimize_implicit_multibody_accelerations(false);
    constructProperty_implicit_multibody_accelerations_weight(1.0);
    constructProperty_minimize_implicit_auxiliary_derivatives(false);
    constructProperty_implicit_auxiliary_derivatives_weight(1.0);

    constructProperty_enforce_path_constraint_mesh_interior_points(false);
    constructProperty_minimize_state_projection_distance(true);
    constructProperty_state_projection_distance_weight(1e-6);
    constructProperty_projection_slack_variable_bounds({-1e-1, 1e-1});
}

bool MocoCasADiSolver::isAvailable() {
#ifdef OPENSIM_WITH_CASADI
    return true;
#else
    return false;
#endif
}

MocoTrajectory MocoCasADiSolver::createGuess(const std::string& type) const {
#ifdef OPENSIM_WITH_CASADI
    OPENSIM_THROW_IF_FRMOBJ(
            type != "bounds" && type != "random" && type != "time-stepping",
            Exception,
            "Unexpected guess type '" + type +
                    "'; supported types are 'bounds', 'random', and "
                    "'time-stepping'.");

    if (type == "time-stepping") { return createGuessTimeStepping(); }

    auto casProblem = createCasOCProblem();
    auto casSolver = createCasOCSolver(*casProblem);

    std::vector<int> inputControlIndexes = 
            getProblemRep().getInputControlIndexes();
    if (type == "bounds") {
        return convertToMocoTrajectory(
                casSolver->createInitialGuessFromBounds(), 
                inputControlIndexes);
    } else if (type == "random") {
        return convertToMocoTrajectory(
                casSolver->createRandomIterateWithinBounds(),
                inputControlIndexes);
    } else {
        OPENSIM_THROW(Exception, "Internal error.");
    }
#else
    OPENSIM_THROW(MocoCasADiSolverNotAvailable);
#endif
}

void MocoCasADiSolver::setGuess(MocoTrajectory guess) {
    // Ensure the guess is compatible with this solver/problem.
    checkGuess(guess);
    clearGuess();
    m_guessFromAPI = std::move(guess);
}
void MocoCasADiSolver::setGuessFile(const std::string& file) {
    clearGuess();
    set_guess_file(file);
}

void MocoCasADiSolver::checkGuess(const MocoTrajectory& guess) const {
    OPENSIM_THROW_IF(get_multibody_dynamics_mode() == "implicit" &&
            guess.hasCoordinateStates() &&
            guess.getDerivativeNames().empty(),
            Exception,
            "'multibody_dynamics_mode' set to 'implicit' and coordinate states "
            "exist in the guess, but no coordinate accelerations were found in "
            "the guess. Consider using "
            "MocoTrajectory::generateAccelerationsFromValues() or "
            "MocoTrajectory::generateAccelerationsFromSpeeds() to construct an "
            "appropriate guess.");
    guess.isCompatible(
            getProblemRep(), get_multibody_dynamics_mode() == "implicit", true);
}

void MocoCasADiSolver::clearGuess() {
    m_guessFromAPI = MocoTrajectory();
    m_guessFromFile = MocoTrajectory();
    set_guess_file("");
    m_guessToUse.reset();
}
const MocoTrajectory& MocoCasADiSolver::getGuess() const {
    if (!m_guessToUse) {
        if (get_guess_file() != "" && m_guessFromFile.empty()) {
            // The API should make it impossible for both guessFromFile and
            // guessFromAPI to be non-empty.
            OPENSIM_ASSERT_FRMOBJ(m_guessFromAPI.empty());
            // No need to load from file again if we've already loaded it.
            MocoTrajectory guessFromFile(get_guess_file());
            checkGuess(guessFromFile);
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

std::unique_ptr<MocoCasOCProblem> MocoCasADiSolver::createCasOCProblem() const {
#ifdef OPENSIM_WITH_CASADI
    const auto& problemRep = getProblemRep();
    int parallel = 1;
    int parallelEV = getMocoParallelEnvironmentVariable();
    if (getProperty_parallel().size()) {
        parallel = get_parallel();
    } else if (parallelEV != -1) {
        parallel = parallelEV;
    }
    int numThreads;
    if (parallel == 0) {
        numThreads = 1;
    } else if (parallel == 1) {
        numThreads = std::thread::hardware_concurrency();
    } else {
        numThreads = parallel;
    }

    checkPropertyValueIsInSet(
            getProperty_multibody_dynamics_mode(), {"explicit", "implicit"});
    if (problemRep.isPrescribedKinematics()) {
        OPENSIM_THROW_IF(get_multibody_dynamics_mode() != "implicit", Exception,
                "Prescribed kinematics (PositionMotion) requires implicit "
                "dynamics mode.");
    }

    const auto& model = problemRep.getModelBase();
    OPENSIM_THROW_IF(!model.getMatterSubsystem().getUseEulerAngles(
                             model.getWorkingState()),
            Exception, "Quaternions are not supported.");

    if (getProblemRep().getNumKinematicConstraintEquations()) {
        checkPropertyValueIsInSet(getProperty_kinematic_constraint_method(),
                                  {"Posa2016", "Bordalba2023"});
        OPENSIM_THROW_IF(get_transcription_scheme() != "hermite-simpson" &&
                     get_kinematic_constraint_method() == "Posa2016", Exception,
            "Expected the 'hermite-simpson' transcription scheme when using "
            "the Posa et al. 2016 method for enforcing kinematic constraints, "
            "but received '{}'.", get_transcription_scheme());
    }

    return OpenSim::make_unique<MocoCasOCProblem>(*this, problemRep,
            createProblemRepJar(numThreads), get_multibody_dynamics_mode(),
            get_kinematic_constraint_method());
#else
    OPENSIM_THROW(MocoCasADiSolverNotAvailable);
#endif
}

std::unique_ptr<CasOC::Solver> MocoCasADiSolver::createCasOCSolver(
        const MocoCasOCProblem& casProblem) const {
#ifdef OPENSIM_WITH_CASADI
    auto casSolver = OpenSim::make_unique<CasOC::Solver>(casProblem);

    // Set solver options.
    // -------------------
    Dict solverOptions;
    checkPropertyValueIsInSet(getProperty_optim_solver(), 
            {"ipopt", "snopt", "fatrop"});
    checkPropertyValueIsInSet(getProperty_transcription_scheme(),
            {"trapezoidal", "hermite-simpson", "legendre-gauss-1",
             "legendre-gauss-2", "legendre-gauss-3", "legendre-gauss-4",
             "legendre-gauss-5", "legendre-gauss-6", "legendre-gauss-7",
             "legendre-gauss-8", "legendre-gauss-9", "legendre-gauss-radau-1",
             "legendre-gauss-radau-2", "legendre-gauss-radau-3",
             "legendre-gauss-radau-4", "legendre-gauss-radau-5",
             "legendre-gauss-radau-6", "legendre-gauss-radau-7",
             "legendre-gauss-radau-8", "legendre-gauss-radau-9"});

    checkPropertyValueIsInRangeOrSet(getProperty_num_mesh_intervals(), 0,
            std::numeric_limits<int>::max(), {});

    if (getProperty_mesh().size() > 0) {

        OPENSIM_THROW_IF_FRMOBJ((get_mesh(0) != 0), Exception,
                "Invalid custom mesh; first mesh "
                "point must be zero.");

        for (int i = 1; i < (int)this->getProperty_mesh().size(); ++i) {

            OPENSIM_THROW_IF_FRMOBJ((get_mesh(i) <= get_mesh(i - 1)), Exception,
                    "Invalid custom mesh; mesh "
                    "points must be strictly increasing.");
        }

        OPENSIM_THROW_IF_FRMOBJ((get_mesh(getProperty_mesh().size() - 1) != 1),
                Exception,
                "Invalid custom mesh; last mesh "
                "point must be one.");
    }

    checkPropertyValueIsInRangeOrSet(getProperty_optim_max_iterations(), 0,
            std::numeric_limits<int>::max(), {-1});
    checkPropertyValueIsInRangeOrSet(getProperty_optim_convergence_tolerance(),
            0.0, SimTK::NTraits<double>::getInfinity(), {-1.0});
    checkPropertyValueIsInRangeOrSet(getProperty_optim_constraint_tolerance(),
            0.0, SimTK::NTraits<double>::getInfinity(), {-1.0});
    checkPropertyValueIsInSet(getProperty_verbosity(), {0, 1, 2});
    if (get_optim_solver() == "ipopt") {
        solverOptions["print_user_options"] = "yes";
        if (get_verbosity() < 2) {
            solverOptions["print_level"] = 0;
        } else if (get_optim_ipopt_print_level() != -1) {
            solverOptions["print_level"] = get_optim_ipopt_print_level();
        }
        solverOptions["hessian_approximation"] =
                get_optim_hessian_approximation();

        if (get_optim_max_iterations() != -1)
            solverOptions["max_iter"] = get_optim_max_iterations();

        if (get_optim_convergence_tolerance() != -1) {
            const auto& tol = get_optim_convergence_tolerance();
            // This is based on what Simbody does.
            solverOptions["tol"] = tol;
            solverOptions["dual_inf_tol"] = tol;
            solverOptions["compl_inf_tol"] = tol;
            solverOptions["acceptable_tol"] = tol;
            solverOptions["acceptable_dual_inf_tol"] = tol;
            solverOptions["acceptable_compl_inf_tol"] = tol;
        }
        if (get_optim_constraint_tolerance() != -1) {
            const auto& tol = get_optim_constraint_tolerance();
            solverOptions["constr_viol_tol"] = tol;
            solverOptions["acceptable_constr_viol_tol"] = tol;
        }
    }

    checkPropertyValueIsInSet(getProperty_optim_sparsity_detection(),
            {"none", "random", "initial-guess"});
    casSolver->setSparsityDetection(get_optim_sparsity_detection());
    casSolver->setSparsityDetectionRandomCount(3);

    casSolver->setWriteSparsity(get_optim_write_sparsity());

    checkPropertyValueIsInSet(getProperty_optim_finite_difference_scheme(),
            {"central", "forward", "backward"});
    casSolver->setFiniteDifferenceScheme(get_optim_finite_difference_scheme());

    casSolver->setCallbackInterval(get_output_interval());

    Dict pluginOptions;
    pluginOptions["verbose_init"] = true;

    if (getProperty_mesh().empty()) {
        casSolver->setNumMeshIntervals(get_num_mesh_intervals());
    } else {
        std::vector<double> mesh;
        for (int i = 0; i < getProperty_mesh().size(); ++i) {
            mesh.push_back(get_mesh(i));
        }
        casSolver->setMesh(mesh);
    }
    casSolver->setTranscriptionScheme(get_transcription_scheme());
    casSolver->setScaleVariablesUsingBounds(get_scale_variables_using_bounds());
    casSolver->setMinimizeLagrangeMultipliers(
            get_minimize_lagrange_multipliers());
    casSolver->setLagrangeMultiplierWeight(get_lagrange_multiplier_weight());

    casSolver->setImplicitMultibodyAccelerationBounds(
            convertBounds(get_implicit_multibody_acceleration_bounds()));
    casSolver->setMinimizeImplicitMultibodyAccelerations(
            get_minimize_implicit_multibody_accelerations());
    OPENSIM_THROW_IF(get_implicit_multibody_accelerations_weight() < 0,
            Exception,
            "Property implicit_multibody_accelerations_weight must be "
            "non-negative, but it is set to {}.",
            get_implicit_multibody_accelerations_weight());
    casSolver->setImplicitMultibodyAccelerationsWeight(
            get_implicit_multibody_accelerations_weight());

    casSolver->setImplicitAuxiliaryDerivativeBounds(
            convertBounds(get_implicit_auxiliary_derivative_bounds()));
    casSolver->setMinimizeImplicitAuxiliaryDerivatives(
            get_minimize_implicit_auxiliary_derivatives());
    OPENSIM_THROW_IF(get_implicit_auxiliary_derivatives_weight() < 0, Exception,
            "Property implicit_auxiliary_derivatives_weight must be "
            "non-negative, but it is set to {}.",
            get_implicit_auxiliary_derivatives_weight());
    casSolver->setImplicitAuxiliaryDerivativesWeight(
            get_implicit_auxiliary_derivatives_weight());

    casSolver->setMinimizeStateProjection(
            get_minimize_state_projection_distance());
    casSolver->setStateProjectionWeight(get_state_projection_distance_weight());

    casSolver->setOptimSolver(get_optim_solver());
    casSolver->setInterpolateControlMeshInteriorPoints(
            get_interpolate_control_mesh_interior_points());
    casSolver->setEnforcePathConstraintMeshInteriorPoints(
            get_enforce_path_constraint_mesh_interior_points());
    if (casProblem.getJarSize() > 1) {
        casSolver->setParallelism("thread", casProblem.getJarSize());
    }
    casSolver->setPluginOptions(pluginOptions);
    casSolver->setSolverOptions(solverOptions);
    return casSolver;
#else
    OPENSIM_THROW(MocoCasADiSolverNotAvailable);
#endif
}

MocoSolution MocoCasADiSolver::solveImpl() const {
#ifdef OPENSIM_WITH_CASADI
    const Stopwatch stopwatch;

    if (get_verbosity()) {
        log_info(std::string(72, '='));
        log_info("MocoCasADiSolver starting.");
        log_info(getFormattedDateTime(false, "%c"));
        log_info(std::string(72, '-'));
        getProblemRep().printDescription();
    }
    auto casProblem = createCasOCProblem();
    auto casSolver = createCasOCSolver(*casProblem);
    if (get_verbosity()) {
        log_info("Number of threads: {}", casProblem->getJarSize());
    }

    std::vector<int> inputControlIndexes = 
            getProblemRep().getInputControlIndexes();
    MocoTrajectory guess = getGuess();
    CasOC::Iterate casGuess;
    // We do not need to append projection states here since they will be 
    // appended later when the guess is resampled by the solver (if needed).
    bool appendProjectionStates = false;
    if (guess.empty()) {
        casGuess = casSolver->createInitialGuessFromBounds();
    } else {
        std::vector<std::string> expectedSlackNames;
        for (const auto& info : casProblem->getSlackInfos()) {
            expectedSlackNames.push_back(info.name);
        }
        casGuess = convertToCasOCIterate(guess, expectedSlackNames, 
                appendProjectionStates, inputControlIndexes);
    }

    // Temporarily disable printing of negative muscle force warnings so the
    // log isn't flooded while computing finite differences.
    Logger::Level origLoggerLevel = Logger::getLevel();
    Logger::setLevel(Logger::Level::Warn);
    CasOC::Solution casSolution;
    // try {
    casSolution = casSolver->solve(casGuess);
    // } catch (...) {
    //     OpenSim::Logger::setLevel(origLoggerLevel);
    // }
    OpenSim::Logger::setLevel(origLoggerLevel);

    MocoSolution mocoSolution = convertToMocoTrajectory<MocoSolution>(
            casSolution, inputControlIndexes);

    // If enforcing model constraints and not minimizing Lagrange multipliers,
    // check the rank of the constraint Jacobian and if rank-deficient, print
    // recommendation to the user to enable Lagrange multiplier minimization.
    if (getProblemRep().getNumKinematicConstraintEquations() &&
            !get_enforce_constraint_derivatives() &&
            !get_minimize_lagrange_multipliers()) {
        checkConstraintJacobianRank(mocoSolution);
    }

    const long long elapsed = stopwatch.getElapsedTimeInNs();
    setSolutionStats(mocoSolution, casSolution.stats.at("success"),
            casSolution.objective, casSolution.stats.at("return_status"),
            casSolution.stats.at("iter_count"), SimTK::nsToSec(elapsed),
            casSolution.objective_breakdown);

    if (get_verbosity()) {
        log_info(std::string(72, '-'));
        log_info("Elapsed real time: {}.", stopwatch.formatNs(elapsed));
        log_info(getFormattedDateTime(false, "%c"));
        if (mocoSolution) {
            log_info("MocoCasADiSolver succeeded!");
        } else {
            log_warn("MocoCasADiSolver did NOT succeed:");
            log_warn("  {}", mocoSolution.getStatus());
        }
        log_info(std::string(72, '='));
    }
    return mocoSolution;
#else
    OPENSIM_THROW(MocoCasADiSolverNotAvailable);
#endif
}
