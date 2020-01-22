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

#include "../MocoUtilities.h"
#include "CasOCSolver.h"
#include "MocoCasOCProblem.h"
#include <casadi/casadi.hpp>

using casadi::Callback;
using casadi::Dict;
using casadi::DM;
using casadi::MX;
using casadi::Slice;
using casadi::Sparsity;

using namespace OpenSim;

MocoCasADiSolver::MocoCasADiSolver() { constructProperties(); }

void MocoCasADiSolver::constructProperties() {
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
}

MocoTrajectory MocoCasADiSolver::createGuess(const std::string& type) const {
    OPENSIM_THROW_IF_FRMOBJ(
            type != "bounds" && type != "random" && type != "time-stepping",
            Exception,
            "Unexpected guess type '" + type +
                    "'; supported types are 'bounds', 'random', and "
                    "'time-stepping'.");

    if (type == "time-stepping") { return createGuessTimeStepping(); }

    auto casProblem = createCasOCProblem();
    auto casSolver = createCasOCSolver(*casProblem);

    if (type == "bounds") {
        return convertToMocoTrajectory(
                casSolver->createInitialGuessFromBounds());
    } else if (type == "random") {
        return convertToMocoTrajectory(
                casSolver->createRandomIterateWithinBounds());
    } else {
        OPENSIM_THROW(Exception, "Internal error.");
    }
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
            assert(m_guessFromAPI.empty());
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
    const auto& problemRep = getProblemRep();
    int parallel = 1;
    int parallelEV = getMocoParallelEnvironmentVariable();
    if (getProperty_parallel().size()) {
        parallel = get_parallel();
    } else if (parallelEV != -1) {
        parallel = parallelEV;
    }
    // if (m_runningInPython && parallel) {
    //    std::cout << "Warning: "
    //                 "Cannot use parallelism in Python due to its "
    //                 "Global Interpreter Lock. "
    //                 "Set the environment variable OPENSIM_MOCO_PARALLEL or "
    //                 "MocoCasADiSolver's 'parallel' property to 0, "
    //                 "or use the command-line or Matlab interfaces."
    //              << std::endl;
    //}
    int numThreads;
    if (parallel == 0) {
        numThreads = 1;
    } else if (parallel == 1) {
        numThreads = std::thread::hardware_concurrency();
    } else {
        numThreads = parallel;
    }

    checkPropertyInSet(
            *this, getProperty_multibody_dynamics_mode(), {"explicit", "implicit"});
    if (problemRep.isPrescribedKinematics()) {
        OPENSIM_THROW_IF(get_multibody_dynamics_mode() != "implicit", Exception,
                "Prescribed kinematics (PositionMotion) requires implicit "
                "dynamics mode.");
    }

    const auto& model = problemRep.getModelBase();
    OPENSIM_THROW_IF(!model.getMatterSubsystem().getUseEulerAngles(
                             model.getWorkingState()),
            Exception, "Quaternions are not supported.");
    return OpenSim::make_unique<MocoCasOCProblem>(*this, problemRep,
            createProblemRepJar(numThreads), get_multibody_dynamics_mode());
}

std::unique_ptr<CasOC::Solver> MocoCasADiSolver::createCasOCSolver(
        const MocoCasOCProblem& casProblem) const {
    auto casSolver = OpenSim::make_unique<CasOC::Solver>(casProblem);

    // Set solver options.
    // -------------------
    Dict solverOptions;
    checkPropertyInSet(*this, getProperty_optim_solver(), {"ipopt", "snopt"});
    checkPropertyInSet(*this, getProperty_transcription_scheme(),
            {"trapezoidal", "hermite-simpson"});
    OPENSIM_THROW_IF(casProblem.getNumKinematicConstraintEquations() != 0 &&
                             get_transcription_scheme() == "trapezoidal",
            OpenSim::Exception,
            "Kinematic constraints not supported with "
            "trapezoidal transcription.");
    // Enforcing constraint derivatives is only supported when Hermite-Simpson
    // is set as the transcription scheme.
    if (casProblem.getNumKinematicConstraintEquations() != 0) {
        OPENSIM_THROW_IF(get_transcription_scheme() != "hermite-simpson" &&
                                 get_enforce_constraint_derivatives(),
                Exception,
                format("If enforcing derivatives of model kinematic "
                       "constraints, then the property 'transcription_scheme' "
                       "must be set to 'hermite-simpson'. "
                       "Currently, it is set to '%s'.",
                        get_transcription_scheme()));
    }

    checkPropertyInRangeOrSet(*this, getProperty_num_mesh_intervals(), 0,
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

    checkPropertyInRangeOrSet(*this, getProperty_optim_max_iterations(), 0,
            std::numeric_limits<int>::max(), {-1});
    checkPropertyInRangeOrSet(*this, getProperty_optim_convergence_tolerance(),
            0.0, SimTK::NTraits<double>::getInfinity(), {-1.0});
    checkPropertyInRangeOrSet(*this, getProperty_optim_constraint_tolerance(),
            0.0, SimTK::NTraits<double>::getInfinity(), {-1.0});
    checkPropertyInSet(*this, getProperty_verbosity(), {0, 1, 2});
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

    checkPropertyInSet(*this, getProperty_optim_sparsity_detection(),
            {"none", "random", "initial-guess"});
    casSolver->setSparsityDetection(get_optim_sparsity_detection());
    casSolver->setSparsityDetectionRandomCount(3);

    casSolver->setWriteSparsity(get_optim_write_sparsity());

    checkPropertyInSet(*this, getProperty_optim_finite_difference_scheme(),
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
    casSolver->setMinimizeLagrangeMultipliers(
            get_minimize_lagrange_multipliers());
    casSolver->setLagrangeMultiplierWeight(get_lagrange_multiplier_weight());

    casSolver->setImplicitMultibodyAccelerationBounds(
            convertBounds(get_implicit_multibody_acceleration_bounds()));
    casSolver->setMinimizeImplicitMultibodyAccelerations(
            get_minimize_implicit_multibody_accelerations());
    OPENSIM_THROW_IF(get_implicit_multibody_accelerations_weight() < 0,
            Exception, format(
                "Property implicit_multibody_accelerations_weight must be "
                "non-negative, but it is set to %f.",
                get_implicit_multibody_accelerations_weight()));
    casSolver->setImplicitMultibodyAccelerationsWeight(
            get_implicit_multibody_accelerations_weight());

    casSolver->setImplicitAuxiliaryDerivativeBounds(
            convertBounds(get_implicit_auxiliary_derivative_bounds()));
    casSolver->setMinimizeImplicitAuxiliaryDerivatives(
            get_minimize_implicit_auxiliary_derivatives());
    OPENSIM_THROW_IF(get_implicit_auxiliary_derivatives_weight() < 0,
            Exception, format(
                    "Property implicit_auxiliary_derivatives_weight must be "
                    "non-negative, but it is set to %f.",
                    get_implicit_auxiliary_derivatives_weight()));
    casSolver->setImplicitAuxiliaryDerivativesWeight(
            get_implicit_auxiliary_derivatives_weight());

    casSolver->setOptimSolver(get_optim_solver());
    casSolver->setInterpolateControlMidpoints(
            get_interpolate_control_midpoints());
    if (casProblem.getJarSize() > 1) {
        casSolver->setParallelism("thread", casProblem.getJarSize());
    }
    casSolver->setPluginOptions(pluginOptions);
    casSolver->setSolverOptions(solverOptions);
    return casSolver;
}

MocoSolution MocoCasADiSolver::solveImpl() const {
    const Stopwatch stopwatch;

    if (get_verbosity()) {
        std::cout << std::string(79, '=') << "\n";
        std::cout << "MocoCasADiSolver starting. ";
        std::cout << getMocoFormattedDateTime(false, "%c") << "\n";
        std::cout << std::string(79, '-') << std::endl;
        getProblemRep().printDescription();
    }
    auto casProblem = createCasOCProblem();
    auto casSolver = createCasOCSolver(*casProblem);
    if (get_verbosity()) {
        std::cout << "Number of threads: " << casProblem->getJarSize()
                  << std::endl;
    }

    MocoTrajectory guess = getGuess();
    CasOC::Iterate casGuess;
    if (guess.empty()) {
        casGuess = casSolver->createInitialGuessFromBounds();
    } else {
        casGuess = convertToCasOCIterate(guess);
    }
    CasOC::Solution casSolution = casSolver->solve(casGuess);
    MocoSolution mocoSolution =
            convertToMocoTrajectory<MocoSolution>(casSolution);

    // If enforcing model constraints and not minimizing Lagrange multipliers,
    // check the rank of the constraint Jacobian and if rank-deficient, print
    // recommendation to the user to enable Lagrange multiplier minimization.
    if (getProblemRep().getNumKinematicConstraintEquations() &&
            !get_enforce_constraint_derivatives() &&
            !get_minimize_lagrange_multipliers()) {
        const auto& model = getProblemRep().getModelBase();
        const auto& matter = model.getMatterSubsystem();
        Storage storage = mocoSolution.exportToStatesStorage();
        // TODO update when we support multiple phases.
        auto statesTraj =
                StatesTrajectory::createFromStatesStorage(model, storage);
        SimTK::Matrix G;
        SimTK::FactorQTZ G_qtz;
        bool isJacobianFullRank = true;
        int rank;
        for (const auto& s : statesTraj) {
            // Jacobian is at most velocity-dependent.
            model.realizeVelocity(s);
            matter.calcG(s, G);
            G_qtz.factor<double>(G);
            if (G_qtz.getRank() < G.nrow()) {
                isJacobianFullRank = false;
                rank = G_qtz.getRank();
                break;
            }
        }

        if (!isJacobianFullRank) {
            std::cout << std::endl;
            std::cout << "---------------------------------------------------"
                      << "--\n";
            std::cout << "WARNING: rank-deficient constraint Jacobian "
                      << "detected.\n";
            std::cout << "---------------------------------------------------"
                      << "--\n";
            std::cout << "The model constraint Jacobian has "
                      << std::to_string(G.nrow()) + " row(s) but is only rank "
                      << std::to_string(rank) + ".\nTry removing "
                      << "redundant constraints from the model or enable \n"
                      << "minimization of Lagrange multipliers by utilizing "
                      << "the solver \nproperties "
                      << "'minimize_lagrange_multipliers' and \n"
                      << "'lagrange_multiplier_weight'.\n";
            std::cout << "---------------------------------------------------"
                      << "--\n\n";
        }
    }

    const long long elapsed = stopwatch.getElapsedTimeInNs();
    setSolutionStats(mocoSolution, casSolution.stats.at("success"),
            casSolution.objective, casSolution.stats.at("return_status"),
            casSolution.stats.at("iter_count"), SimTK::nsToSec(elapsed),
            casSolution.objective_breakdown);

    if (get_verbosity()) {
        std::cout << std::string(79, '-') << "\n";
        std::cout << "Elapsed real time: " << stopwatch.formatNs(elapsed)
                  << ".\n";
        std::cout << getMocoFormattedDateTime(false, "%c") << "\n";
        if (mocoSolution) {
            std::cout << "MocoCasADiSolver succeeded!\n";
        } else {
            std::cout << "MocoCasADiSolver did NOT succeed:\n";
            std::cout << "  " << mocoSolution.getStatus() << "\n";
        }
        std::cout << std::string(79, '=') << std::endl;
    }
    return mocoSolution;
}
