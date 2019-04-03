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
    constructProperty_optim_sparsity_detection("none");
    constructProperty_optim_write_sparsity("");
    constructProperty_optim_finite_difference_scheme("central");
    constructProperty_parallel();
}

MocoIterate MocoCasADiSolver::createGuess(const std::string& type) const {
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
        return convertToMocoIterate(casSolver->createInitialGuessFromBounds());
    } else if (type == "random") {
        return convertToMocoIterate(
                casSolver->createRandomIterateWithinBounds());
    } else {
        OPENSIM_THROW(Exception, "Internal error.");
    }
}

void MocoCasADiSolver::setGuess(MocoIterate guess) {
    // Ensure the guess is compatible with this solver/problem.
    guess.isCompatible(getProblemRep(), true);
    clearGuess();
    m_guessFromAPI = std::move(guess);
}
void MocoCasADiSolver::setGuessFile(const std::string& file) {
    clearGuess();
    set_guess_file(file);
}
void MocoCasADiSolver::clearGuess() {
    m_guessFromAPI = MocoIterate();
    m_guessFromFile = MocoIterate();
    set_guess_file("");
    m_guessToUse.reset();
}
const MocoIterate& MocoCasADiSolver::getGuess() const {
    if (!m_guessToUse) {
        if (get_guess_file() != "" && m_guessFromFile.empty()) {
            // The API should make it impossible for both guessFromFile and
            // guessFromAPI to be non-empty.
            assert(m_guessFromAPI.empty());
            // No need to load from file again if we've already loaded it.
            MocoIterate guessFromFile(get_guess_file());
            guessFromFile.isCompatible(getProblemRep(), true);
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
    // Not an issue with Python 3?
    // if (m_runningInPython && parallel) {
    //     std::cout << "Warning: "
    //                  "Cannot use parallelism in Python due to its "
    //                  "Global Interpreter Lock. "
    //                  "Set the environment variable OPENSIM_MOCO_PARALLEL or "
    //                  "MocoCasADiSolver's 'parallel' property to 0, "
    //                  "or use the command-line or Matlab interfaces."
    //               << std::endl;
    // }
    int numThreads;
    if (parallel == 0) {
        numThreads = 1;
    } else if (parallel == 1) {
        numThreads = std::thread::hardware_concurrency();
    } else {
        numThreads = parallel;
    }

    checkPropertyInSet(
            *this, getProperty_dynamics_mode(), {"explicit", "implicit"});
    if (problemRep.isPrescribedKinematics()) {
        OPENSIM_THROW_IF(get_dynamics_mode() != "implicit", Exception,
                "Prescribed kinematics (PositionMotion) requires implicit "
                "dynamics mode.");
    }

    const auto& model = problemRep.getModelBase();
    OPENSIM_THROW_IF(!model.getMatterSubsystem().getUseEulerAngles(
            model.getWorkingState()),
            Exception, "Quaternions are not supported.");
    return OpenSim::make_unique<MocoCasOCProblem>(*this, problemRep,
            createProblemRepJar(numThreads),
            get_dynamics_mode());
}

std::unique_ptr<CasOC::Solver> MocoCasADiSolver::createCasOCSolver(
        const MocoCasOCProblem& casProblem) const {
    auto casSolver = make_unique<CasOC::Solver>(casProblem);

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
    if (!getProperty_enforce_constraint_derivatives().empty()) {
        OPENSIM_THROW_IF(get_transcription_scheme() != "hermite-simpson" &&
                                 get_enforce_constraint_derivatives(),
                Exception,
                format("If enforcing derivatives of model kinematic "
                       "constraints, then the property 'transcription_scheme' "
                       "must be set to 'hermite-simpson'. "
                       "Currently, it is set to '%s'.",
                        get_transcription_scheme()));
    }

    checkPropertyIsPositive(*this, getProperty_num_mesh_points());
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

    Dict pluginOptions;
    pluginOptions["verbose_init"] = true;

    casSolver->setNumMeshPoints(get_num_mesh_points());
    casSolver->setTranscriptionScheme(get_transcription_scheme());
    casSolver->setMinimizeLagrangeMultipliers(
            get_minimize_lagrange_multipliers());
    casSolver->setLagrangeMultiplierWeight(get_lagrange_multiplier_weight());
    casSolver->setOptimSolver(get_optim_solver());
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
        std::cout << "MocoCasADiSolver starting.\n";
        std::cout << std::string(79, '-') << std::endl;
        getProblemRep().printDescription();
    }
    auto casProblem = createCasOCProblem();
    auto casSolver = createCasOCSolver(*casProblem);
    if (get_verbosity()) {
        std::cout << "Number of threads: " << casProblem->getJarSize()
                  << std::endl;
    }

    MocoIterate guess = getGuess();
    CasOC::Iterate casGuess;
    if (guess.empty()) {
        casGuess = casSolver->createInitialGuessFromBounds();
    } else {
        casGuess = convertToCasOCIterate(*m_guessToUse);
    }
    CasOC::Solution casSolution = casSolver->solve(casGuess);
    MocoSolution mocoSolution = convertToMocoIterate<MocoSolution>(casSolution);
    const long long elapsed = stopwatch.getElapsedTimeInNs();
    setSolutionStats(mocoSolution, casSolution.stats.at("success"),
            casSolution.objective,
            casSolution.stats.at("return_status"),
            casSolution.stats.at("iter_count"),
            SimTK::nsToSec(elapsed));

    if (get_verbosity()) {
        std::cout << std::string(79, '-') << "\n";
        std::cout << "Elapsed real time: "
                << stopwatch.formatNs(elapsed) << ".\n";
        if (mocoSolution) {
            std::cout << "MocoCasADiSolver succeeded!\n";
        } else {
            std::cerr << "MocoCasADiSolver did NOT succeed:\n";
            std::cerr << "  " << mocoSolution.getStatus() << "\n";
        }
        std::cout << std::string(79, '=') << std::endl;
    }
    return mocoSolution;
}
