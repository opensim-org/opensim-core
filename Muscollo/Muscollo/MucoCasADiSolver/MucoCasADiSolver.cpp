/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoCasADiSolver.cpp                                     *
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

#include "MucoCasADiSolver.h"
#include "../MuscolloUtilities.h"
#include "CasADiTrapezoidal.h"

#include <casadi/casadi.hpp>

// TODO
// - create separate tests for tropter and CasADi.
// - parameters are very inefficient: reapplying parameters more than necessary.
// - how to handle avoiding interpolation of splines? mesh index?
// - variable allocation order MATTERS: try allocating variables in a
//   more efficient manner, or create views again.
// - does CasADi rearrange to improve sparsity? does order of variables
//   (and sparsity pattern) matter for performance? ....almost no time is spent
//   in the solver, so this can't matter yet.
// - Remove time as a variable to greatly reduce coupling, or just, for now,
//   do not use time when computing cost and constraints.
//   what is the performance benefit of removing time?
// - improve sparsity pattern?
// - Expected much better performance, b/c I thought we would not need to evaluate
//   cost at time i when perturbing at time j, i \neq j..
//   I can try to investigate this on my own.
// - Is there a way to collect stats on how many times a function is called?
// - IntegrandCost: no point in realizing to velocity if there aren't even
//   any integral cost terms, or if integral costs depend only on controls
//   (not any more complex calculations).
// - support multibody constraints
// - get testConstraints working with CasADi.

using casadi::MX;
using casadi::DM;
using casadi::Sparsity;
using casadi::Slice;
using casadi::Callback;
using casadi::Dict;

using namespace OpenSim;

MucoCasADiSolver::MucoCasADiSolver() {
    constructProperties();
}

void MucoCasADiSolver::constructProperties() {
    constructProperty_num_mesh_points(100);
    constructProperty_verbosity(2);
    constructProperty_dynamics_mode("explicit");
    constructProperty_optim_solver("ipopt");
    constructProperty_optim_max_iterations(-1);
    constructProperty_optim_convergence_tolerance(-1);
    constructProperty_optim_constraint_tolerance(-1);
    constructProperty_optim_hessian_approximation("limited-memory");
    constructProperty_optim_ipopt_print_level(-1);
    constructProperty_guess_file("");
}

MucoIterate MucoCasADiSolver::createGuess(const std::string& type) const {
    OPENSIM_THROW_IF_FRMOBJ(
            type != "bounds"
                    && type != "random"
                    && type != "time-stepping",
            Exception,
            "Unexpected guess type '" + type +
                    "'; supported types are 'bounds', 'random', and "
                    "'time-stepping'.");

    if (type == "time-stepping") {
        return createGuessTimeStepping();
    }

    auto transcription = createTranscription();

    if (type == "bounds") {
        return transcription->createInitialGuessFromBounds();
    } else if (type == "random") {
        return transcription->createRandomIterateWithinBounds();
    } else {
        OPENSIM_THROW(Exception, "Internal error.");
    }
}

void MucoCasADiSolver::setGuess(MucoIterate guess) {
    // Ensure the guess is compatible with this solver/problem.
    // Make sure to initialize the problem. TODO put in a better place.
    // TODO createTropterProblem();
    guess.isCompatible(getProblemRep(), true);
    clearGuess();
    m_guessFromAPI = std::move(guess);
}
void MucoCasADiSolver::setGuessFile(const std::string& file) {
    clearGuess();
    set_guess_file(file);
}
void MucoCasADiSolver::clearGuess() {
    m_guessFromAPI = MucoIterate();
    m_guessFromFile = MucoIterate();
    set_guess_file("");
    m_guessToUse.reset();
}
const MucoIterate& MucoCasADiSolver::getGuess() const {
    if (!m_guessToUse) {
        if (get_guess_file() != "" && m_guessFromFile.empty()) {
            // The API should make it impossible for both guessFromFile and
            // guessFromAPI to be non-empty.
            assert(m_guessFromAPI.empty());
            // No need to load from file again if we've already loaded it.
            MucoIterate guessFromFile(get_guess_file());
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

std::unique_ptr<CasADiTranscription>
MucoCasADiSolver::createTranscription() const {
    checkPropertyInSet(*this, getProperty_dynamics_mode(),
            {"explicit", "implicit"});
    std::unique_ptr<CasADiTranscription> transcrip;
    if (get_dynamics_mode() == "explicit") {
        transcrip = make_unique<CasADiTrapezoidal>(*this, getProblemRep());
    } else if (get_dynamics_mode() == "implicit") {
        transcrip = make_unique<CasADiTrapezoidalImplicit>(
                *this, getProblemRep());
    }
    transcrip->initialize();
    return transcrip;
}

MucoSolution MucoCasADiSolver::solveImpl() const {
    const Stopwatch stopwatch;

    checkPropertyInSet(*this, getProperty_verbosity(), {0, 1, 2});

    if (get_verbosity()) {
        std::cout << std::string(79, '=') << "\n";
        std::cout << "MucoCasADiSolver starting.\n";
        std::cout << std::string(79, '-') << std::endl;
        getProblemRep().printDescription();
    }
    checkPropertyIsPositive(*this, getProperty_num_mesh_points());
    auto transcription = createTranscription();
    // opt.disp(std::cout, true);

    // Initial guess.
    // --------------
    MucoIterate guess = getGuess();
    if (guess.empty()) {
        transcription->setGuess(transcription->createInitialGuessFromBounds());
    } else {
        transcription->setGuess(*m_guessToUse);
    }

    /*
    m_opti.disp(std::cout, true);
    std::cout << "DEBUG jacobian " << std::endl;
    std::cout << jacobian(m_opti.g(), m_opti.x()) << std::endl;
    std::cout << "DEBUG sparsity " << std::endl;
    jacobian(m_opti.g(), m_opti.x()).sparsity().to_file("DEBUG_sparsity.mtx");
    // TODO look at portions of the hessian (individual integrands).
    // TODO is it really the hessian or is it the constraints that are
    // expensive?
    hessian(m_opti.f(), m_opti.x()).sparsity().to_file("DEBUG_sparsity.mtx");
    */

    // Set solver options.
    // -------------------
    Dict solverOptions;
    checkPropertyInSet(*this, getProperty_optim_solver(), {"ipopt"});


    checkPropertyInRangeOrSet(*this, getProperty_optim_max_iterations(),
            0, std::numeric_limits<int>::max(), {-1});
    checkPropertyInRangeOrSet(*this, getProperty_optim_convergence_tolerance(),
            0.0, SimTK::NTraits<double>::getInfinity(), {-1.0});
    checkPropertyInRangeOrSet(*this, getProperty_optim_constraint_tolerance(),
            0.0, SimTK::NTraits<double>::getInfinity(), {-1.0});
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
    Dict pluginOptions;
    pluginOptions["verbose_init"] = true;

    MucoSolution mucoSolution = transcription->solve(get_optim_solver(),
                pluginOptions, solverOptions);
    const auto& casadiStats = transcription->getStats();
    setSolutionStats(mucoSolution, casadiStats.at("success"),
            casadiStats.at("return_status"), casadiStats.at("iter_count"));

    if (get_verbosity()) {
        std::cout << std::string(79, '-') << "\n";
        std::cout << "Elapsed real time: "
                << stopwatch.getElapsedTimeFormatted() << ".\n";
        if (mucoSolution) {
            std::cout << "MucoCasADiSolver succeeded!\n";
        } else {
            // TODO cout or cerr?
            std::cout << "MucoCasADiSolver did NOT succeed:\n";
            std::cout << "  " << mucoSolution.getStatus() << "\n";
        }
        std::cout << std::string(79, '=') << std::endl;
    }
    return mucoSolution;
}
