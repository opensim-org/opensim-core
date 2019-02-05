/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoTropterSolver.cpp                                        *
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
#include "MocoTropterSolver.h"
#include "MocoProblemRep.h"
#include "MocoUtilities.h"

#include "tropter/TropterProblem.h"

using namespace OpenSim;

MocoTropterSolver::MocoTropterSolver() {
    constructProperties();
}

void MocoTropterSolver::constructProperties() {
    constructProperty_optim_jacobian_approximation("exact");
    constructProperty_optim_sparsity_detection("random");
    constructProperty_transcription_scheme("trapezoidal");
    constructProperty_velocity_correction_bounds({-0.1, 0.1});
    constructProperty_exact_hessian_block_sparsity_mode();
    constructProperty_minimize_lagrange_multipliers(false);
    constructProperty_lagrange_multiplier_weight(1);

    // This is empty to allow user input error checking.
    constructProperty_enforce_constraint_derivatives();
}

std::shared_ptr<const MocoTropterSolver::TropterProblemBase<double>>
MocoTropterSolver::createTropterProblem() const {
    checkPropertyInSet(*this, getProperty_dynamics_mode(), {"explicit",
                                                            "implicit"});
    if (get_dynamics_mode() == "explicit") {
        return std::make_shared<ExplicitTropterProblem<double>>(*this);
    } else if (get_dynamics_mode() == "implicit") {
        return std::make_shared<ImplicitTropterProblem<double>>(*this);
    } else {
        OPENSIM_THROW_FRMOBJ(Exception, "Internal error.");
    }
}

void MocoTropterSolver::resetProblemImpl(const MocoProblemRep&) const {
}

MocoIterate MocoTropterSolver::createGuess(const std::string& type) const {
    OPENSIM_THROW_IF_FRMOBJ(
               type != "bounds"
            && type != "random"
            && type != "time-stepping",
            Exception,
            format("Unexpected guess type '%s'; supported types are 'bounds', "
                   "'random', and 'time-stepping'.", type));

    if (type == "time-stepping") {
        return createGuessTimeStepping();
    }

    // TODO avoid performing error checks multiple times; use
    // isObjectUpToDateWithProperties();
    checkPropertyIsPositive(*this, getProperty_num_mesh_points());
    int N = get_num_mesh_points();

    checkPropertyInSet(*this, getProperty_optim_solver(), {"ipopt", "snopt"});
    checkPropertyInSet(*this, getProperty_transcription_scheme(), 
        {"trapezoidal", "hermite-simpson"});
    auto ocp = createTropterProblem();
    tropter::DirectCollocationSolver<double> dircol(ocp, 
            get_transcription_scheme(),
            get_optim_solver(), N);

    tropter::Iterate tropIter;
    if (type == "bounds") {
        tropIter = dircol.make_initial_guess_from_bounds();
    } else if (type == "random") {
        tropIter = dircol.make_random_iterate_within_bounds();
    }
    return ocp->convertToMocoIterate(tropIter);
}

void MocoTropterSolver::setGuess(MocoIterate guess) {
    // Ensure the guess is compatible with this solver/problem.
    // Make sure to initialize the problem. TODO put in a better place.
    createTropterProblem();
    guess.isCompatible(getProblemRep(), true);
    clearGuess();
    m_guessFromAPI = std::move(guess);
}
void MocoTropterSolver::setGuessFile(const std::string& file) {
    clearGuess();
    set_guess_file(file);
}
void MocoTropterSolver::clearGuess() {
    m_guessFromAPI = MocoIterate();
    m_guessFromFile = MocoIterate();
    set_guess_file("");
    m_guessToUse.reset();
}
const MocoIterate& MocoTropterSolver::getGuess() const {
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
    // if (m_guessToUse) m_guessToUse->write("DEBUG_tropter_guess.sto");
    return m_guessToUse.getRef();
}

void MocoTropterSolver::printOptimizationSolverOptions(std::string solver) {
    if (solver == "ipopt") {
        tropter::optimization::IPOPTSolver::print_available_options();
    } else {
        std::cout << "No info available for " << solver << " options." <<
                std::endl;
    }
}

MocoSolution MocoTropterSolver::solveImpl() const {
    const Stopwatch stopwatch;

    auto ocp = createTropterProblem();

    // Apply settings/options.
    // -----------------------
    // Check that a valid verbosity level was provided.
    checkPropertyInSet(*this, getProperty_verbosity(), {0, 1, 2});
    // Problem print information is verbosity 1 or 2.
    if (get_verbosity()) {
        std::cout << std::string(79, '=') << "\n";
        std::cout << "MocoTropterSolver starting.\n";
        std::cout << std::string(79, '-') << std::endl;
        getProblemRep().printDescription();
    }
    // Check that a positive number of mesh points was provided.
    checkPropertyIsPositive(*this, getProperty_num_mesh_points());
    // Check that a valid optimization solver was specified.
    checkPropertyInSet(*this, getProperty_optim_solver(), {"ipopt", "snopt"});
    // Check that a valid transcription scheme was specified.
    checkPropertyInSet(*this, getProperty_transcription_scheme(),
        {"trapezoidal", "hermite-simpson"});
    // Enforcing constraint derivatives is only supported when Hermite-Simpson
    // is set as the transcription scheme.
    if (!getProperty_enforce_constraint_derivatives().empty()) {
        OPENSIM_THROW_IF(get_transcription_scheme() != "hermite-simpson" &&
                get_enforce_constraint_derivatives(), Exception,
                format("If enforcing derivatives of model kinematic "
                       "constraints, then the property 'transcription_scheme' "
                       "must be set to 'hermite-simpson'. "
                       "Currently, it is set to '%s'.",
                        get_transcription_scheme()));
    }
    // Block sparsity detected is only in effect when using an exact Hessian
    // approximation.
    OPENSIM_THROW_IF(get_optim_hessian_approximation() == "limited-memory" &&
        !getProperty_exact_hessian_block_sparsity_mode().empty(), Exception, 
        "A value for solver property 'exact_hessian_block_sparsity_mode' was "
        "provided, but is unused when using a 'limited-memory' Hessian "
        "approximation. Set solver property 'optim_hessian_approximation' to "
        "'exact' for Hessian block sparsity to take effect.");
    if (!getProperty_exact_hessian_block_sparsity_mode().empty()) {
        checkPropertyInSet(*this, 
            getProperty_exact_hessian_block_sparsity_mode(),
            {"dense", "sparse"});
    }
    // Hessian information is not used in SNOPT.
    OPENSIM_THROW_IF(get_optim_hessian_approximation() == "exact" &&
        get_optim_solver() == "snopt", Exception,
        "The property 'optim_hessian_approximation' was set to exact while "
        "using SNOPT as the optimization solver, but SNOPT does not utilize "
        "Hessian information.");

    // Check that the Lagrange multiplier weight is positive
     checkPropertyIsPositive(*this, getProperty_lagrange_multiplier_weight());

    // Create direct collocation solver.
    // ---------------------------------
    tropter::DirectCollocationSolver<double> dircol(ocp,
        get_transcription_scheme(),
        get_optim_solver(), 
        get_num_mesh_points());
    dircol.set_verbosity(get_verbosity() >= 1);
    if (getProperty_exact_hessian_block_sparsity_mode().empty()) {
        dircol.set_exact_hessian_block_sparsity_mode("dense");
    } else {
        dircol.set_exact_hessian_block_sparsity_mode(
            get_exact_hessian_block_sparsity_mode());
    }

    // Get optimization solver to check the remaining property settings.
    auto& optsolver = dircol.get_opt_solver();

    // Check that number of max iterations is valid.
    checkPropertyInRangeOrSet(*this, getProperty_optim_max_iterations(),
            0, std::numeric_limits<int>::max(), {-1});
    if (get_optim_max_iterations() != -1)
        optsolver.set_max_iterations(get_optim_max_iterations());
    // Check that convergence tolerance is valid.
    checkPropertyInRangeOrSet(*this,
            getProperty_optim_convergence_tolerance(),
            0.0, SimTK::NTraits<double>::getInfinity(), {-1.0});
    if (get_optim_convergence_tolerance() != -1)
        optsolver.set_convergence_tolerance(get_optim_convergence_tolerance());
    // Check that constraint tolerance is valid.
    checkPropertyInRangeOrSet(*this,
            getProperty_optim_constraint_tolerance(),
            0.0, SimTK::NTraits<double>::getInfinity(), {-1.0});
    if (get_optim_constraint_tolerance() != -1)
        optsolver.set_constraint_tolerance(get_optim_constraint_tolerance());

    optsolver.set_jacobian_approximation(get_optim_jacobian_approximation());
    optsolver.set_hessian_approximation(get_optim_hessian_approximation());

    if (get_optim_solver() == "ipopt") {
        // Check that IPOPT print level is valid.
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
    // Check that sparsity detection mode is valid.
    checkPropertyInSet(*this, getProperty_optim_sparsity_detection(),
            {"random", "initial-guess"});
    optsolver.set_sparsity_detection(get_optim_sparsity_detection());

    // Set advanced settings.
    //for (int i = 0; i < getProperty_optim_solver_options(); ++i) {
    //    optsolver.set_advanced_option(TODO);
    //}
    //optsolver.set_advanced_option_string("print_timing_statistics", "yes");
    // TODO optsolver.set_advanced_option_string("derivative_test", "second-order");
    // TODO optsolver.set_findiff_hessian_step_size(1e-3);

    tropter::Iterate tropIterate = ocp->convertToTropterIterate(getGuess());
    tropter::Solution tropSolution = dircol.solve(tropIterate);

    if (get_verbosity()) {
        dircol.print_constraint_values(tropSolution);
    }

    MocoSolution mocoSolution = ocp->convertToMocoSolution(tropSolution);

    // If enforcing model constraints and not minimizing Lagrange multipliers,
    // check the rank of the constraint Jacobian and if rank-deficient, print
    // recommendation to the user to enable Lagrange multiplier minimization.
    if (!getProperty_enforce_constraint_derivatives().empty() && 
             !get_minimize_lagrange_multipliers()) {
        const auto& model = getProblemRep().getModel();
        const auto& matter = model.getMatterSubsystem();
        Storage storage = mocoSolution.exportToStatesStorage();
        // TODO update when we support multiple phases.
        auto statesTraj = StatesTrajectory::createFromStatesStorage(model, 
            storage);
        SimTK::Matrix G;
        SimTK::FactorQTZ G_qtz;
        bool isJacobianFullRank = true;
        int rank;
        // Jacobian rank should be time-independent, but loop through states 
        // until rank deficiency detected, just in case.
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

    // TODO move this to convert():
    MocoSolver::setSolutionStats(mocoSolution, tropSolution.success,
            tropSolution.status, tropSolution.num_iterations);

    if (get_verbosity()) {
        std::cout << std::string(79, '-') << "\n";
        std::cout << "Elapsed real time: "
                << stopwatch.getElapsedTimeFormatted() << ".\n";
        if (mocoSolution) {
            std::cout << "MocoTropterSolver succeeded!\n";
        } else {
            // TODO cout or cerr?
            std::cout << "MocoTropterSolver did NOT succeed:\n";
            std::cout << "  " << mocoSolution.getStatus() << "\n";
        }
        std::cout << std::string(79, '=') << std::endl;
    }

    return mocoSolution;
}
