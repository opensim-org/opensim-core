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

#include <OpenSim/Common/Stopwatch.h>

#ifdef OPENSIM_WITH_TROPTER
    #include "tropter/TropterProblem.h"
#endif

using namespace OpenSim;

MocoTropterSolver::MocoTropterSolver() { constructProperties(); }

void MocoTropterSolver::constructProperties() {
    constructProperty_optim_jacobian_approximation("exact");
    constructProperty_optim_sparsity_detection("random");
    constructProperty_exact_hessian_block_sparsity_mode();
}

bool MocoTropterSolver::isAvailable() {
#ifdef OPENSIM_WITH_TROPTER
    return true;
#else
    return false;
#endif
}

std::shared_ptr<const MocoTropterSolver::TropterProblemBase<double>>
MocoTropterSolver::createTropterProblem() const {
#ifdef OPENSIM_WITH_TROPTER
    checkPropertyValueIsInSet(
            getProperty_multibody_dynamics_mode(), {"explicit", "implicit"});
    if (get_multibody_dynamics_mode() == "explicit") {
        return std::make_shared<ExplicitTropterProblem<double>>(*this);
    } else if (get_multibody_dynamics_mode() == "implicit") {
        return std::make_shared<ImplicitTropterProblem<double>>(*this);
    } else {
        OPENSIM_THROW_FRMOBJ(Exception, "Internal error.");
    }
#else
    OPENSIM_THROW(MocoTropterSolverNotAvailable);
#endif
}
std::unique_ptr<tropter::DirectCollocationSolver<double>>
MocoTropterSolver::createTropterSolver(
        std::shared_ptr<const MocoTropterSolver::TropterProblemBase<double>>
                ocp) const {
#ifdef OPENSIM_WITH_TROPTER
    // Check that a non-negative number of mesh points was provided.
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
    // Check that a valid optimization solver was specified.
    checkPropertyValueIsInSet(getProperty_optim_solver(), {"ipopt", "snopt"});
    // Check that a valid transcription scheme was specified.
    checkPropertyValueIsInSet(getProperty_transcription_scheme(),
            {"trapezoidal", "hermite-simpson"});
    // Enforcing constraint derivatives is only supported when Hermite-Simpson
    // is set as the transcription scheme.

    if (getProblemRep().getNumKinematicConstraintEquations()) {
        OPENSIM_THROW_IF(get_transcription_scheme() != "hermite-simpson" &&
                                 get_enforce_constraint_derivatives(),
                Exception,
                "If enforcing derivatives of model kinematic constraints, then "
                "the property 'transcription_scheme' must be set to "
                "'hermite-simpson'. Currently, it is set to '{}'.",
                get_transcription_scheme());
    }
    OPENSIM_THROW_IF_FRMOBJ(
            getProblemRep().getNumImplicitAuxiliaryResiduals(),
            Exception, "MocoTropterSolver does not support problems "
                       "with implicit auxiliary dynamics.");

    // Block sparsity detected is only in effect when using an exact Hessian
    // approximation.
    OPENSIM_THROW_IF(
            get_optim_hessian_approximation() == "limited-memory" &&
                    !getProperty_exact_hessian_block_sparsity_mode().empty(),
            Exception,
            "A value for solver property 'exact_hessian_block_sparsity_mode' "
            "was provided, but is unused when using a 'limited-memory' Hessian "
            "approximation. Set solver property 'optim_hessian_approximation' "
            "to 'exact' for Hessian block sparsity to take effect.");
    if (!getProperty_exact_hessian_block_sparsity_mode().empty()) {
        checkPropertyValueIsInSet(
                getProperty_exact_hessian_block_sparsity_mode(),
                {"dense", "sparse"});
    }
    // Hessian information is not used in SNOPT.
    OPENSIM_THROW_IF(get_optim_hessian_approximation() == "exact" &&
                             get_optim_solver() == "snopt",
            Exception,
            "The property 'optim_hessian_approximation' was set to exact while "
            "using SNOPT as the optimization solver, but SNOPT does not "
            "utilize "
            "Hessian information.");

    // Check that the Lagrange multiplier weight is positive
    checkPropertyValueIsPositive(getProperty_lagrange_multiplier_weight());

    // Create direct collocation solver.
    // ---------------------------------

    std::unique_ptr<tropter::DirectCollocationSolver<double>> dircol;

    if (getProperty_mesh().empty()) {
        dircol = OpenSim::make_unique<tropter::DirectCollocationSolver<double>>(
                ocp, get_transcription_scheme(), get_optim_solver(),
                get_num_mesh_intervals());
    } else {
        std::vector<double> mesh;
        for (int i = 0; i < getProperty_mesh().size(); ++i) {
            mesh.push_back(get_mesh(i));
        }
        dircol = OpenSim::make_unique<tropter::DirectCollocationSolver<double>>(
                ocp, get_transcription_scheme(), get_optim_solver(), mesh);
    }

    dircol->set_verbosity(get_verbosity() >= 1);
    if (getProperty_exact_hessian_block_sparsity_mode().empty()) {
        dircol->set_exact_hessian_block_sparsity_mode("dense");
    } else {
        dircol->set_exact_hessian_block_sparsity_mode(
                get_exact_hessian_block_sparsity_mode());
    }

    // Get optimization solver to check the remaining property settings.
    auto& optsolver = dircol->get_opt_solver();

    // Check that number of max iterations is valid.
    checkPropertyValueIsInRangeOrSet(getProperty_optim_max_iterations(), 0,
            std::numeric_limits<int>::max(), {-1});
    if (get_optim_max_iterations() != -1)
        optsolver.set_max_iterations(get_optim_max_iterations());
    // Check that convergence tolerance is valid.
    checkPropertyValueIsInRangeOrSet(getProperty_optim_convergence_tolerance(),
            0.0, SimTK::NTraits<double>::getInfinity(), {-1.0});
    if (get_optim_convergence_tolerance() != -1)
        optsolver.set_convergence_tolerance(get_optim_convergence_tolerance());
    // Check that constraint tolerance is valid.
    checkPropertyValueIsInRangeOrSet(getProperty_optim_constraint_tolerance(),
            0.0, SimTK::NTraits<double>::getInfinity(), {-1.0});
    if (get_optim_constraint_tolerance() != -1)
        optsolver.set_constraint_tolerance(get_optim_constraint_tolerance());

    optsolver.set_jacobian_approximation(get_optim_jacobian_approximation());
    optsolver.set_hessian_approximation(get_optim_hessian_approximation());

    if (get_optim_solver() == "ipopt") {
        // Check that IPOPT print level is valid.
        checkPropertyValueIsInRangeOrSet(
                getProperty_optim_ipopt_print_level(), 0, 12, {-1});
        if (get_verbosity() < 2) {
            optsolver.set_advanced_option_int("print_level", 0);
        } else {
            if (get_optim_ipopt_print_level() != -1) {
                optsolver.set_advanced_option_int(
                        "print_level", get_optim_ipopt_print_level());
            }
        }
    }
    // Check that sparsity detection mode is valid.
    checkPropertyValueIsInSet(getProperty_optim_sparsity_detection(),
            {"random", "initial-guess"});
    optsolver.set_sparsity_detection(get_optim_sparsity_detection());

    // Set advanced settings.
    // for (int i = 0; i < getProperty_optim_solver_options(); ++i) {
    //    optsolver.set_advanced_option(TODO);
    //}
    // optsolver.set_advanced_option_string("print_timing_statistics",
    // "yes");
    // TODO optsolver.set_advanced_option_string("derivative_test",
    // "second-order");
    // TODO optsolver.set_findiff_hessian_step_size(1e-3);

    return dircol;
#else
    OPENSIM_THROW(MocoTropterSolverNotAvailable);
#endif
}

MocoTrajectory MocoTropterSolver::createGuess(const std::string& type) const {
#ifdef OPENSIM_WITH_TROPTER
    OPENSIM_THROW_IF_FRMOBJ(
            type != "bounds" && type != "random" && type != "time-stepping",
            Exception,
            "Unexpected guess type '{}'; supported types are "
            "'bounds', 'random', and 'time-stepping'.",
            type);

    if (type == "time-stepping") { return createGuessTimeStepping(); }

    auto ocp = createTropterProblem();
    auto dircol = createTropterSolver(ocp);

    tropter::Iterate tropIter;
    if (type == "bounds") {
        tropIter = dircol->make_initial_guess_from_bounds();
    } else if (type == "random") {
        tropIter = dircol->make_random_iterate_within_bounds();
    }
    return ocp->convertToMocoTrajectory(tropIter);
#else
    OPENSIM_THROW(MocoTropterSolverNotAvailable);
#endif
}

void MocoTropterSolver::setGuess(MocoTrajectory guess) {
    // Ensure the guess is compatible with this solver/problem.
    // Make sure to initialize the problem. TODO put in a better place.
    auto ocp = createTropterProblem();
    checkGuess(guess);
    clearGuess();
    m_guessFromAPI = std::move(guess);
}

void MocoTropterSolver::setGuessFile(const std::string& file) {
    clearGuess();
    set_guess_file(file);
}

void MocoTropterSolver::checkGuess(const MocoTrajectory& guess) const {
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

void MocoTropterSolver::clearGuess() {
    m_guessFromAPI = MocoTrajectory();
    m_guessFromFile = MocoTrajectory();
    set_guess_file("");
    m_guessToUse.reset();
}

const MocoTrajectory& MocoTropterSolver::getGuess() const {
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
            // This will either be a guess specified via the API, or empty
            // to signal that tropter should use the default guess.
            m_guessToUse.reset(&m_guessFromAPI);
        }
    }
    // if (m_guessToUse) m_guessToUse->write("DEBUG_tropter_guess.sto");
    return m_guessToUse.getRef();
}

void MocoTropterSolver::printOptimizationSolverOptions(std::string solver) {
#ifdef OPENSIM_WITH_TROPTER
    if (solver == "ipopt") {
        tropter::optimization::IPOPTSolver::print_available_options();
    } else {
        log_info("No info available for {} options.", solver);
    }
#else
    OPENSIM_THROW(MocoTropterSolverNotAvailable);
#endif
}

MocoSolution MocoTropterSolver::solveImpl() const {
#ifdef OPENSIM_WITH_TROPTER
    const Stopwatch stopwatch;

    OPENSIM_THROW_IF_FRMOBJ(getProblemRep().isPrescribedKinematics(), Exception,
            "MocoTropterSolver does not support prescribed kinematics. "
            "Try using prescribed motion constraints in the Coordinates.");

    auto ocp = createTropterProblem();

    // Apply settings/options.
    // -----------------------
    // Check that a valid verbosity level was provided.
    checkPropertyValueIsInSet(getProperty_verbosity(), {0, 1, 2});
    // Problem print information is verbosity 1 or 2.
    if (get_verbosity()) {
        log_info(std::string(72, '='));
        log_info("MocoTropterSolver starting.");
        log_info(getFormattedDateTime(false, "%c"));
        log_info(std::string(72, '-'));
        getProblemRep().printDescription();
    }
    auto dircol = createTropterSolver(ocp);
    MocoTrajectory guess = getGuess();
    tropter::Iterate tropIterate = ocp->convertToTropterIterate(guess);

    // Temporarily disable printing of negative muscle force warnings so the
    // output stream isn't flooded while computing finite differences.
    Logger::Level origLoggerLevel = Logger::getLevel();
    Logger::setLevel(Logger::Level::Warn);
    tropter::Solution tropSolution;
    try {
        tropSolution = dircol->solve(tropIterate);
    } catch (...) {
        OpenSim::Logger::setLevel(origLoggerLevel);
    }
    OpenSim::Logger::setLevel(origLoggerLevel);

    if (get_verbosity()) { dircol->print_constraint_values(tropSolution); }

    MocoSolution mocoSolution = ocp->convertToMocoSolution(tropSolution);

    // If enforcing model constraints and not minimizing Lagrange
    // multipliers, check the rank of the constraint Jacobian and if
    // rank-deficient, print recommendation to the user to enable Lagrange
    // multiplier minimization.
    if (getProblemRep().getNumKinematicConstraintEquations() &&
            !get_enforce_constraint_derivatives() &&
            !get_minimize_lagrange_multipliers()) {
        const auto& model = getProblemRep().getModelBase();
        const auto& matter = model.getMatterSubsystem();
        TimeSeriesTable states = mocoSolution.exportToStatesTable();
        // TODO update when we support multiple phases.
        auto statesTraj =
                StatesTrajectory::createFromStatesTable(model, states);
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
            const std::string dashes(53, '-');
            log_warn(dashes);
            log_warn("Rank-deficient constraint Jacobian detected.");
            log_warn(dashes);
            log_warn("The model constraint Jacobian has {} row(s) but is only "
                     "rank {}. ", G.nrow(), rank);
            log_warn("Try removing redundant constraints from the model or "
                     "enable");
            log_warn("minimization of Lagrange multipliers by utilizing the "
                     "solver ");
            log_warn("properties 'minimize_lagrange_multipliers' and");
            log_warn("'lagrange_multiplier_weight'.");
            log_warn(dashes);
        }
    }

    // TODO move this to convert():
    const long long elapsed = stopwatch.getElapsedTimeInNs();
    MocoSolver::setSolutionStats(mocoSolution, tropSolution.success,
            tropSolution.objective, tropSolution.status,
            tropSolution.num_iterations, SimTK::nsToSec(elapsed));

    if (get_verbosity()) {
        log_info(std::string(72, '-'));
        log_info("Elapsed real time: {}", stopwatch.formatNs(elapsed));
        log_info(getFormattedDateTime(false, "%c"));
        if (mocoSolution) {
            log_info("MocoTropterSolver succeeded!");
        } else {
            log_warn("MocoTropterSolver did NOT succeed:");
            log_warn("  {}", mocoSolution.getStatus());
        }
        log_info(std::string(72, '='));
    }

    return mocoSolution;
#else
    OPENSIM_THROW(MocoTropterSolverNotAvailable);
#endif
}
