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
#include "MocoCasADiBridge.h"
extern template class OpenSim::MocoCasADiMultibodySystem<false>;
extern template class OpenSim::MocoCasADiMultibodySystem<true>;
extern template class OpenSim::MocoCasADiMultibodySystemImplicit<false>;
extern template class OpenSim::MocoCasADiMultibodySystemImplicit<true>;

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

std::unique_ptr<CasOC::Problem> MocoCasADiSolver::createCasOCProblem() const {
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
    m_jar = createProblemRepJar(numThreads);

    auto casProblem = make_unique<CasOC::Problem>();
    checkPropertyInSet(
            *this, getProperty_dynamics_mode(), {"explicit", "implicit"});
    const auto& model = problemRep.getModel();

    OPENSIM_THROW_IF(!model.getMatterSubsystem().getUseEulerAngles(
                             model.getWorkingState()),
            Exception, "Quaternions are not supported.");

    std::unordered_map<int, int> yIndexMap;
    auto stateNames = createStateVariableNamesInSystemOrder(model, yIndexMap);
    casProblem->setTimeBounds(convertBounds(problemRep.getTimeInitialBounds()),
            convertBounds(problemRep.getTimeFinalBounds()));
    for (const auto& stateName : stateNames) {
        const auto& info = problemRep.getStateInfo(stateName);
        CasOC::StateType stateType;
        if (endsWith(stateName, "/value"))
            stateType = CasOC::StateType::Coordinate;
        else if (endsWith(stateName, "/speed"))
            stateType = CasOC::StateType::Speed;
        else
            stateType = CasOC::StateType::Auxiliary;
        casProblem->addState(stateName, stateType,
                convertBounds(info.getBounds()),
                convertBounds(info.getInitialBounds()),
                convertBounds(info.getFinalBounds()));
    }
    for (const auto& actu : model.getComponentList<Actuator>()) {
        // TODO handle a variable number of control signals.
        const auto& actuName = actu.getAbsolutePathString();
        const auto& info = problemRep.getControlInfo(actuName);
        casProblem->addControl(actuName, convertBounds(info.getBounds()),
                convertBounds(info.getInitialBounds()),
                convertBounds(info.getFinalBounds()));
    }

    // Add any scalar constraints associated with kinematic constraints in
    // the model as path constraints in the problem.
    // Whether or not enabled kinematic constraints exist in the model,
    // check that optional solver properties related to constraints are
    // set properly.
    const auto kcNames = problemRep.createKinematicConstraintNames();
    if (kcNames.empty()) {
        OPENSIM_THROW_IF(!getProperty_enforce_constraint_derivatives().empty(),
                Exception,
                "Solver property 'enforce_constraint_derivatives' "
                "was set but no enabled kinematic constraints exist in the "
                "model.");
        OPENSIM_THROW_IF(get_minimize_lagrange_multipliers(), Exception,
                "Solver property 'minimize_lagrange_multipliers' "
                "was enabled but no enabled kinematic constraints exist in the "
                "model.");
    } else {
        OPENSIM_THROW_IF(getProperty_enforce_constraint_derivatives().empty(),
                Exception,
                "Enabled kinematic constraints exist in the "
                "provided model. Please set the solver property "
                "'enforce_constraint_derivatives' to either 'true' or "
                "'false'.");

        int cid, mp, mv, ma;
        int multIndexThisConstraint;
        int total_mp = 0;
        int total_mv = 0;
        int total_ma = 0;
        std::vector<KinematicLevel> kinLevels;
        const bool enforceConstraintDerivs =
                get_enforce_constraint_derivatives();
        for (const auto& kcName : kcNames) {
            const auto& kc = problemRep.getKinematicConstraint(kcName);
            const auto& multInfos = problemRep.getMultiplierInfos(kcName);
            cid = kc.getSimbodyConstraintIndex();
            mp = kc.getNumPositionEquations();
            mv = kc.getNumVelocityEquations();
            ma = kc.getNumAccelerationEquations();
            kinLevels = kc.getKinematicLevels();

            // TODO only add velocity correction variables for holonomic
            // constraint derivatives? For now, disallow enforcing derivatives
            // if non-holonomic or acceleration constraints present.
            OPENSIM_THROW_IF(enforceConstraintDerivs && mv != 0, Exception,
                    format("Enforcing constraint derivatives is supported only "
                           "for "
                           "holonomic (position-level) constraints. "
                           "There are %i velocity-level "
                           "scalar constraints associated with the model "
                           "Constraint "
                           "at ConstraintIndex %i.",
                            mv, cid));
            OPENSIM_THROW_IF(enforceConstraintDerivs && ma != 0, Exception,
                    format("Enforcing constraint derivatives is supported only "
                           "for "
                           "holonomic (position-level) constraints. "
                           "There are %i acceleration-level "
                           "scalar constraints associated with the model "
                           "Constraint "
                           "at ConstraintIndex %i.",
                            ma, cid));

            total_mp += mp;
            total_mv += mv;
            total_ma += ma;

            // Loop through all scalar constraints associated with the model
            // constraint and corresponding path constraints to the optimal
            // control problem.
            //
            // We need a different index for the Lagrange multipliers since
            // they are only added if the current constraint equation is not a
            // derivative of a position- or velocity-level equation.
            multIndexThisConstraint = 0;
            for (int i = 0; i < kc.getConstraintInfo().getNumEquations(); ++i) {

                // If the index for this path constraint represents an
                // a non-derivative scalar constraint equation, add a
                // Lagrange multiplier to the problem.
                if (kinLevels[i] == KinematicLevel::Position ||
                        kinLevels[i] == KinematicLevel::Velocity ||
                        kinLevels[i] == KinematicLevel::Acceleration) {

                    const auto& multInfo = multInfos[multIndexThisConstraint];

                    CasOC::KinematicLevel kinLevel;
                    if (kinLevels[i] == KinematicLevel::Position)
                        kinLevel = CasOC::KinematicLevel::Position;
                    else if (kinLevels[i] == KinematicLevel::Velocity)
                        kinLevel = CasOC::KinematicLevel::Velocity;
                    else if (kinLevels[i] == KinematicLevel::Acceleration)
                        kinLevel = CasOC::KinematicLevel::Acceleration;
                    else {
                        OPENSIM_THROW(OpenSim::Exception,
                                "Unrecognized KinematicLevel");
                    }

                    casProblem->addKinematicConstraint(multInfo.getName(),
                            convertBounds(multInfo.getBounds()),
                            convertBounds(multInfo.getInitialBounds()),
                            convertBounds(multInfo.getFinalBounds()), kinLevel);

                    // Add velocity correction variables if enforcing
                    // constraint equation derivatives.
                    if (enforceConstraintDerivs) {
                        // TODO this naming convention assumes that the
                        // associated Lagrange multiplier name begins with
                        // "lambda", which may change in the future.
                        OPENSIM_THROW_IF(
                                multInfo.getName().substr(0, 6) != "lambda",
                                Exception,
                                OpenSim::format(
                                        "Expected the multiplier name for "
                                        "this constraint to begin with "
                                        "'lambda' but it "
                                        "begins with '%s'.",
                                        multInfo.getName().substr(0, 6)));
                        casProblem->addSlack(std::string(multInfo.getName())
                                                     .replace(0, 6, "gamma"),
                                convertBounds(
                                        get_velocity_correction_bounds()));
                    }
                    ++multIndexThisConstraint;
                }
            }
        }

        // Set kinematic constraint information on the CasOC::Problem.
        casProblem->setEnforceConstraintDerivatives(enforceConstraintDerivs);
        // The bounds are the same for all kinematic constraints in the
        // MocoProblem, so just grab the bounds from the first constraint.
        const auto& kc = problemRep.getKinematicConstraint(kcNames.at(0));
        std::vector<MocoBounds> bounds = kc.getConstraintInfo().getBounds();
        casProblem->setKinematicConstraintBounds(convertBounds(bounds.at(0)));
        // Only add the velocity correction if enforcing constraint derivatives.
        if (enforceConstraintDerivs) {
            casProblem->setVelocityCorrection<MocoCasADiVelocityCorrection>(
                    *m_jar, yIndexMap);
        }
    }

    for (const auto& paramName : problemRep.createParameterNames()) {
        const auto& param = problemRep.getParameter(paramName);
        casProblem->addParameter(paramName, convertBounds(param.getBounds()));
    }
    const auto pathConstraintNames = problemRep.createPathConstraintNames();
    for (const auto& name : pathConstraintNames) {
        const auto& pathCon = problemRep.getPathConstraint(name);
        std::vector<CasOC::Bounds> casBounds;
        for (const auto& bounds : pathCon.getConstraintInfo().getBounds()) {
            casBounds.push_back(convertBounds(bounds));
        }
        casProblem->addPathConstraint<MocoCasADiPathConstraint>(
                name, casBounds, *m_jar, yIndexMap, name);
    }
    casProblem->setIntegralCost<MocoCasADiIntegralCostIntegrand>(
            *m_jar, yIndexMap);
    casProblem->setEndpointCost<MocoCasADiEndpointCost>(*m_jar, yIndexMap);
    casProblem->setMultibodySystem<MocoCasADiMultibodySystem>(
            *m_jar, *this, yIndexMap);
    casProblem->setImplicitMultibodySystem<MocoCasADiMultibodySystemImplicit>(
            *m_jar, *this, yIndexMap);

    return casProblem;
}

std::unique_ptr<CasOC::Solver> MocoCasADiSolver::createCasOCSolver(
        const CasOC::Problem& casProblem) const {
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
    casSolver->setDynamicsMode(get_dynamics_mode());
    casSolver->setMinimizeLagrangeMultipliers(
            get_minimize_lagrange_multipliers());
    casSolver->setLagrangeMultiplierWeight(get_lagrange_multiplier_weight());
    casSolver->setOptimSolver(get_optim_solver());
    if (m_jar->size() > 1) {
        casSolver->setParallelism("thread", m_jar->size());
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
        std::cout << "Number of threads: " << m_jar->size() << std::endl;
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
    setSolutionStats(mocoSolution, casSolution.stats.at("success"),
            casSolution.stats.at("return_status"),
            casSolution.stats.at("iter_count"));

    if (get_verbosity()) {
        std::cout << std::string(79, '-') << "\n";
        std::cout << "Elapsed real time: "
                  << stopwatch.getElapsedTimeFormatted() << ".\n";
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
