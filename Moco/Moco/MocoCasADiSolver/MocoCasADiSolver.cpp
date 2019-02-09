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
#include "MocoCasADiMisc.h"
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
    // TODO: Move to MocoCasADiMisc.h
    const auto& problemRep = getProblemRep();
    OPENSIM_THROW_IF(problemRep.getNumKinematicConstraintEquations(), Exception,
            "MocoCasADiSolver does not support kinematic constraints yet.");
    auto casProblem = make_unique<CasOC::Problem>();
    checkPropertyInSet(
            *this, getProperty_dynamics_mode(), {"explicit", "implicit"});
    const auto& model = problemRep.getModel();
    auto stateNames = createStateVariableNamesInSystemOrder(model);
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
                name, casBounds, problemRep, pathCon);
    }
    casProblem->setIntegralCost<MocoCasADiIntegralCostIntegrand>(problemRep);
    casProblem->setEndpointCost<MocoCasADiEndpointCost>(problemRep);
    // TODO if implicit, use different function.
    casProblem->setMultibodySystem<MocoCasADiMultibodySystem>(problemRep);
    return casProblem;
}

std::unique_ptr<CasOC::Solver> MocoCasADiSolver::createCasOCSolver(
        const CasOC::Problem& casProblem) const {
    auto casSolver = make_unique<CasOC::Solver>(casProblem);

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

    checkPropertyInRangeOrSet(*this, getProperty_optim_max_iterations(), 0,
            std::numeric_limits<int>::max(), {-1});
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

    checkPropertyIsPositive(*this, getProperty_num_mesh_points());
    casSolver->setNumMeshPoints(get_num_mesh_points());
    casSolver->setTranscriptionScheme("trapezoidal");
    casSolver->setOptimSolver(get_optim_solver());
    casSolver->setPluginOptions(pluginOptions);
    casSolver->setSolverOptions(solverOptions);
    return casSolver;
}

MocoSolution MocoCasADiSolver::solveImpl() const {
    const Stopwatch stopwatch;

    checkPropertyInSet(*this, getProperty_verbosity(), {0, 1, 2});

    if (get_verbosity()) {
        std::cout << std::string(79, '=') << "\n";
        std::cout << "MocoCasADiSolver starting.\n";
        std::cout << std::string(79, '-') << std::endl;
        getProblemRep().printDescription();
    }
    auto casProblem = createCasOCProblem();
    auto casSolver = createCasOCSolver(*casProblem);

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
