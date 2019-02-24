/* -------------------------------------------------------------------------- *
 * OpenSim Moco: CasOCTranscription.cpp                                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2018 Stanford University and the Authors                     *
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
#include "CasOCTranscription.h"

using casadi::DM;
using casadi::MX;
using casadi::MXVector;
using casadi::Slice;

namespace CasOC {

void Transcription::createVariablesAndSetBounds() {
    // Set the grid.
    // -------------
    // The grid for a transcription scheme includes both mesh points (i.e.
    // points that lie on the endpoints of a mesh interval) and any
    // additional collocation points that may lie on mesh interior (as in
    // Hermite-Simpson collocation, etc.).
    m_numMeshIntervals = m_numMeshPoints - 1;
    m_numPointsIgnoringConstraints = m_numGridPoints - m_numMeshPoints;
    m_grid = DM::linspace(0, 1, m_numGridPoints);

    // Create variables.
    // -----------------
    m_vars[Var::initial_time] = MX::sym("initial_time");
    m_vars[Var::final_time] = MX::sym("final_time");
    m_duration = m_vars[Var::final_time] - m_vars[Var::initial_time];
    m_times = createTimes(m_vars[Var::initial_time], m_vars[Var::final_time]);
    m_vars[Var::states] =
            MX::sym("states", m_problem.getNumStates(), m_numGridPoints);
    m_vars[Var::controls] =
            MX::sym("controls", m_problem.getNumControls(), m_numGridPoints);
    m_vars[Var::multipliers] = MX::sym(
            "multipliers", m_problem.getNumMultipliers(), m_numGridPoints);
    if (m_solver.isDynamicsModeImplicit()) {
        m_vars[Var::derivatives] = MX::sym(
                "derivatives", m_problem.getNumSpeeds(), m_numGridPoints);
    }
    // TODO: This assumes that slack variables are applied at all
    // collocation points on the mesh interval interior.
    m_vars[Var::slacks] = MX::sym(
            "slacks", m_problem.getNumSlacks(), m_numPointsIgnoringConstraints);
    m_vars[Var::parameters] =
            MX::sym("parameters", m_problem.getNumParameters(), 1);

    m_paramsTrajGrid = MX::repmat(m_vars[Var::parameters], 1, m_numGridPoints);
    m_paramsTraj = MX::repmat(m_vars[Var::parameters], 1, m_numMeshPoints);
    m_paramsTrajIgnoringConstraints = MX::repmat(
            m_vars[Var::parameters], 1, m_numPointsIgnoringConstraints);

    m_kinematicConstraintIndices = createKinematicConstraintIndices();
    m_residualIndices = createResidualConstraintIndicesImpl();
    std::vector<int> daeIndicesVector;
    std::vector<int> daeIndicesIgnoringConstraintsVector;
    for (int i = 0; i < m_kinematicConstraintIndices.size2(); ++i) {
        if (m_kinematicConstraintIndices(i).scalar() == 1) {
            daeIndicesVector.push_back(i);
        } else {
            daeIndicesIgnoringConstraintsVector.push_back(i);
        }
    }

    auto makeTimeIndices = [](const std::vector<int>& in) {
        casadi::Matrix<casadi_int> out(1, in.size());
        for (int i = 0; i < (int)in.size(); ++i) { out(i) = in[i]; }
        return out;
    };
    {
        std::vector<int> gridIndicesVector(m_numGridPoints);
        std::iota(gridIndicesVector.begin(), gridIndicesVector.end(), 0);
        m_gridIndices = makeTimeIndices(gridIndicesVector);
    }
    m_daeIndices = makeTimeIndices(daeIndicesVector);
    m_daeIndicesIgnoringConstraints =
            makeTimeIndices(daeIndicesIgnoringConstraintsVector);

    // Set variable bounds.
    // --------------------
    auto initializeBounds = [&](VariablesDM& bounds) {
        for (auto& kv : m_vars) {
            bounds[kv.first] = DM(kv.second.rows(), kv.second.columns());
        }
    };
    initializeBounds(m_lowerBounds);
    initializeBounds(m_upperBounds);

    setVariableBounds(
            Var::initial_time, 0, 0, m_problem.getTimeInitialBounds());
    setVariableBounds(Var::final_time, 0, 0, m_problem.getTimeFinalBounds());

    {
        const auto& stateInfos = m_problem.getStateInfos();
        int is = 0;
        for (const auto& info : stateInfos) {
            setVariableBounds(Var::states, is, Slice(1, m_numGridPoints - 1),
                    info.bounds);
            // The "0" grabs the first column (first mesh point).
            setVariableBounds(Var::states, is, 0, info.initialBounds);
            // The "-1" grabs the last column (last mesh point).
            setVariableBounds(Var::states, is, -1, info.finalBounds);
            ++is;
        }
    }
    {
        const auto& controlInfos = m_problem.getControlInfos();
        int ic = 0;
        for (const auto& info : controlInfos) {
            setVariableBounds(Var::controls, ic, Slice(1, m_numGridPoints - 1),
                    info.bounds);
            setVariableBounds(Var::controls, ic, 0, info.initialBounds);
            setVariableBounds(Var::controls, ic, -1, info.finalBounds);
            ++ic;
        }
    }
    {
        const auto& multiplierInfos = m_problem.getMultiplierInfos();
        int im = 0;
        for (const auto& info : multiplierInfos) {
            setVariableBounds(Var::multipliers, im,
                    Slice(1, m_numGridPoints - 1), info.bounds);
            setVariableBounds(Var::multipliers, im, 0, info.initialBounds);
            setVariableBounds(Var::multipliers, im, -1, info.finalBounds);
            ++im;
        }
    }
    {
        if (m_solver.isDynamicsModeImplicit()) {
            // TODO: How to choose bounds on udot?
            setVariableBounds(
                    Var::derivatives, Slice(), Slice(), {-1000, 1000});
        }
    }
    {
        const auto& slackInfos = m_problem.getSlackInfos();
        int isl = 0;
        for (const auto& info : slackInfos) {
            setVariableBounds(Var::slacks, isl, Slice(), info.bounds);
            ++isl;
        }
    }
    {
        const auto& paramInfos = m_problem.getParameterInfos();
        int ip = 0;
        for (const auto& info : paramInfos) {
            setVariableBounds(Var::parameters, ip, 0, info.bounds);
            ++ip;
        }
    }
}

void Transcription::transcribe() {

    // Cost.
    // -----
    setObjective();

    // Compute DAEs at necessary grid points.
    // --------------------------------------
    const int NQ = m_problem.getNumCoordinates();
    const int NU = m_problem.getNumSpeeds();
    const int NS = m_problem.getNumStates();
    OPENSIM_THROW_IF(NQ != NU, OpenSim::Exception,
            "Problems with differing numbers of coordinates and speeds are "
            "not supported (e.g., quaternions).");

    // TODO: Does creating all this memory have efficiency implications in
    // CasADi?
    // Initialize memory for state derivatives, implicit residuals, and
    // constraint errors.
    m_xdot = MX(m_problem.getNumStates(), m_numGridPoints);
    if (m_solver.isDynamicsModeImplicit()) {
        m_residual = MX(m_problem.getNumSpeeds(), m_residualIndices.nnz());
    }
    m_kcerr = MX(m_problem.getNumKinematicConstraintEquations(),
            m_kinematicConstraintIndices.nnz());

    // Iterate through the grid, computing state derivatives and constraint
    // errors as necessary.
    if (m_solver.isDynamicsModeImplicit()) {
        const MX u = m_vars[Var::states](Slice(NQ, NQ + NU), Slice());
        // qdot.
        m_xdot(Slice(0, NQ), Slice()) = u;
        const MX w = m_vars[Var::derivatives];
        // udot.
        m_xdot(Slice(NQ, NQ + NU), Slice()) = w;
        if (m_problem.getEnforceConstraintDerivatives()) {
            // Points where we compute algebraic constraints.
            {
                // TODO: Nearly an exact duplicate with below.
                // Evaluate the multibody system function and get multibody
                // implicit differential equation residuals and zdot
                // (auxiliary derivatives).
                const auto& timeSlice = m_daeIndices;
                const auto& multibodyPointFunc =
                        m_problem.getImplicitMultibodySystem();
                const MXVector inputs{m_times(timeSlice),
                        m_vars[Var::states](Slice(), timeSlice),
                        m_vars[Var::controls](Slice(), timeSlice),
                        m_vars[Var::multipliers](Slice(), timeSlice),
                        m_vars[Var::derivatives](Slice(), timeSlice),
                        m_paramsTraj};
                const auto trajOut =
                        evalOnTrajectory(multibodyPointFunc, inputs, timeSlice);

                m_residual(Slice(), timeSlice) = trajOut.at(0);
                // zdot.
                m_xdot(Slice(NQ + NU, NS), timeSlice) = trajOut.at(1);
                m_kcerr = trajOut.at(2);
            }
            // Points where we ignore algebraic constraints.
            if (m_numPointsIgnoringConstraints) {
                const auto& timeSlice = m_daeIndicesIgnoringConstraints;

                // In Hermite-Simpson, this is a midpoint, so we must compute a
                // velocity correction and update qdot. See
                // MocoCasADiVelocityCorrection for more details. This function
                // only takes multibody state variables: coordinates and speeds.
                // TODO: The points at which we apply the velocity correction
                // are correct for Trapezoidal and Hermite-Simpson, but might
                // not be correct in general. Revisit this if we add other
                // transcription schemes.
                const auto& velocityCorrPointFunc =
                        m_problem.getVelocityCorrection();
                const MXVector vcInputs{m_times(timeSlice),
                        m_vars[Var::states](Slice(0, NQ + NU), timeSlice),
                        m_vars[Var::slacks], m_paramsTrajIgnoringConstraints};
                auto velocityCorrTrajOut = evalOnTrajectory(
                        velocityCorrPointFunc, vcInputs, timeSlice);
                const auto uCorr = velocityCorrTrajOut.at(0);

                m_xdot(Slice(0, NQ), timeSlice) += uCorr;

                const auto& multibodyPointFunc =
                        m_problem
                                .getImplicitMultibodySystemIgnoringConstraints();
                const MXVector inputs{m_times(timeSlice),
                        m_vars[Var::states](Slice(), timeSlice),
                        m_vars[Var::controls](Slice(), timeSlice),
                        m_vars[Var::multipliers](Slice(), timeSlice),
                        m_vars[Var::derivatives](Slice(), timeSlice),
                        m_paramsTrajIgnoringConstraints};
                const auto trajOut =
                        evalOnTrajectory(multibodyPointFunc, inputs, timeSlice);
                m_residual(Slice(), timeSlice) = trajOut.at(0);
                // zdot.
                m_xdot(Slice(NQ + NU, NS), timeSlice) = trajOut.at(1);
            }
        } else {
            // Points where we compute algebraic constraints.
            {
                const auto& timeSlice = m_daeIndices;
                const auto& multibodyPointFunc =
                        m_problem.getImplicitMultibodySystem();
                const MXVector inputs{m_times(timeSlice),
                        m_vars[Var::states](Slice(), timeSlice),
                        m_vars[Var::controls](Slice(), timeSlice),
                        m_vars[Var::multipliers](Slice(), timeSlice),
                        m_vars[Var::derivatives](Slice(), timeSlice),
                        m_paramsTraj};
                const auto trajOut =
                        evalOnTrajectory(multibodyPointFunc, inputs, timeSlice);
                m_residual(Slice(), timeSlice) = trajOut.at(0);
                // zdot.
                m_xdot(Slice(NQ + NU, NS), timeSlice) = trajOut.at(1);
                m_kcerr = trajOut.at(2);
            }
            // Points where we ignore algebraic constraints.
            if (m_numPointsIgnoringConstraints) {
                const auto& timeSlice = m_daeIndicesIgnoringConstraints;
                const auto& multibodyPointFunc =
                        m_problem
                                .getImplicitMultibodySystemIgnoringConstraints();
                const MXVector inputs{m_times(timeSlice),
                        m_vars[Var::states](Slice(), timeSlice),
                        m_vars[Var::controls](Slice(), timeSlice),
                        m_vars[Var::multipliers](Slice(), timeSlice),
                        m_vars[Var::derivatives](Slice(), timeSlice),
                        m_paramsTrajIgnoringConstraints};
                const auto trajOut =
                        evalOnTrajectory(multibodyPointFunc, inputs, timeSlice);
                m_residual(Slice(), timeSlice) = trajOut.at(0);
                // zdot.
                m_xdot(Slice(NQ + NU, NS), timeSlice) = trajOut.at(1);
            }
        }
    } else {
        const MX u = m_vars[Var::states](Slice(NQ, NQ + NU), Slice());
        m_xdot(Slice(0, NQ), Slice()) = u;
        if (m_problem.getEnforceConstraintDerivatives()) {
            // Points where we compute algebraic constraints.
            {
                // TODO: Nearly an exact duplicate with below.
                const auto& timeSlice = m_daeIndices;
                const auto& multibodyPointFunc = m_problem.getMultibodySystem();
                const MXVector inputs{m_times(timeSlice),
                        m_vars[Var::states](Slice(), timeSlice),
                        m_vars[Var::controls](Slice(), timeSlice),
                        m_vars[Var::multipliers](Slice(), timeSlice),
                        m_paramsTraj};
                // Evaluate the multibody system function and get udot
                // (speed derivatives) and zdot (auxiliary derivatives).
                const auto trajOut =
                        evalOnTrajectory(multibodyPointFunc, inputs, timeSlice);
                m_xdot(Slice(NQ, NQ + NU), timeSlice) = trajOut.at(0);
                m_xdot(Slice(NQ + NU, NS), timeSlice) = trajOut.at(1);
                m_kcerr = trajOut.at(2);
            }
            // Points where we ignore algebraic constraints.
            if (m_numPointsIgnoringConstraints) {
                const auto& timeSlice = m_daeIndicesIgnoringConstraints;

                // TODO: The points at which we apply the velocity correction is
                // correct for Trapezoidal and Hermite-Simpson, but might not be
                // correct in general. Revisit this if we add other
                // transcription schemes.
                const auto& velocityCorrPointFunc =
                        m_problem.getVelocityCorrection();
                const MXVector vcInputs{m_times(timeSlice),
                        m_vars[Var::states](Slice(0, NQ + NU), timeSlice),
                        m_vars[Var::slacks], m_paramsTrajIgnoringConstraints};
                auto velocityCorrTrajOut = evalOnTrajectory(
                        velocityCorrPointFunc, vcInputs, timeSlice);
                const auto uCorr = velocityCorrTrajOut.at(0);

                m_xdot(Slice(0, NQ), timeSlice) += uCorr;

                const auto& multibodyPointFunc =
                        m_problem.getMultibodySystemIgnoringConstraints();
                const MXVector inputs{m_times(timeSlice),
                        m_vars[Var::states](Slice(), timeSlice),
                        m_vars[Var::controls](Slice(), timeSlice),
                        m_vars[Var::multipliers](Slice(), timeSlice),
                        m_paramsTrajIgnoringConstraints};
                const auto trajOut =
                        evalOnTrajectory(multibodyPointFunc, inputs, timeSlice);
                m_xdot(Slice(NQ, NQ + NU), timeSlice) = trajOut.at(0);
                m_xdot(Slice(NQ + NU, NS), timeSlice) = trajOut.at(1);
            }

        } else {
            // Points where we compute algebraic constraints.
            {
                const auto& timeSlice = m_daeIndices;
                const auto& multibodyPointFunc = m_problem.getMultibodySystem();
                const MXVector inputs{m_times(timeSlice),
                        m_vars[Var::states](Slice(), timeSlice),
                        m_vars[Var::controls](Slice(), timeSlice),
                        m_vars[Var::multipliers](Slice(), timeSlice),
                        m_paramsTraj};
                const auto trajOut =
                        evalOnTrajectory(multibodyPointFunc, inputs, timeSlice);
                m_xdot(Slice(NQ, NQ + NU), timeSlice) = trajOut.at(0);
                m_xdot(Slice(NQ + NU, NS), timeSlice) = trajOut.at(1);
                m_kcerr = trajOut.at(2);
            }
            // Points where we ignore algebraic constraints.
            if (m_numPointsIgnoringConstraints) {
                const auto& timeSlice = m_daeIndicesIgnoringConstraints;
                const auto& multibodyPointFunc =
                        m_problem.getMultibodySystemIgnoringConstraints();
                const MXVector inputs{m_times(timeSlice),
                        m_vars[Var::states](Slice(), timeSlice),
                        m_vars[Var::controls](Slice(), timeSlice),
                        m_vars[Var::multipliers](Slice(), timeSlice),
                        m_paramsTrajIgnoringConstraints};
                const auto trajOut =
                        evalOnTrajectory(multibodyPointFunc, inputs, timeSlice);
                m_xdot(Slice(NQ, NQ + NU), timeSlice) = trajOut.at(0);
                m_xdot(Slice(NQ + NU, NS), timeSlice) = trajOut.at(1);
            }
        }
    }

    // Apply constraints.
    // ------------------
    applyConstraints();
}

void Transcription::setObjective() {
    DM quadCoeffs = this->createQuadratureCoefficients();
    MX integrandTraj;
    {
        // Here, we include evaluations of the integral cost
        // integrand into the symbolic expression graph for the integral
        // cost. We are *not* numerically evaluating the integral cost
        // integrand here--that occurs when the function by casadi::nlpsol()
        // is evaluated.
        const auto integrandFunc = m_problem.getIntegralCostIntegrand();
        const MXVector inputs{m_times, m_vars[Var::states],
                m_vars[Var::controls],
                // TODO m_vars[Var::multipliers],
                m_paramsTrajGrid};
        integrandTraj =
                evalOnTrajectory(integrandFunc, inputs, m_gridIndices).at(0);
    }

    // Minimize Lagrange multipliers if specified by the solver.
    if (m_solver.getMinimizeLagrangeMultipliers() &&
            m_problem.getNumMultipliers()) {
        const auto mults = m_vars[Var::multipliers];
        const double multiplierWeight = m_solver.getLagrangeMultiplierWeight();
        // Sum across constraints of each multiplier element squared.
        using casadi::MX;
        integrandTraj += multiplierWeight * MX::sum1(MX::sq(mults));
    }
    MX integralCost = m_duration * dot(quadCoeffs.T(), integrandTraj);

    // "Slice()" grabs everything in that dimension (like ":" in Matlab).
    MXVector endpointCostOut;
    m_problem.getEndpointCost().call(
            {m_vars[Var::final_time], m_vars[Var::states](Slice(), -1),
                    m_vars[Var::parameters]},
            endpointCostOut);
    const auto endpointCost = endpointCostOut.at(0);

    m_objective = integralCost + endpointCost;
}

Solution Transcription::solve(const Iterate& guessOrig) {

    // Define the NLP.
    // ---------------
    transcribe();

    // Resample the guess.
    // -------------------
    const auto guessTimes =
            createTimes(guessOrig.variables.at(Var::initial_time),
                    guessOrig.variables.at(Var::final_time));
    auto guess = guessOrig.resample(guessTimes);

    // Adjust guesses for the slack variables to ensure they are the correct
    // length (i.e. slacks.size2() == m_numPointsIgnoringConstraints).
    if (guess.variables.find(Var::slacks) != guess.variables.end()) {
        auto& slacks = guess.variables.at(Var::slacks);

        // If slack variables provided in the guess are equal to the grid
        // length, remove the elements on the mesh points where the slack
        // variables are not defined.
        if (slacks.size2() == m_numGridPoints) {
            casadi::DM kinConIndices = createKinematicConstraintIndices();
            std::vector<casadi_int> slackColumnsToRemove;
            for (int itime = 0; itime < m_numGridPoints; ++itime) {
                if (kinConIndices(itime).__nonzero__()) {
                    slackColumnsToRemove.push_back(itime);
                }
            }
            // The first argument is an empty vector since we don't want to
            // remove an entire row.
            slacks.remove(std::vector<casadi_int>(), slackColumnsToRemove);
        }

        // Check that either that the slack variables provided in the guess
        // are the correct length, or that the correct number of columns
        // were removed.
        OPENSIM_THROW_IF(slacks.size2() != m_numPointsIgnoringConstraints,
                OpenSim::Exception,
                OpenSim::format("Expected slack variables to be length %i, "
                                "but they are length %i.",
                        m_numPointsIgnoringConstraints, slacks.size2()));
    }

    // Create the CasADi NLP function.
    // -------------------------------
    // Option handling is copied from casadi::OptiNode::solver().
    casadi::Dict options = m_solver.getPluginOptions();
    if (!options.empty()) {
        options[m_solver.getOptimSolver()] = m_solver.getSolverOptions();
    }
    // The inputs to nlpsol() are symbolic (casadi::MX).
    casadi::MXDict nlp;
    nlp.emplace(std::make_pair("x", flatten(m_vars)));
    // The m_objective symbolic variable holds an expression graph including
    // all the calculations performed on the variables x.
    nlp.emplace(std::make_pair("f", m_objective));
    // The m_constraints symbolic vector holds all of the expressions for
    // the constraint functions.
    // veccat() concatenates std::vectors into a single MX vector.
    nlp.emplace(std::make_pair("g", casadi::MX::veccat(m_constraints)));
    // auto gradient = casadi::MX::gradient(nlp["f"], nlp["x"]);
    // gradient.sparsity().to_file(
    //         "CasOCTranscription_objective_gradient_sparsity.mtx");
    // auto hessian = casadi::MX::hessian(nlp["f"], nlp["x"]);
    // hessian.sparsity().to_file(
    //         "CasOCTranscription_objective_Hessian_sparsity.mtx");
    // auto lagrangian = m_objective +
    //         casadi::MX::dot(casadi::MX::ones(nlp["g"].sparsity()),
    //         nlp["g"]);
    // auto hessian_lagr = casadi::MX::hessian(lagrangian, nlp["x"]);
    // hessian_lagr.sparsity().to_file(
    //                  "CasOCTranscription_Lagrangian_Hessian_sparsity.mtx");
    // auto jacobian = casadi::MX::jacobian(nlp["g"], nlp["x"]);
    // jacobian.sparsity().to_file(
    //         "CasOCTranscription_constraint_Jacobian_sparsity.mtx");
    const casadi::Function nlpFunc =
            casadi::nlpsol("nlp", m_solver.getOptimSolver(), nlp, options);

    // Run the optimization (evaluate the CasADi NLP function).
    // --------------------------------------------------------
    // The inputs and outputs of nlpFunc are numeric (casadi::DM).
    const casadi::DMDict nlpResult = nlpFunc(casadi::DMDict{
            {"x0", flatten(guess.variables)}, {"lbx", flatten(m_lowerBounds)},
            {"ubx", flatten(m_upperBounds)},
            {"lbg", casadi::DM::veccat(m_constraintsLowerBounds)},
            {"ubg", casadi::DM::veccat(m_constraintsUpperBounds)}});

    // Create a CasOC::Solution.
    // -------------------------
    Solution solution = m_problem.createIterate<Solution>();
    solution.variables = expand(nlpResult.at("x"));
    solution.times = createTimes(solution.variables[Var::initial_time],
            solution.variables[Var::final_time]);
    solution.stats = nlpFunc.stats();
    return solution;
}

Iterate Transcription::createInitialGuessFromBounds() const {
    auto setToMidpoint = [](DM& output, const DM& lowerDM, const DM& upperDM) {
        for (int irow = 0; irow < output.rows(); ++irow) {
            for (int icol = 0; icol < output.columns(); ++icol) {
                const auto& lower = double(lowerDM(irow, icol));
                const auto& upper = double(upperDM(irow, icol));
                if (!std::isinf(lower) && !std::isinf(upper)) {
                    output(irow, icol) = 0.5 * (upper + lower);
                } else if (!std::isinf(lower))
                    output(irow, icol) = lower;
                else if (!std::isinf(upper))
                    output(irow, icol) = upper;
                else
                    output(irow, icol) = 0;
            }
        }
    };
    Iterate casGuess = m_problem.createIterate();
    casGuess.variables = m_lowerBounds;
    for (auto& kv : casGuess.variables) {
        setToMidpoint(kv.second, m_lowerBounds.at(kv.first),
                m_upperBounds.at(kv.first));
    }
    casGuess.times = createTimes(casGuess.variables[Var::initial_time],
            casGuess.variables[Var::final_time]);
    return casGuess;
}

Iterate Transcription::createRandomIterateWithinBounds() const {
    static SimTK::Random::Uniform randGen(-1, 1);
    auto setRandom = [](DM& output, const DM& lowerDM, const DM& upperDM) {
        for (int irow = 0; irow < output.rows(); ++irow) {
            for (int icol = 0; icol < output.columns(); ++icol) {
                const auto& lower = double(lowerDM(irow, icol));
                const auto& upper = double(upperDM(irow, icol));
                const auto rand = randGen.getValue();
                auto value = 0.5 * (rand + 1.0) * (upper - lower) + lower;
                if (std::isnan(value)) value = SimTK::clamp(lower, rand, upper);
                output(irow, icol) = value;
            }
        }
    };
    Iterate casIterate = m_problem.createIterate();
    casIterate.variables = m_lowerBounds;
    for (auto& kv : casIterate.variables) {
        setRandom(kv.second, m_lowerBounds.at(kv.first),
                m_upperBounds.at(kv.first));
    }
    casIterate.times = createTimes(casIterate.variables[Var::initial_time],
            casIterate.variables[Var::final_time]);
    return casIterate;
}

casadi::MXVector Transcription::evalOnTrajectory(
        const casadi::Function& pointFunction, const casadi::MXVector& inputs,
        const casadi::Matrix<casadi_int>& timeIndices) const {
    auto parallelism = m_solver.getParallelism();
    // TODO if (parallelism.second > 1) {
    const auto trajFunc = pointFunction.map(
            timeIndices.size2(), parallelism.first, parallelism.second);
    casadi::MXVector out;
    trajFunc.call(inputs, out);
    return out;
    // TODO: Avoid the overhead of map() if not running in parallel.
    /* TODO } else {
    casadi::MXVector out(pointFunction.n_out());
    for (int iout = 0; iout < (int)out.size(); ++iout) {
    out[iout] = casadi::MX(pointFunction.sparsity_out(iout).rows(),
    timeIndices.size2());
    }
    for (int itime = 0; itime < timeIndices.size2(); ++itime) {

    }
    }*/
}

/*
void Transcription::calcDifferentialAlgebraicEquationsExplicit(casadi_int itime,
        casadi_int islack, bool calcKinematicConstraintErrors, MX& xdot,
        MX& kcerr) {
    const auto& states = m_vars[Var::states];
    const int NQ = m_problem.getNumCoordinates();
    const int NU = m_problem.getNumSpeeds();
    const MX u = states(Slice(NQ, NQ + NU), itime);
    MX qdot = u; // TODO: This assumes the N matrix is identity.
    // TODO: What to do with this function?
}

void Transcription::calcDifferentialAlgebraicEquationsImplicit(casadi_int itime,
        casadi_int islack, bool calcResidual,
        bool calcKinematicConstraintErrors, MX& xdot, MX& residual, MX& kcerr) {
    const auto& states = m_vars[Var::states];
    const int NQ = m_problem.getNumCoordinates();
    const int NU = m_problem.getNumSpeeds();
    const MX u = states(Slice(NQ, NQ + NU), itime);
    MX qdot = u; // TODO: This assumes the N matrix is identity.
}
 */

void Transcription::addConstraints(const casadi::DM& lower,
        const casadi::DM& upper, const casadi::MX& equations) {
    OPENSIM_THROW_IF(
            lower.size() != upper.size() || lower.size() != equations.size(),
            OpenSim::Exception, "Arguments must have the same size.");
    m_constraintsLowerBounds.push_back(lower);
    m_constraintsUpperBounds.push_back(upper);
    m_constraints.push_back(equations);
}

} // namespace CasOC
