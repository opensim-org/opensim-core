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
    m_vars[initial_time] = MX::sym("initial_time");
    m_vars[final_time] = MX::sym("final_time");
    m_duration = m_vars[final_time] - m_vars[initial_time];
    m_times = createTimes(m_vars[initial_time], m_vars[final_time]);
    m_vars[states] =
            MX::sym("states", m_problem.getNumStates(), m_numGridPoints);
    m_vars[controls] =
            MX::sym("controls", m_problem.getNumControls(), m_numGridPoints);
    m_vars[multipliers] = MX::sym(
            "multipliers", m_problem.getNumMultipliers(), m_numGridPoints);
    m_vars[derivatives] = MX::sym(
            "derivatives", m_problem.getNumDerivatives(), m_numGridPoints);
    // TODO: This assumes that slack variables are applied at all
    // collocation points on the mesh interval interior.
    m_vars[slacks] = MX::sym(
            "slacks", m_problem.getNumSlacks(), m_numPointsIgnoringConstraints);
    m_vars[parameters] = MX::sym("parameters", m_problem.getNumParameters(), 1);

    m_paramsTrajGrid = MX::repmat(m_vars[parameters], 1, m_numGridPoints);
    m_paramsTraj = MX::repmat(m_vars[parameters], 1, m_numMeshPoints);
    m_paramsTrajIgnoringConstraints =
            MX::repmat(m_vars[parameters], 1, m_numPointsIgnoringConstraints);

    m_kinematicConstraintIndices = createKinematicConstraintIndices();
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

    setVariableBounds(initial_time, 0, 0, m_problem.getTimeInitialBounds());
    setVariableBounds(final_time, 0, 0, m_problem.getTimeFinalBounds());

    {
        const auto& stateInfos = m_problem.getStateInfos();
        int is = 0;
        for (const auto& info : stateInfos) {
            setVariableBounds(
                    states, is, Slice(1, m_numGridPoints - 1), info.bounds);
            // The "0" grabs the first column (first mesh point).
            setVariableBounds(states, is, 0, info.initialBounds);
            // The "-1" grabs the last column (last mesh point).
            setVariableBounds(states, is, -1, info.finalBounds);
            ++is;
        }
    }
    {
        const auto& controlInfos = m_problem.getControlInfos();
        int ic = 0;
        for (const auto& info : controlInfos) {
            setVariableBounds(
                    controls, ic, Slice(1, m_numGridPoints - 1), info.bounds);
            setVariableBounds(controls, ic, 0, info.initialBounds);
            setVariableBounds(controls, ic, -1, info.finalBounds);
            ++ic;
        }
    }
    {
        const auto& multiplierInfos = m_problem.getMultiplierInfos();
        int im = 0;
        for (const auto& info : multiplierInfos) {
            setVariableBounds(multipliers, im, Slice(1, m_numGridPoints - 1),
                    info.bounds);
            setVariableBounds(multipliers, im, 0, info.initialBounds);
            setVariableBounds(multipliers, im, -1, info.finalBounds);
            ++im;
        }
    }
    {
        if (m_solver.isDynamicsModeImplicit()) {
            // "Slice()" grabs everything in that dimension (like ":" in
            // Matlab).
            // TODO: How to choose bounds on udot?
            setVariableBounds(derivatives, Slice(), Slice(), {-1000, 1000});
        }
    }
    {
        const auto& slackInfos = m_problem.getSlackInfos();
        int isl = 0;
        for (const auto& info : slackInfos) {
            setVariableBounds(slacks, isl, Slice(), info.bounds);
            ++isl;
        }
    }
    {
        const auto& paramInfos = m_problem.getParameterInfos();
        int ip = 0;
        for (const auto& info : paramInfos) {
            setVariableBounds(parameters, ip, 0, info.bounds);
            ++ip;
        }
    }
}

void Transcription::transcribe() {

    // Cost.
    // =====
    setObjective();

    // Compute DAEs at necessary grid points.
    // ======================================
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
    m_xdot = MX(NS, m_numGridPoints);
    if (m_solver.isDynamicsModeImplicit()) {
        m_residual = MX(
                m_problem.getNumMultibodyDynamicsEquations(), m_numGridPoints);
    }
    m_kcerr = MX(m_problem.getNumKinematicConstraintEquations(),
            m_kinematicConstraintIndices.nnz());

    // qdot
    // ----
    const MX u = m_vars[states](Slice(NQ, NQ + NU), Slice());
    m_xdot(Slice(0, NQ), Slice()) = u;

    if (m_problem.getEnforceConstraintDerivatives() &&
            m_numPointsIgnoringConstraints &&
            !m_problem.isPrescribedKinematics()) {
        // In Hermite-Simpson, we must compute a velocity correction at all mesh
        // interval midpoints and update qdot. See MocoCasADiVelocityCorrection
        // for more details. This function only takes multibody state variables:
        // coordinates and speeds.
        // TODO: The points at which we apply the velocity correction
        // are correct for Trapezoidal (no points) and Hermite-Simpson (mesh
        // interval midpoints), but might not be correct in general. Revisit
        // this if we add other transcription schemes.
        const auto velocityCorrOut = evalOnTrajectory(
                m_problem.getVelocityCorrection(), {multibody_states, slacks},
                m_daeIndicesIgnoringConstraints);
        const auto uCorr = velocityCorrOut.at(0);

        m_xdot(Slice(0, NQ), m_daeIndicesIgnoringConstraints) += uCorr;
    }

    // udot, zdot, residual, kcerr
    // ---------------------------
    if (m_solver.isDynamicsModeImplicit()) {

        // udot.
        const MX w = m_vars[derivatives];
        m_xdot(Slice(NQ, NQ + NU), Slice()) = w;

        std::vector<Var> inputs{states, controls, multipliers, derivatives};

        // When the model has kinematic constraints, we must treat grid points
        // differently, as kinematic constraints are computed for only some
        // grid points. When the model does *not* have kinematic constraints,
        // the DAE is the same for all grid points, but the evaluation is still
        // done separately to keep implementation general.

        // residual, zdot, kcerr
        // Points where we compute algebraic constraints.
        {
            const auto out =
                    evalOnTrajectory(m_problem.getImplicitMultibodySystem(),
                            inputs, m_daeIndices);
            m_residual(Slice(), m_daeIndices) = out.at(0);
            // zdot.
            m_xdot(Slice(NQ + NU, NS), m_daeIndices) = out.at(1);
            m_kcerr = out.at(2);
        }

        // Points where we ignore algebraic constraints.
        if (m_numPointsIgnoringConstraints) {
            const auto out = evalOnTrajectory(
                    m_problem.getImplicitMultibodySystemIgnoringConstraints(),
                    inputs, m_daeIndicesIgnoringConstraints);
            m_residual(Slice(), m_daeIndicesIgnoringConstraints) = out.at(0);
            // zdot.
            m_xdot(Slice(NQ + NU, NS), m_daeIndicesIgnoringConstraints) =
                    out.at(1);
        }

    } else { // Explicit dynamics mode.
        std::vector<Var> inputs{states, controls, multipliers, derivatives};

        // udot, zdot, kcerr.
        // Points where we compute algebraic constraints.
        {
            // Evaluate the multibody system function and get udot
            // (speed derivatives) and zdot (auxiliary derivatives).
            const auto out = evalOnTrajectory(
                    m_problem.getMultibodySystem(), inputs, m_daeIndices);
            m_xdot(Slice(NQ, NQ + NU), m_daeIndices) = out.at(0);
            m_xdot(Slice(NQ + NU, NS), m_daeIndices) = out.at(1);
            m_kcerr = out.at(2);
        }

        // Points where we ignore algebraic constraints.
        if (m_numPointsIgnoringConstraints) {
            const auto out = evalOnTrajectory(
                    m_problem.getMultibodySystemIgnoringConstraints(), inputs,
                    m_daeIndicesIgnoringConstraints);
            m_xdot(Slice(NQ, NQ + NU), m_daeIndicesIgnoringConstraints) =
                    out.at(0);
            m_xdot(Slice(NQ + NU, NS), m_daeIndicesIgnoringConstraints) =
                    out.at(1);
        }
    }

    // Path constraints
    // ----------------
    // The individual path constraint functions are passed to CasADi to
    // maximize CasADi's ability to take derivatives efficiently.
    m_path.resize(m_problem.getPathConstraintInfos().size());
    for (int ipc = 0; ipc < (int)m_path.size(); ++ipc) {
        const auto& info = m_problem.getPathConstraintInfos()[ipc];
        // TODO: Is it sufficiently general to apply these to mesh points?
        const auto out = evalOnTrajectory(*info.function,
                {states, controls, multipliers, derivatives}, m_daeIndices);
        m_path[ipc] = out.at(0);
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
        integrandTraj = evalOnTrajectory(m_problem.getIntegralCostIntegrand(),
                {states, controls, multipliers, derivatives}, m_gridIndices)
                                .at(0);
    }

    // Minimize Lagrange multipliers if specified by the solver.
    if (m_solver.getMinimizeLagrangeMultipliers() &&
            m_problem.getNumMultipliers()) {
        const auto mults = m_vars[multipliers];
        const double multiplierWeight = m_solver.getLagrangeMultiplierWeight();
        // Sum across constraints of each multiplier element squared.
        integrandTraj += multiplierWeight * MX::sum1(MX::sq(mults));
    }
    MX integralCost = m_duration * dot(quadCoeffs.T(), integrandTraj);

    MXVector endpointCostOut;
    m_problem.getEndpointCost().call(
            {m_vars[final_time], m_vars[states](Slice(), -1),
                    m_vars[controls](Slice(), -1),
                    m_vars[multipliers](Slice(), -1),
                    m_vars[derivatives](Slice(), -1), m_vars[parameters]},
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
    const auto guessTimes = createTimes(guessOrig.variables.at(initial_time),
            guessOrig.variables.at(final_time));
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
    if (!m_solver.getWriteSparsity().empty()) {
        const auto prefix = m_solver.getWriteSparsity();
        auto gradient = casadi::MX::gradient(nlp["f"], nlp["x"]);
        gradient.sparsity().to_file(
                prefix + "_objective_gradient_sparsity.mtx");
        auto hessian = casadi::MX::hessian(nlp["f"], nlp["x"]);
        hessian.sparsity().to_file(prefix + "_objective_Hessian_sparsity.mtx");
        auto lagrangian = m_objective +
                          casadi::MX::dot(casadi::MX::ones(nlp["g"].sparsity()),
                                  nlp["g"]);
        auto hessian_lagr = casadi::MX::hessian(lagrangian, nlp["x"]);
        hessian_lagr.sparsity().to_file(
                prefix + "_Lagrangian_Hessian_sparsity.mtx");
        auto jacobian = casadi::MX::jacobian(nlp["g"], nlp["x"]);
        jacobian.sparsity().to_file(
                prefix + "constraint_Jacobian_sparsity.mtx");
    }
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
    solution.objective = nlpResult.at("f").scalar();
    solution.times = createTimes(
            solution.variables[initial_time], solution.variables[final_time]);
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
    casGuess.times = createTimes(
            casGuess.variables[initial_time], casGuess.variables[final_time]);
    return casGuess;
}

Iterate Transcription::createRandomIterateWithinBounds(
        const SimTK::Random* randGen) const {
    static const SimTK::Random::Uniform randGenDefault(-1, 1);
    const SimTK::Random* randGenToUse = &randGenDefault;
    if (randGen) randGenToUse = randGen;
    auto setRandom = [&](DM& output, const DM& lowerDM, const DM& upperDM) {
        for (int irow = 0; irow < output.rows(); ++irow) {
            for (int icol = 0; icol < output.columns(); ++icol) {
                const auto& lower = double(lowerDM(irow, icol));
                const auto& upper = double(upperDM(irow, icol));
                const auto rand = randGenToUse->getValue();
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
    casIterate.times = createTimes(casIterate.variables[initial_time],
            casIterate.variables[final_time]);
    return casIterate;
}

casadi::MXVector Transcription::evalOnTrajectory(
        const casadi::Function& pointFunction, const std::vector<Var>& inputs,
        const casadi::Matrix<casadi_int>& timeIndices) const {
    auto parallelism = m_solver.getParallelism();
    const auto trajFunc = pointFunction.map(
            timeIndices.size2(), parallelism.first, parallelism.second);

    // Assemble input.
    // Add 1 for time input and 1 for parameters input.
    MXVector mxIn(inputs.size() + 2);
    mxIn[0] = m_times(timeIndices);
    for (int i = 0; i < (int)inputs.size(); ++i) {
        if (inputs[i] == multibody_states) {
            const auto NQ = m_problem.getNumCoordinates();
            const auto NU = m_problem.getNumSpeeds();
            mxIn[i + 1] = m_vars.at(states)(Slice(0, NQ + NU), timeIndices);
        } else if (inputs[i] == slacks) {
            mxIn[i + 1] = m_vars.at(inputs[i]);
        } else {
            mxIn[i + 1] = m_vars.at(inputs[i])(Slice(), timeIndices);
        }
    }
    if (&timeIndices == &m_gridIndices) {
        mxIn[mxIn.size() - 1] = m_paramsTrajGrid;
    } else if (&timeIndices == &m_daeIndices) {
        mxIn[mxIn.size() - 1] = m_paramsTraj;
    } else if (&timeIndices == &m_daeIndicesIgnoringConstraints) {
        mxIn[mxIn.size() - 1] = m_paramsTrajIgnoringConstraints;
    } else {
        OPENSIM_THROW(OpenSim::Exception, "Internal error.");
    }
    MXVector mxOut;
    trajFunc.call(mxIn, mxOut);
    return mxOut;
    // TODO: Avoid the overhead of map() if not running in parallel.
    /* } else {
    casadi::MXVector out(pointFunction.n_out());
    for (int iout = 0; iout < (int)out.size(); ++iout) {
    out[iout] = casadi::MX(pointFunction.sparsity_out(iout).rows(),
    timeIndices.size2());
    }
    for (int itime = 0; itime < timeIndices.size2(); ++itime) {

    }
    }*/
}

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
