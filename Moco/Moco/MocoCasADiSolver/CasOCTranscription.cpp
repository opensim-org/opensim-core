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
using casadi::Slice;

namespace CasOC {

void Transcription::transcribe() {

    // Set the grid.
    // -------------
    // The grid for a transcription scheme includes both mesh points (i.e.
    // points that lie on the endpoints of a mesh interval) and any additional
    // collocation points that may lie on mesh interior (as in Hermite-Simpson
    // collocation, etc.).
    m_numMeshIntervals = m_numMeshPoints - 1;
    m_numSlackPoints = m_numGridPoints - m_numMeshPoints;
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
    // TODO: This assumes that slack variables are applied at all collocation
    // points on the mesh interval interior.
    m_vars[Var::slacks] =
            MX::sym("slacks", m_problem.getNumSlacks(), m_numSlackPoints);
    m_vars[Var::parameters] =
            MX::sym("parameters", m_problem.getNumParameters(), 1);

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

    // Cost.
    // -----
    DM quadCoeffs = this->createQuadratureCoefficients();
    MX integralCost = 0;
    for (int itime = 0; itime < m_numGridPoints; ++itime) {
        // "Slice()" grabs everything in that dimension (like ":" in Matlab).
        // Here, we include evaluations of the integral cost integrand
        // into the symbolic expression graph for the integral cost. We are
        // *not* numerically evaluating the integral cost integrand here--that
        // occurs when the function by casadi::nlpsol() is evaluated.
        const auto out = m_problem.getIntegralCostIntegrand().operator()(
                {m_times(itime, 0), m_vars[Var::states](Slice(), itime),
                        m_vars[Var::controls](Slice(), itime),
                        m_vars[Var::parameters]});
        integralCost += quadCoeffs(itime) * out.at(0);

        // Minimize Lagrange multipliers if specified by the solver.
        if (m_solver.getMinimizeLagrangeMultipliers() &&
                m_problem.getNumMultipliers()) {
            const auto mults = m_vars[Var::multipliers](Slice(), itime);
            const double multiplierWeight =
                    m_solver.getLagrangeMultiplierWeight();
            integralCost += multiplierWeight * dot(mults, mults);
        }
    }
    integralCost *= m_duration;

    const auto endpointCostOut =
            m_problem.getEndpointCost().operator()({m_vars[Var::final_time],
                    m_vars[Var::states](Slice(), -1), m_vars[Var::parameters]});
    const auto endpointCost = endpointCostOut.at(0);
    setObjective(integralCost + endpointCost);

    // Compute DAEs at necessary grid points.
    // --------------------------------------
    const int NQ = m_problem.getNumCoordinates();
    const int NU = m_problem.getNumSpeeds();
    OPENSIM_THROW_IF(NQ != NU, OpenSim::Exception,
            "Problems with differing numbers of coordinates and speeds are not "
            "supported (e.g., quaternions).");
    const DM kinConIndices = createKinematicConstraintIndices();
    const DM resConIndices = createResidualConstraintIndicesImpl();

    // TODO: Does creating all this memory have efficiency implications in
    // CasADi?
    // Initialize memory for state derivatives and constraint errors.

    m_xdot = MX(m_problem.getNumStates(), m_numGridPoints);
    if (m_solver.isDynamicsModeImplicit()) {
        m_residual = MX(m_problem.getNumSpeeds(), resConIndices.nnz());
    }
    m_pvaerr = MX(m_problem.getNumKinematicConstraintEquations(),
            kinConIndices.nnz());

    // Temporary memory for state derivatives and constraint errors while
    // iterating through time points.
    MX this_xdot;
    MX this_residual;
    MX this_pvaerr;
    // Updateable index for constraint errors vector.
    int ikc = 0;
    // Updateable index for residual errors vector.
    int irc = 0;
    // Updateable index for slack variables. This index will always be -1 if
    // constraint derivatives are not being enforced (i.e. no slack variables),
    // which tells calcDAE() not to compute the velocity correction.
    int islack = m_problem.getEnforceConstraintDerivatives() ? 0 : -1;
    // Iterate through the grid, computing state derivatives and constraint
    // errors as necessary.
    for (int itime = 0; itime < m_numGridPoints; ++itime) {

        // If the value of kinConIndices is non-zero at this time point, then
        // we need to enforce kinematic constraint derivatives.
        const bool calcPVAErr = kinConIndices(itime).__nonzero__();

        // Calculate differential-algebraic equations and update the state
        // derivatives vector.
        if (m_solver.isDynamicsModeImplicit()) {
            const bool calcResidual = resConIndices(itime).__nonzero__();
            calcDAEImplicit(itime, this_xdot, calcResidual, this_residual,
                    calcPVAErr, this_pvaerr, islack);
            if (calcResidual) {
                m_residual(Slice(), irc) = this_residual;
                ++irc;
            }
        } else {
            calcDAEExplicit(itime, this_xdot, calcPVAErr, this_pvaerr, islack);
        }
        m_xdot(Slice(), itime) = this_xdot;

        // If calculating kinematic contraint errors, also update the constraint
        // errors vector and index.
        if (calcPVAErr) {
            m_pvaerr(Slice(), ikc) = this_pvaerr;
            ++ikc;
            // If not calculating constraint errors at this time point but
            // enforcing constraint derivatives in the problem, update the slack
            // variable index since we just used the current index.
        } else if (m_problem.getEnforceConstraintDerivatives()) {
            ++islack;
        }
    }

    // Apply constraints.
    // ------------------
    applyConstraints();
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

void Transcription::calcDAEExplicit(casadi_int itime, MX& xdot, bool calcPVAErr,
        MX& pvaerr, casadi_int islack) {
    const auto& states = m_vars[Var::states];
    const int NQ = m_problem.getNumCoordinates();
    const int NU = m_problem.getNumSpeeds();
    const MX u = states(Slice(NQ, NQ + NU), itime);
    MX qdot = u; // TODO: This assumes the N matrix is identity.

    // If slack variables exist and we're not computing constraint errors at
    // this time point, compute the velocity correction and update qdot.
    // See MocoCasADiVelocityCorrection for more details.
    if (!calcPVAErr && islack != -1) {
        const auto& velocityCorrFunc = m_problem.getVelocityCorrection();
        const auto velocityCorrOutput = velocityCorrFunc.operator()(
                {m_times(itime), m_vars[Var::states](Slice(), itime),
                        m_vars[Var::slacks](Slice(), islack)});
        qdot += velocityCorrOutput.at(0);
    }

    // Get the multibody system function.
    const auto& multibodyFunc =
            calcPVAErr ? m_problem.getMultibodySystem()
                       : m_problem.getUnconstrainedMultibodySystem();

    // Evaluate the multibody system function and get udot (speed derivatives)
    // and zdot (auxiliary derivatives).
    const auto dynamicsOutput = multibodyFunc.operator()({m_times(itime),
            m_vars[Var::states](Slice(), itime),
            m_vars[Var::controls](Slice(), itime),
            m_vars[Var::multipliers](Slice(), itime), m_vars[Var::parameters]});
    const MX udot = dynamicsOutput.at(0);
    const MX zdot = dynamicsOutput.at(1);

    // Concatenate derivatives to update the state derivatives vector.
    xdot = casadi::MX::vertcat({qdot, udot, zdot});

    // If calculating constraint errors, also update the constraint errors
    // vector.
    if (calcPVAErr) { pvaerr = dynamicsOutput.at(2); }
}

void Transcription::calcDAEImplicit(casadi_int itime, MX& xdot,
        bool calcResidual, MX& residual, bool calcPVAErr, MX& pvaerr,
        casadi_int islack) {
    const auto& states = m_vars[Var::states];
    const int NQ = m_problem.getNumCoordinates();
    const int NU = m_problem.getNumSpeeds();
    const MX u = states(Slice(NQ, NQ + NU), itime);
    MX qdot = u; // TODO: This assumes the N matrix is identity.

    const MX w = m_vars[Var::derivatives](Slice(), itime);
    MX udot = w;

    // Get the multibody system function.
    const auto& implicitMultibodyFunc = m_problem.getImplicitMultibodySystem();

    // Evaluate the multibody system function and get udot (speed derivatives)
    // and zdot (auxiliary derivatives).
    const auto dynamicsOutput = implicitMultibodyFunc.operator()({
            m_times(itime),
            m_vars[Var::states](Slice(), itime),
            m_vars[Var::controls](Slice(), itime),
            m_vars[Var::multipliers](Slice(), itime),
            m_vars[Var::derivatives](Slice(), itime), m_vars[Var::parameters]});
    residual = dynamicsOutput.at(0);
    const MX zdot = dynamicsOutput.at(1);

    // Concatenate derivatives to update the state derivatives vector.
    xdot = casadi::MX::vertcat({qdot, udot, zdot});

    if (calcPVAErr) pvaerr = casadi::MX::zeros(0, 1);
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
