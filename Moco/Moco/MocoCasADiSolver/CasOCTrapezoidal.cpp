/* -------------------------------------------------------------------------- *
 * OpenSim Moco: CasOCTrapezoidal.cpp                                         *
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
#include "CasOCTrapezoidal.h"

using casadi::DM;
using casadi::MX;
using casadi::Slice;

namespace CasOC {

Trapezoidal::Trapezoidal(const Solver& solver, const Problem& problem)
        : Transcription(solver, problem) {

    // Create variables.
    // -----------------
    // TODO: Move some of this to the Transcription base class?
    const int numMeshPoints = m_solver.getNumMeshPoints();
    m_vars[Var::initial_time] = MX::sym("initial_time");
    m_vars[Var::final_time] = MX::sym("final_time");
    m_vars[Var::states] =
            MX::sym("states", m_problem.getNumStates(), numMeshPoints);
    m_vars[Var::controls] =
            MX::sym("controls", m_problem.getNumControls(), numMeshPoints);
    m_vars[Var::multipliers] = MX::sym(
            "multipliers", m_problem.getNumMultipliers(), numMeshPoints);
    m_vars[Var::parameters] =
            MX::sym("parameters", m_problem.getNumParameters(), 1);

    m_mesh = DM::linspace(0, 1, numMeshPoints);
    m_duration = m_vars[Var::final_time] - m_vars[Var::initial_time];
    m_times = createTimes(m_vars[Var::initial_time], m_vars[Var::final_time]);

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
            setVariableBounds(
                    Var::states, is, Slice(1, numMeshPoints - 1), info.bounds);
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
            setVariableBounds(Var::controls, ic, Slice(1, numMeshPoints - 1),
                    info.bounds);
            setVariableBounds(Var::controls, ic, 0, info.initialBounds);
            setVariableBounds(Var::controls, ic, -1, info.finalBounds);
            ++ic;
        }
    }

    {
        const auto& paramInfos = m_problem.getParameterInfos();
        int ip = 0;
        for (const auto& info : paramInfos) {
            setVariableBounds(Var::parameters, ip, 0, info.bounds);
        }
    }

    // Cost.
    // -----
    const DM meshIntervals = m_mesh(Slice(1, numMeshPoints)) -
                             m_mesh(Slice(0, numMeshPoints - 1));
    DM quadCoeffs(numMeshPoints, 1);
    quadCoeffs(Slice(0, numMeshPoints - 1)) = 0.5 * meshIntervals;
    quadCoeffs(Slice(1, numMeshPoints)) += 0.5 * meshIntervals;
    MX integralCost = 0;
    for (int itime = 0; itime < numMeshPoints; ++itime) {
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
        // TODO: Obey user option for if this penalty should exist.
        if (m_problem.getNumMultipliers()) {
            const auto mults = m_vars[Var::multipliers](Slice(), itime);
            const int multiplierWeight =
                    100.0; // TODO
                           // m_mocoSolver.get_lagrange_multiplier_weight();
            integralCost += multiplierWeight * dot(mults, mults);
        }
    }
    integralCost *= m_duration;

    const auto endpointCostOut =
            m_problem.getEndpointCost().operator()({m_vars[Var::final_time],
                    m_vars[Var::states](Slice(), -1), m_vars[Var::parameters]});
    const auto endpointCost = endpointCostOut.at(0);
    setObjective(integralCost + endpointCost);

    // Compute DAE at all mesh points.
    // -------------------------------
    const auto& states = m_vars[Var::states];
    const int NQ = m_problem.getNumCoordinates();
    const int NU = m_problem.getNumSpeeds();
    OPENSIM_THROW_IF(NQ != NU, OpenSim::Exception, "NQ != NU");
    const auto calcDAE = [&](casadi_int itime, MX& xdot, MX& qerr) {
        const MX u = states(Slice(NQ, 2 * NQ), itime);
        const MX qdot = u; // TODO: This assumes the N matrix is identity.
        const auto dynamicsOutput = m_problem.getMultibodySystem().operator()(
                {m_times(itime), m_vars[Var::states](Slice(), itime),
                        m_vars[Var::controls](Slice(), itime),
                        m_vars[Var::multipliers](Slice(), itime),
                        m_vars[Var::parameters]});
        const MX udot = dynamicsOutput.at(0);
        const MX zdot = dynamicsOutput.at(1);
        xdot = casadi::MX::vertcat({qdot, udot, zdot});
        qerr = dynamicsOutput.at(2);
    };
    // TODO: Does creating all this memory have efficiency implications in
    //  CasADi?
    MX xdot(m_problem.getNumStates(), numMeshPoints);
    MX qerr(m_problem.getNumKinematicConstraintEquations(), numMeshPoints);
    MX this_xdot;
    MX this_qerr;
    for (int itime = 0; itime < numMeshPoints; ++itime) {
        calcDAE(itime, this_xdot, this_qerr);
        xdot(Slice(), itime) = this_xdot;
        qerr(Slice(), itime) = this_qerr;
    }

    // Apply defect and path constraints.
    // ----------------------------------
    // We have arranged the code this way so that all constraints (defects and
    // path constraints) at a given mesh point are grouped together. Organizing
    // the sparsity of the Jacobian this way might have benefits for sparse
    // linear algebra.
    const DM zero(m_problem.getNumStates(), 1);
    for (int itime = 0; itime < numMeshPoints; ++itime) {
        if (itime > 0) {
            const auto h = m_times(itime) - m_times(itime - 1);
            const auto x_i = states(Slice(), itime);
            const auto x_im1 = states(Slice(), itime - 1);
            const auto xdot_i = xdot(Slice(), itime);
            const auto xdot_im1 = xdot(Slice(), itime - 1);
            addConstraints(
                    zero, zero, x_i - (x_im1 + 0.5 * h * (xdot_i + xdot_im1)));
        }
        // TODO
        // if (m_problem.getNumKinematicConstraintEquations()) {
        //     addConstraints(kcLowerBounds, kcUpperBounds, qerr(Slice(),
        //     itime));
        // }
        // The individual path constraint functions are passed to CasADi to
        // maximize CasADi's ability to take derivatives efficiently.
        for (const auto& pathInfo : m_problem.getPathConstraintInfos()) {
            const auto output = pathInfo.function->operator()(
                    {m_times(itime), m_vars[Var::states](Slice(), itime),
                            m_vars[Var::controls](Slice(), itime),
                            m_vars[Var::parameters]});
            const auto& errors = output.at(0);
            addConstraints(pathInfo.lowerBounds, pathInfo.upperBounds, errors);
        }
    }
}

Iterate Trapezoidal::createInitialGuessFromBoundsImpl() const {
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

Iterate Trapezoidal::createRandomIterateWithinBoundsImpl() const {
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

} // namespace CasOC
