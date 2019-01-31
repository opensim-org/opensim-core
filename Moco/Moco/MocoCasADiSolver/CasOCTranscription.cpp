/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoCasOCTranscription.h                                     *
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

Trapezoidal::Trapezoidal(const Solver& solver, const Problem& problem)
        : Transcription(solver, problem) {

    // Create variables
    // ----------------
    // TODO: Move some of this to the Transcription base class?
    const int numMeshPoints = m_solver.getNumMeshPoints();
    m_vars[Var::initial_time] = m_opti.variable();
    m_vars[Var::final_time] = m_opti.variable();
    m_vars[Var::states] =
            m_opti.variable(m_problem.getNumStates(), numMeshPoints);
    m_vars[Var::controls] =
            m_opti.variable(m_problem.getNumControls(), numMeshPoints);
    m_vars[Var::multipliers] =
            m_opti.variable(m_problem.getNumMultipliers(), numMeshPoints);
    m_vars[Var::parameters] = m_opti.variable(m_problem.getNumParameters(), 1);
    m_mesh = DM::linspace(0, 1, numMeshPoints);
    m_duration = m_vars[Var::final_time] - m_vars[Var::initial_time];
    m_times = createTimes(m_vars[Var::initial_time], m_vars[Var::final_time]);

    // Set variable bounds
    // -------------------
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

    const auto& stateInfos = m_problem.getStateInfos();
    int is = 0;
    for (const auto& info : stateInfos) {
        setVariableBounds(Var::states, is, Slice(1, -2), info.bounds);
        setVariableBounds(Var::states, is, 0, info.initialBounds);
        setVariableBounds(Var::states, is, -1, info.finalBounds);
        ++is;
    }

    const auto& controlInfos = m_problem.getControlInfos();
    int ic = 0;
    for (const auto& info : controlInfos) {
        setVariableBounds(Var::controls, ic, Slice(1, -2), info.bounds);
        setVariableBounds(Var::controls, ic, 0, info.initialBounds);
        setVariableBounds(Var::controls, ic, -1, info.finalBounds);
        ++ic;
    }

    // Cost.
    // -----
    const DM meshIntervals = m_mesh(Slice(1, -1)) - m_mesh(Slice(0, -2));
    DM quadCoeffs(numMeshPoints, 1);
    quadCoeffs(Slice(0, -2)) = 0.5 * meshIntervals;
    quadCoeffs(Slice(1, -1)) += 0.5 * meshIntervals;
    MX integralCost = 0;
    for (int itime = 0; itime < numMeshPoints; ++itime) {
        const auto out = m_problem.getIntegralCostIntegrand().operator()(
                {m_times(itime, 0), m_vars[Var::states](Slice(), itime),
                        m_vars[Var::controls](Slice(), itime),
                        m_vars[Var::parameters]});
        integralCost += quadCoeffs(itime) * out.at(0);
        // TODO: Penalize multipliers.
    }
    integralCost *= m_duration;

    const auto endpointCostOut =
            m_problem.getEndpointCost().operator()({m_vars[Var::final_time],
                    m_vars[Var::states](Slice(), -1), m_vars[Var::parameters]});
    const auto endpointCost = endpointCostOut.at(0);
    m_opti.minimize(integralCost + endpointCost);

    // Defects.
    // --------
    const auto& states = m_vars[Var::states];
    const int NQ = m_problem.getNumCoordinates();
    const int NU = m_problem.getNumSpeeds();
    OPENSIM_THROW_IF(NQ != NU, OpenSim::Exception, "NQ != NU");
    const auto calcDAE = [&](casadi_int itime, MX& xdot, MX& qerr) {
        MX u = states(Slice(NQ, 2 * NQ), itime);
        MX qdot = u; // TODO: This assumes the N matrix is identity.
        auto dynamicsOutput = m_problem.getMultibodySystem().operator()(
                {m_times(itime), m_vars[Var::states](Slice(), itime),
                        m_vars[Var::controls](Slice(), itime),
                        m_vars[Var::multipliers](Slice(), itime),
                        m_vars[Var::parameters]});
        MX udot = dynamicsOutput.at(0);
        MX zdot = dynamicsOutput.at(1);
        xdot = casadi::MX::vertcat({qdot, udot, zdot});
        qerr = dynamicsOutput.at(2);
    };
    MX xdot(m_problem.getNumStates(), numMeshPoints);
    MX this_xdot;
    for (int itime = 0; itime < numMeshPoints; ++itime) {
        MX qerr;
        calcDAE(itime, this_xdot, qerr);
        xdot(Slice(), itime) = this_xdot;
        // if (m_problem.getNumKinematicConstraintEquations()) {
        //     m_opti.subject_to(kcLowerBounds <= qerr <= kcUpperBounds);
        // }
    }

    // Repeat h so that we have a (numStates x numMeshPoints) matrix to
    // multiply, elementwise, all states in the itime interval by h(itime).
    // repmat(v, n, m) repeats v vertically n times and horizontally m times.
    auto h = m_times(Slice(1, -1)) - m_times(Slice(0, -2));
    auto h_mat = repmat(h.T(), m_problem.getNumStates(), 1);
    auto x_i = states(Slice(), Slice(1, -1));
    auto x_im1 = states(Slice(), Slice(0, -2));
    auto xdot_i = xdot(Slice(), Slice(1, -1));
    auto xdot_im1 = xdot(Slice(), Slice(0, -2));
    m_opti.subject_to(x_i == (x_im1 + 0.5 * h_mat * (xdot_i + xdot_im1)));

    // Path constraints.
    // -----------------
    // TODO
    /*
    casadi::MX path_error;
    for (int itime = 0; itime < m_numTimes; ++itime) {
            pathError = prob.m_path_func({time, state, control})
            m_opti.subject_to(prob.m_pathLowerBounds <
    }
    m_opti.subje
     */
}

Solution Trapezoidal::solveImpl() {
    m_opti.solver(m_solver.getOptimSolver(), m_solver.getPluginOptions(),
            m_solver.getSolverOptions());
    VariablesDM varsDM;
    try {
        m_opti.solve();
        varsDM = convertToVariablesDM(m_opti, m_vars);
    } catch (const std::exception& e) {
        std::cerr << "MocoCasADiSolver did not succeed: " << e.what()
                  << std::endl;
        varsDM = convertToVariablesDM(m_opti.debug(), m_vars);
    }
    Solution solution = m_problem.createIterate<Solution>();
    solution.variables = varsDM;
    solution.times =
            createTimes(varsDM[Var::initial_time], varsDM[Var::final_time]);
    solution.stats = m_opti.stats();
    return solution;
}

} // namespace CasOC
