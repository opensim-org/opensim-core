/* -------------------------------------------------------------------------- *
 * OpenSim Moco: CasOCHermiteSimpson.cpp                                      *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
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
#include "CasOCHermiteSimpson.h"

using casadi::DM;
using casadi::MX;
using casadi::Slice;

namespace CasOC {

DM HermiteSimpson::createQuadratureCoefficientsImpl() const {

    // The duration of each mesh interval.
    const DM mesh(m_solver.getMesh());
    const DM meshIntervals = mesh(Slice(1, m_numMeshPoints)) -
                             mesh(Slice(0, m_numMeshPoints - 1));
    // Simpson quadrature includes integrand evaluations at the midpoint.
    DM quadCoeffs(m_numGridPoints, 1);
    // Loop through each mesh interval and update the corresponding components
    // in the total coefficients vector.
    for (int i = 0; i < m_numMeshIntervals; ++i) {
        // The mesh interval quadrature coefficients overlap at the mesh grid
        // points in the total coefficients vector, so we slice at every other
        // index to update the coefficients vector.
        quadCoeffs(2 * i) += (1.0 / 6.0) * meshIntervals(i);
        quadCoeffs(2 * i + 1) += (2.0 / 3.0) * meshIntervals(i);
        quadCoeffs(2 * i + 2) += (1.0 / 6.0) * meshIntervals(i);
    }
    return quadCoeffs;
}

DM HermiteSimpson::createMeshIndicesImpl() const {
    DM indices = DM::zeros(1, m_numGridPoints);
    for (int i = 0; i < m_numGridPoints; i += 2) { indices(i) = 1; }
    return indices;
}

DM HermiteSimpson::createControlIndicesImpl() const {
    DM indices = DM::zeros(1, m_numGridPoints);
    for (int i = 0; i < m_numGridPoints; i += 2) { indices(i) = 1; }
    return indices;
}

void HermiteSimpson::calcDefectsImpl(const casadi::MX& x,
        const casadi::MX& xdot, const casadi::MX& ti, const casadi::MX& tf,
        const casadi::MX& p, casadi::MX& defects) const {
    // For more information, see doxygen documentation for the class.

    int time_i;
    int time_mid;
    int time_ip1;
    const int NS = m_problem.getNumStates();
    const int NP = m_problem.getNumParameters();
    for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
        time_i = 2 * imesh; // Needed for defects and path constraints.

        // We enforce defect constraints on a mesh interval basis, so add
        // constraints until the number of mesh intervals is reached.
        time_mid = 2 * imesh + 1;
        time_ip1 = 2 * imesh + 2;

        const auto h = m_intervals(imesh);
        const auto x_i = x(Slice(), time_i);
        const auto x_mid = x(Slice(), time_mid);
        const auto x_ip1 = x(Slice(), time_ip1);
        const auto xdot_i = xdot(Slice(), time_i);
        const auto xdot_mid = xdot(Slice(), time_mid);
        const auto xdot_ip1 = xdot(Slice(), time_ip1);

        // Time variables.
        defects(Slice(0, 1), imesh) = ti(imesh + 1) - ti(imesh);
        defects(Slice(1, 2), imesh) = tf(imesh + 1) - tf(imesh);

        // Parameters.
        defects(Slice(2, 2 + NP), imesh) =
                p(Slice(), imesh + 1) - p(Slice(), imesh);

        // Hermite interpolant defects.
        defects(Slice(2 + NP, 2 + NP + NS), imesh) =
                x_mid - 0.5 * (x_ip1 + x_i) - (h / 8.0) * (xdot_i - xdot_ip1);

        // Simpson integration defects.
        defects(Slice(2 + NP + NS, 2 + NP + 2*NS), imesh) =
                x_ip1 - x_i - (h / 6.0) * (xdot_ip1 + 4.0 * xdot_mid + xdot_i);
    }
}

void HermiteSimpson::calcInterpolatingControlsImpl(casadi::MX& controls) const {
    calcInterpolatingControlsHelper(controls);
}

void HermiteSimpson::calcInterpolatingControlsImpl(casadi::DM& controls) const {
    calcInterpolatingControlsHelper(controls);
}

std::vector<std::pair<Var, int>> HermiteSimpson::getVariableOrder() const {
    std::vector<std::pair<Var, int>> order;
    int N = m_numPointsPerMeshInterval - 1;
    for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
        int igrid = imesh * N;
        order.push_back({initial_time, imesh});
        order.push_back({final_time, imesh});
        order.push_back({parameters, imesh});
        order.push_back({states, igrid});
        order.push_back({states, igrid + 1});
        if (m_solver.getInterpolateControlMidpoints()) {
            order.push_back({controls, igrid});
        } else {
            order.push_back({controls, igrid});
            order.push_back({controls, igrid + 1});
        }
        for (int i = 0; i < N; ++i) {
            order.push_back({multipliers, igrid + i});
        }
        for (int i = 0; i < N; ++i) {
            order.push_back({derivatives, igrid + i});
        }
        order.push_back({slacks, imesh});
    }
    order.push_back({initial_time, m_numMeshIntervals});
    order.push_back({final_time, m_numMeshIntervals});
    order.push_back({parameters, m_numMeshIntervals});
    order.push_back({states, m_numGridPoints - 1});
    order.push_back({controls, m_numGridPoints - 1});
    order.push_back({multipliers, m_numGridPoints - 1});
    order.push_back({derivatives, m_numGridPoints - 1});

    return order;
}

} // namespace CasOC
