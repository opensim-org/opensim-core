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

void HermiteSimpson::calcDefectsImpl(const casadi::MXVector& x,
        const casadi::MXVector& xdot, casadi::MX& defects) const {
    // For more information, see doxygen documentation for the class.

    const int NS = m_problem.getNumStates();
    const int NP = m_problem.getNumParameters();
    for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
        // We enforce defect constraints on a mesh interval basis, so add
        // constraints until the number of mesh intervals is reached.
        const auto h = m_intervals(imesh);
        const auto x_i = x[imesh](Slice(), 0);
        const auto x_mid = x[imesh](Slice(), 1);
        const auto x_ip1 = x[imesh](Slice(), 2);
        const auto xdot_i = xdot[imesh](Slice(), 0);
        const auto xdot_mid = xdot[imesh](Slice(), 1);
        const auto xdot_ip1 = xdot[imesh](Slice(), 2);

        // Hermite interpolant defects.
        defects(Slice(0, NS), imesh) =
                x_mid - 0.5 * (x_ip1 + x_i) - (h / 8.0) * (xdot_i - xdot_ip1);

        // Simpson integration defects.
        defects(Slice(NS, 2*NS), imesh) =
                x_ip1 - x_i - (h / 6.0) * (xdot_ip1 + 4.0 * xdot_mid + xdot_i);
    }
}

void HermiteSimpson::calcInterpolatingControlsImpl(casadi::MX& controls) const {
    calcInterpolatingControlsHelper(controls);
}

void HermiteSimpson::calcInterpolatingControlsImpl(casadi::DM& controls) const {
    calcInterpolatingControlsHelper(controls);
}

Transcription::FlattenedVariableInfo 
HermiteSimpson::getFlattenedVariableInfo() const {
    FlattenedVariableInfo info;
    int N = m_numPointsPerMeshInterval - 1;
    int nx = 0;
    int nu = 0;
    for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
        int igrid = imesh * N;
        info.order.push_back({initial_time, imesh});
        info.order.push_back({final_time, imesh});
        info.order.push_back({parameters, imesh});
        info.order.push_back({states, igrid});
        info.order.push_back({states, igrid + 1});
        if (m_solver.getInterpolateControlMeshInteriorPoints()) {
            info.order.push_back({controls, igrid});
            info.order.push_back({multipliers, igrid});
            info.order.push_back({derivatives, igrid});
        } else {
            info.order.push_back({controls, igrid});
            info.order.push_back({controls, igrid + 1});
            info.order.push_back({multipliers, igrid});
            info.order.push_back({multipliers, igrid + 1});
            info.order.push_back({derivatives, igrid});
            info.order.push_back({derivatives, igrid + 1});
        }
        info.order.push_back({projection_states, imesh});
        info.order.push_back({slacks, imesh});
    }
    info.order.push_back({initial_time, m_numMeshIntervals});
    info.order.push_back({final_time, m_numMeshIntervals});
    info.order.push_back({parameters, m_numMeshIntervals});
    info.order.push_back({states, m_numGridPoints - 1});
    info.order.push_back({controls, m_numGridPoints - 1});
    info.order.push_back({multipliers, m_numGridPoints - 1});
    info.order.push_back({derivatives, m_numGridPoints - 1});

    return info;
}

} // namespace CasOC
