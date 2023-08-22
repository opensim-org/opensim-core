/* -------------------------------------------------------------------------- *
 * OpenSim: CasOCLegendreGaussRadau.cpp                                       *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2023 Stanford University and the Authors                     *
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
#include "CasOCLegendreGaussRadau.h"

using casadi::DM;
using casadi::MX;
using casadi::Slice;

namespace CasOC {

DM LegendreGaussRadau::createQuadratureCoefficientsImpl() const {

    // The duration of each mesh interval.
    const DM mesh(m_solver.getMesh());
    const DM meshIntervals = mesh(Slice(1, m_numMeshPoints)) -
                             mesh(Slice(0, m_numMeshPoints - 1));
    const DM w = m_quadratureCoefficients;

    // Loop through each mesh interval and update the corresponding
    // components in the total coefficients vector.
    DM quadCoeffs(m_numGridPoints, 1);
    for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
        const int igrid = imesh * m_degree;
        for (int d = 0; d < m_degree; ++d) {
            quadCoeffs(igrid + d + 1) += w(d) * meshIntervals(imesh);
        }
    }
    return quadCoeffs;
}

DM LegendreGaussRadau::createMeshIndicesImpl() const {
    DM indices = DM::zeros(1, m_numGridPoints);
    for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
        indices(imesh * m_degree) = 1;
    }
    indices(m_numGridPoints - 1) = 1;
    return indices;
}

void LegendreGaussRadau::calcDefectsImpl(const casadi::MX& x,
        const casadi::MX& xdot, casadi::MX& defects) const {
    // For more information, see doxygen documentation for the class.

    const int NS = m_problem.getNumStates();
    for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
        const int igrid = imesh * m_degree;
        const auto h = m_times(igrid + m_degree) - m_times(igrid);
        const auto x_i = x(Slice(), Slice(igrid, igrid + m_degree + 1));
        const auto xdot_i = xdot(Slice(),
                Slice(igrid + 1, igrid + m_degree + 1));

        // Residual function defects.
        MX residual = h * xdot_i - MX::mtimes(x_i, m_differentiationMatrix);
        for (int d = 0; d < m_degree; ++d) {
            defects(Slice(d * NS, (d + 1) * NS), imesh) = residual(Slice(), d);
        }
    }
}

void LegendreGaussRadau::calcInterpolatingControlsImpl(
        const casadi::MX& controls, casadi::MX& interpControls) const {
    if (m_problem.getNumControls() &&
            m_solver.getInterpolateControlMidpoints()) {
        for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
            const int igrid = imesh * m_degree;
            const auto c_i = controls(Slice(), igrid);
            const auto c_ip1 = controls(Slice(), igrid + m_degree);
            for (int d = 0; d < m_degree-1; ++d) {
                const auto c_t = controls(Slice(), igrid + d + 1);
                interpControls(Slice(), imesh * (m_degree - 1) + d) =
                        c_t - (m_legendreRoots[d] * (c_ip1 - c_i) + c_i);
            }
        }
    }
}

} // namespace CasOC