/* -------------------------------------------------------------------------- *
 * OpenSim: CasOCLegendreGauss.cpp                                            *
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
#include "CasOCLegendreGauss.h"

using casadi::DM;
using casadi::MX;
using casadi::Slice;

namespace CasOC {

DM LegendreGauss::createQuadratureCoefficientsImpl() const {

    // The duration of each mesh interval.
    const DM mesh(m_solver.getMesh());
    const DM meshIntervals = mesh(Slice(1, m_numMeshPoints)) -
                             mesh(Slice(0, m_numMeshPoints - 1));
    const DM w = m_quadratureCoefficients;

    // Loop through each mesh interval and update the corresponding
    // components in the total coefficients vector.
    DM quadCoeffs(m_numGridPoints, 1);
    for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
        const int igrid = imesh * (m_degree + 1);
        // There are no quadrature coefficients at the mesh points (i.e.,
        // quadCoeffs(igrid) = 0).
        for (int d = 0; d < m_degree; ++d) {
            quadCoeffs(igrid + d + 1) += w(d) * meshIntervals(imesh);
        }
    }
    return quadCoeffs;
}

DM LegendreGauss::createMeshIndicesImpl() const {
    DM indices = DM::zeros(1, m_numGridPoints);
    for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
        indices(imesh * (m_degree + 1)) = 1;
    }
    indices(m_numGridPoints - 1) = 1;
    return indices;
}

void LegendreGauss::calcDefectsImpl(const casadi::MX& x, const casadi::MX& xdot,
        const casadi::MX& ti, const casadi::MX& tf, const casadi::MX& p,
        casadi::MX& defects) const {
    // For more information, see doxygen documentation for the class.

    const int NP = m_problem.getNumParameters();
    const int NS = m_problem.getNumStates();
    for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
        const int i = imesh * (m_degree + 1);
        const int ip1 = i + m_degree + 1;
        const auto h = m_delta_t(imesh);
        const auto x_i = x(Slice(), Slice(i, ip1));
        const auto xdot_i = xdot(Slice(), Slice(i + 1, ip1));
        const auto x_ip1 = x(Slice(), ip1);

        // End state interpolation.
        defects(Slice(0, NS), imesh) =
                x_ip1 - MX::mtimes(x_i, m_interpolationCoefficients);

        // Initial time.
        defects(Slice(NS, NS + 1), imesh) = ti(imesh + 1) - ti(imesh);

        // Final time.
        defects(Slice(NS + 1, NS + 2), imesh) = tf(imesh + 1) - tf(imesh);

        // Parameters.
        if (NP) {
            const auto p_i = p(Slice(), imesh);
            const auto p_ip1 = p(Slice(), imesh + 1);
            defects(Slice(NS + 2, NS + 2 + NP), imesh) = p_ip1 - p_i;
        }

        // Residual function defects.
        MX residual = h * xdot_i - MX::mtimes(x_i, m_differentiationMatrix);
        for (int d = 0; d < m_degree; ++d) {
            const int istart = (d + 1) * NS + 2 + NP;
            const int iend = (d + 2) * NS + 2 + NP;
            defects(Slice(istart, iend), imesh) = residual(Slice(), d);
        }
    }
}

void LegendreGauss::calcInterpolatingControlsImpl(
        const casadi::MX& controls, casadi::MX& interpControls) const {
    if (m_problem.getNumControls() &&
            m_solver.getInterpolateControlMidpoints()) {
        for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
            const int igrid = imesh * (m_degree + 1);
            const auto c_i = controls(Slice(), igrid);
            const auto c_ip1 = controls(Slice(), igrid + m_degree + 1);
            for (int d = 0; d < m_degree; ++d) {
                const auto c_t = controls(Slice(), igrid + d + 1);
                interpControls(Slice(), imesh * m_degree + d) =
                        c_t - (m_legendreRoots[d] * (c_ip1 - c_i) + c_i);
            }
        }
    }
}

// void LegendreGauss::calcExtrapolatedControlsImpl(casadi::MX& controls) const {
//     if (m_problem.getNumControls()) {
//         // TODO
//         // for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
//         //     const int igrid = imesh * (m_degree + 1);
//         //     const auto c_i = controls(Slice(), igrid);
//         //     const auto c_ip1 = controls(Slice(), igrid + m_degree + 1);
//         //     for (int d = 0; d < m_degree; ++d) {
//         //         const auto c_t = controls(Slice(), igrid + d + 1);
//         //         interpControls(Slice(), imesh * m_degree + d) =
//         //                 c_t - (m_legendreRoots[d] * (c_ip1 - c_i) + c_i);
//         //     }
//         // }
//     }
// }

} // namespace CasOC