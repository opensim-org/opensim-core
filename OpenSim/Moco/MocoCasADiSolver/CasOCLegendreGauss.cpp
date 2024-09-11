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

DM LegendreGauss::createControlIndicesImpl() const {
    DM indices = DM::zeros(1, m_numGridPoints);
    for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
        const int igrid = imesh * (m_degree + 1);
        for (int d = 0; d < m_degree; ++d) {
            indices(igrid + d + 1) = 1;
        }
    }
    return indices;
}

void LegendreGauss::calcDefectsImpl(const casadi::MXVector& x, 
        const casadi::MXVector& xdot, casadi::MX& defects) const {
    // For more information, see doxygen documentation for the class.

    const int NS = m_problem.getNumStates();
    const int NP = m_problem.getNumParameters();
    for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
        const int igrid = imesh * (m_degree + 1);
        const auto h = m_intervals(imesh);
        const auto x_i = x[imesh](Slice(), Slice(0, m_degree + 1));
        const auto xdot_i = xdot[imesh](Slice(), Slice(1, m_degree + 1));
        const auto x_ip1 = x[imesh](Slice(), m_degree + 1);

        // End state interpolation.
        defects(Slice(0, NS), imesh) =
                x_ip1 - MX::mtimes(x_i, m_interpolationCoefficients);

        // Residual function defects.
        MX residual = h * xdot_i - MX::mtimes(x_i, m_differentiationMatrix);
        for (int d = 0; d < m_degree; ++d) {
            const int istart = (d + 1) * NS;
            const int iend = (d + 2) * NS;
            defects(Slice(istart, iend), imesh) = residual(Slice(), d);
        }
    }
}

void LegendreGauss::calcInterpolatingControlsImpl(casadi::MX& controls) const {
    calcInterpolatingControlsHelper(controls);
}

void LegendreGauss::calcInterpolatingControlsImpl(casadi::DM& controls) const {
    calcInterpolatingControlsHelper(controls);
}

Transcription::FlattenedVariableInfo 
LegendreGauss::getFlattenedVariableInfo() const {
    FlattenedVariableInfo info;
    int N = m_numPointsPerMeshInterval - 1;
    int nx = 0;
    int nu = 0;
    bool lastState = false;
    for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
        int igrid = imesh * N;

        info.order.push_back({initial_time, imesh});
        info.order.push_back({final_time, imesh});
        nx += 2;
        
        info.order.push_back({parameters, imesh});
        nx += m_problem.getNumParameters();

        if (imesh && m_numProjectionStates) {
            info.order.push_back({projection_states, imesh - 1});
            nx += m_numProjectionStates;
            lastState = true;

            info.order.push_back({slacks, imesh - 1});
            nu += m_problem.getNumSlacks();
        }

        for (int i = 0; i < N; ++i) {
            info.order.push_back({states, igrid + i});
            if (lastState) {
                nu += m_problem.getNumStates();
            } else {
                nx += m_problem.getNumStates();
                lastState = true;
            }
        }

        if (m_solver.getInterpolateControlMeshInteriorPoints()) {
            for (int d = 0; d < m_degree; ++d) {
                info.order.push_back({controls, igrid + d + 1});
                nu += m_problem.getNumControls();
            }
            for (int d = 0; d < m_degree; ++d) {
                info.order.push_back({multipliers, igrid + d + 1});
                nu += m_problem.getNumMultipliers();
            }
            for (int d = 0; d < m_degree; ++d) {
                info.order.push_back({derivatives, igrid + d + 1});
                nu += m_problem.getNumDerivatives();
            }
        } else {
            for (int i = 0; i < N; ++i) {
                info.order.push_back({controls, igrid + i});
                nu += m_problem.getNumControls();
            }
            for (int i = 0; i < N; ++i) {
                info.order.push_back({multipliers, igrid + i});
                nu += m_problem.getNumMultipliers();
            }
            for (int i = 0; i < N; ++i) {
                info.order.push_back({derivatives, igrid + i});
                nu += m_problem.getNumDerivatives();
            }
        }

        info.nx.push_back(nx);
        info.nu.push_back(nu);
        nx = 0;
        nu = 0;
        lastState = false;
    }
    info.order.push_back({initial_time, m_numMeshIntervals});
    info.order.push_back({final_time, m_numMeshIntervals});
    nx += 2;

    info.order.push_back({parameters, m_numMeshIntervals});
    nx += m_problem.getNumParameters();

    if (m_numProjectionStates) {
        info.order.push_back({projection_states, m_numMeshIntervals - 1});
        nx += m_numProjectionStates;

        info.order.push_back({slacks, m_numMeshIntervals - 1});
        nu += m_problem.getNumSlacks();
    }

    info.order.push_back({states, m_numGridPoints - 1});
    if (m_numProjectionStates) {
        nu += m_problem.getNumStates();
    } else {
        nx += m_problem.getNumStates();
    }

    if (!m_solver.getInterpolateControlMeshInteriorPoints()) {
        info.order.push_back({controls, m_numGridPoints - 1});
        nu += m_problem.getNumControls();
        info.order.push_back({multipliers, m_numGridPoints - 1});
        nu += m_problem.getNumMultipliers();
        info.order.push_back({derivatives, m_numGridPoints - 1});
        nu += m_problem.getNumDerivatives();
    }

    info.nx.push_back(nx);
    info.nu.push_back(nu);

    return info;
}

} // namespace CasOC
