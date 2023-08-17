/* -------------------------------------------------------------------------- *
 * OpenSim: CasOCGaussPseudospectral.cpp                                        *
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
#include "CasOCGaussPseudospectral.h"
#include <Eigen/Dense>

using casadi::DM;
using casadi::MX;
using casadi::Slice;

namespace CasOC {

DM GaussPseudospectral::createMeshIndicesImpl() const {
    DM indices = DM::zeros(1, m_numGridPoints);
    for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
        const int igrid = imesh * (m_degree + 1);
        indices(igrid) = 1;
    }
    indices(m_numGridPoints - 1) = 1;
    return indices;
}


void GaussPseudospectral::calcInterpolatingControlsImpl(
        const casadi::MX& controls, casadi::MX& interpControls) const {
    if (m_problem.getNumControls() &&
            m_solver.getInterpolateControlMidpoints()) {
        // TODO based on Bordalba et al. (2023).
        for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
            const int igrid = imesh * (m_degree + 1);
            const auto t_i = m_times(igrid);
            const auto t_ip1 = m_times(igrid + m_degree + 1);
            const auto h = t_ip1 - t_i;
            const auto c_i = controls(Slice(), igrid);
            const auto c_ip1 = controls(Slice(), igrid + m_degree + 1);
            for (int d = 0; d < m_degree; ++d) {
                const auto t = m_times(igrid + d + 1);
                const auto c_t = controls(Slice(), igrid + d + 1);
                interpControls(Slice(), imesh * m_degree + d) =
                        c_i + (c_ip1 - c_i) * (t - t_i) / h - c_t;
            }
        }
    }
}

DM GaussPseudospectral::createLegendrePolynomialRoots(int degree) const {
    // TODO based on the Benson thesis.

    // Create indices.
    std::vector<int> n(degree-1);
    casadi::linspace(n, 1, degree-1);

    // Create the subdiagonals.
    std::vector<double> d(degree-1);
    for (int i = 0; i < degree-1; ++i) {
        d[i] = n[i] / sqrt(4 * pow(n[i],2) - 1);
    }

    // Create the Jacobi matrix.
    Eigen::MatrixXd matrix(degree, degree);
    for (int i = 0; i < degree; ++i) {
        for (int j = 0; j < degree; ++j) {
            if (i == j) {
                matrix(i,j) = 0;
            } else if (i == j+1) {
                matrix(i,j) = d[j];
            } else if (i == j-1) {
                matrix(i,j) = d[i];
            } else {
                matrix(i,j) = 0;
            }
        }
    }

    // Compute the eigenvalues.
    Eigen::EigenSolver<Eigen::MatrixXd> solver(matrix);
    Eigen::VectorXd eigenvalues = solver.eigenvalues().real();
    std::vector<double> stdVector(eigenvalues.data(),
            eigenvalues.data() + eigenvalues.size());
    std::sort(stdVector.begin(), stdVector.end());

    // Return the roots.
    DM roots = DM::zeros(1, degree);
    for (int i = 0; i < degree; ++i) {
        roots(i) = stdVector[i];
    }
    return roots;
}


} // namespace CasOC