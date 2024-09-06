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

DM Trapezoidal::createQuadratureCoefficientsImpl() const {

    // For trapezoidal rule, grid points and mesh points are synonymous.
    const int numMeshPoints = m_numGridPoints;
    const DM meshIntervals = m_grid(Slice(1, numMeshPoints)) -
                             m_grid(Slice(0, numMeshPoints - 1));
    DM quadCoeffs(numMeshPoints, 1);
    quadCoeffs(Slice(0, numMeshPoints - 1)) = 0.5 * meshIntervals;
    quadCoeffs(Slice(1, numMeshPoints)) += 0.5 * meshIntervals;

    return quadCoeffs;
}

DM Trapezoidal::createMeshIndicesImpl() const {
    return DM::ones(1, m_numGridPoints);
}

DM Trapezoidal::createControlIndicesImpl() const {
    return DM::ones(1, m_numGridPoints);
}

void Trapezoidal::calcDefectsImpl(const casadi::MXVector& x, 
        const casadi::MXVector& xdot, casadi::MX& defects) const {

    // We have arranged the code this way so that all constraints at a given
    // mesh point are grouped together (organizing the sparsity of the Jacobian
    // this way might have benefits for sparse linear algebra).
    const int NS = m_problem.getNumStates();
    const int NP = m_problem.getNumParameters();
    for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
        const auto h = m_intervals(imesh);
        const auto x_i = x[imesh](Slice(), 0);
        const auto x_ip1 = x[imesh](Slice(), 1);
        const auto xdot_i = xdot[imesh](Slice(), 0);
        const auto xdot_ip1 = xdot[imesh](Slice(), 1);

        // Trapezoidal defects.
        defects(Slice(0, NS), imesh) =
                x_ip1 - (x_i + 0.5 * h * (xdot_ip1 + xdot_i));
    }
}

std::vector<std::pair<Var, int>> Trapezoidal::getVariableOrder() const {
    std::vector<std::pair<Var, int>> order;
    for (int imesh = 0; imesh < m_numMeshPoints; ++imesh) {
        order.push_back({initial_time, imesh});
        order.push_back({final_time, imesh});
        order.push_back({parameters, imesh});
        order.push_back({states, imesh});
        order.push_back({controls, imesh});
        order.push_back({multipliers, imesh});
        order.push_back({derivatives, imesh});
    }

    return order;
}

} // namespace CasOC
