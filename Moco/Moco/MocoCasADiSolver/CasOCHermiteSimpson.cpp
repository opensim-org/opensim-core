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

    const int numMeshPoints = (m_numGridPoints + 1) / 2;
    const int numMeshIntervals = numMeshPoints - 1;
    // The duration of each mesh interval.
    const DM mesh = DM::linspace(0, 1, numMeshPoints);
    const DM meshIntervals = mesh(Slice(1, numMeshPoints)) -
        mesh(Slice(0, numMeshPoints - 1));
    // Simpson quadrature includes integrand evaluations at the midpoint.
    DM quadCoeffs(m_numGridPoints, 1);
    // Loop through each mesh interval and update the corresponding components
    // in the total coefficients vector.
    for (int i = 0; i < numMeshIntervals; ++i) {
        // The mesh interval quadrature coefficients overlap at the mesh grid 
        // points in the total coefficients vector, so we slice at every other 
        // index to update the coefficients vector.
        quadCoeffs(Slice(2*i)) += (1/6) * meshIntervals(i);
        quadCoeffs(Slice(2*i + 1)) += (2/3) * meshIntervals(i);
        quadCoeffs(Slice(2*i + 2)) += (1/6) * meshIntervals(i);
    }

    return quadCoeffs;
}

DM HermiteSimpson::createKinematicConstraintIndicesImpl() const {
    DM indices(m_numGridPoints, 1);
    for (int i = 0; i < m_numGridPoints; i += 2) {
        indices(Slice(i)) = 1;
    }
    return indices;
}

void HermiteSimpson::applyConstraintsImpl() {

    // We have arranged the code this way so that all constraints at a given
    // mesh point are grouped together (organizing the sparsity of the Jacobian
    // this way might have benefits for sparse linear algebra).
    const auto& states = m_vars[Var::states];
    const DM zero(m_problem.getNumStates(), 1);
    const int numMeshPoints = (m_numGridPoints + 1) / 2;
    const int numMeshIntervals = numMeshPoints - 1;

    int time_i, time_mid, time_ip1;
    for (int imesh = 0; imesh < numMeshPoints; ++imesh) {
        time_i = 2*imesh; // Needed for defects and path constraints.
        if (imesh < numMeshIntervals) {
            time_mid = 2*imesh + 1;
            time_ip1 = 2*imesh + 2;

            const auto h = m_times(time_ip1) - m_times(time_i);
            const auto x_i = states(Slice(), time_i);
            const auto x_mid = states(Slice(), time_mid);
            const auto x_ip1 = states(Slice(), time_ip1);
            const auto xdot_i = xdot(Slice(), time_i);
            const auto xdot_mid = xdot(Slice(), time_mid);
            const auto xdot_ip1 = xdot(Slice(), time_ip1);

            // Hermite interpolant defects
            addConstraints(zero, zero,
                x_mid - 0.5*(x_ip1 + x_i) - (h / 8.0) * (xdot_i - xdot_ip1));

            // Simpson integration defects
            addConstraints(zero, zero,
                x_ip1 - x_i - (h / 6.0) * (xdot_ip1 + 4.0*xdot_mid + xdot_i));
        }
        
        // TODO
        // if (m_problem.getNumKinematicConstraintEquations()) {
        //     addConstraints(kcLowerBounds, kcUpperBounds, qerr(Slice(), itime));
        // }
        for (const auto& pathInfo : m_problem.getPathConstraintInfos()) {
            const auto output = pathInfo.function->operator()(
            {m_times(time_i), m_vars[Var::states](Slice(), time_i),
                m_vars[Var::controls](Slice(), time_i),
                m_vars[Var::parameters]});
            const auto& errors = output.at(0);
            addConstraints(
                pathInfo.lowerBounds, pathInfo.upperBounds, errors);
        }
    }
}

} // namespace CasOC