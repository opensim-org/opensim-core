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

    // Calculate the number of mesh points based on the number of grid points.
    const int numMeshPoints = (m_numGridPoints + 1) / 2;
    const int numMeshIntervals = numMeshPoints - 1;
    // The duration of each mesh interval.
    const DM mesh = DM::linspace(0, 1, numMeshIntervals);
    const DM meshIntervals = mesh(Slice(1, numMeshPoints)) - 
        mesh(Slice(0, numMeshPoints - 1));
    // Simpson quadrature includes integrand evaluations at the midpoint.
    DM quadCoeffs(m_numGridPoints);

    // Loop through each mesh interval and update the corresponding components
    // in the total coefficients vector.
    for (int imesh = 0; imesh < numMeshIntervals; ++imesh) {
        // The mesh interval coefficients overlap at the mesh grid points in the
        // total coefficients vector, so we slice at every other index to update
        // the coefficients vector.
        quadCoeffs(Slice(2*imesh, 1)) += meshIntervals(imesh) * (1.0/6.0);
        quadCoeffs(Slice(2*imesh+1, 1)) += meshIntervals(imesh) * (2.0/3.0);
        quadCoeffs(Slice(2*imesh+2, 1)) += meshIntervals(imesh) * (1.0/6.0);
    }

    return quadCoeffs;
}

void HermiteSimpson::applyConstraintsImpl() {

    // We have arranged the code this way so that all constraints at a given
    // mesh point are grouped together (organizing the sparsity of the Jacobian
    // this way might have benefits for sparse linear algebra).
    const auto& states = m_vars[Var::states];
    const DM zero(m_problem.getNumStates(), 1);
    for (int itime = 0; itime < m_numGridPoints; ++itime) {
        if (itime > 0) {
            const auto h = m_times(itime) - m_times(itime - 1);
            const auto x_i = states(Slice(), itime);
            const auto x_im1 = states(Slice(), itime - 1);
            const auto xdot_i = xdot(Slice(), itime);
            const auto xdot_im1 = xdot(Slice(), itime - 1);
            addConstraints(zero, zero,
                x_i - (x_im1 + 0.5 * h * (xdot_i + xdot_im1)));
        }

        // TODO
        // if (m_problem.getNumKinematicConstraintEquations()) {
        //     addConstraints(kcLowerBounds, kcUpperBounds, qerr(Slice(), itime));
        // }
        for (const auto& pathInfo : m_problem.getPathConstraintInfos()) {
            const auto output = pathInfo.function->operator()(
            {m_times(itime), m_vars[Var::states](Slice(), itime),
                m_vars[Var::controls](Slice(), itime),
                m_vars[Var::parameters]});
            const auto& errors = output.at(0);
            addConstraints(
                pathInfo.lowerBounds, pathInfo.upperBounds, errors);
        }
    }
}

} // namespace CasOC