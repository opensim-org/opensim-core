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

DM Trapezoidal::createKinematicConstraintIndicesImpl() const {
    return DM::ones(1, m_numGridPoints);
}

DM Trapezoidal::createResidualConstraintIndicesImpl() const {
    return DM::ones(1, m_numGridPoints);
}

void Trapezoidal::applyConstraintsImpl() {

    // We have arranged the code this way so that all constraints at a given
    // mesh point are grouped together (organizing the sparsity of the Jacobian
    // this way might have benefits for sparse linear algebra).
    const auto& states = m_vars[Var::states];
    const DM zeroS = casadi::DM::zeros(m_problem.getNumStates(), 1);
    const DM zeroU = casadi::DM::zeros(m_problem.getNumSpeeds(), 1);
    for (int itime = 0; itime < m_numGridPoints; ++itime) {
        if (itime > 0) {
            const auto h = m_times(itime) - m_times(itime - 1);
            const auto x_i = states(Slice(), itime);
            const auto x_im1 = states(Slice(), itime - 1);
            const auto xdot_i = m_xdot(Slice(), itime);
            const auto xdot_im1 = m_xdot(Slice(), itime - 1);

            // Trapezoidal defects.
            addConstraints(zeroS, zeroS,
                x_i - (x_im1 + 0.5 * h * (xdot_i + xdot_im1)));
        }

        if (m_solver.isDynamicsModeImplicit()) {
            addConstraints(zeroU, zeroU, m_residual(Slice(), itime));
        }

        // Kinematic constraint errors.
        if (m_problem.getNumKinematicConstraintEquations()) {
            DM kinConZero(m_problem.getNumKinematicConstraintEquations(), 1);
            addConstraints(kinConZero, kinConZero, m_pvaerr(Slice(), itime));
        }

        // The individual path constraint functions are passed to CasADi to
        // maximize CasADi's ability to take derivatives efficiently.
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
