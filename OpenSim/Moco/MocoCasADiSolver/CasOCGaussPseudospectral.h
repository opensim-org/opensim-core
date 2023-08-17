#ifndef OPENSIM_CASOCGAUSSPSEUDOSPECTRAL_H
#define OPENSIM_CASOCGAUSSPSEUDOSPECTRAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: CasOCGaussPseudospectral.h                                        *
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

#include "CasOCTranscription.h"

namespace CasOC {

class GaussPseudospectral : public Transcription {
public:
    GaussPseudospectral(const Solver& solver, const Problem& problem, int degree)
            : Transcription(solver, problem), m_degree(degree) {
        const auto& mesh = m_solver.getMesh();
        const int numMeshIntervals = (int)mesh.size() - 1;
        const int numGridPoints = (int)mesh.size() + numMeshIntervals * m_degree;
        casadi::DM grid = casadi::DM::zeros(1, numGridPoints);
        const bool interpControls = m_solver.getInterpolateControlMidpoints();
        casadi::DM pointsForInterpControls;
        if (interpControls) {
            pointsForInterpControls = casadi::DM::zeros(1,
                    numMeshIntervals * m_degree);
        }

        // Create the grid points.
        const auto legroots = createLegendrePolynomialRoots(m_degree);
        for (int imesh = 0; imesh < numMeshIntervals; ++imesh) {
            const double t_i = mesh[imesh];
            const double t_ip1 = mesh[imesh + 1];
            const double h = t_ip1 - t_i;
            int igrid = imesh * (m_degree + 1);
            grid(igrid) = t_i;
            for (int d = 0; d < m_degree; ++d) {
                const auto root = legroots(d);
                grid(igrid + d + 1) = t_i + .5 * h * (1 + legroots(d));
                if (interpControls) {
                    pointsForInterpControls(imesh * m_degree + d) =
                            grid(igrid + d + 1);
                }
            }
        }
        grid(numGridPoints - 1) = mesh[numMeshIntervals];
    }

private:
    casadi::DM createQuadratureCoefficientsImpl() const override;
    casadi::DM createMeshIndicesImpl() const override;
    void calcDefectsImpl(const casadi::MX& x, const casadi::MX& xdot,
            casadi::MX& defects) const override;
    void calcInterpolatingControlsImpl(const casadi::MX& controls,
            casadi::MX& interpControls) const override;

    casadi::DM createLegendrePolynomialRoots(int degree) const;
    int m_degree;
    casadi::DM m_diffMatrix;
};


} // namespace CasOC

#endif // OPENSIM_CASOCGAUSSPSEUDOSPECTRAL_H
