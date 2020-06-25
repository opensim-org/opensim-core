#ifndef OPENSIM_CASOCHERMITESIMPSON_H
#define OPENSIM_CASOCHERMITESIMPSON_H
/* -------------------------------------------------------------------------- *
 * OpenSim: CasOCHermiteSimpson.h                                             *
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

#include "CasOCTranscription.h"

namespace CasOC {

/// Enforce the differential equations in the problem using a Hermite-
/// Simpson (third-order) approximation. The integral in the objective
/// function is approximated by Simpson quadrature.
///
/// Defect constraints.
/// -------------------
/// For each state variable, there is one pair of defect constraints
/// (Hermite interpolant defect + Simpson integration defect) per mesh
/// interval. Each mesh interval includes two mesh points (at the interval's
/// endpoints) and an additional collocation point at the mesh interval
/// midpoint. All three mesh interval points (2 mesh points + 1 collocation
/// point) are used to construct the defects.
///
/// Kinematic constraints and path constraints.
/// -------------------------------------------
/// Kinematic constraint and path constraint errors are enforced only at the
/// mesh points. Errors at collocation points at the mesh interval midpoint
/// are ignored.
class HermiteSimpson : public Transcription {
public:
    HermiteSimpson(const Solver& solver, const Problem& problem)
            : Transcription(solver, problem) {
        casadi::DM grid =
                casadi::DM::zeros(1, (2 * m_solver.getMesh().size()) - 1);
        const auto& mesh = m_solver.getMesh();
        const bool interpControls = m_solver.getInterpolateControlMidpoints();
        casadi::DM pointsForInterpControls;
        if (interpControls) {
            pointsForInterpControls =
                    casadi::DM::zeros(1, m_solver.getMesh().size() - 1);
        }
        for (int i = 0; i < grid.numel(); ++i) {
            if (i % 2 == 0) {
                grid(i) = mesh[i / 2];
            } else {
                grid(i) = .5 * (mesh[i / 2] + mesh[i / 2 + 1]);
                if (interpControls) {
                    pointsForInterpControls(i / 2) = grid(i);
                }
            }
        }
        createVariablesAndSetBounds(grid, 2 * m_problem.getNumStates(),
                pointsForInterpControls);
    }

private:
    casadi::DM createQuadratureCoefficientsImpl() const override;
    casadi::DM createMeshIndicesImpl() const override;
    void calcDefectsImpl(const casadi::MX& x, const casadi::MX& xdot,
            casadi::MX& defects) const override;
    void calcInterpolatingControlsImpl(const casadi::MX& controls,
            casadi::MX& interpControls) const override;
};

} // namespace CasOC

#endif // OPENSIM_CASOCHERMITESIMPSON_H
