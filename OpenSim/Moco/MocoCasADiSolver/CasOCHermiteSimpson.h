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
/// Position- and velocity-level kinematic constraint errors and path constraint 
/// errors are enforced only at the mesh points. In the kinematic constraint 
/// method by Bordalba et al. (2023) [2], the acceleration-level constraints are 
/// also enforced at the collocation points. In the kinematic constraint method 
/// by Posa et al. (2016) [3], the acceleration-level constraints are only enforced 
/// at the mesh points.
///
/// References
/// ----------
/// [1] Hargraves, C.R., Paris, S.W. "Direct Trajectory Optimization Using 
///     Nonlinear Programming and Collocation." Journal of Guidance, Control, 
///     and Dynamics (1987).
/// [2] Bordalba, Ricard, Tobias Schoels, Lluís Ros, Josep M. Porta, and
///     Moritz Diehl. "Direct collocation methods for trajectory optimization
///     in constrained robotic systems." IEEE Transactions on Robotics (2023).
/// [3] TODO Posa et al. 2016
class HermiteSimpson : public Transcription {
public:
    HermiteSimpson(const Solver& solver, const Problem& problem)
            : Transcription(solver, problem) {
        casadi::DM grid =
                casadi::DM::zeros(1, (2 * m_solver.getMesh().size()) - 1);
        const auto& mesh = m_solver.getMesh();
        for (int i = 0; i < grid.numel(); ++i) {
            if (i % 2 == 0) {
                grid(i) = mesh[i / 2];
            } else {
                grid(i) = .5 * (mesh[i / 2] + mesh[i / 2 + 1]);
            }
        }
        createVariablesAndSetBounds(grid, 2 * m_problem.getNumStates(), 3);
    }

private:
    casadi::DM createQuadratureCoefficientsImpl() const override;
    casadi::DM createMeshIndicesImpl() const override;
    casadi::DM createControlIndicesImpl() const override;
    void calcDefectsImpl(const casadi::MXVector& x, 
            const casadi::MXVector& xdot, casadi::MX& defects) const override;
    std::vector<std::pair<Var, int>> getVariableOrder() const override;
    void calcInterpolatingControlsImpl(casadi::MX& controls) const override;
    void calcInterpolatingControlsImpl(casadi::DM& controls) const override;

    template <typename T>
    void calcInterpolatingControlsHelper(T& controls) const {
        using casadi::Slice;

        // This control approximation scheme is based on the control scheme 
        // proposed by Hargraves and Paris (1987) [1]. Linear interpolation of 
        // controls is also recommended by Bordalba et al. (2023) [2].
        for (int imesh = 0; imesh < m_numMeshIntervals; ++imesh) {
            controls(Slice(), 2*imesh + 1) = 0.5 * (
                    controls(Slice(), 2*imesh) + 
                    controls(Slice(), 2*imesh + 2));
        }
    }
};

} // namespace CasOC

#endif // OPENSIM_CASOCHERMITESIMPSON_H
