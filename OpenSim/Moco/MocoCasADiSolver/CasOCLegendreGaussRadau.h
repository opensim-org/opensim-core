#ifndef OPENSIM_CASOCLEGENDREGAUSSRADAU_H
#define OPENSIM_CASOCLEGENDREGAUSSRADAU_H
/* -------------------------------------------------------------------------- *
 * OpenSim: CasOCLegendreGaussRadau.h                                         *
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

/// Enforce the differential equations in the problem using pseudospectral
/// transcription with Legendre-Gauss-Radau (LGR) collocation points. This
/// method is sometimes referred to as the Radau Pseudospectral Method (GPM)
/// [1, 2]. This implementation supports Lagrange polynomials of degree within
/// the range [1, 9]. The number of collocation points per mesh interval is
/// equal to the degree of the Lagrange polynomials, where one collocation point
/// is at the end of the mesh interval and the remaining points lie in the mesh
/// interval interior. The integral in the objective function is approximated
/// using the Gauss weights associated with these points.
///
/// Defect constraints.
/// -------------------
/// For each state variable, there is a set of defect constraints equal to the
/// number of LGR collocation points in each mesh interval.
///
/// Control approximation.
/// ----------------------
/// We use the control approximation strategy from Bordalba et al. [3], where
/// control values are linearly interpolated between mesh and collocation points,
/// due to its simplicity and ease of implementation within the existing
/// CasOCTranscription framework.
///
/// Kinematic constraints and path constraints.
/// -------------------------------------------
/// Position- and velocity-level kinematic constraint errors and path constraint 
/// errors are enforced only at the mesh points. In the kinematic constraint 
/// method by Bordalba et al. [3], the acceleration-level constraints are also
/// enforced at the collocation points.
///
/// References
/// ----------
/// [1] Benson, David. "A Gauss pseudospectral transcription for optimal
///     control." PhD diss., Massachusetts Institute of Technology, 2005.
/// [2] Huntington, Geoffrey Todd. "Advancement and analysis of a Gauss
///     pseudospectral transcription for optimal control problems." PhD diss.,
///     Massachusetts Institute of Technology, Department of Aeronautics and
///     Astronautics, 2007.
/// [3] Bordalba, Ricard, Tobias Schoels, Lluís Ros, Josep M. Porta, and
///     Moritz Diehl. "Direct collocation methods for trajectory optimization
///     in constrained robotic systems." IEEE Transactions on Robotics (2023).
///
class LegendreGaussRadau : public Transcription {
public:
    LegendreGaussRadau(const Solver& solver, const Problem& problem, int degree)
            : Transcription(solver, problem), m_degree(degree) {
        const auto& mesh = m_solver.getMesh();
        const int numMeshIntervals = (int)mesh.size() - 1;
        const int numGridPoints =
                (int)mesh.size() + numMeshIntervals * (m_degree - 1);
        casadi::DM grid = casadi::DM::zeros(1, numGridPoints);

        // Get the collocation points (roots of Legendre polynomials). The roots
        // are returned on the interval (0, 1], not (-1, 1] as in the theses of
        // Benson and Huntington. Note that the range (0, 1] includes points on
        // the interior of the mesh interval plus one collocation point at the
        // mesh interval endpoint.
        m_legendreRoots = casadi::collocation_points(m_degree, "radau");
        casadi::collocation_coeff(m_legendreRoots,
                m_differentiationMatrix,
                m_interpolationCoefficients,
                m_quadratureCoefficients);

        // Create the grid points.
        for (int imesh = 0; imesh < numMeshIntervals; ++imesh) {
            const double t_i = mesh[imesh];
            const double t_ip1 = mesh[imesh + 1];
            int igrid = imesh * m_degree;
            grid(igrid) = t_i;
            grid(igrid + m_degree) = t_ip1;
            for (int d = 0; d < m_degree-1; ++d) {
                grid(igrid + d + 1) = t_i + (t_ip1 - t_i) * m_legendreRoots[d];
            }
        }
        grid(numGridPoints - 1) = mesh[numMeshIntervals];

        createVariablesAndSetBounds(grid, m_degree * m_problem.getNumStates(),
                m_degree + 1);
    }

private:
    casadi::DM createQuadratureCoefficientsImpl() const override;
    casadi::DM createMeshIndicesImpl() const override;
    casadi::DM createControlIndicesImpl() const override;
    void calcDefectsImpl(const casadi::MXVector& x, 
            const casadi::MXVector& xdot, casadi::MX& defects) const override;
    void calcInterpolatingControlsImpl(casadi::MX& controls) const override;
    void calcInterpolatingControlsImpl(casadi::DM& controls) const override;
    std::vector<std::pair<Var, int>> getVariableOrder() const override;

    int m_degree;
    std::vector<double> m_legendreRoots;
    casadi::DM m_differentiationMatrix;
    casadi::DM m_interpolationCoefficients;
    casadi::DM m_quadratureCoefficients;

    template <typename T>
    void calcInterpolatingControlsHelper(T& controls) const {
        using casadi::Slice;

        // This interpolation scheme is based on control approximation defined 
        // by Eq. 6.16 in the thesis of Huntington [2].

        // Evaluate the `i`th Lagrange polynomial at the point `tau`.
        auto getLagrangePolynomial = [this](int i, double tau) -> double {
            double polynomial = 1.0;
            for (int d = 0; d < m_degree; ++d) {
                if (i != d) {
                    polynomial *= (tau - m_legendreRoots[d]) /
                                (m_legendreRoots[i] - m_legendreRoots[d]);
                }
            }
            return polynomial;
        };

        // The Legendre-Gauss-Radau points include a collocation point at the 
        // final mesh point for all mesh intervals. Therefore, we only need to
        // interpolate the control at the first mesh point.
        controls(Slice(), 0) = 0;
        for (int d = 0; d < m_degree; ++d) {
            const auto c_t = controls(Slice(), d + 1);
            controls(Slice(), 0) += getLagrangePolynomial(d, 0) * c_t;
        }
    }
};

} // namespace CasOC

#endif // OPENSIM_CASOCLEGENDREGAUSSRADAU_H
