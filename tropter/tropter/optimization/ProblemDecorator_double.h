#ifndef TROPTER_OPTIMIZATION_PROBLEMDECORATOR_DOUBLE_H
#define TROPTER_OPTIMIZATION_PROBLEMDECORATOR_DOUBLE_H
// ----------------------------------------------------------------------------
// tropter: ProblemDecorator_double.h
// ----------------------------------------------------------------------------
// Copyright (c) 2017 tropter authors
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may
// not use this file except in compliance with the License. You may obtain a
// copy of the License at http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------

#include "Problem.h"
#include "ProblemDecorator.h"

#include <tropter/SparsityPattern.h>

namespace tropter {

namespace optimization {

class JacobianColoring;
class HessianColoring;

/// @ingroup optimization
/// The gradient, Jacobian, and Hessian are computed in a way that exploits
/// sparsity, using ColPack and graph coloring algorithms [1].
/// [1] Gebremedhin, Assefaw Hadish, Fredrik Manne, and Alex Pothen. "What color
/// is your Jacobian? Graph coloring for computing derivatives." SIAM review
/// 47.4 (2005): 629-705.
template<>
class Problem<double>::Decorator
        : public ProblemDecorator {
public:
    Decorator(const Problem<double>& problem);
    ~Decorator();
    void calc_sparsity(const Eigen::VectorXd& variables,
            SparsityCoordinates& jacobian_sparsity,
            bool provide_hessian_sparsity,
            SparsityCoordinates& hessian_sparsity) const override;
    void calc_objective(unsigned num_variables, const double* variables,
            bool new_variables,
            double& obj_value) const override;
    void calc_constraints(unsigned num_variables, const double* variables,
            bool new_variables,
            unsigned num_constraints, double* constr) const override;
    void calc_gradient(unsigned num_variables, const double* variables,
            bool new_variables,
            double* grad) const override;
    void calc_jacobian(unsigned num_variables, const double* variables,
            bool new_variables,
            unsigned num_nonzeros, double* nonzeros) const override;
    /// The Hessian is computed in a way that exploits sparsity and requires
    /// as few perturbations of the objective and constraint functions as
    /// possible. This algorithm was taken from Algorithm 9.8 on page 294 of
    /// [1].
    ///
    /// [1] Bohme TJ, Frank B. Hybrid Systems, Optimal Control and Hybrid
    /// Vehicles: Theory, Methods and Applications. Springer 2017.
    void calc_hessian_lagrangian(unsigned num_variables,
            const double* variables,
            bool new_variables, double obj_factor,
            unsigned num_constraints, const double* lambda,
            bool new_lambda,
            unsigned num_nonzeros, double* nonzeros) const override;
private:

    void calc_sparsity_hessian_lagrangian(
            const Eigen::VectorXd&, SparsityCoordinates&) const;

    void calc_hessian_objective(const Eigen::VectorXd& x0,
            Eigen::VectorXd& hesobj_values) const;
    void calc_lagrangian(
            const Eigen::VectorXd& variables,
            double obj_factor,
            // TODO use Eigen::Ref?
            const Eigen::Map<const Eigen::VectorXd>& lambda,
            double& lagrangian_value) const;

    const Problem<double>& m_problem;

    // Working memory shared by multiple functions.
    mutable Eigen::VectorXd m_x_working;

    // mutable double m_time_hescon = 0;
    // mutable double m_time_hesobj = 0;

    // Gradient.
    // ---------
    // The indices of the variables used in the objective function
    // (conservative estimate of the indicies of the gradient that are nonzero).
    mutable std::vector<unsigned int> m_gradient_nonzero_indices;

    // Jacobian.
    // ---------
    // This class (a) determines the directions in which to perturb
    // the variables to compute the Jacobian and (b) recovers the sparse
    // Jacobian (to pass to the optimization solver) after computing finite
    // differences.
    mutable std::unique_ptr<JacobianColoring> m_jacobian_coloring;
    // Working memory.
    // TODO this could be a column vector unless we are using parallelization.
    mutable Eigen::VectorXd m_constr_pos;
    mutable Eigen::VectorXd m_constr_neg;
    mutable Eigen::MatrixXd m_jacobian_compressed;

    // Hessian/Lagrangian.
    // -------------------
    mutable std::unique_ptr<HessianColoring> m_hescon_coloring;
    mutable std::unique_ptr<HessianColoring> m_hesobj_coloring;
    mutable std::unique_ptr<HessianColoring> m_hessian_coloring;
    // TODO temporary until we use ColPack.
    mutable SparsityCoordinates m_hesobj_indices;
    // Only set if using the slow Hessian approximation.
    mutable SparsityCoordinates m_hessian_indices;
    // Working memory.
    // mutable Eigen::VectorXd m_constr_working;
    mutable Eigen::Matrix<bool, Eigen::Dynamic, 1>
            m_perturbed_objective_is_cached;
    mutable Eigen::VectorXd m_perturbed_objective_cache;

    // Deprecated.
    void calc_hessian_lagrangian_slow(unsigned num_variables,
            const double* variables,
            bool new_variables, double obj_factor,
            unsigned num_constraints, const double* lambda,
            bool new_lambda,
            unsigned num_nonzeros, double* nonzeros) const;
};

} // namespace optimization
} // namespace tropter

#endif // TROPTER_OPTIMIZATION_PROBLEMDECORATOR_DOUBLE_H
