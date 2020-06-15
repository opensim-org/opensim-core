#ifndef TROPTER_OPTIMIZATION_PROBLEMDECORATOR_ADOUBLE_H
#define TROPTER_OPTIMIZATION_PROBLEMDECORATOR_ADOUBLE_H
// ----------------------------------------------------------------------------
// tropter: ProblemDecorator_adouble.h
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

namespace tropter {

struct SparsityCoordinates;

namespace optimization {

/// This specialization uses automatic differentiation (via ADOL-C) to
/// compute the derivatives of the objective and constraints.
/// @ingroup optimization
template<>
class Problem<adouble>::Decorator
        : public ProblemDecorator {
public:
    Decorator(const Problem<adouble>& problem);
    /// Delete memory allocated by ADOL-C.
    virtual ~Decorator();
    void calc_sparsity(const Eigen::VectorXd& variables,
            SparsityCoordinates& jacobian,
            bool provide_hessian_sparsity,
            SparsityCoordinates& hessian) const override;
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
    void calc_hessian_lagrangian(unsigned num_variables,
            const double* variables, bool new_variables, double obj_factor,
            unsigned num_constraints, const double* lambda, bool new_lambda,
            unsigned num_nonzeros, double* nonzeros) const override;
private:
    void trace_objective(short int tag,
            unsigned num_variables, const double* variables,
            double& obj_value) const;
    void trace_constraints(short int tag,
            unsigned num_variables, const double* variables,
            unsigned num_constraints, double* constr) const;
    void trace_lagrangian(short int tag,
            unsigned num_variables, const double* variables,
            const double& obj_factor,
            unsigned num_constraints, const double* lambda,
            double& lagrangian_value) const;

    const Problem<adouble>& m_problem;

    // ADOL-C
    // ------
    // TODO if we want to be able to solve multiple problems at once, these
    // cannot be static. We could create a registry of tags, and the tags can
    // be "checked out" and "returned."
    static const short int m_objective_tag   = 1;
    static const short int m_constraints_tag = 2;
    static const short int m_lagrangian_tag  = 3;

    // We must hold onto the sparsity pattern for the Jacobian and
    // Hessian so that we can pass them to subsequent calls to sparse_jac().
    // ADOL-C allocates this memory, but we must delete it.
    mutable int m_jacobian_num_nonzeros = -1;
    mutable unsigned int* m_jacobian_row_indices = nullptr;
    mutable unsigned int* m_jacobian_col_indices = nullptr;
    std::vector<int> m_sparse_jac_options;

    mutable int m_hessian_num_nonzeros = -1;
    mutable unsigned int* m_hessian_row_indices = nullptr;
    mutable unsigned int* m_hessian_col_indices = nullptr;
    // Working memory for lambda multipliers and the "obj_factor."
    mutable std::vector<double> m_hessian_obj_factor_lambda;
    std::vector<int> m_sparse_hess_options;
};

} // namespace optimization
} // namespace tropter

#endif // TROPTER_OPTIMIZATION_PROBLEMDECORATOR_ADOUBLE_H
