#ifndef TROPTER_OPTIMIZATIONPROBLEMDECORATOR_H
#define TROPTER_OPTIMIZATIONPROBLEMDECORATOR_H
// ----------------------------------------------------------------------------
// tropter: OptimizationProblemDecorator.h
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

#include <tropter/common.h>
#include <tropter/utilities.h>

#include "AbstractOptimizationProblem.h"

namespace tropter {

/// This class provides an interface of the OptimizationProblem to the
/// OptimizationSolvers.
/// In general, users do not use this class directly.
/// There is a derived type for each scalar type, and these derived classes
/// compute the gradient, Jacobian, and Hessian (using either finite differences
/// or automatic differentiation).
/// @ingroup optimization
// TODO alternative name: OptimizationProblemDelegate
class OptimizationProblemDecorator {
public:
    OptimizationProblemDecorator(const AbstractOptimizationProblem& problem)
            : m_problem(problem) {}
    unsigned get_num_variables() const
    {   return m_problem.get_num_variables(); }
    unsigned get_num_constraints() const
    {   return m_problem.get_num_constraints(); }
    const Eigen::VectorXd& get_variable_lower_bounds() const
    {   return m_problem.get_variable_lower_bounds(); }
    const Eigen::VectorXd& get_variable_upper_bounds() const
    {   return m_problem.get_variable_upper_bounds(); }
    const Eigen::VectorXd& get_constraint_lower_bounds() const
    {   return m_problem.get_constraint_lower_bounds(); }
    const Eigen::VectorXd& get_constraint_upper_bounds() const
    {   return m_problem.get_constraint_upper_bounds(); }
    /// Create an initial guess for this problem according to the
    /// following rules:
    ///   - unconstrained variable: 0.
    ///   - lower and upper bounds: midpoint of the bounds.
    ///   - only one bound: value of the bound.
    // TODO move to AbstractOptimizationProblem.
    Eigen::VectorXd make_initial_guess_from_bounds() const;
    // TODO document
    Eigen::VectorXd make_random_iterate_within_bounds() const;
    // TODO b/c of SNOPT, want to be able to ask for sparsity separately.
    // You must call this function first before calling objective(),
    // constraints(), etc.
    virtual void calc_sparsity(const Eigen::VectorXd& variables,
            std::vector<unsigned int>& jacobian_row_indices,
            std::vector<unsigned int>& jacobian_col_indices,
            bool provide_hessian_indices,
            std::vector<unsigned int>& hessian_row_indices,
            std::vector<unsigned int>& hessian_col_indices) const = 0;
    virtual void calc_objective(unsigned num_variables, const double* variables,
            bool new_variables,
            double& obj_value) const = 0;
    virtual void calc_constraints(unsigned num_variables,
            const double* variables,
            bool new_variables,
            unsigned num_constraints, double* constr) const = 0;
    virtual void calc_gradient(unsigned num_variables, const double* variables,
            bool new_variables,
            double* grad) const = 0;
    virtual void calc_jacobian(unsigned num_variables, const double* variables,
            bool new_variables,
            unsigned num_nonzeros, double* nonzeros) const = 0;
    virtual void calc_hessian_lagrangian(
            unsigned num_variables, const double* variables, bool new_variables,
            double obj_factor,
            unsigned num_constraints, const double* lambda, bool new_lambda,
            unsigned num_nonzeros, double* nonzeros) const = 0;

    /// 0 for silent, 1 for verbose.
    void set_verbosity(int verbosity);
    /// @copydoc set_verbosity()
    int get_verbosity() const;

    /// @name Options for finite differences
    /// These options are only used when the scalar type is double.
    /// @{

    /// The finite difference step size used when approximating the Hessian.
    /// (default: 1e-5, based on [1] section 9.2.4.4).
    /// [1] Bohme TJ, Frank B. Hybrid Systems, Optimal Control and Hybrid
    /// Vehicles: Theory, Methods and Applications. Springer 2017.
    void set_findiff_hessian_step_size(double value);
    ///  - "fast": default
    ///  - "slow": Slower mode to be used only for debugging.
    void set_findiff_hessian_mode(const std::string& setting);

    /// @copydoc set_findiff_hessian_step_size()
    double get_findiff_hessian_step_size() const;
    const std::string& get_findiff_hessian_mode() const;
    /// @}

protected:
    template <typename ...Types>
    void print(const std::string& format_string, Types... args) const;
private:
    const AbstractOptimizationProblem& m_problem;

    int m_verbosity = 1;
    double m_findiff_hessian_step_size = 1e-5;
    std::string m_findiff_hessian_mode = "fast";

};

inline int OptimizationProblemDecorator::get_verbosity() const {
    return m_verbosity;
}

inline double OptimizationProblemDecorator::
get_findiff_hessian_step_size() const {
    return m_findiff_hessian_step_size;
}

inline const std::string& OptimizationProblemDecorator::
get_findiff_hessian_mode() const {
    return m_findiff_hessian_mode;
}

template <typename ...Types>
inline void OptimizationProblemDecorator::print(
        const std::string& format_string, Types... args) const {
    if (m_verbosity)
        std::cout << "[tropter] " << format(format_string.c_str(), args...) <<
                std::endl;
}

} // namespace tropter

#endif // TROPTER_OPTIMIZATIONPROBLEMDECORATOR_H
