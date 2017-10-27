#ifndef TROPTER_OPTIMIZATIONPROBLEM_H
#define TROPTER_OPTIMIZATIONPROBLEM_H
// ----------------------------------------------------------------------------
// tropter: OptimizationProblem.h
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
#include <memory>

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

/// Users define an optimization problem by deriving from this class template.
/// The type T determines how the derivatives of the objective and constraint
/// functions are computed: T = double for finite differences, T = adouble
/// for automatic differentiation.
/// @ingroup optimization
template<typename T>
class OptimizationProblem : public AbstractOptimizationProblem {
public:
    /// The Decorator computes the gradient, Jacobian, and Hessian in a
    /// generic way for the objective and constraint functions provided in
    /// OptimizationProblem.
    class Decorator;

    virtual ~OptimizationProblem() = default;
    OptimizationProblem() = default;

    OptimizationProblem(unsigned num_variables, unsigned num_constraints) :
            AbstractOptimizationProblem(num_variables, num_constraints) {}

    /// Implement this function to compute the objective function.
    /// @param variables
    ///     This holds the values of the variables at the current iteration of
    ///     the optimization problem.
    /// @param obj_value
    ///     Store the objective function value in this variable.
    virtual void calc_objective(const VectorX<T>& variables,
            T& obj_value) const;

    /// Implement this function to compute the constraint function (no need
    /// to implement if your problem has no constraints).
    /// @param variables
    ///     This holds the values of the variables at the current iteration of
    ///     the optimization problem.
    /// @param constr
    ///     Store the constraint equation values in this vector, which has
    ///     `num_constraints` elements.
    virtual void calc_constraints(const VectorX<T>& variables,
            Eigen::Ref<VectorX<T>> constr) const;

    /// Create an interface to this problem that can provide the derivatives
    /// of the objective and constraint functions. This is for use by the
    /// optimization solver, but users might call this if they are interested
    /// in obtaining the sparsity pattern or derivatives for their problem.
    std::unique_ptr<OptimizationProblemDecorator> make_decorator()
            const override final;

    // TODO can override to provide custom derivatives.
    //virtual void gradient(const std::vector<T>& x, std::vector<T>& grad) const;
    //virtual void jacobian(const std::vector<T>& x, TODO) const;
    //virtual void hessian() const;
};

template<typename T>
std::unique_ptr<OptimizationProblemDecorator>
OptimizationProblem<T>::make_decorator() const {
    return std::unique_ptr<Decorator>(new Decorator(*this));
}

template<typename T>
void OptimizationProblem<T>::calc_objective(const VectorX<T>&, T&) const {
    // TODO proper error messages.
    throw std::runtime_error("Not implemented.");
}

template<typename T>
void OptimizationProblem<T>::calc_constraints(const VectorX<T>&,
        Eigen::Ref<VectorX<T>>) const
{}

/// We must specialize this template for each scalar type.
/// @ingroup optimization
template<typename T>
class OptimizationProblem<T>::Decorator : public OptimizationProblemDecorator {
};

} // namespace tropter

#endif // TROPTER_OPTIMIZATIONPROBLEM_H
