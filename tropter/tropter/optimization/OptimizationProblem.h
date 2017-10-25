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
#include <map>

namespace tropter {

class JacobianColoring;
class HessianColoring;

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
    /// @copydoc set_findiff_hessian_step_size()
    double get_findiff_hessian_step_size() const;
    /// @}

protected:
    template <typename ...Types>
    void print(const std::string& format_string, Types... args) const;
private:
    const AbstractOptimizationProblem& m_problem;

    int m_verbosity = 1;
    double m_findiff_hessian_step_size = 1e-5;

};

inline int OptimizationProblemDecorator::get_verbosity() const {
    return m_verbosity;
}

inline double OptimizationProblemDecorator::
get_findiff_hessian_step_size() const {
    return m_findiff_hessian_step_size;
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

/// @ingroup optimization
/// The gradient, Jacobian, and Hessian are computed in a way that exploits
/// sparsity, using ColPack and graph coloring algorithms [1].
/// [1] Gebremedhin, Assefaw Hadish, Fredrik Manne, and Alex Pothen. "What color
/// is your Jacobian? Graph coloring for computing derivatives." SIAM review
/// 47.4 (2005): 629-705.
template<>
class OptimizationProblem<double>::Decorator
        : public OptimizationProblemDecorator {
public:
    Decorator(const OptimizationProblem<double>& problem);
    ~Decorator();
    void calc_sparsity(const Eigen::VectorXd& variables,
            std::vector<unsigned int>& jacobian_row_indices,
            std::vector<unsigned int>& jacobian_col_indices,
            bool provide_hessian_indices,
            std::vector<unsigned int>& hessian_row_indices,
            std::vector<unsigned int>& hessian_col_indices) const override;
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

    using CompressedRowSparsity = std::vector<std::vector<unsigned int>>;
    void calc_sparsity_hessian_lagrangian(
            const Eigen::VectorXd&, CompressedRowSparsity&) const;

    void calc_hessian_objective(const Eigen::VectorXd& x0, double eps,
            Eigen::Ref<Eigen::VectorXd> hessian_values) const;
    void calc_lagrangian(
            const Eigen::VectorXd& variables,
            double obj_factor,
            // TODO use Eigen::Ref?
            const Eigen::Map<const Eigen::VectorXd>& lambda,
            double& lagrangian_value) const;

    const OptimizationProblem<double>& m_problem;

    // Working memory shared by multiple functions.
    mutable Eigen::VectorXd m_x_working;

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
    mutable std::unique_ptr<HessianColoring> m_hessian_coloring;
    // TODO temporary until we use ColPack.
    mutable std::vector<unsigned int> m_hessian_row_indices;
    mutable std::vector<unsigned int> m_hessian_col_indices;
    // Working memory.
    mutable Eigen::VectorXd m_constr_working;

    // Deprecated.
    void calc_hessian_lagrangian_slow(unsigned num_variables,
            const double* variables,
            bool new_variables, double obj_factor,
            unsigned num_constraints, const double* lambda,
            bool new_lambda,
            unsigned num_nonzeros, double* nonzeros) const;
};

/// This specialization uses automatic differentiation (via ADOL-C) to
/// compute the derivatives of the objective and constraints.
/// @ingroup optimization
template<>
class OptimizationProblem<adouble>::Decorator
        : public OptimizationProblemDecorator {
public:
    Decorator(const OptimizationProblem<adouble>& problem);
    /// Delete memory allocated by ADOL-C.
    virtual ~Decorator();
    void calc_sparsity(const Eigen::VectorXd& variables,
            std::vector<unsigned int>& jacobian_row_indices,
            std::vector<unsigned int>& jacobian_col_indices,
            bool provide_hessian_indices,
            std::vector<unsigned int>& hessian_row_indices,
            std::vector<unsigned int>& hessian_col_indices) const override;
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

    const OptimizationProblem<adouble>& m_problem;

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

} // namespace tropter

#endif // TROPTER_OPTIMIZATIONPROBLEM_H
