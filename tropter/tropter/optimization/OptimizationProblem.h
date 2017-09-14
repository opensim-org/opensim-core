#ifndef TROPTER_OPTIMIZATIONPROBLEM_H
#define TROPTER_OPTIMIZATIONPROBLEM_H

#include <tropter/common.h>
#include "AbstractOptimizationProblem.h"
#include <memory>
#include <map>

namespace ColPack {
class BipartiteGraphPartialColoringInterface;
class JacobianRecovery1D;
}

namespace tropter {

/// This class provides an interface of the OptimizationProblem to the
/// OptimizationSolvers.
/// In general, users do not use this class directly.
/// There is a derived type for each scalar type, and these derived classes
/// compute the gradient, Jacobian, and Hessian (using either finite differences
/// or automatic differentiation).
/// @ingroup optimization
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
private:
    const AbstractOptimizationProblem& m_problem;
};

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
    std::shared_ptr<OptimizationProblemDecorator> make_decorator()
            const override final;

    // TODO can override to provide custom derivatives.
    //virtual void gradient(const std::vector<T>& x, std::vector<T>& grad) const;
    //virtual void jacobian(const std::vector<T>& x, TODO) const;
    //virtual void hessian() const;
};

template<typename T>
std::shared_ptr<OptimizationProblemDecorator>
OptimizationProblem<T>::make_decorator() const {
    // TODO is this what we want? a shared_ptr??
    return std::make_shared<Decorator>(*this);
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
template<>
class OptimizationProblem<double>::Decorator
        : public OptimizationProblemDecorator {
public:
    Decorator(const OptimizationProblem<double>& problem);
    ~Decorator();
    void calc_sparsity(const Eigen::VectorXd& variables,
            std::vector<unsigned int>& jacobian_row_indices,
            std::vector<unsigned int>& jacobian_col_indices,
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
            const double* variables,
            bool new_variables, double obj_factor,
            unsigned num_constraints, const double* lambda,
            bool new_lambda,
            unsigned num_nonzeros, double* nonzeros) const override;
private:
    const OptimizationProblem<double>& m_problem;

    // Working memory.
    mutable Eigen::VectorXd m_x_working;
    mutable Eigen::VectorXd m_constr_pos;
    mutable Eigen::VectorXd m_constr_neg;
    // TODO this could be a column vector unless we are using parallelization.
    mutable Eigen::MatrixXd m_jacobian_compressed_eigen;

    // Gradient.
    // ---------
    // The indices of the variables used in the objective function
    // (conservative estimate of the indicies of the gradient that are nonzero).
    mutable std::vector<unsigned int> m_gradient_nonzero_indices;

    // Jacobian.
    // ---------

    // ColPack objects for (a) determining the directions in which to perturb
    // the variables to compute the Jacobian and (b) recovering the sparse
    // Jacobian (to pass to the optimization solver) after computing finite
    // differences.
    mutable std::unique_ptr<ColPack::BipartiteGraphPartialColoringInterface>
            m_jacobian_coloring;
    mutable std::unique_ptr<ColPack::JacobianRecovery1D> m_jacobian_recovery;

    // This has dimensions num_variables x num_perturbation_directions. All
    // entries are either 0 or 1, and each column is a direction in which we
    // will perturb the variables. This matrix is computed for us by
    // ColPack's graph coloring algorithms.
    mutable Eigen::MatrixXd m_jacobian_seed;

    // We determine the sparsity structure of the Jacobian (by propagating
    // NaNs through the constraint function) and hold the result in this
    // variable to pass to ColPack methods.
    // We resort to using a unique_ptr with a custom deleter because ColPack
    // requires that we provide the sparsity structure as two-dimensional C
    // array.
    using UnsignedInt2DPtr =
            std::unique_ptr<unsigned*[], std::function<void(unsigned**)>>;
    mutable UnsignedInt2DPtr m_jacobian_pattern_ADOLC_format;

    // Working memory to hold onto the Jacobian finite differences to pass to
    // ColPack.
    // We resort to using a unique_ptr with a custom deleter because ColPack
    // requires that we provide the data as a two-dimensional C array.
    using Double2DPtr =
            std::unique_ptr<double*[], std::function<void(double**)>>;
    mutable Double2DPtr m_jacobian_compressed;

    // These variables store the output of ColPack's recovery routine, but the
    // values are not used.
    mutable std::vector<unsigned int> m_jacobian_recovered_row_indices;
    mutable std::vector<unsigned int> m_jacobian_recovered_col_indices;
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
