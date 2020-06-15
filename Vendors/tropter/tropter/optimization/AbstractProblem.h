#ifndef TROPTER_OPTIMIZATION_ABSTRACTPROBLEM_H
#define TROPTER_OPTIMIZATION_ABSTRACTPROBLEM_H
// ----------------------------------------------------------------------------
// tropter: AbstractProblem.h
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
#include <tropter/Exception.h>
#include <memory>

namespace tropter {

class SymmetricSparsityPattern;

namespace optimization {

class ProblemDecorator;

/// @ingroup optimization
class AbstractProblem {
public:

    AbstractProblem() = default;
    AbstractProblem(unsigned num_variables, unsigned num_constraints)
            :m_num_variables(num_variables),
             m_num_constraints(num_constraints) { }
    virtual ~AbstractProblem() = default;
    unsigned get_num_variables() const { return m_num_variables; }
    unsigned get_num_constraints() const { return m_num_constraints; }
    const Eigen::VectorXd&
    get_variable_lower_bounds() const { return m_variable_lower_bounds; }
    const Eigen::VectorXd&
    get_variable_upper_bounds() const { return m_variable_upper_bounds; }
    const Eigen::VectorXd&
    get_constraint_lower_bounds() const { return m_constraint_lower_bounds; }
    const Eigen::VectorXd&
    get_constraint_upper_bounds() const { return m_constraint_upper_bounds; }

    /// Get a vector of names of all variables in the optimization problem,
    /// in the correct order.
    /// If unimplemented, this returns an empty vector.
    virtual std::vector<std::string> get_variable_names() const { return {}; }

    /// Get a vector of names of the constraints in the optimization problem, in
    /// the correct order.
    /// If unimplemented, this returns an empty vector.
    virtual std::vector<std::string> get_constraint_names() const { return {}; }

    /// This method throws an exception if the following are not true:
    /// - the number of variable bounds matches the number of variables,
    /// - the number of constraint bounds matches the number of constraints.
    void validate() const;

    /// Create an initial guess for this problem according to the
    /// following rules:
    ///   - unconstrained variable: 0.
    ///   - lower and upper bounds: midpoint of the bounds.
    ///   - only one bound: value of the bound.
    Eigen::VectorXd make_initial_guess_from_bounds() const;
    /// Create a vector with random variable values within the variable
    /// bounds, potentially for use as an initial guess. If, for a given
    /// variable, either bound is infinite, then the element is a random number
    /// in [-1, 1] clamped by the bounds.
    // TODO rename to random_variables
    Eigen::VectorXd make_random_iterate_within_bounds() const;
    /// When using finite differences to compute derivatives, should we use
    /// the user-supplied sparsity pattern of the Hessian (provided by
    /// implementing calc_sparsity_hessian_lagrangian())? If false, then we
    /// assume the Hessian is dense, which will have a very negative impact
    /// on performance.
    bool get_use_supplied_sparsity_hessian_lagrangian() const
    {   return m_use_supplied_sparsity_hessian_lagrangian; }
    /// @copydoc get_use_supplied_sparsity_hessian_lagrangian()
    /// If this is true and calc_sparsity_hessian_lagrangian() is not
    /// implemented, an exception is thrown.
    /// This must be false if using automatic differentiation.
    void set_use_supplied_sparsity_hessian_lagrangian(bool value)
    {   m_use_supplied_sparsity_hessian_lagrangian = value; }
    /// If using finite differences (double) with a Newton method (exact
    /// Hessian in IPOPT), then we require the sparsity pattern of the
    /// Hessian of the Lagrangian. By default, we estimate the Hessian's
    /// sparsity pattern using estimates of the sparsity of the Jacobian of
    /// the constraints and gradient of the objective.
    /// If you know the sparsity pattern of the Hessian, implement this
    /// function to provide it. This could have a *huge* impact on the speed
    /// of the optimization for sparse problems. Provide the pattern for the
    /// Hessian of the constraints (for lambda^T * constraints, or sum_i c_i(x))
    /// and for the Hessian of the objective, separately.
    /// Use the supplied SymmetricSparsityPattern objects; call set_nonzero()
    /// for each nonzero element in the upper triangle of the relevant Hessian.
    ///
    /// An iterate is provided for use in detecting sparsity, if
    /// necessary (e.g., by perturbing the objective or constraint functions).
    /// The caller determines what kind if iterate to provide (the initial
    /// guess, a random iterate, etc.).
    virtual void calc_sparsity_hessian_lagrangian(const Eigen::VectorXd& x,
            SymmetricSparsityPattern& hescon_sparsity,
            SymmetricSparsityPattern& hesobj_sparsity) const;

    class CalcSparsityHessianLagrangianNotImplemented : public Exception {};

    virtual std::unique_ptr<ProblemDecorator>
    make_decorator() const = 0;

protected:

    void set_num_variables(unsigned num_variables) {
        // TODO if set, invalidate variable bounds.
        m_num_variables = num_variables;
    }
    void set_num_constraints(unsigned num_constraints) {
        m_num_constraints = num_constraints;
    }
    void set_variable_bounds(const Eigen::VectorXd& lower,
            const Eigen::VectorXd& upper) {
        // TODO make sure num_variables has been set.
        // TODO can only call this if m_num_variables etc are already set.
        assert(lower.size() == m_num_variables);
        assert(upper.size() == m_num_variables);
        // TODO where should this ordering check go?
        // TODO make sure that this check works (test it).
        assert((lower.array() <= upper.array()).all());
        m_variable_lower_bounds = lower;
        m_variable_upper_bounds = upper;
    }
    void set_constraint_bounds(const Eigen::VectorXd& lower,
            const Eigen::VectorXd& upper) {
        assert(lower.size() == m_num_constraints);
        assert(upper.size() == m_num_constraints);
        // TODO assert(lower <= upper);
        m_constraint_lower_bounds = lower;
        m_constraint_upper_bounds = upper;
    }

private:
    // TODO use safer types that will give exceptions for improper values.
    unsigned m_num_variables;
    unsigned m_num_constraints;
    bool m_use_supplied_sparsity_hessian_lagrangian = false;
    Eigen::VectorXd m_variable_lower_bounds;
    Eigen::VectorXd m_variable_upper_bounds;
    Eigen::VectorXd m_constraint_lower_bounds;
    Eigen::VectorXd m_constraint_upper_bounds;
};

inline void AbstractProblem::calc_sparsity_hessian_lagrangian(
        const Eigen::VectorXd&,
        SymmetricSparsityPattern&,
        SymmetricSparsityPattern&) const {
    throw CalcSparsityHessianLagrangianNotImplemented();
}
inline Eigen::VectorXd
AbstractProblem::make_initial_guess_from_bounds() const
{
    const auto& lower = get_variable_lower_bounds();
    const auto& upper = get_variable_upper_bounds();
    assert(lower.size() == upper.size());
    Eigen::VectorXd guess(lower.size());
    const auto inf = std::numeric_limits<double>::infinity();
    for (Eigen::Index i = 0; i < lower.size(); ++i) {
        if (lower[i] != -inf && upper[i] != inf) {
            guess[i] = 0.5 * (upper[i] + lower[i]);
        }
        else if (lower[i] != -inf) guess[i] = lower[i];
        else if (upper[i] != inf) guess[i] = upper[i];
        else guess[i] = 0;
    }
    return guess;
}
inline Eigen::VectorXd
AbstractProblem::make_random_iterate_within_bounds() const {
    const auto lower = get_variable_lower_bounds().array();
    const auto upper = get_variable_upper_bounds().array();
    // random's values are within [-1, 1]
    Eigen::ArrayXd random = Eigen::ArrayXd::Random(lower.size());
    // Get values between [0, 1], then scale by width and shift by lower.
    Eigen::ArrayXd scaled = 0.5 * (random + 1.0) * (upper - lower) + lower;
    for (int i = 0; i < scaled.size(); ++i) {
        if (std::isnan(scaled[i]))
            scaled[i] = std::min(std::max(random[i], lower[i]), upper[i]);
    }
    return scaled;
}

} // namespace optimization
} // namespace tropter

#endif // TROPTER_OPTIMIZATION_ABSTRACTPROBLEM_H
