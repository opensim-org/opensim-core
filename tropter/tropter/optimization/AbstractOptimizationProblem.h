#ifndef TROPTER_ABSTRACTOPTIMIZATIONPROBLEM_H
#define TROPTER_ABSTRACTOPTIMIZATIONPROBLEM_H
// ----------------------------------------------------------------------------
// tropter: AbstractOptimizationProblem.h
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
#include <memory>

namespace tropter {

class OptimizationProblemDecorator;

/// @ingroup optimization
class AbstractOptimizationProblem {
public:

    AbstractOptimizationProblem() = default;
    AbstractOptimizationProblem(
            unsigned num_variables, unsigned num_constraints)
            : m_num_variables(num_variables),
              m_num_constraints(num_constraints) {}
    unsigned get_num_variables() const { return m_num_variables; }
    unsigned get_num_constraints() const { return m_num_constraints; }
    const Eigen::VectorXd& get_variable_lower_bounds() const
    {   return m_variable_lower_bounds; }
    const Eigen::VectorXd& get_variable_upper_bounds() const
    {   return m_variable_upper_bounds; }
    const Eigen::VectorXd& get_constraint_lower_bounds() const
    {   return m_constraint_lower_bounds; }
    const Eigen::VectorXd& get_constraint_upper_bounds() const
    {   return m_constraint_upper_bounds; }

    /// WHen using finite differences to compute derivatives, should we use
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
    /// Hessian in Ipopt), then we require the sparsity pattern of the
    /// Hessian of the Lagrangian. By default, we assume the Hessian is dense.
    /// If you know the sparsity pattern of the Hessian, implement this
    /// function to provide it. This has a *huge* impact on the speed
    /// of the optimization for sparse problems.
    // TODO do the elements need to be ordered in any particular way?
    // TODO require ADOL-C's compressed row format.
    // TODO sparsity will already have the correct outer dimension.
    virtual void calc_sparsity_hessian_lagrangian(const Eigen::VectorXd& x,
            std::vector<std::vector<unsigned int>>& sparsity) const;
    class CalcSparsityHessianLagrangianNotImplemented
            : public std::exception {};

    virtual std::shared_ptr<OptimizationProblemDecorator>
    make_decorator() const = 0;

protected:

    void set_num_variables(unsigned num_variables) {
        // TODO if set, invalidate variable bounds.
        m_num_variables = num_variables;
    }
    void set_num_constraints(unsigned num_constraints) {
        m_num_constraints = num_constraints;
    }
    // TODO eigen wants these to be more generic to avoid temporaries.
    // TODO allow specifying these as std::vector<std::pair<double>>;
    // this is a more logical way to specify bounds for users.
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

inline void AbstractOptimizationProblem::calc_sparsity_hessian_lagrangian(
        const Eigen::VectorXd&, std::vector<std::vector<unsigned int>>&) const {
    throw CalcSparsityHessianLagrangianNotImplemented();
}

} // namespace tropter

#endif // TROPTER_ABSTRACTOPTIMIZATIONPROBLEM_H
