#ifndef TROPTER_OPTIMIZATION_PROBLEM_H
#define TROPTER_OPTIMIZATION_PROBLEM_H
// ----------------------------------------------------------------------------
// tropter: Problem.h
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
#include "AbstractProblem.h"
#include "ProblemDecorator.h"
#include <memory>

namespace tropter {
namespace optimization {

/// Users define an optimization problem by deriving from this class template.
/// The type T determines how the derivatives of the objective and constraint
/// functions are computed: T = double for finite differences, T = adouble
/// for automatic differentiation.
/// @ingroup optimization
template<typename T>
class Problem : public AbstractProblem {
public:
    /// The Decorator computes the gradient, Jacobian, and Hessian in a
    /// generic way for the objective and constraint functions provided in
    /// OptimizationProblem.
    class Decorator;

    Problem() = default;

    Problem(unsigned num_variables, unsigned num_constraints) :
            AbstractProblem(num_variables, num_constraints) {}

    virtual ~Problem() = default;

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
    std::unique_ptr<ProblemDecorator> make_decorator()
            const override final;

    // TODO can override to provide custom derivatives.
    //virtual void gradient(const std::vector<T>& x, std::vector<T>& grad) const;
    //virtual void jacobian(const std::vector<T>& x, TODO) const;
    //virtual void hessian() const;
};

template<typename T>
std::unique_ptr<ProblemDecorator>
Problem<T>::make_decorator() const {
    return std::unique_ptr<Decorator>(new Decorator(*this));
}

template<typename T>
void Problem<T>::calc_objective(const VectorX<T>&, T&) const {
    // TODO proper error messages.
    throw std::runtime_error("Not implemented.");
}

template<typename T>
void Problem<T>::calc_constraints(const VectorX<T>&,
        Eigen::Ref<VectorX<T>>) const
{}

/// We must specialize this template for each scalar type.
/// @ingroup optimization
template<typename T>
class Problem<T>::Decorator : public ProblemDecorator {
};


} // namespace optimization
} // namespace tropter

#endif // TROPTER_OPTIMIZATION_PROBLEM_H
