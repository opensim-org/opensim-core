#ifndef TROPTER_OPTIMIZATIONSOLVER_H
#define TROPTER_OPTIMIZATIONSOLVER_H
// ----------------------------------------------------------------------------
// tropter: OptimizationSolver.h
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
#include <Eigen/Dense>

#include <memory>

namespace tropter {

class AbstractOptimizationProblem;

class OptimizationProblemDecorator;

/// @ingroup optimization
class OptimizationSolver {
public:
    /// Provide the problem to solve.
    OptimizationSolver(const AbstractOptimizationProblem& problem);
    /// The maximum number of iterations the optimizer is allowed to take.
    int get_max_iterations() const;
    /// @copydoc get_max_iterations()
    /// Set to -1 to use the optimizer's default setting.
    void set_max_iterations(int setting);
    /// Whether a full Hessian should be computed or if the Hessian
    /// should be approximated from the gradient using BFGS updates.
    /// See https://www.coin-or.org/Ipopt/documentation/node53.html#SECTION0001113010000000000000
    // TODO use enum.
    const std::string& get_hessian_approximation() const;
    /// @copydoc get_hessian_approximation()
    /// Set to an empty string to use the optimizer's default.
    void set_hessian_approximation(const std::string& setting);
    /// Optimize the optimization problem.
    /// @param[in,out] variables Pass in the initial guess to the problem, or
    ///     leave empty to use a naive initial guess based on the variables'
    ///     bounds (see OptimizationProblemProxy::initial_guess_from_bounds()).
    ///     After this function returns, `variables` contains the solution to
    ///     the optimization problem.
    /// @returns The value of the objective function evaluated at the solution.
    // TODO return struct that contains info about the optimizer's return
    // status.
    double optimize(Eigen::VectorXd& variables) const;
protected:
    virtual double optimize_impl(Eigen::VectorXd& variables) const = 0;
    std::shared_ptr<const OptimizationProblemDecorator> m_problem;
    int m_max_iterations = -1;
    std::string m_hessian_approximation = "";
};

} // namespace tropter

#endif // TROPTER_OPTIMIZATIONSOLVER_H
