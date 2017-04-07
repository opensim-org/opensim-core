#ifndef MESH_OPTIMIZATIONSOLVER_H
#define MESH_OPTIMIZATIONSOLVER_H

#include "common.h"
#include <Eigen/Dense>

#include <memory>

namespace mesh {

class AbstractOptimizationProblem;

class OptimizationProblemProxy;

class OptimizationSolver {
public:
    /// Provide the problem to solve.
    OptimizationSolver(const AbstractOptimizationProblem& problem);
    /// Optimize the optimization problem.
    /// @param[in,out] variables Pass in the initial guess to the problem, or
    ///     leave empty to use a naive initial guess based on the variables'
    ///     bounds (see OptimizationProblemProxy::initial_guess_from_bounds()).
    ///     After this function returns, `variables` contains the solution to
    ///     the optimization problem.
    /// @returns The value of the objective function evaluated at the solution.
    double optimize(Eigen::VectorXd& variables) const;
protected:
    virtual double optimize_impl(Eigen::VectorXd& variables) const = 0;
    std::shared_ptr<const OptimizationProblemProxy> m_problem;
    // TODO rename m_problem to m_proxy? m_probproxy?
//    const OptimizationProblemProxy& m_problem;
};

// TODO perhaps NewtonSolver {}; ?

} // namespace mesh

#endif // MESH_OPTIMIZATIONSOLVER_H
