
#include "OptimizationSolver.h"
#include "OptimizationProblem.h"

using Eigen::VectorXd;

using namespace mesh;

OptimizationSolver::OptimizationSolver(
        const AbstractOptimizationProblem& problem)
        : m_problem(problem.make_proxy()) {}

double OptimizationSolver::optimize(Eigen::VectorXd& variables) const
{
    // If the user did not provide an initial guess, then we choose
    // the initial guess based on the bounds.
    if (variables.size() == 0) {
        variables = m_problem->initial_guess_from_bounds();
    } // else TODO make sure variables has the correct size.
    return optimize_impl(variables);
}
