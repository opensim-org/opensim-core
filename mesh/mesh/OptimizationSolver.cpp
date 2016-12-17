
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
    // the initial guess to be all zeros.
    // TODO Could be smarter: use the midpoint of the bounds of the variables.
    if (variables.size() == 0) {
        variables = VectorXd::Zero(m_problem->num_variables());
    }
    return optimize_impl(variables);
}
