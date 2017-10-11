
#include "OptimizationSolver.h"
#include "OptimizationProblem.h"

#include <tropter/Exception.hpp>

using Eigen::VectorXd;

using namespace tropter;

OptimizationSolver::OptimizationSolver(
        const AbstractOptimizationProblem& problem)
        : m_problem(problem.make_decorator()) {}

int OptimizationSolver::get_max_iterations() const
{
    return m_max_iterations;
}
void OptimizationSolver::set_max_iterations(int max_iterations)
{
    TROPTER_VALUECHECK(max_iterations > 0 || max_iterations == -1,
            "max_iterations", max_iterations, "positive or -1");

    m_max_iterations = max_iterations;
}

const std::string& OptimizationSolver::get_hessian_approximation() const
{
    return m_hessian_approximation;
}
void OptimizationSolver::set_hessian_approximation(
        const std::string& setting)
{
    m_hessian_approximation = setting;
}

double OptimizationSolver::optimize(Eigen::VectorXd& variables) const
{
    // If the user did not provide an initial guess, then we choose
    // the initial guess based on the bounds.
    if (variables.size() == 0) {
        variables = m_problem->make_initial_guess_from_bounds();
    } // else TODO make sure variables has the correct size.
    return optimize_impl(variables);
}
