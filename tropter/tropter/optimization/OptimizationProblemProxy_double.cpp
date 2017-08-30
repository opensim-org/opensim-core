#include "OptimizationProblem.hpp"

using Eigen::VectorXd;

namespace tropter {

OptimizationProblem<double>::Proxy::Proxy(
        const OptimizationProblem<double>& problem) :
        OptimizationProblemProxy(problem), m_problem(problem) {}

void OptimizationProblem<double>::Proxy::
sparsity(const Eigen::VectorXd& x,
        std::vector<unsigned int>& jacobian_row_indices,
        std::vector<unsigned int>& jacobian_col_indices,
        std::vector<unsigned int>& hessian_row_indices,
        std::vector<unsigned int>& hessian_col_indices) const
{
    // TODO
}

void OptimizationProblem<double>::Proxy::
objective(unsigned num_variables, const double* variables,
        bool /*new_x*/,
        double& obj_value) const
{
    // TODO avoid copy.
    const VectorXd xvec = Eigen::Map<const VectorXd>(variables, num_variables);
    m_problem.objective(xvec, obj_value);
}

void OptimizationProblem<double>::Proxy::
constraints(unsigned num_variables, const double* variables,
        bool /*new_variables*/,
        unsigned num_constraints, double* constr) const
{
    // TODO avoid copy.
    const VectorXd xvec = Eigen::Map<const VectorXd>(variables, num_variables);
    VectorXd constrvec(num_constraints); // TODO avoid copy.
    m_problem.constraints(xvec, constrvec);
}

void OptimizationProblem<double>::Proxy::
jacobian(unsigned num_variables, const double* x, bool /*new_x*/,
        unsigned /*num_nonzeros*/, double* jacobian_values) const
{
    // TODO
}

void OptimizationProblem<double>::Proxy::
hessian_lagrangian(unsigned num_variables, const double* x,
        bool /*new_x*/, double obj_factor,
        unsigned num_constraints, const double* lambda,
        bool /*new_lambda TODO */,
        unsigned /*num_nonzeros*/, double* hessian_values) const
{
    // TODO
}

} // namespace tropter
