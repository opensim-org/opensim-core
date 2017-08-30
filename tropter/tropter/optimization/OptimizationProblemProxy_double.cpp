#include "OptimizationProblem.hpp"

using Eigen::VectorXd;

// TODO leave all of this templatized, so floats can be used also.

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
    unsigned int num_jacobian_elements = num_constraints() * num_variables();
    jacobian_row_indices.resize(num_jacobian_elements);
    jacobian_col_indices.resize(num_jacobian_elements);
    // Example: the entries of a 2 x 4 Jacobian would be ordered as follows:
    //          0 2 4 6
    //          1 3 5 7
    // This is a column-major storage order.
    for (unsigned int i = 0; i < num_jacobian_elements; ++i) {
        jacobian_row_indices[i] = i % num_constraints();
        jacobian_col_indices[i] = i / num_constraints();
    }

    // Exact hesisan mode is unsupported for now.
    hessian_row_indices.clear();
    hessian_col_indices.clear();
    //const auto& num_vars = get_num_variables();
    //// Dense upper triangle.
    //unsigned int num_hessian_elements = num_vars * (num_vars - 1) / 2;
    //hessian_row_indices.resize(num_hessian_elements);
    //hessian_col_indices.resize(num_hessian_elements);
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
    // TODO at least keep constrvec as working memory.
    m_problem.constraints(xvec, constrvec);
    // TODO avoid copy.
    std::copy(constrvec.data(), constrvec.data() + num_constraints, constr);
}

void OptimizationProblem<double>::Proxy::
gradient(unsigned num_variables, const double* x, bool /*new_x*/,
        double* grad) const
{
    // TODO
    VectorXd x_working = Eigen::Map<const VectorXd>(x, num_variables);

    // TODO use a better estimate for this step size.
    const double eps = std::sqrt(Eigen::NumTraits<double>::epsilon());
    const double two_eps = 2 * eps;

    double obj_pos;
    double obj_neg;
    for (Eigen::Index i = 0; i < num_variables; ++i) {
        // Perform a central difference.
        x_working[i] += eps;
        m_problem.objective(x_working, obj_pos);
        x_working[i] = x[i] - eps;
        m_problem.objective(x_working, obj_neg);
        // Restore the original value.
        x_working[i] = x[i];
        grad[i] = (obj_pos - obj_neg) / two_eps;
    }
}

void OptimizationProblem<double>::Proxy::
jacobian(unsigned num_variables, const double* x, bool /*new_x*/,
        unsigned /*num_nonzeros*/, double* jacobian_values) const
{
    VectorXd x_working = Eigen::Map<const VectorXd>(x, num_variables);
    const double eps = std::sqrt(Eigen::NumTraits<double>::epsilon());
    const double two_eps = 2 * eps;
    // TODO

    Eigen::Map<Eigen::MatrixXd> jacobian(jacobian_values, num_constraints(),
            num_variables);

    VectorXd constr_pos(num_constraints());
    VectorXd constr_neg(num_constraints());
    for (int icol = 0; icol < num_variables; ++icol) {
        x_working[icol] += eps;
        m_problem.constraints(x_working, constr_pos);
        x_working[icol] = x[icol] - eps;
        m_problem.constraints(x_working, constr_neg);
        // Restore the original value.
        x_working[icol] = x[icol];
        jacobian.col(icol) = (constr_pos - constr_neg) / two_eps;
    }
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
