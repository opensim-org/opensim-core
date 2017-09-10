#include "IpoptSolver.h"
#include "OptimizationProblem.h"
#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Ref;
using Ipopt::Index;
using Ipopt::Number;

using namespace tropter;

double IpoptSolver::optimize_impl(VectorXd& variables) const {
    Ipopt::SmartPtr<TNLP> nlp = new TNLP(m_problem);
    // TODO avoid copying x (initial guess).
    // Determine sparsity pattern of Jacobian, Hessian, etc.
    nlp->initialize(variables);

    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
    // Set options.
    if (m_max_iterations != -1) {
        app->Options()->SetIntegerValue("max_iter", m_max_iterations);
    }
    // TODO app->Options()->SetStringValue("derivative_test", "second-order");
    if (!m_hessian_approximation.empty()) {
        if (m_hessian_approximation != "exact"
                && m_hessian_approximation != "limited-memory") {
            throw std::runtime_error("[tropter] When using Ipopt, the "
                    "'hessian_approximation' setting must be either "
                    "'exact' or 'limited-memory', but '" +
                    m_hessian_approximation + "' was provided.");
        }
        app->Options()->SetStringValue("hessian_approximation",
                m_hessian_approximation);
    }
    //app->Options()->SetStringValue("linear_solver", "ma97");
    Ipopt::ApplicationReturnStatus status;
    // TODO give istream or data file?
    status = app->Initialize();
    //TROPTER_THROW_IF(status != Ipopt::Solve_Succeeded, Exception,
    //        "Error during initialization");
    if (status != Ipopt::Solve_Succeeded) {
        std::cerr << "Error during initialization" << std::endl;
        // TODO throw exception.
    }

    // Optimize!!!
    // -----------
    status = app->OptimizeTNLP(nlp);
    if (status != Ipopt::Solve_Succeeded
            && status != Ipopt::Solved_To_Acceptable_Level) {
        // TODO give detailed diagnostics.
        // TODO throw exception.
        throw std::runtime_error("[tropter] Failed to find a solution.");
        //std::cerr << "[tropter] Failed to find a solution." << std::endl;
    }
    variables = nlp->get_solution();
    return nlp->get_optimal_objective_value();
}

IpoptSolver::TNLP::TNLP(
        std::shared_ptr<const OptimizationProblemDecorator> problem)
        : m_problem(problem)
{
    m_num_variables = m_problem->num_variables();
    m_num_constraints = m_problem->num_constraints();
}

bool IpoptSolver::TNLP::get_nlp_info(Index& num_variables,
                                     Index& num_constraints,
                                     Index& num_nonzeros_jacobian,
                                     Index& num_nonzeros_hessian,
                                     IndexStyleEnum& index_style)
{
    num_variables = m_problem->num_variables();
    num_constraints = m_problem->num_constraints();
    num_nonzeros_jacobian = m_jacobian_num_nonzeros;
    num_nonzeros_hessian = m_hessian_num_nonzeros;
    index_style = TNLP::C_STYLE;
    return true;
}

void IpoptSolver::TNLP::initialize(const VectorXd& guess) {
    // TODO all of this content should be taken care of for us by
    // OptimizationProblem.

    // TODO should not be storing the solution at all.
    // TODO consider giving an error if initialize()
    // is ever called twice...TNLP should be one-time use!
    assert(m_solution.size() == 0);
    //m_solution.resize(0);
    // TODO be smart about the need to copy "guess" (could be long)?
    m_initial_guess = guess;
    // TODO check their sizes.
    assert(guess.size() == m_num_variables);

    // TODO use VectorXi for the sparsity pattern? allows not initializing.
    m_problem->sparsity(guess,
                        m_jacobian_row_indices, m_jacobian_col_indices,
                        m_hessian_row_indices,  m_hessian_col_indices);
    m_jacobian_num_nonzeros = (unsigned)m_jacobian_row_indices.size();
    m_hessian_num_nonzeros = (unsigned)m_hessian_row_indices.size();
}

bool IpoptSolver::TNLP::get_bounds_info(
        Index num_variables, Number* x_lower, Number* x_upper,
        Index num_constraints, Number* g_lower, Number* g_upper) {
    assert((unsigned)num_variables   == m_num_variables);
    assert((unsigned)num_constraints == m_num_constraints);

    // TODO pass onto subclass.
    // TODO efficient copying.

    // TODO make sure bounds have been set.
    const auto& variable_lower = m_problem->variable_lower_bounds();
    const auto& variable_upper = m_problem->variable_upper_bounds();
    assert((variable_lower.array() <= variable_upper.array()).all());
    for (Index ivar = 0; ivar < num_variables; ++ivar) {
        // TODO can get rid of this in favor of the vectorized version.
        const auto& lower = variable_lower[ivar];
        const auto& upper = variable_upper[ivar];
        assert(lower <= upper);
        x_lower[ivar] = variable_lower[ivar];
        x_upper[ivar] = variable_upper[ivar];
    }
    // TODO do not assume that there are no inequality constraints.
    const auto& constraint_lower = m_problem->constraint_lower_bounds();
    const auto& constraint_upper = m_problem->constraint_upper_bounds();
    if (    constraint_lower.size() != (unsigned)num_constraints ||
            constraint_upper.size() != (unsigned)num_constraints) {
        // TODO better error handling.
        for (Index icon = 0; icon < num_constraints; ++icon) {
            g_lower[icon] = 0;
            g_upper[icon] = 0;
        }
    } else {
        // TODO vectorized:
        // TODO turn the following into an exception message:
        // assert((constraint_lower.array() <= constraint_upper.array()).all());
        for (Index icon = 0; icon < num_constraints; ++icon) {
            const auto& lower = constraint_lower[icon];
            const auto& upper = constraint_upper[icon];
            assert(lower <= upper);
            g_lower[icon] = lower;
            g_upper[icon] = upper;
        }
    }
    return true;
}

// z: multipliers for bound constraints on x.
// warmstart will require giving initial values for the multipliers.
bool IpoptSolver::TNLP::get_starting_point(
        Index num_variables, bool init_x, Number* x,
        bool init_z, Number* /*z_L*/, Number* /*z_U*/,
        Index num_constraints, bool init_lambda,
        Number* /*lambda*/) {
    // Must this method provide initial values for x, z, lambda?
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);
    assert((unsigned)num_constraints == m_num_constraints);
    for (Index ivar = 0; ivar < num_variables; ++ivar) {
        x[ivar] = m_initial_guess[ivar];
    }
    return true;
}

bool IpoptSolver::TNLP::eval_f(
        Index num_variables, const Number* x, bool new_x,
        Number& obj_value) {
    assert((unsigned)num_variables == m_num_variables);
    m_problem->objective(num_variables, x, new_x, obj_value);
    return true;
}

bool IpoptSolver::TNLP::eval_grad_f(
        Index num_variables, const Number* x, bool new_x,
        Number* grad_f) {
    assert((unsigned)num_variables == m_num_variables);
    m_problem->gradient(num_variables, x, new_x, grad_f);
    return true;
}

bool IpoptSolver::TNLP::eval_g(
        Index num_variables, const Number* x, bool new_x,
        Index num_constraints, Number* g) {
    assert((unsigned)num_variables   == m_num_variables);
    assert((unsigned)num_constraints == m_num_constraints);
    //// TODO if (!num_constraints) return true;
    m_problem->constraints(num_variables, x, new_x, num_constraints, g);
    return true;
}

// TODO can Ipopt do finite differencing for us?
bool IpoptSolver::TNLP::eval_jac_g(
        Index num_variables, const Number* x, bool new_x,
        Index num_constraints, Index num_nonzeros_jacobian,
        Index* iRow, Index *jCol, Number* values) {
    assert((unsigned)num_constraints == m_num_constraints);
    // TODO if (!num_constraints) return true;
    if (values == nullptr) {
        // TODO document: provide sparsity pattern.
        assert((unsigned)num_nonzeros_jacobian == m_jacobian_num_nonzeros);
        for (Index inz = 0; inz < num_nonzeros_jacobian; ++inz) {
            iRow[inz] = m_jacobian_row_indices[inz];
            jCol[inz] = m_jacobian_col_indices[inz];
        }
        return true;
    }

    m_problem->jacobian(num_variables, x, new_x, num_nonzeros_jacobian, values);

    return true;
}

bool IpoptSolver::TNLP::eval_h(
        Index num_variables, const Number* x, bool new_x,
        Number obj_factor, Index num_constraints, const Number* lambda,
        bool new_lambda, Index num_nonzeros_hessian,
        Index* iRow, Index *jCol, Number* values) {
    assert((unsigned)num_nonzeros_hessian == m_hessian_num_nonzeros);
    if (values == nullptr) {
        for (Index inz = 0; inz < num_nonzeros_hessian; ++inz) {
            iRow[inz] = m_hessian_row_indices[inz];
            jCol[inz] = m_hessian_col_indices[inz];
        }
        return true;
    }

    // TODO use obj_factor here to determine what computation to do exactly.

    m_problem->hessian_lagrangian(num_variables, x, new_x, obj_factor,
                                  num_constraints, lambda, new_lambda,
                                  num_nonzeros_hessian, values);

    return true;
}

void IpoptSolver::TNLP::finalize_solution(Ipopt::SolverReturn /*status*/,
                                          Index num_variables,
                                          const Number* x,
                                          const Number* /*z_L*/, const Number* /*z_U*/,
                                          Index /*num_constraints*/,
                                          const Number* /*g*/, const Number* /*lambda*/,
                                          Number obj_value, const Ipopt::IpoptData* /*ip_data*/,
                                          Ipopt::IpoptCalculatedQuantities* /*ip_cq*/)
{
    m_solution.resize(num_variables);
    //printf("\nSolution of the primal variables, x\n");
    for (Index i = 0; i < num_variables; ++i) {
        //printf("x[%d]: %e\n", i, x[i]);
        m_solution[i] = x[i];
    }
    m_optimal_obj_value = obj_value;
    //printf("\nSolution of the bound multipliers, z_L and z_U\n");
    //for (Index i = 0; i < num_variables; ++i) {
    //    printf("z_L[%d] = %e\n", i, z_L[i]);
    //}
    //for (Index i = 0; i < num_variables; ++i) {
    //    printf("z_U[%d] = %e\n", i, z_U[i]);
    //}
    //printf("\nObjective value\n");
    //printf("f(x*) = %e\n", obj_value);
    // TODO also implement Ipopt's intermediate_() function.
}


