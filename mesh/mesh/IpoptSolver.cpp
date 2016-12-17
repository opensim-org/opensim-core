#include "IpoptSolver.h"
#include "OptimizationProblem.h"
#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Ref;
using Ipopt::Index;
using Ipopt::Number;

using namespace mesh;

double IpoptSolver::optimize_impl(VectorXd& variables) const {
    Ipopt::SmartPtr<TNLP> nlp = new TNLP(m_problem);
    // TODO avoid copying x (initial guess).
    // Determine sparsity pattern of Jacobian, Hessian, etc.
    nlp->initialize(variables);

    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
    // Set options.
    // TODO app->Options()->SetStringValue("derivative_test", "second-order");
    Ipopt::ApplicationReturnStatus status;
    // TODO give istream or data file?
    status = app->Initialize();
    //MESH_THROW_IF(status != Ipopt::Solve_Succeeded, Exception,
    //        "Error during initialization");
    if (status != Ipopt::Solve_Succeeded) {
        std::cerr << "Error during initialization" << std::endl;
        // TODO throw exception.
    }

    // Optimize!!!
    // -----------
    status = app->OptimizeTNLP(nlp);
    if (status != Ipopt::Solve_Succeeded) {
        // TODO give detailed diagnostics.
        throw std::runtime_error("Failed to find a solution.");
    }
    variables = nlp->get_solution();
    return nlp->get_optimal_objective_value();
}

IpoptSolver::TNLP::TNLP(
        std::shared_ptr<const OptimizationProblemProxy> problem)
        : m_problem(problem)
{
    m_num_variables = m_problem->num_variables();
    m_num_constraints = m_problem->num_constraints();
}

bool IpoptSolver::TNLP::get_nlp_info(Index& num_variables,
                                     Index& num_constraints,
                                     Index& num_nonzeros_jacobian, Index& num_nonzeros_hessian,
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
    m_jacobian_num_nonzeros = m_jacobian_row_indices.size();
    m_hessian_num_nonzeros = m_hessian_row_indices.size();

    // Determine sparsity patterns.
    // ----------------------------
//    // TODO allow user to provide multiple points at which to determine
//    // sparsity?
//    // TODO remove from here and utilize new_x.
//    // TODO or can I reuse the tape?
//    /* TODO if (m_num_constraints)*/ {
//        short int tag = 0;
//        VectorXd g(m_num_constraints);
//        trace_constraints(tag, m_num_variables, guess.data(),
//                m_num_constraints, g.data());

//        // TODO use jac_pat function instead.
//        int repeated_call = 0;
//        int num_nonzeros = -1; /*TODO*/
//        unsigned int* row_indices = NULL; // Allocated by ADOL-C.
//        unsigned int* col_indices = NULL; // Allocated by ADOL-C.
//        double* jacobian = NULL;          // Allocated by ADOL-C.
//        int options[4];
//        options[0] = 0; /*TODO*/
//        options[1] = 0; /*TODO*/
//        options[2] = 0; /*TODO*/
//        options[3] = 0; /*TODO*/
//        int success = sparse_jac(tag, m_num_constraints, m_num_variables,
//                repeated_call, &guess[0],
//                &num_nonzeros, &row_indices, &col_indices,
//                &jacobian, options);
//        assert(success);
//        m_jacobian_num_nonzeros = num_nonzeros;
//        m_jacobian_row_indices.reserve(num_nonzeros);
//        m_jacobian_col_indices.reserve(num_nonzeros);
//        for (int i = 0; i < num_nonzeros; ++i) {
//            m_jacobian_row_indices[i] = row_indices[i];
//            m_jacobian_col_indices[i] = col_indices[i];
//        }
//        delete [] row_indices;
//        delete [] col_indices;
//        delete [] jacobian;
//    }

//    {
//        short int tag = 0;
//        // =================================================================
//        // START ACTIVE
//        // -----------------------------------------------------------------
//        trace_on(tag);
//        VectorXa x_adouble(m_num_variables);
//        VectorXd lambda_vector(m_num_constraints, 1);
//        adouble lagrangian_adouble;
//        double lagr;
//        for (unsigned i = 0; i < m_num_variables; ++i) {
//            x_adouble[i] <<= guess[i];
//        }
//        lagrangian(1.0, x_adouble, lambda_vector, lagrangian_adouble);
//        lagrangian_adouble >>= lagr;
//        trace_off();
//        // -----------------------------------------------------------------
//        // END ACTIVE
//        // =================================================================
//        // TODO efficiently use the "repeat" argument.
//        int repeated_call = 0;
//        int options[2];
//        options[0] = 0; /* test the computational graph control flow? TODO*/
//        options[1] = 0; /* way of recovery TODO */
//        unsigned int* row_indices = NULL;
//        unsigned int* col_indices = NULL;
//        double* hessian = NULL; // We don't actually need the hessian...
//        // TODO use hess_pat instead!!!
//        int num_nonzeros;
//        //VectorXd x_and_lambda(m_num_variables + m_num_constraints);
//        //for (unsigned ivar = 0; ivar < m_num_variables; ++ivar) {
//        //    x_and_lambda[ivar] = guess[ivar];
//        //}
//        //for (unsigned icon = 0; icon < m_num_constraints; ++icon) {
//        //    x_and_lambda[icon + m_num_variables] = 1; // TODO consistency?
//        //}
//        int success = sparse_hess(tag, m_num_variables, repeated_call,
//                &guess[0], &num_nonzeros,
//                &row_indices, &col_indices, &hessian,
//                options);
//        // TODO See ADOL-C manual Table 1 to interpret the return value.
//        // TODO improve error handling.
//        assert(success);
//        m_hessian_num_nonzeros = num_nonzeros;
//        m_hessian_row_indices.reserve(num_nonzeros);
//        m_hessian_col_indices.reserve(num_nonzeros);
//        for (int i = 0; i < num_nonzeros; ++i) {
//            m_hessian_row_indices[i] = row_indices[i];
//            m_hessian_col_indices[i] = col_indices[i];
//        }
//        // TODO try to use modern memory management.
//        delete [] row_indices;
//        delete [] col_indices;
//        delete [] hessian;
//    }
}

//void IpoptSolver::TNLP::lagrangian(double obj_factor,
//        const VectorXa& x,
//        const VectorXd& lambda,
//        adouble& result) const {
//    assert(x.size() == m_num_variables);
//    assert(lambda.size() == m_num_constraints);

//    result = 0;
//    //if (obj_factor != 0) {
//    //    objective(x, result);
//    //    result *= obj_factor;
//    //}
//    m_problem.objective(x, result); // TODO should not compute obj if
//    // obj_factor = 0.
//    // but that optimization makes ADOLC unhappy.
//    result *= obj_factor;

//    // TODO if (!m_num_constraints) return;

//    VectorXa constr(m_num_constraints);
//    m_problem.constraints(x, constr);
//    for (unsigned icon = 0; icon<m_num_constraints; ++icon) {
//        result += lambda[icon]*constr[icon];
//    }
//}

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
    // TODO change this interface so that the user specifies the initial
    // guess at the time they request the optimization?
    for (Index ivar = 0; ivar < num_variables; ++ivar) {
        x[ivar] = m_initial_guess[ivar];
    }
    return true;
}

//double IpoptSolver::TNLP::trace_objective(short int tag,
//        Index num_variables, const Number* x) {
//    // =====================================================================
//    // START ACTIVE
//    // ---------------------------------------------------------------------
//    trace_on(tag);
//    VectorXa x_adouble(num_variables);
//    adouble f_adouble = 0;
//    double f = 0;
//    for (Index i = 0; i < num_variables; ++i) x_adouble[i] <<= x[i];
//    m_problem.objective(x_adouble, f_adouble);
//    f_adouble >>= f;
//    trace_off();
//    // ---------------------------------------------------------------------
//    // END ACTIVE
//    // =====================================================================
//    return f;
//}

//void IpoptSolver::TNLP::trace_constraints(short int tag,
//        Index num_variables, const Number* x,
//        Index num_constraints, Number* g) {
//    // TODO if (!num_constraints) return true;
//    // =====================================================================
//    // START ACTIVE
//    // ---------------------------------------------------------------------
//    trace_on(tag);
//    VectorXa x_adouble(num_variables);
//    // TODO efficiently store this result so it can be used in grad_f, etc.
//    for (Index i = 0; i < num_variables; ++i) x_adouble[i] <<= x[i];
//    VectorXa g_adouble(num_constraints);
//    m_problem.constraints(x_adouble, g_adouble);
//    for (Index i = 0; i < num_constraints; ++i) g_adouble[i] >>= g[i];
//    trace_off();
//    // ---------------------------------------------------------------------
//    // END ACTIVE
//    // =====================================================================
//}

bool IpoptSolver::TNLP::eval_f(
        Index num_variables, const Number* x, bool new_x,
        Number& obj_value) {
    assert((unsigned)num_variables == m_num_variables);
    //std::vector<adouble> x_adouble(num_variables);
    //// TODO efficiently store this result so it can be used in grad_f, etc.
    //for (Index i = 0; i < num_variables; ++i) x_adouble[i] <<= x[i];
    //adouble f = 0;
    //// TODO objective() could be templatized... so that we could use finite
    //// difference if necessary.
    //m_problem.objective(x_adouble, f);
    //obj_value = f.value();
    //short int tag = 1;
    //if (new_x) {
    //    obj_value = trace_objective(tag, num_variables, x);
    //    // TODO is this caching okay? threadsafe??
    //    m_cached_obj_value = obj_value;
    //} else {
    //    obj_value = m_cached_obj_value;
    //obj_value = trace_objective(m_objective_tag, num_variables, x);

    // TODO this copy is disgusting:
    // TODO Eigen::Map<VectorXd> x_eigen(x, num_variables);
    m_problem->objective(num_variables, x, new_x, obj_value);

    return true;
}

bool IpoptSolver::TNLP::eval_grad_f(
        Index num_variables, const Number* x, bool new_x,
        Number* grad_f) {
    assert((unsigned)num_variables == m_num_variables);
    // TODO it is important to use an independent tag!
    // TODO create an enum for this tag!!!

    m_problem->gradient(num_variables, x, new_x, grad_f);

//    if (new_x) trace_objective(m_objective_tag, num_variables, x);
//    int success = gradient(m_objective_tag, num_variables, x, grad_f);
//    assert(success); // TODO probably want assert(status >= 0);

    return true;
}

bool IpoptSolver::TNLP::eval_g(
        Index num_variables, const Number* x, bool new_x,
        Index num_constraints, Number* g) {
    assert((unsigned)num_variables   == m_num_variables);
    assert((unsigned)num_constraints == m_num_constraints);
    //// TODO if (!num_constraints) return true;
    //// TODO efficiently store this result so it can be used in grad_f, etc.
//    trace_constraints(m_constraint_tag, num_variables, x, num_constraints, g);
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

//    if (new_x) {
//        VectorXd g(num_constraints);
//        trace_constraints(m_constraint_tag,
//                num_variables, x, num_constraints, g.data());
//    }
//
//    int repeated_call = 0;
//    int num_nonzeros = -1; /*TODO*/
//    unsigned int* row_indices = NULL; // Allocated by ADOL-C.
//    unsigned int* col_indices = NULL; // Allocated by ADOL-C.
//    double* jacobian = NULL;          // Allocated by ADOL-C.
//    int options[4];
//    options[0] = 0; /*TODO*/
//    options[1] = 0; /*TODO*/
//    options[2] = 0; /*TODO*/
//    options[3] = 0; /*TODO*/
//    int success = sparse_jac(m_constraint_tag, num_constraints, num_variables,
//            repeated_call, x,
//            &num_nonzeros, &row_indices, &col_indices,
//            &jacobian, options);
//    assert(success);
//    // TODO ideally we would avoid this copy. Should we use std::copy?
//    for (int inz = 0; inz < num_nonzeros; ++inz) {
//        values[inz] = jacobian[inz];
//    }
//
//    delete [] row_indices;
//    delete [] col_indices;
//    delete [] jacobian;

    return true;
}

bool IpoptSolver::TNLP::eval_h(
        Index num_variables, const Number* x, bool new_x,
        Number obj_factor, Index num_constraints, const Number* lambda,
        bool new_lambda, Index num_nonzeros_hessian,
        Index* iRow, Index *jCol, Number* values) {
    assert((unsigned)num_nonzeros_hessian == m_hessian_num_nonzeros);
    if (values == nullptr) {
        // TODO use ADOLC to determine sparsity pattern; hess_pat
        // TODO
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

//    // TODO this hessian must include the constraint portion!!!
//    // TODO if not new_x, then do NOT re-eval objective()!!!
//
//    // TODO remove from here and utilize new_x.
//    // TODO or can I reuse the tape?
//    short int tag = 0;
//    // -----------------------------------------------------------------
//    // START ACTIVE
//    trace_on(tag);
//    VectorXa x_adouble(num_variables);
//    VectorXd lambda_vector(num_constraints);
//    adouble lagrangian_adouble;
//    double lagr;
//    for (Index ivar = 0; ivar < num_variables; ++ivar) {
//        // TODO add this operator for std::vector.
//        x_adouble[ivar] <<= x[ivar];
//    }
//    for (Index icon = 0; icon < num_constraints; ++icon) {
//        lambda_vector[icon] = lambda[icon];
//    }
//    lagrangian(obj_factor, x_adouble, lambda_vector, lagrangian_adouble);
//    lagrangian_adouble >>= lagr;
//    trace_off();
//    // END ACTIVE
//    // -----------------------------------------------------------------
//    // TODO efficiently use the "repeat" argument.
//    int repeated_call = 0;
//    int options[2];
//    options[0] = 0; /* test the computational graph control flow? TODO*/
//    options[1] = 0; /* way of recovery TODO */
//    // TODO make general:
//    unsigned int* row_indices = NULL;
//    unsigned int* col_indices = NULL;
//    // TODO hope that the row indices are the same between IpOopt and
//    // ADOL-C.
//    double* vals = NULL;
//    int num_nonzeros;
//    // TODO compute sparse hessian for each element of the constraint
//    // vector....TODO trace with respect to both x and lambda..
//    // http://list.coin-or.org/pipermail/adol-c/2013-April/000900.html
//    // TODO "since lambda changes, the Lagrangian function has to be
//    // repated every time ...cannot set repeat = 1"
//    // The following link suggests more efficient methods:
//    // http://list.coin-or.org/pipermail/adol-c/2013-April/000903.html
//    // Quote:
//    // We made the experience that it really depends on the application
//    // whether
//    //
//    // * tracing the Lagrangian once with x and lambda as inputs
//    //    and evaluating only a part of the Hessian reusing the trace
//    //       in all iterations
//    //
//    // or
//    //
//    // *  retracing the Lagrangian with x as adoubles and lambda as doubles
//    // in each iteration and computing then the whole Hessian
//    //
//    // performs better in terms of runtime. You could give both approaches
//    // a try and see what works better for you. Both approaches have their
//    // pros and cons with respect to efficiency.
//    //
//
//    //std::vector<double> x_and_lambda(num_variables + num_constraints);
//    //for (unsigned ivar = 0; ivar < num_variables; ++ivar) {
//    //    x_and_lambda[ivar] = x[ivar];
//    //}
//    //for (unsigned icon = 0; icon < num_constraints; ++icon) {
//    //    x_and_lambda[icon + num_variables] = lambda[icon];
//    //}
//    int success = sparse_hess(tag, num_variables, repeated_call,
//            x, &num_nonzeros, &row_indices, &col_indices,
//            &vals, options);
//    assert(success);
//    for (int i = 0; i < num_nonzeros; ++i) {
//        values[i] = vals[i];
//    }
//    // TODO try to use modern memory management.
//    delete [] row_indices;
//    delete [] col_indices;
//    // TODO avoid reallocating vals each time!!!
//    delete [] vals;

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


