#include "OptimizationProblem.hpp"

// TODO put adolc sparsedrivers in their own namespace. mesh::adolc
#include <adolc/adolc.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Ref;


namespace mesh {

Eigen::VectorXd OptimizationProblemProxy::initial_guess_from_bounds() const
{
    const auto& lower = variable_lower_bounds();
    const auto& upper = variable_upper_bounds();
    assert(lower.size() == upper.size());
    Eigen::VectorXd guess(lower.size());
    const auto inf = std::numeric_limits<double>::infinity();
    for (Eigen::Index i = 0; i < lower.size(); ++i) {
        if (lower[i] != -inf && upper[i] != inf) {
            guess[i] = 0.5 * (upper[i] + lower[i]);
        }
        else if (lower[i] != -inf) guess[i] = lower[i];
        else if (upper[i] !=  inf) guess[i] = upper[i];
        else guess[i] = 0;
    }
    return guess;
}

//template<>
//void OptimizationProblem<adouble>::objective(
//        const Eigen::VectorXd& x, double& obj_value) const
//{
//    assert(x.size() == m_num_variables);
//    // TODO where to store this tag? the proxy would be a good location.
//    // TODO b/c we can't add a member variable to the template class.
//    //static const short int objective_tag = 1;
//    // =====================================================================
//    // START ACTIVE
//    // ---------------------------------------------------------------------
//    trace_on(m_objective_tag);
//    VectorXa x_adouble(m_num_variables);
//    adouble f_adouble = 0;
//    for (unsigned i = 0; i < m_num_variables; ++i) x_adouble[i] <<= x[i];
//    m_problem.objective(x_adouble, f_adouble);
//    f_adouble >>= obj_value;
//    trace_off();
//    // ---------------------------------------------------------------------
//    // END ACTIVE
//    // =====================================================================
//}
//
//template<>
//void OptimizationProblem<adouble>::constraints(const Eigen::VectorXd& x,
//        Eigen::Ref<Eigen::VectorXd> constr) const
//{
//    assert(x.size() == m_num_variables);
//    assert(constr.size() == m_num_constraints);
//    // TODO if (!num_constraints) return true;
//
//    //static const short int constraints_tag = 2;
//    // =====================================================================
//    // START ACTIVE
//    // ---------------------------------------------------------------------
//    trace_on(constraints_tag);
//    VectorXa x_adouble(m_num_variables);
//    // TODO efficiently store this result so it can be used in grad_f, etc.
//    for (unsigned i = 0; i < m_num_variables; ++i) x_adouble[i] <<= x[i];
//    VectorXa g_adouble(m_num_constraints);
//    mconstraints(x_adouble, g_adouble);
//    for (unsigned i = 0; i < m_num_constraints; ++i) g_adouble[i] >>= constr[i];
//    trace_off();
//    // ---------------------------------------------------------------------
//    // END ACTIVE
//    // =====================================================================
//}


//template<typename T>
//void OptimizationProblem<T>::objective(
//        const VectorX<T>& x, T& obj_value) const
//{
//    objective_impl(x, obj_value);
//}

//template<typename T>
//void OptimizationProblem<T>::constraints(const VectorX<T>& x,
//        Eigen::Ref<VectorX<T>> constr) const
//{
//    constraints_impl(x, constr);
//}

OptimizationProblem<adouble>::Proxy::~Proxy() {
    if (m_jacobian_row_indices) {
        delete [] m_jacobian_row_indices;
        m_jacobian_row_indices = nullptr;
    }
    if (m_jacobian_col_indices) {
        delete [] m_jacobian_col_indices;
        m_jacobian_col_indices = nullptr;
    }
}

void OptimizationProblem<adouble>::Proxy::
sparsity(const Eigen::VectorXd& x,
        std::vector<unsigned int>& jacobian_row_indices,
        std::vector<unsigned int>& jacobian_col_indices,
        std::vector<unsigned int>& hessian_row_indices,
        std::vector<unsigned int>& hessian_col_indices) const
{
    // TODO check their sizes.
    assert(x.size() == num_variables());

    // Create tapes and determine sparsity patterns.
    // ---------------------------------------------
    double obj_value; // We don't actually need the obj. value.
    trace_objective(m_objective_tag, num_variables(), x.data(), obj_value);

    // Determine sparsity patterns.
    // ----------------------------
    // TODO allow user to provide multiple points at which to determine
    // sparsity?
    // TODO or can I reuse the tape?
    /* TODO if (m_num_constraints)*/ {
        Eigen::VectorXd constraint_values(num_constraints()); // Unused.
        trace_constraints(m_constraints_tag,
                num_variables(), x.data(),
                num_constraints(), constraint_values.data());

        int repeated_call = 0; // this is the first call at this value of x.
        double* jacobian_values = nullptr; // Unused.
        int success = ::sparse_jac(m_constraints_tag, num_constraints(),
                num_variables(), repeated_call, x.data(),
                // The next 4 arguments are outputs.
                &m_jacobian_num_nonzeros,
                &m_jacobian_row_indices, &m_jacobian_col_indices,
                &jacobian_values,
                const_cast<int*>(m_sparse_jac_options.data()));
        //assert(success == 3);
        assert(success >= 0);
        delete [] jacobian_values;
        jacobian_row_indices.resize(m_jacobian_num_nonzeros);
        jacobian_col_indices.resize(m_jacobian_num_nonzeros);
        // Copy ADOL-C's sparsity memory into MESH's sparsity memory.
        std::copy(m_jacobian_row_indices,
                m_jacobian_row_indices + m_jacobian_num_nonzeros,
                jacobian_row_indices.data());
        std::copy(m_jacobian_col_indices,
                m_jacobian_col_indices + m_jacobian_num_nonzeros,
                jacobian_col_indices.data());
    }

    {
        short int tag = 0;
        // =================================================================
        // START ACTIVE
        // -----------------------------------------------------------------
        trace_on(tag);
        VectorXa x_adouble(num_variables());
        auto lambda_vector = Eigen::VectorXd::Ones(num_constraints());
        adouble lagrangian_adouble;
        double lagr;
        for (unsigned i = 0; i < num_variables(); ++i) {
            x_adouble[i] <<= x[i];
        }
        lagrangian(1.0, x_adouble, lambda_vector, lagrangian_adouble);
        lagrangian_adouble >>= lagr;
        trace_off();
        // -----------------------------------------------------------------
        // END ACTIVE
        // =================================================================
        // TODO efficiently use the "repeat" argument.
        int repeated_call = 0;
        int options[2];
        options[0] = 0; /* test the computational graph control flow? TODO*/
        options[1] = 0; /* way of recovery TODO */
        unsigned int* row_indices = NULL;
        unsigned int* col_indices = NULL;
        double* hessian = NULL; // We don't actually need the hessian...
        // TODO use hess_pat instead!!!
        int num_nonzeros;
        //VectorXd x_and_lambda(m_num_variables + m_num_constraints);
        //for (unsigned ivar = 0; ivar < m_num_variables; ++ivar) {
        //    x_and_lambda[ivar] = guess[ivar];
        //}
        //for (unsigned icon = 0; icon < m_num_constraints; ++icon) {
        //    x_and_lambda[icon + m_num_variables] = 1; // TODO consistency?
        //}
        int success = ::sparse_hess(tag, num_variables(),
                repeated_call, x.data(), &num_nonzeros,
                &row_indices, &col_indices, &hessian,
                options);
        // TODO See ADOL-C manual Table 1 to interpret the return value.
        // TODO improve error handling.
        assert(success);
        hessian_row_indices.resize(num_nonzeros);
        hessian_col_indices.resize(num_nonzeros);
        for (int i = 0; i < num_nonzeros; ++i) {
            hessian_row_indices[i] = row_indices[i];
            hessian_col_indices[i] = col_indices[i];
        }
        // TODO try to use modern memory management.
        delete [] row_indices;
        delete [] col_indices;
        delete [] hessian;
    }
}

void OptimizationProblem<adouble>::Proxy::
objective(unsigned num_variables, const double* x,
        bool /*new_x*/,
        double& obj_value) const
{
    int status = ::function(m_objective_tag,
            1, // number of dependent variables.
            num_variables, // number of independent variables.
            // The signature of ::function() should take a const double*; I'm
            // fairly sure ADOL-C won't try to edit the independent variables.
            const_cast<double*>(x), &obj_value);
    // TODO create fancy return value checking (create a class for it).
    // check_adolc_driver_return_value(status);
    //assert(status == 3);
    assert(status >= 0);
    // TODO if status != 3, retape.
}

void OptimizationProblem<adouble>::Proxy::
constraints(unsigned num_variables, const double* variables,
        bool /*new_variables*/,
        unsigned num_constraints, double* constr) const
{
    // TODO cache constraint values? we are ignoring new_variables.
    // Evaluate the constraints tape.
    int status = ::function(m_constraints_tag,
            num_constraints, // number of dependent variables.
            num_variables, // number of independent variables.
            // The signature of ::function() should take a const double*; I'm
            // fairly sure ADOL-C won't try to edit the independent variables.
            const_cast<double*>(variables), constr);
    //assert(status == 3);
    assert(status >= 0);
}

void OptimizationProblem<adouble>::Proxy::
gradient(unsigned num_variables, const double* x, bool /*new_x*/,
        double* grad) const
{
    int status = ::gradient(m_objective_tag, num_variables, x, grad);
    assert(status); // TODO error codes can be -2,-1,0,1,2,3; improve assert!
}

void OptimizationProblem<adouble>::Proxy::
jacobian(unsigned num_variables, const double* x, bool /*new_x*/,
        unsigned /*num_nonzeros*/, double* jacobian_values) const
{
    int repeated_call = 1; // We already have the sparsity structure.
    int status = ::sparse_jac(m_constraints_tag, num_constraints(),
            num_variables, repeated_call, x,
            &m_jacobian_num_nonzeros,
            &m_jacobian_row_indices, &m_jacobian_col_indices,
            &jacobian_values, const_cast<int*>(m_sparse_jac_options.data()));
    // TODO create enums for ADOL-C's return values.
    //assert(status == 3);
    assert(status >= 0);

    // TODO if we call with repeated_call == 0, we must first delete the
    // previous memory for row indices, etc.
}

void OptimizationProblem<adouble>::Proxy::
hessian_lagrangian(unsigned num_variables, const double* x,
        bool /*new_x*/, double obj_factor,
        unsigned num_constraints, const double* lambda,
        bool /*new_lambda TODO */,
        unsigned num_nonzeros, double* nonzeros) const
{

    // TODO this hessian must include the constraint portion!!!
    // TODO if not new_x, then do NOT re-eval objective()!!!

    // TODO remove from here and utilize new_x.
    // TODO or can I reuse the tape?
    short int tag = 0;
    // -----------------------------------------------------------------
    // START ACTIVE
    trace_on(tag);
    VectorXa x_adouble(num_variables);
    VectorXd lambda_vector(num_constraints); // TODO use Eigen::Map.
    adouble lagrangian_adouble;
    double lagr;
    for (unsigned ivar = 0; ivar < num_variables; ++ivar) {
        // TODO add this operator for std::vector.
        x_adouble[ivar] <<= x[ivar];
    }
    for (unsigned icon = 0; icon < num_constraints; ++icon) {
        lambda_vector[icon] = lambda[icon];
    }
    // TODO use ADOLC's set_param_vec().
    lagrangian(obj_factor, x_adouble, lambda_vector, lagrangian_adouble);
    lagrangian_adouble >>= lagr;
    trace_off();
    // END ACTIVE
    // -----------------------------------------------------------------
    // TODO efficiently use the "repeat" argument.
    int repeated_call = 0;
    int options[2];
    options[0] = 0; /* test the computational graph control flow? TODO*/
    options[1] = 0; /* way of recovery TODO */
    // TODO make general:
    unsigned int* row_indices = NULL;
    unsigned int* col_indices = NULL;
    // TODO hope that the row indices are the same between IpOopt and
    // ADOL-C.
    double* vals = NULL;
    int num_nz;
    // TODO compute sparse hessian for each element of the constraint
    // vector....TODO trace with respect to both x and lambda..
    // http://list.coin-or.org/pipermail/adol-c/2013-April/000900.html
    // TODO "since lambda changes, the Lagrangian function has to be
    // repated every time ...cannot set repeat = 1"
    // The following link suggests more efficient methods:
    // http://list.coin-or.org/pipermail/adol-c/2013-April/000903.html
    // Quote:
    // We made the experience that it really depends on the application
    // whether
    //
    // * tracing the Lagrangian once with x and lambda as inputs
    //    and evaluating only a part of the Hessian reusing the trace
    //       in all iterations
    //
    // or
    //
    // *  retracing the Lagrangian with x as adoubles and lambda as doubles
    // in each iteration and computing then the whole Hessian
    //
    // performs better in terms of runtime. You could give both approaches
    // a try and see what works better for you. Both approaches have their
    // pros and cons with respect to efficiency.
    //

    //std::vector<double> x_and_lambda(num_variables + num_constraints);
    //for (unsigned ivar = 0; ivar < num_variables; ++ivar) {
    //    x_and_lambda[ivar] = x[ivar];
    //}
    //for (unsigned icon = 0; icon < num_constraints; ++icon) {
    //    x_and_lambda[icon + num_variables] = lambda[icon];
    //}
    int success = sparse_hess(tag, num_variables, repeated_call,
            x, &num_nz, &row_indices, &col_indices,
            &vals, options);
    assert(success);
    for (unsigned i = 0; i < num_nonzeros; ++i) {
        nonzeros[i] = vals[i];
    }
    // TODO try to use modern memory management.
    delete [] row_indices;
    delete [] col_indices;
    // TODO avoid reallocating vals each time!!!
    delete [] vals;
}

void OptimizationProblem<adouble>::Proxy::
trace_objective(short int tag,
        unsigned num_variables, const double* x,
        double& obj_value) const
{
//    assert(x.size() == m_num_variables);
    // =====================================================================
    // START ACTIVE
    // ---------------------------------------------------------------------
    trace_on(tag);
    VectorXa x_adouble(num_variables);
    adouble f_adouble = 0;
    for (unsigned i = 0; i < num_variables; ++i) x_adouble[i] <<= x[i];
    m_problem.objective(x_adouble, f_adouble);
    f_adouble >>= obj_value;
    trace_off();
    // ---------------------------------------------------------------------
    // END ACTIVE
    // =====================================================================
}

void OptimizationProblem<adouble>::Proxy::
trace_constraints(short int tag,
        unsigned num_variables, const double* x,
        unsigned num_constraints, double* constr) const
{
//    assert(x.size() == m_num_variables);
//    assert(constr.size() == m_num_constraints);
    // TODO if (!num_constraints) return true;
    // =====================================================================
    // START ACTIVE
    // ---------------------------------------------------------------------
    trace_on(tag);
    VectorXa x_adouble(num_variables);
    // TODO efficiently store this result so it can be used in grad_f, etc.
    for (unsigned i = 0; i < num_variables; ++i) x_adouble[i] <<= x[i];
    VectorXa g_adouble(num_constraints);
    m_problem.constraints(x_adouble, g_adouble);
    for (unsigned i = 0; i < num_constraints; ++i) g_adouble[i] >>= constr[i];
    trace_off();
    // ---------------------------------------------------------------------
    // END ACTIVE
    // =====================================================================
}

void OptimizationProblem<adouble>::Proxy::
lagrangian(double obj_factor, const VectorXa& x,
        const Eigen::VectorXd& lambda, adouble& result) const
{
    assert(x.size() == num_variables());
    assert(lambda.size() == num_constraints());

    result = 0;
    // TODO should not compute obj if obj_factor = 0 but this messes up with
    // ADOL-C.
    //if (obj_factor != 0) {
    //    objective(x, result);
    //    result *= obj_factor;
    //}
    m_problem.objective(x, result);
    result *= obj_factor;

    // TODO if (!m_num_constraints) return;
    VectorXa constr(num_constraints());
    m_problem.constraints(x, constr); // TODO ...?
    // TODO it's highly unlikely that this works:
    // TODO result += lambda.dot(constr);
    for (unsigned icon = 0; icon < num_constraints(); ++icon) {
        result += lambda[icon] * constr[icon];
    }
}

// Explicit instantiation.
template class OptimizationProblem<adouble>;
// TODO extern to avoid implicit instantiation and improve compile time?
} // namespace mesh
