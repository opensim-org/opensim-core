#include "OptimizationProblem.h"

#ifdef _MSC_VER
// Ignore warnings from ADOL-C headers.
    #pragma warning(push)
    // 'argument': conversion from 'size_t' to 'locint', possible loss of data.
    #pragma warning(disable: 4267)
#endif
// TODO put adolc sparsedrivers in their own namespace. tropter::adolc
#include <adolc/adolc.h>
#include <adolc/sparse/sparsedrivers.h>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

using Eigen::VectorXd;
using Eigen::Ref;

namespace tropter {

OptimizationProblem<adouble>::Decorator::Decorator(
        const OptimizationProblem<adouble>& problem) :
        OptimizationProblemDecorator(problem), m_problem(problem)
{
    // Use 0 (default) for all 4 options to ADOL-C's sparse_jac().
    // [0]: Way of sparsity pattern computation (propagation of index domains).
    // [1]: Test the computational graph control flow (safe mode).
    // [2]: Way of bit pattern propagation (automatic detection).
    // [3]: Way of compression (column compression).
    m_sparse_jac_options = {0, 0, 0, 0};

    // Test the computational graph control flow (safe mode).
    // This finds more nonzeros than necessary, but will generate a
    // sparsity pattern that should be valid across all possible variables.
    m_sparse_hess_options.resize(2);
    m_sparse_hess_options[0] = 0;
    // Way of recovery (indirect). This is a setting for ColPack.
    // TODO try using direct recovery (1) also.
    m_sparse_hess_options[1] = 0;
}

OptimizationProblem<adouble>::Decorator::~Decorator() {
    if (m_jacobian_row_indices) {
        delete [] m_jacobian_row_indices;
        m_jacobian_row_indices = nullptr;
    }
    if (m_jacobian_col_indices) {
        delete [] m_jacobian_col_indices;
        m_jacobian_col_indices = nullptr;
    }
    if (m_hessian_row_indices) {
        delete [] m_hessian_row_indices;
        m_hessian_row_indices = nullptr;
    }
    if (m_hessian_col_indices) {
        delete [] m_hessian_col_indices;
        m_hessian_col_indices = nullptr;
    }
}

void OptimizationProblem<adouble>::Decorator::
calc_sparsity(const Eigen::VectorXd& x,
        std::vector<unsigned int>& jacobian_row_indices,
        std::vector<unsigned int>& jacobian_col_indices,
        std::vector<unsigned int>& hessian_row_indices,
        std::vector<unsigned int>& hessian_col_indices) const
{
    const auto& num_variables = get_num_variables();
    assert(x.size() == num_variables);
    const auto& num_constraints = get_num_constraints();

    // This function also creates the ADOL-C tapes that are used in the other
    // function calls.

    // Objective.
    // ----------
    {
        double obj_value; // We don't actually need the obj. value.
        trace_objective(m_objective_tag, num_variables, x.data(), obj_value);
    }

    // Jacobian.
    // ---------
    // TODO allow user to provide multiple points at which to determine
    // sparsity?
    // TODO if (m_num_constraints)
    {
        Eigen::VectorXd constraint_values(num_constraints); // Unused.
        trace_constraints(m_constraints_tag,
                num_variables, x.data(),
                num_constraints, constraint_values.data());

        int repeated_call = 0; // No previous call, need to create tape.
        double* jacobian_values = nullptr; // Unused.
        int success = ::sparse_jac(m_constraints_tag, num_constraints,
                num_variables, repeated_call, x.data(),
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
        // Copy ADOL-C's sparsity memory into Tropter's sparsity memory.
        std::copy(m_jacobian_row_indices,
                m_jacobian_row_indices + m_jacobian_num_nonzeros,
                jacobian_row_indices.data());
        std::copy(m_jacobian_col_indices,
                m_jacobian_col_indices + m_jacobian_num_nonzeros,
                jacobian_col_indices.data());
        // TODO don't duplicate the memory consumption for storing the sparsity
        // pattern: store the pointer to Ipopt's sparsity pattern?

        //std::ofstream file("DEBUG_jacobian_sparsity.csv");
        //file << "row_indices,column_indices" << std::endl;
        //for (int i = 0; i < (int)jacobian_row_indices.size(); ++i) {
        //    file << jacobian_row_indices[i] << "," << jacobian_col_indices[i]
        //            << std::endl;
        //}
        //file.close();
    }

    // Lagrangian.
    // -----------
    if (m_problem.get_use_supplied_sparsity_hessian_lagrangian()) {
        throw std::runtime_error("Cannot use supplied sparsity pattern for "
                "Hessian of Lagrangian when using automatic differentiation.");
    }
    {
        VectorXd lambda_vector = Eigen::VectorXd::Ones(num_constraints);
        double lagr_value; // Unused.
        trace_lagrangian(m_lagrangian_tag, num_variables, x.data(), 1.0,
                num_constraints, lambda_vector.data(), lagr_value);
        int repeated_call = 0; // No previous call, need to create tape.
        double* hessian_values = nullptr; // Unused.
        int status = ::sparse_hess(m_lagrangian_tag, num_variables,
                repeated_call, x.data(), &m_hessian_num_nonzeros,
                &m_hessian_row_indices, &m_hessian_col_indices,
                &hessian_values,
                // TODO &hessian_values,
                const_cast<int*>(m_sparse_hess_options.data()));

        int optTODO = 0;
        unsigned int** hess_pattern = (unsigned int **) malloc
                (num_variables*sizeof(unsigned int*));
        ::hess_pat(m_lagrangian_tag, num_variables, x.data(), hess_pattern,
                optTODO);
        int num_seeds;
        double** seed;
        std::cout << "DEBUG ADOL-C's hess_pat " << std::endl;
        for (int ivar = 0; ivar < num_variables; ++ivar) {
            for (int j = 0; j < hess_pattern[ivar][0]; ++j) {
                std::cout << hess_pattern[ivar][j+1] << " ";
            }
            std::cout << std::endl;
        }
        ::generate_seed_hess(num_variables, hess_pattern, &seed, &num_seeds,
                optTODO);
        std::cout << "DEBUG ADOL-C's seed" << std::endl;
        for (int ivar = 0; ivar < num_variables; ++ivar) {
            for (int iseed = 0; iseed < num_seeds; ++iseed) {
                std::cout << seed[ivar][iseed] << " ";
            }
            std::cout << std::endl;
        }
        // TODO See ADOL-C manual Table 1 to interpret the return value.
        // TODO improve error handling.
        assert(status >= 0);
        delete [] hessian_values;
        hessian_row_indices.resize(m_hessian_num_nonzeros);
        hessian_col_indices.resize(m_hessian_num_nonzeros);
        std::copy(m_hessian_row_indices,
                m_hessian_row_indices + m_hessian_num_nonzeros,
                hessian_row_indices.data());
        std::copy(m_hessian_col_indices,
                m_hessian_col_indices + m_hessian_num_nonzeros,
                hessian_col_indices.data());
        // TODO don't duplicate the memory consumption for storing the sparsity
        // pattern: store the pointer to Ipopt's sparsity pattern?

        // Working memory to hold obj_factor and lambda (multipliers).
        m_hessian_obj_factor_lambda.resize(1 + num_constraints);

        //std::ofstream file("DEBUG_hessian_lagrangian_sparsity.csv");
        //file << "row_indices,column_indices" << std::endl;
        //for (int i = 0; i < (int)hessian_row_indices.size(); ++i) {
        //    file << hessian_row_indices[i] << "," << hessian_col_indices[i]
        //            << std::endl;
        //}
        //file.close();
    }
}

void OptimizationProblem<adouble>::Decorator::
calc_objective(unsigned num_variables, const double* x,
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

void OptimizationProblem<adouble>::Decorator::
calc_constraints(unsigned num_variables, const double* variables,
        bool /*new_variables*/,
        unsigned num_constraints, double* constr) const
{
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

void OptimizationProblem<adouble>::Decorator::
calc_gradient(unsigned num_variables, const double* x, bool /*new_x*/,
        double* grad) const
{
    int status = ::gradient(m_objective_tag, num_variables, x, grad);
    assert(status); // TODO error codes can be -2,-1,0,1,2,3; improve assert!
}

void OptimizationProblem<adouble>::Decorator::
calc_jacobian(unsigned num_variables, const double* x, bool /*new_x*/,
        unsigned /*num_nonzeros*/, double* jacobian_values) const
{
    int repeated_call = 1; // We already have the sparsity structure.
    int status = ::sparse_jac(m_constraints_tag, get_num_constraints(),
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

void OptimizationProblem<adouble>::Decorator::
calc_hessian_lagrangian(unsigned num_variables, const double* x,
        bool /*new_x*/, double obj_factor,
        unsigned num_constraints, const double* lambda,
        bool /*new_lambda TODO */,
        unsigned /*num_nonzeros*/, double* hessian_values) const
{
    // TODO if not new_x, then do NOT re-eval objective()!!!

    int repeated_call = 1;
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

    // Update the passive parameters.
    m_hessian_obj_factor_lambda[0] = obj_factor;
    std::copy(lambda, lambda + num_constraints,
            m_hessian_obj_factor_lambda.begin() + 1);
    set_param_vec(m_lagrangian_tag, 1 + num_constraints,
            m_hessian_obj_factor_lambda.data());

    int status = sparse_hess(m_lagrangian_tag, num_variables, repeated_call,
            x, &m_hessian_num_nonzeros, &m_hessian_row_indices,
            &m_hessian_col_indices,
            &hessian_values,
            const_cast<int*>(m_sparse_hess_options.data()));
    assert(status >= 0);
}

void OptimizationProblem<adouble>::Decorator::
trace_objective(short int tag,
        unsigned num_variables, const double* x,
        double& obj_value) const
{
    // =========================================================================
    // START ACTIVE
    // -------------------------------------------------------------------------
    trace_on(tag);
    VectorXa x_adouble(num_variables);
    adouble f_adouble = 0;
    for (unsigned i = 0; i < num_variables; ++i) x_adouble[i] <<= x[i];
    m_problem.calc_objective(x_adouble, f_adouble);
    f_adouble >>= obj_value;
    trace_off();
    // -------------------------------------------------------------------------
    // END ACTIVE
    // =========================================================================
}

void OptimizationProblem<adouble>::Decorator::
trace_constraints(short int tag,
        unsigned num_variables, const double* x,
        unsigned num_constraints, double* constr) const
{
    // TODO if (!num_constraints) return true;
    // =========================================================================
    // START ACTIVE
    // -------------------------------------------------------------------------
    trace_on(tag);
    VectorXa x_adouble(num_variables);
    // TODO efficiently store this result so it can be used in grad_f, etc.
    for (unsigned i = 0; i < num_variables; ++i) x_adouble[i] <<= x[i];
    VectorXa g_adouble(num_constraints);
    m_problem.calc_constraints(x_adouble, g_adouble);
    for (unsigned i = 0; i < num_constraints; ++i) g_adouble[i] >>= constr[i];
    trace_off();
    // -------------------------------------------------------------------------
    // END ACTIVE
    // =========================================================================
}

void OptimizationProblem<adouble>::Decorator::
trace_lagrangian(short int tag,
        unsigned num_variables, const double* x, const double& obj_factor,
        unsigned num_constraints, const double* lambda,
        double& lagrangian_value) const {
    // =========================================================================
    // START ACTIVE
    // -------------------------------------------------------------------------
    trace_on(tag);
    VectorXa x_adouble(num_variables);
    VectorXd lambda_vector = Eigen::VectorXd::Map(lambda, num_constraints);
    adouble lagrangian_adouble;
    for (unsigned i = 0; i < num_variables; ++i) x_adouble[i] <<= x[i];

    // TODO should not compute obj if obj_factor = 0 but this messes up with
    // ADOL-C.
    //if (obj_factor != 0) {
    //    objective(x, result);
    //    result *= obj_factor;
    //}
    m_problem.calc_objective(x_adouble, lagrangian_adouble);
    // TODO make sure not to create more params if trace_lagrangian is called
    // multiple times.
    lagrangian_adouble *= ::mkparam(obj_factor);

    // TODO if (!m_num_constraints) return;
    VectorXa constr(num_constraints);
    // TODO ...might not need all constraints.
    m_problem.calc_constraints(x_adouble, constr);
    // TODO it's highly unlikely that this works (can't use with mkparam()).
    // TODO result += lambda.dot(constr);
    for (unsigned icon = 0; icon < num_constraints; ++icon) {
        lagrangian_adouble += ::mkparam(lambda[icon]) * constr[icon];
    }

    lagrangian_adouble >>= lagrangian_value;
    trace_off();
    // -------------------------------------------------------------------------
    // END ACTIVE
    // =========================================================================
}

} // namespace tropter
