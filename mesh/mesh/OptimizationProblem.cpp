#include "OptimizationProblem.hpp"

namespace {
/*static*/ const short int objective_tag = 1;
/*static*/ const short int constraints_tag = 2;
}

namespace mesh {

template<>
void OptimizationProblem<adouble>::objective(
        const Eigen::VectorXd& x, double& obj_value) const
{
    assert(x.size() == m_num_variables);
    // TODO where to store this tag? the proxy would be a good location.
    // TODO b/c we can't add a member variable to the template class.
    //static const short int objective_tag = 1;
    // =====================================================================
    // START ACTIVE
    // ---------------------------------------------------------------------
    trace_on(objective_tag);
    VectorXa x_adouble(m_num_variables);
    adouble f_adouble = 0;
    for (unsigned i = 0; i < m_num_variables; ++i) x_adouble[i] <<= x[i];
    objective_impl(x_adouble, f_adouble);
    f_adouble >>= obj_value;
    trace_off();
    // ---------------------------------------------------------------------
    // END ACTIVE
    // =====================================================================
}

template<>
void OptimizationProblem<adouble>::constraints(const Eigen::VectorXd& x,
        Eigen::Ref<Eigen::VectorXd> constr) const
{
    assert(x.size() == m_num_variables);
    assert(constr.size() == m_num_constraints);
    // TODO if (!num_constraints) return true;

    //static const short int constraints_tag = 2;
    // =====================================================================
    // START ACTIVE
    // ---------------------------------------------------------------------
    trace_on(constraints_tag);
    VectorXa x_adouble(m_num_variables);
    // TODO efficiently store this result so it can be used in grad_f, etc.
    for (unsigned i = 0; i < m_num_variables; ++i) x_adouble[i] <<= x[i];
    VectorXa g_adouble(m_num_constraints);
    constraints_impl(x_adouble, g_adouble);
    for (unsigned i = 0; i < m_num_constraints; ++i) g_adouble[i] >>= constr[i];
    trace_off();
    // ---------------------------------------------------------------------
    // END ACTIVE
    // =====================================================================
}


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

template<>
void OptimizationProblem<adouble>::determine_sparsity(const Eigen::VectorXd& x,
        std::vector<unsigned int>& jacobian_row_indices,
        std::vector<unsigned int>& jacobian_col_indices,
        std::vector<unsigned int>& hessian_row_indices,
        std::vector<unsigned int>& hessian_col_indices) const
{
    // TODO check their sizes.
    assert(x.size() == m_num_variables);

    // Determine sparsity patterns.
    // ----------------------------
    // TODO allow user to provide multiple points at which to determine
    // sparsity?
    // TODO or can I reuse the tape?
    /* TODO if (m_num_constraints)*/ {
        //short int tag = 0;
        Eigen::VectorXd constraint_values(m_num_constraints); // Unused.
        //trace_constraints(tag, m_num_variables, x.data(),
        //        m_num_constraints, constraint_values.data());
        constraints(x, constraint_values);

        // TODO use jac_pat function instead.
        int repeated_call = 0;
        int num_nonzeros = -1; /*TODO*/
        unsigned int* row_indices = NULL; // Allocated by ADOL-C.
        unsigned int* col_indices = NULL; // Allocated by ADOL-C.
        double* jacobian = NULL;          // Allocated by ADOL-C.
        int options[4];
        options[0] = 0; /*TODO*/
        options[1] = 0; /*TODO*/
        options[2] = 0; /*TODO*/
        options[3] = 0; /*TODO*/
        int success = sparse_jac(constraints_tag, m_num_constraints,
                m_num_variables, repeated_call, x.data(),
                &num_nonzeros, &row_indices, &col_indices,
                &jacobian, options);
        assert(success);
        jacobian_row_indices.reserve(num_nonzeros);
        jacobian_col_indices.reserve(num_nonzeros);
        for (int i = 0; i < num_nonzeros; ++i) {
            jacobian_row_indices[i] = row_indices[i];
            jacobian_col_indices[i] = col_indices[i];
        }
        delete [] row_indices;
        delete [] col_indices;
        delete [] jacobian;
    }

    {
        //short int tag = 0;
        // =================================================================
        // START ACTIVE
        // -----------------------------------------------------------------
        trace_on(tag);
        VectorXa x_adouble(m_num_variables);
        Eigen::VectorXd lambda_vector(m_num_constraints, 1);
        adouble lagrangian_adouble;
        double lagr;
        for (unsigned i = 0; i < m_num_variables; ++i) {
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
        int success = sparse_hess(lagrangian_tag, m_num_variables,
                repeated_call, x.data(), &num_nonzeros,
                &row_indices, &col_indices, &hessian,
                options);
        // TODO See ADOL-C manual Table 1 to interpret the return value.
        // TODO improve error handling.
        assert(success);
        hessian_row_indices.reserve(num_nonzeros);
        hessian_col_indices.reserve(num_nonzeros);
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

// Explicit instantiation.
template class OptimizationProblem<adouble>;
// TODO extern to avoid implicit instantiation and improve compile time?

} // namespace mesh
