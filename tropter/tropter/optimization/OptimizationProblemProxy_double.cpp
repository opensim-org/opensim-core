#include "OptimizationProblem.hpp"
#include <ColPack/ColPackHeaders.h>

using Eigen::VectorXd;

// TODO leave all of this templatized, so floats can be used also.

namespace tropter {

// We must implement the destructor in a context where ColPack's coloring
// class is complete (since it's used in a unique ptr member variable.).
OptimizationProblem<double>::Proxy::~Proxy() {}

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
    //unsigned int num_jacobian_elements = num_constraints() * num_variables();
    //jacobian_row_indices.resize(num_jacobian_elements);
    //jacobian_col_indices.resize(num_jacobian_elements);
    //// Example: the entries of a 2 x 4 Jacobian would be ordered as follows:
    ////          0 2 4 6
    ////          1 3 5 7
    //// This is a column-major storage order.
    //for (unsigned int i = 0; i < num_jacobian_elements; ++i) {
    //    jacobian_row_indices[i] = i % num_constraints();
    //    jacobian_col_indices[i] = i / num_constraints();
    //}

    // Determine the sparsity pattern.
    // -------------------------------
    //SparsityPattern sparsity(num_constraints(), num_variables());
    VectorXd x_working = VectorXd::Zero(num_variables());
    VectorXd constr_working(num_constraints());
    // TODO unsigned int inonzero = 0;
    std::vector<unsigned int> jacobian_row_indices_init;
    std::vector<unsigned int> jacobian_col_indices_init;
    for (int j = 0; j < num_variables(); ++j) {
        constr_working.setZero();
        x_working[j] = std::numeric_limits<double>::quiet_NaN();
        m_problem.constraints(x_working, constr_working);
        x_working[j] = 0;
        //std::cout << std::endl;
        for (int i = 0; i < num_constraints(); ++i) {
            //std::cout << y[i] << std::endl;
            if (std::isnan(constr_working[i])) {
                //sparsity.add_nonzero(i, j);
                jacobian_row_indices_init.push_back(i);
                jacobian_col_indices_init.push_back(j);

                //std::pair<unsigned, unsigned> idxpair(i, j);
                //m_jacobian_nonzero_indices[idxpair] = inonzero;
                //++inonzero;
            }
        }
    }
    const size_t num_jacobian_nonzeros = jacobian_row_indices_init.size();

    // Convert sparsity pattern to ADOL-C compressed row format.
    // ---------------------------------------------------------
    //using UnsignedInt2DPtr =
    //    std::unique_ptr<unsigned*[], std::function<void(unsigned**)>>;
    const auto num_jac_rows = num_constraints();
    const auto num_vars = num_variables();
    auto unsigned_int_2d_deleter = [num_jac_rows](unsigned** x) {
        std::for_each(x, x + num_jac_rows, std::default_delete<unsigned[]>());
        delete [] x;
    };
    m_jacobian_pattern_ADOLC_format = UnsignedInt2DPtr(
            new unsigned*[num_jac_rows], unsigned_int_2d_deleter);
    for (int i = 0; i < num_jac_rows; ++i) {
        std::vector<unsigned int> col_idx_for_nonzeros;
        for (int inz = 0; inz < num_jacobian_nonzeros; ++inz) {
            if (jacobian_row_indices_init[inz] == i) {
                col_idx_for_nonzeros.push_back(jacobian_col_indices_init[inz]);
            }
            const auto num_nonzeros_this_row = col_idx_for_nonzeros.size();
            m_jacobian_pattern_ADOLC_format[i] =
                    new unsigned[num_nonzeros_this_row+1];
            m_jacobian_pattern_ADOLC_format[i][0] = num_nonzeros_this_row;
            std::copy(col_idx_for_nonzeros.begin(), col_idx_for_nonzeros.end(),
                    // Skip over the first element.
                    m_jacobian_pattern_ADOLC_format[i] + 1);
        }
    }

    // Determine the efficient perturbation directions.
    // ------------------------------------------------
    m_jacobian_coloring.reset(
            new ColPack::BipartiteGraphPartialColoringInterface(
                    SRC_MEM_ADOLC, m_jacobian_pattern_ADOLC_format.get(),
                    num_jac_rows, num_vars));
    // No longer need this memory.
    //jac_pattern_ADOLC_format.reset();

    // TODO better memory management.
    double** jacobian_seed_raw = nullptr;
    int jacobian_seed_num_rows; // Should be num_vars.
    int jacobian_seed_num_cols; // Number of seeds.
    m_jacobian_coloring->GenerateSeedJacobian_unmanaged(&jacobian_seed_raw,
            &jacobian_seed_num_rows, &jacobian_seed_num_cols,
            "SMALLEST_LAST", "COLUMN_PARTIAL_DISTANCE_TWO");
    const int num_jacobian_seeds = jacobian_seed_num_cols;
    m_jacobian_seed.resize(jacobian_seed_num_rows, jacobian_seed_num_cols);
    for (int i = 0; i < jacobian_seed_num_rows; ++i) {
        for (int j = 0; j < jacobian_seed_num_cols; ++j) {
            m_jacobian_seed(i, j) = jacobian_seed_raw[i][j];
        }
        delete [] jacobian_seed_raw[i];
    }
    delete [] jacobian_seed_raw;


    // TODO move this class to a member variable.
    m_jacobian_recovery.reset(new ColPack::JacobianRecovery1D());
    // The next two variables are not used here; they are used in jacobian().
    m_jacobian_recovered_row_indices.resize(num_jacobian_nonzeros);
    m_jacobian_recovered_col_indices.resize(num_jacobian_nonzeros);
    // TODO use fancy DoublePtr.
    // TODO dummy memory.
    double** m_jacobian_compressed = new double*[num_constraints()];
    for (int i = 0; i < num_constraints(); ++i) {
        m_jacobian_compressed[i] = new double[num_jacobian_seeds];
    }
    //std::unique_ptr<unsigned int[]> jacobian_row_indices_recovered(
    //        new unsigned int[sparsity.num_nonzeros()]);
    //std::vector<double> jacobian_recovered(sparsity.num_nonzeros());
    //unsigned int* jricd = m_jacobian_recovered_row_indices.data();
    //unsigned int* jcicd = m_jacobian_recovered_col_indices.data();
    jacobian_row_indices.resize(num_jacobian_nonzeros);
    unsigned int* jricd = jacobian_row_indices.data();
    jacobian_col_indices.resize(num_jacobian_nonzeros);
    unsigned int* jcicd = jacobian_col_indices.data();
    //double* jrd = jacobian_recovered.data();
    //unsigned int* jricd = new unsigned int[sparsity.num_nonzeros()];
    //unsigned int* jcicd = new unsigned int[sparsity.num_nonzeros()];
    //double* jrd = new double[sparsity.num_nonzeros()];
    //unsigned int** jricd = new unsigned int*;
    //unsigned int** jcicd = new unsigned int*;
    //double** jrd = new double*;
    std::vector<double> jacobian_values_dummy(num_jacobian_nonzeros);
    double* jacobian_values_dummy_ptr = jacobian_values_dummy.data();
    m_jacobian_recovery->RecoverD2Cln_CoordinateFormat_usermem(
            m_jacobian_coloring.get(),
            m_jacobian_compressed, m_jacobian_pattern_ADOLC_format.get(),
            &jricd, &jcicd, &jacobian_values_dummy_ptr);
    for (int i = 0 ; i < num_jacobian_nonzeros; ++i) {
        std::cout << jacobian_values_dummy[i] << std::endl;
    }

    /*

    // TODO
    const Eigen::Index num_seeds = m_jacobian_seed.cols();
    m_jacobian_seed_info.resize(num_seeds);
    for (int iseed = 0; iseed < num_seeds; ++iseed) {
        for (int j = 0; j < m_jacobian_seed.rows(); ++j) {
            if (m_jacobian_seed(j, iseed) == 1) {
                m_jacobian_seed_info[iseed].push_back(j);
            }
        }
    }

    m_jacobian_sparsity_cc.resize(num_variables());
    for (int j = 0; j < num_variables(); ++j) {
        for (int inz = 0; inz < num_jacobian_nonzeros; ++inz) {
            if (jacobian_col_indices[inz] == j) {
                m_jacobian_sparsity_cc[j].push_back(jacobian_row_indices[inz]);
            }
        }
    }
    */

    // Hessian.
    // ========
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
jacobian(unsigned num_variables, const double* variables, bool /*new_x*/,
        unsigned num_nonzeros, double* jacobian_values) const
{
    /*
    VectorXd x_working = Eigen::Map<const VectorXd>(variables, num_variables);
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
        x_working[icol] = variables[icol] - eps;
        m_problem.constraints(x_working, constr_neg);
        // Restore the original value.
        x_working[icol] = variables[icol];
        jacobian.col(icol) = (constr_pos - constr_neg) / two_eps;
    }
    */

    const double eps = std::sqrt(Eigen::NumTraits<double>::epsilon());
    const double two_eps = 2 * eps;
    const Eigen::Index num_seeds = m_jacobian_seed.cols();
    VectorXd constr_pos(num_constraints()); // TODO avoid reallocating.
    VectorXd constr_neg(num_constraints());
    VectorXd deriv(num_constraints());
    Eigen::Map<const VectorXd> x0(variables, num_variables);
    // TODO put as member variable
    double** m_jacobian_compressed = new double*[num_constraints()];
    for (int i = 0; i < num_constraints(); ++i) {
        m_jacobian_compressed[i] = new double[num_seeds];
    }
    int inonzero = 0;
    for (Eigen::Index iseed = 0; iseed < num_seeds; ++iseed) {
        const auto direction = m_jacobian_seed.col(iseed);
        // Perturb x in the positive direction.
        m_problem.constraints(x0 + eps * direction, constr_pos);
        // Perturb x in the negative direction.
        m_problem.constraints(x0 - eps * direction, constr_neg);
        // Compute central difference.
        deriv = (constr_pos - constr_neg) / two_eps;

        for (int i = 0; i < num_constraints(); ++i) {
            m_jacobian_compressed[i][iseed] = deriv[i];
        }
        /*
        const auto& columns_in_this_seed = m_jacobian_seed_info[iseed];
        for (const auto& ijaccol : columns_in_this_seed) {
            for (const auto& ijacrow : m_jacobian_sparsity_cc[ijaccol]) {
                std::pair<unsigned int, unsigned int> indices(ijacrow, ijaccol);
                const auto& inonzero = m_jacobian_nonzero_indices[indices];
                jacobian_values[inonzero] = deriv_this_seed[ijacrow];
            }
        }
         */

        /*
        for (int i = 0; i < num_constraints(); ++i) {
            jacobian_values[inonzero] = jacobian_column[i];
            ++inonzero;
        }
         */
    }

    // TODO use working memory.
    /*
    double** compressed_jacobian_raw = new double*[num_constraints()];
    for (int i = 0; i < m; ++i) {
        compressed_jacobian_raw[i] = new double[num_seeds];
        // TODO replace with std::copy(compressed_jacobian.)
        for (int j = 0; j < num_seeds; ++j) {
            compressed_jacobian_raw[i][j] = m_jacobian_compressed(i, j);
        }
    }
     */

    // TODO move this class to a member variable.
    std::vector<unsigned int> jacobian_row_indices_recovered(
            num_nonzeros);
    //std::unique_ptr<unsigned int[]> jacobian_row_indices_recovered(
    //        new unsigned int[sparsity.num_nonzeros()]);
    std::vector<unsigned int> jacobian_col_indices_recovered(
            num_nonzeros);
    //std::vector<double> jacobian_recovered(sparsity.num_nonzeros());
    unsigned int* jricd = m_jacobian_recovered_row_indices.data();
    unsigned int* jcicd = m_jacobian_recovered_col_indices.data();
    //double* jrd = jacobian_recovered.data();
    //unsigned int* jricd = new unsigned int[sparsity.num_nonzeros()];
    //unsigned int* jcicd = new unsigned int[sparsity.num_nonzeros()];
    //double* jrd = new double[sparsity.num_nonzeros()];
    //unsigned int** jricd = new unsigned int*;
    //unsigned int** jcicd = new unsigned int*;
    //double** jrd = new double*;
    m_jacobian_recovery->RecoverD2Cln_CoordinateFormat_usermem(
            m_jacobian_coloring.get(),
            m_jacobian_compressed, m_jacobian_pattern_ADOLC_format.get(),
            &jricd, &jcicd, &jacobian_values);
    for (int i = 0 ; i < num_nonzeros; ++i) {
        std::cout << jacobian_values[i] << std::endl;
    }
         // &jrd);
//    for (unsigned int inz = 0; inz < num_nonzeros; ++inz) {
//
//    }

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
