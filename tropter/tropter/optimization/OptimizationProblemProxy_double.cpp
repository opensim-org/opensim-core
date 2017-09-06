#include "OptimizationProblem.hpp"
#include <ColPack/ColPackHeaders.h>

#ifdef TROPTER_WITH_OPENMP
// TODO only include ifdef _OPENMP
    #include <omp.h>
#endif

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
sparsity(const Eigen::VectorXd& /*x*/,
        std::vector<unsigned int>& jacobian_row_indices,
        std::vector<unsigned int>& jacobian_col_indices,
        std::vector<unsigned int>& hessian_row_indices,
        std::vector<unsigned int>& hessian_col_indices) const
{
    const auto num_vars = num_variables();

    // Jacobian.
    // =========
    const auto num_jac_rows = num_constraints();

    // Determine the sparsity pattern.
    // -------------------------------
    // We do this by setting an element of x to NaN, and examining which
    // constraint equations end up as NaN (and therefore depend on that
    // element of x).
    VectorXd x_working = VectorXd::Zero(num_vars);
    VectorXd constr_working(num_jac_rows);
    // Initially, we store the sparsity structure in ADOL-C's compressed row
    // format, since this is what ColPack accepts.
    // This format, as described in the ADOL-C manual, is a 2-Dish array.
    // The length of the first dimension is the number of rows in the Jacobian.
    // The first element of each row is the number of nonzeros in that row of
    // the Jacobian. The remaining elements are the column indices of those
    // nonzeros. The length of each row (the second dimension) is
    // num_nonzeros_in_the_row + 1.
    std::vector<std::vector<unsigned int>> jacobian_sparsity_temp(num_jac_rows);
    size_t num_jacobian_nonzeros = 0;
    for (int j = 0; j < (int)num_vars; ++j) {
        constr_working.setZero();
        x_working[j] = std::numeric_limits<double>::quiet_NaN();
        m_problem.constraints(x_working, constr_working);
        x_working[j] = 0;
        for (int i = 0; i < (int)num_jac_rows; ++i) {
            if (std::isnan(constr_working[i])) {
                jacobian_sparsity_temp[i].push_back(j);
                ++num_jacobian_nonzeros;
            }
        }
    }

    // Store the sparsity pattern in a raw C array, since this is what
    // ColPack requires.
    // Create a lambda that deletes the 2D C array.
    auto unsigned_int_2d_deleter = [num_jac_rows](unsigned** x) {
        std::for_each(x, x + num_jac_rows, std::default_delete<unsigned[]>());
        delete [] x;
    };
    m_jacobian_pattern_ADOLC_format = UnsignedInt2DPtr(
            new unsigned*[num_jac_rows], unsigned_int_2d_deleter);
    for (int i = 0; i < (int)num_jac_rows; ++i) {
        const auto& col_idx_for_nonzeros = jacobian_sparsity_temp[i];
        const auto num_nonzeros_this_row = col_idx_for_nonzeros.size();
        m_jacobian_pattern_ADOLC_format[i] =
                new unsigned[num_nonzeros_this_row+1];
        m_jacobian_pattern_ADOLC_format[i][0] = (unsigned)num_nonzeros_this_row;
        std::copy(col_idx_for_nonzeros.begin(), col_idx_for_nonzeros.end(),
                // Skip over the first element.
                m_jacobian_pattern_ADOLC_format[i] + 1);
    }

    // Determine the efficient perturbation directions.
    // ------------------------------------------------
    m_jacobian_coloring.reset(
            new ColPack::BipartiteGraphPartialColoringInterface(
                    SRC_MEM_ADOLC, // We're using the ADOLC sparsity format.
                    m_jacobian_pattern_ADOLC_format.get(), // Sparsity.
                    num_jac_rows, num_vars));

    // ColPack will allocate and store the seed matrix in jacobian_seed_raw.
    double** jacobian_seed_raw = nullptr;
    int jacobian_seed_num_rows; // Should be num_vars.
    int jacobian_seed_num_cols; // Number of seeds.
    m_jacobian_coloring->GenerateSeedJacobian_unmanaged(&jacobian_seed_raw,
            &jacobian_seed_num_rows, &jacobian_seed_num_cols, // Outputs.
            // Copied from what ADOL-C uses in generate_seed_jac():
            "SMALLEST_LAST", "COLUMN_PARTIAL_DISTANCE_TWO");
    // Convert the seed matrix into an Eigen Matrix for ease of use; delete
    // the memory that ColPack created for the seed matrix.
    const int num_jacobian_seeds = jacobian_seed_num_cols;
    std::cout << "[tropter] Number of finite difference perturbations required "
            "for sparse Jacobian: " << num_jacobian_seeds << std::endl;
    m_jacobian_seed.resize(jacobian_seed_num_rows, jacobian_seed_num_cols);
    for (int i = 0; i < jacobian_seed_num_rows; ++i) {
        for (int j = 0; j < jacobian_seed_num_cols; ++j) {
            m_jacobian_seed(i, j) = jacobian_seed_raw[i][j];
        }
        delete [] jacobian_seed_raw[i];
    }
    delete [] jacobian_seed_raw;


    // Obtain sparsity pattern format to return.
    // -----------------------------------------
    // Provide Jacobian row and column indices in the same order that ColPack
    // will use in jacobian() when recovering the sparse Jacobian from the
    // dense compressed Jacobian.
    // Allocate the recovery object used in jacobian().
    m_jacobian_recovery.reset(new ColPack::JacobianRecovery1D());
    // Allocate memory for the compressed jacobian; we use this in jacobian().
    // Create a lambda that deletes the 2D C array.
    auto double_2d_deleter = [num_jac_rows](double** x) {
        std::for_each(x, x + num_jac_rows, std::default_delete<double[]>());
        delete [] x;
    };
    m_jacobian_compressed = Double2DPtr(new double*[num_jac_rows],
                                        double_2d_deleter);
    for (int i = 0; i < (int)num_jac_rows; ++i) {
        // We don't actually care about the value of m_jacobian_compressed here;
        // no need to set values.
        m_jacobian_compressed[i] = new double[num_jacobian_seeds];
    }
    // Our objective here is to set these vectors from the recovery routine.
    jacobian_row_indices.resize(num_jacobian_nonzeros);
    jacobian_col_indices.resize(num_jacobian_nonzeros);
    unsigned int* jac_row_ptr = jacobian_row_indices.data();
    unsigned int* jac_col_ptr = jacobian_col_indices.data();
    // Again, we don't actually need Jacobian values right now.
    std::vector<double> jacobian_values_dummy(num_jacobian_nonzeros);
    double* jacobian_values_dummy_ptr = jacobian_values_dummy.data();
    m_jacobian_recovery->RecoverD2Cln_CoordinateFormat_usermem(
            m_jacobian_coloring.get(),
            m_jacobian_compressed.get(), m_jacobian_pattern_ADOLC_format.get(),
            &jac_row_ptr, &jac_col_ptr, &jacobian_values_dummy_ptr);

    // Allocate memory that is used in jacobian().
    m_constr_pos.resize(num_jac_rows);
    m_constr_neg.resize(num_jac_rows);
    m_jacobian_compressed_column.resize(num_jac_rows);
    // Used as dummy variables:
    m_jacobian_recovered_row_indices.resize(num_jacobian_nonzeros);
    m_jacobian_recovered_col_indices.resize(num_jacobian_nonzeros);


    // Hessian.
    // ========
    // Exact hessian mode is unsupported for now.
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
    m_x_working = Eigen::Map<const VectorXd>(variables, num_variables);
    VectorXd constrvec(num_constraints); // TODO avoid copy.
    // TODO at least keep constrvec as working memory.
    m_problem.constraints(m_x_working, constrvec);
    // TODO avoid copy.
    std::copy(constrvec.data(), constrvec.data() + num_constraints, constr);
}

void OptimizationProblem<double>::Proxy::
gradient(unsigned num_variables, const double* x, bool /*new_x*/,
        double* grad) const
{
    // TODO avoid copying.
    m_x_working = Eigen::Map<const VectorXd>(x, num_variables);

    // TODO use a better estimate for this step size.
    const double eps = std::sqrt(Eigen::NumTraits<double>::epsilon());
    const double two_eps = 2 * eps;

    // TODO parallelize.
    // TODO "if(parallelize)"
    // "firstprivate" means that each thread will get its own copy of
    // m_x_working, and that it will be copy constructed from m_x_working.
    // All other variables are shared across threads.
    // TODO rather than copy-constructing, ensure the right size.
    // TODO m_x_working.resize(num_variables);
    #pragma omp parallel firstprivate(m_x_working)
    {
        double obj_pos;
        double obj_neg;
        #pragma omp for
        for (Eigen::Index i = 0; i < num_variables; ++i) {
            // Perform a central difference.
            m_x_working[i] += eps;
            m_problem.objective(m_x_working, obj_pos);
            m_x_working[i] = x[i] - eps;
            m_problem.objective(m_x_working, obj_neg);
            // Restore the original value.
            m_x_working[i] = x[i];
            grad[i] = (obj_pos - obj_neg) / two_eps;
        }
    }
    /*
    Eigen::VectorXd grad_temp = Eigen::VectorXd::Zero(num_variables);
    #pragma omp parallel firstprivate(m_x_working)
    {
        Eigen::VectorXd grad_private = Eigen::VectorXd::Zero(num_variables);
        double obj_pos;
        double obj_neg;
        #pragma omp for
        for (Eigen::Index i = 0; i < num_variables; ++i) {
            // Perform a central difference.
            m_x_working[i] += eps;
            m_problem.objective(m_x_working, obj_pos);
            m_x_working[i] = x[i] - eps;
            m_problem.objective(m_x_working, obj_neg);
            // Restore the original value.
            m_x_working[i] = x[i];
            grad_private[i] = (obj_pos - obj_neg) / two_eps;
        }
        #pragma omp critical
        grad_temp += grad_private;
    }
    // TODO
    std::copy(grad_temp.data(), grad_temp.data() + num_variables, grad);
    */

    //double obj_pos;
    //double obj_neg;
    //for (Eigen::Index i = 0; i < num_variables; ++i) {
    //    // Perform a central difference.
    //    m_x_working[i] += eps;
    //    m_problem.objective(m_x_working, obj_pos);
    //    m_x_working[i] = x[i] - eps;
    //    m_problem.objective(m_x_working, obj_neg);
    //    // Restore the original value.
    //    m_x_working[i] = x[i];
    //    grad[i] = (obj_pos - obj_neg) / two_eps;
    //}
}

void OptimizationProblem<double>::Proxy::
jacobian(unsigned num_variables, const double* variables, bool /*new_x*/,
        unsigned /*num_nonzeros*/, double* jacobian_values) const
{
    // TODO give error message that sparsity() must be called first.

    // TODO scale by magnitude of x.
    const double eps = std::sqrt(Eigen::NumTraits<double>::epsilon());
    const double two_eps = 2 * eps;
    // Number of perturbation directions.
    const Eigen::Index num_seeds = m_jacobian_seed.cols();
    Eigen::Map<const VectorXd> x0(variables, num_variables);

    // Compute the dense "compressed Jacobian" using the directions ColPack
    // told us to use.
    Eigen::MatrixXd jacobian_compressed_eigen(num_constraints(), num_seeds);
    // TODO OMP_WAIT_POLICY has huge effect.
    #pragma omp parallel for firstprivate(m_constr_pos, m_constr_neg)
    for (Eigen::Index iseed = 0; iseed < num_seeds; ++iseed) {
        const auto direction = m_jacobian_seed.col(iseed);
        // Perturb x in the positive direction.
        // TODO LowOrder has working memory!
        m_problem.constraints(x0 + eps * direction, m_constr_pos);
        // Perturb x in the negative direction.
        m_problem.constraints(x0 - eps * direction, m_constr_neg);
        // Compute central difference.
        jacobian_compressed_eigen.col(iseed) =
                (m_constr_pos - m_constr_neg) / two_eps;

        // Store this column of the compressed Jacobian in the data structure
        // that ColPack will use.
        for (unsigned int i = 0; i < num_constraints(); ++i) {
            m_jacobian_compressed[i][iseed] =
                    jacobian_compressed_eigen(i, iseed);
        }
    }

    //std::cout << "DEBUG " << jacobian_compressed_eigen << std::endl;
    //std::exit(-1);
    /*
    for (Eigen::Index iseed = 0; iseed < num_seeds; ++iseed) {
        const auto direction = m_jacobian_seed.col(iseed);
        // Perturb x in the positive direction.
        m_problem.constraints(x0 + eps * direction, m_constr_pos);
        // Perturb x in the negative direction.
        m_problem.constraints(x0 - eps * direction, m_constr_neg);
        // Compute central difference.
        m_jacobian_compressed_column = (m_constr_pos - m_constr_neg) / two_eps;

        // Store this column of the compressed Jacobian in the data structure
        // that ColPack will use.
        #pragma omp ordered
        for (unsigned int i = 0; i < num_constraints(); ++i) {
            m_jacobian_compressed[i][iseed] = m_jacobian_compressed_column[i];
        }
    }
     */

    // Convert the dense compressed Jacobian into the sparse Jacobian layout
    // (specified as triplets {row indices, column indices, values}).
    unsigned int* row_ptr = m_jacobian_recovered_row_indices.data();
    unsigned int* col_ptr = m_jacobian_recovered_col_indices.data();
    m_jacobian_recovery->RecoverD2Cln_CoordinateFormat_usermem(
            m_jacobian_coloring.get(), // ColPack's graph coloring object.
            m_jacobian_compressed.get(), // Holds the finite differences.
            m_jacobian_pattern_ADOLC_format.get(), // Input sparsity pattern.
            &row_ptr, &col_ptr, // Row and col. indices of nonzeros; not needed.
            &jacobian_values); // Corresponding values in the Jacobian.
}

void OptimizationProblem<double>::Proxy::
hessian_lagrangian(unsigned /*num_variables*/, const double* /*variables*/,
        bool /*new_x*/, double /*obj_factor*/,
        unsigned /*num_constraints*/, const double* /*lambda*/,
        bool /*new_lambda TODO */,
        unsigned /*num_nonzeros*/, double* /*hessian_values*/) const
{
    // TODO
    std::string msg =
            "[tropter] Hessian not available with finite differences.";
    std::cerr << msg << std::endl;
    throw std::runtime_error(msg);
}

} // namespace tropter
