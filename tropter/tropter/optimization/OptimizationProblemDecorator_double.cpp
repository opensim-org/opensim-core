#include "OptimizationProblem.h"
#include "internal/GraphColoring.h"

#include <iomanip>

//#if defined(TROPTER_WITH_OPENMP) && _OPENMP
//    // TODO only include ifdef _OPENMP
//    #include <omp.h>
//    #if defined(__GNUC__)
//        #pragma GCC diagnostic push
//        #pragma GCC diagnostic ignored "-Wunknown-pragmas"
//    #elif defined(_MSC_VER)
//        #pragma warning(push)
//        #pragma warning(disable: 4068) // Disable unknown pragma warnings.
//    #endif
//#endif

using Eigen::VectorXd;

namespace tropter {

// We must implement the destructor in a context where the JacobianColoring
// class is complete (since it's used in a unique ptr member variable.).
OptimizationProblem<double>::Decorator::~Decorator() {}

OptimizationProblem<double>::Decorator::Decorator(
        const OptimizationProblem<double>& problem) :
        OptimizationProblemDecorator(problem), m_problem(problem) {}

void OptimizationProblem<double>::Decorator::
calc_sparsity(const Eigen::VectorXd& x,
        std::vector<unsigned int>& jacobian_row_indices,
        std::vector<unsigned int>& jacobian_col_indices,
        std::vector<unsigned int>& hessian_row_indices,
        std::vector<unsigned int>& hessian_col_indices) const
{
    const auto num_vars = get_num_variables();

    // Gradient.
    // =========
    // Determine the indicies of the variables used in the objective function
    // (conservative estimate of the indicies of the gradient that are nonzero).
    m_x_working = VectorXd::Zero(num_vars);
    double obj_value;
    for (int j = 0; j < (int)num_vars; ++j) {
        obj_value = 0;
        m_x_working[j] = std::numeric_limits<double>::quiet_NaN();
        m_problem.calc_objective(m_x_working, obj_value);
        m_x_working[j] = 0;
        if (std::isnan(obj_value)) {
            m_gradient_nonzero_indices.push_back(j);
        }
    }

    // Jacobian.
    // =========
    const auto num_jac_rows = get_num_constraints();

    // Determine the sparsity pattern.
    // -------------------------------
    // We do this by setting an element of x to NaN, and examining which
    // constraint equations end up as NaN (and therefore depend on that
    // element of x).
    m_x_working.setZero();
    VectorXd constr_working(num_jac_rows);
    // Initially, we store the sparsity structure in ADOL-C's compressed row
    // format, since this is what ColPack accepts.
    // This format, as described in the ADOL-C manual, is a 2-Dish array.
    // The length of the first dimension is the number of rows in the Jacobian.
    // Each element represents a row and is a vector of the column indices of
    // the nonzeros in that row. The length of each row (the inner dimension) is
    // the number of nonzeros in that row.
    std::vector<std::vector<unsigned int>> jacobian_sparsity(num_jac_rows);
    for (int j = 0; j < (int)num_vars; ++j) {
        constr_working.setZero();
        m_x_working[j] = std::numeric_limits<double>::quiet_NaN();
        m_problem.calc_constraints(m_x_working, constr_working);
        m_x_working[j] = 0;
        for (int i = 0; i < (int)num_jac_rows; ++i) {
            if (std::isnan(constr_working[i])) {
                jacobian_sparsity[i].push_back(j);
            }
        }
    }

    m_jacobian_coloring.reset(
            new JacobianColoring(num_jac_rows, num_vars, jacobian_sparsity));
    m_jacobian_coloring->get_coordinate_format(
            jacobian_row_indices, jacobian_col_indices);
    int num_jacobian_seeds = (int)m_jacobian_coloring->get_seed_matrix().cols();
    std::cout << "[tropter] Number of finite difference perturbations required "
            "for sparse Jacobian: " << num_jacobian_seeds << std::endl;

    // Allocate memory that is used in jacobian().
    m_constr_pos.resize(num_jac_rows);
    m_constr_neg.resize(num_jac_rows);
    m_jacobian_compressed.resize(num_jac_rows, num_jacobian_seeds);

    // Hessian.
    // ========
    // TODO only compute Hessian sparsity if it's needed.
    if (m_problem.get_use_supplied_sparsity_hessian_lagrangian()) {

        using CalcSparsityHessianLagrangianNotImplemented =
                AbstractOptimizationProblem::
                CalcSparsityHessianLagrangianNotImplemented;
        try {
            m_problem.calc_sparsity_hessian_lagrangian(x,
                    hessian_row_indices, hessian_col_indices);
        } catch (const CalcSparsityHessianLagrangianNotImplemented& ex) {
            throw std::runtime_error("[tropter] User requested use of "
                    "user-supplied sparsity for the Hessian of the Lagrangian, "
                    "but calc_sparsity_hessian_lagrangian() is not "
                    "implemented.");
        }
        if (hessian_row_indices.size() != hessian_col_indices.size()) {
            throw std::runtime_error("Expected hessian_row_indices (size " +
                    std::to_string(hessian_row_indices.size()) + ") and "
                    "hessian_col_indices (size " +
                    std::to_string(hessian_col_indices.size()) +
                    ") to have the same size.");
        }
        // TODO user must specify sparsity pattern in the order that
        // TODO sort what the user provided so that the ordering matches what
        // ColPack will give us.
        // TODO make sure there are no duplicates.
    } else {
        // Dense upper triangle.

        const int num_hessian_nonzeros = num_vars * (num_vars + 1) / 2;
        hessian_row_indices.resize(num_hessian_nonzeros);
        hessian_col_indices.resize(num_hessian_nonzeros);
        int inz = 0;
        for (int j = 0; j < (int)num_vars; ++j) {
            for (int i = 0; i <= j; ++i) {
                hessian_row_indices[inz] = i;
                hessian_col_indices[inz] = j;
                ++inz;
            }
        }
    }
    m_hessian_row_indices = hessian_row_indices;
    m_hessian_col_indices = hessian_col_indices;

    m_constr_working.resize(num_jac_rows);
}

void OptimizationProblem<double>::Decorator::
calc_objective(unsigned num_variables, const double* variables,
        bool /*new_x*/,
        double& obj_value) const
{
    // TODO avoid copy.
    const VectorXd xvec = Eigen::Map<const VectorXd>(variables, num_variables);
    m_problem.calc_objective(xvec, obj_value);
}

void OptimizationProblem<double>::Decorator::
calc_constraints(unsigned num_variables, const double* variables,
        bool /*new_variables*/,
        unsigned num_constraints, double* constr) const
{
    // TODO avoid copy.
    m_x_working = Eigen::Map<const VectorXd>(variables, num_variables);
    VectorXd constrvec(num_constraints); // TODO avoid copy.
    // TODO at least keep constrvec as working memory.
    m_problem.calc_constraints(m_x_working, constrvec);
    // TODO avoid copy.
    std::copy(constrvec.data(), constrvec.data() + num_constraints, constr);
}

void OptimizationProblem<double>::Decorator::
calc_gradient(unsigned num_variables, const double* x, bool /*new_x*/,
        double* grad) const
{
    m_x_working = Eigen::Map<const VectorXd>(x, num_variables);

    // TODO use a better estimate for this step size.
    const double eps = std::sqrt(Eigen::NumTraits<double>::epsilon());
    const double two_eps = 2 * eps;

    // We only compute the entries that are nonzero, and we must make sure
    // all other entries are 0.
    std::fill(grad, grad + num_variables, 0);

    double obj_pos;
    double obj_neg;
    // TODO parallelize.
    // "firstprivate" means that each thread will get its own copy of
    // m_x_working, and that it will be copy constructed from m_x_working.
    // All other variables are shared across threads.
    // TODO speedup in Release using OpenMP requires setting environment var
    // OMP_WAIT_POLICY=passive.
    // TODO add `if(parallel)`
    //#pragma omp parallel for
    //            firstprivate(m_x_working)
    //            private(obj_pos, obj_neg)
    for (const auto& i : m_gradient_nonzero_indices) {
        // Perform a central difference.
        m_x_working[i] += eps;
        m_problem.calc_objective(m_x_working, obj_pos);
        m_x_working[i] = x[i] - eps;
        m_problem.calc_objective(m_x_working, obj_neg);
        // Restore the original value.
        m_x_working[i] = x[i];
        grad[i] = (obj_pos - obj_neg) / two_eps;
    }
}

void OptimizationProblem<double>::Decorator::
calc_jacobian(unsigned num_variables, const double* variables, bool /*new_x*/,
        unsigned /*num_nonzeros*/, double* jacobian_values) const
{
    // TODO give error message that sparsity() must be called first.

    // TODO scale by magnitude of x.
    const double eps = std::sqrt(Eigen::NumTraits<double>::epsilon());
    const double two_eps = 2 * eps;
    // Number of perturbation directions.
    const auto& seed = m_jacobian_coloring->get_seed_matrix();
    const Eigen::Index num_seeds = seed.cols();
    Eigen::Map<const VectorXd> x0(variables, num_variables);

    // Compute the dense "compressed Jacobian" using the directions ColPack
    // told us to use.
    // TODO for OpenMP: LowOrder has working memory!
    //#pragma omp parallel for firstprivate(m_constr_pos, m_constr_neg)
    for (Eigen::Index iseed = 0; iseed < num_seeds; ++iseed) {
        const auto direction = seed.col(iseed);
        // Perturb x in the positive direction.
        m_problem.calc_constraints(x0 + eps * direction, m_constr_pos);
        // Perturb x in the negative direction.
        m_problem.calc_constraints(x0 - eps * direction, m_constr_neg);
        // Compute central difference.
        m_jacobian_compressed.col(iseed) =
                (m_constr_pos - m_constr_neg) / two_eps;
    }

    m_jacobian_coloring->recover(m_jacobian_compressed, jacobian_values);
}

void OptimizationProblem<double>::Decorator::
calc_hessian_lagrangian(unsigned num_variables, const double* x,
        bool /*new_x*/, double obj_factor,
        unsigned num_constraints, const double* lambda_raw,
        bool /*new_lambda */,
        unsigned /*num_nonzeros*/, double* hessian_values) const
{
    const double eps = 1e-2; // TODO
    const double eps_squared = eps * eps;

    m_x_working = Eigen::Map<const VectorXd>(x, num_variables);

    Eigen::Map<const VectorXd> lambda(lambda_raw, num_constraints);

    double lagr_0;
    calc_lagrangian(m_x_working, obj_factor, lambda, lagr_0);

    assert(m_hessian_row_indices.size() == m_hessian_col_indices.size());

    // TODO can avoid computing f(x + eps * x[i]) multiple times.
    for (int inz = 0; inz < (int)m_hessian_row_indices.size(); ++inz) {
        int i = m_hessian_row_indices[inz];
        int j = m_hessian_col_indices[inz];

        if (i == j) {

            m_x_working[i] += eps;
            double lagr_pos;
            calc_lagrangian(m_x_working, obj_factor, lambda, lagr_pos);

            m_x_working[i] = x[i] - eps;
            double lagr_neg;
            calc_lagrangian(m_x_working, obj_factor, lambda, lagr_neg);

            m_x_working[i] = x[i];

            hessian_values[inz] =
                    (lagr_pos + lagr_neg - 2 * lagr_0) / eps_squared;

        } else {

            m_x_working[i] += eps;
            double lagr_i;
            calc_lagrangian(m_x_working, obj_factor, lambda, lagr_i);

            m_x_working[j] += eps;
            double lagr_ij;
            calc_lagrangian(m_x_working, obj_factor, lambda, lagr_ij);

            m_x_working[i] = x[i];
            double lagr_j;
            calc_lagrangian(m_x_working, obj_factor, lambda, lagr_j);

            m_x_working[j] = x[j];

            hessian_values[inz] =
                    (lagr_ij + lagr_0 - lagr_i - lagr_j) / eps_squared;
        }

    }
    // TODO
//    std::string msg =
//            "[tropter] Hessian not available with finite differences.";
//    std::cerr << msg << std::endl;
//    throw std::runtime_error(msg);
}

void OptimizationProblem<double>::Decorator::
calc_lagrangian(const Eigen::VectorXd& x, double obj_factor,
        const Eigen::Map<const Eigen::VectorXd>& lambda,
        double& lagrangian_value) const {

    // TODO check if lambda is ever all zeros.

    lagrangian_value = 0;

    if (obj_factor != 0) {
        m_problem.calc_objective(m_x_working, lagrangian_value);
    }

    m_problem.calc_constraints(x, m_constr_working);
    lagrangian_value += lambda.dot(m_constr_working);
}

} // namespace tropter

//#ifdef TROPTER_WITH_OPENMP && _OPENMP
//    #if defined(__GNUC__)
//        #pragma GCC diagnostic pop
//    #elif defined(_MSC_VER)
//        #pragma warning(pop)
//    #endif
//#endif
