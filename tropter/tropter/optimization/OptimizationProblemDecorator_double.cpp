// ----------------------------------------------------------------------------
// tropter: OptimizationProblemDecorator_double.cpp
// ----------------------------------------------------------------------------
// Copyright (c) 2017 tropter authors
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may
// not use this file except in compliance with the License. You may obtain a
// copy of the License at http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------
#include "OptimizationProblem.h"
#include <tropter/Exception.hpp>
#include "internal/GraphColoring.h"
#include <Eigen/SparseCore>

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

using CompressedRowSparsity = std::vector<std::vector<unsigned int>>;

// References for finite differences:
// Nocedal and Wright
// Betts 2010
// https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19850025225.pdf
// https://github.com/casadi/casadi/issues/1026
// Gebremedhin 2005 What color is your Jacobian? Graph coloring for computing
// derivatives
// https://cran.r-project.org/web/packages/sparseHessianFD/vignettes/sparseHessianFD.pdf
// Requires a gradient:
// Powell and Toint, ON THE ESTIMATION OF SPARSE HESSIAN MATRICES 1979
// http://epubs.siam.org/doi/pdf/10.1137/0716078

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
        bool provide_hessian_indices,
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
    CompressedRowSparsity jacobian_sparsity(num_jac_rows);
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
    print("Number of seeds for Jacobian: %i", num_jacobian_seeds);
    // std::ofstream file("DEBUG_finitediff_jacobian_sparsity.csv");
    // file << "row_indices,column_indices" << std::endl;
    // for (int i = 0; i < (int)jacobian_row_indices.size(); ++i) {
    //     file << jacobian_row_indices[i] << "," << jacobian_col_indices[i]
    //             << std::endl;
    // }
    // file.close();

    // Allocate memory that is used in jacobian().
    m_constr_pos.resize(num_jac_rows);
    m_constr_neg.resize(num_jac_rows);
    m_jacobian_compressed.resize(num_jac_rows, num_jacobian_seeds);

    // Hessian.
    // ========
    if (provide_hessian_indices) {
        calc_sparsity_hessian_lagrangian(x,
                hessian_row_indices, hessian_col_indices);

        int num_hessian_seeds =
                (int)m_hessian_coloring->get_seed_matrix().cols();
        // TODO produce a more informative number/description.
        print("Number of seeds for Hessian: %i", num_hessian_seeds);
        // m_constr_working.resize(num_jac_rows);

        if (get_findiff_hessian_mode() == "slow") {
            m_hessian_row_indices = hessian_row_indices;
            m_hessian_col_indices = hessian_col_indices;
        }
    }

}


namespace {
Eigen::SparseMatrix<bool> convert_to_Eigen_SparseMatrix(
        const CompressedRowSparsity& sparsity) {
    Eigen::SparseMatrix<bool> mat(sparsity.size(), sparsity.size());
    int num_nonzeros = 0;
    for (const auto& row : sparsity) num_nonzeros += row.size();
    mat.reserve(num_nonzeros);

    for (int i = 0; i < (int)sparsity.size(); ++i) {
        for (const auto& j : sparsity[i]) mat.insert(i, j) = 1;
    }
    mat.makeCompressed();
    return mat;
}
CompressedRowSparsity convert_to_CompressedRowSparsity(
        const Eigen::SparseMatrix<bool>& mat) {
    CompressedRowSparsity sparsity(mat.rows());
    for (int i = 0; i < mat.outerSize(); ++i) {
        for (Eigen::SparseMatrix<bool>::InnerIterator it(mat, i); it; ++it) {
            if (it.row() <= it.col()) // Upper triangle only.
                sparsity[it.row()].push_back((unsigned)it.col());
        }
    }
    return sparsity;
}
} // namespace

void OptimizationProblem<double>::Decorator::
calc_sparsity_hessian_lagrangian(
        const VectorXd& x,
        std::vector<unsigned int>& hessian_row_indices,
        std::vector<unsigned int>& hessian_col_indices) const {
    const auto num_vars = m_problem.get_num_variables();

    CompressedRowSparsity hescon_sparsity;
    CompressedRowSparsity hesobj_sparsity;
    Eigen::SparseMatrix<bool> hescon_sparsity_mat(num_vars, num_vars);
    Eigen::SparseMatrix<bool> hesobj_sparsity_mat(num_vars, num_vars);

    if (m_problem.get_use_supplied_sparsity_hessian_lagrangian()) {

        using CalcSparsityHessianLagrangianNotImplemented =
                AbstractOptimizationProblem::
                CalcSparsityHessianLagrangianNotImplemented;
        try {
            hescon_sparsity.resize(num_vars);
            hesobj_sparsity.resize(num_vars);
            m_problem.calc_sparsity_hessian_lagrangian(x, hescon_sparsity,
                    hesobj_sparsity);
        } catch (const CalcSparsityHessianLagrangianNotImplemented& ex) {
            TROPTER_THROW("User requested use of user-supplied sparsity for "
                "the Hessian of the Lagrangian, but "
                "calc_sparsity_hessian_lagrangian() is not implemented.")
        }

        // Place these here to check for errors in the user-provided sparsity
        // pattern.
        m_hescon_coloring.reset(
                new HessianColoring(num_vars, hescon_sparsity));
        m_hesobj_coloring.reset(
                new HessianColoring(num_vars, hesobj_sparsity));

        hescon_sparsity_mat = convert_to_Eigen_SparseMatrix(hescon_sparsity);
        hesobj_sparsity_mat = convert_to_Eigen_SparseMatrix(hesobj_sparsity);

    } else {

        // We get the sparsity pattern of the Hessian of the objective and
        // the Hessian of lambda*constraints *separately* because we use
        // different algorithms to compute these two Hessians. However, we
        // have to report the total sparsity for use in the OptimizationSolver.
        // Sparsity of Hessian of lambda*constraints
        // -----------------------------------------
        // Conservative estimate of sparsity of Hessian of lambda*constraints.
        // See Patterson, Michael A., and Anil V. Rao. "GPOPS-II: A MATLAB
        // software for solving multiple-phase optimal control problems using
        // hp-adaptive Gaussian quadrature collocation methods and sparse
        // nonlinear programming." ACM Transactions on Mathematical Software
        // (TOMS) 41.1 (2014): 1.
        VectorXd ones = VectorXd::Ones(m_jacobian_coloring->get_num_nonzeros());
        // Represent the sparsity pattern as a binary sparsity matrix.
        Eigen::SparseMatrix<bool> jac_sparsity_mat =
                m_jacobian_coloring->convert(ones.data()).cast<bool>();
        // The estimate for the sparsity pattern of the Hessian of
        // lambda*constraints. In Patterson 2013, this is S2 = S1^T * S1.
        hescon_sparsity_mat = jac_sparsity_mat.transpose() * jac_sparsity_mat;

        // Sparsity of Hessian of objective
        // --------------------------------
        /*
        */
        Eigen::SparseMatrix<bool> gradient_sparsity_mat(1, num_vars);
        gradient_sparsity_mat.reserve(m_gradient_nonzero_indices.size());
        for (const auto& grad_index : m_gradient_nonzero_indices) {
            gradient_sparsity_mat.insert(0, (int)grad_index) = 1;
        }
        gradient_sparsity_mat.makeCompressed();
        hesobj_sparsity_mat =
                gradient_sparsity_mat.transpose() * gradient_sparsity_mat;
        /* Assume the objective function has no cross terms.
        VectorX<bool> gradient_sparsity_mat(num_vars);
        for (const auto& grad_index : m_gradient_nonzero_indices) {
            gradient_sparsity_mat[grad_index] = 1;
        }
        hesobj_sparsity_mat = gradient_sparsity_mat.asDiagonal();
         * TODO
         */

        const CompressedRowSparsity hescon_sparsity =
                convert_to_CompressedRowSparsity(hescon_sparsity_mat);
        const CompressedRowSparsity hesobj_sparsity =
                convert_to_CompressedRowSparsity(hesobj_sparsity_mat);

        m_hescon_coloring.reset(
                new HessianColoring(num_vars, hescon_sparsity));
        m_hesobj_coloring.reset(
                new HessianColoring(num_vars, hesobj_sparsity));

    }

    // Create GraphColoring objects.
    m_hesobj_coloring->get_coordinate_format(
            m_hesobj_row_indices, m_hesobj_col_indices);

    // Sparsity of Hessian of Lagrangian.
    // ----------------------------------
    Eigen::SparseMatrix<bool> hessian_sparsity_mat =
            hescon_sparsity_mat || hesobj_sparsity_mat;
    const CompressedRowSparsity hessian_sparsity =
            convert_to_CompressedRowSparsity(hessian_sparsity_mat);

    m_hessian_coloring.reset(
            new HessianColoring(num_vars, hessian_sparsity));
    m_hessian_coloring->get_coordinate_format(
            hessian_row_indices, hessian_col_indices);

    //std::ofstream file("DEBUG_finitediff_hessian_lagrangian_sparsity.csv");
    //file << "row_indices,column_indices" << std::endl;
    //for (int i = 0; i < (int)hessian_row_indices.size(); ++i) {
    //    file << hessian_row_indices[i] << "," << hessian_col_indices[i]
    //            << std::endl;
    //}
    //file.close();
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
        obj_pos = 0;
        obj_neg = 0;
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
    // TODO for OpenMP: Trapezoidal has working memory!
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
calc_hessian_lagrangian(unsigned num_variables, const double* x_raw,
        bool new_x, double obj_factor,
        unsigned num_constraints, const double* lambda_raw,
        bool new_lambda,
        unsigned num_hes_nonzeros, double* hessian_values_raw) const {

    // TODO remove this string comparison.
    if (get_findiff_hessian_mode() == "slow") {
        calc_hessian_lagrangian_slow(num_variables, x_raw,
                new_x, obj_factor, num_constraints, lambda_raw, new_lambda,
                num_hes_nonzeros, hessian_values_raw);
        return;
    }

    // Bohme book has guidelines for step size (section 9.2.4.4).
    const double& eps = get_findiff_hessian_step_size();
    const double eps_squared = eps * eps;
    // TODO m_x_working = Eigen::Map<const VectorXd>(xraw, num_variables);
    Eigen::Map<const VectorXd> x0(x_raw, num_variables);

    Eigen::Map<const VectorXd> lambda(lambda_raw, num_constraints);

    // TODO improve variable names.

    // TODO reuse perturbations between the Jacobian and Hessian calculations.

    // Compute the unperturbed constraints value.
    VectorXd p1 = VectorXd::Zero(num_constraints);
    m_problem.calc_constraints(x0, p1);

    const auto& hes_seed = m_hescon_coloring->get_seed_matrix();
    const Eigen::Index num_hes_seeds = hes_seed.cols();

    const auto& jac_seed = m_jacobian_coloring->get_seed_matrix();
    const Eigen::Index num_jac_seeds = jac_seed.cols();
    int num_jac_nonzeros = m_jacobian_coloring->get_num_nonzeros();

    // Hessian of constraints.
    // -----------------------
    // Compressed Hessian of constraints.
    Eigen::MatrixXd Bgc(num_variables, num_hes_seeds);
    // Double-compressed second derivatives; same shape as a compressed
    // Jacobian. Used in the inner loop.
    Eigen::MatrixXd Bgcc(num_constraints, num_jac_seeds);

    // Loop through Hessian seeds.
    for (int ihesseed = 0; ihesseed < num_hes_seeds; ++ihesseed) {
        const auto hes_direction = hes_seed.col(ihesseed);
        VectorXd xb = x0 + eps * hes_direction;
        // TODO avoid reallocating, simply initialize to 0.
        VectorXd p2 = VectorXd::Zero(num_constraints);
        m_problem.calc_constraints(xb, p2);

        for (int ijacseed = 0; ijacseed < num_jac_seeds; ++ijacseed) {
            const auto jac_direction = jac_seed.col(ijacseed);
            VectorXd p3 = VectorXd::Zero(num_constraints);
            m_problem.calc_constraints(x0 + eps * jac_direction, p3);
            VectorXd p4 = VectorXd::Zero(num_constraints);
            m_problem.calc_constraints(xb + eps * jac_direction, p4);

            // Finite difference.
            Bgcc.col(ijacseed) = (p1 - p2 - p3 + p4) / eps_squared;
        }

        // Recover (uncompress).
        Eigen::VectorXd Bgunc_coeffs(num_jac_nonzeros);
        m_jacobian_coloring->recover(Bgcc, Bgunc_coeffs.data());
        // TODO preallocate:
        Eigen::SparseMatrix<double> Bgunc =
                m_jacobian_coloring->convert(Bgunc_coeffs.data());

        Bgc.col(ihesseed) = Bgunc.transpose() * lambda;
    }

    // TODO preallocate.
    VectorXd Bg = VectorXd::Zero(num_hes_nonzeros);
    m_hescon_coloring->recover(Bgc, Bg.data());

    Eigen::SparseMatrix<double> hessian =
            m_hescon_coloring->convert(Bg.data());

    // Add in Hessian of objective.
    // ----------------------------
    if (obj_factor) {
        Eigen::VectorXd hesobj_vec;
        calc_hessian_objective(x0, hesobj_vec);
        Eigen::SparseMatrix<double> hesobj =
                m_hesobj_coloring->convert(hesobj_vec.data());
        hessian += obj_factor * hesobj;
    }

    m_hessian_coloring->convert(hessian, hessian_values_raw);
}

void OptimizationProblem<double>::Decorator::
calc_hessian_objective(const VectorXd& x0,
        VectorXd& hesobj_values) const {
    assert(m_hesobj_row_indices.size() == m_hesobj_col_indices.size());

    hesobj_values = VectorXd::Zero(m_hesobj_row_indices.size());

    const double& eps = get_findiff_hessian_step_size();
    const double eps_squared = eps * eps;

    VectorXd x(x0);

    double obj_0 = 0;
    m_problem.calc_objective(x0, obj_0);


    // TODO can avoid computing f(x + eps * x[i]) multiple times.
    for (int inz = 0; inz < (int)m_hesobj_row_indices.size(); ++inz) {
        int i = m_hesobj_row_indices[inz];
        int j = m_hesobj_col_indices[inz];

        if (i == j) {

            x[i] += eps;
            double obj_pos = 0;
            m_problem.calc_objective(x, obj_pos);

            x[i] = x0[i] - eps;
            double obj_neg = 0;
            m_problem.calc_objective(x, obj_neg);

            x[i] = x0[i];

            hesobj_values[inz] =
                    (obj_pos + obj_neg - 2 * obj_0) / eps_squared;

        } else {

            x[i] += eps;
            double obj_i = 0;
            m_problem.calc_objective(x, obj_i);

            x[j] += eps;
            double obj_ij = 0;
            m_problem.calc_objective(x, obj_ij);

            x[i] = x0[i];
            double obj_j = 0;
            m_problem.calc_objective(x, obj_j);

            x[j] = x0[j];

            hesobj_values[inz] =
                    (obj_ij - obj_i - obj_j + obj_0) / eps_squared;

            //std::cout << "DEBUG " << i << " " << j
            //        << " obj_ij " << obj_ij
            //        << " obj_i " << obj_i
            //        << " obj_j " << obj_j
            //        << " obj_0 " << obj_0 << std::endl;
        }

    }
    // std::cout << "DEBUG hessian_objective\n";
    // for (int inz = 0; inz < (int)hesobj_values.size(); ++inz) {
    //     std::cout << "(" << m_hesobj_row_indices[inz] << "," <<
    //             m_hesobj_col_indices[inz] << "): " <<
    //             hesobj_values[inz] << std::endl;
    // }
}

void OptimizationProblem<double>::Decorator::
calc_hessian_lagrangian_slow(unsigned num_variables, const double* x_raw,
        bool /*new_x*/, double obj_factor,
        unsigned num_constraints, const double* lambda_raw,
        bool /*new_lambda */,
        unsigned /*num_nonzeros*/, double* hessian_values) const
{
    const double eps = get_findiff_hessian_step_size();
    const double eps_squared = eps * eps;

//    m_x_working = Eigen::Map<const VectorXd>(x, num_variables);
    Eigen::VectorXd x = Eigen::Map<const VectorXd>(x_raw, num_variables);
    // std::cout << "DEBUG x " << x << std::endl;

    Eigen::Map<const VectorXd> lambda(lambda_raw, num_constraints);
    // std::cout << "DEBUG lambda " << lambda << std::endl;

    double lagr_0;
    calc_lagrangian(x, obj_factor, lambda, lagr_0);

    assert(m_hessian_row_indices.size() == m_hessian_col_indices.size());

    // TODO can avoid computing f(x + eps * x[i]) multiple times.
    for (int inz = 0; inz < (int)m_hessian_row_indices.size(); ++inz) {
        int i = m_hessian_row_indices[inz];
        int j = m_hessian_col_indices[inz];

        if (i == j) {

            x[i] += eps;
            double lagr_pos;
            calc_lagrangian(x, obj_factor, lambda, lagr_pos);

            x[i] = x_raw[i] - eps;
            double lagr_neg;
            calc_lagrangian(x, obj_factor, lambda, lagr_neg);

            x[i] = x_raw[i];

            hessian_values[inz] =
                    (lagr_pos + lagr_neg - 2 * lagr_0) / eps_squared;

        } else {

            x[i] += eps;
            double lagr_i;
            calc_lagrangian(x, obj_factor, lambda, lagr_i);

            x[j] += eps;
            double lagr_ij;
            calc_lagrangian(x, obj_factor, lambda, lagr_ij);

            x[i] = x_raw[i];
            double lagr_j;
            calc_lagrangian(x, obj_factor, lambda, lagr_j);

            x[j] = x_raw[j];

            hessian_values[inz] =
                    (lagr_ij + lagr_0 - lagr_i - lagr_j) / eps_squared;
        }

    }
}

void OptimizationProblem<double>::Decorator::
calc_lagrangian(const Eigen::VectorXd& x, double obj_factor,
        const Eigen::Map<const Eigen::VectorXd>& lambda,
        double& lagrangian_value) const {

    // TODO check if lambda is ever all zeros.

    lagrangian_value = 0;

    if (obj_factor != 0) {
        m_problem.calc_objective(x, lagrangian_value);
    }

    Eigen::VectorXd constr = Eigen::VectorXd::Zero(lambda.size());
    m_problem.calc_constraints(x, constr);
    lagrangian_value += lambda.dot(constr);
}


} // namespace tropter

//#ifdef TROPTER_WITH_OPENMP && _OPENMP
//    #if defined(__GNUC__)
//        #pragma GCC diagnostic pop
//    #elif defined(_MSC_VER)
//        #pragma warning(pop)
//    #endif
//#endif
