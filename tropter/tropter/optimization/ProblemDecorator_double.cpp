// ----------------------------------------------------------------------------
// tropter: ProblemDecorator_double.cpp
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
#include "ProblemDecorator_double.h"
#include <tropter/Exception.hpp>
#include "internal/GraphColoring.h"

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
namespace optimization {

// We must implement the destructor in a context where the JacobianColoring
// class is complete (since it's used in a unique ptr member variable.).
Problem<double>::Decorator::~Decorator() {}

Problem<double>::Decorator::Decorator(
        const Problem<double>& problem) :
        ProblemDecorator(problem), m_problem(problem) {}

void Problem<double>::Decorator::
calc_sparsity(const Eigen::VectorXd& variables,
        SparsityCoordinates& jacobian_sparsity_coordinates,
        bool provide_hessian_sparsity,
        SparsityCoordinates& hessian_sparsity_coordinates) const
{
    const auto num_vars = get_num_variables();
    m_x_working = VectorXd::Zero(num_vars);

    // Gradient.
    // =========
    // Determine the indicies of the variables used in the objective function
    // (conservative estimate of the indicies of the gradient that are nonzero).
    std::function<double(const VectorXd&)> calc_objective =
            [this](const VectorXd& vars) {
                double obj_value = 0;
                m_problem.calc_objective(vars, obj_value);
                return obj_value;
            };
    SparsityPattern gradient_sparsity =
            calc_gradient_sparsity_with_perturbation(variables,
                    calc_objective);
    m_gradient_nonzero_indices =
            gradient_sparsity.convert_to_CompressedRowSparsity()[0];

    // Jacobian.
    // =========
    const auto num_jac_rows = get_num_constraints();

    // Determine the sparsity pattern.
    // -------------------------------
    // We do this by setting an element of x to NaN, and examining which
    // constraint equations end up as NaN (and therefore depend on that
    // element of x).
    std::function<void(const VectorXd&, VectorXd&)> calc_constraints =
            [this](const VectorXd& vars, VectorXd& constr) {
                m_problem.calc_constraints(vars, constr);
            };
    const auto var_names = m_problem.get_variable_names();
    const auto constr_names = m_problem.get_constraint_names();
    SparsityPattern jacobian_sparsity =
            calc_jacobian_sparsity_with_perturbation(variables,
                    num_jac_rows, calc_constraints, constr_names, var_names);

    m_jacobian_coloring.reset(new JacobianColoring(jacobian_sparsity));
    m_jacobian_coloring->get_coordinate_format(jacobian_sparsity_coordinates);
    int num_jacobian_seeds = (int)m_jacobian_coloring->get_seed_matrix().cols();
    print("Number of seeds for Jacobian: %i", num_jacobian_seeds);
    // jacobian_sparsity.write("DEBUG_findiff_jacobian_sparsity.csv");

    // Allocate memory that is used in jacobian().
    m_constr_pos.resize(num_jac_rows);
    m_constr_neg.resize(num_jac_rows);
    m_jacobian_compressed.resize(num_jac_rows, num_jacobian_seeds);

    // Hessian.
    // ========
    if (provide_hessian_sparsity) {
        calc_sparsity_hessian_lagrangian(variables,
                hessian_sparsity_coordinates);

        // TODO produce a more informative number/description.
        print("Number of seeds for Hessian of constraints: %i",
                m_hescon_coloring->get_seed_matrix().cols());
        print("Number of seeds for Hessian of objective: %i",
                m_hesobj_coloring->get_seed_matrix().cols());
        // m_constr_working.resize(num_jac_rows);

        if (get_findiff_hessian_mode() == "slow") {
            m_hessian_indices = hessian_sparsity_coordinates;
        }
    }
}


void Problem<double>::Decorator::
calc_sparsity_hessian_lagrangian(const VectorXd& x,
        SparsityCoordinates& hessian_sparsity_coordinates) const {
    const int num_vars = (int)m_problem.get_num_variables();

    SymmetricSparsityPattern hescon_sparsity(num_vars);
    SymmetricSparsityPattern hesobj_sparsity(num_vars);

    if (m_problem.get_use_supplied_sparsity_hessian_lagrangian()) {

        using CalcSparsityHessianLagrangianNotImplemented =
                AbstractProblem::CalcSparsityHessianLagrangianNotImplemented;
        try {
            m_problem.calc_sparsity_hessian_lagrangian(x, hescon_sparsity,
                    hesobj_sparsity);
            TROPTER_THROW_IF(hescon_sparsity.get_num_rows() != num_vars,
                    "Expected sparsity pattern of Hessian of constraints to "
                    "have dimensions %i, but it has dimensions %i.",
                    num_vars, hescon_sparsity.get_num_rows());
            TROPTER_THROW_IF(hesobj_sparsity.get_num_rows() != num_vars,
                    "Expected sparsity pattern of Hessian of objective to "
                    "have dimensions %i, but it has dimensions %i.",
                    num_vars, hesobj_sparsity.get_num_rows());
        } catch (const CalcSparsityHessianLagrangianNotImplemented&) {
            TROPTER_THROW("User requested use of user-supplied sparsity for "
                "the Hessian of the Lagrangian, but "
                "calc_sparsity_hessian_lagrangian() is not implemented.");
        }

    } else {
        // We get the sparsity pattern of the Hessian of the objective and
        // the Hessian of lambda*constraints *separately* because we use
        // different algorithms to compute these two Hessians. However, we
        // have to report the total sparsity for use in the Solver.

        // Sparsity of Hessian of lambda*constraints; conservative estimate.
        hescon_sparsity =
                SymmetricSparsityPattern::create_from_jacobian_sparsity(
                        m_jacobian_coloring->get_sparsity());

        // Sparsity of Hessian of objective.
        SparsityPattern gradient_sparsity(num_vars, m_gradient_nonzero_indices);
        hesobj_sparsity =
                SymmetricSparsityPattern::create_from_jacobian_sparsity(
                        gradient_sparsity);
    }

    // Create GraphColoring objects.
    m_hescon_coloring.reset(new HessianColoring(hescon_sparsity));
    m_hesobj_coloring.reset(new HessianColoring(hesobj_sparsity));
    m_hesobj_coloring->get_coordinate_format(m_hesobj_indices);

    // Sparsity of Hessian of Lagrangian.
    // ----------------------------------
    SymmetricSparsityPattern hessian_sparsity = hescon_sparsity;
    hessian_sparsity.add_in_nonzeros(hesobj_sparsity);

    m_hessian_coloring.reset(new HessianColoring(hessian_sparsity));
    m_hessian_coloring->get_coordinate_format(hessian_sparsity_coordinates);

    //hessian_sparsity.write("DEBUG_findiff_hessian_lagrangian_sparsity.csv");
}


void Problem<double>::Decorator::
calc_objective(unsigned num_variables, const double* variables,
        bool /*new_x*/,
        double& obj_value) const
{
    obj_value = 0.0;
    // TODO avoid copy.
    const VectorXd xvec = Eigen::Map<const VectorXd>(variables, num_variables);
    m_problem.calc_objective(xvec, obj_value);
}

void Problem<double>::Decorator::
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

void Problem<double>::Decorator::
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

void Problem<double>::Decorator::
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

void Problem<double>::Decorator::
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

    //using namespace std::chrono;
    //auto start = high_resolution_clock::now();


    // Bohme book has guidelines for step size (section 9.2.4.4).
    const double& eps = get_findiff_hessian_step_size();
    const double eps_squared = eps * eps;
    // TODO m_x_working = Eigen::Map<const VectorXd>(xraw, num_variables);
    Eigen::Map<const VectorXd> x0(x_raw, num_variables);

    Eigen::Map<const VectorXd> lambda(lambda_raw, num_constraints);

    // TODO reuse perturbations between the Jacobian and Hessian calculations
    // (if step size is the same).

    // Compute the unperturbed constraints value.
    VectorXd p1 = VectorXd::Zero(num_constraints);
    m_problem.calc_constraints(x0, p1);

    const auto& hescon_seed = m_hescon_coloring->get_seed_matrix();
    const Eigen::Index num_hescon_seeds = hescon_seed.cols();

    const auto& jac_seed = m_jacobian_coloring->get_seed_matrix();
    const Eigen::Index num_jac_seeds = jac_seed.cols();
    int num_jac_nonzeros = m_jacobian_coloring->get_num_nonzeros();

    // Hessian of constraints.
    // -----------------------
    // Allocate memory (TODO preallocate once in calc_sparsity()).
    // Compressed Hessian of constraints.
    Eigen::MatrixXd hescon_c(num_variables, num_hescon_seeds);
    // Double-compressed second derivatives; same shape as a compressed
    // Jacobian. Used in the inner loop.
    Eigen::MatrixXd hescon_cc(num_constraints, num_jac_seeds);
    // Store perturbed values of constraints.
    VectorXd p2(num_constraints);
    VectorXd p3(num_constraints);
    VectorXd p4(num_constraints);

    // Loop through Hessian seeds.
    for (int ihesseed = 0; ihesseed < num_hescon_seeds; ++ihesseed) {
        const auto hes_direction = hescon_seed.col(ihesseed);
        VectorXd xb = x0 + eps * hes_direction;
        p2.setZero();
        m_problem.calc_constraints(xb, p2);

        for (int ijacseed = 0; ijacseed < num_jac_seeds; ++ijacseed) {
            const auto jac_direction = jac_seed.col(ijacseed);
            p3.setZero();
            m_problem.calc_constraints(x0 + eps * jac_direction, p3);
            p4.setZero();
            m_problem.calc_constraints(xb + eps * jac_direction, p4);

            // Finite difference.
            hescon_cc.col(ijacseed) = (p1 - p2 - p3 + p4) / eps_squared;
        }

        // Recover (uncompress).
        Eigen::VectorXd Bgunc_coeffs(num_jac_nonzeros);
        m_jacobian_coloring->recover(hescon_cc, Bgunc_coeffs.data());
        // TODO preallocate:
        Eigen::SparseMatrix<double> Bgunc;
        m_jacobian_coloring->convert(Bgunc_coeffs.data(), Bgunc);

        hescon_c.col(ihesseed) = Bgunc.transpose() * lambda;
    }

    // Convert the compressed Hessian of constraints into a SparseMatrix, for
    // ease of combining with Hessian of objective.
    Eigen::SparseMatrix<double> hessian;
    m_hescon_coloring->recover(hescon_c, hessian);

    //m_time_hescon +=
    //        duration_cast<duration<double>>(high_resolution_clock::now() -
    //                start).count();
    //start = high_resolution_clock::now();

    // Add in Hessian of objective.
    // ----------------------------
    if (obj_factor) {
        Eigen::VectorXd hesobj_vec;
        calc_hessian_objective(x0, hesobj_vec);
        Eigen::SparseMatrix<double> hesobj;
        m_hesobj_coloring->convert(hesobj_vec.data(), hesobj);
        hessian += obj_factor * hesobj;
    }

    //m_time_hesobj +=
    //        duration_cast<duration<double>>(high_resolution_clock::now() -
    //                start).count();
    //std::cout << "DEBUG " << m_time_hescon << " " << m_time_hesobj << std::endl;

    // Convert the SparseMatrix into coordinate format.
    m_hessian_coloring->convert(hessian, hessian_values_raw);
}

void Problem<double>::Decorator::
calc_hessian_objective(const VectorXd& x0,
        VectorXd& hesobj_values) const {
    assert(m_hesobj_indices.row.size() == m_hesobj_indices.col.size());

    hesobj_values = VectorXd::Zero(m_hesobj_indices.row.size());

    const double& eps = get_findiff_hessian_step_size();
    const double eps_squared = eps * eps;

    VectorXd x(x0);

    double obj_0 = 0;
    m_problem.calc_objective(x0, obj_0);


    // Avoid computing f(x + eps * e_i) multiple times.
    // TODO preallocate these two vectors.
    m_perturbed_objective_is_cached.resize(x0.size());
    m_perturbed_objective_is_cached.setConstant(false);
    m_perturbed_objective_cache.resize(x0.size());
    auto get_perturbed_objective = [&](int i) {
        if (!m_perturbed_objective_is_cached[i]) {
            x[i] += eps;
            m_perturbed_objective_cache[i] = 0;
            m_problem.calc_objective(x, m_perturbed_objective_cache[i]);
            x[i] = x0[i];
            m_perturbed_objective_is_cached[i] = true;
        }
        return m_perturbed_objective_cache[i];
    };
    for (int inz = 0; inz < (int)m_hesobj_indices.row.size(); ++inz) {
        int i = m_hesobj_indices.row[inz];
        int j = m_hesobj_indices.col[inz];

        if (i == j) {

            // x + eps e_i
            double obj_pos = get_perturbed_objective(i);

            // x - eps e_i
            x[i] = x0[i] - eps;
            double obj_neg = 0;
            m_problem.calc_objective(x, obj_neg);
            x[i] = x0[i];

            hesobj_values[inz] =
                    (obj_pos + obj_neg - 2 * obj_0) / eps_squared;

        } else {

            // x + eps e_i
            double obj_i = get_perturbed_objective(i);

            // x + eps (e_i + e_j)
            x[i] += eps;
            x[j] += eps;
            double obj_ij = 0;
            m_problem.calc_objective(x, obj_ij);
            x[i] = x0[i];
            x[j] = x0[j];

            // x + eps e_j
            double obj_j = get_perturbed_objective(j);

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
    //     std::cout << "(" << m_hesobj_indices.row[inz] << "," <<
    //             m_hesobj_indices.col[inz] << "): " <<
    //             hesobj_values[inz] << std::endl;
    // }
}

void Problem<double>::Decorator::
calc_hessian_lagrangian_slow(unsigned num_variables, const double* x_raw,
        bool /*new_x*/, double obj_factor,
        unsigned num_constraints, const double* lambda_raw,
        bool /*new_lambda */,
        unsigned /*num_nonzeros*/, double* hessian_values) const
{
    const double eps = get_findiff_hessian_step_size();
    const double eps_squared = eps * eps;

    // m_x_working = Eigen::Map<const VectorXd>(x, num_variables);
    Eigen::VectorXd x = Eigen::Map<const VectorXd>(x_raw, num_variables);

    Eigen::Map<const VectorXd> lambda(lambda_raw, num_constraints);

    double lagr_0;
    calc_lagrangian(x, obj_factor, lambda, lagr_0);

    assert(m_hessian_indices.row.size() == m_hessian_indices.col.size());

    // TODO can avoid computing f(x + eps * e_i) multiple times.
    for (int inz = 0; inz < (int)m_hessian_indices.row.size(); ++inz) {
        int i = m_hessian_indices.row[inz];
        int j = m_hessian_indices.col[inz];

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

void Problem<double>::Decorator::
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


} // namespace optimization
} // namespace tropter

//#ifdef TROPTER_WITH_OPENMP && _OPENMP
//    #if defined(__GNUC__)
//        #pragma GCC diagnostic pop
//    #elif defined(_MSC_VER)
//        #pragma warning(pop)
//    #endif
//#endif
