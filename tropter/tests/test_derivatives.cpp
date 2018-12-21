// ----------------------------------------------------------------------------
// tropter: test_derivatives.cpp
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
#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <tropter/tropter.h>

#include "testing.h"

using Eigen::Ref;
using Eigen::VectorXd;
using Eigen::RowVectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Vector5d = Eigen::Matrix<double, 5, 1>;
using Eigen::MatrixXd;

using tropter::VectorX;
using tropter::SparsityPattern;
using tropter::SymmetricSparsityPattern;
using tropter::SparsityCoordinates;
using namespace tropter::optimization;

template<typename T>
class Unconstrained : public Problem<T> {
public:
    Unconstrained() : Problem<T>(3, 0)
    {
        this->set_variable_bounds(Vector3d(-3, -3, -3), Vector3d(3, 3, 3));
    }
    void calc_objective(const VectorX<T>& x, T& obj_value) const override
    {
        obj_value = x[0]*x[0] * x[1]*x[1] * x[2]*x[2];
    }
};

TEST_CASE("Unconstrained Hessian")
{
    Unconstrained<adouble> problem;
    VectorXd x(problem.get_num_variables());
    x << 1.5, 1.6, 1.7;

    auto deca = problem.make_decorator();

    // Expected derivatives.
    // ---------------------
    int num_hessian_nonzeros;
    {
        SparsityCoordinates jac_sparsity;
        SparsityCoordinates hes_sparsity;
        deca->calc_sparsity(deca->make_initial_guess_from_bounds(),
                jac_sparsity, true, hes_sparsity);
        num_hessian_nonzeros = (int)hes_sparsity.row.size();
    }

    // Hessian.
    const double obj_factor = 1.0;
    VectorXd lambda(problem.get_num_constraints());
    VectorXd expected_hessian_values(num_hessian_nonzeros);
    deca->calc_hessian_lagrangian(
            problem.get_num_variables(), x.data(), true, obj_factor,
            problem.get_num_constraints(), lambda.data(), true,
            num_hessian_nonzeros, expected_hessian_values.data());

    // Finite differences.
    // -------------------
    SECTION("Finite differences") {
        Unconstrained<double> problemd;
        auto decorator = problemd.make_decorator();
        // Must first initialize.
        SparsityCoordinates jac_sparsity;
        SparsityCoordinates hes_sparsity;
        decorator->calc_sparsity(decorator->make_random_iterate_within_bounds(),
                jac_sparsity, true, hes_sparsity);

        // Hessian sparsity.
        std::vector<unsigned int> expected_hess_row_indices{
                0, 0, 0,
                1, 1,
                2};
        std::vector<unsigned int> expected_hess_col_indices{
                0, 1, 2,
                1, 2,
                2};

        const unsigned num_hessian_nonzeros =
                (unsigned)hes_sparsity.row.size();
        REQUIRE(hes_sparsity.row == expected_hess_row_indices);
        REQUIRE(hes_sparsity.col == expected_hess_col_indices);

        // Hessian (of the Lagrangian).
        VectorXd actual_hessian_values(num_hessian_nonzeros);
        decorator->calc_hessian_lagrangian(
                problemd.get_num_variables(), x.data(), true, obj_factor,
                problem.get_num_constraints(), lambda.data(), true,
                num_hessian_nonzeros, actual_hessian_values.data());
        for (int inz = 0; inz < (int)num_hessian_nonzeros; ++inz) {
            REQUIRE(expected_hessian_values[inz] ==
                    Approx(actual_hessian_values[inz]).epsilon(1e-5));
        }
    }
}

template<typename T>
class HS071 : public Problem<T> {
public:
    HS071() : Problem<T>(4, 2) {
        this->set_variable_bounds(Vector4d(1, 1, 1, 1), Vector4d(5, 5, 5, 5));
        this->set_constraint_bounds(Vector2d(25, 40), Vector2d(2e19, 40.0));
    }
    void calc_objective(const VectorX<T>& x, T& obj_value) const override {
        obj_value = x[0] * x[3] * (x[0] + x[1] + x[2]) + x[2];
    }
    void calc_constraints(
            const VectorX<T>& x, Eigen::Ref<VectorX<T>> constr) const override {
        constr[0] = x.prod();
        constr[1] = x.squaredNorm();
    }
    void analytical_gradient(const VectorXd& x,
            Eigen::Ref<VectorXd> grad) const {
        assert(grad.size() == this->get_num_variables());
        grad[0] = x[3] * (x[0] + x[1] + x[2]) + x[0] * x[3];
        grad[1] = x[0] * x[3];
        grad[2] = x[0] * x[3] + 1;
        grad[3] = x[0] * (x[0] + x[1] + x[2]);
    }
    void analytical_jacobian(const VectorXd& x,
            Eigen::Ref<MatrixXd> jacobian) {
        assert(jacobian.rows() == this->get_num_constraints());
        assert(jacobian.cols() == this->get_num_variables());
        jacobian(0, 0) = x[1] * x[2] * x[3];
        jacobian(0, 1) = x[0] * x[2] * x[3];
        jacobian(0, 2) = x[0] * x[1] * x[3];
        jacobian(0, 3) = x[0] * x[1] * x[2];

        jacobian(1, 0) = 2 * x[0];
        jacobian(1, 1) = 2 * x[1];
        jacobian(1, 2) = 2 * x[2];
        jacobian(1, 3) = 2 * x[3];
    }

    /// Only the upper right triangle is filled in.
    void analytical_hessian_lagrangian(const VectorXd& x,
            const double& obj_factor,
            const VectorXd& lambda,
            Eigen::Ref<MatrixXd> hessian) {
        assert(hessian.rows() == this->get_num_variables());
        assert(hessian.cols() == this->get_num_variables());
        // obj_factor * grad^2 f(x)
        hessian(0, 0) = obj_factor * (2 * x[3]);
        hessian(0, 1) = obj_factor * (x[3]);
        hessian(1, 1) = 0;
        hessian(0, 2) = obj_factor * (x[3]);
        hessian(1, 2) = 0;
        hessian(2, 2) = 0;
        hessian(0, 3) = obj_factor * (2 * x[0] + x[1] + x[2]);
        hessian(1, 3) = obj_factor * (x[0]);
        hessian(2, 3) = obj_factor * (x[0]);
        hessian(3, 3) = 0;

        // lambda_0 * grad^2 g_0(x)
        hessian(0, 1) += lambda[0] * (x[2] * x[3]);
        hessian(0, 2) += lambda[0] * (x[1] * x[3]);
        hessian(1, 2) += lambda[0] * (x[0] * x[3]);
        hessian(0, 3) += lambda[0] * (x[1] * x[2]);
        hessian(1, 3) += lambda[0] * (x[0] * x[2]);
        hessian(2, 3) += lambda[0] * (x[0] * x[1]);

        // lambda_1 * grad^2 g_1(x)
        hessian(0, 0) += lambda[1] * 2;
        hessian(1, 1) += lambda[1] * 2;
        hessian(2, 2) += lambda[1] * 2;
        hessian(3, 3) += lambda[1] * 2;
    }
};

//TEST_CASE("Check finite differences with analytical deriv.", "[finitediff]")
//{
//    HS071<double> problem;
//    VectorXd x(problem.get_num_variables());
//    x << 1.5, 1.6, 1.7, 1.8;
//    VectorXd gradient;
//    problem.analytical_gradient(x, gradient);
//    std::cout << "DEBUG " << gradient << std::endl;
//}

TEST_CASE("Check derivatives with analytical deriv.")
{
    HS071<adouble> problem;
    VectorXd x(problem.get_num_variables());
    x << 1.5, 1.6, 1.7, 1.8;

    // Analytical derivatives.
    // -----------------------

    // Gradient.
    VectorXd analytical_gradient(problem.get_num_variables());
    problem.analytical_gradient(x, analytical_gradient);
    // Hessian.
    MatrixXd analytical_hessian(problem.get_num_variables(),
            problem.get_num_variables());
    const double obj_factor = 1.0;
    VectorXd lambda(problem.get_num_constraints()); lambda << 0.5, 1.5;
    problem.analytical_hessian_lagrangian(x, obj_factor, lambda,
            analytical_hessian);
    // Jacobian.
    MatrixXd analytical_jacobian(problem.get_num_constraints(),
            problem.get_num_variables());
    problem.analytical_jacobian(x, analytical_jacobian);

    // Finite differences.
    // -------------------
    SECTION("Finite differences") {
        HS071<double> problemd;
        auto decorator = problemd.make_decorator();
        // Must first initialize.
        SparsityCoordinates jac_sparsity;
        SparsityCoordinates hes_sparsity;
        decorator->calc_sparsity(decorator->make_initial_guess_from_bounds(),
                jac_sparsity, true, hes_sparsity);

        // Gradient.
        VectorXd fd_gradient(problem.get_num_variables());
        decorator->calc_gradient(problem.get_num_variables(), x.data(), true,
                fd_gradient.data());
        TROPTER_REQUIRE_EIGEN(analytical_gradient, fd_gradient, 1e-8);

        // Hessian (of the Lagrangian).
        std::vector<unsigned int> expected_hess_row_indices{
                0, 0, 0, 0,
                   1, 1, 1,
                      2, 2,
                         3};
        std::vector<unsigned int> expected_hess_col_indices{
                0, 1, 2, 3,
                   1, 2, 3,
                      2, 3,
                        3};
        REQUIRE(hes_sparsity.row == expected_hess_row_indices);
        REQUIRE(hes_sparsity.col == expected_hess_col_indices);
        const unsigned num_hessian_nonzeros =
                (unsigned)hes_sparsity.row.size();
        VectorXd fd_hessian_values(num_hessian_nonzeros);
        decorator->calc_hessian_lagrangian(
                problemd.get_num_variables(), x.data(), true, obj_factor,
                problem.get_num_constraints(), lambda.data(), true,
                num_hessian_nonzeros, fd_hessian_values.data());
        for (int inz = 0; inz < (int)num_hessian_nonzeros; ++inz) {
            const auto& i = hes_sparsity.row[inz];
            const auto& j = hes_sparsity.col[inz];
            REQUIRE(analytical_hessian(i, j) ==
                    Approx(fd_hessian_values[inz]).epsilon(1e-5));
        }

        // Jacobian.
        const auto num_jacobian_elem = problem.get_num_constraints() *
                problem.get_num_variables();
        // The Jacobian is dense.
        REQUIRE(jac_sparsity.row.size() == num_jacobian_elem);
        REQUIRE(jac_sparsity.col.size() == num_jacobian_elem);
        VectorXd fd_jacobian_values(num_jacobian_elem);
        decorator->calc_jacobian(problem.get_num_variables(), x.data(), true,
                num_jacobian_elem, fd_jacobian_values.data());
        INFO(analytical_jacobian);
        INFO(fd_jacobian_values);
        for (int inz = 0; inz < (int)num_jacobian_elem; ++inz) {
            const auto& i = jac_sparsity.row[inz];
            const auto& j = jac_sparsity.col[inz];
            REQUIRE(analytical_jacobian(i, j) ==
                    Approx(fd_jacobian_values[inz]).margin(1e-8));
        }
    }

    // Automatic derivatives.
    // ----------------------
    SECTION("ADOL-C") {

        auto decorator = problem.make_decorator();
        // Must first initialize the ADOL-C tapes.
        SparsityCoordinates jac_sparsity;
        SparsityCoordinates hes_sparsity;
        decorator->calc_sparsity(decorator->make_initial_guess_from_bounds(),
                jac_sparsity, true, hes_sparsity);

        // Gradient.
        VectorXd adolc_gradient(problem.get_num_variables());
        decorator->calc_gradient(problem.get_num_variables(), x.data(), true,
                adolc_gradient.data());
        TROPTER_REQUIRE_EIGEN(analytical_gradient, adolc_gradient, 1e-16);

        // Hessian (of the Lagrangian).
        const unsigned num_hessian_nonzeros =
                (unsigned)hes_sparsity.row.size();
        VectorXd adolc_hessian_values(num_hessian_nonzeros);
        decorator->calc_hessian_lagrangian(problem.get_num_variables(),
                x.data(), true,
                obj_factor, problem.get_num_constraints(), lambda.data(), true,
                num_hessian_nonzeros, adolc_hessian_values.data());
        for (int inz = 0; inz < (int)num_hessian_nonzeros; ++inz) {
            const auto& i = hes_sparsity.row[inz];
            const auto& j = hes_sparsity.col[inz];
            REQUIRE(analytical_hessian(i, j) == adolc_hessian_values[inz]);
        }

        // Jacobian.
        const auto num_jacobian_elem = problem.get_num_constraints() *
                problem.get_num_variables();
        // The Jacobian is dense.
        REQUIRE(jac_sparsity.row.size() == num_jacobian_elem);
        REQUIRE(jac_sparsity.col.size() == num_jacobian_elem);
        VectorXd adolc_jacobian_values(num_jacobian_elem);
        decorator->calc_jacobian(problem.get_num_variables(), x.data(), true,
                num_jacobian_elem, adolc_jacobian_values.data());
        for (int inz = 0; inz < (int)num_jacobian_elem; ++inz) {
            const auto& i = jac_sparsity.row[inz];
            const auto& j = jac_sparsity.col[inz];
            REQUIRE(analytical_jacobian(i, j) == adolc_jacobian_values[inz]);
        }
    }
}

template<typename T>
class SparseJacobian : public Problem<T> {
public:
    SparseJacobian() : Problem<T>(4, 5) {
        this->set_variable_bounds(Vector4d(1, 1, 1, 1), Vector4d(5, 5, 5, 5));
        this->set_constraint_bounds(VectorXd::Ones(5) * -2e19,
                                    VectorXd::Ones(5) *  2e19);
    }
    void calc_objective(const VectorX<T>& x, T& obj_value) const override {
        obj_value = x.squaredNorm();
    }
    void calc_constraints(
            const VectorX<T>& x, Eigen::Ref<VectorX<T>> constr) const override {
        const int m = this->get_num_constraints();
        const int n = (int)x.size();
        constr.setZero();
        // Sparsity pattern (and order of jacobian_values).
        // 0 . . .
        // 1 2 . .
        // . 3 4 .
        // . . 5 6
        // . . . 7
        for (int i = 0; i < m; ++i) {
            for (int j = std::max(i - 1, 0); j < std::min(i + 1, n); ++j) {
                constr[i] += x[j] * x[j];
            }
        }
    }
    void analytical_gradient(const VectorXd& x,
            Eigen::Ref<VectorXd> grad) const {
        assert(grad.size() == this->get_num_variables());
        grad.setZero();
        for (int i = 0; i < grad.size(); ++i) {
            grad[i] = 2 * x[i];
        }
    }
    void analytical_jacobian(const VectorXd& x,
            Eigen::Ref<MatrixXd> jacobian) {
        const int m = this->get_num_constraints();
        const int n = (int)x.size();
        assert(jacobian.rows() == m);
        assert(jacobian.cols() == n);
        jacobian.setZero();
        for (int i = 0; i < m; ++i) {
            for (int j = std::max(i - 1, 0); j < std::min(i + 1, n); ++j) {
                jacobian(i, j) += 2 * x[j];
            }
        }
    }

    /// Only the upper right triangle is filled in.
    void analytical_hessian_lagrangian(const VectorXd& x,
            const double& obj_factor,
            const VectorXd& lambda,
            Eigen::Ref<MatrixXd> hessian) {
        assert(hessian.rows() == this->get_num_variables());
        assert(hessian.cols() == this->get_num_variables());
        // obj_factor * grad^2 f(x)
        hessian.setZero();
        for (int i = 0; i < (int)this->get_num_variables(); ++i) {
            hessian(i, i) = obj_factor * 2;
        }

        // lambda_j * grad^2 g_j(x)
        const int m = this->get_num_constraints();
        const int n = (int)x.size();
        for (int i = 0; i < m; ++i) {
            for (int j = std::max(i - 1, 0); j < std::min(i + 1, n); ++j) {
                hessian(j, j) += lambda[i] * 2;
            }
        }
    }
};

TEST_CASE("Check derivatives with analytical deriv.; sparse Jacobian.")
{
    SparseJacobian<adouble> problem;
    VectorXd x(problem.get_num_variables());
    x << 3.1, -1.5, -0.25, 5.3;

    // Analytical derivatives.
    // -----------------------

    // Gradient.
    VectorXd analytical_gradient(problem.get_num_variables());
    problem.analytical_gradient(x, analytical_gradient);
    // Hessian.
    MatrixXd analytical_hessian(problem.get_num_variables(),
            problem.get_num_variables());
    const double obj_factor = 1.0;
    VectorXd lambda(problem.get_num_constraints());
    lambda << 0.5, 1.5, 2.5, 3.0, 0.19;
    problem.analytical_hessian_lagrangian(x, obj_factor, lambda,
            analytical_hessian);
    // Jacobian.
    MatrixXd analytical_jacobian(problem.get_num_constraints(),
            problem.get_num_variables());
    problem.analytical_jacobian(x, analytical_jacobian);
    const auto num_jacobian_elem = 2 * problem.get_num_variables();

    // Finite differences.
    // -------------------
    SECTION("Finite differences") {
        SparseJacobian<double> problemd;
        auto proxy = problemd.make_decorator();
        // Must first initialize.
        SparsityCoordinates jac_sparsity;
        SparsityCoordinates hes_sparsity;
        proxy->calc_sparsity(proxy->make_initial_guess_from_bounds(),
                jac_sparsity, true, hes_sparsity);

        // Gradient.
        VectorXd fd_gradient(problem.get_num_variables());
        proxy->calc_gradient(problem.get_num_variables(), x.data(), true,
                fd_gradient.data());
        INFO(analytical_gradient);
        INFO(fd_gradient);
        TROPTER_REQUIRE_EIGEN(analytical_gradient, fd_gradient, 1e-7);

        // Hessian (of the Lagrangian).
        std::vector<unsigned int> expected_hes_row_indices{
                0, 0, 0, 0,
                1, 1, 1,
                2, 2,
                3
        };
        std::vector<unsigned int> expected_hes_col_indices{
                0, 1, 2, 3,
                1, 2, 3,
                2, 3,
                3
        };
        REQUIRE(hes_sparsity.row == expected_hes_row_indices);
        REQUIRE(hes_sparsity.col == expected_hes_col_indices);

        const unsigned num_hessian_nonzeros = (unsigned)hes_sparsity.row.size();
        VectorXd actual_hessian_values(num_hessian_nonzeros);
        proxy->set_findiff_hessian_step_size(1e-3);
        proxy->calc_hessian_lagrangian(
                problemd.get_num_variables(), x.data(), true, obj_factor,
                problem.get_num_constraints(), lambda.data(), true,
                num_hessian_nonzeros, actual_hessian_values.data());
        for (int inz = 0; inz < (int)num_hessian_nonzeros; ++inz) {
            const auto& i = hes_sparsity.row[inz];
            const auto& j = hes_sparsity.col[inz];
            REQUIRE(analytical_hessian(i, j) ==
                    Approx(actual_hessian_values[inz]).margin(1e-7));
        }

        // Jacobian.
        REQUIRE(jac_sparsity.row.size() == num_jacobian_elem);
        REQUIRE(jac_sparsity.col.size() == num_jacobian_elem);
        VectorXd fd_jacobian_values(num_jacobian_elem);
        proxy->calc_jacobian(problem.get_num_variables(), x.data(), true,
                num_jacobian_elem, fd_jacobian_values.data());
        INFO(analytical_jacobian);
        INFO(fd_jacobian_values);
        for (int inz = 0; inz < (int)num_jacobian_elem; ++inz) {
            const auto& i = jac_sparsity.row[inz];
            const auto& j = jac_sparsity.col[inz];
            REQUIRE(analytical_jacobian(i, j) ==
                    Approx(fd_jacobian_values[inz]).epsilon(1e-8));
        }
    }

    // Automatic derivatives.
    // ----------------------
    SECTION("ADOL-C") {

        auto proxy = problem.make_decorator();
        // Must first initialize the ADOL-C tapes.
        SparsityCoordinates jac_sparsity;
        SparsityCoordinates hes_sparsity;
        proxy->calc_sparsity(proxy->make_initial_guess_from_bounds(),
                jac_sparsity, true, hes_sparsity);

        // Gradient.
        VectorXd adolc_gradient(problem.get_num_variables());
        proxy->calc_gradient(problem.get_num_variables(), x.data(), true,
                adolc_gradient.data());
        TROPTER_REQUIRE_EIGEN(analytical_gradient, adolc_gradient, 1e-16);

        // Hessian (of the Lagrangian).
        const unsigned num_hessian_nonzeros = (unsigned)hes_sparsity.row.size();
        VectorXd adolc_hessian_values(num_hessian_nonzeros);
        proxy->calc_hessian_lagrangian(problem.get_num_variables(), x.data(),
                true,
                obj_factor, problem.get_num_constraints(), lambda.data(), true,
                num_hessian_nonzeros, adolc_hessian_values.data());
        for (int inz = 0; inz < (int)num_hessian_nonzeros; ++inz) {
            const auto& i = hes_sparsity.row[inz];
            const auto& j = hes_sparsity.col[inz];
            REQUIRE(analytical_hessian(i, j) ==
                    Approx(adolc_hessian_values[inz]).epsilon(1e-15));
        }

        // Jacobian.
        REQUIRE(jac_sparsity.row.size() == num_jacobian_elem);
        REQUIRE(jac_sparsity.col.size() == num_jacobian_elem);
        VectorXd adolc_jacobian_values(num_jacobian_elem);
        proxy->calc_jacobian(problem.get_num_variables(), x.data(), true,
                num_jacobian_elem, adolc_jacobian_values.data());
        for (int inz = 0; inz < (int)num_jacobian_elem; ++inz) {
            const auto& i = jac_sparsity.row[inz];
            const auto& j = jac_sparsity.col[inz];
            REQUIRE(analytical_jacobian(i, j) == adolc_jacobian_values[inz]);
        }
    }
}

TEST_CASE("Check finite differences on bounds", "[finitediff][!mayfail]")
{
    HS071<adouble> problem;
    VectorXd x(problem.get_num_variables());
    x << 1, 5, 1, 5;

    // Preserve the integrity of the test by making sure that x actually lies
    // on the bounds.
    REQUIRE(x[0] == problem.get_variable_lower_bounds()[0]);
    REQUIRE(x[1] == problem.get_variable_upper_bounds()[1]);
    REQUIRE(x[2] == problem.get_variable_lower_bounds()[2]);
    REQUIRE(x[3] == problem.get_variable_upper_bounds()[3]);
   // TODO

    throw std::runtime_error("Test not written yet.");
}

// TODO try x with a very different magnitude (x = 1000, x = 1e-4).



template<typename T>
class SparseJacUserSpecifiedSparsity : public SparseJacobian<T> {
public:
    void calc_sparsity_hessian_lagrangian(const VectorXd&,
            SymmetricSparsityPattern& hescon_sparsity,
            SymmetricSparsityPattern& hesobj_sparsity) const override {
        m_calc_sparsity_hessian_lagrangian_called = true;
        assert(hescon_sparsity.get_num_rows() == 4);
        assert(hesobj_sparsity.get_num_rows() == 4);
        // Treat the Hessian as dense but with a zero at (0, 3) (conservative
        // estimate) so that we can detect that this function is called.
        const int num_vars = (int)this->get_num_variables();
        for (int i = 0; i < num_vars; ++i) {
            for (int j = i; j < num_vars; ++j) {
                if (i == 0 && j == 3) continue;
                hescon_sparsity.set_nonzero(i, j);
            }
            // Hessian of objective is diagonal.
            hesobj_sparsity.set_nonzero(i, i);
        }
    }
    mutable bool m_calc_sparsity_hessian_lagrangian_called = false;
};


TEST_CASE("User-supplied sparsity of Hessian of Lagrangian")
{

    SparseJacUserSpecifiedSparsity<adouble> problem;
    VectorXd x(problem.get_num_variables());
    x << 3.1, -1.5, -0.25, 5.3;

    // Analytical Hessian.
    // -------------------
    // Hessian.
    MatrixXd analytical_hessian(problem.get_num_variables(),
            problem.get_num_variables());
    const double obj_factor = 1.0;
    VectorXd lambda(problem.get_num_constraints());
    lambda << 0.5, 1.5, 2.5, 3.0, 0.19;
    problem.analytical_hessian_lagrangian(x, obj_factor, lambda,
            analytical_hessian);
    std::vector<unsigned int> expected_jac_row_indices{0, 1, 1, 2, 2, 3, 3, 4};
    std::vector<unsigned int> expected_jac_col_indices{0, 0, 1, 1, 2, 2, 3, 3};

    // TODO allow validating a user-supplied sparsity.
    SECTION("Finite differences") {
        {
            SparseJacUserSpecifiedSparsity<double> problemd;
            problemd.set_use_supplied_sparsity_hessian_lagrangian(true);
            auto decorator = problemd.make_decorator();
            SparsityCoordinates jac_sparsity, hes_sparsity;
            decorator->calc_sparsity(
                    decorator->make_initial_guess_from_bounds(),
                    jac_sparsity, true, hes_sparsity);
            REQUIRE(problemd.m_calc_sparsity_hessian_lagrangian_called);
            REQUIRE(jac_sparsity.row == expected_jac_row_indices);
            REQUIRE(jac_sparsity.col == expected_jac_col_indices);
            // Check that the pattern is as expected (dense with a zero
            // at (0, 3)).
            std::vector<unsigned int> expected_hess_row_indices{
                    0, 0, 0,
                       1, 1, 1,
                          2, 2,
                             3
            };
            std::vector<unsigned int> expected_hess_col_indices{
                    0, 1, 2,
                       1, 2, 3,
                          2, 3,
                             3
            };

            REQUIRE(hes_sparsity.row == expected_hess_row_indices);
            REQUIRE(hes_sparsity.col == expected_hess_col_indices);

            const unsigned num_hessian_nonzeros =
                    (unsigned)hes_sparsity.row.size();
            decorator->set_findiff_hessian_step_size(1e-3);
            VectorXd actual_hessian_values(num_hessian_nonzeros);
            decorator->calc_hessian_lagrangian(
                    problemd.get_num_variables(), x.data(), true, obj_factor,
                    problem.get_num_constraints(), lambda.data(), true,
                    num_hessian_nonzeros, actual_hessian_values.data());
            CAPTURE(analytical_hessian);
            CAPTURE(actual_hessian_values);
            for (int inz = 0; inz < (int)num_hessian_nonzeros; ++inz) {
                const auto& i = hes_sparsity.row[inz];
                const auto& j = hes_sparsity.col[inz];
                INFO(inz << " (" << i << " " << j << ")");
                REQUIRE(analytical_hessian(i, j) ==
                        Approx(actual_hessian_values[inz]).epsilon(1e-5));
            }
        }
        {
            // Do not use supplied sparsity.
            // If user does not supply sparsity, then the Hessian sparsity is
            // determined from the Jacobian and gradient sparsity pattern.
            SparseJacUserSpecifiedSparsity<double> problemd;
            // *** KEY DIFFERENCE IN THIS TEST ***
            problemd.set_use_supplied_sparsity_hessian_lagrangian(false);
            // ^^^ KEY DIFFERENCE IN THIS TEST ^^^
            auto decorator = problemd.make_decorator();
            SparsityCoordinates jac_sparsity, hes_sparsity;
            decorator->calc_sparsity(
                    decorator->make_initial_guess_from_bounds(),
                    jac_sparsity, true, hes_sparsity);
            std::vector<unsigned int> expected_hess_row_indices{
                    0, 0, 0, 0,
                       1, 1, 1,
                          2, 2,
                             3
            };
            std::vector<unsigned int> expected_hess_col_indices{
                    0, 1, 2, 3,
                       1, 2, 3,
                          2, 3,
                             3
            };
            REQUIRE(hes_sparsity.row == expected_hess_row_indices);
            REQUIRE(hes_sparsity.col == expected_hess_col_indices);

            // Evaluate Hessian to ensure entries are the desired order.
            const unsigned num_hessian_nonzeros =
                    (unsigned)hes_sparsity.row.size();
            VectorXd actual_hessian_values(num_hessian_nonzeros);
            decorator->set_findiff_hessian_step_size(1e-3);
            decorator->calc_hessian_lagrangian(
                    problemd.get_num_variables(), x.data(), true, obj_factor,
                    problem.get_num_constraints(), lambda.data(), true,
                    num_hessian_nonzeros, actual_hessian_values.data());
            for (int inz = 0; inz < (int)num_hessian_nonzeros; ++inz) {
                const auto& i = hes_sparsity.row[inz];
                const auto& j = hes_sparsity.col[inz];
                REQUIRE(analytical_hessian(i, j) ==
                        Approx(actual_hessian_values[inz]).margin(1e-7));
            }
        }
        {
            // Requesting use of supplied sparsity if none is provided causes
            // an exception.
            SparseJacobian<double> problemd;
            problemd.set_use_supplied_sparsity_hessian_lagrangian(true);
            auto decorator = problemd.make_decorator();
            SparsityCoordinates jac_sparsity, hes_sparsity;
            REQUIRE_THROWS_WITH(
                    decorator->calc_sparsity(
                            decorator->make_initial_guess_from_bounds(),
                            jac_sparsity, true, hes_sparsity),
                    Catch::Contains("requested use of user-supplied sparsity"));
        }
    }

    SECTION("ADOL-C") {
        // Requesting use of supplied sparsity with ADOL-C causes an error.
        SparseJacUserSpecifiedSparsity<adouble> problem;
        problem.set_use_supplied_sparsity_hessian_lagrangian(true);
        auto decorator = problem.make_decorator();
        SparsityCoordinates jac_sparsity, hes_sparsity;
        REQUIRE_THROWS_WITH(
                decorator->calc_sparsity(
                        decorator->make_initial_guess_from_bounds(),
                        jac_sparsity, true, hes_sparsity),
                Catch::Contains("Cannot use supplied sparsity pattern"));
    }
}

TEST_CASE("Validate sparsity input") {

    SECTION("Number of rows") {
        class ConfigurableUserSpecifiedSparsity : public Problem<double> {
        public:
            ConfigurableUserSpecifiedSparsity() : Problem<double>(4, 0) {}
            void calc_objective(const VectorXd&, double&) const override {}
            void calc_constraints(const VectorXd&,
                    Eigen::Ref<VectorXd>) const override {}
            void calc_sparsity_hessian_lagrangian(
                    const VectorXd&, SymmetricSparsityPattern& hescon_sparsity,
                    SymmetricSparsityPattern& hesobj_sparsity) const override {
                hescon_sparsity = m_hescon_sparsity;
                hesobj_sparsity = m_hesobj_sparsity;
            }
            SymmetricSparsityPattern m_hescon_sparsity =
                    SymmetricSparsityPattern(4);
            SymmetricSparsityPattern m_hesobj_sparsity =
                    SymmetricSparsityPattern(4);
        };
        ConfigurableUserSpecifiedSparsity problemd;
        problemd.set_use_supplied_sparsity_hessian_lagrangian(true);
        SparsityCoordinates jac_sparsity, hes_sparsity;

        auto decorator = problemd.make_decorator();

        problemd.m_hescon_sparsity = SymmetricSparsityPattern(3);
        REQUIRE_THROWS_WITH(
                decorator->calc_sparsity(Vector4d(1, 2, 3, 4),
                        jac_sparsity, true, hes_sparsity),
                Catch::Contains("Expected sparsity pattern of Hessian of "
                        "constraints to have dimensions 4"));

        problemd.m_hescon_sparsity = SymmetricSparsityPattern(4);
        problemd.m_hesobj_sparsity = SymmetricSparsityPattern(5);
        REQUIRE_THROWS_WITH(
                decorator->calc_sparsity(Vector4d(1, 2, 3, 4),
                        jac_sparsity, true, hes_sparsity),
                Catch::Contains("Expected sparsity pattern of Hessian of "
                        "objective to have dimensions 4"));
    }

    SECTION("Column indices too large") {
        {
            SparsityPattern s(2, 3);
            REQUIRE_THROWS_WITH(s.set_nonzero(2, 0),
                    Catch::Contains("Expected row_index to be in [0, 2)"));
            REQUIRE_THROWS_WITH(s.set_nonzero(0, 3),
                    Catch::Contains("Expected col_index to be in [0, 3)"));
        }
        {
            SymmetricSparsityPattern s(5);
            REQUIRE_THROWS_WITH(s.set_nonzero(0, 5),
                    Catch::Contains("Expected col_index to be in [0, 5)"));
        }
    }

    SECTION("Idempotency") {
        {
            SparsityPattern s(2, 2);
            s.set_nonzero(0, 0);
            s.set_nonzero(0, 0);
            REQUIRE(s.get_num_nonzeros() == 1);
        }
        {
            SymmetricSparsityPattern sparsity(2);
            sparsity.set_nonzero(1, 1);
            sparsity.set_nonzero(1, 1);
            REQUIRE(sparsity.get_num_nonzeros() == 1);
        }
    }

    SECTION("Lower triangle") {
        SymmetricSparsityPattern sparsity(2);
        REQUIRE_THROWS_WITH(sparsity.set_nonzero(1, 0),
                Catch::Contains("must be in the upper triangle"));
    }

}


// Provide access to the protected calc_sparsity() function.
class IPOPTSolverCalcSparsity : public IPOPTSolver {
public:
    using IPOPTSolver::IPOPTSolver;
    void calc_sparsity(const Eigen::VectorXd guess,
            SparsityCoordinates& js, bool provide_hessian_sparsity,
            SparsityCoordinates& hs) const {
        IPOPTSolver::calc_sparsity(guess, js, provide_hessian_sparsity, hs);
    }
};
template <typename T>
class SparsityDetectionProblem : public Problem<T> {
public:
    SparsityDetectionProblem() : Problem<T>(2, 2) {
        this->set_variable_bounds(Vector2d(0, 0), Vector2d(1, 1));
        this->set_constraint_bounds(Vector2d(0, 0), Vector2d(0, 0));
    }
    void calc_objective(const VectorX<T>&, T& obj_value)
    const override {
        obj_value = 0;
    }
    void calc_constraints(const VectorX<T>& x, Eigen::Ref<VectorX<T>> constr)
    const override {
        // If x is random, the Jacobian has 2 entries.
        // If x is the initial guess, the Jacobian has 3 entries.
        constr.setZero();
        constr[0] = x[0] * x[1];
        // Need a loose tolerance, as the perturbation
        if ((guess->cast<T>() - x).norm() < 1e-4) {
            constr[1] = x[1];
        }
    }
    const VectorXd* guess = nullptr;

    static void run_test() {
        SparsityDetectionProblem<double> problem;
        const auto decorator = problem.make_decorator();
        IPOPTSolverCalcSparsity solver(problem);
        REQUIRE(solver.get_sparsity_detection() == "initial-guess");
        SparsityCoordinates jac_sparsity, hes_sparsity;
        VectorXd guess = decorator->make_initial_guess_from_bounds();
        problem.guess = &guess;
        solver.calc_sparsity(guess, jac_sparsity, false, hes_sparsity);
        REQUIRE(jac_sparsity.row.size() == 3);
        REQUIRE(jac_sparsity.col.size() == 3);

        solver.set_sparsity_detection("random");
        solver.calc_sparsity(guess, jac_sparsity, false, hes_sparsity);
        REQUIRE(jac_sparsity.row.size() == 2);
        REQUIRE(jac_sparsity.col.size() == 2);

        REQUIRE_THROWS(solver.set_sparsity_detection("invalid"));
    }
};
TEST_CASE("ProblemDecorator sparsity_detection") {
    // Ensure that the sparsity_detection setting is processed properly.

    SparsityDetectionProblem<double>::run_test();
    SparsityDetectionProblem<adouble>::run_test();
}

// TODO add test_derivatives_optimal_control
