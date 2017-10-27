#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <tropter/tropter.h>

#include "testing.h"

// TODO remove this test case, bring it into test_derivatives.cpp

using Eigen::Ref;
using Eigen::VectorXd;
using Eigen::RowVectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Vector5d = Eigen::Matrix<double, 5, 1>;
using Eigen::MatrixXd;

using namespace tropter;

template<typename T>
class Unconstrained : public OptimizationProblem<T> {
public:
    Unconstrained() : OptimizationProblem<T>(3, 0)
    {
        this->set_variable_bounds(Vector3d(-3, -3, -3), Vector3d(3, 3, 3));
    }
    void calc_objective(const VectorX<T>& x, T& obj_value) const override
    {
        obj_value = x[0]*x[0] * x[1]*x[1] * x[2]*x[2];
    }
};

TEST_CASE("TODO")
{
    Unconstrained<adouble> problem;
    VectorXd x(problem.get_num_variables());
    x << 1.5, 1.6, 1.7;

    auto deca = problem.make_decorator();

    // Expected derivatives.
    // -------------------
    std::vector<unsigned int> jacobian_row_indices;
    std::vector<unsigned int> jacobian_col_indices;
    std::vector<unsigned int> hessian_row_indices;
    std::vector<unsigned int> hessian_col_indices;
    deca->calc_sparsity(deca->make_initial_guess_from_bounds(),
            jacobian_row_indices, jacobian_col_indices,
            true, hessian_row_indices, hessian_col_indices);

    // Hessian.
    const double obj_factor = 1.0;
    VectorXd lambda(problem.get_num_constraints());
    int num_hessian_nonzeros = (int)hessian_row_indices.size();
    VectorXd expected_hessian_values(num_hessian_nonzeros);
    deca->calc_hessian_lagrangian(
            problem.get_num_variables(), x.data(), false, obj_factor,
            problem.get_num_constraints(), lambda.data(), false,
            num_hessian_nonzeros, expected_hessian_values.data());

    std::cout << "expected hessian:\n" << expected_hessian_values << std::endl;

    // Finite differences.
    // -------------------
    SECTION("Finite differences") {
        Unconstrained<double> problemd;
        auto decorator = problemd.make_decorator();
        // Must first initialize.
        std::vector<unsigned int> jacobian_row_indices;
        std::vector<unsigned int> jacobian_col_indices;
        std::vector<unsigned int> hessian_row_indices;
        std::vector<unsigned int> hessian_col_indices;
        decorator->calc_sparsity(decorator->make_initial_guess_from_bounds(),
                jacobian_row_indices, jacobian_col_indices,
                true, hessian_row_indices, hessian_col_indices);

        std::vector<unsigned int> expected_hess_row_indices{
                0, 0, 0,
                1, 1,
                2};
        std::vector<unsigned int> expected_hess_col_indices{
                0, 1, 2,
                1, 2,
                2};

        const unsigned num_hessian_nonzeros =
                (unsigned)hessian_row_indices.size();
        //std::cout << "DEBUG ColPack's coordinate sparsity" << std::endl;
        //for (int inz = 0; inz < (int)num_hessian_nonzeros; ++inz) {
        //    std::cout << "(" << hessian_row_indices[inz] << "," <<
        //            hessian_col_indices[inz] << ")" << std::endl;
        //}
        REQUIRE(hessian_row_indices == expected_hess_row_indices);
        REQUIRE(hessian_col_indices == expected_hess_col_indices);

        // Hessian sparsity.


        // Hessian (of the Lagrangian).
        VectorXd actual_hessian_values(num_hessian_nonzeros);
        decorator->calc_hessian_lagrangian(
                problemd.get_num_variables(), x.data(), false, obj_factor,
                problem.get_num_constraints(), lambda.data(), false,
                num_hessian_nonzeros, actual_hessian_values.data());
        for (int inz = 0; inz < (int)num_hessian_nonzeros; ++inz) {
            // TODO choose error bound.
            REQUIRE(expected_hessian_values[inz] ==
                    Approx(actual_hessian_values[inz]).epsilon(1e-1));
        }
    }

    // TODO how is hessian seed all 1's in first column and all other columns
    // are 0, if we have a dense hessian?

    // TODO maybe sparsity pattern has to be nonsymmetric?
}

template<typename T>
class HS071 : public OptimizationProblem<T> {
public:
    HS071() : OptimizationProblem<T>(4, 2) {
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
        std::cout << "DEBUG analytical grad^2 f\n " << hessian << std::endl;

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
    analytical_hessian.setZero();
    const double obj_factor = 1.0;
    VectorXd lambda(problem.get_num_constraints());
    lambda << 0.5, 1.5;
    problem.analytical_hessian_lagrangian(x, obj_factor, lambda,
            analytical_hessian);
    std::cout << "analytical hessian:\n" << analytical_hessian << std::endl;

    // Finite differences.
    // -------------------
    SECTION("Finite differences") {
        HS071<double> problemd;
        auto decorator = problemd.make_decorator();
        // Must first initialize.
        std::vector<unsigned int> jacobian_row_indices;
        std::vector<unsigned int> jacobian_col_indices;
        std::vector<unsigned int> hessian_row_indices;
        std::vector<unsigned int> hessian_col_indices;
        decorator->calc_sparsity(decorator->make_initial_guess_from_bounds(),
                jacobian_row_indices, jacobian_col_indices,
                true, hessian_row_indices, hessian_col_indices);

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

        REQUIRE(hessian_row_indices == expected_hess_row_indices);
        REQUIRE(hessian_col_indices == expected_hess_col_indices);

        // Hessian sparsity.
        const unsigned num_hessian_nonzeros =
                (unsigned)hessian_row_indices.size();


        // Hessian (of the Lagrangian).
        VectorXd actual_hessian_values(num_hessian_nonzeros);
        decorator->calc_hessian_lagrangian(
                problemd.get_num_variables(), x.data(), false, obj_factor,
                problem.get_num_constraints(), lambda.data(), false,
                num_hessian_nonzeros, actual_hessian_values.data());
        for (int inz = 0; inz < (int)num_hessian_nonzeros; ++inz) {
            const auto& i = hessian_row_indices[inz];
            const auto& j = hessian_col_indices[inz];
            REQUIRE(analytical_hessian(i, j) ==
                    Approx(actual_hessian_values[inz]).epsilon(1e-3));
        }
    }

    // TODO how is hessian seed all 1's in first column and all other columns
    // are 0, if we have a dense hessian?

    // TODO maybe sparsity pattern has to be nonsymmetric?
}
