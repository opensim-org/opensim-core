// ----------------------------------------------------------------------------
// tropter: test_sliding_mass_minimum_effort.cpp
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

#include <tropter/tropter.h>

#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include "testing_optimalcontrol.h"

using Eigen::Ref;
using Eigen::VectorXd;
using Eigen::RowVectorXd;
using Eigen::Vector2d;
using Eigen::MatrixXd;

using namespace tropter;

template<typename T>
class SlidingMass : public tropter::Problem<T> {
public:
    SlidingMass() {
        this->set_time({0}, {2});
        this->add_state("x", {0, 2}, {0}, {1});
        this->add_state("u", {-10, 10}, {0}, {0});
        this->add_control("F", {-50, 50});
        this->add_cost("effort", 1);
    }
    const double mass = 10.0;
    void calc_differential_algebraic_equations(
            const Input<T>& in, Output<T> out) const override {
        out.dynamics[0] = in.states[1];
        out.dynamics[1] = in.controls[0] / mass;
    }
    // TODO alternate form that takes a matrix; state at every time.
    //virtual void continuous(const MatrixXd& x, MatrixXd& xdot) const = 0;
    void calc_cost(int, const CostInput<T>& in, T& cost) const override {
        cost = in.integral;
    }
    void calc_cost_integrand(
            int, const Input<T>& in, T& integrand) const override {
        const auto& controls = in.controls;
        integrand = controls[0] * controls[0];
    }
};

TEST_CASE("IPOPT") {

    SECTION("ADOL-C, trapezoidal") {
        auto ocp = std::make_shared<SlidingMass<adouble>>();
        DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal", "ipopt");
        Solution solution = dircol.solve();
        solution.write("sliding_mass_solution.csv");
        //Iterate initial_guess = ocp->make_guess_template();
        //Solution solution = dircol.solve(initial_guess);

        // Initial and final position.
        REQUIRE(Approx(solution.states(0, 0)) == 0.0);
        REQUIRE(Approx(solution.states.rightCols<1>()[0]) == 1.0);
        // Initial and final speed.
        REQUIRE(Approx(solution.states(1, 0)) == 0.0);
        REQUIRE(Approx(solution.states.rightCols<1>()[1]) == 0.0);

        int N = (int)solution.time.size();
        //std::cout << "DEBUG solution.controls " << solution.controls << std::endl;
        // TODO is this really the correct solution?
        RowVectorXd expected = RowVectorXd::LinSpaced(N - 2, 14.6119, -14.6119);
        TROPTER_REQUIRE_EIGEN(solution.controls.middleCols(1, N - 2), expected,
                0.1);
        //RowVectorXd errors = solution.controls.rightCols(N - 1) - expected;
        //REQUIRE(Approx(errors.norm()) == 0);

    }
    SECTION("ADOL-C, hermite-simpson") {
        auto ocp = std::make_shared<SlidingMass<adouble>>();
        DirectCollocationSolver<adouble> dircol(ocp, "hermite-simpson", 
            "ipopt");
        Solution solution = dircol.solve();
        solution.write("sliding_mass_solution.csv");
        //Iterate initial_guess = ocp->make_guess_template();
        //Solution solution = dircol.solve(initial_guess);

        // Initial and final position.
        REQUIRE(Approx(solution.states(0, 0)) == 0.0);
        REQUIRE(Approx(solution.states.rightCols<1>()[0]) == 1.0);
        // Initial and final speed.
        REQUIRE(Approx(solution.states(1, 0)) == 0.0);
        REQUIRE(Approx(solution.states.rightCols<1>()[1]) == 0.0);

        int N = (int)solution.time.size();
        //std::cout << "DEBUG solution.controls " << solution.controls << std::endl;
        // TODO is this really the correct solution?
        RowVectorXd expected = RowVectorXd::LinSpaced(N - 2, 14.6119, -14.6119);
        TROPTER_REQUIRE_EIGEN(solution.controls.middleCols(1, N - 2), expected,
            0.1);
        //RowVectorXd errors = solution.controls.rightCols(N - 1) - expected;
        //REQUIRE(Approx(errors.norm()) == 0);
    }
    SECTION("Compare derivatives") {
        OCPDerivativesComparison<SlidingMass> comp;
        comp.findiff_hessian_step_size = 1e-3;
        comp.gradient_error_tolerance = 1e-5;
        comp.hessian_error_tolerance = 1e-3;
        comp.compare();
    }
    SECTION("Finite differences, limited memory, trapezoidal") {
        auto ocp = std::make_shared<SlidingMass<double>>();
        DirectCollocationSolver<double> dircol(ocp, "trapezoidal", "ipopt");
        dircol.get_opt_solver().set_findiff_hessian_step_size(1e-3);
        dircol.get_opt_solver()
            .set_jacobian_approximation("finite-difference-values");
        dircol.get_opt_solver().set_hessian_approximation("limited-memory");
        Solution solution = dircol.solve();
        REQUIRE(Approx(solution.states(0, 0)) == 0.0);
        REQUIRE(Approx(solution.states.rightCols<1>()[0]) == 1.0);
        // Initial and final speed.
        REQUIRE(Approx(solution.states(1, 0)) == 0.0);
        REQUIRE(Approx(solution.states.rightCols<1>()[1]) == 0.0);
        int N = (int)solution.time.size();
        RowVectorXd expected = RowVectorXd::LinSpaced(N - 2, 14.6119, -14.6119);
        TROPTER_REQUIRE_EIGEN(solution.controls.middleCols(1, N - 2), expected,
                0.1);
    }
    SECTION("Finite differences, limited memory, hermite-simpson") {
        auto ocp = std::make_shared<SlidingMass<double>>();
        DirectCollocationSolver<double> dircol(ocp, "hermite-simpson", "ipopt");
        dircol.get_opt_solver().set_findiff_hessian_step_size(1e-3);
        dircol.get_opt_solver()
            .set_jacobian_approximation("finite-difference-values");
        dircol.get_opt_solver().set_hessian_approximation("limited-memory");
        Solution solution = dircol.solve();
        REQUIRE(Approx(solution.states(0, 0)) == 0.0);
        REQUIRE(Approx(solution.states.rightCols<1>()[0]) == 1.0);
        // Initial and final speed.
        REQUIRE(Approx(solution.states(1, 0)) == 0.0);
        REQUIRE(Approx(solution.states.rightCols<1>()[1]) == 0.0);
        int N = (int)solution.time.size();
        RowVectorXd expected = RowVectorXd::LinSpaced(N - 2, 14.6119, -14.6119);
        TROPTER_REQUIRE_EIGEN(solution.controls.middleCols(1, N - 2), expected,
            0.1);
    }
    SECTION("Finite differences, exact Hessian, trapezoidal") {
        auto ocp = std::make_shared<SlidingMass<double>>();
        DirectCollocationSolver<double> dircol(ocp, "trapezoidal", "ipopt");
        dircol.get_opt_solver().set_findiff_hessian_step_size(1e-3);
        dircol.get_opt_solver().set_jacobian_approximation("exact");
        dircol.get_opt_solver().set_hessian_approximation("exact");
        Solution solution = dircol.solve();
        REQUIRE(Approx(solution.states(0, 0)) == 0.0);
        REQUIRE(Approx(solution.states.rightCols<1>()[0]) == 1.0);
        // Initial and final speed.
        REQUIRE(Approx(solution.states(1, 0)) == 0.0);
        REQUIRE(Approx(solution.states.rightCols<1>()[1]) == 0.0);
        int N = (int)solution.time.size();
        RowVectorXd expected = RowVectorXd::LinSpaced(N - 2, 14.6119, -14.6119);
        TROPTER_REQUIRE_EIGEN(solution.controls.middleCols(1, N - 2), expected,
                0.1);
    }
    SECTION("Finite differences, exact Hessian, hermite-simpson") {
        auto ocp = std::make_shared<SlidingMass<double>>();
        DirectCollocationSolver<double> dircol(ocp, "hermite-simpson", "ipopt");
        dircol.get_opt_solver().set_findiff_hessian_step_size(1e-3);
        dircol.get_opt_solver().set_jacobian_approximation("exact");
        dircol.get_opt_solver().set_hessian_approximation("exact");
        Solution solution = dircol.solve();
        REQUIRE(Approx(solution.states(0, 0)) == 0.0);
        REQUIRE(Approx(solution.states.rightCols<1>()[0]) == 1.0);
        // Initial and final speed.
        REQUIRE(Approx(solution.states(1, 0)) == 0.0);
        REQUIRE(Approx(solution.states.rightCols<1>()[1]) == 0.0);
        int N = (int)solution.time.size();
        solution.write("sliding_mass_solution_hermite_simpson.csv");
        RowVectorXd expected = RowVectorXd::LinSpaced(N - 2, 14.6119, -14.6119);
        TROPTER_REQUIRE_EIGEN(solution.controls.middleCols(1, N - 2), expected,
            0.1);
    }
}

#if defined(TROPTER_WITH_SNOPT)
TEST_CASE("SNOPT, trapezoidal") {

    auto ocp = std::make_shared<SlidingMass<double>>();
    DirectCollocationSolver<double> dircol(ocp, "trapezoidal", "snopt");
    Solution solution = dircol.solve();
    solution.write("sliding_mass_solution.csv");
    //Iterate initial_guess = ocp->make_guess_template();
    //Solution solution = dircol.solve(initial_guess);

    // Initial and final position.
    REQUIRE(Approx(solution.states(0, 0)) == 0.0);
    REQUIRE(Approx(solution.states.rightCols<1>()[0]) == 1.0);
    // Initial and final speed.
    REQUIRE(Approx(solution.states(1, 0)) == 0.0);
    REQUIRE(Approx(solution.states.rightCols<1>()[1]) == 0.0);

    int N = (int)solution.time.size();
    //std::cout << "DEBUG solution.controls " << solution.controls << std::endl;
    // TODO is this really the correct solution?
    RowVectorXd expected = RowVectorXd::LinSpaced(N - 2, 14.6119, -14.6119);
    TROPTER_REQUIRE_EIGEN(solution.controls.middleCols(1, N - 2), expected,
     0.1);
    //RowVectorXd errors = solution.controls.rightCols(N - 1) - expected;
    //REQUIRE(Approx(errors.norm()) == 0);
}

TEST_CASE("SNOPT, hermite-simpson") {

    auto ocp = std::make_shared<SlidingMass<double>>();
    DirectCollocationSolver<double> dircol(ocp, "hermite-simpson", "snopt");
    Solution solution = dircol.solve();
    solution.write("sliding_mass_solution.csv");
    //Iterate initial_guess = ocp->make_guess_template();
    //Solution solution = dircol.solve(initial_guess);

    // Initial and final position.
    REQUIRE(Approx(solution.states(0, 0)) == 0.0);
    REQUIRE(Approx(solution.states.rightCols<1>()[0]) == 1.0);
    // Initial and final speed.
    REQUIRE(Approx(solution.states(1, 0)) == 0.0);
    REQUIRE(Approx(solution.states.rightCols<1>()[1]) == 0.0);

    int N = (int)solution.time.size();
    //std::cout << "DEBUG solution.controls " << solution.controls << std::endl;
    // TODO is this really the correct solution?
    RowVectorXd expected = RowVectorXd::LinSpaced(N - 2, 14.6119, -14.6119);
    TROPTER_REQUIRE_EIGEN(solution.controls.middleCols(1, N - 2), expected,
        0.1);
    //RowVectorXd errors = solution.controls.rightCols(N - 1) - expected;
    //REQUIRE(Approx(errors.norm()) == 0);
}
#endif

