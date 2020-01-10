// ----------------------------------------------------------------------------
// tropter: test_analytic_optimal_control_solutions.cpp
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
#include "testing.h"

#include <tropter/tropter.h>

using Eigen::Ref;
using Eigen::VectorXd;
using Eigen::RowVectorXd;
using Eigen::Vector2d;
using Eigen::MatrixXd;
using Eigen::Matrix2d;

using namespace tropter;

/// Kirk 1998, Example 5.1-1, page 198.
template<typename T>
class SecondOrderLinearMinEffort : public tropter::Problem<T> {
public:
    SecondOrderLinearMinEffort() {
        this->set_time(0, 2);
        this->add_state("x0", {-10, 10}, {0}, {5});
        this->add_state("x1", {-10, 10}, {0}, {2});
        this->add_control("u", {-50, 50});
        this->add_cost("effort", 1);
    }
    void calc_differential_algebraic_equations(
            const Input<T>& in, Output<T> out) const override {
        const auto& x = in.states;
        const auto& u = in.controls;
        out.dynamics[0] = x[1];
        out.dynamics[1] = x[1] + u[0];
        // xdot.row(0) = x.row(1);
        // xdot.row(1) = -x.row(1) + u.row(0);
    }
    void calc_cost(
            int, const CostInput<T>& in, T& cost) const override {
        cost = in.integral;
    }
    void calc_cost_integrand(
            int, const Input<T>& in, T& integrand) const override {
        const auto& u = in.controls;
        integrand = 0.5 * u[0] * u[0];
    }
    MatrixXd states_solution(const VectorXd& time) const {
        using std::exp;

        MatrixXd result(2, time.size());

        // Taken from Kirk 1998 equations 5.1-69 and 5.1-70, page 199.
        double c2;
        double c3;
        {
            Vector2d b(5, 2);
            Matrix2d A;
            A << -2 - 0.5*exp(-2) + 0.5*exp(2), 1 - 0.5*exp(-2) - 0.5*exp(2),
                 -1 + 0.5*exp(-2) + 0.5*exp(2),     0.5*exp(-2) - 0.5*exp(2);
            Vector2d c = A.lu().solve(b);
            c2 = c[0];
            c3 = c[1];
        }

        auto x0_func = [&c2, &c3](const double& t) -> double {
            return c2*(-t - 0.5*exp(-t) + 0.5*exp(t))
                 + c3*( 1 - 0.5*exp(-t) - 0.5*exp(t));
        };
        result.row(0) = time.unaryExpr(x0_func);

        auto x1_func = [&c2, &c3](const double& t) -> double {
            return c2*(-1 + 0.5*exp(-t) + 0.5*exp(t))
                 + c3*(     0.5*exp(-t) - 0.5*exp(t));
        };
        result.row(1) = time.unaryExpr(x1_func);

        return result;
    }
    static void run_test(int N, std::string solver,
            std::string hessian_approx, std::string transcription) {
        auto ocp = std::make_shared<SecondOrderLinearMinEffort<T>>();
        DirectCollocationSolver<T> dircol(ocp, transcription, solver, N);
        std::string jacobian_approx;
        if (hessian_approx == "exact") {
            jacobian_approx = hessian_approx;
        } else {
            jacobian_approx = "finite-difference-values";
        }
        dircol.get_opt_solver().set_jacobian_approximation
            (jacobian_approx);
        dircol.get_opt_solver().set_hessian_approximation
            (hessian_approx);
        Solution solution = dircol.solve();
        //solution.write("second_order_linear_min_effort_solution.csv");

        MatrixXd expected_states = ocp->states_solution(solution.time);
        TROPTER_REQUIRE_EIGEN(solution.states, expected_states, 0.005);
    }
};

TEST_CASE("Second order linear min effort", 
          "[adolc][trapezoidal][hermite-simpson]") 
{
    SECTION("ADOL-C") {
        SecondOrderLinearMinEffort<adouble>::run_test(1000, "ipopt", "exact",
            "trapezoidal");
    }
    //SECTION("Finite differences") {
    //    SecondOrderLinearMinEffort<double>::run_test(20, "ipopt", "exact"
    //        "trapezoidal");
    //}
    SECTION("ADOL-C") {
        SecondOrderLinearMinEffort<adouble>::run_test(500, "ipopt", "exact",
            "hermite-simpson");
    }
    //SECTION("Finite differences") {
    //    SecondOrderLinearMinEffort<double>::run_test(10, "ipopt", "exact"
    //        "hermite-simpson");
    //}
}

// TODO add linear tangent steering (Bryson 1975). Also in Betts' book.
