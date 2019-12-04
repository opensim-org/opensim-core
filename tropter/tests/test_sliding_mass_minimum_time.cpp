// ----------------------------------------------------------------------------
// tropter: test_sliding_mass_minimum_time.cpp
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
#include <cmath>

using Eigen::Ref;
using Eigen::VectorXd;
using Eigen::RowVectorXd;
using Eigen::MatrixXd;

using namespace tropter;

template<typename T>
class SlidingMassMinimumTime : public tropter::Problem<T> {
public:
    const double mass = 10.0;
    const double Fmax = 10;
    SlidingMassMinimumTime()
    {
        this->set_time({0}, {0, 10});
        this->add_state("x", {0, 1}, {0}, {1});
        this->add_state("u", {-100, 100}, {0}, {0});
        this->add_control("F", {-Fmax, Fmax});
        this->add_cost("final_time", 0);
    }
    void calc_differential_algebraic_equations(
            const Input<T>& in, Output<T> out) const override {
        out.dynamics[0] = in.states[1];
        out.dynamics[1] = in.controls[0]/mass;
    }
    // TODO alternate form that takes a matrix; state at every time.
    //virtual void continuous(const MatrixXd& x, MatrixXd& xdot) const = 0;
    void calc_cost(
            int, const CostInput<T>& in, T& cost) const override {
        cost = in.final_time;
    }
    Solution actual_solution(const VectorXd& time) const {
        Solution sol;
        sol.time = time;
        sol.states.resize(2, time.size());
        sol.controls.resize(1, time.size());

        auto x = [&](const double& t) -> double {
            if (t < 0.5 * time[time.size()-1]) { return 0.5*t*t; }
            else { return -0.5*(t - 1)*(t - 1) + 1*(t - 1) + 0.5; }
        };
        sol.states.row(0) = sol.time.unaryExpr(x);

        auto u = [&](const double& t) -> double {
            if (t < 0.5 * time[time.size()-1]) { return t; }
            else { return 2 - t; }
        };
        sol.states.row(1) = sol.time.unaryExpr(u);

        auto F = [&](const double& t) -> double {
            if (t < 0.5 * time[time.size()-1]) { return Fmax; }
            else { return -Fmax; }
        };
        sol.controls = sol.time.unaryExpr(F);

        return sol;
    }
    static void run_test(int N, std::string transcription, double eps) {
        auto ocp = std::make_shared<SlidingMassMinimumTime<T>>();
        DirectCollocationSolver<T> dircol(ocp, transcription, "ipopt", N - 1);
        //dircol.get_opt_solver().set_advanced_option_string
        //        ("derivative_test", "second-order");
        dircol.get_opt_solver().set_sparsity_detection("random");
        dircol.get_opt_solver().set_findiff_hessian_step_size(1e-3);
        Solution solution = dircol.solve();
        solution.write("sliding_mass_minimum_time_" + transcription +
             "_solution.csv");

        Solution expected = ocp->actual_solution(solution.time);
        
        //TROPTER_REQUIRE_EIGEN(solution.states, expected.states, 0.001);
        //TROPTER_REQUIRE_EIGEN(solution.controls, expected.controls, 0.001);
        REQUIRE((solution.states - expected.states).norm() / 
                 sqrt(solution.time.size()) < eps);
        REQUIRE((solution.controls - expected.controls).norm() / 
                 sqrt(solution.time.size()) < eps);
    }
};

TEST_CASE("Sliding mass minimum time.") {
    SECTION("derivative comparison") {
        OCPDerivativesComparison<SlidingMassMinimumTime> comp;
        comp.num_mesh_intervals = 3;
        comp.compare();
    }
    SECTION("trapezoidal") {
        SlidingMassMinimumTime<adouble>::run_test(100, "trapezoidal", 0.001);
        SlidingMassMinimumTime<double>::run_test(100, "trapezoidal", 0.001);
    }
    // TODO midpoint discontinuity causes the RMS error to be quite high for
    // hermite-simpson.
    SECTION("hermite-simpson") {
        SlidingMassMinimumTime<adouble>::run_test(100, "hermite-simpson", 1);
        SlidingMassMinimumTime<double>::run_test(100, "hermite-simpson", 1);
    }
}

