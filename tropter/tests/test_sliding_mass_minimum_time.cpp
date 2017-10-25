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
#include "testing.h"

using Eigen::Ref;
using Eigen::VectorXd;
using Eigen::RowVectorXd;
using Eigen::MatrixXd;

using namespace tropter;

template<typename T>
class SlidingMassMinimumTime : public tropter::OptimalControlProblem<T> {
public:
    const double mass = 10.0;
    const double Fmax = 10;
    SlidingMassMinimumTime()
    {
        // TODO when time is a variable, this has to be more advanced:
        this->set_time({0}, {0, 10});
        this->add_state("x", {0, 1}, {0}, {1});
        this->add_state("u", {-100, 100}, {0}, {0});
        this->add_control("F", {-Fmax, Fmax});
    }
    void calc_differential_algebraic_equations(
            const DAEInput<T>& in, DAEOutput<T> out) const override {
        out.dynamics[0] = in.states[1];
        out.dynamics[1] = in.controls[0]/mass;
    }
    // TODO alternate form that takes a matrix; state at every time.
    //virtual void continuous(const MatrixXd& x, MatrixXd& xdot) const = 0;
    void calc_endpoint_cost(const T& final_time, const VectorX<T>&, T& cost)
            const override {
        cost = final_time;
    }
    OptimalControlSolution actual_solution(const VectorXd& time) const {
        OptimalControlSolution sol;
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
    static void run_test() {
        auto ocp = std::make_shared<SlidingMassMinimumTime<T>>();
        const int halfN = 25;
        const int N = 2 * halfN;
        DirectCollocationSolver<T> dircol(ocp, "trapezoidal", "ipopt", N);
        OptimalControlSolution solution = dircol.solve();
        solution.write("sliding_mass_minimum_time_solution.csv");

        OptimalControlSolution expected = ocp->actual_solution(solution.time);

        TROPTER_REQUIRE_EIGEN(solution.states, expected.states, 0.001);
        TROPTER_REQUIRE_EIGEN(solution.controls, expected.controls, 0.001);
    }
};

TEST_CASE("Sliding mass minimum time.") {
    SlidingMassMinimumTime<adouble>::run_test();
    SlidingMassMinimumTime<double>::run_test();
}

