// ----------------------------------------------------------------------------
// tropter: test_parameter_optimization.cpp
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
#include "testing_optimalcontrol.h"

using namespace tropter;

/// Pose the generic unconstrained optimization defined in
/// test_generic_optimization.cpp as an optimal control problem, now using
/// parameters instead of directly defining optimization variables.
template<typename T>
class Unconstrained : public tropter::OptimalControlProblem<T> {
public:
    Unconstrained() {
        this->set_time({0}, {1});
        this->add_parameter("x0", {-5, 5});
        this->add_parameter("x1", {-5, 5});
    }

    void calc_integral_cost(const T& /*time*/,
            const VectorX<T>& /*states*/,
            const VectorX<T>& /*controls*/,
            const VectorX<T>& parameters,
            T& integrand) const override {
        integrand = (parameters[0] - 1.5) * (parameters[0] - 1.5)
            + (parameters[1] + 2.0) * (parameters[1] + 2.0);
    }

};

TEST_CASE("IPOPT") {
    SECTION("Finite differences, limited memory") {
        auto ocp = std::make_shared<Unconstrained<double>>();
        DirectCollocationSolver<double> dircol(ocp, "trapezoidal", "ipopt");
        dircol.get_opt_solver().set_findiff_hessian_step_size(1e-3);
        dircol.get_opt_solver().set_hessian_approximation("limited-memory");
        OptimalControlSolution solution = dircol.solve();

        // Check correct optimization solution is obtained.
        REQUIRE(Approx(solution.parameters[0]) == 1.5);
        REQUIRE(Approx(solution.parameters[1]) == -2.0);
        REQUIRE(Approx(solution.objective) == 0);
    }
    SECTION("Finite differences, exact Hessian") {
        auto ocp = std::make_shared<Unconstrained<double>>();
        DirectCollocationSolver<double> dircol(ocp, "trapezoidal", "ipopt");
        dircol.get_opt_solver().set_findiff_hessian_step_size(1e-3);
        dircol.get_opt_solver().set_hessian_approximation("exact");
        OptimalControlSolution solution = dircol.solve();

        // Check correct optimization solution is obtained.
        REQUIRE(Approx(solution.parameters[0]) == 1.5);
        REQUIRE(Approx(solution.parameters[1]) == -2.0);
        REQUIRE(Approx(solution.objective) == 0);
    }
    SECTION("ADOL-C") {
        // Solve the optimal control problem.
        auto ocp = std::make_shared<Unconstrained<adouble>>();
        DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal", "ipopt");
        OptimalControlSolution solution = dircol.solve();
        dircol.print_constraint_values(solution);
        solution.write("unconstrained_solution.csv");

        // Check correct optimization solution is obtained.
        REQUIRE(Approx(solution.parameters[0]) == 1.5);
        REQUIRE(Approx(solution.parameters[1]) == -2.0);
        REQUIRE(Approx(solution.objective) == 0);
    }
}