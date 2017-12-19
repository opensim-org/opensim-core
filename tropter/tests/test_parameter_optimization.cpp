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

using Eigen::VectorXd;
using Eigen::Vector2d;

using namespace tropter;

/// Generic unconstrained optimization problem (borrowed from 
/// test_generic_optimization.cpp).
template<typename T>
class UnconstrainedNLP : public OptimizationProblem<T> {
public:
    Unconstrained() : OptimizationProblem<T>(2, 0) {
        this->set_variable_bounds(Vector2d(-5, -5), Vector2d(5, 5));
    }
    void calc_objective(
        const VectorX<T>& x, T& obj_value) const override {
        obj_value = (x[0] - 1.5) * (x[0] - 1.5)
            + (x[1] + 2.0) * (x[1] + 2.0);
    }
};

/// Pose the generic unconstrained optimization defined in
/// test_generic_optimization.cpp as an optimal control problem, now using
/// parameters instead of directly defining optimization variables.
template<typename T>
class UnconstrainedOCP : public tropter::OptimalControlProblem<T> {
public:
    UnconstrainedOCP() {
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

    SECTION("ADOL-C") {
        // Solve optimization problem.
        UnconstrainedNLP<adouble> problem;
        IPOPTSolver solver(problem);
        VectorXd guess = Vector2d(0, 0);
        auto nlp_solution = solver.optimize(guess);

        // Solve the optimal control problem.
        auto ocp = std::make_shared<UnconstrainedOCP<adouble>>();
        DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal", "ipopt");
        OptimalControlSolution ocp_solution = dircol.solve();
        ocp_solution.write("unconstrained_solution.csv");

        // Check correct optimization solution is obtained.
        REQUIRE(Approx(nlp_solution.parameters[0]) == 1.5);
        REQUIRE(Approx(nlp_solution.parameters[1]) == -2.0);
        REQUIRE(Approx(nlp_solution.objective) == 0);

        // Compare optimal control solution to optimization solution.
        REQUIRE(Approx(ocp_solution.parameters[0]) == 
                Approx(nlp_solution.parameters[0]));
        REQUIRE(Approx(ocp_solution.parameters[1]) == 
                Approx(nlp_solution.parameters[1]));
        REQUIRE(Approx(ocp_solution.objective) == 
                Approx(nlp_solution.objective));
    }

}