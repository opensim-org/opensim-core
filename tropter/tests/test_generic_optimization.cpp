// ----------------------------------------------------------------------------
// tropter: test_generic_optimization.cpp
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

using Eigen::Vector4d;
using Eigen::Vector2d;
using Eigen::VectorXd;

using tropter::VectorX;
using namespace tropter::optimization;

// TODO elegantly handle the case where the objective is not defined (do not
// need ADOL-C to do a bunch of stuff.

// TODO test exceptions for upper bound < lower bound.

// TODO allow copying the problem...

template<typename T>
class Unconstrained : public Problem<T> {
public:
    Unconstrained() : Problem<T>(2, 0) {
        this->set_variable_bounds(Vector2d(-5, -5), Vector2d(5, 5));
    }
    void calc_objective(
            const VectorX<T>& x, T& obj_value) const override {
        obj_value = (x[0] - 1.5) * (x[0] - 1.5)
                + (x[1] + 2.0) * (x[1] + 2.0);
    }
};

TEST_CASE("Unconstrained, IPOPTSolver", "[ipopt]") {
    // Make sure it's okay to not have constraints.
    SECTION("Finite differences, Ipopt Jacobian, limited memory Hessian") {
        Unconstrained<double> problem;
        // TODO may not want user to directly use IPOPTSolver; instead, use a
        // generic solver interface?
        IPOPTSolver solver(problem);
        VectorXd guess = Vector2d(0, 0);
        solver.set_jacobian_approximation("finite-difference-values");
        solver.set_hessian_approximation("limited-memory");
        auto solution = solver.optimize(guess);

        REQUIRE(Approx(solution.variables[0]).margin(1e-10) == 1.5);
        REQUIRE(Approx(solution.variables[1]).margin(1e-10) == -2.0);
        REQUIRE(Approx(solution.objective).margin(1e-10)    == 0);

        // TODO throw exception if constraints() is unimplemented and
        // there are nonzero number of constraints.
    }
    SECTION("Finite differences, tropter Jacobian, limited memory Hessian") {
        Unconstrained<double> problem;
        // TODO may not want user to directly use IPOPTSolver; instead, use a
        // generic solver interface?
        IPOPTSolver solver(problem);
        VectorXd guess = Vector2d(0, 0);
        solver.set_jacobian_approximation("exact");
        solver.set_hessian_approximation("limited-memory");
        auto solution = solver.optimize(guess);

        REQUIRE(Approx(solution.variables[0]).margin(1e-10) == 1.5);
        REQUIRE(Approx(solution.variables[1]).margin(1e-10) == -2.0);
        REQUIRE(Approx(solution.objective).margin(1e-10) == 0);

        // TODO throw exception if constraints() is unimplemented and
        // there are nonzero number of constraints.
    }
    SECTION("Finite differences, tropter Jacobian and Hessian") {
        Unconstrained<double> problem;
        IPOPTSolver solver(problem);
        VectorXd guess = Vector2d(0, 0);
        solver.set_jacobian_approximation("exact");
        solver.set_hessian_approximation("exact");
        auto solution = solver.optimize(guess);

        REQUIRE(Approx(solution.variables[0]).margin(1e-10) == 1.5);
        REQUIRE(Approx(solution.variables[1]).margin(1e-10) == -2.0);
        REQUIRE(Approx(solution.objective).margin(1e-10)    == 0);
    }
    SECTION("ADOL-C") {
        // Make sure it's okay to not have constraints.
        Unconstrained<adouble> problem;
        IPOPTSolver solver(problem);
        VectorXd guess = Vector2d(0, 0);
        auto solution = solver.optimize(guess);

        REQUIRE(Approx(solution.variables[0]).margin(1e-10) == 1.5);
        REQUIRE(Approx(solution.variables[1]).margin(1e-10) == -2.0);
        REQUIRE(Approx(solution.objective).margin(1e-10)    == 0);

        // TODO throw exception if constraints() is unimplemented and
        // there are nonzero number of constraints.
    }
}

/// This problem comes from
/// https://www.coin-or.org/Ipopt/documentation/node23.html
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
    void calc_constraints(const VectorX<T>& x,
            Eigen::Ref<VectorX<T>> constr) const override {
        constr[0] = x.prod();
        constr[1] = x.squaredNorm();
    }
};

TEST_CASE("IPOPT C++ tutorial problem HS071; has constraints.") {
    SECTION("Finite differences, Ipopt Jacobian, limited memory Hessian") {
        HS071<double> problem;
        IPOPTSolver solver(problem);
        solver.set_jacobian_approximation("finite-difference-values");
        solver.set_hessian_approximation("limited-memory");
        VectorXd guess = Vector4d(1.5, 2.5, 3.5, 4.5);
        auto solution = solver.optimize(guess);

        REQUIRE(       solution.variables[0]  == 1.0);
        REQUIRE(Approx(solution.variables[1]) == 4.743);
        REQUIRE(Approx(solution.variables[2]) == 3.82115);
        REQUIRE(Approx(solution.variables[3]) == 1.379408);

        REQUIRE(Approx(solution.objective) == 17.014);
    }
    SECTION("Finite differences, tropter Jacobian, limited memory Hessian") {
        HS071<double> problem;
        IPOPTSolver solver(problem);
        solver.set_jacobian_approximation("exact");
        solver.set_hessian_approximation("limited-memory");
        VectorXd guess = Vector4d(1.5, 2.5, 3.5, 4.5);
        auto solution = solver.optimize(guess);

        REQUIRE(solution.variables[0] == 1.0);
        REQUIRE(Approx(solution.variables[1]) == 4.743);
        REQUIRE(Approx(solution.variables[2]) == 3.82115);
        REQUIRE(Approx(solution.variables[3]) == 1.379408);

        REQUIRE(Approx(solution.objective) == 17.014);
    }
    SECTION("Finite differences, tropter Jacobian and Hessian") {
        HS071<double> problem;
        IPOPTSolver solver(problem);
        solver.set_jacobian_approximation("exact");
        solver.set_hessian_approximation("exact");
        VectorXd guess = Vector4d(1.5, 2.5, 3.5, 4.5);
        auto solution = solver.optimize(guess);

        REQUIRE(       solution.variables[0]  == 1.0);
        REQUIRE(Approx(solution.variables[1]) == 4.743);
        REQUIRE(Approx(solution.variables[2]) == 3.82115);
        REQUIRE(Approx(solution.variables[3]) == 1.379408);

        REQUIRE(Approx(solution.objective) == 17.014);
    }
    SECTION("ADOL-C") {
        HS071<adouble> problem;
        IPOPTSolver solver(problem);
        VectorXd guess = Vector4d(1.5, 2.5, 3.5, 4.5);
        auto solution = solver.optimize(guess);

        REQUIRE(       solution.variables[0]  == 1.0);
        REQUIRE(Approx(solution.variables[1]) == 4.743);
        REQUIRE(Approx(solution.variables[2]) == 3.82115);
        REQUIRE(Approx(solution.variables[3]) == 1.379408);

        REQUIRE(Approx(solution.objective) == 17.014);
    }
}

/// This problem has all 4 possible pairs of parameter bounds, and is
/// used to ensure that
/// OptimizationProblemProxy::initial_guess_from_bounds() computes
/// an initial guess as desired for each of these 4 cases.
/// The 4 possible cases:
/// @verbatim
/// (-inf, inf) -> guess is 0.
/// [a, b]      -> guess is (b+a)/2.
/// (-inf, b]   -> guess is b.
/// [a, inf)    -> guess is a.
/// @endverbatim
template<typename T>
class VarietyOfBounds : public Problem<T> {
public:
    const double inf = std::numeric_limits<double>::infinity();
    VarietyOfBounds() : Problem<T>(4, 0) {
        this->set_variable_bounds(Vector4d(-inf, -20, -inf,  50),
                                  Vector4d( inf,  10,   -8, inf));
    }
    void calc_objective(const VectorX<T>& x, T& obj_value) const override {
        obj_value = x.squaredNorm();
    }
};

TEST_CASE("Generating an initial guess using problem bounds",
          "[initialguess]") {
    SECTION("double") {
        VarietyOfBounds<double> problem;
        const auto decorator = problem.make_decorator();
        VectorXd actual = decorator->make_initial_guess_from_bounds();
        Vector4d expected(0, -5, -8, 50);
        TROPTER_REQUIRE_EIGEN(actual, expected, 1e-10);
    }
    SECTION("adouble") {
        VarietyOfBounds<adouble> problem;
        const auto proxy = problem.make_decorator();
        VectorXd actual = proxy->make_initial_guess_from_bounds();
        Vector4d expected(0, -5, -8, 50);
        TROPTER_REQUIRE_EIGEN(actual, expected, 1e-10);
    }
}

TEST_CASE("Test exceptions and error messages") {
    SECTION("Solver max_iterations") {
        class Prob : public Problem<double> {
        public:
            Prob() : Problem<double>(2, 0)
            {   set_variable_bounds(Vector2d(-5, -5), Vector2d(5, 5)); }
            void calc_objective(const VectorXd& x, double& f) const override
            {   f = x.squaredNorm(); }
        };

        Prob problem;
        IPOPTSolver solver(problem);
        REQUIRE_THROWS_WITH(solver.set_max_iterations(0),
                Catch::Contains("Invalid value for max_iterations"));
    }
}

TEST_CASE("OptimizerSolver options") {
    HS071<double> problem;
    IPOPTSolver solver(problem);
    const Solution sol_default = solver.optimize();

    {
        solver.set_convergence_tolerance(1e-2);
        const auto solution = solver.optimize();
        REQUIRE(solution.num_iterations < sol_default.num_iterations);
        // Unset.
        solver.set_convergence_tolerance({});
    }
    {
        solver.set_constraint_tolerance(1e-12);
        const auto solution = solver.optimize();
        REQUIRE(solution.num_iterations > sol_default.num_iterations);
        solver.set_constraint_tolerance({});
    }

}


