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

#define _USE_MATH_DEFINES
#include <cmath>

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

TEST_CASE("Unconstrained, IPOPT") {
    SECTION("Finite differences, limited memory") {
        auto ocp = std::make_shared<Unconstrained<double>>();
        DirectCollocationSolver<double> dircol(ocp, "trapezoidal", "ipopt");
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
        solution.write("unconstrained_solution.csv");

        // Check correct optimization solution is obtained.
        REQUIRE(Approx(solution.parameters[0]) == 1.5);
        REQUIRE(Approx(solution.parameters[1]) == -2.0);
        REQUIRE(Approx(solution.objective) == 0);
    }
}

/// Use parameter optimization to find the constant parameter that causes a 1 kg 
/// mass to fall "g" meters in 1 second, with a final velocity of "g" meters per 
/// second, and check that the parameter is equal to "g" meters per second 
/// squared (g = 9.80665).
double GRAV_ACCEL = 9.80665;
template<typename T>
class GravitationalAcceleration : public tropter::OptimalControlProblem<T> {
public:
    GravitationalAcceleration() {
        this->set_time({0}, {1});
        this->add_state("x", {0, 20}, {0}, {GRAV_ACCEL-0.5, GRAV_ACCEL+0.5);
        this->add_state("u", {0, 20}, {0}, {GRAV_ACCEL});
        this->add_parameter("g", {0, 20});
    }

    void calc_differential_algebraic_equations(
        const DAEInput<T>& in, DAEOutput<T> out) const override {
        out.dynamics[0] = in.states[1];
        out.dynamics[1] = in.parameters[0];
    }

    void calc_endpoint_cost(const T& /*final_time*/,
        const VectorX<T>& final_states,
        const VectorX<T>& /*parameters*/,
        T& cost) {
        cost = (final_states[0] - GRAV_ACCEL) * (final_states[0] - GRAV_ACCEL);
    }

};

TEST_CASE("GravitationalAcceleration, IPOPT") {
    SECTION("Finite differences, limited memory") {
        auto ocp = std::make_shared<GravitationalAcceleration<double>>();
        DirectCollocationSolver<double> dircol(ocp, "trapezoidal", "ipopt");
        dircol.get_opt_solver().set_hessian_approximation("limited-memory");
        OptimalControlSolution solution = dircol.solve();

        REQUIRE(Approx(solution.parameters[0]) == GRAV_ACCEL);
        REQUIRE(Approx(solution.objective) == 0);
    }
    SECTION("Finite differences, exact Hessian") {
        auto ocp = std::make_shared<GravitationalAcceleration<double>>();
        DirectCollocationSolver<double> dircol(ocp, "trapezoidal", "ipopt");
        dircol.get_opt_solver().set_findiff_hessian_step_size(1e-3);
        dircol.get_opt_solver().set_hessian_approximation("exact");
        OptimalControlSolution solution = dircol.solve();

        REQUIRE(Approx(solution.parameters[0]) == GRAV_ACCEL);
        REQUIRE(Approx(solution.objective) == 0);
    }
    SECTION("ADOL-C") {
        auto ocp = std::make_shared<GravitationalAcceleration<adouble>>();
        DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal", "ipopt");
        OptimalControlSolution solution = dircol.solve();
        dircol.print_constraint_values(solution);

        REQUIRE(Approx(solution.parameters[0]) == GRAV_ACCEL);
        REQUIRE(Approx(solution.objective) == 0);
    }
}

/// Optimize the mass parameter for a single harmonic oscillator such that the
/// dynamics and boundary conditions match that of a 5 kg spring mass system,
/// and check that the predicted mass parameter equals 5 kg.
double MASS = 5; // kg
double STIFFNESS = 100; // N/m
double PI = 3.14159265358979323846;
double FINAL_TIME = PI * sqrt(MASS / STIFFNESS);
template<typename T>
class OscillatorMass : public tropter::OptimalControlProblem<T> {
public:
    OscillatorMass() {
        this->set_time({0}, {FINAL_TIME});
        this->add_state("x", {-5.0, 5.0}, {-0.5}, {0.25, 0.75});
        this->add_state("dx", {-20, 20}, {0}, {0});
        this->add_parameter("mass", {0, 10});
    }

    void calc_differential_algebraic_equations(
            const DAEInput<T>& in, DAEOutput<T> out) const override {
        out.dynamics[0] = in.states[1];
        out.dynamics[1] = -(STIFFNESS / in.parameters[0]) * in.states[0];
    }

    void calc_endpoint_cost(const T& /*final_time*/,
            const VectorX<T>& final_states,
            const VectorX<T>& /*parameters*/,
            T& cost) {
        
        cost = (final_states[0] - 0.5) * (final_states[0] - 0.5);

        // TODO: Final time cost approach not finding correct mass parameter.
        //double frequency = sqrt(STIFFNESS / MASS) / (2 * PI);
        //double period = final_time * 2;
        //cost = pow(frequency - (1 / period), 2);
    }

};

// Choose the fewest number of mesh points so the test passes.
// TODO: reduce number of mesh points needed (try different collocation scheme)
unsigned N = 350; 
TEST_CASE("OscillatorMass, IPOPT") {
    SECTION("Finite differences, limited memory") {
        auto ocp = std::make_shared<OscillatorMass<double>>();
        DirectCollocationSolver<double> dircol(ocp, "trapezoidal", "ipopt", N);
        dircol.get_opt_solver().set_hessian_approximation("limited-memory");
        OptimalControlSolution solution = dircol.solve();

        REQUIRE(Approx(solution.parameters[0]) == MASS);
        REQUIRE(Approx(solution.objective) == 0);
    }
    SECTION("Finite differences, exact Hessian") {
        auto ocp = std::make_shared<OscillatorMass<double>>();
        DirectCollocationSolver<double> dircol(ocp, "trapezoidal", "ipopt", N);
        dircol.get_opt_solver().set_findiff_hessian_step_size(1e-3);
        dircol.get_opt_solver().set_hessian_approximation("exact");
        OptimalControlSolution solution = dircol.solve();

        REQUIRE(Approx(solution.parameters[0]) == MASS);
        REQUIRE(Approx(solution.objective) == 0);
    }
    SECTION("ADOL-C") {
        auto ocp = std::make_shared<OscillatorMass<adouble>>();
        DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal", "ipopt", N);
        OptimalControlSolution solution = dircol.solve();
        dircol.print_constraint_values(solution);

        REQUIRE(Approx(solution.parameters[0]) == MASS);
        REQUIRE(Approx(solution.objective) == 0);
    }
}