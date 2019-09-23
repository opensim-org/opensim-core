// ----------------------------------------------------------------------------
// tropter: test_snopt.cpp
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

#include <snoptProblem.hpp>

using namespace tropter;
using Eigen::VectorXd;
using Eigen::RowVectorXd;
using Eigen::Vector2d;
using Eigen::MatrixXd;
using Eigen::Ref;

// This example is taken from
// https://github.com/snopt/snopt-interface/blob/master/cppexamples/sntoya.cpp
void toyusrf_(int*   /* Status */, int* /* n     */, double x[],
              int*   /* needF  */, int* /* neF   */, double F[],
              int*   /* needG  */, int* /* neG   */, double[] /* G */,
              char*  /*    cu  */, int* /* lencu */,
              int   [] /* iu   */, int* /* leniu */,
              double[] /* ru   */, int* /* lenru */)
{
    //==================================================================
    // Computes the nonlinear objective and constraint terms for the toy
    // problem featured in the SnoptA users guide.
    // neF = 3, n = 2.
    //
    //   Minimize     x(2)
    //
    //   subject to   x(1)**2      + 4 x(2)**2  <= 4,
    //               (x(1) - 2)**2 +   x(2)**2  <= 5,
    //                x(1) >= 0.
    //
    //==================================================================
    F[0] = x[1];
    F[1] = x[0]*x[0] + 4*x[1]*x[1];
    F[2] = (x[0] - 2)*(x[0] - 2) + x[1]*x[1];
}

// TODO also test the variant with an exact Jacobian.
TEST_CASE("SNOPT sntoyA example problem; Jacobian not provided.")
{

    snoptProblemA ToyProb;

    // Allocate and initialize;
    int n = 2;
    int neF = 3;
    double* x = new double[n];
    double* xlow = new double[n];
    double* xupp = new double[n];
    double* xmul = new double[n];
    int* xstate = new int[n];
    double* F = new double[neF];
    double* Flow = new double[neF];
    double* Fupp = new double[neF];
    double* Fmul = new double[neF];
    int* Fstate = new int[neF];
    int ObjRow = 0;
    double ObjAdd = 0;
    int Cold = 0, Basis = 1, Warm = 2;
    int nS;
    int nInf;
    double sInf;


    // Set the upper and lower bounds.
    xlow[0] = 0.0;
    xlow[1] = -1e20;
    xupp[0] = 1e20;
    xupp[1] = 1e20;
    xstate[0] = 0;
    xstate[1] = 0;

    Flow[0] = -1e20;
    Flow[1] = -1e20;
    Flow[2] = -1e20;
    Fupp[0] = 1e20;
    Fupp[1] = 4.0;
    Fupp[2] = 5.0;
    x[0] = 1.0;
    x[1] = 1.0;

    // Load the data for ToyProb ...
    ToyProb.initialize("Toy0.out", 1);
    ToyProb.setProbName("Toy0");

    // snopta will compute the Jacobian by finite-differences.
    // The user has the option of calling  snJac  to define the
    // coordinate arrays (iAfun,jAvar,A) and (iGfun, jGvar).
    ToyProb.setIntParameter("Derivative option", 0);
    ToyProb.setIntParameter("Verify level ", 3);

    // Solve the problem.
    // snJac is called implicitly in this case to compute the Jacobian.
    ToyProb.solve(Cold, neF, n, ObjAdd, ObjRow, toyusrf_,
                   xlow, xupp, Flow, Fupp, 
                   x, xstate, xmul, 
                   F, Fstate, Fmul,
                   nS, nInf, sInf);

    for (int i = 0; i < n; i++) {
        std::cout << "x = " << x[i] << " xstate = " << xstate[i] << std::endl;
    }
    for (int i = 0; i < neF; i++) {
        std::cout << "F = " << F[i] << " Fstate = " << Fstate[i] << std::endl;
    }
}
//==================================================================
// Computes the nonlinear objective and constraint terms for the toy
// problem featured in the SnoptA users guide.
// neF = 3, n = 2.
//
//   Minimize     x(2)
//
//   subject to   x(1)**2      + 4 x(2)**2  <= 4,
//               (x(1) - 2)**2 +   x(2)**2  <= 5,
//                x(1) >= 0.
//
//==================================================================

//
TEST_CASE("SNOPT and ADOL-C on SnoptA (sntoyA) example")
{
    class SnoptA : public optimization::Problem<double> {
    public:
        SnoptA() : Problem(2, 2) {
            // TODO support an "infinity"
            set_variable_bounds(Vector2d(0, -1e20), Vector2d(1e20, 1e20));
            set_constraint_bounds(Vector2d(-1e20, -1e20), Vector2d(4, 5));
        }
        void calc_objective(const VectorXd& x, double& obj_value) const
                override {
            obj_value = x[1];
        }
        void calc_constraints(const VectorXd& x,
                         Eigen::Ref<VectorXd> constr) const override {
            constr[0] = x[0]*x[0] + 4*x[1]*x[1];
            constr[1] = (x[0] - 2)*(x[0] - 2) + x[1]*x[1];
        }
    };
    SnoptA problem;
    optimization::SNOPTSolver solver(problem);
    VectorXd variables = Vector2d(2, 2);
    auto solution = solver.optimize(variables);
    REQUIRE(Approx(solution.variables[0]).margin(1e-10) == 0);
    REQUIRE(Approx(solution.variables[1]) == -1);
    REQUIRE(Approx(solution.objective)    == -1);
}
TEST_CASE("First order minimum effort.", "[analytic]")
{

    /// minimize     integral_{t=0}^{t=1) u(t)^2 dt
    /// subject to   xdot = -2x + u
    ///              x(0) = 1
    ///              x(1) = 0
    class FirstOrderMinEffort
            : public tropter::Problem<double> {
    public:
        using T = double;
        FirstOrderMinEffort() {
            set_time(0, 1);
            add_state("x", {-5, 5}, 1, 0);
            add_control("u", {-10, 10});
        }
        void calc_differential_algebraic_equations(
                const Input<T>& in, Output<T> out) const override {
            out.dynamics[0] = -2*in.states[0] + in.controls[0];
        }
        void calc_cost(int cost_index, const CostInput<T>& in,
                T& cost) const override {
            cost = in.integral;
        }
        void calc_cost_integrand(int cost_index, const Input<T>& in,
                T& integrand) const override {
            integrand = in.controls[0] * in.controls[0];
        }
    };
    auto ocp = std::make_shared<FirstOrderMinEffort>();
    DirectCollocationSolver<double> dircol(ocp, "trapezoidal", "snopt", 50);
    dircol.get_opt_solver().set_jacobian_approximation(
            "finite-difference-values");
    Solution solution = dircol.solve();
    solution.write("first_order_minimum_effort_snopt_solution.csv");


    // Initial and final states satisfy constraints.
    REQUIRE(Approx(solution.states(0, 0)) == 1.0);
    REQUIRE(Approx(solution.states.rightCols<1>()[0]) == 0.0);

    // u*(t) = -4 exp(2t) / (e^4 - 1)
    const auto N = solution.time.size();
    const double factor = -4/(std::exp(4) - 1);
    RowVectorXd expected = factor*(2*solution.time).array().exp();
    // TODO the initial and final controls are wrong.
    for (int i = 1; i < (N - 1); ++i) {
        const auto rel_error = std::abs(solution.controls(i)/expected(i) - 1);
        REQUIRE(rel_error < 0.005);
    }
}

template<typename T>
class SlidingMassMinimumEffort : public tropter::Problem<T> {
public:
    SlidingMassMinimumEffort()
    {
        this->set_time(0, 2);
        this->add_state("x", {0, 2}, {0}, {1});
        this->add_state("u", {-10, 10}, {0}, {0});
        this->add_control("F", {-50, 50});
    }
    const double mass = 10.0;
    void calc_differential_algebraic_equations(
            const Input<T>& in, Output<T> out) const override {
        out.dynamics[0] = in.states[1];
        out.dynamics[1] = in.controls[0]/mass;
    }
    void calc_cost(
            int cost_index, const CostInput<T>& in, T& cost) const override {
        cost = in.integral;
    }
    void calc_cost_integrand(
            int cost_index, const Input<T>& in, T& integrand) const override {
        integrand = in.controls[0] * in.controls[0];
    }
};

TEST_CASE("Sliding mass minimum effort with SNOPT.")
{

    auto ocp = std::make_shared<SlidingMassMinimumEffort<double>>();
    DirectCollocationSolver<double> dircol(ocp, "trapezoidal", "snopt", 30);
    dircol.get_opt_solver().set_jacobian_approximation(
            "finite-difference-values");
    Solution solution = dircol.solve();
    solution.write("sliding_mass_minimum_effort_snopt_solution.csv");

    // Initial and final position.
    REQUIRE(Approx(solution.states(0, 0)) == 0.0);
    REQUIRE(Approx(solution.states.rightCols<1>()[0]) == 1.0);
    // Initial and final speed.
    REQUIRE(Approx(solution.states(1, 0)) == 0.0);
    REQUIRE(Approx(solution.states.rightCols<1>()[1]) == 0.0);

    int N = (int)solution.time.size();
    std::cout << "DEBUG solution.controls " << solution.controls << std::endl;
    // TODO is this correct?
    RowVectorXd expected = RowVectorXd::LinSpaced(N - 2, 14.7448, -14.7448);
    TROPTER_REQUIRE_EIGEN(solution.controls.middleCols(1, N - 2), expected,
            0.1);
}

