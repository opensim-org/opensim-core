// ----------------------------------------------------------------------------
// tropter: test_double_pendulum.cpp
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
#include "testing_optimalcontrol.h"

#include <tropter/tropter.h>
#include <Eigen/LU>

using Eigen::Ref;
using Eigen::VectorXd;
using Eigen::RowVectorXd;
using Eigen::Vector2d;
using Eigen::MatrixXd;
using Eigen::Matrix2d;

using namespace tropter;

constexpr double PI = 3.14159;

/// This class template defines the dynamics of a double pendulum. To create
/// an actual optimal control problem, one must derive from this class
/// template and define boundary conditions, cost terms, etc.
template<typename T>
class DoublePendulum : public tropter::Problem<T> {
public:
    constexpr static const double g = 9.81;
    double L0 = 1;
    double L1 = 1;
    double m0 = 1;
    double m1 = 1;

    void calc_differential_algebraic_equations(
            const Input<T>& in, Output<T> out) const override final {
        const auto& x = in.states;
        const auto& tau = in.controls;
        const auto& q0 = x[0];
        const auto& q1 = x[1];
        const auto& u0 = x[2];
        const auto& u1 = x[3];
        const auto& L0 = this->L0;
        const auto& L1 = this->L1;
        const auto& m0 = this->m0;
        const auto& m1 = this->m1;
        out.dynamics[0] = u0;
        out.dynamics[1] = u1;

        const T z0 = m1 * L0 * L1 * cos(q1);
        const T M01 = m1 * L1*L1 + z0;
        Matrix2<T> M;
        M << m0 * L0*L0 + m1 * (L0*L0 + L1*L1) + 2*z0,    M01,
             M01,                                         m1 * L1*L1;
        Vector2<T> V(-u1 * (2 * u0 + u1),
                     u0 * u0);
        V *= m1*L0*L1*sin(q1);
        Vector2<T> G(g * ((m0 + m1) * L0 * cos(q0) + m1 * L1 * cos(q0 + q1)),
                     g * m1 * L1 * cos(q0 + q1));

        //Vector2<T> drag(-10 * u0, -10 * u1);
        // TODO xdot.tail<2>() =
        out.dynamics.tail(2) = M.inverse() * (tau - (V + G));
    }
};

/// The optimal solution for a double pendulum to swing from horizontal to
/// vertical (up) in minimum time is for the links to fall down first and
/// for the first link to rotate clockwise. The second link flips around +360
/// degrees to reach the Cartesian coordinates (0, 2).
template<typename T>
class DoublePendulumSwingUpMinTime : public DoublePendulum<T> {
public:
    DoublePendulumSwingUpMinTime()
    {
        this->set_time(0, {0, 5}); //{0, 5});
        // TODO fix allowing final bounds to be unconstrained.
        this->add_state("q0", {-10, 10}, {0});
        this->add_state("q1", {-10, 10}, {0});
        this->add_state("u0", {-50, 50}, {0}, {0});
        this->add_state("u1", {-50, 50}, {0}, {0});
        this->add_control("tau0", {-50, 50});
        this->add_control("tau1", {-50, 50});
        this->add_cost("target", 0);
    }
    void calc_cost(
            int, const CostInput<T>& in, T& cost) const override {
        // TODO a final state constraint probably makes more sense.
        const auto& q0 = in.final_states[0];
        const auto& q1 = in.final_states[1];
        const auto& L0 = this->L0;
        const auto& L1 = this->L1;
        Vector2<T> actual_location(L0 * cos(q0) + L1 * cos(q0 + q1),
                L0 * sin(q0) + L1 * sin(q0 + q1));
        const Vector2<T> desired_location(0, 2);
        cost = 1000.0 * (actual_location - desired_location).squaredNorm() +
                0.001 * in.final_time;
    }

    static void run_test(std::string solver, std::string hessian_approx,
            std::string transcription, int N = 100) {
        auto ocp = std::make_shared<DoublePendulumSwingUpMinTime<T>>();
        DirectCollocationSolver<T> dircol(ocp, transcription, solver, N);
        std::string jacobian_approx;
        if (hessian_approx == "exact") {
            jacobian_approx = hessian_approx;
        } else {
            jacobian_approx = "finite-difference-values";
        }
        dircol.get_opt_solver().set_jacobian_approximation(jacobian_approx);
        dircol.get_opt_solver().set_hessian_approximation(hessian_approx);
        dircol.get_opt_solver().set_sparsity_detection("random");
        tropter::Iterate guess;
        const int Nguess = 2;
        guess.time.setLinSpaced(Nguess, 0, 1);
        // Give a hint.
        ocp->set_state_guess(guess, "q0",
                Eigen::RowVectorXd::LinSpaced(Nguess, 0, -3./2.*PI));
        ocp->set_state_guess(guess, "q1",
                Eigen::RowVectorXd::LinSpaced(Nguess, 0, 2*PI));
        ocp->set_state_guess(guess, "u0", Eigen::RowVectorXd::Zero(Nguess));
        ocp->set_state_guess(guess, "u1", Eigen::RowVectorXd::Zero(Nguess));
        ocp->set_control_guess(guess, "tau0",
                Eigen::RowVectorXd::LinSpaced(Nguess, -50, 50));
        ocp->set_control_guess(guess, "tau1",
                Eigen::RowVectorXd::LinSpaced(Nguess, 50, -50));
        Solution solution = dircol.solve(guess);
        solution.write("double_pendulum_horizontal_to_vertical_solution.csv");
        // Check the final states.
        INFO(solution.states);
        TROPTER_REQUIRE_EIGEN(solution.states.rightCols<1>(),
                Eigen::Vector4d(-3./2.*PI, 2*PI, 0, 0), 1e-3);
        // Check the controls, which are bang-bang.
        testAlmostEqual(solution.controls.topLeftCorner<1, 40>(),
                Eigen::RowVectorXd::Constant(40, -50), 1e-2);
        testAlmostEqual(solution.controls.topRightCorner<1, 40>(),
                Eigen::RowVectorXd::Constant(40,  50), 1e-2);
        testAlmostEqual(solution.controls.bottomLeftCorner<1, 15>(),
                Eigen::RowVectorXd::Constant(15,  50), 1e-2);
        testAlmostEqual(solution.controls.bottomRightCorner<1, 15>(),
                Eigen::RowVectorXd::Constant(15, -50), 1e-2);
    }
};

TEST_CASE("Double pendulum swing up in minimum time.", 
          "[trapezoidal][hermite-simpson]")
{
    SECTION("IPOPT") {
        SECTION("Finite differences, limited-memory Hessian, trapezoidal") {
            DoublePendulumSwingUpMinTime<double>::run_test("ipopt",
                    "limited-memory", "trapezoidal");
        }
        // This test passes but it's just really slow:
        //SECTION("Finite differences, exact Hessian, trapezoidal") {
        //    DoublePendulumSwingUpMinTime<double>::run_test("ipopt",
        //            "exact", "trapezoidal");
        //}
        SECTION("ADOL-C, trapezoidal") {
            DoublePendulumSwingUpMinTime<adouble>::run_test("ipopt",
                    "limited-memory", "trapezoidal");
        }
        SECTION("Finite differences, limited-memory Hessian, hermite-simpson") {
            DoublePendulumSwingUpMinTime<double>::run_test("ipopt",
                "limited-memory", "hermite-simpson", 50);
        }
        //SECTION("Finite differences, exact Hessian, hermite-simpson") {
        //    DoublePendulumSwingUpMinTime<double>::run_test("ipopt",
        //            "exact", "hermite-simpson");
        //}
        SECTION("ADOL-C, hermite-simpson") {
            DoublePendulumSwingUpMinTime<adouble>::run_test("ipopt",
                "limited-memory", "hermite-simpson", 50);
        }
    }
    // Does not give desired answer (not fully bang-bang controls):
    // #if defined(TROPTER_WITH_SNOPT)
    // SECTION("SNOPT") {
    //     SECTION("ADOL-C, trapezoidal") {
    //         DoublePendulumSwingUpMinTime<adouble>::run_test("snopt",
    //                 "limited-memory", "trapezoidal");
    //     }
    //     SECTION("ADOL-C, hermite-simpson") {
    //         DoublePendulumSwingUpMinTime<adouble>::run_test("snopt",
    //                 "limited-memory", "hermite-simpson");
    //     }
    // }
    // #endif
}


template<typename T>
class DoublePendulumCoordinateTracking : public DoublePendulum<T> {
public:
    DoublePendulumCoordinateTracking()
    {
        this->set_time(0, 1);
        // TODO fix allowing final bounds to be unconstrained.
        this->add_state("q0", {-10, 10});
        this->add_state("q1", {-10, 10});
        this->add_state("u0", {-50, 50});
        this->add_state("u1", {-50, 50});
        this->add_control("tau0", {-100, 100});
        this->add_control("tau1", {-100, 100});
        this->add_cost("tracking", 1);
    }
    void calc_cost(
            int, const CostInput<T>& in, T& cost) const override {
        cost = in.integral;
    }
    void calc_cost_integrand(
            int, const Input<T>& in, T& integrand) const override {

        const auto& time = in.time;
        const auto& states = in.states;

        VectorX<T> desired(2);
        desired << (time / 1.0) * 0.50 * PI,
                (time / 1.0) * 0.25 * PI;
        integrand = (states.head(2) - desired).squaredNorm();
    }

    static Solution run_test(const std::string& solver,
            const std::string& hessian_approx, 
            const std::string& transcription, int N = 50) {
        auto ocp = std::make_shared<DoublePendulumCoordinateTracking<T>>();
        DirectCollocationSolver<T> dircol(ocp, transcription, solver, N);
        // Using an exact Hessian seems really important for this problem
        // (solves in only 20 iterations). Even a limited-memory problem started
        // from the solution using an exact Hessian does not converge.
        std::string jacobian_approx;
        if (hessian_approx == "exact") {
            jacobian_approx = hessian_approx;
        } else {
            jacobian_approx = "finite-difference-values";
        }
        dircol.get_opt_solver().set_jacobian_approximation(jacobian_approx);
        dircol.get_opt_solver().set_hessian_approximation(hessian_approx);
        dircol.get_opt_solver().set_sparsity_detection("random");
        Solution solution = dircol.solve();
        //dircol.print_constraint_values(solution);
        solution.write("double_pendulum_coordinate_tracking.csv");

        TROPTER_REQUIRE_EIGEN(solution.states.row(0),
                Eigen::RowVectorXd::LinSpaced(solution.time.size(), 0, 
                    0.50 * PI), 1e-3);
        TROPTER_REQUIRE_EIGEN(solution.states.row(1),
                Eigen::RowVectorXd::LinSpaced(solution.time.size(), 0, 
                    0.25 * PI), 1e-3);

        return solution;
    }
};

/// This class template defines the dynamics of a double pendulum using an
/// implicit formulation. To create an actual optimal control problem, one must
/// derive from this class template and define boundary conditions, cost terms,
/// etc.
template<typename T>
class ImplicitDoublePendulum : public tropter::Problem<T> {
public:
    constexpr static const double g = 9.81;
    double L0 = 1;
    double L1 = 1;
    double m0 = 1;
    double m1 = 1;
    void calc_differential_algebraic_equations(
            const Input<T>& in, Output<T> out) const override final {
        const auto& x = in.states;
        const auto& udot = in.controls.template head<2>();
        const auto& tau = in.controls.template tail<2>();
        const auto& q0 = x[0];
        const auto& q1 = x[1];
        const auto& u0 = x[2];
        const auto& u1 = x[3];
        const auto& L0 = this->L0;
        const auto& L1 = this->L1;
        const auto& m0 = this->m0;
        const auto& m1 = this->m1;
        out.dynamics[0] = u0;
        out.dynamics[1] = u1;
        out.dynamics[2] = udot[0];
        out.dynamics[3] = udot[1];

        if (out.path.size() != 0) {
            const T z0 = m1 * L0 * L1 * cos(q1);
            const T M01 = m1 * L1*L1 + z0;
            Matrix2<T> M;
            M << m0 * L0*L0 + m1 * (L0*L0 + L1*L1) + 2*z0,    M01,
                    M01,                                         m1 * L1*L1;
            Vector2<T> V(-u1 * (2 * u0 + u1),
                         u0 * u0);
            V *= m1*L0*L1*sin(q1);
            Vector2<T> G(g * ((m0 + m1) * L0 * cos(q0) + m1 * L1 * cos(q0 + q1)),
                         g * m1 * L1 * cos(q0 + q1));

            out.path = M * udot + V + G - tau;
        }
    }
};

template<typename T>
class ImplicitDoublePendulumCoordinateTracking : public
                                                 ImplicitDoublePendulum<T> {
public:
    ImplicitDoublePendulumCoordinateTracking() {
        this->set_time(0, 1);
        // TODO fix allowing final bounds to be unconstrained.
        this->add_state("q0", {-10, 10});
        this->add_state("q1", {-10, 10});
        this->add_state("u0", {-50, 50});
        this->add_state("u1", {-50, 50});
        this->add_control("udot0", {-100, 100});
        this->add_control("udot1", {-100, 100});
        this->add_control("tau0", {-100, 100});
        this->add_control("tau1", {-100, 100});
        this->add_cost("tracking", 1);
        this->add_path_constraint("u0", 0);
        this->add_path_constraint("u1", 0);
    }
    void calc_cost(
            int, const CostInput<T>& in, T& cost) const override {
        cost = in.integral;
    }
    void calc_cost_integrand(
            int, const Input<T>& in, T& integrand) const override {

        const auto& time = in.time;
        const auto& states = in.states;

        VectorX<T> desired(2);
        desired << (time / 1.0) * 0.50 * PI,
                   (time / 1.0) * 0.25 * PI;
        integrand = (states.template head<2>() - desired).squaredNorm();
    }
    static Solution run_test(const std::string& solver,
            const std::string& hessian_approx, 
            const std::string& transcription, int N = 50) {
        auto ocp =
                std::make_shared<ImplicitDoublePendulumCoordinateTracking<T>>();
        DirectCollocationSolver<T> dircol(ocp, transcription, solver, N);
        std::string jacobian_approx;
        if (hessian_approx == "exact") {
            jacobian_approx = hessian_approx;
        } else {
            jacobian_approx = "finite-difference-values";
        }
        dircol.get_opt_solver().set_jacobian_approximation(jacobian_approx);
        dircol.get_opt_solver().set_hessian_approximation(hessian_approx);
        dircol.get_opt_solver().set_sparsity_detection("random");
        dircol.get_opt_solver().set_advanced_option_string
                ("print_timing_statistics", "yes");
        Solution solution = dircol.solve();
        // dircol.print_constraint_values(solution);
        solution.write("implicit_double_pendulum_coordinate_tracking.csv");

        TROPTER_REQUIRE_EIGEN(solution.states.row(0),
                Eigen::RowVectorXd::LinSpaced(solution.time.size(), 0, 
                    0.50 * PI), 1e-3);
        TROPTER_REQUIRE_EIGEN(solution.states.row(1),
                Eigen::RowVectorXd::LinSpaced(solution.time.size(), 0, 
                    0.25 * PI), 1e-3);

        return solution;
    }
};

TEST_CASE("Double pendulum coordinate tracking",
        "[trapezoidal][hermite-simpson][implicitdynamics]")
{
    SECTION("IPOPT, trapezoidal") {
        // Make sure the solutions from the implicit and explicit
        // formulations are similar.

        // The explicit solution takes 20 iterations whereas the implicit
        // solution takes 25 iterations.
        const auto explicit_solution =
                DoublePendulumCoordinateTracking<adouble>::
                run_test("ipopt", "exact", "trapezoidal");

        const auto implicit_solution =
                ImplicitDoublePendulumCoordinateTracking<adouble>::
                run_test("ipopt", "exact", "trapezoidal");

        TROPTER_REQUIRE_EIGEN(explicit_solution.time,
                implicit_solution.time, 1e-10);
        // q0 and q1
        TROPTER_REQUIRE_EIGEN(explicit_solution.states.bottomRows(2),
                implicit_solution.states.bottomRows(2), 1e-2);
        // u0 and u1
        TROPTER_REQUIRE_EIGEN(explicit_solution.states.bottomRows(2),
                implicit_solution.states.bottomRows(2), 1e-2);
        // tau0 and tau1
        // The controls have the same shape but have a pretty large error
        // between them.
        // The peak magnitude of the torques is about 10-30 N-m, so a tolerance
        // of 5.0 N-m means the shape of the torques is preserved.
        CAPTURE(explicit_solution.controls);
        CAPTURE(implicit_solution.controls.bottomRows(2));
        TROPTER_REQUIRE_EIGEN_ABS(explicit_solution.controls,
                implicit_solution.controls.bottomRows(2), 5.0);


        // Finite differences.
        // -------------------
        // Check that finite differences are correct.
        OCPDerivativesComparison<DoublePendulumCoordinateTracking> c;
        c.findiff_hessian_step_size = 1e-5;
        c.gradient_error_tolerance = 1e-5;
        c.jacobian_error_tolerance = 1e-5;
        c.hessian_error_tolerance = 1e-2;
        c.compare();

        OCPDerivativesComparison<ImplicitDoublePendulumCoordinateTracking> ci;
        ci.findiff_hessian_step_size = 1e-5;
        ci.gradient_error_tolerance = 1e-5;
        ci.jacobian_error_tolerance = 1e-5;
        ci.hessian_error_tolerance = 1e-2;
        ci.compare();

        DoublePendulumCoordinateTracking<double>:: run_test("ipopt", "exact",
            "trapezoidal");
        ImplicitDoublePendulumCoordinateTracking<double>::
        run_test("ipopt", "exact", "trapezoidal");

        // The following do not converge:
        // EXIT: Maximum number of iterations exceeded.
        // DoublePendulumCoordinateTracking<adouble>::
        // run_test("ipopt", "limited-memory");
        // EXIT: Solved to Acceptable Level, "Restoration phase is called at
        // almost feasible point, but acceptable point from iteration 810 could
        // be restored." After 812 iterations. But solution is pretty wrong.
        // DoublePendulumCoordinateTracking<double>::
        // run_test("ipopt", "limited-memory");
        // EXIT: Maximum number of iterations exceeded.
        // ImplicitDoublePendulumCoordinateTracking<adouble>::
        // run_test("ipopt", "limited-memory");
        // EXIT: Restoration failed after 235 iterations.
        // ImplicitDoublePendulumCoordinateTracking<double>::
        // run_test("ipopt", "limited-memory");

    }
    SECTION("IPOPT, hermite-simpson") {
        // Make sure the solutions from the implicit and explicit
        // formulations are similar.

        // The explicit solution takes 20 iterations whereas the implicit
        // solution takes 25 iterations.
        const auto explicit_solution =
            DoublePendulumCoordinateTracking<adouble>::
            run_test("ipopt", "exact", "hermite-simpson", 25);

        const auto implicit_solution =
            ImplicitDoublePendulumCoordinateTracking<adouble>::
            run_test("ipopt", "exact", "hermite-simpson", 25);

        TROPTER_REQUIRE_EIGEN(explicit_solution.time,
            implicit_solution.time, 1e-10);
        // q0 and q1
        TROPTER_REQUIRE_EIGEN(explicit_solution.states.bottomRows(2),
            implicit_solution.states.bottomRows(2), 1e-2);
        // u0 and u1
        TROPTER_REQUIRE_EIGEN(explicit_solution.states.bottomRows(2),
            implicit_solution.states.bottomRows(2), 1e-2);
        // tau0 and tau1
        CAPTURE(explicit_solution.controls);
        CAPTURE(implicit_solution.controls.bottomRows(2));
        // TODO this fails since control become zero at the midpoint for 
        // Hermite-Simpson transcription.
        //TROPTER_REQUIRE_EIGEN_ABS(explicit_solution.controls,
        //    implicit_solution.controls.bottomRows(2), 5.0);

        DoublePendulumCoordinateTracking<double>::run_test("ipopt", "exact",
            "hermite-simpson", 25);
        ImplicitDoublePendulumCoordinateTracking<double>::
            run_test("ipopt", "exact", "hermite-simpson", 25);
    }
    /*
    #if defined(TROPTER_WITH_SNOPT)
    SECTION("SNOPT") {
        // TODO SNOPT will get the correct solution if the initial guess is the
        // solution from IPOPT, but otherwise, the solution is off (u0/u1
        // dynamics violation is 1e-7?).
        //ImplicitDoublePendulumCoordinateTracking<adouble>::run_test("snopt",
        //        "limited-memory");
        const auto explicit_solution =
                DoublePendulumCoordinateTracking<adouble>::run_test("ipopt");
        auto ocp = std::make_shared<DoublePendulumCoordinateTracking<adouble>>();
        tropter::Iterate guess;

        const int N = 100;
        guess.time.setLinSpaced(N, 0, 1);
        ocp->set_state_guess(guess, "q0",
                Eigen::RowVectorXd::LinSpaced(N, 0, 0.5*PI));
        ocp->set_state_guess(guess, "q1",
                Eigen::RowVectorXd::LinSpaced(N, 0, 0.25*PI));
        ocp->set_state_guess(guess, "u0", Eigen::RowVectorXd::Zero(N));
        ocp->set_state_guess(guess, "u1", Eigen::RowVectorXd::Zero(N));
        ocp->set_control_guess(guess, "tau0", Eigen::RowVectorXd::Zero(N));
        ocp->set_control_guess(guess, "tau1", Eigen::RowVectorXd::Zero(N));
        DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal", "snopt", N);
        Solution solution = dircol.solve(guess);
        dircol.print_constraint_values(solution);
        solution.write("double_pendulum_coordinate_tracking_snopt.csv");
    }
    #endif
    */
}

// TODO include acceleration and deceleration.
