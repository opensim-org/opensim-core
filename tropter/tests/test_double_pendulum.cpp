
#define CATCH_CONFIG_MAIN
#include <catch.hpp>
#include "testing.h"

#include <tropter/tropter.h>

#include <Eigen/LU>

using Eigen::Ref;
using Eigen::VectorXd;
using Eigen::RowVectorXd;
using Eigen::Vector2d;
using Eigen::MatrixXd;
using Eigen::Matrix2d;

using namespace tropter;

/// This class template defines the dynamics of a double pendulum. To create
/// an actual optimal control problem, one must derive from this class
/// template and define boundary conditions, cost terms, etc.
template<typename T>
class DoublePendulum : public tropter::OptimalControlProblem<T> {
public:
    constexpr static const double g = 9.81;
    double L0 = 1;
    double L1 = 1;
    double m0 = 1;
    double m1 = 1;

    void dynamics(const VectorX<T>& x, const VectorX<T>& tau,
            Ref<VectorX<T>> xdot) const override final
    {
        const auto& q0 = x[0];
        const auto& q1 = x[1];
        const auto& u0 = x[2];
        const auto& u1 = x[3];
        const auto& L0 = this->L0;
        const auto& L1 = this->L1;
        const auto& m0 = this->m0;
        const auto& m1 = this->m1;
        xdot[0] = u0;
        xdot[1] = u1;

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
        xdot.tail(2) = M.inverse() * (tau - (V + G));
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
    }
    void endpoint_cost(const T& final_time, const VectorX<T>& final_states,
            T& cost) const override
    {
        // TODO a final state constraint probably makes more sense.
        const auto& q0 = final_states[0];
        const auto& q1 = final_states[1];
        const auto& L0 = this->L0;
        const auto& L1 = this->L1;
        Vector2<T> actual_location(L0 * cos(q0) + L1 * cos(q0 + q1),
                                   L0 * sin(q0) + L1 * sin(q0 + q1));
        const Vector2<T> desired_location(0, 2);
        cost = 1000.0 * (actual_location - desired_location).squaredNorm() +
                0.001 * final_time;
    }

    static void run_test(const std::string& solver) {
        auto ocp = std::make_shared<DoublePendulumSwingUpMinTime<T>>();
        const int N = 100;
        DirectCollocationSolver<T> dircol(ocp, "trapezoidal", solver, N);
        dircol.get_optimization_solver().set_hessian_approximation(
                "limited-memory");
        tropter::OptimalControlIterate guess;
        guess.time.setLinSpaced(N, 0, 1);
        const double pi = 3.14159;
        // Give a hint (not the exact final state, but something close to it).
        // I tried giving a guess where the final state guess was from the
        // solution (-3/2pi, -2pi), but then Ipopt incorrectly thought the
        // solution was all zeros.
        ocp->set_state_guess(guess, "q0",
                Eigen::RowVectorXd::LinSpaced(N, 0, -pi));
        ocp->set_state_guess(guess, "q1",
                Eigen::RowVectorXd::LinSpaced(N, 0, 2*pi));
        ocp->set_state_guess(guess, "u0", Eigen::RowVectorXd::Zero(N));
        ocp->set_state_guess(guess, "u1", Eigen::RowVectorXd::Zero(N));
        ocp->set_control_guess(guess, "tau0", Eigen::RowVectorXd::Zero(N));
        ocp->set_control_guess(guess, "tau1", Eigen::RowVectorXd::Zero(N));
        OptimalControlSolution solution = dircol.solve(guess);
        solution.write("double_pendulum_horizontal_to_vertical_solution.csv");
        // Check the final states.
        INFO(solution.states);
        TROPTER_REQUIRE_EIGEN(solution.states.rightCols<1>(),
                Eigen::Vector4d(-3./2.*pi, 2*pi, 0, 0), 1e-3);
        // Check the initial and final controls (which are bang-bang).
        TROPTER_REQUIRE_EIGEN(solution.controls.leftCols<1>(),
                Eigen::Vector2d(-50, 50), 1e-2);
        TROPTER_REQUIRE_EIGEN(solution.controls.rightCols<1>(),
                Eigen::Vector2d(50, -50), 1e-2);
    }
};

TEST_CASE("Double pendulum swing up in minimum time.", "[trapezoidal]")
{
    SECTION("Ipopt") {
        SECTION("Finite differences") {
            DoublePendulumSwingUpMinTime<double>::run_test("ipopt");
        }
        SECTION("ADOL-C") {
            DoublePendulumSwingUpMinTime<adouble>::run_test("ipopt");
        }
    }
    #if defined(MUSCOLLO_WITH_SNOPT)
    SECTION("SNOPT") {
        SECTION("ADOL-C") {
            DoublePendulumSwingUpMinTime<adouble>::run_test("snopt");
        }
    }
    #endif
}


