
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

/// Kirk 1998, Example 5.1-1, page 198.
template<typename T>
class SecondOrderLinearMinEffort : public tropter::OptimalControlProblem<T> {
public:
    SecondOrderLinearMinEffort() {
        this->set_time(0, 2);
        this->add_state("x0", {-10, 10}, {0}, {5});
        this->add_state("x1", {-10, 10}, {0}, {2});
        this->add_control("u", {-50, 50});
    }
    void calc_differential_algebraic_equations(unsigned /*mesh_index*/,
            const T& /*time*/, const VectorX<T>& x, const VectorX<T>& u,
            Ref<VectorX<T>> xdot, Ref<VectorX<T>> /*constr*/) const override {
        xdot[0] = x[1];
        xdot[1] = -x[1] + u[0];
        // xdot.row(0) = x.row(1);
        // xdot.row(1) = -x.row(1) + u.row(0);
    }
    void calc_integral_cost(const T& /*t*/,
            const VectorX<T>& /*x*/,
            const VectorX<T>& u,
            T& integrand) const override {
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
};

TEST_CASE("Second order linear min effort", "[adolc][trapezoidal]") {

    auto ocp = std::make_shared<SecondOrderLinearMinEffort<adouble>>();
    DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal", "ipopt", 1000);
    OptimalControlSolution solution = dircol.solve();
    //solution.write("second_order_linear_min_effort_solution.csv");

    MatrixXd expected_states = ocp->states_solution(solution.time);

    TROPTER_REQUIRE_EIGEN(solution.states, expected_states, 0.005);
}

//TEST_CASE("Second order linear min effort", "[finitediff][trapezoidal]") {
//
//    auto ocp = std::make_shared<SecondOrderLinearMinEffort<double>>();
//    DirectCollocationSolver<double> dircol(ocp, "trapezoidal", "ipopt", 1000);
//    OptimalControlSolution solution = dircol.solve();
//    //solution.write("second_order_linear_min_effort_solution.csv");
//
//    MatrixXd expected_states = ocp->states_solution(solution.time);
//
//    TROPTER_REQUIRE_EIGEN(solution.states, expected_states, 0.005);
//}
