#define CATCH_CONFIG_MAIN
#include <catch.hpp>
#include "testing.h"

#include <tropter.h>

using namespace tropter;
using Eigen::VectorXd;
using Eigen::RowVectorXd;
using Eigen::Vector2d;
using Eigen::MatrixXd;
using Eigen::Ref;

template<typename T>
class SlidingMassPathConstraint : public tropter::OptimalControlProblemNamed<T> {
public:
    const double mass = 10.0;
    const double Fmax = 10;
    SlidingMassPathConstraint()
    {
        this->set_time({0}, {0, 10});
        this->add_state("x", {0, 1}, {0}, {1});
        this->add_state("u", {-100, 100}, {0}, {0});
        this->add_control("a", {-100, 100});
        this->add_control("F", {-Fmax, Fmax});
        this->add_path_constraint("F=ma", 0);
    }
    void dynamics(const VectorX<T>& states,
                  const VectorX<T>& controls,
                  Ref<VectorX<T>> derivatives) const override
    {
        derivatives[0] = states[1];
        // udot = a
        derivatives[1] = controls[0];
}
    void path_constraints(unsigned /*i_mesh*/,
                          const T& /*time*/,
                          const VectorX<T>& /*states*/,
                          const VectorX<T>& controls,
                          Ref<VectorX<T>> constr) const override
    {
        // F = ma
        constr[0] = controls[1] - mass*controls[0];
    }
    void endpoint_cost(const T& final_time, const VectorX<T>&, T& cost)
    const override
    {
        cost = final_time;
    }
    OptimalControlSolution actual_solution(const VectorXd& time) const
    {
        OptimalControlSolution sol;
        sol.time = time;
        sol.states.resize(2, time.size());
        sol.controls.resize(2, time.size());

        auto x = [&](const double& t) -> double {
            if (t < 0.5*time[time.size() - 1]) { return 0.5*t*t; }
            else { return -0.5*(t - 1)*(t - 1) + 1*(t - 1) + 0.5; }
        };
        sol.states.row(0) = sol.time.unaryExpr(x);

        auto u = [&](const double& t) -> double {
            if (t < 0.5*time[time.size() - 1]) { return t; }
            else { return 2 - t; }
        };
        sol.states.row(1) = sol.time.unaryExpr(u);

        auto F = [&](const double& t) -> double {
            if (t < 0.5*time[time.size() - 1]) { return Fmax; }
            else { return -Fmax; }
        };
        sol.controls.row(1) = sol.time.unaryExpr(F);

        // a = F/m
        sol.controls.row(0) = sol.controls.row(1)/mass;

        return sol;
    }
};

TEST_CASE("Sliding mass minimum time using path constraints", "[path]")
{
    auto ocp = std::make_shared<SlidingMassPathConstraint<adouble>>();
    const int halfN = 25;
    const int N = 2*halfN;
    DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal", "ipopt", N);
    OptimalControlSolution solution = dircol.solve();
    solution.write("sliding_mass_minimum_time_path_constraints_solution.csv");

    OptimalControlSolution expected = ocp->actual_solution(solution.time);

    TROPTER_REQUIRE_EIGEN(solution.states, expected.states, 0.001);
    TROPTER_REQUIRE_EIGEN(solution.controls, expected.controls, 0.001);
}
