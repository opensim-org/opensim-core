#include <mesh.h>

#define CATCH_CONFIG_MAIN
#include <catch.hpp>

using Eigen::Ref;
using Eigen::VectorXd;
using Eigen::RowVectorXd;
using Eigen::MatrixXd;

using namespace mesh;

template<typename T>
class SlidingMassMinimumTime : public mesh::OptimalControlProblemNamed<T> {
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
    void dynamics(const VectorX<T>& states, const VectorX<T>& controls,
                  Ref<VectorX<T>> derivatives) const override
    {
        derivatives[0] = states[1];
        derivatives[1] = controls[0]/mass;
    }
    // TODO alternate form that takes a matrix; state at every time.
    //virtual void continuous(const MatrixXd& x, MatrixXd& xdot) const = 0;
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
};

TEST_CASE("Sliding mass minimum time.")
{
    auto ocp = std::make_shared<SlidingMassMinimumTime<adouble>>();
    const int halfN = 25;
    const int N = 2 * halfN;
    DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal", "ipopt", N);
    OptimalControlSolution solution = dircol.solve();
    solution.write("sliding_mass_minimum_time_solution.csv");

    OptimalControlSolution expected = ocp->actual_solution(solution.time);

//    REQUIRE_EIGEN(solution.states, expected.states, 0.001);
//    REQUIRE_EIGEN(solution.controls, expected.controls, 0.001);

    for (int im = 0; im < solution.time.size(); ++im) {
        for (int is = 0; is < solution.states.rows(); ++is) {
            const auto abs_error = std::abs(
                    solution.states(is, im) - expected.states(is, im));
            REQUIRE(abs_error < 0.001);
        }
        for (int ic = 0; ic < solution.controls.rows(); ++ic) {
            const auto abs_error = std::abs(
                    solution.controls(ic, im) - expected.controls(ic, im));
            REQUIRE(abs_error < 0.001);
        }
    }
}

