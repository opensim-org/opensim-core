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
    SlidingMassMinimumTime()
    {
        // TODO when time is a variable, this has to be more advanced:
        this->set_time({0}, {1.5, 10});
        this->add_state("x", {0, 2}, {0}, {1});
        this->add_state("u", {-10, 10}, {0}, {0});
        this->add_control("F", {-50, 50});
    }
    const double mass = 10.0;
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
};

TEST_CASE("Sliding mass minimum time.")
{

    auto ocp = std::make_shared<SlidingMassMinimumTime<adouble>>();
    DirectCollocationSolver<adouble> dircol(ocp, "euler", "snopt");
    OptimalControlSolution solution = dircol.solve();
    solution.write("sliding_mass_minimum_time_solution.csv");

    // Initial and final position.
    REQUIRE(Approx(solution.states(0, 0)) == 0.0);
    REQUIRE(Approx(solution.states.rightCols<1>()[0]) == 1.0);
    // Initial and final speed.
    REQUIRE(Approx(solution.states(1, 0)) == 0.0);
    REQUIRE(Approx(solution.states.rightCols<1>()[1]) == 0.0);

    int N = solution.time.size();
    std::cout << "DEBUG solution.controls " << solution.controls << std::endl;
    //RowVectorXd expected = RowVectorXd::LinSpaced(N - 1, 14.25, -14.25);
    //RowVectorXd errors = solution.controls.rightCols(N - 1) - expected;
    //REQUIRE(Approx(errors.norm()) == 0);
}

