
#include <tropter/tropter.h>

#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include "testing.h"

using Eigen::Ref;
using Eigen::VectorXd;
using Eigen::RowVectorXd;
using Eigen::Vector2d;
using Eigen::MatrixXd;

using namespace tropter;

template<typename T>
class SlidingMass : public tropter::OptimalControlProblem<T> {
public:
    SlidingMass() {
        // TODO when time is a variable, this has to be more advanced:
        this->set_time({0}, {2});
        this->add_state("x", {0, 2}, {0}, {1});
        this->add_state("u", {-10, 10}, {0}, {0});
        this->add_control("F", {-50, 50});
        // this->add_state("x", Bounds(  0,  2), InitialBounds(0), FinalBounds(1));
        // this->add_state("u", Bounds(-10, 10), InitialBounds(0), FinalBounds(0));
        // this->add_control("F", Bounds(-50, 50));
    }
    const double mass = 10.0;
    void calc_differential_algebraic_equations(unsigned /*mesh_index*/,
            const T& /*time*/,
            const VectorX<T>& states, const VectorX<T>& controls,
            Ref<VectorX<T>> derivatives,
            Ref<VectorX<T>> /*constr*/) const override {
        derivatives[0] = states[1];
        derivatives[1] = controls[0] / mass;
    }
    // TODO alternate form that takes a matrix; state at every time.
    //virtual void continuous(const MatrixXd& x, MatrixXd& xdot) const = 0;
    void calc_integral_cost(const T& /*time*/,
            const VectorX<T>& /*states*/,
            const VectorX<T>& controls,
            T& integrand) const override {
        integrand = controls[0] * controls[0];
    }
};

TEST_CASE("Sliding mass with Ipopt") {

    auto ocp = std::make_shared<SlidingMass<adouble>>();
    DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal", "ipopt");
    OptimalControlSolution solution = dircol.solve();
    solution.write("sliding_mass_solution.csv");
    //OptimalControlIterate initial_guess = ocp->make_guess_template();
    //OptimalControlSolution solution = dircol.solve(initial_guess);

    // Initial and final position.
    REQUIRE(Approx(solution.states(0, 0)) == 0.0);
    REQUIRE(Approx(solution.states.rightCols<1>()[0]) == 1.0);
    // Initial and final speed.
    REQUIRE(Approx(solution.states(1, 0)) == 0.0);
    REQUIRE(Approx(solution.states.rightCols<1>()[1]) == 0.0);

    int N = (int)solution.time.size();
    std::cout << "DEBUG solution.controls " << solution.controls << std::endl;
    // TODO is this really the correct solution?
    RowVectorXd expected = RowVectorXd::LinSpaced(N - 2, 14.6119, -14.6119);
    TROPTER_REQUIRE_EIGEN(solution.controls.middleCols(1, N - 2), expected,
            0.1);
    //RowVectorXd errors = solution.controls.rightCols(N - 1) - expected;
    //REQUIRE(Approx(errors.norm()) == 0);
}

#if defined(MUSCOLLO_WITH_SNOPT)
TEST_CASE("Sliding mass new interface") {

    auto ocp = std::make_shared<SlidingMass<adouble>>();
    DirectCollocationSolver<adouble> dircol(ocp, "trapezoidal", "snopt");
    OptimalControlSolution solution = dircol.solve();
    solution.write("sliding_mass_solution.csv");
    //OptimalControlIterate initial_guess = ocp->make_guess_template();
    //OptimalControlSolution solution = dircol.solve(initial_guess);

    // Initial and final position.
    REQUIRE(Approx(solution.states(0, 0)) == 0.0);
    REQUIRE(Approx(solution.states.rightCols<1>()[0]) == 1.0);
    // Initial and final speed.
    REQUIRE(Approx(solution.states(1, 0)) == 0.0);
    REQUIRE(Approx(solution.states.rightCols<1>()[1]) == 0.0);

    int N = (int)solution.time.size();
    std::cout << "DEBUG solution.controls " << solution.controls << std::endl;
    // TODO is this really the correct solution?
    RowVectorXd expected = RowVectorXd::LinSpaced(N - 2, 14.6119, -14.6119);
    TROPTER_REQUIRE_EIGEN(solution.controls.middleCols(1, N - 2), expected,
     0.1);
    //RowVectorXd errors = solution.controls.rightCols(N - 1) - expected;
    //REQUIRE(Approx(errors.norm()) == 0);
}
#endif

