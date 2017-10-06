
#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <tropter/tropter.h>

#include "testing.h"

using Eigen::Vector4d;
using Eigen::Vector2d;
using Eigen::VectorXd;

using namespace tropter;

// TODO elegantly handle the case where the objective is not defined (do not
// need ADOL-C to do a bunch of stuff.

// TODO test exceptions for upper bound < lower bound.

// TODO allow copying the problem...

template<typename T>
class Unconstrained : public OptimizationProblem<T> {
public:
    Unconstrained() : OptimizationProblem<T>(2, 0) {
        this->set_variable_bounds(Vector2d(-5, -5), Vector2d(5, 5));
    }
    void calc_objective(
            const VectorX<T>& x, T& obj_value) const override {
        obj_value = (x[0] - 1.5) * (x[0] - 1.5)
                + (x[1] + 2.0) * (x[1] + 2.0);
    }
};

TEST_CASE("Unconstrained, IpoptSolver", "[ipopt]") {
    // Make sure it's okay to not have constraints.
    SECTION("Finite differences, limited memory") {
        Unconstrained<double> problem;
        // TODO may not want user to directly use IpoptSolver; instead, use a
        // generic solver interface?
        IpoptSolver solver(problem);
        VectorXd variables = Vector2d(0, 0);
        solver.set_hessian_approximation("limited-memory");
        double obj_value = solver.optimize(variables);

        REQUIRE(Approx(variables[0]) == 1.5);
        REQUIRE(Approx(variables[1]) == -2.0);
        REQUIRE(Approx(obj_value)   == 0);

        // TODO throw exception if constraints() is unimplemented and
        // there are nonzero number of constraints.
    }
    SECTION("Finite differences, exact Hessian") {
        Unconstrained<double> problem;
        IpoptSolver solver(problem);
        VectorXd variables = Vector2d(0, 0);
        solver.set_hessian_approximation("exact");
        double obj_value = solver.optimize(variables);

        REQUIRE(Approx(variables[0]) == 1.5);
        REQUIRE(Approx(variables[1]) == -2.0);
        REQUIRE(Approx(obj_value)   == 0);
    }
    SECTION("ADOL-C") {
        // Make sure it's okay to not have constraints.
        Unconstrained<adouble> problem;
        IpoptSolver solver(problem);
        VectorXd variables = Vector2d(0, 0);
        double obj_value = solver.optimize(variables);

        REQUIRE(Approx(variables[0]) == 1.5);
        REQUIRE(Approx(variables[1]) == -2.0);
        REQUIRE(Approx(obj_value)   == 0);

        // TODO throw exception if constraints() is unimplemented and
        // there are nonzero number of constraints.
    }
}

/// This problem comes from
/// https://www.coin-or.org/Ipopt/documentation/node23.html
template<typename T>
class HS071 : public OptimizationProblem<T> {
public:
    HS071() : OptimizationProblem<T>(4, 2) {
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

TEST_CASE("Ipopt C++ tutorial problem HS071; has constraints.") {
    SECTION("Finite differences") {
        HS071<double> problem;
        IpoptSolver solver(problem);
        solver.set_hessian_approximation("limited-memory");
        VectorXd variables = Vector4d(1.5, 2.5, 3.5, 4.5);
        double obj_value = solver.optimize(variables);

        REQUIRE(       variables[0]  == 1.0);
        REQUIRE(Approx(variables[1]) == 4.743);
        REQUIRE(Approx(variables[2]) == 3.82115);
        REQUIRE(Approx(variables[3]) == 1.379408);

        REQUIRE(Approx(obj_value) == 17.014);
    }
    SECTION("ADOL-C") {
        HS071<adouble> problem;
        IpoptSolver solver(problem);
        VectorXd variables = Vector4d(1.5, 2.5, 3.5, 4.5);
        double obj_value = solver.optimize(variables);

        REQUIRE(       variables[0]  == 1.0);
        REQUIRE(Approx(variables[1]) == 4.743);
        REQUIRE(Approx(variables[2]) == 3.82115);
        REQUIRE(Approx(variables[3]) == 1.379408);

        REQUIRE(Approx(obj_value) == 17.014);
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
class VarietyOfBounds : public OptimizationProblem<T> {
public:
    const double inf = std::numeric_limits<double>::infinity();
    VarietyOfBounds() : OptimizationProblem<T>(4, 0) {
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




