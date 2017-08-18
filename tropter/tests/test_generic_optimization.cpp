
#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <tropter.h>
#include <legacy.h>

#include "testing.h"

using Eigen::Vector4d;
using Eigen::Vector2d;
using Eigen::VectorXd;

using namespace tropter;
// TODO test an unconstrained problem.

// TODO elegantly handle the case where the objective is not defined (do not
// need ADOL-C to do a bunch of stuff.

// TEST a problem without derivative information.


// TODO test exceptions for upper bound < lower bound.

// TODO allow copying the problem...

TEST_CASE("Ipopt and ADOL-C, unconstrained TODO new interface") {
    // Make sure it's okay to not have constraints.

    class Unconstrained : public OptimizationProblem<adouble> {
    public:
        Unconstrained() : OptimizationProblem(2, 0) {
            set_variable_bounds(Vector2d(-5, -5), Vector2d(5, 5));
        }
        void objective(
                const VectorXa& x, adouble& obj_value) const override {
            obj_value = (x[0] - 1.5) * (x[0] - 1.5)
                      + (x[1] + 2.0) * (x[1] + 2.0);
        }
    };

    Unconstrained problem;
    // TODO may not want user to directly use IpoptSolver; instead, use a
    // generic solver interface?
    IpoptSolver solver(problem);
    VectorXd variables = Vector2d(0, 0);
    double obj_value = solver.optimize(variables);

    REQUIRE(Approx(variables[0]) == 1.5);
    REQUIRE(Approx(variables[1]) == -2.0);
    REQUIRE(Approx(obj_value)   == 0);

    // TODO throw exception if constraints() is unimplemented and
    // there are nonzero number of constraints.
}


TEST_CASE("Ipopt C++ tutorial problem HS071; constraints and ADOL-C. TO") {
    // This is mostly a test that the automatic differentiation works.

    /// This problem comes from
    /// https://www.coin-or.org/Ipopt/documentation/node23.html
    // TODO this class should be templated.
    class HS071 : public OptimizationProblem<adouble> {
    public:
        HS071() : OptimizationProblem(4, 2) {
            set_variable_bounds(Vector4d(1, 1, 1, 1), Vector4d(5, 5, 5, 5));
            set_constraint_bounds(Vector2d(25, 40), Vector2d(2e19, 40.0));
        }
        void objective(const VectorXa& x, adouble& obj_value) const override {
            obj_value = x[0] * x[3] * (x[0] + x[1] + x[2]) + x[2];
        }
        void constraints(
                const VectorXa& x, Eigen::Ref<VectorXa> constr) const override {
            constr[0] = x.prod();
            constr[1] = x.squaredNorm();
        }
    };

    HS071 problem;
    IpoptSolver solver(problem);
    VectorXd variables = Vector4d(1.5, 2.5, 3.5, 4.5);
    double obj_value = solver.optimize(variables);

    // TODO run the Ipopt derivative check. in the test.

    //// Make sure the ADOL-C derivatives are correct.
    //// Note: this check only emits warnings, and won't cause the test to fail.
    //app->Options()->SetStringValue("derivative_test", "second-order");

    REQUIRE(       variables[0]  == 1.0);
    REQUIRE(Approx(variables[1]) == 4.743);
    REQUIRE(Approx(variables[2]) == 3.82115);
    REQUIRE(Approx(variables[3]) == 1.379408);

    REQUIRE(Approx(obj_value) == 17.014);
}

TEST_CASE("Ipopt and ADOL-C, unconstrained") {
    // Make sure it's okay to not have constraints.

    class Unconstrained : public legacy::IpoptADOLC_OptimizationProblem {
    public:
        Unconstrained() : IpoptADOLC_OptimizationProblem(2, 0) {}
        void objective(const std::vector<adouble>& x,
                       adouble& obj_value) const override {
            obj_value = (x[0] - 1.5) * (x[0] - 1.5)
                      + (x[1] + 2.0) * (x[1] + 2.0);
        }
        // TODO Should not have to define this.
        void constraints(const std::vector<adouble>&,
                         std::vector<adouble>&) const override {}
    };

    Ipopt::SmartPtr<Unconstrained> prob = new Unconstrained();
    prob->set_variable_bounds({-5, -5}, {5, 5});
    // TODO Test that setting constraint bounds for an unconstrained problem
    // gives an error.
    // TODO set initial guess?

    // TODO Not setting an initial guess causes an error...is that okay?
    prob->set_initial_guess({0, 0});

    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
    Ipopt::ApplicationReturnStatus status = app->Initialize();
    // Did initialization succeed?
    REQUIRE(status == Ipopt::Solve_Succeeded);
    status = app->OptimizeTNLP(prob);
    REQUIRE(status == Ipopt::Solve_Succeeded);

    std::vector<double> solution = prob->get_solution();
    REQUIRE(Approx(solution[0]) == 1.5);
    REQUIRE(Approx(solution[1]) == -2.0);
}

TEST_CASE("Ipopt C++ tutorial problem HS071; constraints and ADOL-C.") {
    // This is mostly a test that the automatic differentiation works.

    /// This problem comes from
    /// https://www.coin-or.org/Ipopt/documentation/node23.html
    // TODO this class should be templated.
    class HS071 : public legacy::IpoptADOLC_OptimizationProblem {
    public:
        HS071() : IpoptADOLC_OptimizationProblem(4, 2) {}
        void objective(const std::vector<adouble>& x,
                       adouble& obj_value) const override {
            obj_value = x[0] * x[3] * (x[0] + x[1] + x[2]) + x[2];
        }
        void constraints(const std::vector<adouble>& x,
                         std::vector<adouble>& constraints) const override {
            constraints[0] = x[0] * x[1] * x[2] * x[3];
            constraints[1] = x[0] * x[0] + x[1] * x[1]
                           + x[2] * x[2] + x[3] * x[3];
        }
    };

    Ipopt::SmartPtr<HS071> prob = new HS071();
    prob->set_variable_bounds({1, 1, 1, 1}, {5, 5, 5, 5});
    prob->set_constraint_bounds({25, 40}, {2e19, 40.0});
    // Arbitrary initial guess.
    prob->set_initial_guess({1.5, 2.5, 3.5, 4.5});

    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
    // Make sure the ADOL-C derivatives are correct.
    // Note: this check only emits warnings, and won't cause the test to fail.
    app->Options()->SetStringValue("derivative_test", "second-order");
    Ipopt::ApplicationReturnStatus status;
    status = app->Initialize();
    if (status != Ipopt::Solve_Succeeded) {
        FAIL("Error during initialization");
    }
    status = app->OptimizeTNLP(prob);
    if (status == Ipopt::Solve_Succeeded) {
        INFO("\n\n*** The problem solved!\n");
    } else {
        FAIL("\n\n*** The problem FAILED!\n");
    }

    // TODO it's awkward to have to call "get_solution".
    // Wrap the Ipopt interface!
    std::vector<double> solution = prob->get_solution();

    REQUIRE(solution[0] == 1.0);
    REQUIRE(Approx(solution[1]) == 4.743);
    REQUIRE(Approx(solution[2]) == 3.82115);
    REQUIRE(Approx(solution[3]) == 1.379408);

}

TEST_CASE("Generating an initial guess using problem bounds",
          "[initialguess]") {
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
    class VarietyOfBounds : public OptimizationProblem<adouble> {
    public:
        const double inf = std::numeric_limits<double>::infinity();
        VarietyOfBounds() : OptimizationProblem(4, 0) {
            set_variable_bounds(Vector4d(-inf, -20, -inf,  50),
                                Vector4d( inf,  10,   -8, inf));
        }
        void objective(const VectorXa& x, adouble& obj_value) const override {
            obj_value = x.squaredNorm();
        }
    };

    VarietyOfBounds problem;
    const auto proxy = problem.make_proxy();

    VectorXd actual = proxy->initial_guess_from_bounds();
    Vector4d expected(0, -5, -8, 50);
    TROPTER_REQUIRE_EIGEN(actual, expected, 1e-10);
}




