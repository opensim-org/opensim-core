
#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <mesh.h>

// TODO test an unconstrained problem.

// TEST a problem without derivative information.

TEST_CASE("Ipopt and ADOL-C, unconstrained") {
    // Make sure it's okay to not have constraints.

    class Unconstrained : public IpoptADOLC_OptimizationProblem {
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
    class HS071 : public IpoptADOLC_OptimizationProblem {
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
