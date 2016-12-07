
#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <mesh.h>

// TODO test an unconstrained problem.

// TEST a problem without derivative information.

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
    // TODO try two different initial guesses.
    prob->set_initial_guess({1.5, 2.5, 3.5, 4.5});

    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
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
// TODO move elsewhere.
class ToyProblem : public IpoptADOLC_OptimizationProblem {
public:
    ToyProblem() : IpoptADOLC_OptimizationProblem(2, 1) {}
    void objective(const std::vector<adouble>& x,
                   adouble& obj_value) const override {
        obj_value = (x[0] - 1.5) * (x[0] - 1.5)
                    + (x[1] + 2.0) * (x[1] + 2.0);
    }
    void constraints(const std::vector<adouble>& x,
                     std::vector<adouble>& constraints) const override {
        constraints[0] = x[1] - x[0] * x[0]; //x[0] + x[1];
    }
};
//        // Ipopt::SmartPtr<ToyProblem> mynlp = new ToyProblem();
//        // mynlp->set_variable_bounds({-5, -5}, {5, 5});
//        // mynlp->set_constraint_bounds({-0.1}, {0.1});
//        // mynlp->set_initial_guess({0, 0});
//        Ipopt::SmartPtr<DirectCollocationSolver> mynlp =
//                new DirectCollocationSolver();
//        std::shared_ptr<OptimalControlProblem<adouble>> problem(
//                new SlidingMass());
//        mynlp->set_problem(problem);
//        // TODO return 0;
//        Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
//        app->Options()->SetNumericValue("tol", 1e-9);
//        app->Options()->SetStringValue("mu_strategy", "adaptive");
//        app->Options()->SetStringValue("output_file", "ipopt.out");
//        // TODO temporary:
//        app->Options()->SetStringValue("derivative_test", "second-order");
//        Ipopt::ApplicationReturnStatus status;
//        status = app->Initialize();
//        if (status != Ipopt::Solve_Succeeded) {
//            printf("\n\n*** Error during initialization!\n");
//            FAIL("Error during initialization");
//        }
//        status = app->OptimizeTNLP(mynlp);
//        if (status == Ipopt::Solve_Succeeded) {
//            printf("\n\n*** The problem solved!\n");
//        } else {
//            printf("\n\n*** The problem FAILED!\n");
//        }
//        //ToyProblem toy;
//        //adouble f;
//        //toy.objective({1.5, -2.0}, f);
//        //std::cout << "DEBUG " << f << std::endl;
