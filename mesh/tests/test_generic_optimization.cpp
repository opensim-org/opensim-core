//
// Created by Chris Dembia on 12/6/16.
//

#include <mesh.h>

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
