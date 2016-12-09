
#include <mesh.h>

#define CATCH_CONFIG_MAIN
#include <catch.hpp>


TEST_CASE("Minimize effort of sliding a mass TODO new interface.") {

    class SlidingMass : public mesh::OptimalControlProblem<adouble> {
        // TODO difficult... virtual void initial_guess()
        // TODO really want to declare each state variable individually, and give
        // each one a name.
        // TODO is there a better way to provide the number of states and
        // controls?
        int num_states() const override { return 2; }
        int num_controls() const override { return 1; }
        void bounds(double& initial_time, double& final_time,
                std::vector<double>& states_lower,
                std::vector<double>& states_upper,
                std::vector<double>& initial_states_lower,
                std::vector<double>& initial_states_upper,
                std::vector<double>& final_states_lower,
                std::vector<double>& final_states_upper,
                std::vector<double>& controls_lower,
                std::vector<double>& controls_upper,
                std::vector<double>& initial_controls_lower,
                std::vector<double>& initial_controls_upper,
                std::vector<double>& final_controls_lower,
                std::vector<double>& final_controls_upper) const override {
            // TODO turn into bounds on time.
            initial_time = 0.0;
            final_time = 2.0;
            states_lower           = {0, -10};
            states_upper           = {2,  10};
            initial_states_lower   = {0, 0};
            initial_states_upper   = initial_states_lower;
            final_states_lower     = {1, 0};
            final_states_upper     = final_states_lower;
            controls_lower         = {-50};
            controls_upper         = {50};
            initial_controls_lower = controls_lower;
            initial_controls_upper = controls_upper;
            final_controls_lower   = controls_lower;
            final_controls_upper   = controls_upper;
        }
        const double mass = 10.0;
        void dynamics(const std::vector<adouble>& states,
                const std::vector<adouble>& controls,
                std::vector<adouble>& derivatives) const override {
            derivatives[0] = states[1];
            derivatives[1] = controls[0] / mass;
        }
        // TODO alternate form that takes a matrix; state at every time.
        //virtual void continuous(const MatrixXd& x, MatrixXd& xdot) const = 0;
        //void endpoint_cost(const T& final_time,
        //                   const std::vector<T>& final_states) const override {

        //}
        void integral_cost(const double& /*time*/,
                const std::vector<adouble>& /*states*/,
                const std::vector<adouble>& controls,
                adouble& integrand) const override {
            integrand = controls[0] * controls[0];
        }
    };

    auto ocp = std::make_shared<SlidingMass>();
    mesh::EulerTranscription dircol(ocp);
    mesh::IpoptSolver solver(dircol);
    //// TODO no initial guess; midpoint between bounds, or 0 if no bounds?
    std::vector<double> variables;
    // TODO user should never get/want raw variables...wrap the solver
    // interface for direct collocation!
    double obj_value = solver.optimize(variables);
    std::vector<std::vector<double>> states_trajectory;
    std::vector<std::vector<double>> controls_trajectory;
    dircol.interpret_iterate(variables, states_trajectory, controls_trajectory);

    // Initial and final position.
    REQUIRE(Approx(states_trajectory.front()[0]) == 0.0);
    REQUIRE(Approx(states_trajectory.back()[0]) == 1.0);
    // Initial and final speed.
    REQUIRE(Approx(states_trajectory.front()[1]) == 0.0);
    REQUIRE(Approx(states_trajectory.back()[1]) == 0.0);
}



TEST_CASE("Minimize effort of sliding a mass.") {

    class SlidingMass : public OptimalControlProblem<adouble> {
        // TODO difficult... virtual void initial_guess()
        // TODO really want to declare each state variable individually, and give
        // each one a name.
        int num_states() const override { return 2; }
        int num_controls() const override { return 1; }
        void bounds(double& initial_time, double& final_time,
                std::vector<double>& states_lower,
                std::vector<double>& states_upper,
                std::vector<double>& initial_states_lower,
                std::vector<double>& initial_states_upper,
                std::vector<double>& final_states_lower,
                std::vector<double>& final_states_upper,
                std::vector<double>& controls_lower,
                std::vector<double>& controls_upper,
                std::vector<double>& initial_controls_lower,
                std::vector<double>& initial_controls_upper,
                std::vector<double>& final_controls_lower,
                std::vector<double>& final_controls_upper) const override {
            // TODO turn into bounds on time.
            initial_time = 0.0;
            final_time = 2.0;
            states_lower           = {0, -10};
            states_upper           = {2,  10};
            initial_states_lower   = {0, 0};
            initial_states_upper   = initial_states_lower;
            final_states_lower     = {1, 0};
            final_states_upper     = final_states_lower;
            controls_lower         = {-50};
            controls_upper         = {50};
            initial_controls_lower = controls_lower;
            initial_controls_upper = controls_upper;
            final_controls_lower   = controls_lower;
            final_controls_upper   = controls_upper;
        }
        const double mass = 10.0;
        void dynamics(const std::vector<adouble>& states,
                const std::vector<adouble>& controls,
                std::vector<adouble>& derivatives) const override {
            derivatives[0] = states[1];
            derivatives[1] = controls[0] / mass;
        }
        // TODO alternate form that takes a matrix; state at every time.
        //virtual void continuous(const MatrixXd& x, MatrixXd& xdot) const = 0;
        //void endpoint_cost(const T& final_time,
        //                   const std::vector<T>& final_states) const override {

        //}
        void integral_cost(const double& /*time*/,
                const std::vector<adouble>& /*states*/,
                const std::vector<adouble>& controls,
                adouble& integrand) const override {
            integrand = controls[0] * controls[0];
        }
    };

    // TODO user interface should not involve directly using Ipopt.
    Ipopt::SmartPtr<DirectCollocationSolver> mynlp =
            new DirectCollocationSolver();
    std::shared_ptr<OptimalControlProblem<adouble>> problem(
            new SlidingMass());
    mynlp->set_problem(problem);
    // TODO return 0;
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
    app->Options()->SetNumericValue("tol", 1e-9);
    app->Options()->SetStringValue("mu_strategy", "adaptive");
    app->Options()->SetStringValue("output_file", "ipopt.out");
    // TODO temporary:
    app->Options()->SetStringValue("derivative_test", "second-order");
    Ipopt::ApplicationReturnStatus status;
    status = app->Initialize();
    if (status != Ipopt::Solve_Succeeded) {
        printf("\n\n*** Error during initialization!\n");

    }
    status = app->OptimizeTNLP(mynlp);
    if (status == Ipopt::Solve_Succeeded) {
        printf("\n\n*** The problem solved!\n");
    } else {
        printf("\n\n*** The problem FAILED!\n");
    }
}

// double g = 9.81;

//class MyProb : public Problem {
//    void ode(const VectorXd& x, const VectorXd& u, VectorXd& xdot) const
//            override {
//        xdot[0] = x[1];
//        xdot[1] = u[0];
//        //xdot[1] = -g * sin(x[0]);
//    }
//    int num_states() const override { return 2; }
//    int num_controls() const override { return 1; }
//};

//int main() {
//    {
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
//    }
//
//    //int num_points = 100;
//    //std::unique_ptr<Problem> problem(new MyProb());
//    //const int num_states = problem->num_states();
//    //const int num_controls = problem->num_controls();
//    ////MatrixXd xdot(num_states, num_points);
//    ////MatrixXd x(num_states, num_points);
//    ////MatrixXd u(num_controls, num_points);
//
//    //// TODO forward integration.
//    //const double step_size = 0.001;
//    //const int num_steps = 10000;
//    //const double final_time = step_size * num_steps;
//
//    //VectorXd xdot(num_states);
//    //VectorXd u(num_controls);
//    //u[0] = 1.0;
//    //VectorXd initial_x(num_states);
//    //initial_x[0] = 0;
//    //initial_x[1] = 0;
//    //VectorXd current_x = initial_x;
//    //for (int itime = 0; itime < num_steps; ++itime) {
//    //    problem->ode(current_x, u/*.col(itime)*/, xdot);
//    //    current_x = current_x + step_size * xdot;
//    //    std::cout << current_x << std::endl << std::endl;
//    //}
//    //std::cout << "Final time: " << final_time << std::endl;
//
//    /*
//    for (int ipt = 0; ipt < num_points; ++ipt) {
//        VectorXd this_xdot = xdot.col(0); // TODO unnecessary copy?
//        problem->ode(x.col(ipt), u.col(ipt), this_xdot);
//        std::cout << this_xdot << std::endl;
        //problem->ode(x[ipt], xdot[ipt]);
//    }
//    */
//}
