
#include <mesh.h>
#include <legacy.h>

#define CATCH_CONFIG_MAIN
#include <catch.hpp>

using Eigen::Ref;
using Eigen::VectorXd;
using Eigen::RowVectorXd;
using Eigen::Vector2d;
using Eigen::MatrixXd;

using namespace mesh;

template<typename T>
class SlidingMassNew : public mesh::OptimalControlProblemNamed<T> {
public:
    SlidingMassNew() {
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
    void dynamics(const VectorX<T>& states,
            const VectorX<T>& controls,
            Ref<VectorX<T>> derivatives) const override
    {
        derivatives[0] = states[1];
        derivatives[1] = controls[0] / mass;
    }
    // TODO alternate form that takes a matrix; state at every time.
    //virtual void continuous(const MatrixXd& x, MatrixXd& xdot) const = 0;
    void integral_cost(const T& /*time*/,
            const VectorX<T>& /*states*/,
            const VectorX<T>& controls,
            adouble& integrand) const override {
        integrand = controls[0] * controls[0];
    }
};

TEST_CASE("Sliding mass new interface") {

    auto ocp = std::make_shared<SlidingMassNew<adouble>>();
    DirectCollocationSolver<adouble> dircol(ocp, "euler", "ipopt");
    OptimalControlSolution solution = dircol.solve();
    solution.write("sliding_mass_solution.csv");
    //OptimalControlIterate initial_guess = ocp->make_guess_template();
    //OptimalControlSolution solution = dircol.solve(initial_guess);

    //auto ocp = std::make_shared<SlidingMassNew<adouble>>();
    //EulerTranscription transcription;
    //DirectCollocationSolver dircol(ocp, transcription);
    //mesh::transcription::LowOrder<adouble> dircol(ocp);
    //mesh::IpoptSolver solver(dircol);
    //// TODO no initial guess; midpoint between bounds, or 0 if no bounds?
    //VectorXd variables;
    //// TODO user should never get/want raw variables...wrap the solver
    //// interface for direct collocation!
    //double obj_value = solver.optimize(variables);
    //using Trajectory = transcription::LowOrder<adouble>::Trajectory;
    //Trajectory traj = dircol.interpret_iterate(variables);

    // Initial and final position.
    REQUIRE(Approx(solution.states(0, 0)) == 0.0);
    REQUIRE(Approx(solution.states.rightCols<1>()[0]) == 1.0);
    // Initial and final speed.
    REQUIRE(Approx(solution.states(1, 0)) == 0.0);
    REQUIRE(Approx(solution.states.rightCols<1>()[1]) == 0.0);

    int N = solution.time.size();
    std::cout << "DEBUG solution.controls " << solution.controls << std::endl;
    RowVectorXd expected = RowVectorXd::LinSpaced(N - 1, 14.25, -14.25);
    RowVectorXd errors = solution.controls.rightCols(N - 1) - expected;
    REQUIRE(Approx(errors.norm()) == 0);
}

template<typename T>
class SlidingMass : public mesh::OptimalControlProblem<T> {
    // TODO difficult... virtual void initial_guess()
    // TODO really want to declare each state variable individually, and give
    // each one a name.
    // TODO is there a better way to provide the number of states and
    // controls?
    int num_states() const override { return 2; }
    int num_controls() const override { return 1; }
    void bounds(double& initial_time_lower, double& initial_time_upper,
            double& final_time_lower, double& final_time_upper,
            Ref<VectorXd> states_lower,
            Ref<VectorXd> states_upper,
            Ref<VectorXd> initial_states_lower,
            Ref<VectorXd> initial_states_upper,
            Ref<VectorXd> final_states_upper,
            Ref<VectorXd> final_states_lower,
            Ref<VectorXd> controls_lower,
            Ref<VectorXd> controls_upper,
            Ref<VectorXd> initial_controls_lower,
            Ref<VectorXd> initial_controls_upper,
            Ref<VectorXd> final_controls_lower,
            Ref<VectorXd> final_controls_upper) const override
    {
        // TODO turn into bounds on time.
        initial_time_lower = 0.0;
        initial_time_upper = 0.0;
        final_time_lower = 2.0;
        final_time_upper = 2.0;
        states_lower           = Vector2d(0, -10);
        states_upper           = Vector2d(2,  10);
        initial_states_lower   = Vector2d(0, 0);
        initial_states_upper   = initial_states_lower;
        final_states_lower     = Vector2d(1, 0);
        final_states_upper     = final_states_lower;
        controls_lower         = VectorXd::Constant(1, -50);
        controls_upper         = VectorXd::Constant(1,  50);
        initial_controls_lower = controls_lower;
        initial_controls_upper = controls_upper;
        final_controls_lower   = controls_lower;
        final_controls_upper   = controls_upper;
    }
    const double mass = 10.0;
    void dynamics(const VectorX<T>& states,
            const VectorX<T>& controls,
            Ref<VectorX<T>> derivatives) const override
    {
        derivatives[0] = states[1];
        derivatives[1] = controls[0] / mass;
    }
    // TODO alternate form that takes a matrix; state at every time.
    //virtual void continuous(const MatrixXd& x, MatrixXd& xdot) const = 0;
    //void endpoint_cost(const T& final_time,
    //                   const std::vector<T>& final_states) const override {

    //}
    void integral_cost(const T& /*time*/,
            const VectorX<T>& /*states*/,
            const VectorX<T>& controls,
            adouble& integrand) const override {
        integrand = controls[0] * controls[0];
    }
};

TEST_CASE("Minimize effort of sliding a mass TODO new interface.") {

    auto ocp = std::make_shared<SlidingMass<adouble>>();
    //EulerTranscription transcription;
    //DirectCollocationSolver dircol(ocp, transcription);
    mesh::transcription::LowOrder<adouble> dircol(ocp);
    mesh::IpoptSolver solver(dircol);
    //// TODO no initial guess; midpoint between bounds, or 0 if no bounds?
    VectorXd variables;
    // TODO user should never get/want raw variables...wrap the solver
    // interface for direct collocation!
    double obj_value = solver.optimize(variables);
    using Trajectory = transcription::LowOrder<adouble>::Trajectory;
    Trajectory traj = dircol.interpret_iterate(variables);

    // Initial and final position.
    REQUIRE(Approx(traj.states(0, 0)) == 0.0);
    REQUIRE(Approx(traj.states.rightCols<1>()[0]) == 1.0);
    // Initial and final speed.
    REQUIRE(Approx(traj.states(1, 0)) == 0.0);
    REQUIRE(Approx(traj.states.rightCols<1>()[1]) == 0.0);

    int N = traj.time.size();
    RowVectorXd expected = RowVectorXd::LinSpaced(N - 1, 14.25, -14.25);
    RowVectorXd errors = traj.controls.rightCols(N - 1) - expected;
    REQUIRE(Approx(errors.norm()) == 0);
}



TEST_CASE("Minimize effort of sliding a mass. LEGACY") {

    class SlidingMass : public legacy::OptimalControlProblem<adouble> {
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
    Ipopt::SmartPtr<legacy::DirectCollocationSolver> mynlp =
            new legacy::DirectCollocationSolver();
    std::shared_ptr<legacy::OptimalControlProblem<adouble>> problem(
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
