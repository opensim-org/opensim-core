
#include <tropter.h>

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
class SlidingMassNew : public tropter::OptimalControlProblemNamed<T> {
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

#if defined(MUSCOLLO_WITH_SNOPT)
TEST_CASE("Sliding mass new interface") {

    auto ocp = std::make_shared<SlidingMassNew<adouble>>();
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

    int N = solution.time.size();
    std::cout << "DEBUG solution.controls " << solution.controls << std::endl;
    // TODO is this really the correct solution?
    RowVectorXd expected = RowVectorXd::LinSpaced(N - 2, 14.6119, -14.6119);
    TROPTER_REQUIRE_EIGEN(solution.controls.middleCols(1, N - 2), expected,
     0.1);
    //RowVectorXd errors = solution.controls.rightCols(N - 1) - expected;
    //REQUIRE(Approx(errors.norm()) == 0);
}
#endif

template<typename T>
class SlidingMass : public tropter::OptimalControlProblem<T> {
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
            Ref<VectorXd> final_controls_upper,
            Ref<VectorXd> /*path_constraints_lower*/,
            Ref<VectorXd> /*path_constraints_upper*/) const override
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
    tropter::transcription::LowOrder<adouble> dircol(ocp);
    tropter::IpoptSolver solver(dircol);
    //// TODO no initial guess; midpoint between bounds, or 0 if no bounds?
    VectorXd variables;
    // TODO user should never get/want raw variables...wrap the solver
    // interface for direct collocation!
    double obj_value = solver.optimize(variables);
    tropter::OptimalControlIterate traj = dircol.deconstruct_iterate(variables);

    // Initial and final position.
    REQUIRE(Approx(traj.states(0, 0)) == 0.0);
    REQUIRE(Approx(traj.states.rightCols<1>()[0]) == 1.0);
    // Initial and final speed.
    REQUIRE(Approx(traj.states(1, 0)) == 0.0);
    REQUIRE(Approx(traj.states.rightCols<1>()[1]) == 0.0);

    const auto N = traj.time.size();
    RowVectorXd expected = RowVectorXd::LinSpaced(N - 2, 14.6119, -14.6119);
    RowVectorXd errors = traj.controls.middleCols(1, N - 2) - expected;
    TROPTER_REQUIRE_EIGEN(traj.controls.middleCols(1, N - 2), expected, 0.1);
}

