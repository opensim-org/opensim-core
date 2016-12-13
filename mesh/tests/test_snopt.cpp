
#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <mesh.h>

#include <snoptProblem.hpp>

using namespace mesh;
using Eigen::VectorXd;
using Eigen::RowVectorXd;
using Eigen::Vector2d;
using Eigen::MatrixXd;
using Eigen::Ref;

// This example is taken from
// https://github.com/snopt/snopt-interface/blob/master/cppexamples/sntoya.cpp
void toyusrf_(int*   /* Status */, int* /* n     */, double      x[],
              int*   /* needF  */, int* /* neF   */, double      F[],
              int*   /* needG  */, int* /* neG   */, double[] /* G */,
              char*  /*    cu  */, int* /* lencu */,
              int   [] /* iu   */, int* /* leniu */,
              double[] /* ru   */, int* /* lenru */)
{
    //==================================================================
    // Computes the nonlinear objective and constraint terms for the toy
    // problem featured in the SnoptA users guide.
    // neF = 3, n = 2.
    //
    //   Minimize     x(2)
    //
    //   subject to   x(1)**2      + 4 x(2)**2  <= 4,
    //               (x(1) - 2)**2 +   x(2)**2  <= 5,
    //                x(1) >= 0.
    //
    //==================================================================
    F[0] =  x[1];
    F[1] =  x[0]*x[0] + 4*x[1]*x[1];
    F[2] = (x[0] - 2)*(x[0] - 2) + x[1]*x[1];
}

// TODO also test the variant with an exact Jacobian.
TEST_CASE("SNOPT sntoyA example problem; Jacobian not provided.") {

    snoptProblemA ToyProb;

    // Allocate and initialize;
    int n     =  2;
    int neF   =  3;

    double *x      = new double[n];
    double *xlow   = new double[n];
    double *xupp   = new double[n];
    double *xmul   = new double[n];
    int    *xstate = new    int[n];

    double *F      = new double[neF];
    double *Flow   = new double[neF];
    double *Fupp   = new double[neF];
    double *Fmul   = new double[neF];
    int    *Fstate = new int[neF];

    int    ObjRow  = 0;
    double ObjAdd  = 0;

    int Cold = 0, Basis = 1, Warm = 2;


    // Set the upper and lower bounds.
    xlow[0]   =  0.0;  xlow[1]   = -1e20;
    xupp[0]   = 1e20;  xupp[1]   =  1e20;
    xstate[0] =    0;  xstate[1] =  0;

    Flow[0] = -1e20; Flow[1] = -1e20; Flow[2] = -1e20;
    Fupp[0] =  1e20; Fupp[1] =   4.0; Fupp[2] =  5.0;
    x[0]    = 1.0;
    x[1]    = 1.0;


    // Load the data for ToyProb ...
    ToyProb.setProbName   ("Toy0");
    ToyProb.setPrintFile  ( "Toy0.out" );

    ToyProb.setProblemSize( n, neF );
    ToyProb.setObjective  ( ObjRow, ObjAdd );
    ToyProb.setX          ( x, xlow, xupp, xmul, xstate );
    ToyProb.setF          ( F, Flow, Fupp, Fmul, Fstate );

    ToyProb.setUserFun    ( toyusrf_ );


    // snopta will compute the Jacobian by finite-differences.
    // The user has the option of calling  snJac  to define the
    // coordinate arrays (iAfun,jAvar,A) and (iGfun, jGvar).
    ToyProb.setIntParameter( "Derivative option", 0 );
    ToyProb.setIntParameter( "Verify level ", 3 );


    // Solve the problem.
    // snJac is called implicitly in this case to compute the Jacobian.
    ToyProb.solve( Cold );

    for (int i = 0; i < n; i++ ){
        std::cout << "x = " << x[i] << " xstate = " << xstate[i] << std::endl;
    }
    for (int i = 0; i < neF; i++ ){
        std::cout << "F = " << F[i] << " Fstate = " << Fstate[i] << std::endl;
    }
}
//==================================================================
// Computes the nonlinear objective and constraint terms for the toy
// problem featured in the SnoptA users guide.
// neF = 3, n = 2.
//
//   Minimize     x(2)
//
//   subject to   x(1)**2      + 4 x(2)**2  <= 4,
//               (x(1) - 2)**2 +   x(2)**2  <= 5,
//                x(1) >= 0.
//
//==================================================================


TEST_CASE("SNOPT and ADOL-C on SnoptA (sntoyA) example") {
    class SnoptA : public OptimizationProblem<adouble> {
    public:
        SnoptA() : OptimizationProblem(2, 2)
        {
            // TODO support an "infinity"
            set_variable_bounds(Vector2d(0, -1e20), Vector2d(1e20, 1e20));
            set_constraint_bounds(Vector2d(-1e20, -1e20), Vector2d(4, 5));
        }
        void objective(const VectorXa& x, adouble& obj_value) const override
        {
            obj_value = x[1];
        }
        void constraints(const VectorXa& x,
                Eigen::Ref<VectorXa> constr) const override
        {
            constr[0] = x[0]*x[0] + 4*x[1]*x[1];
            constr[1] = (x[0] - 2)*(x[0] - 2) + x[1]*x[1];
        }
    };

    SnoptA problem;
    SNOPTSolver solver(problem);
    VectorXd variables = Vector2d(2, 2);
    double obj_value = solver.optimize(variables);
    REQUIRE(Approx(variables[0]) == 0);
    REQUIRE(Approx(variables[1]) == -1);
    REQUIRE(Approx(obj_value)    == -1);
}


//TEST_CASE("First order minimum effort.") {
//
//    /// minimize     integral_{t=0}^{t=1) u(t)^2 dt
//    /// subject to   xdot = -2x + u
//    ///              x(0) = 1
//    ///              x(1) = 0
//    class FirstOrderMinEffort : public mesh::OptimalControlProblem<adouble> {
//        int num_states() const override { return 1; }
//        int num_controls() const override { return 1; }
//        void bounds(double& initial_time, double& final_time,
//                Ref<VectorXd> states_lower,
//                Ref<VectorXd> states_upper,
//                Ref<VectorXd> initial_states_lower,
//                Ref<VectorXd> initial_states_upper,
//                Ref<VectorXd> final_states_upper,
//                Ref<VectorXd> final_states_lower,
//                Ref<VectorXd> controls_lower,
//                Ref<VectorXd> controls_upper,
//                Ref<VectorXd> initial_controls_lower,
//                Ref<VectorXd> initial_controls_upper,
//                Ref<VectorXd> final_controls_lower,
//                Ref<VectorXd> final_controls_upper) const override
//        {
//            // TODO turn into bounds on time.
//            initial_time = 0.0;
//            final_time = 1.0;
//            states_lower           = VectorXd::Constant(1, -50);
//            states_upper           = VectorXd::Constant(1,  50);
//            initial_states_lower   = VectorXd::Constant(1, 1);
//            initial_states_upper   = initial_states_lower;
//            final_states_lower     = VectorXd::Constant(1, 0);
//            final_states_upper     = final_states_lower;
//            controls_lower         = VectorXd::Constant(1, -100);
//            controls_upper         = VectorXd::Constant(1,  100);
//            initial_controls_lower = controls_lower;
//            initial_controls_upper = controls_upper;
//            final_controls_lower   = controls_lower;
//            final_controls_upper   = controls_upper;
//        }
//        void dynamics(const VectorXa& states,
//                const VectorXa& controls,
//                Ref<VectorXa> derivatives) const override
//        {
//            derivatives[0] = -2 * states[0] + controls[0];
//        }
//        void integral_cost(const double& /*time*/,
//                const VectorXa& /*states*/,
//                const VectorXa& controls,
//                adouble& integrand) const override {
//            integrand = controls[0] * controls[0];
//        }
//    };
//
//    auto ocp = std::make_shared<FirstOrderMinEffort>();
//    mesh::EulerTranscription dircol(ocp, 100);
//    mesh::SNOPTSolver solver(dircol);
//    VectorXd variables;
//    // TODO user should never get/want raw variables...wrap the solver
//    // interface for direct collocation!
//    double obj_value = solver.optimize(variables);
//    EulerTranscription::Trajectory traj = dircol.interpret_iterate(variables);
//
//    std::cout << traj.controls.transpose() << std::endl;
//    // Initial and final states satisfy constraints.
//    REQUIRE(Approx(traj.states(0, 0)) == 1.0);
//    REQUIRE(Approx(traj.states.rightCols<1>()[0]) == 0.0);
//    std::cout << traj.states << std::endl;
//
//    // u*(t) = -4 exp(2t) / (e^4 - 1)
//    const double factor = -4 / (std::exp(4) - 1);
//    RowVectorXd expected = factor * (2 * traj.time).array().exp();
//    std::cout << "DEBUG" << std::endl;
//    std::cout << expected.transpose() << std::endl;
//    RowVectorXd errors = traj.controls - expected;
//    REQUIRE(Approx(errors.norm()) == 0);
//}



TEST_CASE("Sliding mass optimal control with SNOPT.") {

    class SlidingMass : public mesh::OptimalControlProblem<adouble> {
        // TODO difficult... virtual void initial_guess()
        // TODO really want to declare each state variable individually, and give
        // each one a name.
        // TODO is there a better way to provide the number of states and
        // controls?
        int num_states() const override { return 2; }
        int num_controls() const override { return 1; }
        void bounds(double& initial_time, double& final_time,
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
            initial_time = 0.0;
            final_time = 2.0;
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
        void dynamics(const VectorXa& states,
                const VectorXa& controls,
                Ref<VectorXa> derivatives) const override
        {
            derivatives[0] = states[1];
            derivatives[1] = controls[0] / mass;
        }
        // TODO alternate form that takes a matrix; state at every time.
        //virtual void continuous(const MatrixXd& x, MatrixXd& xdot) const = 0;
        //void endpoint_cost(const T& final_time,
        //                   const std::vector<T>& final_states) const override {

        //}
        void integral_cost(const double& /*time*/,
                const VectorXa& /*states*/,
                const VectorXa& controls,
                adouble& integrand) const override {
            integrand = controls[0] * controls[0];
        }
    };

    auto ocp = std::make_shared<SlidingMass>();
    mesh::EulerTranscription<adouble> dircol(ocp);
    mesh::SNOPTSolver solver(dircol);
    //// TODO no initial guess; midpoint between bounds, or 0 if no bounds?
    VectorXd variables;
    // TODO user should never get/want raw variables...wrap the solver
    // interface for direct collocation!
    double obj_value = solver.optimize(variables);
    auto traj = dircol.interpret_iterate(variables);

    // Initial and final position.
    REQUIRE(Approx(traj.states(0, 0)) == 0.0);
    REQUIRE(Approx(traj.states.rightCols<1>()[0]) == 1.0);
    // Initial and final speed.
    REQUIRE(Approx(traj.states(1, 0)) == 0.0);
    REQUIRE(Approx(traj.states.rightCols<1>()[1]) == 0.0);

    int N = traj.time.size();
    RowVectorXd expected = RowVectorXd::LinSpaced(N-1, 14.25, -14.25);
    RowVectorXd errors = traj.controls.rightCols(N-1) - expected;
    REQUIRE(Approx(errors.norm()) == 0);
}

