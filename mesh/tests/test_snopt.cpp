
#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <mesh.h>

#include <snoptProblem.hpp>

using namespace mesh;
using Eigen::VectorXd;
using Eigen::Vector2d;

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
    REQUIRE(variables[0] == 0);
    REQUIRE(Approx(variables[1]) == -1);
    REQUIRE(Approx(obj_value)    == -1);
}




