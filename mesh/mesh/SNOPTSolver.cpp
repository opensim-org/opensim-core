
#include "SNOPTSolver.h"
#include "OptimizationProblem.h"

// TODO building the mesh library should not *require* snopt.
#include <snoptProblem.hpp>

using namespace mesh;
using Eigen::VectorXd;

// TODO this is a big no-no:
std::shared_ptr<const OptimizationProblemProxy> probproxy = nullptr;

// TODO make this into a lambda?
void snopt_userfunction(int*   /* Status */,
        int* num_variables, double x[],
        int*   /* needF  */, int* neF  , double      F[],
        int*   /* needG  */, int* /* neG   */, double[] /* G */,
        char*  /*    cu  */, int* /* lencu */,
        int   [] /* iu   */, int* /* leniu */,
        double[] /* ru   */, int* /* lenru */)
{

    // TODO make use of needF, needG
    const bool new_variables = true;
    probproxy->objective(*num_variables, x, new_variables, F[0]);
    probproxy->constraints(*num_variables, x, new_variables, *neF - 1, &F[1]);
}

double SNOPTSolver::optimize_impl(VectorXd& variables) const {

    if (variables.size() == 0) {
        variables = VectorXd::Zero(m_problem->num_variables());
    }

    probproxy = m_problem;

    snoptProblemA snopt_prob;

    // Allocate and initialize;
    int n     =  m_problem->num_variables();
    int neF   =  1 + m_problem->num_constraints();

    // TODO directly use the VectorXd's here.
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

    int Cold = 0 /*, Basis = 1, Warm = 2 */;

    const auto& variable_lower   = m_problem->variable_lower_bounds();
    const auto& variable_upper   = m_problem->variable_upper_bounds();
    const auto& constraint_lower = m_problem->constraint_lower_bounds();
    const auto& constraint_upper = m_problem->constraint_upper_bounds();
    for (int ivar = 0; ivar < n; ++ivar) {
        xlow[ivar] = variable_lower[ivar];
        xupp[ivar] = variable_upper[ivar];
        xstate[ivar] = 0;
        x[ivar] = variables[ivar];
    }
    Flow[0] = -1e20;
    Fupp[0] =  1e20;
    for (int iF = 1; iF < neF; ++iF) {
        Flow[iF] = constraint_lower[iF - 1];
        Fupp[iF] = constraint_upper[iF - 1];
        // TODO Fstate? Fmul?
    }


    // Load the data for snopt_prob ...
    snopt_prob.setProbName   ("SNOPTTODO");
    snopt_prob.setPrintFile  ("snopt.out");

    snopt_prob.setProblemSize( n, neF );
    snopt_prob.setObjective  ( ObjRow, ObjAdd );
    snopt_prob.setX          ( x, xlow, xupp, xmul, xstate );
    snopt_prob.setF          ( F, Flow, Fupp, Fmul, Fstate );

    snopt_prob.setUserFun    ( snopt_userfunction );


    // snopta will compute the Jacobian by finite-differences.
    // The user has the option of calling  snJac  to define the
    // coordinate arrays (iAfun,jAvar,A) and (iGfun, jGvar).
    snopt_prob.setIntParameter( "Derivative option", 0 );
    snopt_prob.setIntParameter( "Verify level ", 3 );


    // Solve the problem.
    // snJac is called implicitly in this case to compute the Jacobian.
    snopt_prob.solve( Cold );

    // TODO don't need an intermediate x variable; directly use variables with
    // snopt.
    for (unsigned ivar = 0; ivar < variables.size(); ++ivar) {
        variables[ivar] = x[ivar];
    }

    return F[0];
}
