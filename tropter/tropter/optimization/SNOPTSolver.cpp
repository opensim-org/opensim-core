
#include "SNOPTSolver.h"
#include "OptimizationProblem.h"

using namespace tropter;
using Eigen::VectorXd;
using Eigen::VectorXi;

// TODO building the tropter library should not *require* snopt.
#if !defined(MUSCOLLO_WITH_SNOPT)

double SNOPTSolver::optimize_impl(VectorXd& /*variables*/) const
{
    throw std::runtime_error("SNOPT is not available.");
    return -1; // TODO return NaN.
}

#else

#include <snoptProblem.hpp>

namespace {
// TODO this global is a big no-no. I thought of using a lambda that captures
// the Proxy pointer; lambdas can be converted into C function pointers, but
// not if they capture variables (like the Proxy pointer).
// TODO another option is to derive from snoptProblemA.
// TODO another option is to pass the pointer within cu (see Drake).
std::shared_ptr<const OptimizationProblemProxy> probproxy = nullptr;
}

// TODO make this into a lambda?
void snopt_userfunction(int*   /* Status */,
        int* num_variables, double x[],
        int*   needF, int* length_F  , double  F[],
        int*   needG, int* neG, double G[],
        char*  /*    cu  */, int* /* lencu */,
        int   [] /* iu   */, int* /* leniu */,
        double[] /* ru   */, int* /* lenru */)
{

    // TODO make use of needF, needG
    bool new_variables = true; // TODO can be smarter about this.
    if (*needF > 0) {
        probproxy->objective(*num_variables, x, new_variables, F[0]);
        probproxy->constraints(*num_variables, x, new_variables,
                *length_F - 1, &F[1]);
    }
    if (*needG > 0) {
        if (*needF > 0) new_variables = false;
        // The first num_variables elements of G are the gradient.
        probproxy->gradient(*num_variables, x, new_variables, &G[0]);
        // The jacobian's nonzeros start at G[n].
        probproxy->jacobian(*num_variables, x, new_variables,
                *neG - *num_variables, &G[*num_variables]);
    }
}

double SNOPTSolver::optimize_impl(VectorXd& variables) const {

    probproxy = m_problem;

    // Allocate and initialize.
    int num_variables = m_problem->num_variables();
    // The F vector contains both the objective and constraints.
    // TODO handle the case that the user does not define an objective function.
    int length_F = 1 + m_problem->num_constraints();

    // TODO what does this do?
    VectorXi xstate = VectorXi::Zero(num_variables);
    // This is an output variable; SNOPT will fill it with the multipliers.
    // TODO unless this is a warm start?
    VectorXd xmul(num_variables);

    VectorXd F(length_F);
    // TODO what does this do?
    VectorXi Fstate = VectorXi::Zero(length_F);
    // This is an output variable; SNOPT will fill it with the multipliers.
    // TODO unless this is a warm start?
    VectorXd Fmul(length_F);

    // Variable and constraint bounds.
    // -------------------------------
    // We make a copy of these because setX takes double*, not const double*.
    auto xlow                    = m_problem->variable_lower_bounds();
    auto xupp                    = m_problem->variable_upper_bounds();
    const auto& constraint_lower = m_problem->constraint_lower_bounds();
    const auto& constraint_upper = m_problem->constraint_upper_bounds();
    // There is no bound on the objective, thus the -1e20, 1e20.
    // The `finished()` is to get rid of Eigen's "expression templates."
    VectorXd Flow = (VectorXd(length_F) << -1e20, constraint_lower).finished();
    VectorXd Fupp = (VectorXd(length_F) <<  1e20, constraint_upper).finished();
    // TODO Fstate?


    // Sparsity pattern of the Jacobian.
    // ---------------------------------
    // TODO perhaps these should give ints, not unsigneds; then we can reuse
    // the memory.
    std::vector<unsigned int> jacobian_row_indices;
    std::vector<unsigned int> jacobian_col_indices;
    // TODO do not need these:
    std::vector<unsigned int> hessian_row_indices;
    std::vector<unsigned int> hessian_col_indices;
    m_problem->sparsity(variables, jacobian_row_indices, jacobian_col_indices,
            hessian_row_indices,  hessian_col_indices);
    int jacobian_num_nonzeros = jacobian_row_indices.size();
    int length_G = num_variables + jacobian_num_nonzeros;
    int num_nonzeros_G = length_G;
    // Row indices of Jacobian G (rows correspond to "fun"ctions).
    VectorXi iGfun(length_G);
    // Column indices of Jacobian G (columns correspond to "var"iables).
    VectorXi jGvar(length_G);
    // The first row is the gradient of the objective; we assume it is dense.
    iGfun.head(num_variables).setZero();
    // In MATLAB, this would be jGvar(1:num_variables) = 0:(num_variables-1).
    jGvar.head(num_variables).setLinSpaced(num_variables, 0, num_variables - 1);
    for (int index = 0; index < jacobian_num_nonzeros; ++index) {
        // The Jacobian of the constraints is shifted down one row, since
        // the first row of G is the gradient of the objective function.
        iGfun[num_variables + index] = jacobian_row_indices[index] + 1;
        jGvar[num_variables + index] = jacobian_col_indices[index];
    }


    // Create the snoptProblemA.
    // -------------------------
    snoptProblemA snopt_prob;
    snopt_prob.setProbName("SNOPTTODO");
    snopt_prob.setPrintFile("snopt.out");

    snopt_prob.setProblemSize(num_variables, length_F);

    int    ObjRow  = 0; // The element of F that contains the objective func.
    double ObjAdd  = 0; // A constant to add to the objective for reporting.
    snopt_prob.setObjective(ObjRow, ObjAdd);

    // Memory related to the variables (unknowns).
    snopt_prob.setX(variables.data(), xlow.data(), xupp.data(),
            xmul.data(), xstate.data());

    // Memory related to the objective and constraint values.
    snopt_prob.setF(F.data(), Flow.data(), Fupp.data(),
            Fmul.data(), Fstate.data());
    // TODO linear portion of F. Can we omit this?
    // TODO for our generic problems, we cannot provide this.
    int lenA = 1; int neA = 0;
    VectorXi iAfun(lenA); VectorXi jAvar(lenA); VectorXd A;
    snopt_prob.setA(lenA, neA, iAfun.data(), jAvar.data(), A.data());
    // For some reason, SNOPT allows the length of the iGfun and jGvar arrays
    // to be greater than the number of nonzero elements.
    snopt_prob.setG(length_G, num_nonzeros_G, iGfun.data(), jGvar.data());

    // This function computes the objective and constraints (defined above).
    snopt_prob.setUserFun(snopt_userfunction);

    // TODO call snJac().
    // snopta will compute the Jacobian by finite-differences.
    // The user has the option of calling  snJac  to define the
    // coordinate arrays (iAfun,jAvar,A) and (iGfun, jGvar).
    snopt_prob.setIntParameter("Derivative option", 0 /* TODO 1 */);
    if (m_max_iterations != -1) { // TODO untested.
        snopt_prob.setIntParameter("Iterations", m_max_iterations);
    }
    snopt_prob.setIntParameter("Verify level ", 3);

    // Solve the problem.
    // snJac is called implicitly in this case to compute the Jacobian.
    int Cold = 0 /*, Basis = 1, Warm = 2 */;
    snopt_prob.solve(Cold);

    return F[0];
}

#endif // !defined(MUSCOLLO_WITH_SNOPT)
