// ----------------------------------------------------------------------------
// tropter: SNOPTSolver.cpp
// ----------------------------------------------------------------------------
// Copyright (c) 2017 tropter authors
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may
// not use this file except in compliance with the License. You may obtain a
// copy of the License at http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------

#include "SNOPTSolver.h"
#include "OptimizationProblem.h"

using namespace tropter;
using Eigen::VectorXd;
using Eigen::VectorXi;

// TODO building the tropter library should not *require* snopt.
#if !defined(TROPTER_WITH_SNOPT)

OptimizationSolution
SNOPTSolver::optimize_impl(const VectorXd& /*variables*/) const
{
    throw std::runtime_error("SNOPT is not available.");
    return OptimizationSolution(); // TODO return NaN.
}

#else

#include <snoptProblem.hpp>

namespace {
// TODO this global is a big no-no. I thought of using a lambda that captures
// the Decorator pointer; lambdas can be converted into C function pointers, but
// not if they capture variables (like the Decorator pointer).
// TODO another option is to derive from snoptProblemA.
// TODO another option is to pass the pointer within cu (see Drake).
const OptimizationProblemDecorator* probproxy = nullptr;

std::string convert_info_integer_to_string(int info) {
    switch(info) {
    case 1: return "Finished successfully: optimality conditions satisfied";
    case 2: return "Finished successfully: feasible point found";
    case 3: return "Finished successfully: requested accuracy could not be achieved";
    case 11: return "The problem appears to be infeasible: infeasible linear constraints";
    case 12: return "The problem appears to be infeasible: infeasible linear equalities";
    case 13: return "The problem appears to be infeasible: nonlinear infeasibilities minimized";
    case 14: return "The problem appears to be infeasible: infeasibilities minimized";
    case 21: return "The problem appears to be unbounded: unbounded objective";
    case 22: return "The problem appears to be unbounded: constraint violation limit reached";
    case 31: return "Resource limit error: iteration limit reached";
    case 32: return "Resource limit error: major iteration limit reached";
    case 33: return "Resource limit error: the superbasics limit is too small";
    case 41: return "Terminated after numerical diffculties: current point cannot be improved";
    case 42: return "Terminated after numerical diffculties: singular basis";
    case 43: return "Terminated after numerical diffculties: cannot satisfy the general constraints";
    case 44: return "Terminated after numerical diffculties: ill-conditioned null-space basis";
    case 51: return "Error in the user-supplied functions: incorrect objective derivatives";
    case 52: return "Error in the user-supplied functions: incorrect constraint derivatives";
    case 61: return "Undefined user-supplied functions: undefined function at the first feasible point";
    case 62: return "Undefined user-supplied functions: undefined function at the initial point";
    case 63: return "Undefined user-supplied functions: unable to proceed into undefined region";
    case 71: return "User requested termination: terminated during function evaluation";
    case 74: return "User requested termination: terminated from monitor routine";
    case 81: return "Insufficient storage allocated: work arrays must have at least 500 elements";
    case 82: return "Insufficient storage allocated: not enough character storage";
    case 83: return "Insufficient storage allocated: not enough integer storage";
    case 84: return "Insufficient storage allocated: not enough real storage";
    case 91: return "Input arguments out of range: invalid input argument";
    case 92: return "Input arguments out of range: basis file dimensions do not match this problem";
    case 141: return "System error: wrong number of basic variables";
    case 142: return "System error: error in basis package";
    default: return "Unrecognized SNOPT return status";
    }
}
}

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
        probproxy->calc_objective(*num_variables, x, new_variables, F[0]);
        probproxy->calc_constraints(*num_variables, x, new_variables,
                *length_F - 1, &F[1]);
    }
    if (*needG > 0) {
        if (*needF > 0) new_variables = false;
        // The first num_variables elements of G are the gradient.
        probproxy->calc_gradient(*num_variables, x, new_variables, &G[0]);
        // The jacobian's nonzeros start at G[n].
        probproxy->calc_jacobian(*num_variables, x, new_variables,
                *neG - *num_variables, &G[*num_variables]);
    }
}

OptimizationSolution
SNOPTSolver::optimize_impl(const VectorXd& variablesArg) const {

    VectorXd variables(variablesArg);

    probproxy = m_problem.get();

    // Allocate and initialize.
    int num_variables = m_problem->get_num_variables();
    // The F vector contains both the objective and constraints.
    // TODO handle the case that the user does not define an objective function.
    int length_F = 1 + m_problem->get_num_constraints();

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
    auto xlow                    = m_problem->get_variable_lower_bounds();
    auto xupp                    = m_problem->get_variable_upper_bounds();
    const auto& constraint_lower = m_problem->get_constraint_lower_bounds();
    const auto& constraint_upper = m_problem->get_constraint_upper_bounds();
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
    m_problem->calc_sparsity(variables,
            jacobian_row_indices, jacobian_col_indices, false,
            hessian_row_indices,  hessian_col_indices);
    assert(hessian_row_indices.size() == 0);
    assert(hessian_col_indices.size() == 0);
    int jacobian_num_nonzeros = (int)jacobian_row_indices.size();
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
    if (const auto opt = get_max_iterations()) { // TODO untested
        snopt_prob.setIntParameter("Iterations", opt.value());
    }
    snopt_prob.setIntParameter("Verify level ", 3);

    // TODO string options?
    for (const auto& option : get_advanced_options_int()) {
        if (option.second) {
            snopt_prob.setIntParameter(option.first.c_str(),
                    option.second.value());
        }
    }
    for (const auto& option : get_advanced_options_real()) {
        if (option.second) {
            snopt_prob.setRealParameter(option.first.c_str(),
                    option.second.value());
        }
    }

    // Solve the problem.
    // snJac is called implicitly in this case to compute the Jacobian.
    int Cold = 0 /*, Basis = 1, Warm = 2 */;
    int info = snopt_prob.solve(Cold);

    OptimizationSolution solution;
    solution.variables = variables;
    solution.objective = F[0];
    solution.status = convert_info_integer_to_string(info);
    solution.success = (info == 1 || info == 2 || info == 3);
    // TODO set number of iterations: solution.num_iterations.
    return solution;
}


#endif // !defined(TROPTER_WITH_SNOPT)
