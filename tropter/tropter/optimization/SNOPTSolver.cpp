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
#include "Problem.h"
#include <tropter/SparsityPattern.h>

using namespace tropter::optimization;
using Eigen::VectorXd;
using Eigen::VectorXi;

// TODO building the tropter library should not *require* snopt.
#if !defined(TROPTER_WITH_SNOPT)

Solution
SNOPTSolver::optimize_impl(const VectorXd& /*variables*/) const
{
    throw std::runtime_error("SNOPT is not available.");
    return Solution(); // TODO return NaN.
}

#else

#include <snoptProblem.hpp>

namespace {
// TODO this global is a big no-no. I thought of using a lambda that captures
// the Decorator pointer; lambdas can be converted into C function pointers, but
// not if they capture variables (like the Decorator pointer).
// TODO another option is to derive from snoptProblemA.
// TODO another option is to pass the pointer within cu (see Drake).
const ProblemDecorator* probproxy = nullptr;

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
        int*   /* needG */, int* /* neG */, double /* G */[],
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

    //if (*needG > 0) {
    //    if (*needF > 0) new_variables = false;
    //    // The first num_variables elements of G are the gradient.
    //    probproxy->calc_gradient(*num_variables, x, new_variables, &G[0]);
    //    // The jacobian's nonzeros start at G[n].
    //    probproxy->calc_jacobian(*num_variables, x, new_variables,
    //            *neG - *num_variables, &G[*num_variables]);
    //}
}

Solution
SNOPTSolver::optimize_impl(const VectorXd& variablesArg) const {

    VectorXd variables(variablesArg);
    probproxy = m_problem.get();

    // Allocate and initialize.
    int num_variables = m_problem->get_num_variables();

    // Initial "states" for variables x. This lets you specify if you think the
    // the optimal value for a variable will probably be one of its bounds.
    // By default, set all states to 0, which does not make this assumption, or
    // in SNOPT's terminology, "all variables will be eligible for the initial
    // basis."
    VectorXi xstate = VectorXi::Zero(num_variables);
    // This is an output variable; SNOPT will fill it with the multipliers.
    // TODO unless this is a warm start?
    VectorXd xmul(num_variables);
    // The F vector contains both the objective and constraints.
    // F[0] = objective, F[1:end] = constraints.
    // TODO handle the case that the user does not define an objective function.
    int length_F = 1 + m_problem->get_num_constraints();
    VectorXd F(length_F);
    // Initial "states" for problem function F. This lets you specify if you
    // think the optimal value for a row will probably be one of its bounds.
    // By default, set all states to 0, which does not make this assumption, or
    // in SNOPT's terminology, "all rows of F will be eligible for the initial
    // basis."
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

    // Derivative information.
    // -----------------------
    // TODO SNOPT's derivative checker throws an exception in certain problems 
    // for errors in the tropter-computed Jacobian. For now, the SNOPT-
    // computed Jacobian is recommended.

    SparsityCoordinates jacobian_sparsity;
    SparsityCoordinates hessian_sparsity;
    calc_sparsity(variables,
            jacobian_sparsity, false,
            hessian_sparsity);
    int jacobian_num_nonzeros = (int)jacobian_sparsity.row.size();
    int neG = num_variables + jacobian_num_nonzeros;

    // Row indices of Jacobian G (rows correspond to "fun"ctions).
    VectorXi iGfun(neG);
    // Column indices of Jacobian G (columns correspond to "var"iables).
    VectorXi jGvar(neG);
    // The first row is the gradient of the objective; we assume it is dense.
    iGfun.head(num_variables).setZero();
    // In MATLAB, this would be jGvar(1:num_variables) = 0:num_variables-1.
    jGvar.head(num_variables).setLinSpaced(num_variables, 0, num_variables-1);

    for (int index = 0; index < jacobian_num_nonzeros; ++index) {
        // The Jacobian of the constraints is shifted down one row, since
        // the first row of G is the gradient of the objective function.
        iGfun[num_variables + index] = jacobian_sparsity.row[index] + 1;
        jGvar[num_variables + index] = jacobian_sparsity.col[index];
    }

    // TODO linear portion of F. Can we omit this?
    // TODO for our generic problems, we cannot provide this.
    int lenA = 1; int neA = 0;
    VectorXi iAfun(lenA); VectorXi jAvar(lenA); VectorXd A;
    // For some reason, SNOPT allows the length of the iGfun and jGvar arrays
    // to be greater than the number of nonzero elements.

    // Create the snoptProblemA.
    // -------------------------
    snoptProblemA snopt_prob;
    snopt_prob.initialize("snopt.out", 1);
    snopt_prob.setProbName(""); // TODO create problem name

    // Main solver settings.
    if (const auto opt = get_max_iterations()) { // TODO untested
        snopt_prob.setIntParameter("Iterations", opt.value());
    }
    if (const auto opt = get_convergence_tolerance()) { // TODO untested
        snopt_prob.setRealParameter("Major optimality tolerance", opt.value());
    }
    if (const auto opt = get_constraint_tolerance()) { // TODO untested
        snopt_prob.setRealParameter("Major feasibility tolerance", opt.value());
    }
    
    // Derivative settings.
    snopt_prob.setIntParameter("Derivative option", 0 /* TODO 1 */);
    snopt_prob.setIntParameter("Verify level", 3);

    const auto& jacobian_approx = get_jacobian_approximation();
    TROPTER_THROW_IF(jacobian_approx != "exact" &&
        jacobian_approx != "finite-difference-values",
        "When using SNOPT, the 'jacobian_approximation' setting must be "
        "either 'exact' or 'finite-difference-values', but '%s' was "
        "provided.", jacobian_approx);

    // Advanced settings.
    // TODO try QPSolver Cholesky setting, which stores the reduced Hessian
    // a Cholesky factorization approach. Could speed things up for moderately
    // sized problems (# variables < 1000).
    for (const auto& option : get_advanced_options_string()) {
        if (option.second) {
            std::string param = format("%s %s", option.first.c_str(), 
                option.second.value().c_str());
            snopt_prob.setParameter(param.c_str());
        }
    }
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
    // ------------------
    int      Cold = 0  /*, Basis = 1, Warm = 2 */;
    int    ObjRow = 0; // The objective is the first row of F.
    double ObjAdd = 0; // A constant to add to the objective for reporting.
    int            nS; // Final number of superbasic variables ("free" variables)
    int          nInf = -1; // Final number of infeasibilities
    double       sInf = -1; // Final sum of infeasibilities
    int          info = -1; // Output status.

    if (jacobian_approx == "finite-difference-values") {
        // snJac is called implicitly in this case to compute the Jacobian.
        info = snopt_prob.solve(Cold, length_F, num_variables, ObjAdd,
            ObjRow, snopt_userfunction,
            xlow.data(), xupp.data(), Flow.data(), Fupp.data(),
            variables.data(), xstate.data(), xmul.data(),
            F.data(), Fstate.data(), Fmul.data(),
            nS, nInf, sInf);

    } else if (jacobian_approx == "exact") {

        TROPTER_THROW("User-supplied derivatives currently not supported for "
            "SNOPT");

        // When solving problems while providing derivative infomration from 
        // tropter, SNOPT sometime exits with error 52: "incorrect constraint 
        // derivatives", but only for problems with muscles. This may suggest a 
        // bug in our own Jacobian derivative calculations. Why only for 
        // muscles?

        // Use this form of solve() if providing Jacobian information.
        //info = snopt_prob.solve(Cold, length_F, num_variables, ObjAdd,
        //    ObjRow, snopt_userfunction,
        //    iAfun.data(), jAvar.data(), A.data(), neA,
        //    iGfun.data(), jGvar.data(), neG,
        //    xlow.data(), xupp.data(), Flow.data(), Fupp.data(),
        //    variables.data(), xstate.data(), xmul.data(),
        //    F.data(), Fstate.data(), Fmul.data(),
        //    nS, nInf, sInf);
    }


    
    // Output problem information.
    // ---------------------------
    std::cout << std::endl;
    std::cout << "Final number of superbasic variables: " << nInf << std::endl;
    std::cout << "Final number of infeasibilities: " << nInf << std::endl;
    std::cout << "Final sum of infeasibilities: " << sInf << std::endl;

    // Return solution.
    // ----------------
    Solution solution;
    solution.variables = variables;
    solution.objective = F[0];
    solution.status = convert_info_integer_to_string(info);
    solution.success = (info == 1 || info == 2 || info == 3);
    // TODO set number of iterations: solution.num_iterations.
    return solution;
}


#endif // !defined(TROPTER_WITH_SNOPT)
