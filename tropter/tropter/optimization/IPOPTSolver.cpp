// ----------------------------------------------------------------------------
// tropter: IPOPTSolver.cpp
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
#include "IPOPTSolver.h"
#include "Problem.h"
#include <tropter/SparsityPattern.h>
#include <tropter/Exception.hpp>
#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include <IpIpoptData.hpp>
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Ref;
using Ipopt::Index;
using Ipopt::Number;

using tropter::SparsityCoordinates;
using namespace tropter::optimization;

class IPOPTSolver::TNLP : public Ipopt::TNLP {
public:
    using Index = Ipopt::Index;
    using Number = Ipopt::Number;
    TNLP(const ProblemDecorator& problem);
    void initialize(const VectorXd& guess,
            SparsityCoordinates jacobian_sparsity,
            SparsityCoordinates hessian_sparsity);
    const Eigen::VectorXd& get_solution() const { return m_solution; }
    const double& get_optimal_objective_value() const
    {   return m_optimal_obj_value; }
    const int& get_num_iterations() const { return m_num_iterations; }
private:
    // TODO move to Problem if more than one solver would need this.
    // TODO should use fancy arguments to avoid temporaries and to exploit
    // expression templates.
//    void lagrangian(double obj_factor, const VectorXa& x,
//            const Eigen::VectorXd& lambda,
//            adouble& result) const;
    // TODO should move to Problem<adouble>
//    double trace_objective(short int tag, Index num_variables, const Number* x);
//    void trace_constraints(short int tag, Index num_variables, const Number* x,
//            Index num_constraints, Number* g);
    bool get_nlp_info(Index& num_variables, Index& num_constraints,
                      Index& num_nonzeros_jacobian, Index& num_nonzeros_hessian,
                      IndexStyleEnum& index_style) override;
    bool get_bounds_info(Index num_variables,
                         Number* x_lower, Number* x_upper,
                         Index num_constraints,
                         Number* g_lower, Number* g_upper) override;

    // z: multipliers for bound constraints on x.
    // warmstart will require giving initial values for the multipliers.
    bool get_starting_point(Index num_variables, bool init_x, Number* x,
                            bool init_z, Number* z_L, Number* z_U,
                            Index num_constraints, bool init_lambda,
                            Number* lambda) override;

    bool eval_f(Index num_variables, const Number* x, bool new_x,
                Number& obj_value) override;

    bool eval_grad_f(Index num_variables, const Number* x, bool new_x,
                     Number* grad_f) override;

    bool eval_g(Index num_variables, const Number* x, bool new_x,
                Index num_constraints, Number* g) override;

    // TODO can Ipopt do finite differencing for us?
    bool eval_jac_g(Index num_variables, const Number* x, bool new_x,
                    Index num_constraints, Index num_nonzeros_jacobian,
                    Index* iRow, Index *jCol, Number* values) override;

    bool eval_h(Index num_variables, const Number* x, bool new_x,
                Number obj_factor, Index num_constraints, const Number* lambda,
                bool new_lambda, Index num_nonzeros_hessian,
                Index* iRow, Index *jCol, Number* values) override;

    void finalize_solution(Ipopt::SolverReturn status,
                           Index num_variables,
                           const Number* x,
                           const Number* z_L, const Number* z_U,
                           Index num_constraints,
                           const Number* g, const Number* lambda,
                           Number obj_value, const Ipopt::IpoptData* ip_data,
                           Ipopt::IpoptCalculatedQuantities* ip_cq) override;
    /*
    /// This allows us to inspect intermediate iterations while solving the
    /// problem.
    virtual bool intermediate_callback(Ipopt::AlgorithmMode mode,
            Index iter, Number obj_value,
            Number inf_pr, Number inf_du,
            Number mu, Number d_norm,
            Number regularization_size,
            Number alpha_du, Number alpha_pr,
            Index ls_trials,
            const Ipopt::IpoptData* ip_data,
            Ipopt::IpoptCalculatedQuantities* ip_cq) override {
    }*/

    // Members.
//    const ProblemDecorator& m_problem;
    // TODO reconsider the type of this variable:
    const ProblemDecorator& m_problem;

    unsigned m_num_variables = std::numeric_limits<unsigned>::max();
    unsigned m_num_constraints = std::numeric_limits<unsigned>::max();

    Eigen::VectorXd m_initial_guess;
    Eigen::VectorXd m_solution;
    double m_optimal_obj_value = std::numeric_limits<double>::quiet_NaN();
    int m_num_iterations = -1;

    unsigned m_hessian_num_nonzeros = std::numeric_limits<unsigned>::max();
    SparsityCoordinates m_hessian_sparsity;
    unsigned m_jacobian_num_nonzeros = std::numeric_limits<unsigned>::max();
    SparsityCoordinates m_jacobian_sparsity;

    //double m_cached_obj_value = std::nan(nullptr);
    // TODO what about for lagrangian??
};

void IPOPTSolver::print_available_options() {
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
    app->Options()->SetStringValue("print_options_documentation", "yes");
    app->Initialize();
}

/// Based on a similar function in Simbody.
std::string convert_IPOPT_ApplicationReturnStatus_to_string(
        Ipopt::ApplicationReturnStatus status) {
    using namespace Ipopt;
    switch(status) {
    case Solve_Succeeded: return "Solve succeeded";
    case Solved_To_Acceptable_Level: return "Solved to acceptable level";
    case Infeasible_Problem_Detected: return "Infeasible problem detected";
    case Search_Direction_Becomes_Too_Small: return "Search direction becomes too small";
    case Diverging_Iterates: return "Diverging iterates";
    case User_Requested_Stop: return "User requested stop";
    case Feasible_Point_Found: return "Feasible point found";
    case Maximum_Iterations_Exceeded: return "Maximum iterations exceeded";
    case Restoration_Failed: return "Restoration failed";
    case Error_In_Step_Computation: return "Error in step computation";
    case Maximum_CpuTime_Exceeded: return "Maximum CPU time exceeded";
    case Not_Enough_Degrees_Of_Freedom: return "Not enough degrees of freedom";
    case Invalid_Problem_Definition: return "Invalid problem definition";
    case Invalid_Option: return "Invalid option";
    case Invalid_Number_Detected: return "Invalid number detected";
    case Unrecoverable_Exception: return "Unrecoverable exception";
    case NonIpopt_Exception_Thrown: return "Non-Ipopt exception thrown";
    case Insufficient_Memory: return "Insufficient memory";
    case Internal_Error: return "Internal error";
    default: return "Unrecognized IPOPT return status";
    }
}

Solution IPOPTSolver::optimize_impl(const VectorXd& guess) const {

    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
    // Set options.
    auto ipoptions = app->Options();
    ipoptions->SetStringValue("print_user_options", "yes");

    if (get_verbosity() == 0) {
        ipoptions->SetIntegerValue("print_level", 0);
    }
    if (const auto opt = get_max_iterations()) {
        ipoptions->SetIntegerValue("max_iter", opt.value());
    }
    if (const auto opt = get_convergence_tolerance()) {
        // This is based on what Simbody does.
        ipoptions->SetNumericValue("tol", opt.value());
        ipoptions->SetNumericValue("dual_inf_tol", opt.value());
        ipoptions->SetNumericValue("compl_inf_tol", opt.value());
        ipoptions->SetNumericValue("acceptable_tol", opt.value());
        ipoptions->SetNumericValue("acceptable_dual_inf_tol", opt.value());
        ipoptions->SetNumericValue("acceptable_compl_inf_tol", opt.value());
    }
    if (const auto opt = get_constraint_tolerance()) {
        // This is based on what Simbody does.
        ipoptions->SetNumericValue("constr_viol_tol", opt.value());
        ipoptions->SetNumericValue("acceptable_constr_viol_tol", opt.value());
    }

    const auto& jacobian_approx = get_jacobian_approximation();
    TROPTER_THROW_IF(jacobian_approx != "exact" &&
                     jacobian_approx != "finite-difference-values",
        "When using Ipopt, the 'jacobian_approximation' setting must be "
        "either 'exact' or 'finite-difference-values', but '%s' was "
        "provided.", jacobian_approx);
    ipoptions->SetStringValue("jacobian_approximation", jacobian_approx);

    if (const auto opt = get_hessian_approximation()) {
        const auto& value = opt.value();
        TROPTER_THROW_IF(value != "exact" && value != "limited-memory",
                "When using Ipopt, the 'hessian_approximation' setting must be "
                "either 'exact' or 'limited-memory', but '%s' was provided.",
                value);
        ipoptions->SetStringValue("hessian_approximation", value);

        TROPTER_THROW_IF(jacobian_approx == "finite-difference-values" &&
                         value == "exact",
            "The 'hessian_approximation' setting for Ipopt was set to 'exact' "
            "(i.e. computed by tropter) while the 'jacobian_approximation' "
            "setting was set to 'finite-difference-values' (i.e. computed by "
            "Ipopt). This may lead to a mismatch in derivative information, so "
            "please set 'jacobian_approximation' to 'exact' if using tropter-"
            "computed Hessian information.");
    }

    // Set advanced options.
    for (const auto& option : get_advanced_options_string()) {
        if (option.second) {
            ipoptions->SetStringValue(option.first, option.second.value());
        }
    }
    for (const auto& option : get_advanced_options_int()) {
        if (option.second) {
            ipoptions->SetIntegerValue(option.first, option.second.value());
        }
    }
    for (const auto& option : get_advanced_options_real()) {
        if (option.second) {
            ipoptions->SetNumericValue(option.first, option.second.value());
        }
    }
    std::string hes_approx_final;
    bool need_exact_hessian = true;
    if (ipoptions->GetStringValue("hessian_approximation", hes_approx_final, "")
            && hes_approx_final == "limited-memory") {
        need_exact_hessian = false;

        std::string derivative_test;
        if (ipoptions->GetStringValue("derivative_test", derivative_test, "")
            && derivative_test.find("second-order") != std::string::npos) {
            TROPTER_THROW("Cannot perform second-order derivative test if "
                    "using a limited-memory Hessian approximation.");
        }
    }
    //std::string all_options;
    //app->Options()->PrintList(all_options);
    //std::cout << all_options << std::endl;
    //app->Options()->SetStringValue("derivative_test", "first-order");
    //app->Options()->SetStringValue("linear_solver", "ma97");
    //app->Options()->SetNumericValue("tol", 1e-5);
    Ipopt::ApplicationReturnStatus status;
    // TODO give istream or data file?
    status = app->Initialize();
    TROPTER_THROW_IF(status != Ipopt::Solve_Succeeded,
            "Error during initialization");

    // Create NLP.
    // -----------
    Ipopt::SmartPtr<TNLP> nlp = new TNLP(*m_problem.get());
    // TODO avoid copying x (initial guess).
    // Determine sparsity pattern of Jacobian, Hessian, etc.
    SparsityCoordinates jacobian_sparsity;
    SparsityCoordinates hessian_sparsity;
    calc_sparsity(guess, jacobian_sparsity,
            need_exact_hessian, hessian_sparsity);
    nlp->initialize(guess, std::move(jacobian_sparsity),
            std::move(hessian_sparsity));

    // Optimize!!!
    // -----------
    status = app->OptimizeTNLP(nlp);
    Solution solution;
    solution.variables = nlp->get_solution();
    solution.objective = nlp->get_optimal_objective_value();
    if (status == Ipopt::Solve_Succeeded
            || status == Ipopt::Solved_To_Acceptable_Level
            || status == Ipopt::Feasible_Point_Found) {
        solution.success = true;
    } else {
        solution.success = false;
        // http://llvm.org/doxygen/classllvm_1_1ErrorOr.html
        // https://akrzemi1.wordpress.com/2017/07/12/your-own-error-code/
        std::cerr << "[tropter] Failed to find a solution." << std::endl;
    }
    solution.status = convert_IPOPT_ApplicationReturnStatus_to_string(status);
    solution.num_iterations = nlp->get_num_iterations();
    return solution;
}

IPOPTSolver::TNLP::TNLP(const ProblemDecorator& problem)
        : m_problem(problem)
{
    m_num_variables = m_problem.get_num_variables();
    m_num_constraints = m_problem.get_num_constraints();
}

bool IPOPTSolver::TNLP::get_nlp_info(Index& num_variables,
                                     Index& num_constraints,
                                     Index& num_nonzeros_jacobian,
                                     Index& num_nonzeros_hessian,
                                     IndexStyleEnum& index_style)
{
    num_variables = m_problem.get_num_variables();
    num_constraints = m_problem.get_num_constraints();
    num_nonzeros_jacobian = m_jacobian_num_nonzeros;
    num_nonzeros_hessian = m_hessian_num_nonzeros;
    index_style = TNLP::C_STYLE;
    return true;
}

void IPOPTSolver::TNLP::initialize(const VectorXd& guess,
        SparsityCoordinates jacobian_sparsity,
        SparsityCoordinates hessian_sparsity) {
    // TODO all of this content should be taken care of for us by
    // Problem.

    // TODO should not be storing the solution at all.
    // TODO consider giving an error if initialize()
    // is ever called twice...TNLP should be one-time use!
    assert(m_solution.size() == 0);
    //m_solution.resize(0);
    m_initial_guess = guess;
    assert(guess.size() == m_num_variables);

    m_jacobian_sparsity = std::move(jacobian_sparsity);
    m_hessian_sparsity = std::move(hessian_sparsity);

    m_jacobian_num_nonzeros = (unsigned)m_jacobian_sparsity.row.size();
    m_hessian_num_nonzeros = (unsigned)m_hessian_sparsity.row.size();
}

bool IPOPTSolver::TNLP::get_bounds_info(
        Index num_variables, Number* x_lower, Number* x_upper,
        Index num_constraints, Number* g_lower, Number* g_upper) {
    assert((unsigned)num_variables   == m_num_variables);
    assert((unsigned)num_constraints == m_num_constraints);

    // TODO pass onto subclass.
    // TODO efficient copying.

    // TODO make sure bounds have been set.
    const auto& variable_lower = m_problem.get_variable_lower_bounds();
    const auto& variable_upper = m_problem.get_variable_upper_bounds();
    assert((variable_lower.array() <= variable_upper.array()).all());
    for (Index ivar = 0; ivar < num_variables; ++ivar) {
        // TODO can get rid of this in favor of the vectorized version.
        const auto& lower = variable_lower[ivar];
        const auto& upper = variable_upper[ivar];
        assert(lower <= upper);
        x_lower[ivar] = variable_lower[ivar];
        x_upper[ivar] = variable_upper[ivar];
    }
    // TODO do not assume that there are no inequality constraints.
    const auto& constraint_lower = m_problem.get_constraint_lower_bounds();
    const auto& constraint_upper = m_problem.get_constraint_upper_bounds();
    if (    constraint_lower.size() != (unsigned)num_constraints ||
            constraint_upper.size() != (unsigned)num_constraints) {
        // TODO better error handling.
        for (Index icon = 0; icon < num_constraints; ++icon) {
            g_lower[icon] = 0;
            g_upper[icon] = 0;
        }
    } else {
        // TODO vectorized:
        // TODO turn the following into an exception message:
        // assert((constraint_lower.array() <= constraint_upper.array()).all());
        for (Index icon = 0; icon < num_constraints; ++icon) {
            const auto& lower = constraint_lower[icon];
            const auto& upper = constraint_upper[icon];
            assert(lower <= upper);
            g_lower[icon] = lower;
            g_upper[icon] = upper;
        }
    }
    return true;
}

// z: multipliers for bound constraints on x.
// warmstart will require giving initial values for the multipliers.
bool IPOPTSolver::TNLP::get_starting_point(
        Index num_variables, bool init_x, Number* x,
        bool init_z, Number* /*z_L*/, Number* /*z_U*/,
        Index num_constraints, bool init_lambda,
        Number* /*lambda*/) {
    // Must this method provide initial values for x, z, lambda?
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);
    assert((unsigned)num_constraints == m_num_constraints);
    for (Index ivar = 0; ivar < num_variables; ++ivar) {
        x[ivar] = m_initial_guess[ivar];
    }
    return true;
}

bool IPOPTSolver::TNLP::eval_f(
        Index num_variables, const Number* x, bool new_x,
        Number& obj_value) {
    assert((unsigned)num_variables == m_num_variables);
    m_problem.calc_objective(num_variables, x, new_x, obj_value);
    return true;
}

bool IPOPTSolver::TNLP::eval_grad_f(
        Index num_variables, const Number* x, bool new_x,
        Number* grad_f) {
    assert((unsigned)num_variables == m_num_variables);
    m_problem.calc_gradient(num_variables, x, new_x, grad_f);
    return true;
}

bool IPOPTSolver::TNLP::eval_g(
        Index num_variables, const Number* x, bool new_x,
        Index num_constraints, Number* g) {
    assert((unsigned)num_variables   == m_num_variables);
    assert((unsigned)num_constraints == m_num_constraints);
    //// TODO if (!num_constraints) return true;
    m_problem.calc_constraints(num_variables, x, new_x, num_constraints, g);
    return true;
}

// TODO can Ipopt do finite differencing for us?
bool IPOPTSolver::TNLP::eval_jac_g(
        Index num_variables, const Number* x, bool new_x,
        Index num_constraints, Index num_nonzeros_jacobian,
        Index* iRow, Index *jCol, Number* values) {
    assert((unsigned)num_constraints == m_num_constraints);
    // TODO if (!num_constraints) return true;
    if (values == nullptr) {
        // TODO document: provide sparsity pattern.
        assert((unsigned)num_nonzeros_jacobian == m_jacobian_num_nonzeros);
        for (Index inz = 0; inz < num_nonzeros_jacobian; ++inz) {
            iRow[inz] = m_jacobian_sparsity.row[inz];
            jCol[inz] = m_jacobian_sparsity.col[inz];
        }
        return true;
    }

    m_problem.calc_jacobian(num_variables, x, new_x, num_nonzeros_jacobian,
            values);
    return true;
}

bool IPOPTSolver::TNLP::eval_h(
        Index num_variables, const Number* x, bool new_x,
        Number obj_factor, Index num_constraints, const Number* lambda,
        bool new_lambda, Index num_nonzeros_hessian,
        Index* iRow, Index *jCol, Number* values) {
    assert((unsigned)num_nonzeros_hessian == m_hessian_num_nonzeros);
    if (values == nullptr) {
        for (Index inz = 0; inz < num_nonzeros_hessian; ++inz) {
            iRow[inz] = m_hessian_sparsity.row[inz];
            jCol[inz] = m_hessian_sparsity.col[inz];
        }
        return true;
    }

    // TODO use obj_factor here to determine what computation to do exactly.

    m_problem.calc_hessian_lagrangian(num_variables, x, new_x, obj_factor,
            num_constraints, lambda, new_lambda,
            num_nonzeros_hessian, values);
    return true;
}

void IPOPTSolver::TNLP::finalize_solution(Ipopt::SolverReturn /*status*/,
                                          Index num_variables,
                                          const Number* x,
                                          const Number* /*z_L*/, const Number* /*z_U*/,
                                          Index /*num_constraints*/,
                                          const Number* /*g*/, const Number* /*lambda*/,
                                          Number obj_value,
                                          const Ipopt::IpoptData* ip_data,
                                          Ipopt::IpoptCalculatedQuantities* /*ip_cq*/)
{
    m_solution.resize(num_variables);
    //printf("\nSolution of the primal variables, x\n");
    for (Index i = 0; i < num_variables; ++i) {
        //printf("x[%d]: %e\n", i, x[i]);
        m_solution[i] = x[i];
    }
    m_optimal_obj_value = obj_value;
    m_num_iterations = ip_data->iter_count();
    //printf("\nSolution of the bound multipliers, z_L and z_U\n");
    //for (Index i = 0; i < num_variables; ++i) {
    //    printf("z_L[%d] = %e\n", i, z_L[i]);
    //}
    //for (Index i = 0; i < num_variables; ++i) {
    //    printf("z_U[%d] = %e\n", i, z_U[i]);
    //}
    //printf("\nObjective value\n");
    //printf("f(x*) = %e\n", obj_value);
    // TODO also implement Ipopt's intermediate_() function.
}




void IPOPTSolver::get_available_options(
        std::vector<std::string>& options_string,
        std::vector<std::string>& options_int,
        std::vector<std::string>& options_real) const {
    options_string = {
            "output_file",
            "print_user_options",
            "print_options_documentation",
            "print_timing_statistics",
            "option_file_name",
            "replace_bounds",
            "skip_finalize_solution_call",
            "print_info_string",
            "inf_pr_output",
            "nlp_scaling_method",
            "fixed_variable_treatment",
            "dependency_detector",
            "dependency_detection_with_rhs",
            "honor_original_bounds",
            "check_derivatives_for_naninf",
            "jac_c_constant",
            "jac_d_constant",
            "hessian_constant",
            "bound_mult_init_method",
            "least_square_init_primal",
            "least_square_init_duals",
            "adaptive_mu_globalization",
            "adaptive_mu_restore_previous_iterate",
            "adaptive_mu_kkt_norm_type",
            "mu_strategy",
            "mu_oracle",
            "fixed_mu_oracle",
            "mu_allow_fast_monotone_decrease",
            "quality_function_norm_type",
            "quality_function_centrality",
            "quality_function_balancing_term",
            "line_search_method",
            "accept_every_trial_step",
            "alpha_for_y",
            "corrector_type",
            "skip_corr_if_neg_curv",
            "skip_corr_in_monotone_mode",
            "recalc_y",
            "constraint_violation_norm_type",
            "warm_start_init_point",
            "warm_start_same_structure",
            "warm_start_entire_iterate",
            "linear_solver",
            "linear_system_scaling",
            "linear_scaling_on_demand",
            "mehrotra_algorithm",
            "fast_step_computation",
            "neg_curv_test_reg",
            "perturb_always_cd",
            "expect_infeasible_problem",
            "start_with_resto",
            "evaluate_orig_obj_at_resto_trial",
            "derivative_test",
            "derivative_test_print_all",
            "jacobian_approximation",
            "limited_memory_aug_solver",
            "limited_memory_update_type",
            "limited_memory_initialization",
            "limited_memory_special_for_resto",
            "hessian_approximation",
            "hessian_approximation_space",
            "ma27_skip_inertia_check",
            "ma27_ignore_singularity",
            "ma57_automatic_scaling",
            "pardiso_matching_strategy",
            "pardiso_redo_symbolic_fact_only_if_inertia_wrong",
            "pardiso_repeated_perturbation_means_singular",
            "pardiso_skip_inertia_check",
            "pardiso_order",
            "pardiso_iterative"
    };
    options_int = {
            "print_level",
            "file_print_level",
            "print_frequency_iter",
            "print_frequency_time",
            "max_iter",
            "acceptable_iter",
            "num_linear_variables",
            "adaptive_mu_kkterror_red_iters",
            "quality_function_max_section_steps",
            "accept_after_max_steps",
            "watchdog_shortened_iter_trigger",
            "watchdog_trial_iter_max",
            "max_soc",
            "max_filter_resets",
            "filter_reset_trigger",
            "soc_method",
            "min_refinement_steps",
            "max_refinement_steps",
            "max_soft_resto_iters",
            "max_resto_iter",
            "derivative_test_first_index",
            "limited_memory_max_history",
            "limited_memory_max_skipping",
            "ma57_pivot_order",
            "ma57_block_size",
            "ma57_node_amalgamation",
            "ma57_small_pivot_flag",
            "pardiso_msglvl",
            "pardiso_max_iterative_refinement_steps",
            "pardiso_max_iter",
            "pardiso_iter_coarse_size",
            "pardiso_iter_max_levels",
            "pardiso_iter_max_row_fill",
            "pardiso_max_droptol_corrections",
            "mumps_permuting_scaling",
            "mumps_pivot_order",
            "mumps_scaling"
    };
    options_real = {
            "tol",
            "s_max",
            "max_cpu_time",
            "dual_inf_tol",
            "constr_viol_tol",
            "compl_inf_tol",
            "acceptable_tol",
            "acceptable_dual_inf_tol",
            "acceptable_constr_viol_tol",
            "acceptable_compl_inf_tol",
            "acceptable_obj_change_tol",
            "diverging_iterates_tol",
            "mu_target",
            "obj_scaling_factor",
            "nlp_scaling_max_gradient",
            "nlp_scaling_obj_target_gradient",
            "nlp_scaling_constr_target_gradient",
            "nlp_scaling_min_value",
            "nlp_lower_bound_inf",
            "nlp_upper_bound_inf",
            "kappa_d",
            "bound_relax_factor",
            "bound_push",
            "bound_frac",
            "slack_bound_push",
            "slack_bound_frac",
            "constr_mult_init_max",
            "bound_mult_init_val",
            "mu_max_fact",
            "mu_max",
            "mu_min",
            "adaptive_mu_kkterror_red_fact",
            "filter_margin_fact",
            "filter_max_margin",
            "adaptive_mu_monotone_init_factor",
            "mu_init",
            "barrier_tol_factor",
            "mu_linear_decrease_factor",
            "mu_superlinear_decrease_power",
            "tau_min",
            "sigma_max",
            "sigma_min",
            "quality_function_section_sigma_tol",
            "quality_function_section_qf_tol",
            "alpha_red_factor",
            "alpha_for_y_tol",
            "tiny_step_tol",
            "tiny_step_y_tol",
            "theta_max_fact",
            "theta_min_fact",
            "eta_phi",
            "delta",
            "s_phi",
            "s_theta",
            "gamma_phi",
            "gamma_theta",
            "alpha_min_frac",
            "kappa_soc",
            "obj_max_inc",
            "corrector_compl_avrg_red_fact",
            "nu_init",
            "nu_inc",
            "rho",
            "kappa_sigma",
            "recalc_y_feas_tol",
            "slack_move",
            "warm_start_bound_push",
            "warm_start_bound_frac",
            "warm_start_slack_bound_push",
            "warm_start_slack_bound_frac",
            "warm_start_mult_bound_push",
            "warm_start_mult_init_max",
            "residual_ratio_max",
            "residual_ratio_singular",
            "residual_improvement_factor",
            "neg_curv_test_tol",
            "max_hessian_perturbation",
            "min_hessian_perturbation",
            "perturb_inc_fact_first",
            "perturb_inc_fact",
            "perturb_dec_fact",
            "first_hessian_perturbation",
            "jacobian_regularization_value",
            "jacobian_regularization_exponent",
            "expect_infeasible_problem_ctol",
            "expect_infeasible_problem_ytol",
            "soft_resto_pderror_reduction_factor",
            "required_infeasibility_reduction",
            "resto_penalty_parameter",
            "resto_proximity_weight",
            "bound_mult_reset_threshold",
            "constr_mult_reset_threshold",
            "resto_failure_feasibility_threshold",
            "derivative_test_perturbation",
            "derivative_test_tol",
            "findiff_perturbation",
            "point_perturbation_radius",
            "limited_memory_init_val",
            "limited_memory_init_val_max",
            "limited_memory_init_val_min",
            "ma27_pivtol",
            "ma27_pivtolmax",
            "ma27_liw_init_factor",
            "ma27_la_init_factor",
            "ma27_meminc_factor",
            "ma57_pivtol",
            "ma57_pivtolmax",
            "ma57_pre_alloc",
            "pardiso_iter_relative_tol",
            "pardiso_iter_dropping_factor",
            "pardiso_iter_dropping_schur",
            "pardiso_iter_inverse_norm_factor",
            "mumps_pivtol",
            "mumps_pivtolmax",
            "mumps_mem_percent",
            "mumps_dep_tol",
            "ma28_pivtol",
            "warm_start_target_mu"
    };
}
