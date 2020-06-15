#ifndef TROPTER_OPTIMALCONTROL_TRANSCRIPTION_TRAPEZOIDAL_HPP
#define TROPTER_OPTIMALCONTROL_TRANSCRIPTION_TRAPEZOIDAL_HPP
// ----------------------------------------------------------------------------
// tropter: Trapezoidal.hpp
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

#include "Trapezoidal.h"
#include <iomanip>

#include <tropter/Exception.hpp>
#include <tropter/SparsityPattern.h>

namespace tropter {
namespace transcription {

template <typename T>
void Trapezoidal<T>::set_ocproblem(std::shared_ptr<const OCProblem> ocproblem) {
    m_ocproblem = ocproblem;
    m_num_states = m_ocproblem->get_num_states();
    m_num_controls = m_ocproblem->get_num_controls();
    m_num_adjuncts = m_ocproblem->get_num_adjuncts();
    TROPTER_THROW_IF(m_ocproblem->get_num_diffuses(),
            "Trapezoidal transcription does not support diffuse variables.");
    m_num_continuous_variables = m_num_states + m_num_controls + m_num_adjuncts;
    m_num_time_variables = 2;
    m_num_parameters = m_ocproblem->get_num_parameters();
    m_num_dense_variables = m_num_time_variables + m_num_parameters;
    m_num_mesh_points = (int)m_mesh.size();
    m_num_mesh_intervals = m_num_mesh_points - 1;
    int num_variables = m_num_time_variables + m_num_parameters +
                        m_num_mesh_points * m_num_continuous_variables;
    this->set_num_variables(num_variables);
    m_num_defects = m_num_states ? m_num_mesh_intervals : 0;
    m_num_dynamics_constraints = m_num_defects * m_num_states;
    m_num_path_constraints = m_ocproblem->get_num_path_constraints();
    // TODO rename..."total_path_constraints"?
    int num_path_traj_constraints = m_num_mesh_points * m_num_path_constraints;
    int num_constraints =
            m_num_dynamics_constraints + num_path_traj_constraints;
    this->set_num_constraints(num_constraints);

    // Variable and constraint names.
    // ------------------------------
    m_variable_names.clear();
    m_variable_names.emplace_back("initial_time");
    m_variable_names.emplace_back("final_time");

    const auto param_names = m_ocproblem->get_parameter_names();
    for (const auto& param_name : param_names)
        m_variable_names.push_back(param_name);

    // For padding, count the number of digits in num_mesh_points.
    int num_digits_max_mesh_index = 0;
    {
        // The printed index goes up to m_num_mesh_intervals.
        int max_index = m_num_mesh_intervals;
        while (max_index != 0) {
            max_index /= 10;
            num_digits_max_mesh_index++;
        }
    }
    const auto state_names = m_ocproblem->get_state_names();
    const auto control_names = m_ocproblem->get_control_names();
    const auto adjunct_names = m_ocproblem->get_adjunct_names();
    for (int i_mesh = 0; i_mesh < m_num_mesh_points; ++i_mesh) {
        for (const auto& state_name : state_names) {
            std::stringstream ss;
            ss << state_name << "_" << std::setfill('0')
               << std::setw(num_digits_max_mesh_index) << i_mesh;
            m_variable_names.push_back(ss.str());
        }
        for (const auto& control_name : control_names) {
            std::stringstream ss;
            ss << control_name << "_" << std::setfill('0')
               << std::setw(num_digits_max_mesh_index) << i_mesh;
            m_variable_names.push_back(ss.str());
        }
        for (const auto& adjunct_name : adjunct_names) {
            std::stringstream ss;
            ss << adjunct_name << "_" << std::setfill('0')
               << std::setw(num_digits_max_mesh_index) << i_mesh;
            m_variable_names.push_back(ss.str());
        }
    }

    m_constraint_names.clear();
    // Start at index 1 (counting mesh intervals; not mesh points).
    for (int i_mesh = 1; i_mesh < m_num_mesh_points; ++i_mesh) {
        for (const auto& state_name : state_names) {
            std::stringstream ss;
            ss << state_name << "_" << std::setfill('0')
               << std::setw(num_digits_max_mesh_index) << i_mesh;
            m_constraint_names.push_back(ss.str());
        }
    }
    const auto path_constraint_names = m_ocproblem->get_path_constraint_names();
    for (int i_mesh = 0; i_mesh < m_num_mesh_points; ++i_mesh) {
        for (const auto& path_constraint_name : path_constraint_names) {
            std::stringstream ss;
            ss << path_constraint_name << "_" << std::setfill('0')
               << std::setw(num_digits_max_mesh_index) << i_mesh;
            m_constraint_names.push_back(ss.str());
        }
    }

    // Bounds.
    // -------
    double initial_time_lower;
    double initial_time_upper;
    double final_time_lower;
    double final_time_upper;
    using Eigen::VectorXd;
    VectorXd states_lower(m_num_states);
    VectorXd states_upper(m_num_states);
    VectorXd initial_states_lower(m_num_states);
    VectorXd initial_states_upper(m_num_states);
    VectorXd final_states_lower(m_num_states);
    VectorXd final_states_upper(m_num_states);
    VectorXd controls_lower(m_num_controls);
    VectorXd controls_upper(m_num_controls);
    VectorXd initial_controls_lower(m_num_controls);
    VectorXd initial_controls_upper(m_num_controls);
    VectorXd final_controls_lower(m_num_controls);
    VectorXd final_controls_upper(m_num_controls);
    VectorXd adjuncts_lower(m_num_adjuncts);
    VectorXd adjuncts_upper(m_num_adjuncts);
    VectorXd initial_adjuncts_lower(m_num_adjuncts);
    VectorXd initial_adjuncts_upper(m_num_adjuncts);
    VectorXd final_adjuncts_lower(m_num_adjuncts);
    VectorXd final_adjuncts_upper(m_num_adjuncts);
    VectorXd diffuses_lower; // empty
    VectorXd diffuses_upper; // empty
    VectorXd parameters_upper(m_num_parameters);
    VectorXd parameters_lower(m_num_parameters);
    VectorXd path_constraints_lower(m_num_path_constraints);
    VectorXd path_constraints_upper(m_num_path_constraints);
    m_ocproblem->get_all_bounds(initial_time_lower, initial_time_upper,
            final_time_lower, final_time_upper, states_lower, states_upper,
            initial_states_lower, initial_states_upper, final_states_lower,
            final_states_upper, controls_lower, controls_upper,
            initial_controls_lower, initial_controls_upper,
            final_controls_lower, final_controls_upper, adjuncts_lower,
            adjuncts_upper, initial_adjuncts_lower, initial_adjuncts_upper,
            final_adjuncts_lower, final_adjuncts_upper, diffuses_lower,
            diffuses_upper, parameters_lower, parameters_upper,
            path_constraints_lower, path_constraints_upper);
    // TODO validate sizes.
    // Bounds on variables.
    VectorXd variable_lower(num_variables);
    variable_lower << initial_time_lower, final_time_lower, parameters_lower,
            initial_states_lower, initial_controls_lower,
            initial_adjuncts_lower,
            (VectorXd(m_num_continuous_variables) << states_lower,
                    controls_lower, adjuncts_lower)
                    .finished()
                    .replicate(m_num_mesh_points - 2, 1),
            final_states_lower, final_controls_lower, final_adjuncts_lower;
    VectorXd variable_upper(num_variables);
    variable_upper << initial_time_upper, final_time_upper, parameters_upper,
            initial_states_upper, initial_controls_upper,
            initial_adjuncts_upper,
            (VectorXd(m_num_continuous_variables) << states_upper,
                    controls_upper, adjuncts_upper)
                    .finished()
                    .replicate(m_num_mesh_points - 2, 1),
            final_states_upper, final_controls_upper, final_adjuncts_upper;
    this->set_variable_bounds(variable_lower, variable_upper);
    // Bounds for constraints.
    VectorXd constraint_lower(num_constraints);
    VectorXd constraint_upper(num_constraints);
    // Defects must be 0.
    VectorXd dynamics_bounds = VectorXd::Zero(m_num_dynamics_constraints);
    VectorXd path_constraints_traj_lower =
            path_constraints_lower.replicate(m_num_mesh_points, 1);
    VectorXd path_constraints_traj_upper =
            path_constraints_upper.replicate(m_num_mesh_points, 1);
    constraint_lower << dynamics_bounds, path_constraints_traj_lower;
    constraint_upper << dynamics_bounds, path_constraints_traj_upper;
    this->set_constraint_bounds(constraint_lower, constraint_upper);
    // TODO won't work if the bounds don't include zero!
    // TODO set_initial_guess(std::vector<double>(num_variables)); // TODO user
    // input

    // Set the mesh.
    // -------------
    // For integrating the integral cost.
    // The duration of each mesh interval.
    m_mesh_eigen = Eigen::Map<VectorXd>(m_mesh.data(), m_mesh.size());
    m_mesh_intervals = m_mesh_eigen.tail(m_num_mesh_intervals) -
                       m_mesh_eigen.head(m_num_mesh_intervals);
    m_trapezoidal_quadrature_coefficients = VectorXd::Zero(m_num_mesh_points);
    // Betts 2010 equation 4.195, page 169.
    // b = 0.5 * [tau0, tau0 + tau1, tau1 + tau2, ..., tauM-2 + tauM-1, tauM-1]
    m_trapezoidal_quadrature_coefficients.head(m_num_mesh_intervals) =
            0.5 * m_mesh_intervals;
    m_trapezoidal_quadrature_coefficients.tail(m_num_mesh_intervals) +=
            0.5 * m_mesh_intervals;

    // Allocate working memory.
    m_integrand.resize(m_num_mesh_points);
    m_derivs.resize(m_num_states, m_num_mesh_points);

    m_ocproblem->initialize_on_mesh(m_mesh_eigen);
}

template <typename T>
void Trapezoidal<T>::calc_objective(const VectorX<T>& x, T& obj_value) const {
    // TODO move this to a "make_variables_view()"
    const T& initial_time = x[0];
    const T& final_time = x[1];
    const T duration = final_time - initial_time;

    // TODO I don't actually need to make a new view each time; just change the
    // data pointer. TODO probably don't even need to update the data pointer!
    auto states = make_states_trajectory_view(x);
    auto controls = make_controls_trajectory_view(x);
    auto adjuncts = make_adjuncts_trajectory_view(x);
    auto parameters = make_parameters_view(x);

    // Initialize on iterate.
    // ----------------------
    m_ocproblem->initialize_on_iterate(parameters);

    for (int i_cost = 0; i_cost < m_ocproblem->get_num_costs(); ++i_cost) {
        // Compute integral.
        // -----------------
        T integral = 0;
        if (m_ocproblem->get_cost_requires_integral(i_cost)) {
            m_integrand.setZero();
            for (int i_mesh = 0; i_mesh < m_num_mesh_points; ++i_mesh) {
                const T time = duration * m_mesh[i_mesh] + initial_time;
                m_ocproblem->calc_cost_integrand(i_cost,
                        {i_mesh, time, states.col(i_mesh), controls.col(i_mesh),
                                adjuncts.col(i_mesh), m_empty_diffuse_col,
                                parameters},
                        m_integrand[i_mesh]);
            }

            for (int i_mesh = 0; i_mesh < m_num_mesh_points; ++i_mesh) {
                integral += m_trapezoidal_quadrature_coefficients[i_mesh] *
                            m_integrand[i_mesh];
            }
            // The quadrature coefficients are fractions of the duration;
            // multiply by duration to get the correct units.
            integral *= duration;
        } else {
            integral = std::numeric_limits<T>::quiet_NaN();
        }

        // Compute cost.
        // -------------
        T cost = 0;
        m_ocproblem->calc_cost(i_cost,
                {0, initial_time, states.leftCols(1), controls.leftCols(1),
                        adjuncts.leftCols(1), m_num_mesh_points - 1, final_time,
                        states.rightCols(1), controls.rightCols(1),
                        adjuncts.rightCols(1), parameters, integral},
                cost);

        obj_value += cost;
    }
}

template <typename T>
void Trapezoidal<T>::calc_constraints(
        const VectorX<T>& x, Eigen::Ref<VectorX<T>> constraints) const {
    // TODO parallelize.
    const T& initial_time = x[0];
    const T& final_time = x[1];
    const T duration = final_time - initial_time;
    auto states = make_states_trajectory_view(x);
    auto controls = make_controls_trajectory_view(x);
    auto adjuncts = make_adjuncts_trajectory_view(x);
    auto parameters = make_parameters_view(x);

    // Initialize on iterate.
    // ======================
    m_ocproblem->initialize_on_iterate(parameters);

    // Organize the constraints vector.
    ConstraintsView constr_view = make_constraints_view(constraints);

    // Dynamics and path constraints.
    // ==============================
    // "Continuous function"

    // Obtain state derivatives at each mesh point.
    // --------------------------------------------
    // TODO storing 1 too many derivatives trajectory; don't need the first
    // xdot (at t0). (TODO I don't think this is true anymore).
    // TODO tradeoff between memory and parallelism.
    for (int i_mesh = 0; i_mesh < m_num_mesh_points; ++i_mesh) {
        // TODO should pass the time.
        const T time = duration * m_mesh[i_mesh] + initial_time;
        m_ocproblem->calc_differential_algebraic_equations(
                {i_mesh, time, states.col(i_mesh), controls.col(i_mesh),
                        adjuncts.col(i_mesh), m_empty_diffuse_col, parameters},
                {m_derivs.col(i_mesh),
                        constr_view.path_constraints.col(i_mesh)});
    }

    // Compute constraint defects.
    // ---------------------------
    // Backwards Euler (not used here):
    // defect_i = x_i - (x_{i-1} + h * xdot_i)  for i = 1, ..., N.
    if (m_num_defects) {
        const unsigned N = m_num_mesh_points;
        const auto& x_i = states.rightCols(N - 1);
        const auto& x_im1 = states.leftCols(N - 1);
        const auto& xdot_i = m_derivs.rightCols(N - 1);
        const auto& xdot_im1 = m_derivs.leftCols(N - 1);
        for (int i_mesh = 0; i_mesh < (int)N - 1; ++i_mesh) {
            const auto& h = duration * m_mesh_intervals[i_mesh];
            const auto f = T(0.5) * (xdot_i.col(i_mesh) + xdot_im1.col(i_mesh));
            constr_view.defects.col(i_mesh) =
                    x_i.col(i_mesh) - (x_im1.col(i_mesh) + h * f);
        }
    }
}

template <typename T>
void Trapezoidal<T>::calc_sparsity_hessian_lagrangian(const Eigen::VectorXd& x,
        SymmetricSparsityPattern& hescon_sparsity,
        SymmetricSparsityPattern& hesobj_sparsity) const {
    const auto& num_variables = this->get_num_variables();

    const auto& num_con_vars = m_num_continuous_variables;

    // TODO provide option for assuming dense (conservative)! To avoid trying to
    // get an aggressive sparsity pattern; this is necessary if contact has
    // if-statements.

    // Hessian of constraints.
    // -----------------------
    // The first two rows of the Hessian contain partial derivatives with
    // initial_time and final_time. We assume time is coupled to all other
    // variables.
    // TODO can be smarter about interaction of variables with time; implicit
    // formulations (see test_double_pendulum) give additional sparsity here.
    for (int irow = 0; irow < m_num_dense_variables; ++irow) {
        for (int icol = irow; icol < (int)num_variables; ++icol) {
            hescon_sparsity.set_nonzero(irow, icol);
        }
    }

    SymmetricSparsityPattern dae_sparsity(m_num_continuous_variables);
    if (this->get_exact_hessian_block_sparsity_mode() == "sparse") {
        // The Hessian of sum_i lambda_i * constraint_i over constraints i has a
        // certain structure as a result of the direct collocation formulation.
        // The diagonal contains the same repeated square block of dimensions
        // num_continuous_variables.
        // We estimate the sparsity of this block by combining the sparsity from
        // constraint_i for i covering the defects at mesh point 0 and the
        // path constraints at mesh point 0.
        // Note that defect constraint i actually depends on mesh points i and i
        // + 1. However, since the sparsity pattern repeats for each mesh point,
        // we can "ignore" the dependence on mesh point i + 1.

        // This function evaluates the DAE at the mesh point 0, and returns a
        // single DAE derivative or path constraint.
        std::function<T(const VectorX<T>&, int)> calc_dae =
                [this, &x](const VectorX<T>& vars, int idx) {
                    T t = x[0]; // initial time.
                    VectorX<T> s = vars.head(m_num_states);
                    VectorX<T> c = vars.segment(m_num_states, m_num_controls);
                    VectorX<T> a = vars.tail(m_num_adjuncts);
                    VectorX<T> d; // empty
                    VectorX<T> p =
                            x.segment(m_num_time_variables, m_num_parameters)
                                    .template cast<T>();
                    VectorX<T> deriv(m_num_states);
                    VectorX<T> path(m_num_path_constraints);
                    m_ocproblem->calc_differential_algebraic_equations(
                            {0, t, s, c, a, d, p}, {deriv, path});
                    return idx < m_num_states ? deriv[idx]
                                              : path[idx - m_num_states];
                };
        for (int i = 0; i < (m_num_states + m_num_path_constraints); ++i) {
            // Create a function for a specific derivative or path constraint.
            std::function<T(const VectorX<T>&)> calc_dae_i =
                    std::bind(calc_dae, std::placeholders::_1, i);
            // Determine the sparsity for this specific derivative/path
            // constraint.
            auto block_sparsity = calc_hessian_sparsity_with_perturbation(
                    x.segment(
                            m_num_dense_variables, m_num_continuous_variables),
                    calc_dae_i);
            // Add in this sparsity to the block that we'll repeat.
            dae_sparsity.add_in_nonzeros(block_sparsity);
        }
    } else if (this->get_exact_hessian_block_sparsity_mode() == "dense") {
        dae_sparsity.set_dense();
    }

    // Repeat the block down the diagonal of the Hessian of constraints.
    for (int imesh = 0; imesh < m_num_mesh_points; ++imesh) {
        const auto istart = m_num_dense_variables + imesh * num_con_vars;
        hescon_sparsity.set_nonzero_block(istart, dae_sparsity);
    }

    // Hessian of objective.
    // ---------------------
    // Assume time and parameters are coupled to all other variables.
    // TODO not necessarily; detect this sparsity.
    for (int irow = 0; irow < m_num_dense_variables; ++irow) {
        for (int icol = irow; icol < (int)num_variables; ++icol) {
            hesobj_sparsity.set_nonzero(irow, icol);
        }
    }

    // To get the sparsity of the objective, combine the sparsity of each cost
    // term.
    for (int icost = 0; icost < m_ocproblem->get_num_costs(); ++icost) {
        if (m_ocproblem->get_cost_requires_integral(icost)) {
            SymmetricSparsityPattern integral_cost_sparsity(num_con_vars);
            if (this->get_exact_hessian_block_sparsity_mode() == "sparse") {
                // Despite our attempt to correctly implementing the sparse
                // sparsity detection for costs, we disable this functionality
                // until we add adequate tests.
                TROPTER_THROW(
                        "Automatic sparsity detection for Hessian diagonal "
                        "blocks not implemented for Trapezoidal "
                        "transcription.");
                // Integral cost depends on states and controls at all times.
                // Determine how the integrand depends on the state and control
                // at mesh point 0, then repeat this block down the diagonal.
                std::function<T(const VectorX<T>&)> calc_cost_integrand =
                        [this, icost, &x](const VectorX<T>& vars) {
                            T t = x[0]; // initial time.
                            VectorX<T> s = vars.head(m_num_states);
                            VectorX<T> c =
                                    vars.segment(m_num_states, m_num_controls);
                            VectorX<T> a = vars.tail(m_num_adjuncts);
                            VectorX<T> d; // empty
                            VectorX<T> p = x.segment(m_num_time_variables,
                                                    m_num_parameters)
                                                   .template cast<T>();
                            T integrand = 0;
                            m_ocproblem->calc_cost_integrand(
                                    icost, {0, t, s, c, a, d, p}, integrand);
                            return integrand;
                        };
                integral_cost_sparsity =
                        calc_hessian_sparsity_with_perturbation(
                                // Grab the first state, first controls and
                                // first adjuncts.
                                x.segment(m_num_dense_variables, num_con_vars),
                                calc_cost_integrand);
            } else if (this->get_exact_hessian_block_sparsity_mode() ==
                       "dense") {
                integral_cost_sparsity.set_dense();
            }

            // Repeat the block down the diagonal of the Hessian of the
            // objective.
            for (int imesh = 0; imesh < m_num_mesh_points; ++imesh) {
                const auto istart =
                        m_num_dense_variables + imesh * num_con_vars;
                hesobj_sparsity.set_nonzero_block(
                        istart, integral_cost_sparsity);
            }
        }

        // The cost depends on the initial state/controls, final state/controls,
        // and the integral if it exists.
        SymmetricSparsityPattern cost_initial_sparsity(num_con_vars);
        SymmetricSparsityPattern cost_final_sparsity(num_con_vars);
        const auto lastmeshstart =
                m_num_dense_variables + (m_num_mesh_points - 1) * num_con_vars;
        if (this->get_exact_hessian_block_sparsity_mode() == "sparse") {
            auto st = make_states_trajectory_view(x);
            auto ct = make_controls_trajectory_view(x);
            auto at = make_adjuncts_trajectory_view(x);
            // This function evaluates the cost using a dummy integral value
            // and states/controls from the provided x. But only either the
            // initial OR final states/controls are perturbed. The non-perturbed
            // values come directly from x.
            std::function<T(const VectorX<T>&, bool)> calc_cost =
                    [this, icost, &x, &st, &ct, &at](
                            const VectorX<T>& vars, bool initial) {
                        // TODO see if endpoint cost actually depends on
                        // time; put it in vars.
                        T it = x[0]; // initial time.
                        T ft = x[1]; // final time.
                        VectorX<T> is;
                        VectorX<T> ic;
                        VectorX<T> ia;
                        VectorX<T> fs;
                        VectorX<T> fc;
                        VectorX<T> fa;
                        VectorX<T> p = x.segment(m_num_time_variables,
                                                m_num_parameters)
                                               .template cast<T>();
                        if (initial) {
                            is = vars.head(m_num_states);
                            ic = vars.segment(m_num_states, m_num_controls);
                            ia = vars.tail(m_num_adjuncts);
                            fs = st.rightCols(1).template cast<T>();
                            fc = ct.rightCols(1).template cast<T>();
                            fa = at.rightCols(1).template cast<T>();
                        } else {
                            fs = st.leftCols(1).template cast<T>();
                            fc = ct.leftCols(1).template cast<T>();
                            fa = at.leftCols(1).template cast<T>();
                            fs = vars.head(m_num_states);
                            fc = vars.segment(m_num_states, m_num_controls);
                            fa = vars.tail(m_num_adjuncts);
                        }
                        T cost = 0;
                        // TODO: Should ideally be the actual integral computed
                        // at x, but that requires using the quadrature
                        // coefficients.
                        T integral = 0.0;
                        m_ocproblem->calc_cost(icost,
                                {0, it, is, ic, ia, m_num_mesh_points - 1, ft,
                                        fs, fc, fa, p, integral},
                                cost);
                        return cost;
                    };
            // This is a function of the initial states/controls, and the
            // final states/controls come from x.
            std::function<T(const VectorX<T>&)> calc_cost_perturb_initial =
                    std::bind(calc_cost, std::placeholders::_1, true);
            cost_initial_sparsity = calc_hessian_sparsity_with_perturbation(
                    // Grab the initial mesh point continuous variables.
                    x.segment(
                            m_num_dense_variables, m_num_continuous_variables),
                    calc_cost_perturb_initial);
            // This is a function of the final states/controls, and the
            // initial states/controls come from x.
            std::function<T(const VectorX<T>&)> calc_cost_perturb_final =
                    std::bind(calc_cost, std::placeholders::_1, false);
            cost_final_sparsity = calc_hessian_sparsity_with_perturbation(
                    // Grab the final mesh point continuous variables.
                    x.segment(lastmeshstart, m_num_continuous_variables),
                    calc_cost_perturb_final);
        } else if (this->get_exact_hessian_block_sparsity_mode() == "dense") {
            cost_initial_sparsity.set_dense();
            cost_final_sparsity.set_dense();
        }
        hesobj_sparsity.set_nonzero_block(
                m_num_dense_variables, cost_initial_sparsity);
        hesobj_sparsity.set_nonzero_block(lastmeshstart, cost_final_sparsity);
    }

    // TODO most objectives do *NOT* depend on time; should time actually
    // affect hesobj for most problems?
}

template <typename T>
std::vector<std::string> Trapezoidal<T>::get_variable_names() const {
    return m_variable_names;
}

template <typename T>
std::vector<std::string> Trapezoidal<T>::get_constraint_names() const {
    return m_constraint_names;
}

template <typename T>
Eigen::VectorXd Trapezoidal<T>::construct_iterate(
        const Iterate& traj, bool interpolate) const {
    // Check for errors with dimensions.
    // ---------------------------------
    // TODO move some of this to Iterate::validate().
    // Check rows.
    TROPTER_THROW_IF(traj.states.rows() != m_num_states,
            "Expected states to have %i row(s), but it has %i.", m_num_states,
            traj.states.rows());
    TROPTER_THROW_IF(traj.controls.rows() != m_num_controls,
            "Expected controls to have %i row(s), but it has %i.",
            m_num_controls, traj.controls.rows());
    TROPTER_THROW_IF(traj.adjuncts.rows() != m_num_adjuncts,
            "Expected adjuncts to have %i row(s), but it has %i.",
            m_num_adjuncts, traj.adjuncts.rows());
    TROPTER_THROW_IF(traj.parameters.rows() != m_num_parameters,
            "Expected parameters to have %i element(s), but it has %i.",
            m_num_parameters, traj.parameters.size());
    TROPTER_THROW_IF(traj.diffuses.rows(),
            "Trapezoidal transcription does not support diffuse variables.");
    // Check columns.
    if (interpolate) {
        // If interpolating, only check that non-empty matrices have the same
        // number of columns as elements in time vector.
        if (traj.states.cols()) {
            TROPTER_THROW_IF(traj.time.size() != traj.states.cols(),
                    "Expected time and states to have the same number of "
                    "columns, but they have %i and %i column(s), "
                    "respectively.",
                    traj.time.size(), traj.states.cols());
        }
        if (traj.controls.cols()) {
            TROPTER_THROW_IF(traj.time.size() != traj.controls.cols(),
                    "Expected time and controls to have the same number of "
                    "columns, but they have %i and %i column(s), "
                    "respectively.",
                    traj.time.size(), traj.controls.cols());
        }
        if (traj.adjuncts.cols()) {
            TROPTER_THROW_IF(traj.time.size() != traj.adjuncts.cols(),
                    "Expected time and adjuncts to have the same number of "
                    "columns, but they have %i and %i column(s), "
                    "respectively.",
                    traj.time.size(), traj.adjuncts.cols());
        }
    } else {
        TROPTER_THROW_IF(traj.time.size() != m_num_mesh_points,
                "Expected time to have %i element(s), but it has %i.",
                m_num_mesh_points, traj.time.size());
        TROPTER_THROW_IF(traj.states.cols() != m_num_mesh_points,
                "Expected states to have %i column(s), but it has %i.",
                m_num_mesh_points, traj.states.cols());
        TROPTER_THROW_IF(traj.controls.cols() != m_num_mesh_points,
                "Expected controls to have %i column(s), but it has %i.",
                m_num_mesh_points, traj.controls.cols());
        TROPTER_THROW_IF(traj.adjuncts.cols() != m_num_mesh_points,
                "Expected adjuncts to have %i column(s), but it has %i.",
                m_num_mesh_points, traj.adjuncts.cols());
    }

    // Interpolate the trajectory, as it might have a different number of mesh
    // points than m_num_mesh_points.
    Iterate traj_interp;
    const Iterate* traj_to_use;
    if (interpolate) {
        const auto duration = traj.time.tail<1>()[0] - traj.time[0];
        traj_interp = traj.interpolate(
                duration * m_mesh_eigen.array() + traj.time[0]);
        traj_to_use = &traj_interp;
    } else {
        traj_to_use = &traj;
    }

    // TODO reorder columns !!!!!

    Eigen::VectorXd iterate(this->get_num_variables());
    // Initial and final time.
    iterate[0] = traj_to_use->time[0];
    iterate[1] = traj_to_use->time.tail<1>()[0];
    // Create mutable views. This will probably fail miserably if the
    // dimensions do not match.
    this->make_states_trajectory_view(iterate) = traj_to_use->states;
    this->make_controls_trajectory_view(iterate) = traj_to_use->controls;
    if (traj_to_use->adjuncts.cols())
        this->make_adjuncts_trajectory_view(iterate) = traj_to_use->adjuncts;
    if (traj_to_use->parameters.size())
        this->make_parameters_view(iterate) = traj_to_use->parameters;

    return iterate;
}

template <typename T>
Iterate Trapezoidal<T>::deconstruct_iterate(const Eigen::VectorXd& x) const {
    // TODO move time variables to the end.
    const double& initial_time = x[0];
    const double& final_time = x[1];
    Iterate traj;
    traj.time =
            (final_time - initial_time) * m_mesh_eigen.array() + initial_time;

    traj.states = this->make_states_trajectory_view(x);
    traj.controls = this->make_controls_trajectory_view(x);
    traj.adjuncts = this->make_adjuncts_trajectory_view(x);
    traj.parameters = this->make_parameters_view(x);

    traj.state_names = m_ocproblem->get_state_names();
    traj.control_names = m_ocproblem->get_control_names();
    traj.adjunct_names = m_ocproblem->get_adjunct_names();
    traj.parameter_names = m_ocproblem->get_parameter_names();

    return traj;
}

template <typename T>
void Trapezoidal<T>::print_constraint_values(
        const Iterate& ocp_vars, std::ostream& stream) const {
    // TODO also print_bounds() information.
    // TODO allow passing an option for only showing when bounds are
    // violated, not simply active.
    // TODO allow passing a threshold to see if a value is within range of a
    // bound.

    // We want to be able to restore the stream's original formatting.
    StreamFormat streamFormat(stream);

    // Gather and organize all constraint values and bounds.
    VectorX<T> vars = construct_iterate(ocp_vars).template cast<T>();
    VectorX<T> constraint_values(this->get_num_constraints());
    calc_constraints(vars, constraint_values);
    ConstraintsView values = make_constraints_view(constraint_values);

    // TODO avoid cast by templatizing make_constraints_view().
    VectorX<T> lower_T = this->get_constraint_lower_bounds().template cast<T>();
    // ConstraintsView lower = make_constraints_view(lower_T);
    VectorX<T> upper_T = this->get_constraint_upper_bounds().template cast<T>();
    // ConstraintsView upper = make_constraints_view(upper_T);
    auto state_names = m_ocproblem->get_state_names();
    auto control_names = m_ocproblem->get_control_names();
    auto adjunct_names = m_ocproblem->get_adjunct_names();
    auto parameter_names = m_ocproblem->get_parameter_names();
    std::vector<std::string> time_names = {"initial_time", "final_time"};

    Iterate ocp_vars_lower =
            deconstruct_iterate(this->get_variable_lower_bounds());
    Iterate ocp_vars_upper =
            deconstruct_iterate(this->get_variable_upper_bounds());

    // TODO handle the case where there are no states or no controls.

    // Find the longest state, control, or adjunct name.
    auto compare_size = [](const std::string& a, const std::string& b) {
        return a.size() < b.size();
    };
    int max_name_length = 0;
    if (!state_names.empty()) {
        max_name_length = (int)std::max_element(
                state_names.begin(), state_names.end(), compare_size)
                                  ->size();
    }
    if (!control_names.empty()) {
        max_name_length = (int)std::max((size_t)max_name_length,
                std::max_element(control_names.begin(), control_names.end(),
                        compare_size)
                        ->size());
    }
    if (!adjunct_names.empty()) {
        max_name_length = (int)std::max((size_t)max_name_length,
                std::max_element(adjunct_names.begin(), adjunct_names.end(),
                        compare_size)
                        ->size());
    }

    stream << "\nActive or violated continuous variable bounds" << std::endl;
    stream << "L and U indicate which bound is active; "
              "'*' indicates a bound is violated. "
           << std::endl;
    stream << "The case of lower==upper==value is ignored." << std::endl;

    // Bounds on state, control and adjunct variables.
    // -----------------------------------------------
    using Eigen::MatrixXd;
    using Eigen::RowVectorXd;
    auto print_bounds = [&stream, max_name_length](
                                const std::string& description,
                                const std::vector<std::string>& names,
                                const RowVectorXd& times,
                                const MatrixXd& values, const MatrixXd& lower,
                                const MatrixXd& upper) {
        // TODO
        stream << "\n" << description << ": ";

        bool bounds_active = false;
        bool bounds_violated = false;
        for (Eigen::Index ivar = 0; ivar < values.rows(); ++ivar) {
            for (Eigen::Index itime = 0; itime < times.size(); ++itime) {
                const auto& L = lower(ivar, itime);
                const auto& V = values(ivar, itime);
                const auto& U = upper(ivar, itime);
                if (V <= L || V >= U) {
                    if (V == L && L == U) continue;
                    bounds_active = true;
                    if (V < L || V > U) {
                        bounds_violated = true;
                        break;
                    }
                }
            }
        }

        if (!bounds_active && !bounds_violated) {
            stream << "no bounds active or violated" << std::endl;
            return;
        }

        if (!bounds_violated) {
            stream << "some bounds active but no bounds violated";
        } else {
            stream << "some bounds active or violated";
        }

        stream << "\n"
               << std::setw(max_name_length) << "  " << std::setw(9) << "time "
               << "  " << std::setw(9) << "lower"
               << "    " << std::setw(9) << "value"
               << "    " << std::setw(9) << "upper"
               << " " << std::endl;

        for (Eigen::Index ivar = 0; ivar < values.rows(); ++ivar) {
            for (Eigen::Index itime = 0; itime < times.size(); ++itime) {
                const auto& L = lower(ivar, itime);
                const auto& V = values(ivar, itime);
                const auto& U = upper(ivar, itime);
                if (V <= L || V >= U) {
                    // In the case where lower==upper==value, there is no
                    // issue; ignore.
                    if (V == L && L == U) continue;
                    const auto& time = times[itime];
                    stream << std::setw(max_name_length) << names[ivar] << "  "
                           << std::setprecision(2) << std::scientific
                           << std::setw(9) << time << "  " << std::setw(9) << L
                           << " <= " << std::setw(9) << V
                           << " <= " << std::setw(9) << U << " ";
                    // Show if the constraint is violated.
                    if (V <= L)
                        stream << "L";
                    else
                        stream << " ";
                    if (V >= U)
                        stream << "U";
                    else
                        stream << " ";
                    if (V < L || V > U) stream << "*";
                    stream << std::endl;
                }
            }
        }
    };
    print_bounds("State bounds", state_names, ocp_vars.time, ocp_vars.states,
            ocp_vars_lower.states, ocp_vars_upper.states);
    print_bounds("Control bounds", control_names, ocp_vars.time,
            ocp_vars.controls, ocp_vars_lower.controls,
            ocp_vars_upper.controls);
    print_bounds("Adjunct bounds", adjunct_names, ocp_vars.time,
            ocp_vars.adjuncts, ocp_vars_lower.adjuncts,
            ocp_vars_upper.adjuncts);

    // Bounds on time and parameter variables.
    // ---------------------------------------
    max_name_length = 0;
    if (!parameter_names.empty()) {
        max_name_length = (int)std::max((size_t)max_name_length,
                std::max_element(parameter_names.begin(), parameter_names.end(),
                        compare_size)
                        ->size());
    }
    // Check if max parameter name length is larger than "initial_time".
    if (max_name_length < (int)time_names[0].size()) {
        max_name_length = (int)time_names[0].size();
    }

    stream << "\nActive or violated parameter bounds" << std::endl;
    stream << "L and U indicate which bound is active; "
              "'*' indicates a bound is violated. "
           << std::endl;
    stream << "The case of lower==upper==value is ignored." << std::endl;

    using Eigen::VectorXd;
    auto print_parameter_bounds = [&stream, max_name_length](
                                          const std::string& description,
                                          const std::vector<std::string>& names,
                                          const VectorXd& values,
                                          const VectorXd& lower,
                                          const VectorXd& upper) {
        stream << "\n" << description << ": ";

        bool bounds_active = false;
        bool bounds_violated = false;
        for (Eigen::Index ivar = 0; ivar < values.rows(); ++ivar) {
            const auto& L = lower(ivar);
            const auto& V = values(ivar);
            const auto& U = upper(ivar);
            if (V <= L || V >= U) {
                if (V == L && L == U) continue;
                bounds_active = true;
                if (V < L || V > U) {
                    bounds_violated = true;
                    break;
                }
            }
        }

        if (!bounds_active && !bounds_violated) {
            stream << "no bounds active or violated" << std::endl;
            return;
        }

        if (!bounds_violated) {
            stream << "some bounds active but no bounds violated";
        } else {
            stream << "some bounds active or violated";
        }

        stream << "\n"
               << std::setw(max_name_length) << "  " << std::setw(9) << "lower"
               << "    " << std::setw(9) << "value"
               << "    " << std::setw(9) << "upper"
               << " " << std::endl;

        for (Eigen::Index ivar = 0; ivar < values.rows(); ++ivar) {
            const auto& L = lower(ivar);
            const auto& V = values(ivar);
            const auto& U = upper(ivar);
            if (V <= L || V >= U) {
                // In the case where lower==upper==value, there is no
                // issue; ignore.
                if (V == L && L == U) continue;
                stream << std::setw(max_name_length) << names[ivar] << "  "
                       << std::setprecision(2) << std::scientific
                       << std::setw(9) << L << " <= " << std::setw(9) << V
                       << " <= " << std::setw(9) << U << " ";
                // Show if the constraint is violated.
                if (V <= L)
                    stream << "L";
                else
                    stream << " ";
                if (V >= U)
                    stream << "U";
                else
                    stream << " ";
                if (V < L || V > U) stream << "*";
                stream << std::endl;
            }
        }
    };
    VectorXd time_values(2);
    time_values << ocp_vars.time[0], ocp_vars.time.tail<1>()[0];
    VectorXd time_lower(2);
    time_lower << ocp_vars_lower.time[0], ocp_vars_lower.time.tail<1>()[0];
    VectorXd time_upper(2);
    time_upper << ocp_vars_upper.time[0], ocp_vars_upper.time.tail<1>()[0];
    print_parameter_bounds(
            "Time bounds", time_names, time_values, time_lower, time_upper);
    print_parameter_bounds("Parameter bounds", parameter_names,
            ocp_vars.parameters, ocp_vars_lower.parameters,
            ocp_vars_upper.parameters);

    // Constraints.
    // ============

    stream << "\nTotal number of constraints: " << constraint_values.size()
           << "." << std::endl;

    // Differential equation defects.
    // ------------------------------
    // stream << "\nDifferential equation defects:" << std::endl;
    // stream << std::setw(max_name_length) << " "
    //        << " L2 norm across the mesh"
    //        << "   max abs value" // infinity norm.
    //        << "   time of max abs"
    //        << std::endl;
    stream << "\nDifferential equation defects:"
           << "\n  L2 norm across mesh, max abs value (L1 norm), time of max "
              "abs"
           << std::endl;

    std::string spacer(7, ' ');
    Eigen::RowVectorXd rowd(values.defects.cols());
    for (size_t i_state = 0; i_state < state_names.size(); ++i_state) {
        RowVectorX<T> rowT = values.defects.row(i_state);
        for (int i = 0; i < rowd.size(); ++i)
            rowd[i] = static_cast<const double&>(rowT[i]);
        const double L2 = rowd.norm();
        Eigen::Index argmax;
        const double L1 = rowd.cwiseAbs().maxCoeff(&argmax);
        const double time_of_max = ocp_vars.time[argmax];

        stream << std::setw(max_name_length) << state_names[i_state] << spacer
               << std::setprecision(2) << std::scientific << std::setw(9) << L2
               << spacer << L1 << spacer << std::setprecision(6) << std::fixed
               << time_of_max << std::endl;
    }
    /*
    stream << "Differential equation defects for each mesh interval:"
            << std::endl;
    stream << std::setw(9) << "time" << "  ";
    for (size_t i_state = 0; i_state < state_names.size(); ++i_state) {
        stream << std::setw(9) << i_state << "  ";
    }
    stream << std::endl;
    for (int i_mesh = 0; i_mesh < (int)values.defects.cols(); ++i_mesh) {

        stream << std::setw(4) << i_mesh << "  "
                << ocp_vars.time[i_mesh] << "  ";
        for (size_t i_state = 0; i_state < state_names.size(); ++i_state) {
            auto& value = static_cast<const double&>(
                    values.defects(i_state, i_mesh));
            stream << std::setprecision(2) << std::scientific << std::setw(9)
                    << value << "  ";
        }
        stream << std::endl;
    }
     */

    // Path constraints.
    // -----------------
    stream << "\nPath constraints:";
    auto pathcon_names = m_ocproblem->get_path_constraint_names();

    if (pathcon_names.empty()) {
        stream << " none" << std::endl;
        // Return early if there are no path constraints.
        return;
    }
    stream << std::endl;

    const int max_pathcon_name_length = (int)std::max_element(
            pathcon_names.begin(), pathcon_names.end(), compare_size)
                                                ->size();
    stream << "\n  L2 norm across mesh, max abs value (L1 norm), time of max "
              "abs"
           << std::endl;
    rowd.resize(values.path_constraints.cols());
    for (size_t i_pc = 0; i_pc < pathcon_names.size(); ++i_pc) {
        RowVectorX<T> rowT = values.path_constraints.row(i_pc);
        for (int i = 0; i < rowd.size(); ++i)
            rowd[i] = static_cast<const double&>(rowT[i]);
        const double L2 = rowd.norm();
        Eigen::Index argmax;
        const double L1 = rowd.cwiseAbs().maxCoeff(&argmax);
        const auto time_of_max = ocp_vars.time[argmax];

        stream << std::setw(2) << i_pc << ":"
               << std::setw(max_pathcon_name_length) << pathcon_names[i_pc]
               << spacer << std::setprecision(2) << std::scientific
               << std::setw(9) << L2 << spacer << L1 << spacer
               << std::setprecision(6) << std::fixed << time_of_max
               << std::endl;
    }
    stream << "Path constraint values at each mesh point:" << std::endl;
    stream << std::setw(9) << "time"
           << "  ";
    for (size_t i_pc = 0; i_pc < pathcon_names.size(); ++i_pc) {
        stream << std::setw(9) << i_pc << "  ";
    }
    stream << std::endl;
    for (size_t i_mesh = 0; i_mesh < size_t(values.path_constraints.cols());
            ++i_mesh) {

        stream << std::setw(4) << i_mesh << "  " << ocp_vars.time[i_mesh]
               << "  ";
        for (size_t i_pc = 0; i_pc < pathcon_names.size(); ++i_pc) {
            auto& value = static_cast<const double&>(
                    values.path_constraints(i_pc, i_mesh));
            stream << std::setprecision(2) << std::scientific << std::setw(9)
                   << value << "  ";
        }
        stream << std::endl;
    }
}

template <typename T>
template <typename S>
typename Trapezoidal<T>::template ParameterViewConst<S>
Trapezoidal<T>::make_parameters_view(const VectorX<S>& x) const {
    return {// Pointer to the start of the parameters.
            x.data() + m_num_time_variables,
            // Size of the parameter view vector.
            m_num_parameters};
}

template <typename T>
template <typename S>
typename Trapezoidal<T>::template TrajectoryViewConst<S>
Trapezoidal<T>::make_states_trajectory_view(const VectorX<S>& x) const {
    // TODO move time variables to the end.
    return {// Pointer to the start of the states.
            x.data() + m_num_dense_variables,
            m_num_states,      // Number of rows.
            m_num_mesh_points, // Number of columns.
                               // Distance between the start of each column.
            Eigen::OuterStride<Eigen::Dynamic>(m_num_continuous_variables)};
}

template <typename T>
template <typename S>
typename Trapezoidal<T>::template TrajectoryViewConst<S>
Trapezoidal<T>::make_controls_trajectory_view(const VectorX<S>& x) const {
    return {// Start of controls for first mesh interval.
            x.data() + m_num_dense_variables + m_num_states,
            m_num_controls,    // Number of rows.
            m_num_mesh_points, // Number of columns.
                               // Distance between the start of each column;
                               // same as above.
            Eigen::OuterStride<Eigen::Dynamic>(m_num_continuous_variables)};
}

template <typename T>
template <typename S>
typename Trapezoidal<T>::template TrajectoryViewConst<S>
Trapezoidal<T>::make_adjuncts_trajectory_view(const VectorX<S>& x) const {
    return {// Start of adjuncts for first mesh interval.
            x.data() + m_num_dense_variables + m_num_states + m_num_controls,
            m_num_adjuncts,    // Number of rows.
            m_num_mesh_points, // Number of columns.
                               // Distance between the start of each column;
                               // same as above.
            Eigen::OuterStride<Eigen::Dynamic>(m_num_continuous_variables)};
}

// TODO avoid the duplication with the above.
template <typename T>
template <typename S>
typename Trapezoidal<T>::template ParameterView<S>
Trapezoidal<T>::make_parameters_view(VectorX<S>& x) const {
    return {// Pointer to the start of the parameters.
            x.data() + m_num_time_variables,
            // Size of the parameter view vector.
            m_num_parameters};
}

template <typename T>
template <typename S>
typename Trapezoidal<T>::template TrajectoryView<S>
Trapezoidal<T>::make_states_trajectory_view(VectorX<S>& x) const {
    return {// Pointer to the start of the states.
            x.data() + m_num_dense_variables,
            m_num_states,      // Number of rows.
            m_num_mesh_points, // Number of columns.
                               // Distance between the start of each column.
            Eigen::OuterStride<Eigen::Dynamic>(m_num_continuous_variables)};
}

template <typename T>
template <typename S>
typename Trapezoidal<T>::template TrajectoryView<S>
Trapezoidal<T>::make_controls_trajectory_view(VectorX<S>& x) const {
    return {// Start of controls for first mesh interval.
            x.data() + m_num_dense_variables + m_num_states,
            m_num_controls,    // Number of rows.
            m_num_mesh_points, // Number of columns.
                               // Distance between the start of each column;
                               // same as above.
            Eigen::OuterStride<Eigen::Dynamic>(m_num_continuous_variables)};
}

template <typename T>
template <typename S>
typename Trapezoidal<T>::template TrajectoryView<S>
Trapezoidal<T>::make_adjuncts_trajectory_view(VectorX<S>& x) const {
    return {// Start of adjuncts for first mesh interval.
            x.data() + m_num_dense_variables + m_num_states + m_num_controls,
            m_num_adjuncts,    // Number of rows.
            m_num_mesh_points, // Number of columns.
                               // Distance between the start of each column;
                               // same as above.
            Eigen::OuterStride<Eigen::Dynamic>(m_num_continuous_variables)};
}

template <typename T>
typename Trapezoidal<T>::ConstraintsView Trapezoidal<T>::make_constraints_view(
        Eigen::Ref<VectorX<T>> constr) const {
    // Starting indices of different parts of the constraints vector.
    T* d_ptr = m_num_defects ? // defects.
                       &constr[0]
                             : nullptr;
    T* pc_ptr = m_num_path_constraints ? // path constraints.
                        &constr[m_num_dynamics_constraints]
                                       : nullptr;
    return {DefectsTrajectoryView(d_ptr, m_num_states, m_num_defects),
            PathConstraintsTrajectoryView(
                    pc_ptr, m_num_path_constraints, m_num_mesh_points)};
}

} // namespace transcription
} // namespace tropter

#endif // TROPTER_OPTIMALCONTROL_TRANSCRIPTION_TRAPEZOIDAL_HPP
