#ifndef TROPTER_OPTIMALCONTROL_TRANSCRIPTION_HERMITESIMPSON_HPP
#define TROPTER_OPTIMALCONTROL_TRANSCRIPTION_HERMITESIMPSON_HPP
// ----------------------------------------------------------------------------
// tropter: HermiteSimpson.hpp
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

#include "HermiteSimpson.h"
#include <iomanip>

#include <tropter/Exception.hpp>
#include <tropter/SparsityPattern.h>

namespace tropter {
namespace transcription {

template <typename T>
void HermiteSimpson<T>::set_ocproblem(
        std::shared_ptr<const OCProblem> ocproblem) {
    m_ocproblem = ocproblem;
    m_num_mesh_points = (int)m_mesh.size();
    m_num_mesh_intervals = m_num_mesh_points - 1;
    // We define the set of collocation points to include any time point where
    // derivatives of the state estimates are required to match the estimated
    // dynamics. For Hermite-Simpson collocation, collocation points include
    // both the mesh points and the mesh interval midpoints, so add N-1 points
    // to the number of mesh points to get the number of collocation points.
    // TODO rename to m_num_grid_points?
    m_num_col_points = 2 * m_num_mesh_intervals + 1;
    m_num_states = m_ocproblem->get_num_states();
    m_num_controls = m_ocproblem->get_num_controls();
    m_num_adjuncts = m_ocproblem->get_num_adjuncts();
    m_num_diffuses = m_ocproblem->get_num_diffuses();
    m_num_continuous_variables = m_num_states + m_num_controls + m_num_adjuncts;
    m_num_time_variables = 2;
    m_num_parameters = m_ocproblem->get_num_parameters();
    m_num_dense_variables = m_num_time_variables + m_num_parameters;
    int num_variables = m_num_time_variables + m_num_parameters +
                        m_num_col_points * m_num_continuous_variables +
                        m_num_mesh_intervals * m_num_diffuses;
    this->set_num_variables(num_variables);
    // The separated form of Hermite-Simpson requires two constraint equations
    // to implement the defects for a single state variable, one for the
    // midpoint interpolation (Hermite) and one for the integration rule
    // (Simpson). Therefore, the number of defects is 2*N.
    m_num_defects = m_num_states ? 2 * m_num_mesh_intervals : 0;
    m_num_dynamics_constraints = m_num_defects * m_num_states;
    m_num_path_constraints = m_ocproblem->get_num_path_constraints();
    // TODO rename..."total_path_constraints"?
    m_num_path_traj_constraints = m_num_mesh_points * m_num_path_constraints;
    m_num_control_midpoint_constraints =
            m_interpolate_control_midpoints
                    ? m_num_controls * m_num_mesh_intervals
                    : 0;
    int num_constraints = m_num_dynamics_constraints +
                          m_num_path_traj_constraints +
                          m_num_control_midpoint_constraints;
    this->set_num_constraints(num_constraints);

    // Variable and constraint names.
    // ------------------------------
    m_variable_names.clear();
    m_variable_names.emplace_back("initial_time");
    m_variable_names.emplace_back("final_time");

    const auto param_names = m_ocproblem->get_parameter_names();
    for (const auto& param_name : param_names)
        m_variable_names.push_back(param_name);

    // For padding, count the number of digits in num_mesh_intervals.
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
    const auto diffuse_names = m_ocproblem->get_diffuse_names();
    for (int i_mesh = 0; i_mesh < m_num_mesh_points; ++i_mesh) {
        // If not the first mesh point, also add in midpoint variable names
        // before the mesh point variable names.
        // Ordering based on Betts 4.105, pg. 144.
        if (i_mesh) {
            for (const auto& state_name : state_names) {
                std::stringstream ss;
                ss << state_name << "_bar_" << std::setfill('0')
                   << std::setw(num_digits_max_mesh_index) << i_mesh;
                m_variable_names.push_back(ss.str());
            }
            for (const auto& control_name : control_names) {
                std::stringstream ss;
                ss << control_name << "_bar_" << std::setfill('0')
                   << std::setw(num_digits_max_mesh_index) << i_mesh;
                m_variable_names.push_back(ss.str());
            }
            for (const auto& adjunct_name : adjunct_names) {
                std::stringstream ss;
                ss << adjunct_name << "_bar_" << std::setfill('0')
                   << std::setw(num_digits_max_mesh_index) << i_mesh;
                m_variable_names.push_back(ss.str());
            }
        }
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
    // Need to append diffuses to end for now to allow slicing.
    for (int i_mesh = 1; i_mesh < m_num_mesh_points; ++i_mesh) {
        for (const auto& diffuse_name : diffuse_names) {
            std::stringstream ss;
            ss << diffuse_name << "_bar_" << std::setfill('0')
               << std::setw(num_digits_max_mesh_index) << i_mesh;
            m_variable_names.push_back(ss.str());
        }
    }

    m_constraint_names.clear();
    // Start at index 1 (counting mesh intervals; not mesh points).
    // Betts eq. 4.107 on page 144 suggests that the Hermite defect constraints
    // (eq. 4.103) for all states at a collocation point should be grouped
    // together, followed by all of the Simpson defect constraints (eq. 4.104).
    // TODO I (nickbianco) don't think we currently take advantage of this
    // constraint ordering in the defect computations. See below.
    for (int i_mesh = 1; i_mesh < m_num_mesh_points; ++i_mesh) {
        for (const auto& state_name : state_names) {
            std::stringstream ss;
            ss << state_name << "_hermite_" << std::setfill('0')
               << std::setw(num_digits_max_mesh_index) << i_mesh;
            m_constraint_names.push_back(ss.str());
        }
        for (const auto& state_name : state_names) {
            std::stringstream ss;
            ss << state_name << "_simpson_" << std::setfill('0')
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
    if (m_interpolate_control_midpoints) {
        for (int i_mesh = 1; i_mesh < m_num_mesh_points; ++i_mesh) {
            for (const auto& control_name : control_names) {
                std::stringstream ss;
                ss << control_name << "_midpoint_" << std::setfill('0')
                   << std::setw(num_digits_max_mesh_index) << i_mesh;
                m_constraint_names.push_back(ss.str());
            }
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
    VectorXd diffuses_lower(m_num_diffuses);
    VectorXd diffuses_upper(m_num_diffuses);
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
                    .replicate(m_num_col_points - 2, 1),
            final_states_lower, final_controls_lower, final_adjuncts_lower,
            (VectorXd(m_num_diffuses) << diffuses_lower)
                    .finished()
                    .replicate(m_num_mesh_intervals, 1);
    VectorXd variable_upper(num_variables);
    variable_upper << initial_time_upper, final_time_upper, parameters_upper,
            initial_states_upper, initial_controls_upper,
            initial_adjuncts_upper,
            (VectorXd(m_num_continuous_variables) << states_upper,
                    controls_upper, adjuncts_upper)
                    .finished()
                    .replicate(m_num_col_points - 2, 1),
            final_states_upper, final_controls_upper, final_adjuncts_upper,
            (VectorXd(m_num_diffuses) << diffuses_upper)
                    .finished()
                    .replicate(m_num_mesh_intervals, 1);
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
    VectorXd control_midpoint_bounds =
            VectorXd::Zero(m_num_control_midpoint_constraints);
    constraint_lower << dynamics_bounds, path_constraints_traj_lower,
            control_midpoint_bounds;
    constraint_upper << dynamics_bounds, path_constraints_traj_upper,
            control_midpoint_bounds;
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
    // Simpson quadrature includes integrand evaluations at the midpoint.
    m_simpson_quadrature_coefficients = VectorXd::Zero(m_num_col_points);
    // The fractional coefficients that, when multiplied by the mesh fraction
    // for a given mesh interval, make up the Simpson quadrature coefficients
    // for that mesh interval.
    Eigen::Vector3d fracs(1.0 / 6.0, 2.0 / 3.0, 1.0 / 6.0);
    // Loop through each mesh interval and update the corresponding components
    // in the total coefficients vector.
    for (int i_mesh = 0; i_mesh < m_num_mesh_intervals; ++i_mesh) {
        // The mesh interval coefficients overlap at the mesh grid points in the
        // total coefficients vector, so we create a map starting at every
        // other index
        Eigen::Map<Eigen::Vector3d> mesh_interval_coefs_map(
                m_simpson_quadrature_coefficients.data() + 2 * i_mesh, 3);

        // Update the total coefficients vector for this mesh interval.
        mesh_interval_coefs_map += m_mesh_intervals[i_mesh] * fracs;
    }

    // Allocate working memory.
    m_integrand.resize(m_num_col_points);
    m_derivs_mesh.resize(m_num_states, m_num_mesh_points);
    m_derivs_mid.resize(m_num_states, m_num_mesh_intervals);
    m_mesh_and_midpoints.resize(m_num_col_points);
    // Return a mesh including the Hermite-Simpson collocation midpoints to
    // enable initialization of mesh-dependent integral cost quantities.

    for (int i = 0; i < (int)m_num_col_points; ++i) {
        if (i % 2 == 0) {
            m_mesh_and_midpoints[i] = m_mesh[i / 2];
        } else {
            m_mesh_and_midpoints[i] = 0.5 * (m_mesh[i / 2] + m_mesh[i / 2 + 1]);
        }
    }
    m_ocproblem->initialize_on_mesh(m_mesh_and_midpoints);
}

template <typename T>
void HermiteSimpson<T>::calc_objective(
        const VectorX<T>& x, T& obj_value) const {
    // TODO move this to a "make_variables_view()"
    const T& initial_time = x[0];
    const T& final_time = x[1];
    const T duration = final_time - initial_time;

    // TODO I don't actually need to make a new view each time; just change the
    // data pointer. TODO probably don't even need to update the data pointer!
    auto states = make_states_trajectory_view(x);
    auto controls = make_controls_trajectory_view(x);
    auto adjuncts = make_adjuncts_trajectory_view(x);
    auto diffuses = make_diffuses_trajectory_view(x);
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
            int i_diff = 0;
            // TODO avoid this copy. use Ref?
            VectorX<T> diffuse_to_use;
            for (int i_col = 0; i_col < m_num_col_points; ++i_col) {
                const T time =
                        duration * m_mesh_and_midpoints[i_col] + initial_time;
                // Only pass diffuse variables on the midpoints where they are
                // defined, otherwise pass an empty variable.
                if (i_col % 2) {
                    diffuse_to_use = diffuses.col(i_diff);
                    ++i_diff;
                } else {
                    diffuse_to_use = m_empty_diffuse_col;
                }

                m_ocproblem->calc_cost_integrand(i_cost,
                        {i_col, time, states.col(i_col), controls.col(i_col),
                                adjuncts.col(i_col), diffuse_to_use,
                                parameters},
                        m_integrand[i_col]);
            }

            for (int i_col = 0; i_col < m_num_col_points; ++i_col) {
                integral += m_simpson_quadrature_coefficients[i_col] *
                            m_integrand[i_col];
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
void HermiteSimpson<T>::calc_constraints(
        const VectorX<T>& x, Eigen::Ref<VectorX<T>> constraints) const {
    // TODO parallelize.
    const T& initial_time = x[0];
    const T& final_time = x[1];
    const T duration = final_time - initial_time;

    auto states = make_states_trajectory_view(x);
    auto controls = make_controls_trajectory_view(x);
    auto adjuncts = make_adjuncts_trajectory_view(x);
    auto diffuses = make_diffuses_trajectory_view(x);
    auto parameters = make_parameters_view(x);

    // Initialize on iterate.
    // ======================
    m_ocproblem->initialize_on_iterate(parameters);

    // Organize the constrants vector.
    ConstraintsView constr_view = make_constraints_view(constraints);

    // Dynamics and path constraints.
    // ==============================
    // "Continuous function"

    // Obtain state derivatives at each mesh point.
    // --------------------------------------------
    // Evaluate points on the mesh.
    int i_mesh = 0;
    for (int i_col = 0; i_col < m_num_col_points; i_col += 2) {
        const T time = duration * m_mesh_and_midpoints[i_col] + initial_time;
        m_ocproblem->calc_differential_algebraic_equations(
                {i_col, time, states.col(i_col), controls.col(i_col),
                        adjuncts.col(i_col), m_empty_diffuse_col, parameters},
                {m_derivs_mesh.col(i_mesh),
                        constr_view.path_constraints.col(i_mesh)});
        i_mesh++;
    }
    // Evaluate points on the mesh interval interior.
    int i_mid = 0;
    for (int i_col = 1; i_col < m_num_col_points; i_col += 2) {
        const T time = duration * m_mesh_and_midpoints[i_col] + initial_time;
        m_ocproblem->calc_differential_algebraic_equations(
                {i_col, time, states.col(i_col), controls.col(i_col),
                        adjuncts.col(i_col), diffuses.col(i_mid), parameters},
                {m_derivs_mid.col(i_mid), m_empty_path_constraint_col});
        TROPTER_THROW_IF(m_empty_path_constraint_col.size() != 0,
                "Invalid resize of empty path constraint output.");
        i_mid++;
    }

    // Compute constraint defects.
    // ---------------------------
    if (m_num_defects) {
        // Betts eq. 4.107 on page 144 suggests that the Hermite defect
        // constraints (eq. 4.103) for all states at a collocation point should
        // be grouped together, followed by all of the Simpson defect
        // constraints (eq. 4.104).
        const unsigned N = m_num_mesh_intervals;

        // States.
        auto x_mesh = make_states_trajectory_view_mesh(x);
        auto x_mid = make_states_trajectory_view_mid(x);
        const auto& x_i = x_mesh.rightCols(N);
        const auto& x_im1 = x_mesh.leftCols(N);

        // State derivatives.
        const auto& xdot_i = m_derivs_mesh.rightCols(N);
        const auto& xdot_im1 = m_derivs_mesh.leftCols(N);
        const auto& xdot_mid = m_derivs_mid;

        // TODO separate out nonlinear components of the constraint vector per
        // Bett's eq. 4.107 on page 144 to fully take advantage of the separated
        // Hermite-Simpson form.

        // Hermite interpolant defects
        // ---------------------------
        for (int imesh = 0; imesh < m_num_mesh_intervals; ++imesh) {

            const auto& h = duration * m_mesh_intervals[imesh];
            constr_view.defects.topRows(m_num_states).col(imesh) =
                    x_mid.col(imesh) -
                    T(0.5) * (x_i.col(imesh) + x_im1.col(imesh)) -
                    (h / T(8.0)) * (xdot_im1.col(imesh) - xdot_i.col(imesh));

            // Simpson integration defects
            // ---------------------------
            constr_view.defects.bottomRows(m_num_states).col(imesh) =
                    x_i.col(imesh) - x_im1.col(imesh) -
                    (h / T(6.0)) *
                            (xdot_i.col(imesh) + T(4.0) * xdot_mid.col(imesh) +
                                    xdot_im1.col(imesh));
        }
    }

    if (m_num_controls && m_interpolate_control_midpoints) {
        const unsigned N = m_num_mesh_intervals;
        auto c_mesh = make_controls_trajectory_view_mesh(x);
        auto c_mid = make_controls_trajectory_view_mid(x);
        const auto& c_i = c_mesh.rightCols(N);
        const auto& c_im1 = c_mesh.leftCols(N);

        constr_view.control_midpoints = c_mid - T(0.5) * (c_i + c_im1);
    }
}

template <typename T>
void HermiteSimpson<T>::calc_sparsity_hessian_lagrangian(
        const Eigen::VectorXd& x, SymmetricSparsityPattern& hescon_sparsity,
        SymmetricSparsityPattern& hesobj_sparsity) const {
    const auto& num_variables = this->get_num_variables();

    const auto& num_con_vars = m_num_continuous_variables;

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
        TROPTER_THROW(
                "Automatic sparsity detection for Hessian diagonal "
                "blocks not implemented for Hermite-Simpson transcription.");
    } else if (this->get_exact_hessian_block_sparsity_mode() == "dense") {
        dae_sparsity.set_dense();
    }

    // Repeat the block down the diagonal of the Hessian of constraints.
    // TODO may need to move this into each branch of if-statement above,
    // depending on how automatic sparsity detection is implemented.
    for (int i_col = 0; i_col < m_num_col_points; ++i_col) {
        const auto istart = m_num_dense_variables + i_col * num_con_vars;
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

    for (int icost = 0; icost < m_ocproblem->get_num_costs(); ++icost) {
        if (m_ocproblem->get_cost_requires_integral(icost)) {
            SymmetricSparsityPattern integral_cost_sparsity(num_con_vars);
            if (this->get_exact_hessian_block_sparsity_mode() == "sparse") {
                TROPTER_THROW(
                        "Automatic sparsity detection for Hessian diagonal "
                        "blocks not implemented for Hermite-Simpson "
                        "transcription.");
            } else if (this->get_exact_hessian_block_sparsity_mode() ==
                       "dense") {
                integral_cost_sparsity.set_dense();
            }

            // Repeat the block down the diagonal of the Hessian of the
            // objective.
            // TODO may need to move this into each branch of if-statement
            // above, depending on how automatic sparsity detection is
            // implemented.
            for (int i_col = 0; i_col < m_num_col_points; ++i_col) {
                const auto istart =
                        m_num_dense_variables + i_col * num_con_vars;
                hesobj_sparsity.set_nonzero_block(
                        istart, integral_cost_sparsity);
            }
        }

        // The cost depends on the initial state/controls, final state/controls,
        // and the integral if it exists.
        SymmetricSparsityPattern cost_initial_sparsity(num_con_vars);
        SymmetricSparsityPattern cost_final_sparsity(num_con_vars);
        const auto lastmeshstart =
                m_num_dense_variables + (m_num_col_points - 1) * num_con_vars;
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
                                {0, it, is, ic, ia, m_num_mesh_points - 1, ft, fs,
                                        fc, fa, p, integral},
                                cost);
                        return cost;
                    };
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
std::vector<std::string> HermiteSimpson<T>::get_variable_names() const {
    return m_variable_names;
}

template <typename T>
std::vector<std::string> HermiteSimpson<T>::get_constraint_names() const {
    return m_constraint_names;
}

template <typename T>
Eigen::RowVectorXd HermiteSimpson<T>::get_diffuse_times(
        const Eigen::RowVectorXd& time) const {

    Eigen::RowVectorXd diffuse_times(m_num_mesh_intervals);
    int diffuse_time_index = 0;
    for (int i = 1; i < time.size() - 1; i += 2) {
        diffuse_times[diffuse_time_index] = time[i];
        ++diffuse_time_index;
    }
    return diffuse_times;
}

template <typename T>
Eigen::MatrixXd HermiteSimpson<T>::get_diffuses_with_nans(
        const Eigen::MatrixXd& diffuses_without_nans) const {
    assert(diffuses_without_nans.rows() == m_num_diffuses);
    assert(diffuses_without_nans.cols() == m_num_mesh_points - 1);
    Eigen::MatrixXd diffuses_with_nans =
            Eigen::MatrixXd::Constant(m_num_diffuses, m_num_col_points,
                    std::numeric_limits<double>::quiet_NaN());

    int i_mid = 0;
    for (int i_col = 1; i_col < m_num_col_points - 1; i_col += 2) {
        diffuses_with_nans.col(i_col) = diffuses_without_nans.col(i_mid);
        ++i_mid;
    }

    return diffuses_with_nans;
}

template <typename T>
Eigen::MatrixXd HermiteSimpson<T>::get_diffuses_without_nans(
        const Eigen::MatrixXd& diffuses_with_nans) const {
    assert(diffuses_with_nans.rows() == m_num_diffuses);
    assert(diffuses_with_nans.cols() == m_num_col_points);
    Eigen::MatrixXd diffuses_without_nans(m_num_diffuses, m_num_mesh_intervals);

    int i_mid = 0;
    for (int i_col = 1; i_col < m_num_col_points - 1; i_col += 2) {
        diffuses_without_nans.col(i_mid) = diffuses_with_nans.col(i_col);
        ++i_mid;
    }

    return diffuses_without_nans;
}

template <typename T>
Eigen::VectorXd HermiteSimpson<T>::construct_iterate(
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
    TROPTER_THROW_IF(traj.diffuses.rows() != m_num_diffuses,
            "Expected diffuses to have %i row(s), but it has %i.",
            m_num_diffuses, traj.diffuses.rows());
    TROPTER_THROW_IF(traj.parameters.rows() != m_num_parameters,
            "Expected parameters to have %i element(s), but it has %i.",
            m_num_parameters, traj.parameters.size());
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
        if (traj.diffuses.cols()) {
            TROPTER_THROW_IF(traj.time.size() != traj.diffuses.cols(),
                    "Expected time and diffuses to have the same number of "
                    "columns, but they have %i and %i column(s), "
                    "respectively.",
                    traj.time.size(), traj.diffuses.cols());
        }
    } else {
        TROPTER_THROW_IF(traj.time.size() != m_num_col_points,
                "Expected time to have %i element(s), but it has %i.",
                m_num_col_points, traj.time.size());
        TROPTER_THROW_IF(traj.states.cols() != m_num_col_points,
                "Expected states to have %i column(s), but it has %i.",
                m_num_col_points, traj.states.cols());
        TROPTER_THROW_IF(traj.controls.cols() != m_num_col_points,
                "Expected controls to have %i column(s), but it has %i.",
                m_num_col_points, traj.controls.cols());
        TROPTER_THROW_IF(traj.adjuncts.cols() != m_num_col_points,
                "Expected adjuncts to have %i column(s), but it has %i.",
                m_num_col_points, traj.adjuncts.cols());
        TROPTER_THROW_IF(traj.diffuses.cols() != m_num_col_points,
                "Expected diffuses to have %i column(s), but it has %i.",
                m_num_col_points, traj.diffuses.cols());
    }

    // Interpolate the trajectory, as it might have a different number of
    // collocation points than m_num_col_points.
    Iterate traj_interp;
    const Iterate* traj_to_use;
    if (interpolate) {
        const auto duration = traj.time.tail<1>()[0] - traj.time[0];
        traj_interp = traj.interpolate(
                duration * m_mesh_and_midpoints.array() + traj.time[0]);
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
    if (traj_to_use->diffuses.cols())
        this->make_diffuses_trajectory_view(iterate) =
                get_diffuses_without_nans(traj_to_use->diffuses);
    if (traj_to_use->parameters.size())
        this->make_parameters_view(iterate) = traj_to_use->parameters;

    return iterate;
}

template <typename T>
Iterate HermiteSimpson<T>::deconstruct_iterate(const Eigen::VectorXd& x) const {
    // TODO move time variables to the end.
    const double& initial_time = x[0];
    const double& final_time = x[1];
    Iterate traj;
    traj.time = (final_time - initial_time) * m_mesh_and_midpoints.array() +
                initial_time;

    traj.states = this->make_states_trajectory_view(x);
    traj.controls = this->make_controls_trajectory_view(x);
    traj.adjuncts = this->make_adjuncts_trajectory_view(x);
    traj.diffuses =
            get_diffuses_with_nans(this->make_diffuses_trajectory_view(x));
    traj.parameters = this->make_parameters_view(x);

    traj.state_names = m_ocproblem->get_state_names();
    traj.control_names = m_ocproblem->get_control_names();
    traj.adjunct_names = m_ocproblem->get_adjunct_names();
    traj.diffuse_names = m_ocproblem->get_diffuse_names();
    traj.parameter_names = m_ocproblem->get_parameter_names();

    return traj;
}

template <typename T>
void HermiteSimpson<T>::print_constraint_values(
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
    auto diffuse_names = m_ocproblem->get_diffuse_names();
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
    if (!diffuse_names.empty()) {
        max_name_length = (int)std::max((size_t)max_name_length,
                std::max_element(diffuse_names.begin(), diffuse_names.end(),
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
    print_bounds("Diffuse bounds", diffuse_names,
            get_diffuse_times(ocp_vars.time),
            get_diffuses_without_nans(ocp_vars.diffuses),
            get_diffuses_without_nans(ocp_vars_lower.diffuses),
            get_diffuses_without_nans(ocp_vars_upper.diffuses));

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

    // stream << "Differential equation defects for each mesh interval:"
    //       << std::endl;
    // stream << std::setw(9) << "time" << "  ";
    // for (size_t i_state = 0; i_state < state_names.size(); ++i_state) {
    //    stream << std::setw(9) << i_state << "  ";
    //}
    // stream << std::endl;
    // for (int i_mesh = 0; i_mesh < (int)values.defects.cols(); ++i_mesh) {

    //    stream << std::setw(4) << i_mesh << "  "
    //           << ocp_vars.time[i_mesh] << "  ";
    //    for (size_t i_state = 0; i_state < state_names.size(); ++i_state) {
    //        auto& value = static_cast<const double&>(
    //        values.defects(i_state, i_mesh));
    //        stream << std::setprecision(2) << std::scientific << std::setw(9)
    //               << value << "  ";
    //    }
    // stream << std::endl;
    //}

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

        stream << std::setw(4) << 2 * i_mesh << "  "
               << ocp_vars.time[2 * i_mesh] << "  ";
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
typename HermiteSimpson<T>::template ParameterViewConst<S>
HermiteSimpson<T>::make_parameters_view(const VectorX<S>& x) const {
    return {// Pointer to the start of the parameters.
            x.data() + m_num_time_variables,
            // Size of the parameter view vector.
            m_num_parameters};
}

template <typename T>
template <typename S>
typename HermiteSimpson<T>::template TrajectoryViewConst<S>
HermiteSimpson<T>::make_states_trajectory_view(const VectorX<S>& x) const {
    // TODO move time variables to the end.
    return {// Pointer to the start of the states.
            x.data() + m_num_dense_variables,
            m_num_states,     // Number of rows.
            m_num_col_points, // Number of columns.
                              // Distance between the start of each column.
            Eigen::OuterStride<Eigen::Dynamic>(m_num_continuous_variables)};
}

template <typename T>
template <typename S>
typename HermiteSimpson<T>::template TrajectoryViewConst<S>
HermiteSimpson<T>::make_states_trajectory_view_mesh(const VectorX<S>& x) const {
    // TODO move time variables to the end.
    return {// Pointer to the start of the states.
            x.data() + m_num_dense_variables,
            m_num_states,      // Number of rows.
            m_num_mesh_points, // Number of columns.
                               // Distance between the start of each column.
            Eigen::OuterStride<Eigen::Dynamic>(2 * m_num_continuous_variables)};
}

template <typename T>
template <typename S>
typename HermiteSimpson<T>::template TrajectoryViewConst<S>
HermiteSimpson<T>::make_states_trajectory_view_mid(const VectorX<S>& x) const {
    // TODO move time variables to the end.
    return {// Pointer to the start of the states.
            x.data() + m_num_dense_variables + m_num_continuous_variables,
            m_num_states,         // Number of rows.
            m_num_mesh_intervals, // Number of columns.
                                  // Distance between the start of each column.
            Eigen::OuterStride<Eigen::Dynamic>(2 * m_num_continuous_variables)};
}

template <typename T>
template <typename S>
typename HermiteSimpson<T>::template TrajectoryViewConst<S>
HermiteSimpson<T>::make_controls_trajectory_view(const VectorX<S>& x) const {
    return {// Start of controls for first mesh interval.
            x.data() + m_num_dense_variables + m_num_states,
            m_num_controls,   // Number of rows.
            m_num_col_points, // Number of columns.
                              // Distance between the start of each column.
            Eigen::OuterStride<Eigen::Dynamic>(m_num_continuous_variables)};
}

template <typename T>
template <typename S>
typename HermiteSimpson<T>::template TrajectoryViewConst<S>
HermiteSimpson<T>::make_controls_trajectory_view_mesh(
        const VectorX<S>& x) const {
    return {// Start of controls for first mesh interval.
            x.data() + m_num_dense_variables + m_num_states,
            m_num_controls,    // Number of rows.
            m_num_mesh_points, // Number of columns.
                               // Distance between the start of each column.
            Eigen::OuterStride<Eigen::Dynamic>(2 * m_num_continuous_variables)};
}

template <typename T>
template <typename S>
typename HermiteSimpson<T>::template TrajectoryViewConst<S>
HermiteSimpson<T>::make_controls_trajectory_view_mid(
        const VectorX<S>& x) const {
    return {// Start of controls for first mesh interval.
            x.data() + m_num_dense_variables + m_num_states +
                    m_num_continuous_variables,
            m_num_controls,       // Number of rows.
            m_num_mesh_intervals, // Number of columns.
                                  // Distance between the start of each column.
            Eigen::OuterStride<Eigen::Dynamic>(2 * m_num_continuous_variables)};
}

template <typename T>
template <typename S>
typename HermiteSimpson<T>::template TrajectoryViewConst<S>
HermiteSimpson<T>::make_adjuncts_trajectory_view(const VectorX<S>& x) const {
    return {// Start of adjuncts for first mesh interval.
            x.data() + m_num_dense_variables + m_num_states + m_num_controls,
            m_num_adjuncts,   // Number of rows.
            m_num_col_points, // Number of columns.
                              // Distance between the start of each column.
            Eigen::OuterStride<Eigen::Dynamic>(m_num_continuous_variables)};
}

template <typename T>
template <typename S>
typename HermiteSimpson<T>::template TrajectoryViewConst<S>
HermiteSimpson<T>::make_diffuses_trajectory_view(const VectorX<S>& x) const {
    return {// Start of diffuses for first mesh interval.
            x.data() + m_num_dense_variables +
                    m_num_continuous_variables * m_num_col_points,
            m_num_diffuses,       // Number of rows.
            m_num_mesh_intervals, // Number of columns.
                                  // Distance between the start of each column.
            Eigen::OuterStride<Eigen::Dynamic>(m_num_diffuses)};
}

// TODO avoid the duplication with the above.
template <typename T>
template <typename S>
typename HermiteSimpson<T>::template ParameterView<S>
HermiteSimpson<T>::make_parameters_view(VectorX<S>& x) const {
    return {// Pointer to the start of the parameters.
            x.data() + m_num_time_variables,
            // Size of the parameter view vector.
            m_num_parameters};
}

template <typename T>
template <typename S>
typename HermiteSimpson<T>::template TrajectoryView<S>
HermiteSimpson<T>::make_states_trajectory_view(VectorX<S>& x) const {
    return {// Pointer to the start of the states.
            x.data() + m_num_dense_variables,
            m_num_states,     // Number of rows.
            m_num_col_points, // Number of columns.
                              // Distance between the start of each column.
            Eigen::OuterStride<Eigen::Dynamic>(m_num_continuous_variables)};
}

template <typename T>
template <typename S>
typename HermiteSimpson<T>::template TrajectoryView<S>
HermiteSimpson<T>::make_states_trajectory_view_mesh(VectorX<S>& x) const {
    // TODO move time variables to the end.
    return {// Pointer to the start of the states.
            x.data() + m_num_dense_variables,
            m_num_states,      // Number of rows.
            m_num_mesh_points, // Number of columns.
                               // Distance between the start of each column.
            Eigen::OuterStride<Eigen::Dynamic>(2 * m_num_continuous_variables)};
}

template <typename T>
template <typename S>
typename HermiteSimpson<T>::template TrajectoryView<S>
HermiteSimpson<T>::make_states_trajectory_view_mid(VectorX<S>& x) const {
    // TODO move time variables to the end.
    return {// Pointer to the start of the states.
            x.data() + m_num_dense_variables + m_num_continuous_variables,
            m_num_states,         // Number of rows.
            m_num_mesh_intervals, // Number of columns.
                                  // Distance between the start of each column.
            Eigen::OuterStride<Eigen::Dynamic>(2 * m_num_continuous_variables)};
}

template <typename T>
template <typename S>
typename HermiteSimpson<T>::template TrajectoryView<S>
HermiteSimpson<T>::make_controls_trajectory_view(VectorX<S>& x) const {
    return {// Start of controls for first mesh interval.
            x.data() + m_num_dense_variables + m_num_states,
            m_num_controls,   // Number of rows.
            m_num_col_points, // Number of columns.
                              // Distance between the start of each column.
            Eigen::OuterStride<Eigen::Dynamic>(m_num_continuous_variables)};
}

template <typename T>
template <typename S>
typename HermiteSimpson<T>::template TrajectoryView<S>
HermiteSimpson<T>::make_controls_trajectory_view_mesh(VectorX<S>& x) const {
    return {// Start of controls for first mesh interval.
            x.data() + m_num_dense_variables + m_num_states,
            m_num_controls,    // Number of rows.
            m_num_mesh_points, // Number of columns.
                               // Distance between the start of each column.
            Eigen::OuterStride<Eigen::Dynamic>(2 * m_num_continuous_variables)};
}

template <typename T>
template <typename S>
typename HermiteSimpson<T>::template TrajectoryView<S>
HermiteSimpson<T>::make_controls_trajectory_view_mid(VectorX<S>& x) const {
    return {// Start of controls for first mesh interval.
            x.data() + m_num_dense_variables + m_num_states +
                    m_num_continuous_variables,
            m_num_controls,       // Number of rows.
            m_num_mesh_intervals, // Number of columns.
                                  // Distance between the start of each column.
            Eigen::OuterStride<Eigen::Dynamic>(2 * m_num_continuous_variables)};
}

template <typename T>
template <typename S>
typename HermiteSimpson<T>::template TrajectoryView<S>
HermiteSimpson<T>::make_adjuncts_trajectory_view(VectorX<S>& x) const {
    return {// Start of adjuncts for first mesh interval.
            x.data() + m_num_dense_variables + m_num_states + m_num_controls,
            m_num_adjuncts,   // Number of rows.
            m_num_col_points, // Number of columns.
                              // Distance between the start of each column.
            Eigen::OuterStride<Eigen::Dynamic>(m_num_continuous_variables)};
}

template <typename T>
template <typename S>
typename HermiteSimpson<T>::template TrajectoryView<S>
HermiteSimpson<T>::make_diffuses_trajectory_view(VectorX<S>& x) const {
    return {// Start of diffuses for first mesh interval.
            x.data() + m_num_dense_variables +
                    m_num_continuous_variables * m_num_col_points,
            m_num_diffuses,       // Number of rows.
            m_num_mesh_intervals, // Number of columns.
                                  // Distance between the start of each column.
            Eigen::OuterStride<Eigen::Dynamic>(m_num_diffuses)};
}

template <typename T>
typename HermiteSimpson<T>::ConstraintsView
HermiteSimpson<T>::make_constraints_view(Eigen::Ref<VectorX<T>> constr) const {
    // Starting indices of different parts of the constraints vector.
    T* d_ptr = m_num_defects ? // defects.
                       &constr[0]
                             : nullptr;
    T* pc_ptr = m_num_path_constraints ? // path constraints.
                        &constr[m_num_dynamics_constraints]
                                       : nullptr;
    // control midpoint constraints.
    T* cmid_ptr = m_num_controls && m_interpolate_control_midpoints
                          ? &constr[m_num_dynamics_constraints +
                                    m_num_path_traj_constraints]
                          : nullptr;
    // Each column of the defects view contains all the Hermite interpolant
    // defects (first m_num_states rows) followed by all the Simpson interpolant
    // defects (bottom m_num_states rows) for each mesh interval.
    return {DefectsTrajectoryView(
                    d_ptr, 2 * m_num_states, m_num_mesh_intervals),
            PathConstraintsTrajectoryView(
                    pc_ptr, m_num_path_constraints, m_num_mesh_points),
            ControlMidpointsTrajectoryView(
                    cmid_ptr, m_num_controls, m_num_mesh_intervals)};
}

} // namespace transcription
} // namespace tropter

#endif // TROPTER_OPTIMALCONTROL_TRANSCRIPTION_HERMITESIMPSON_HPP
