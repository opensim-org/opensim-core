//
// Created by Chris Dembia on 12/9/16.
//

#include "DirectCollocation.h"
#include "OptimalControlProblem.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Ref;

using namespace mesh;

void EulerTranscription::set_problem(std::shared_ptr<Problem> problem) {
    m_problem = problem;
    m_num_states = m_problem->num_states();
    m_num_controls = m_problem->num_controls();
    m_num_continuous_variables = m_num_states + m_num_controls;
    int num_variables = m_num_mesh_points * m_num_continuous_variables;
    set_num_variables(num_variables);
    int num_bound_constraints = 2 * m_num_continuous_variables;
    int num_dynamics_constraints = (m_num_mesh_points - 1) * m_num_states;
    int num_constraints = num_bound_constraints + num_dynamics_constraints;
    set_num_constraints(num_constraints);

    // Bounds.
    double initial_time;
    double final_time;
    // TODO these could be fixed sizes for certain types of problems.
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
    m_problem->bounds(initial_time, final_time,
            states_lower, states_upper,
            initial_states_lower, initial_states_upper,
            final_states_lower, final_states_upper,
            controls_lower, controls_upper,
            initial_controls_lower, initial_controls_upper,
            final_controls_lower, final_controls_upper);
    // TODO validate sizes.
    m_initial_time = initial_time; // TODO make these variables.
    m_final_time = final_time;
    // Bounds on variables.
    VectorXd variable_lower =
            (VectorXd(m_num_continuous_variables)
                    << states_lower, controls_lower)
                    .finished()
                    .replicate(m_num_mesh_points, 1);
    VectorXd variable_upper =
            (VectorXd(m_num_continuous_variables)
                    << states_upper, controls_upper)
                    .finished()
                    .replicate(m_num_mesh_points, 1);
    set_variable_bounds(variable_lower, variable_upper);
    // Bounds for constraints.
    VectorXd constraint_lower(num_constraints);
    VectorXd constraint_upper(num_constraints);
    // Defects must be 0.
    VectorXd dynamics_bounds = VectorXd::Zero(num_dynamics_constraints);
    constraint_lower << initial_states_lower,
            final_states_lower,
            initial_controls_lower,
            final_controls_lower,
            dynamics_bounds;
    constraint_upper << initial_states_upper,
            final_states_upper,
            initial_controls_upper,
            final_controls_upper,
            dynamics_bounds;
    set_constraint_bounds(constraint_lower, constraint_upper);
    // TODO won't work if the bounds don't include zero!
    // TODO set_initial_guess(std::vector<double>(num_variables)); // TODO user
    // input
}

void EulerTranscription::objective(const VectorXa& x,
        adouble& obj_value) const {
    const double step_size = (m_final_time - m_initial_time) /
            (m_num_mesh_points - 1);

    // Create states and controls vectors.
    // TODO remove when using Eigen. Should definitely use a Map<>.
    VectorXa states(m_num_states);
    for (int i_state = 0; i_state < m_num_states; ++i_state) {
        states[i_state] = x[state_index(0, i_state)];
    }
    VectorXa controls(m_num_controls);
    for (int i_control = 0; i_control < m_num_controls; ++i_control) {
        controls[i_control] = x[control_index(0, i_control)];
    }
    // Evaluate integral cost at the initial time.
    adouble integrand_value = 0;
    // TODO avoid duplication here. Use lambda function?
    m_problem->integral_cost(m_initial_time, states, controls,
            integrand_value);
    obj_value = integrand_value;

    for (int i_mesh = 1; i_mesh < m_num_mesh_points; ++i_mesh) {
        for (int i_state = 0; i_state < m_num_states; ++i_state) {
            states[i_state] = x[state_index(i_mesh, i_state)];
        }
        for (int i_control = 0; i_control < m_num_controls; ++i_control) {
            controls[i_control] = x[control_index(i_mesh, i_control)];
        }
        integrand_value = 0;
        m_problem->integral_cost(step_size * i_mesh + m_initial_time,
                states, controls, integrand_value);
        obj_value += step_size * integrand_value;
// TODO use more intelligent quadrature.
    }
}

void EulerTranscription::constraints(const VectorXa& x,
        Ref<VectorXa> constraints) const {
    // TODO parallelize.
    const double step_size = (m_final_time - m_initial_time) /
            (m_num_mesh_points - 1);

    // TODO tradeoff between memory and parallelism.

    // Dynamics.
    // =========

    // Obtain state derivatives at each mesh point.
    // --------------------------------------------
    // TODO these can be Matrix in the future.
    //std::vector<std::vector<adouble>> m_states_trajectory;
    //std::vector<std::vector<adouble>> m_controls_trajectory;
    // TODO storing 1 too many derivatives trajectory; don't need the first
    // xdot (at t0).
    // We have N vectors; each one has length num_states.
    MatrixXa derivatives_trajectory(m_num_states, m_num_mesh_points);
    //std::vector<std::vector<adouble>>
    //        states_trajectory(m_num_mesh_points, {num_states});
    for (int i_mesh_point = 0; i_mesh_point < m_num_mesh_points;
         ++i_mesh_point) {
        // Get the states and controls for this mesh point.
        // TODO prefer having a view, not copying.
        VectorXa states(m_num_states);
        //const auto& states = states_trajectory[i_mesh_point];
        for (int i_state = 0; i_state < m_num_states; ++i_state) {
            states[i_state] = x[state_index(i_mesh_point, i_state)];
        }
        VectorXa controls(m_num_controls);
        for (int i_control = 0; i_control < m_num_controls; ++i_control) {
            controls[i_control] = x[control_index(i_mesh_point, i_control)];
        }
        m_problem->dynamics(states, controls,
                derivatives_trajectory.col(i_mesh_point));
    }

    // Bounds on initial and final states and controls.
    // ------------------------------------------------
    for (int i_state = 0; i_state < m_num_states; ++i_state) {
        constraints[constraint_bound_index(InitialStates, i_state)] =
                x[state_index(0, i_state)];
    }
    // TODO separate loops might help avoid cache misses, based on the
    // orer of the constraint indices.
    for (int i_state = 0; i_state < m_num_states; ++i_state) {
        constraints[constraint_bound_index(FinalStates, i_state)] =
                x[state_index(m_num_mesh_points - 1, i_state)];
    }
    for (int i_control = 0; i_control < m_num_controls; ++i_control) {
        constraints[constraint_bound_index(InitialControls, i_control)] =
                x[control_index(0, i_control)];
    }
    for (int i_control = 0; i_control < m_num_controls; ++i_control) {
        constraints[constraint_bound_index(FinalControls, i_control)] =
                x[control_index(m_num_mesh_points - 1, i_control)];
    }

    // Compute constraint defects.
    // ---------------------------
    for (int i_mesh = 1; i_mesh < m_num_mesh_points; ++i_mesh) {
        // defect_i = x_i - (x_{i-1} + h * xdot_i)  for i = 1, ..., N.
        //const auto& states_i = states_trajectory[i_mesh];
        //const auto& states_im1 = states_trajectory[i_mesh - 1];
        const auto& derivatives_i = derivatives_trajectory.col(i_mesh);
        // TODO temporary:
        assert(derivatives_i.size() == (unsigned)m_num_states);
        for (int i_state = 0; i_state < m_num_states; ++i_state) {
            // TODO do vector math here.
            const auto& state_i =  x[state_index(i_mesh, i_state)];
            const auto& state_im1 = x[state_index(i_mesh - 1, i_state)];
            constraints[constraint_index(i_mesh, i_state)] =
                    state_i - (state_im1 + step_size * derivatives_i[i_state]);
        }
        // TODO this would be so much easier with a matrix library.
    }
}

void EulerTranscription::interpret_iterate(const VectorXd& x,
        MatrixXd& states_trajectory,
        MatrixXd& controls_trajectory) const
{
    states_trajectory.resize(m_num_states, m_num_mesh_points);
    controls_trajectory.resize(m_num_controls, m_num_mesh_points);

    for (int i_mesh = 0; i_mesh < m_num_mesh_points; ++i_mesh) {
        auto states = states_trajectory.col(i_mesh);
        for (int i_state = 0; i_state < m_num_states; ++i_state) {
            states[i_state] = x[state_index(i_mesh, i_state)];
        }
        auto controls = controls_trajectory.col(i_mesh);
        for (int i_control = 0; i_control < m_num_controls; ++i_control) {
            controls[i_control] = x[control_index(i_mesh, i_control)];
        }
    }
}


