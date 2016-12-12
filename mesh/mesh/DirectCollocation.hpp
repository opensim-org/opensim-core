#ifndef MESH_DIRECTCOLLOCATION_HPP
#define MESH_DIRECTCOLLOCATION_HPP

#include "DirectCollocation.h"
#include "OptimalControlProblem.h"

// TODO don't use these
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::Ref;

namespace mesh {

template<typename T>
void EulerTranscription<T>::set_ocproblem(
        std::shared_ptr<OCProblem> ocproblem) {
    m_ocproblem = ocproblem;
    m_num_states = m_ocproblem->num_states();
    m_num_controls = m_ocproblem->num_controls();
    m_num_continuous_variables = m_num_states + m_num_controls;
    int num_variables = m_num_mesh_points * m_num_continuous_variables;
    this->set_num_variables(num_variables);
    int num_bound_constraints = 2 * m_num_continuous_variables;
    int num_dynamics_constraints = (m_num_mesh_points - 1) * m_num_states;
    int num_constraints = num_bound_constraints + num_dynamics_constraints;
    this->set_num_constraints(num_constraints);

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
    m_ocproblem->bounds(initial_time, final_time,
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
    this->set_variable_bounds(variable_lower, variable_upper);
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
    this->set_constraint_bounds(constraint_lower, constraint_upper);
    // TODO won't work if the bounds don't include zero!
    // TODO set_initial_guess(std::vector<double>(num_variables)); // TODO user
    // input
}

template<typename T>
void EulerTranscription<T>::objective(const VectorX<T>& x, T& obj_value) const {
    const double step_size = (m_final_time - m_initial_time) /
            (m_num_mesh_points - 1);

    // TODO I don't actually need to make a new view each time; just change the
    // data pointer. TODO probably don't even need to update the data pointer!
    auto states = make_states_trajectory_view(x);
    auto controls = make_controls_trajectory_view(x);

    // TODO reuse memory; don't allocate every time.
    VectorX<T> integrand = VectorX<T>::Zero(m_num_mesh_points);
    // TODO parallelize.
    for (int i_mesh = 0; i_mesh < m_num_mesh_points; ++i_mesh) {
        const double time = step_size * i_mesh + m_initial_time;
        m_ocproblem->integral_cost(time,
                states.col(i_mesh), controls.col(i_mesh), integrand[i_mesh]);
    }
    // TODO use more intelligent quadrature? trapezoidal rule?
    // Rectangle rule:
    obj_value = integrand[0]
            + step_size * integrand.tail(m_num_mesh_points - 1).sum();
}

template<typename T>
void EulerTranscription<T>::constraints(const VectorX<T>& x,
        Ref<VectorX<T>> constraints) const {
    // TODO parallelize.
    const double step_size = (m_final_time - m_initial_time) /
            (m_num_mesh_points - 1);

    auto states = make_states_trajectory_view(x);
    auto controls = make_controls_trajectory_view(x);

    // Dynamics.
    // =========

    // Obtain state derivatives at each mesh point.
    // --------------------------------------------
    // TODO storing 1 too many derivatives trajectory; don't need the first
    // xdot (at t0).
    // TODO tradeoff between memory and parallelism.
    // TODO reuse this memory!!!!!! Don't allocate every time!!!
    MatrixX<T> derivatives_trajectory(m_num_states, m_num_mesh_points);
    for (int i_mesh = 0; i_mesh < m_num_mesh_points; ++i_mesh) {
        m_ocproblem->dynamics(states.col(i_mesh), controls.col(i_mesh),
                derivatives_trajectory.col(i_mesh));
    }

    // Bounds on initial and final states and controls.
    // ------------------------------------------------
    // TODO upgrade to use states view, controls view.
    for (int i_state = 0; i_state < m_num_states; ++i_state) {
        constraints[constraint_bound_index(InitialStates, i_state)] =
                x[state_index(0, i_state)];
    }
    // TODO separate loops might help avoid cache misses, based on the
    // order of the constraint indices.
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
        const auto& derivatives_i = derivatives_trajectory.col(i_mesh);
        // This is a writable view into the constraints vector:
        auto constraints_i = constraints.segment(
                constraint_index(i_mesh, 0), m_num_states);
        const auto& state_i = states.col(i_mesh);
        const auto& state_im1 = states.col(i_mesh - 1);
        constraints_i = state_i - (state_im1 + step_size * derivatives_i);
    }
}

template<typename T>
typename EulerTranscription<T>::Trajectory EulerTranscription<T>::
interpret_iterate(const VectorXd& x) const
{
    Trajectory traj;
    traj.time = RowVectorXd::LinSpaced(m_num_mesh_points,
            m_initial_time, m_final_time);

    traj.states = this->make_states_trajectory_view(x);
    traj.controls = this->make_controls_trajectory_view(x);

    return traj;
}

template<typename T>
template<typename S>
EulerTranscription<T>::template TrajectoryView<S>
EulerTranscription<T>::make_states_trajectory_view(const VectorX<S>& x) const
{
    return TrajectoryView<S>(
            x.data(),          // Pointer to the start of the data.
            m_num_states,      // Number of rows.
            m_num_mesh_points, // Number of columns.
            // Distance between the start of each column.
            Eigen::OuterStride<Eigen::Dynamic>(m_num_states + m_num_controls));
}

template<typename T>
template<typename S>
EulerTranscription<T>::template TrajectoryView<S>
EulerTranscription<T>::make_controls_trajectory_view(const VectorX<S>& x) const
{
    return TrajectoryView<S>(
            x.data() + m_num_states, // Skip over the states for i_mesh = 0.
            m_num_controls,          // Number of rows.
            m_num_mesh_points,       // Number of columns.
            // Distance between the start of each column; same as above.
            Eigen::OuterStride<Eigen::Dynamic>(m_num_states + m_num_controls));
}

} // namespace mesh

#endif // MESH_DIRECTCOLLOCATION_HPP
