#ifndef MESH_DIRECTCOLLOCATION_HPP
#define MESH_DIRECTCOLLOCATION_HPP

#include "DirectCollocation.h"
#include "OptimalControlProblem.h"
#include "OptimizationSolver.h"
#include "SNOPTSolver.h"

namespace mesh {

template<typename T>
DirectCollocationSolver<T>::DirectCollocationSolver(
        std::shared_ptr<const OCProblem> ocproblem,
        const std::string& transcrip,
        const std::string& optsolver,
        const unsigned& num_mesh_points)
{
    std::locale loc;
    std::string transcrip_lower = transcrip;
    std::transform(transcrip_lower.begin(), transcrip_lower.end(),
            transcrip_lower.begin(), ::tolower);
    if (transcrip_lower == "trapezoidal") {
        m_transcription.reset(new transcription::LowOrder<T>(ocproblem,
                                                             num_mesh_points));
    } else {
        throw std::runtime_error("Unrecognized transcription method '" +
                transcrip + "'.");
    }

    std::string optsolver_lower = optsolver;
    std::transform(optsolver_lower.begin(), optsolver_lower.end(),
            optsolver_lower.begin(), ::tolower);
    if (optsolver_lower == "ipopt") {
        // TODO this may not be good for IpoptSolver; IpoptSolver should
        // have a shared_ptr??
        m_optsolver.reset(new IpoptSolver(*m_transcription.get()));
    } else if (optsolver_lower == "snopt") {
        m_optsolver.reset(new SNOPTSolver(*m_transcription.get()));
    } else {
        throw std::runtime_error("Unrecognized optimization solver '" +
                optsolver + "'.");
    }
}

template<typename T>
OptimalControlSolution DirectCollocationSolver<T>::solve() const
{
    Eigen::VectorXd variables;
    double obj_value = m_optsolver->optimize(variables);
    // TODO
    using Trajectory = typename transcription::LowOrder<T>::Trajectory;
    Trajectory traj = m_transcription->interpret_iterate(variables);
    OptimalControlSolution solution;
    solution.time = traj.time;
    solution.states = traj.states;
    solution.controls = traj.controls;
    solution.objective = obj_value;
    return solution;
}

namespace transcription {

template<typename T>
void LowOrder<T>::set_ocproblem(
        std::shared_ptr<const OCProblem> ocproblem)
{
    m_ocproblem = ocproblem;
    m_num_states = m_ocproblem->num_states();
    m_num_controls = m_ocproblem->num_controls();
    m_num_continuous_variables = m_num_states+m_num_controls;
    int num_variables = m_num_mesh_points*m_num_continuous_variables;
    this->set_num_variables(num_variables);
    int num_bound_constraints = 2*m_num_continuous_variables;
    m_num_defects = m_num_mesh_points-1;
    int num_dynamics_constraints = (m_num_defects)*m_num_states;
    int num_constraints = num_bound_constraints+num_dynamics_constraints;
    this->set_num_constraints(num_constraints);

    // Bounds.
    double initial_time;
    double final_time;
    // TODO these could be fixed sizes for certain types of problems.
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
            initial_controls_lower,
            final_states_lower,
            final_controls_lower,
            dynamics_bounds;
    constraint_upper << initial_states_upper,
            initial_controls_upper,
            final_states_upper,
            final_controls_upper,
            dynamics_bounds;
    this->set_constraint_bounds(constraint_lower, constraint_upper);
    // TODO won't work if the bounds don't include zero!
    // TODO set_initial_guess(std::vector<double>(num_variables)); // TODO user
    // input
    const double step_size = (m_final_time-m_initial_time)/
            (m_num_mesh_points-1);
    // For integrating the integral cost.
    const unsigned num_mesh_intervals = m_num_mesh_points-1;
    // The duration of each mesh interval.
    VectorXd mesh_intervals =
            VectorXd::Constant(num_mesh_intervals, step_size);
    m_trapezoidal_quadrature_coefficients = VectorXd::Zero(m_num_mesh_points);
    // Betts 2010 equation 4.195, page 169.
    m_trapezoidal_quadrature_coefficients.head(num_mesh_intervals) =
            0.5*mesh_intervals;
    m_trapezoidal_quadrature_coefficients.tail(num_mesh_intervals) =
            0.5*mesh_intervals;
}

template<typename T>
void LowOrder<T>::objective(const VectorX<T>& x, T& obj_value) const
{
    const double step_size = (m_final_time-m_initial_time)/
            (m_num_mesh_points-1);

    // TODO I don't actually need to make a new view each time; just change the
    // data pointer. TODO probably don't even need to update the data pointer!
    auto states = make_states_trajectory_view(x);
    auto controls = make_controls_trajectory_view(x);

    // Endpoint cost.
    // --------------
    // TODO does this cause the final_states to get copied?
    m_ocproblem->endpoint_cost(m_final_time, states.rightCols(1), obj_value);


    // Integral cost.
    // --------------
    // TODO reuse memory; don't allocate every time.
    VectorX<T> integrand = VectorX<T>::Zero(m_num_mesh_points);
    // TODO parallelize.
    for (int i_mesh = 0; i_mesh<m_num_mesh_points; ++i_mesh) {
        const double time = step_size*i_mesh+m_initial_time;
        m_ocproblem->integral_cost(time,
                states.col(i_mesh), controls.col(i_mesh), integrand[i_mesh]);
    }
    // TODO use more intelligent quadrature? trapezoidal rule?
    // Rectangle rule:
    //obj_value = integrand[0]
    //        + step_size * integrand.tail(m_num_mesh_points - 1).sum();
    // The left vector is of type T b/c the dot product requires the same type.
    // TODO the following doesn't work because of different numerical types.
    // obj_value = m_trapezoidal_quadrature_coefficients.dot(integrand);
    for (int i_mesh = 0; i_mesh<m_num_mesh_points; ++i_mesh) {
        obj_value += m_trapezoidal_quadrature_coefficients[i_mesh] *
                integrand[i_mesh];
    }
}

template<typename T>
void LowOrder<T>::constraints(const VectorX<T>& x,
        Eigen::Ref<VectorX<T>> constraints) const
{
    // TODO parallelize.
    const double step_size = (m_final_time-m_initial_time)/
            (m_num_mesh_points-1);

    auto states = make_states_trajectory_view(x);
    auto controls = make_controls_trajectory_view(x);

    // Organize the constraints vector.
    ConstraintsView constr_view = make_constraints_view(constraints);

    // Bounds on initial and final states and controls.
    // ================================================
    // TODO order these 4 assignments to reduce cache misses,
    // based on the order of the constraint indices.
    constr_view.initial_states = states.col(0);
    constr_view.initial_controls = controls.col(0);
    constr_view.final_states = states.rightCols(1);
    constr_view.final_controls = controls.rightCols(1);

    // Dynamics.
    // =========

    // Obtain state derivatives at each mesh point.
    // --------------------------------------------
    // TODO storing 1 too many derivatives trajectory; don't need the first
    // xdot (at t0).
    // TODO tradeoff between memory and parallelism.
    // TODO reuse this memory!!!!!! Don't allocate every time!!!
    MatrixX<T> derivs(m_num_states, m_num_mesh_points);
    for (int i_mesh = 0; i_mesh < m_num_mesh_points; ++i_mesh) {
        m_ocproblem->dynamics(states.col(i_mesh), controls.col(i_mesh),
                derivs.col(i_mesh));
    }

    // Compute constraint defects.
    // ---------------------------
    // defect_i = x_i - (x_{i-1} + h * xdot_i)  for i = 1, ..., N.
    // TODO this is backward Euler; do we want forward Euler?
    const unsigned N = m_num_mesh_points;
    const auto& x_i = states.rightCols(N-1);
    const auto& x_im1 = states.leftCols(N-1);
    const auto& xdot_i = derivs.rightCols(N-1);
    const auto& h = step_size;
    //constr_view.defects = x_i-(x_im1+h*xdot_i);
    // TODO Trapezoidal:
    const auto& xdot_im1 = derivs.leftCols(N-1);
    //constr_view.defects = x_i-(x_im1+h*xdot_im1);
    constr_view.defects = x_i - (x_im1 + 0.5 * h * (xdot_i + xdot_im1));
}

template<typename T>
typename Transcription<T>::Trajectory LowOrder<T>::
interpret_iterate(const Eigen::VectorXd& x) const
{
    typename Transcription<T>::Trajectory traj;
    traj.time = Eigen::RowVectorXd::LinSpaced(m_num_mesh_points,
            m_initial_time, m_final_time);

    traj.states = this->make_states_trajectory_view(x);
    traj.controls = this->make_controls_trajectory_view(x);

    return traj;
}

template<typename T>
template<typename S>
LowOrder<T>::template TrajectoryView<S>
LowOrder<T>::make_states_trajectory_view(const VectorX<S>& x) const
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
LowOrder<T>::template TrajectoryView<S>
LowOrder<T>::make_controls_trajectory_view(const VectorX<S>& x) const
{
    return TrajectoryView<S>(
            x.data() + m_num_states, // Skip over the states for i_mesh = 0.
            m_num_controls,          // Number of rows.
            m_num_mesh_points,       // Number of columns.
            // Distance between the start of each column; same as above.
            Eigen::OuterStride<Eigen::Dynamic>(m_num_states + m_num_controls));
}

template<typename T>
typename LowOrder<T>::ConstraintsView
LowOrder<T>::make_constraints_view(Eigen::Ref<VectorX<T>> constr)
const
{
    // Starting indices of different parts of the constraints vector.
    const unsigned is = 0;                   // initial states.
    const unsigned ic = is + m_num_states;   // initial controls.
    const unsigned fs = ic + m_num_controls; // final states.
    const unsigned fc = fs + m_num_states;   // final controls.
    const unsigned d  = fc + m_num_controls; // defects.
    return ConstraintsView(StatesView(&constr[is], m_num_states),
            ControlsView(&constr[ic], m_num_controls),
            StatesView(&constr[fs], m_num_states),
            ControlsView(&constr[fc], m_num_controls),
            DefectsTrajectoryView(&constr[d], m_num_states, m_num_defects));
}

} // namespace transcription
} // namespace mesh

#endif // MESH_DIRECTCOLLOCATION_HPP
