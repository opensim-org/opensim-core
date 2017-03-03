#ifndef MESH_DIRECTCOLLOCATION_HPP
#define MESH_DIRECTCOLLOCATION_HPP

#include "DirectCollocation.h"
#include "OptimalControlProblem.h"
#include "OptimizationSolver.h"
#include "SNOPTSolver.h"
#include "IpoptSolver.h"

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
    m_num_time_variables = 2;
    int num_variables = m_num_time_variables
            + m_num_mesh_points * m_num_continuous_variables;
    this->set_num_variables(num_variables);
    int num_bound_constraints = 2 * m_num_continuous_variables;
    m_num_defects = m_num_mesh_points - 1;
    m_num_dynamics_constraints = m_num_defects * m_num_states;
    m_num_path_constraints = m_ocproblem->num_path_constraints();
    // TODO rename..."total_path_constraints"?
    int num_path_traj_constraints = m_num_mesh_points * m_num_path_constraints;
    int num_constraints = num_bound_constraints + m_num_dynamics_constraints +
            num_path_traj_constraints;
    this->set_num_constraints(num_constraints);

    // Bounds.
    double initial_time_lower;
    double initial_time_upper;
    double final_time_lower;
    double final_time_upper;
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
    VectorXd path_constraints_lower(m_num_path_constraints);
    VectorXd path_constraints_upper(m_num_path_constraints);
    m_ocproblem->bounds(initial_time_lower, initial_time_upper,
                        final_time_lower, final_time_upper,
                        states_lower, states_upper,
                        initial_states_lower, initial_states_upper,
                        final_states_lower, final_states_upper,
                        controls_lower, controls_upper,
                        initial_controls_lower, initial_controls_upper,
                        final_controls_lower, final_controls_upper,
                        path_constraints_lower, path_constraints_upper);
    // TODO validate sizes.
    //m_initial_time = initial_time; // TODO make these variables.
    //m_final_time = final_time;
    // Bounds on variables.
    VectorXd variable_lower(num_variables);
    variable_lower[0] = initial_time_lower;
    variable_lower[1] = final_time_lower;
    variable_lower.tail(num_variables - m_num_time_variables) =
            (VectorXd(m_num_continuous_variables)
                    << states_lower, controls_lower)
                    .finished()
                    .replicate(m_num_mesh_points, 1);
    VectorXd variable_upper(num_variables);
    variable_upper[0] = initial_time_upper;
    variable_upper[1] = final_time_upper;
    variable_upper.tail(num_variables - m_num_time_variables) =
            (VectorXd(m_num_continuous_variables)
                    << states_upper, controls_upper)
                    .finished()
                    .replicate(m_num_mesh_points, 1);
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
    constraint_lower << initial_states_lower,
            initial_controls_lower,
            final_states_lower,
            final_controls_lower,
            dynamics_bounds,
            path_constraints_traj_lower;
    constraint_upper << initial_states_upper,
            initial_controls_upper,
            final_states_upper,
            final_controls_upper,
            dynamics_bounds,
            path_constraints_traj_upper;
    this->set_constraint_bounds(constraint_lower, constraint_upper);
    // TODO won't work if the bounds don't include zero!
    // TODO set_initial_guess(std::vector<double>(num_variables)); // TODO user
    // input

    // Set the mesh.
    // -------------
    const unsigned num_mesh_intervals = m_num_mesh_points - 1;
    // For integrating the integral cost.
    // The duration of each mesh interval.
    VectorXd mesh = VectorXd::LinSpaced(m_num_mesh_points, 0, 1);
    VectorXd mesh_intervals = mesh.tail(num_mesh_intervals)
                            - mesh.head(num_mesh_intervals);
    m_trapezoidal_quadrature_coefficients = VectorXd::Zero(m_num_mesh_points);
    // Betts 2010 equation 4.195, page 169.
    // b = 0.5 * [tau0, tau0 + tau1, tau1 + tau2, ..., tauM-2 + tauM-1, tauM-1]
    m_trapezoidal_quadrature_coefficients.head(num_mesh_intervals) =
            0.5 * mesh_intervals;
    m_trapezoidal_quadrature_coefficients.tail(num_mesh_intervals) +=
            0.5 * mesh_intervals;

    m_ocproblem->initialize_on_mesh(mesh);
}

template<typename T>
void LowOrder<T>::objective(const VectorX<T>& x, T& obj_value) const
{
    // TODO move this to a "make_variables_view()"
    const T& initial_time = x[0];
    const T& final_time = x[1];
    const T duration = final_time - initial_time;
    const T step_size = duration / (m_num_mesh_points - 1);

    // TODO I don't actually need to make a new view each time; just change the
    // data pointer. TODO probably don't even need to update the data pointer!
    auto states = make_states_trajectory_view(x);
    auto controls = make_controls_trajectory_view(x);

    // Endpoint cost.
    // --------------
    // TODO does this cause the final_states to get copied?
    m_ocproblem->endpoint_cost(final_time, states.rightCols(1), obj_value);


    // Integral cost.
    // --------------
    // TODO reuse memory; don't allocate every time.
    VectorX<T> integrand = VectorX<T>::Zero(m_num_mesh_points);
    // TODO parallelize.
    for (int i_mesh = 0; i_mesh < m_num_mesh_points; ++i_mesh) {
        const T time = step_size * i_mesh + initial_time;
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
    T integral_cost = 0;
    for (int i_mesh = 0; i_mesh < m_num_mesh_points; ++i_mesh) {
        integral_cost += m_trapezoidal_quadrature_coefficients[i_mesh] *
                integrand[i_mesh];
    }
    // The quadrature coefficients are fractions of the duration; multiply
    // by duration to get the correct units.
    integral_cost *= duration;
    obj_value += integral_cost;
}

template<typename T>
void LowOrder<T>::constraints(const VectorX<T>& x,
        Eigen::Ref<VectorX<T>> constraints) const
{
    // TODO parallelize.
    const T& initial_time = x[0];
    const T& final_time = x[1];
    const T duration = final_time - initial_time;
    const T step_size = duration / (m_num_mesh_points - 1);

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

    // Dynamics and path constraints.
    // ==============================
    // "Continuous function"

    // Obtain state derivatives at each mesh point.
    // --------------------------------------------
    // TODO storing 1 too many derivatives trajectory; don't need the first
    // xdot (at t0).
    // TODO tradeoff between memory and parallelism.
    // TODO reuse this memory!!!!!! Don't allocate every time!!!
    MatrixX<T> derivs(m_num_states, m_num_mesh_points);
    MatrixX<T> path_constraints(m_num_path_constraints, m_num_mesh_points);
    for (int i_mesh = 0; i_mesh < m_num_mesh_points; ++i_mesh) {
        // TODO should pass the time.
        const T time = step_size * i_mesh + initial_time;
        m_ocproblem->dynamics(states.col(i_mesh), controls.col(i_mesh),
                              derivs.col(i_mesh));
        m_ocproblem->path_constraints(i_mesh, time,
                                      states.col(i_mesh), controls.col(i_mesh),
                                      path_constraints.col(i_mesh));
    }

    // Compute constraint defects.
    // ---------------------------
    // defect_i = x_i - (x_{i-1} + h * xdot_i)  for i = 1, ..., N.
    // TODO this is backward Euler; do we want forward Euler?
    const unsigned N = m_num_mesh_points;
    const auto& x_i = states.rightCols(N - 1);
    const auto& x_im1 = states.leftCols(N - 1);
    const auto& xdot_i = derivs.rightCols(N - 1);
    const auto& h = step_size;
    //constr_view.defects = x_i-(x_im1+h*xdot_i);
    // TODO Trapezoidal:
    const auto& xdot_im1 = derivs.leftCols(N-1);
    //constr_view.defects = x_i-(x_im1+h*xdot_im1);
    constr_view.defects = x_i - (x_im1 + 0.5 * h * (xdot_i + xdot_im1));
    //for (int i_mesh = 0; i_mesh < N - 1; ++i_mesh) {
    //    const auto& h = m_mesh_intervals[i_mesh];
    //    constr_view.defects.col(i_mesh) = x_i.col(i_mesh)
    //            - (x_im1.col(i_mesh) + 0.5 * h * duration * (xdot_i.col
    // (i_mesh) + xdot_im1.col(i_mesh)));
    //}

    // Store path constraints.
    constr_view.path_constraints = path_constraints;
}

template<typename T>
typename Transcription<T>::Trajectory LowOrder<T>::
interpret_iterate(const Eigen::VectorXd& x) const
{
    const double& initial_time = x[0];
    const double& final_time = x[1];
    typename Transcription<T>::Trajectory traj;
    traj.time = Eigen::RowVectorXd::LinSpaced(m_num_mesh_points,
            initial_time, final_time);

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
            // Pointer to the start of the states.
            x.data() + m_num_time_variables,
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
            // Start of controls for first mesh interval.
            x.data() + m_num_time_variables + m_num_states,
            m_num_controls,          // Number of rows.
            m_num_mesh_points,       // Number of columns.
            // Distance between the start of each column; same as above.
            Eigen::OuterStride<Eigen::Dynamic>(m_num_states + m_num_controls));
}

template<typename T>
typename LowOrder<T>::ConstraintsView
LowOrder<T>::make_constraints_view(Eigen::Ref<VectorX<T>> constr) const
{
    // Starting indices of different parts of the constraints vector.
    const unsigned is = 0;                               // initial states.
    const unsigned ic = is + m_num_states;               // initial controls.
    const unsigned fs = ic + m_num_controls;             // final states.
    const unsigned fc = fs + m_num_states;               // final controls.
    const unsigned d  = fc + m_num_controls;             // defects.
    T* pc_ptr= m_num_path_constraints ?                  // path constraints.
               &constr[d + m_num_dynamics_constraints] : nullptr;
    //const unsigned pc =  // path
    // constraints.
    return ConstraintsView(StatesView(&constr[is], m_num_states),
            ControlsView(&constr[ic], m_num_controls),
            StatesView(&constr[fs], m_num_states),
            ControlsView(&constr[fc], m_num_controls),
            DefectsTrajectoryView(&constr[d], m_num_states, m_num_defects),
            PathConstraintsTrajectoryView(pc_ptr, m_num_path_constraints,
                                          m_num_mesh_points));
}

} // namespace transcription
} // namespace mesh

#endif // MESH_DIRECTCOLLOCATION_HPP
