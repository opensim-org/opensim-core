#ifndef MESH_DIRECTCOLLOCATION_HPP
#define MESH_DIRECTCOLLOCATION_HPP

#include "DirectCollocation.h"
#include "OptimalControlProblem.h"
#include "OptimizationSolver.h"
#include "SNOPTSolver.h"
#include "IpoptSolver.h"

#include <iomanip>

namespace mesh {

template<typename T>
DirectCollocationSolver<T>::DirectCollocationSolver(
        std::shared_ptr<const OCProblem> ocproblem,
        const std::string& transcrip,
        const std::string& optsolver,
        const unsigned& num_mesh_points)
        : m_ocproblem(ocproblem)
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
    return solve_internal(variables);
}

template<typename T>
OptimalControlSolution DirectCollocationSolver<T>::solve(
        const OptimalControlIterate& initial_guess) const
{
    // TODO support interpolating the guess so that it does not need to be on
    // the same mesh as specified for the optimal control problem.

    // TODO make sure the dimensions all make sense. num rows/cols.
    Eigen::VectorXd variables =
            m_transcription->construct_iterate(initial_guess);
    return solve_internal(variables);
}

template<typename T>
OptimalControlSolution DirectCollocationSolver<T>::solve_internal(
        Eigen::VectorXd& variables) const
{
    double obj_value = m_optsolver->optimize(variables);
    OptimalControlIterate traj =
            m_transcription->deconstruct_iterate(variables);
    OptimalControlSolution solution;
    solution.time = traj.time;
    solution.states = traj.states;
    solution.controls = traj.controls;
    solution.objective = obj_value;
    solution.state_names = m_ocproblem->get_state_names();
    solution.control_names = m_ocproblem->get_control_names();
    return solution;
}

template<typename T>
void DirectCollocationSolver<T>::print_constraint_values(
        const OptimalControlIterate& ocp_vars, std::ostream& stream) const
{
    m_transcription->print_constraint_values(ocp_vars, stream);
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

    // Allocate working memory.
    m_derivs.resize(m_num_states, m_num_mesh_points);
    m_path_constraints.resize(m_num_path_constraints, m_num_mesh_points);

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
    // xdot (at t0). (TODO I don't think this is true anymore).
    // TODO tradeoff between memory and parallelism.
    for (int i_mesh = 0; i_mesh < m_num_mesh_points; ++i_mesh) {
        // TODO should pass the time.
        const T time = step_size * i_mesh + initial_time;
        m_ocproblem->dynamics(states.col(i_mesh), controls.col(i_mesh),
                              m_derivs.col(i_mesh));
        m_ocproblem->path_constraints(i_mesh, time,
                                      states.col(i_mesh), controls.col(i_mesh),
                                      m_path_constraints.col(i_mesh));
    }

    // Compute constraint defects.
    // ---------------------------
    // defect_i = x_i - (x_{i-1} + h * xdot_i)  for i = 1, ..., N.
    // TODO this is backward Euler; do we want forward Euler?
    const unsigned N = m_num_mesh_points;
    const auto& x_i = states.rightCols(N - 1);
    const auto& x_im1 = states.leftCols(N - 1);
    const auto& xdot_i = m_derivs.rightCols(N - 1);
    const auto& h = step_size;
    //constr_view.defects = x_i-(x_im1+h*xdot_i);
    // TODO Trapezoidal:
    const auto& xdot_im1 = m_derivs.leftCols(N-1);
    //constr_view.defects = x_i-(x_im1+h*xdot_im1);
    constr_view.defects = x_i - (x_im1 + 0.5 * h * (xdot_i + xdot_im1));
    //for (int i_mesh = 0; i_mesh < N - 1; ++i_mesh) {
    //    const auto& h = m_mesh_intervals[i_mesh];
    //    constr_view.defects.col(i_mesh) = x_i.col(i_mesh)
    //            - (x_im1.col(i_mesh) + 0.5 * h * duration * (xdot_i.col
    // (i_mesh) + xdot_im1.col(i_mesh)));
    //}

    // Store path constraints.
    constr_view.path_constraints = m_path_constraints;
}

template<typename T>
Eigen::VectorXd LowOrder<T>::
construct_iterate(const OptimalControlIterate& traj) const
{
    // Check for errors with dimensions.
    if (traj.time.size() != m_num_mesh_points) {
        throw std::runtime_error("[mesh] Expected time to have " +
                std::to_string(m_num_mesh_points) + " elements, but it has " +
                std::to_string(traj.time.size()) + ".");
    }
    if (traj.states.rows() != m_num_states ||
            traj.states.cols() != m_num_mesh_points) {
        throw std::runtime_error("[mesh] Expected states to have dimensions " +
                std::to_string(m_num_states) + " x " +
                std::to_string(m_num_mesh_points) + " but it has dimensions " +
                std::to_string(traj.states.rows()) + " x " +
                std::to_string(traj.states.cols()) + ".");
    }
    if (traj.controls.rows() != m_num_controls ||
            traj.controls.cols() != m_num_mesh_points) {
        throw std::runtime_error("[mesh] Expected controls to have dimensions "
                + std::to_string(m_num_controls) + " x " +
                std::to_string(m_num_mesh_points) + " but it has dimensions " +
                std::to_string(traj.controls.rows()) + " x " +
                std::to_string(traj.controls.cols()) + ".");
    }

    Eigen::VectorXd iterate(this->get_num_variables());
    // Initial and final time.
    iterate[0] = traj.time[0];
    iterate[1] = traj.time.tail<1>()[0];
    // Create mutable views. This will probably fail miserably if the
    // dimensions do not match.
    this->make_states_trajectory_view(iterate) = traj.states;
    this->make_controls_trajectory_view(iterate) = traj.controls;
    return iterate;
}

template<typename T>
OptimalControlIterate LowOrder<T>::
deconstruct_iterate(const Eigen::VectorXd& x) const
{
    const double& initial_time = x[0];
    const double& final_time = x[1];
    OptimalControlIterate traj;
    traj.time = Eigen::RowVectorXd::LinSpaced(m_num_mesh_points,
                                              initial_time, final_time);

    traj.states = this->make_states_trajectory_view(x);
    traj.controls = this->make_controls_trajectory_view(x);

    return traj;
}

template<typename T>
void LowOrder<T>::
print_constraint_values(const OptimalControlIterate& ocp_vars,
                        std::ostream& stream) const
{
    // TODO also print_bounds() information.

    // Gather and organize all constraint values and bounds.
    VectorX<T> vars = construct_iterate(ocp_vars).template cast<T>();
    VectorX<T> constraint_values(this->get_num_constraints());
    constraints(vars, constraint_values);
    ConstraintsView values = make_constraints_view(constraint_values);

    // TODO avoid cast by templatizing make_constraints_view().
    VectorX<T> lower_T =
            this->get_constraint_lower_bounds().template cast<T>();
    ConstraintsView lower = make_constraints_view(lower_T);
    VectorX<T> upper_T =
            this->get_constraint_upper_bounds().template cast<T>();
    ConstraintsView upper = make_constraints_view(upper_T);
    auto state_names = m_ocproblem->get_state_names();
    auto control_names = m_ocproblem->get_control_names();

    // Find the longest state or control name.
    auto compare_size = [](const std::string& a, const std::string& b) {
        return a.size() < b.size();
    };
    int max_name_length = std::max(
            std::max_element(state_names.begin(), state_names.end(),
                             compare_size)->size(),
            std::max_element(control_names.begin(), control_names.end(),
                             compare_size)->size());

    stream << "Total number of constraints: "
            << constraint_values.size() << "." << std::endl;
    stream << "L and U indicate which constraints are violated. " << std::endl;

    // Initial and final bounds on state and control variables.
    // --------------------------------------------------------
    auto print_ifbounds = [&stream, max_name_length](
            const std::string& description, Eigen::Map<VectorX<T>>& lower,
            Eigen::Map<VectorX<T>>& values, Eigen::Map<VectorX<T>>& upper,
            const std::vector<std::string>& names) {
        stream << "\n" << description << ": " << std::endl;
        stream << std::setw(max_name_length) << "  "
               << std::setw(9) << "lower" << "    "
               << std::setw(9) << "value" << "    "
               << std::setw(9) << "upper" << " " << std::endl;
        for (Eigen::Index i = 0; i < lower.size(); ++i) {
            // The nasty static cast is because operator<< for badouble is
            // undefined (though I see it in the ADOL-C source code).
            auto& L = static_cast<const double&>(lower[i]);
            auto& V = static_cast<const double&>(values[i]);
            auto& U = static_cast<const double&>(upper[i]);
            stream << std::setw(max_name_length) << names[i] << "  "
                   << std::setprecision(2) << std::scientific
                   << std::setw(9) << L << " <= "
                   << std::setw(9) << V << " <= "
                   << std::setw(9) << U << " ";
            // Show if the constraint is violated.
            if (L <= V) stream << " ";
            else        stream << "L";
            if (V <= U) stream << " ";
            else        stream << "U";
            stream << std::endl;
        }
    };
    print_ifbounds("Initial state constraints",
                   lower.initial_states, values.initial_states,
                   upper.initial_states, state_names);
    print_ifbounds("Initial control constraints",
                   lower.initial_controls, values.initial_controls,
                   upper.initial_controls, control_names);
    print_ifbounds("Final state constraints",
                   lower.final_states, values.final_states,
                   upper.final_states, state_names);
    print_ifbounds("Final control constraints",
                   lower.final_controls, values.final_controls,
                   upper.final_controls, control_names);

    // Differential equation defects.
    // ------------------------------
    stream << "\nDifferential equation defects:" << std::endl;
    stream << std::setw(max_name_length) << " " << "  norm across the mesh"
           << std::endl;
    std::string spacer(7, ' ');
    for (size_t i_state = 0; i_state < state_names.size(); ++i_state) {
        auto& norm = static_cast<const double&>(
                values.defects.row(i_state).norm());

        stream << std::setw(max_name_length) << state_names[i_state]
               << spacer
               << std::setprecision(2) << std::scientific << std::setw(9)
               << norm << std::endl;
    }

    // Path constraints.
    // -----------------
    stream << "\nPath constraints:" << std::endl;
    auto pathcon_names = m_ocproblem->get_path_constraint_names();

    int max_pathcon_name_length = std::max_element(pathcon_names.begin(),
                                                   pathcon_names.end(),
                                                   compare_size)->size();
    // stream << std::setw(max_pathcon_name_length) << " "
    //        << "  norm across the mesh" << std::endl;
    // for (size_t i_pc = 0; i_pc < pathcon_names.size(); ++i_pc) {
    //     auto& norm = static_cast<const double&>(
    //             values.path_constraints.row(i_pc).norm());
    //     stream << std::setw(max_pathcon_name_length) << pathcon_names[i_pc]
    //             << spacer
    //             << std::setprecision(2) << std::scientific << std::setw(9)
    //             << norm << std::endl;
    // }
    for (size_t i_mesh = 0; i_mesh < values.path_constraints.cols(); ++i_mesh) {

        stream << std::setw(4) << i_mesh << "  ";
        for (size_t i_pc = 0; i_pc < pathcon_names.size(); ++i_pc) {
            auto& value = static_cast<const double&>(
                    values.path_constraints(i_pc, i_mesh));
            stream << std::setprecision(2) << std::scientific << std::setw(9)
                   << value << "  ";
        }
        stream << std::endl;
    }
}

template<typename T>
template<typename S>
typename LowOrder<T>::template TrajectoryViewConst<S>
LowOrder<T>::make_states_trajectory_view(const VectorX<S>& x) const
{
    return {
            // Pointer to the start of the states.
            x.data() + m_num_time_variables,
            m_num_states,      // Number of rows.
            m_num_mesh_points, // Number of columns.
            // Distance between the start of each column.
            Eigen::OuterStride<Eigen::Dynamic>(m_num_states + m_num_controls)};
}

template<typename T>
template<typename S>
typename LowOrder<T>::template TrajectoryViewConst<S>
LowOrder<T>::make_controls_trajectory_view(const VectorX<S>& x) const
{
    return {
            // Start of controls for first mesh interval.
            x.data() + m_num_time_variables + m_num_states,
            m_num_controls,          // Number of rows.
            m_num_mesh_points,       // Number of columns.
            // Distance between the start of each column; same as above.
            Eigen::OuterStride<Eigen::Dynamic>(m_num_states + m_num_controls)};
}

// TODO avoid the duplication with the above.
template<typename T>
template<typename S>
typename LowOrder<T>::template TrajectoryView<S>
LowOrder<T>::make_states_trajectory_view(VectorX<S>& x) const
{
    return {
            // Pointer to the start of the states.
            x.data() + m_num_time_variables,
            m_num_states,      // Number of rows.
            m_num_mesh_points, // Number of columns.
            // Distance between the start of each column.
            Eigen::OuterStride<Eigen::Dynamic>(m_num_states + m_num_controls)};
}

template<typename T>
template<typename S>
typename LowOrder<T>::template TrajectoryView<S>
LowOrder<T>::make_controls_trajectory_view(VectorX<S>& x) const
{
    return {
            // Start of controls for first mesh interval.
            x.data() + m_num_time_variables + m_num_states,
            m_num_controls,          // Number of rows.
            m_num_mesh_points,       // Number of columns.
            // Distance between the start of each column; same as above.
            Eigen::OuterStride<Eigen::Dynamic>(m_num_states + m_num_controls)};
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
    return {StatesView(&constr[is], m_num_states),
            ControlsView(&constr[ic], m_num_controls),
            StatesView(&constr[fs], m_num_states),
            ControlsView(&constr[fc], m_num_controls),
            DefectsTrajectoryView(&constr[d], m_num_states, m_num_defects),
            PathConstraintsTrajectoryView(pc_ptr, m_num_path_constraints,
                                             m_num_mesh_points)};
}

} // namespace transcription
} // namespace mesh

#endif // MESH_DIRECTCOLLOCATION_HPP
