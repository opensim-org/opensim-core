
#include "mesh.h"

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include <adolc/adolc.h>
#include <adolc/sparse/sparsedrivers.h>
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Ipopt::Index;
using Ipopt::Number;


// TODO move elsewhere.
class ToyProblem : public IpoptADOLC_OptimizationProblem {
public:
    ToyProblem() : IpoptADOLC_OptimizationProblem(2, 1) {}
    void objective(const std::vector<adouble>& x,
                   adouble& obj_value) const override {
        obj_value = (x[0] - 1.5) * (x[0] - 1.5)
                  + (x[1] + 2.0) * (x[1] + 2.0);
    }
    void constraints(const std::vector<adouble>& x,
                     std::vector<adouble>& constraints) const override {
        constraints[0] = x[1] - x[0] * x[0]; //x[0] + x[1];
    }
};

void DirectCollocationSolver::set_problem(std::shared_ptr<Problem> problem) {
    m_problem = problem;
    m_num_states = m_problem->num_states();
    m_num_controls = m_problem->num_controls();
    m_num_continuous_variables = m_num_states + m_num_controls;
    int num_variables = m_num_mesh_points * m_num_continuous_variables;
    set_num_variables(num_variables);
    int num_bound_constraints = 2 * m_num_continuous_variables;
    int num_dynamics_constraints = (m_num_mesh_points - 1) * m_num_states;
    set_num_constraints(num_bound_constraints + num_dynamics_constraints);

    // Bounds.
    double initial_time;
    double final_time;
    std::vector<double> states_lower;
    std::vector<double> states_upper;
    std::vector<double> initial_states_lower;
    std::vector<double> initial_states_upper;
    std::vector<double> final_states_lower;
    std::vector<double> final_states_upper;
    std::vector<double> controls_lower;
    std::vector<double> controls_upper;
    std::vector<double> initial_controls_lower;
    std::vector<double> initial_controls_upper;
    std::vector<double> final_controls_lower;
    std::vector<double> final_controls_upper;
    m_problem->bounds(initial_time, final_time,
                      states_lower, states_upper,
                      initial_states_lower, initial_states_upper,
                      final_states_lower, final_states_upper,
                      controls_lower, controls_upper,
                      initial_controls_lower, initial_controls_upper,
                      final_controls_lower, final_controls_upper);
    m_initial_time = initial_time; // TODO make these variables.
    m_final_time = final_time;
    // Bounds on variables.
    std::vector<double> variable_lower;
    std::vector<double> variable_upper;
    for (int i_mesh = 0; i_mesh < m_num_mesh_points; ++i_mesh) {
        // TODO handle redundant constraints
        // (with the initial and final bounds).
        variable_lower.insert(variable_lower.end(),
                              states_lower.begin(), states_lower.end());
        variable_lower.insert(variable_lower.end(),
                              controls_lower.begin(), controls_lower.end());
        variable_upper.insert(variable_upper.end(),
                              states_upper.begin(), states_upper.end());
        variable_upper.insert(variable_upper.end(),
                              controls_upper.begin(), controls_upper.end());
    }
    set_variable_bounds(variable_lower, variable_upper);
    // Bounds for constraints.
    std::vector<double> constraint_lower;
    std::vector<double> constraint_upper;
    // Defects must be 0.
    std::vector<double> dynamics_bounds(num_dynamics_constraints, 0);
    // Lower bounds.
    constraint_lower.insert(constraint_lower.end(),
                            initial_states_lower.begin(),
                            initial_states_lower.end());
    constraint_lower.insert(constraint_lower.end(),
                            final_states_lower.begin(),
                            final_states_lower.end());
    constraint_lower.insert(constraint_lower.end(),
                            initial_controls_lower.begin(),
                            initial_controls_lower.end());
    constraint_lower.insert(constraint_lower.end(),
                            final_controls_lower.begin(),
                            final_controls_lower.end());
    constraint_lower.insert(constraint_lower.end(),
                            dynamics_bounds.begin(), dynamics_bounds.end());
    // Upper bounds.
    constraint_upper.insert(constraint_upper.end(),
                            initial_states_upper.begin(),
                            initial_states_upper.end());
    constraint_upper.insert(constraint_upper.end(),
                            final_states_upper.begin(),
                            final_states_upper.end());
    constraint_upper.insert(constraint_upper.end(),
                            initial_controls_upper.begin(),
                            initial_controls_upper.end());
    constraint_upper.insert(constraint_upper.end(),
                            final_controls_upper.begin(),
                            final_controls_upper.end());
    constraint_upper.insert(constraint_upper.end(),
                            dynamics_bounds.begin(), dynamics_bounds.end());
    set_constraint_bounds(constraint_lower, constraint_upper);
    set_initial_guess(std::vector<double>(num_variables)); // TODO user input
}

// TODO make these inline.
int DirectCollocationSolver::state_index(int i_mesh_point, int i_state) const {
    return i_mesh_point * m_num_continuous_variables + i_state;
}

int DirectCollocationSolver::control_index(int i_mesh_point, int i_control) const {
    return i_mesh_point * m_num_continuous_variables
           + i_control + m_num_states;
}

int DirectCollocationSolver::constraint_index(int i_mesh, int i_state) const {
    int num_bound_constraints = 2 * m_num_continuous_variables;
    return num_bound_constraints + (i_mesh - 1) * m_num_states + i_state;
}

int DirectCollocationSolver::constraint_bound_index(BoundsCategory category,
                                                    int index) const {
    if (category <= 1) {
        assert(index < m_num_states);
        return category * m_num_states + index;
    }
    assert(index < m_num_controls);
    return 2 * m_num_states + (category - 2) * m_num_controls + index;
}

void DirectCollocationSolver::objective(const std::vector<adouble>& x,
                                        adouble& obj_value) const {
    const double step_size = (m_final_time - m_initial_time) /
                             (m_num_mesh_points - 1);

// Create states and controls vectors.
// TODO remove when using Eigen.
    std::vector<adouble> states(m_num_states);
    for (int i_state = 0; i_state < m_num_states; ++i_state) {
        states[i_state] = x[state_index(0, i_state)];
    }
    std::vector<adouble> controls(m_num_controls);
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
        std::vector<adouble> controls(m_num_controls);
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
void DirectCollocationSolver::constraints(const std::vector<adouble>& x,
                                          std::vector<adouble>& constraints) const {
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
    std::vector<std::vector<adouble>>
            derivatives_trajectory(m_num_mesh_points,
                                   std::vector<adouble>(m_num_states));
//std::vector<std::vector<adouble>>
//        states_trajectory(m_num_mesh_points, {num_states});
    for (int i_mesh_point = 0; i_mesh_point < m_num_mesh_points;
         ++i_mesh_point) {
// Get the states and controls for this mesh point.
// TODO prefer having a view, not copying.
        std::vector<adouble> states(m_num_states);
//const auto& states = states_trajectory[i_mesh_point];
        for (int i_state = 0; i_state < m_num_states; ++i_state) {
            states[i_state] = x[state_index(i_mesh_point, i_state)];
        }
        std::vector<adouble> controls(m_num_controls);
        for (int i_control = 0; i_control < m_num_controls; ++i_control) {
            controls[i_control] = x[control_index(i_mesh_point, i_control)];
        }
        auto& derivatives = derivatives_trajectory[i_mesh_point];
        m_problem->dynamics(states, controls, derivatives);
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
        const auto& derivatives_i = derivatives_trajectory[i_mesh];
// TODO temporary:
        assert(derivatives_i.size() == (unsigned)m_num_states);
        for (int i_state = 0; i_state < m_num_states; ++i_state) {
            const auto& state_i =  x[state_index(i_mesh, i_state)];
            const auto& state_im1 = x[state_index(i_mesh - 1, i_state)];
            constraints[constraint_index(i_mesh, i_state)] =
                    state_i - (state_im1 + step_size * derivatives_i[i_state]);
        }
// TODO this would be so much easier with a matrix library.
    }
}

void DirectCollocationSolver::finalize_solution(Ipopt::SolverReturn /*TODO status*/,
                                                Index /*num_variables*/,
                                                const Number* x,
                                                const Number* /*z_L*/, const Number* /*z_U*/,
                                                Index /*num_constraints*/,
                                                const Number* /*g*/, const Number* /*lambda*/,
                                                Number obj_value, const Ipopt::IpoptData* /*ip_data*/,
                                                Ipopt::IpoptCalculatedQuantities* /*ip_cq*/) {
    for (int i_state = 0; i_state < m_num_states; ++i_state) {
        printf("\nTrajectory of state variable %i\n", i_state);
        for (int i_mesh = 0; i_mesh < m_num_mesh_points; ++i_mesh) {
            printf("[%d]: %e\n", i_mesh, x[state_index(i_mesh, i_state)]);
        }
    }
    for (int i_control = 0; i_control < m_num_controls; ++i_control) {
        printf("\nTrajectory of control variable %i\n", i_control);
        for (int i_mesh = 0; i_mesh < m_num_mesh_points; ++i_mesh) {
            printf("[%d]: %e\n", i_mesh,
                   x[control_index(i_mesh, i_control)]);
        }
    }
    std::ofstream f("solution.csv");
    double time;
    double step_size = (m_final_time - m_initial_time) /
                       (m_num_mesh_points - 1);
    f << "time";
    for (int i_state = 0; i_state < m_num_states; ++i_state) {
        f << ",state" << i_state;
    }
    for (int i_control = 0; i_control < m_num_controls; ++i_control) {
        f << ",control" << i_control;
    }
    f << std::endl;
    for (int i_mesh = 0; i_mesh < m_num_mesh_points; ++i_mesh) {
        time = i_mesh * step_size + m_initial_time;
        f << time;
        for (int i_state = 0; i_state < m_num_states; ++i_state) {
            f << "," << x[state_index(i_mesh, i_state)];
        }
        for (int i_control = 0; i_control < m_num_controls; ++i_control) {
            f << "," << x[control_index(i_mesh, i_control)];
        }
        f << std::endl;
    }
    f.close();
    //printf("\nSolution of the bound multipliers, z_L and z_U\n");
    //for (Index i = 0; i < num_variables; ++i) {
    //    printf("z_L[%d] = %e\n", i, z_L[i]);
    //}
    //for (Index i = 0; i < num_variables; ++i) {
    //    printf("z_U[%d] = %e\n", i, z_U[i]);
    //}
    printf("\nObjective value\n");
    printf("f(x*) = %e\n", obj_value);
}
