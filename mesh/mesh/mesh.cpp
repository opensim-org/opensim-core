
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


void IpoptADOLC_OptimizationProblem::
set_initial_guess(const std::vector<double>& guess) {
    // TODO should not be storing the solution at all.
    m_solution.clear();
    // TODO be smart about the need to copy "guess" (could be long)?
    m_initial_guess = guess;
    // TODO check their sizes.
    assert(guess.size() == m_num_variables);

    // Determine sparsity patterns.
    // ----------------------------
    // TODO remove from here and utilize new_x.
    // TODO or can I reuse the tape?
    {
        short int tag = 0;
        // =================================================================
        // START ACTIVE
        // -----------------------------------------------------------------
        trace_on(tag);
        std::vector<adouble> x_adouble(m_num_variables);
        for (unsigned i = 0; i < m_num_variables; ++i) x_adouble[i] <<= guess[i];
        std::vector<adouble> g_adouble(m_num_constraints);
        constraints(x_adouble, g_adouble);
        std::vector<double> g(m_num_constraints);
        for (unsigned i = 0; i < m_num_constraints; ++i) g_adouble[i] >>= g[i];
        trace_off();
        // -----------------------------------------------------------------
        // END ACTIVE
        // =================================================================

        int repeated_call = 0;
        int num_nonzeros = -1; /*TODO*/
        unsigned int* row_indices = NULL; // Allocated by ADOL-C.
        unsigned int* col_indices = NULL; // Allocated by ADOL-C.
        double* jacobian = NULL;          // Allocated by ADOL-C.
        int options[4];
        options[0] = 0; /*TODO*/
        options[1] = 0; /*TODO*/
        options[2] = 0; /*TODO*/
        options[3] = 0; /*TODO*/
        int success = sparse_jac(tag, m_num_constraints, m_num_variables,
                                 repeated_call, &guess[0],
                                 &num_nonzeros, &row_indices, &col_indices,
                                 &jacobian, options);
        assert(success);
        m_jacobian_num_nonzeros = num_nonzeros;
        m_jacobian_row_indices.reserve(num_nonzeros);
        m_jacobian_col_indices.reserve(num_nonzeros);
        for (int i = 0; i < num_nonzeros; ++i) {
            m_jacobian_row_indices[i] = row_indices[i];
            m_jacobian_col_indices[i] = col_indices[i];
        }
        delete [] row_indices;
        delete [] col_indices;
        delete [] jacobian;
    }

    {
        short int tag = 0;
        // =================================================================
        // START ACTIVE
        // -----------------------------------------------------------------
        trace_on(tag);
        std::vector<adouble> x_adouble(m_num_variables);
        std::vector<double> lambda_vector(m_num_constraints, 1);
        adouble lagrangian_adouble;
        double lagr;
        for (unsigned i = 0; i < m_num_variables; ++i) {
            x_adouble[i] <<= guess[i];
        }
        lagrangian(1.0, x_adouble, lambda_vector, lagrangian_adouble);
        lagrangian_adouble >>= lagr;
        trace_off();
        // -----------------------------------------------------------------
        // END ACTIVE
        // =================================================================
        // TODO efficiently use the "repeat" argument.
        int repeated_call = 0;
        int options[2];
        options[0] = 0; /* test the computational graph control flow? TODO*/
        options[1] = 0; /* way of recovery TODO */
        unsigned int* row_indices = NULL;
        unsigned int* col_indices = NULL;
        double* hessian = NULL; // We don't actually need the hessian...
        // TODO use hess_pat instead!!!
        int num_nonzeros;
        std::vector<double> x_and_lambda(m_num_variables + m_num_constraints);
        for (unsigned ivar = 0; ivar < m_num_variables; ++ivar) {
            x_and_lambda[ivar] = guess[ivar];
        }
        for (unsigned icon = 0; icon < m_num_constraints; ++icon) {
            x_and_lambda[icon + m_num_variables] = 1; // TODO consistency?
        }
        int success = sparse_hess(tag, m_num_variables, repeated_call,
                                  &guess[0], &num_nonzeros,
                                  &row_indices, &col_indices, &hessian,
                                  options);
        assert(success);
        m_hessian_num_nonzeros = num_nonzeros;
        m_hessian_row_indices.reserve(num_nonzeros);
        m_hessian_col_indices.reserve(num_nonzeros);
        for (int i = 0; i < num_nonzeros; ++i) {
            m_hessian_row_indices[i] = row_indices[i];
            m_hessian_col_indices[i] = col_indices[i];
        }
        // TODO try to use modern memory management.
        delete [] row_indices;
        delete [] col_indices;
        delete [] hessian;
    }
}

bool IpoptADOLC_OptimizationProblem::get_bounds_info(
        Index num_variables,   Number* x_lower, Number* x_upper,
        Index num_constraints, Number* g_lower, Number* g_upper) {
    assert((unsigned)num_variables   == m_num_variables);
    assert((unsigned)num_constraints == m_num_constraints);

    // TODO pass onto subclass.
    // TODO efficient copying.

    // TODO make sure bounds have been set.
    for (Index ivar = 0; ivar < num_variables; ++ivar) {
        x_lower[ivar] = m_variable_lower_bounds[ivar];
        x_upper[ivar] = m_variable_upper_bounds[ivar];
    }
    // TODO do not assume that there are no inequality constraints.
    if (m_constraint_lower_bounds.size() != (unsigned)num_constraints ||
        m_constraint_upper_bounds.size() != (unsigned)num_constraints) {
        for (Index icon = 0; icon < num_constraints; ++icon) {
            g_lower[icon] = 0;
            g_upper[icon] = 0;
        }
    } else {
        for (Index icon = 0; icon < num_constraints; ++icon) {
            const auto& lower = m_constraint_lower_bounds[icon];
            const auto& upper = m_constraint_upper_bounds[icon];
            // TODO turn the following into an exception message:
            assert(lower <= upper);
            g_lower[icon] = lower;
            g_upper[icon] = upper;
        }
    }
    return true;
}

// z: multipliers for bound constraints on x.
// warmstart will require giving initial values for the multipliers.
bool IpoptADOLC_OptimizationProblem::get_starting_point(
        Index num_variables, bool init_x, Number* x,
        bool init_z, Number* /*z_L*/, Number* /*z_U*/,
        Index num_constraints, bool init_lambda,
        Number* /*lambda*/) {
    // Must this method provide initial values for x, z, lambda?
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);
    assert((unsigned)num_constraints == m_num_constraints);
    // TODO change this interface so that the user specifies the initial
    // guess at the time they request the optimization?
    for (Index ivar = 0; ivar < num_variables; ++ivar) {
        x[ivar] = m_initial_guess[ivar];
    }
    return true;
}

bool IpoptADOLC_OptimizationProblem::eval_f(
        Index num_variables, const Number* x, bool /*new_x*/,
        Number& obj_value) {
    assert((unsigned)num_variables == m_num_variables);
    std::vector<adouble> x_adouble(num_variables);
    // TODO efficiently store this result so it can be used in grad_f, etc.
    for (Index i = 0; i < num_variables; ++i) x_adouble[i] <<= x[i];
    adouble f = 0;
    // TODO objective() could be templatized... so that we could use finite
    // difference if necessary.
    objective(x_adouble, f);
    obj_value = f.value();
    return true;
}
bool IpoptADOLC_OptimizationProblem::eval_grad_f(
        Index num_variables, const Number* x, bool /*new_x*/,
        Number* grad_f) {
    assert((unsigned)num_variables == m_num_variables);
    short int tag = 0;

    // =====================================================================
    // START ACTIVE
    // ---------------------------------------------------------------------
    trace_on(tag);
    std::vector<adouble> x_adouble(num_variables);
    adouble f_adouble = 0;
    double f = 0;
    for (Index i = 0; i < num_variables; ++i) x_adouble[i] <<= x[i];
    objective(x_adouble, f_adouble);
    f_adouble >>= f;
    trace_off();
    // ---------------------------------------------------------------------
    // END ACTIVE
    // =====================================================================
    int success = gradient(tag, num_variables, x, grad_f);
    assert(success);

    return true;
}
bool IpoptADOLC_OptimizationProblem::eval_g(
        Index num_variables, const Number* x, bool /*new_x*/,
        Index num_constraints, Number* g) {
    assert((unsigned)num_variables   == m_num_variables);
    assert((unsigned)num_constraints == m_num_constraints);
    std::vector<adouble> x_adouble(num_variables);
    // TODO efficiently store this result so it can be used in grad_f, etc.
    for (Index i = 0; i < num_variables; ++i) x_adouble[i] <<= x[i];
    std::vector<adouble> gradient(num_constraints);
    constraints(x_adouble, gradient);
    for (Index i = 0; i < num_constraints; ++i) gradient[i] >>= g[i];
    return true;
}
// TODO can Ipopt do finite differencing for us?
bool IpoptADOLC_OptimizationProblem::eval_jac_g(
        Index num_variables, const Number* x, bool /*new_x*/,
        Index num_constraints, Index num_nonzeros_jacobian,
        Index* iRow, Index *jCol, Number* values) {
    if (values == nullptr) {
        // TODO document: provide sparsity pattern.
        assert((unsigned)num_nonzeros_jacobian == m_jacobian_num_nonzeros);
        for (Index inz = 0; inz < num_nonzeros_jacobian; ++inz) {
            iRow[inz] = m_jacobian_row_indices[inz];
            jCol[inz] = m_jacobian_col_indices[inz];
        }
        return true;
    }

    short int tag = 0;
    // =====================================================================
    // START ACTIVE
    // ---------------------------------------------------------------------
    trace_on(tag);
    std::vector<adouble> x_adouble(num_variables);
    for (Index i = 0; i < num_variables; ++i) x_adouble[i] <<= x[i];
    std::vector<adouble> g_adouble(num_constraints);
    constraints(x_adouble, g_adouble);
    std::vector<double> g(num_constraints);
    for (Index i = 0; i < num_constraints; ++i) g_adouble[i] >>= g[i];
    trace_off();
    // ---------------------------------------------------------------------
    // END ACTIVE
    // =====================================================================

    int repeated_call = 0;
    int num_nonzeros = -1; /*TODO*/
    unsigned int* row_indices = NULL; // Allocated by ADOL-C.
    unsigned int* col_indices = NULL; // Allocated by ADOL-C.
    double* jacobian = NULL;          // Allocated by ADOL-C.
    int options[4];
    options[0] = 0; /*TODO*/
    options[1] = 0; /*TODO*/
    options[2] = 0; /*TODO*/
    options[3] = 0; /*TODO*/
    int success = sparse_jac(tag, num_constraints, num_variables,
                             repeated_call, x,
                             &num_nonzeros, &row_indices, &col_indices,
                             &jacobian, options);
    assert(success);
    for (int inz = 0; inz < num_nonzeros; ++inz) {
        values[inz] = jacobian[inz];
    }

    delete [] row_indices;
    delete [] col_indices;
    delete [] jacobian;

    return true;
}
bool IpoptADOLC_OptimizationProblem::eval_h(
        Index num_variables, const Number* x, bool /*new_x*/,
        Number obj_factor, Index num_constraints, const Number* lambda,
        bool /*new_lambda*/, Index num_nonzeros_hessian,
        Index* iRow, Index *jCol, Number* values) {
    assert((unsigned)num_nonzeros_hessian == m_hessian_num_nonzeros);
    if (values == nullptr) {
        // TODO use ADOLC to determine sparsity pattern; hess_pat
        // TODO
        for (Index inz = 0; inz < num_nonzeros_hessian; ++inz) {
            iRow[inz] = m_hessian_row_indices[inz];
            jCol[inz] = m_hessian_col_indices[inz];
        }
        return true;
    }

    // TODO this hessian must include the constraint portion!!!
    // TODO if not new_x, then do NOT re-eval objective()!!!

    // TODO remove from here and utilize new_x.
    // TODO or can I reuse the tape?
    short int tag = 0;
    // -----------------------------------------------------------------
    // START ACTIVE
    trace_on(tag);
    std::vector<adouble> x_adouble(num_variables);
    std::vector<double> lambda_vector(num_constraints);
    adouble lagrangian_adouble;
    double lagr;
    for (Index ivar = 0; ivar < num_variables; ++ivar) {
        // TODO add this operator for std::vector.
        x_adouble[ivar] <<= x[ivar];
    }
    for (Index icon = 0; icon < num_constraints; ++icon) {
        lambda_vector[icon] = lambda[icon];
    }
    lagrangian(obj_factor, x_adouble, lambda_vector, lagrangian_adouble);
    lagrangian_adouble >>= lagr;
    trace_off();
    // END ACTIVE
    // -----------------------------------------------------------------
    // TODO efficiently use the "repeat" argument.
    int repeated_call = 0;
    int options[2];
    options[0] = 0; /* test the computational graph control flow? TODO*/
    options[1] = 0; /* way of recovery TODO */
    // TODO make general:
    unsigned int* row_indices = NULL;
    unsigned int* col_indices = NULL;
    // TODO hope that the row indices are the same between IpOopt and
    // ADOL-C.
    double* vals = NULL;
    int num_nonzeros;
    // TODO compute sparse hessian for each element of the constraint
    // vector....TODO trace with respect to both x and lambda..
    // http://list.coin-or.org/pipermail/adol-c/2013-April/000900.html
    // TODO "since lambda changes, the Lagrangian function has to be
    // repated every time ...cannot set repeat = 1"
    // The following link suggests more efficient methods:
    // http://list.coin-or.org/pipermail/adol-c/2013-April/000903.html
    // Quote:
    // We made the experience that it really depends on the application
    // whether
    //
    // * tracing the Lagrangian once with x and lambda as inputs
    //    and evaluating only a part of the Hessian reusing the trace
    //       in all iterations
    //
    // or
    //
    // *  retracing the Lagrangian with x as adoubles and lambda as doubles
    // in each iteration and computing then the whole Hessian
    //
    // performs better in terms of runtime. You could give both approaches
    // a try and see what works better for you. Both approaches have their
    // pros and cons with respect to efficiency.
    //

    //std::vector<double> x_and_lambda(num_variables + num_constraints);
    //for (unsigned ivar = 0; ivar < num_variables; ++ivar) {
    //    x_and_lambda[ivar] = x[ivar];
    //}
    //for (unsigned icon = 0; icon < num_constraints; ++icon) {
    //    x_and_lambda[icon + num_variables] = lambda[icon];
    //}
    int success = sparse_hess(tag, num_variables, repeated_call,
                              x, &num_nonzeros, &row_indices, &col_indices,
                              &vals, options);
    assert(success);
    for (int i = 0; i < num_nonzeros; ++i) {
        values[i] = vals[i];
    }
    // TODO try to use modern memory management.
    delete [] row_indices;
    delete [] col_indices;
    // TODO avoid reallocating vals each time!!!
    delete [] vals;

    return true;
}
void IpoptADOLC_OptimizationProblem::finalize_solution(
        Ipopt::SolverReturn /*TODO status*/,
        Index num_variables,
        const Number* x, const Number* z_L, const Number* z_U,
        Index /*num_constraints*/,
        const Number* /*g*/, const Number* /*lambda*/,
        Number obj_value, const Ipopt::IpoptData* /*ip_data*/,
        Ipopt::IpoptCalculatedQuantities* /*ip_cq*/) {
    m_solution.resize(num_variables);
    printf("\nSolution of the primal variables, x\n");
    for (Index i = 0; i < num_variables; ++i) {
        printf("x[%d]: %e\n", i, x[i]);
        m_solution[i] = x[i];
    }
    printf("\nSolution of the bound multipliers, z_L and z_U\n");
    for (Index i = 0; i < num_variables; ++i) {
        printf("z_L[%d] = %e\n", i, z_L[i]);
    }
    for (Index i = 0; i < num_variables; ++i) {
        printf("z_U[%d] = %e\n", i, z_U[i]);
    }
    printf("\nObjective value\n");
    printf("f(x*) = %e\n", obj_value);
}

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
