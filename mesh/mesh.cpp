
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

template <typename T>
class OptimalControlProblem {
public:
    virtual ~OptimalControlProblem() = default;
    // TODO this is definitely not the interface I want.
    // TODO difficult... virtual void initial_guess()
    // TODO really want to declare each state variable individually, and give
    // each one a name.
    virtual int num_states() const = 0;
    virtual int num_controls() const = 0;
    virtual void bounds(double& initial_time, double& final_time,
                        std::vector<double>& states_lower,
                        std::vector<double>& states_upper,
                        std::vector<double>& initial_states_lower,
                        std::vector<double>& initial_states_upper,
                        std::vector<double>& final_states_upper,
                        std::vector<double>& final_states_lower,
                        std::vector<double>& controls_lower,
                        std::vector<double>& controls_upper,
                        std::vector<double>& initial_controls_lower,
                        std::vector<double>& initial_controls_upper,
                        std::vector<double>& final_controls_lower,
                        std::vector<double>& final_controls_upper) const = 0;

    // TODO use Eigen, not std::vector.
    virtual void dynamics(const std::vector<T>& states,
                          const std::vector<T>& controls,
                          std::vector<T>& derivative) const = 0;
    // TODO alternate form that takes a matrix; state at every time.
    //virtual void continuous(const MatrixXd& x, MatrixXd& xdot) const = 0;
    //virtual void endpoint_cost(const T& final_time,
    //                           const std::vector<T>& final_states) const = 0;
    // TODO change time to T.
    virtual void integral_cost(const double& time,
                               const std::vector<T>& states,
                               const std::vector<T>& controls,
                               T& integrand) const = 0;
};

class DirectCollocationSolver : public IpoptADOLC_OptimizationProblem {
    typedef OptimalControlProblem<adouble> Problem;
    int m_num_mesh_points = 20;
    int m_num_states = -1;
    int m_num_controls = -1;
    int m_num_continuous_variables = -1;
    double m_initial_time = -1;
    double m_final_time = -1;
public:
    // TODO why shared_ptr???
    void set_problem(std::shared_ptr<Problem> problem) {
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
    int state_index(int i_mesh_point, int i_state) const {
        return i_mesh_point * m_num_continuous_variables + i_state;
    }
    int control_index(int i_mesh_point, int i_control) const {
        return i_mesh_point * m_num_continuous_variables
               + i_control + m_num_states;
    }
    int constraint_index(int i_mesh, int i_state) const {
        int num_bound_constraints = 2 * m_num_continuous_variables;
        return num_bound_constraints + (i_mesh - 1) * m_num_states + i_state;
    }
    enum BoundsCategory {
        InitialStates   = 0,
        FinalStates     = 1,
        InitialControls = 2,
        FinalControls   = 3,
    };
    int constraint_bound_index(BoundsCategory category, int index) const {
        int num_states = m_problem->num_states();
        if (category <= 1) {
            assert(index < num_states);
            return category * num_states + index;
        }
        int num_controls = m_problem->num_controls();
        assert(index < num_controls);
        return 2 * num_states + (category - 2) * num_controls + index;
    }
    void objective(const std::vector<adouble>& x,
                   adouble& obj_value) const override {
        int num_states = m_problem->num_states();
        int num_controls = m_problem->num_controls();
        double step_size = (m_final_time - m_initial_time) /
                           (m_num_mesh_points - 1);

        // Create states and controls vectors.
        // TODO remove when using Eigen.
        std::vector<adouble> states(num_states);
        for (int i_state = 0; i_state < num_states; ++i_state) {
            states[i_state] = x[state_index(0, i_state)];
        }
        std::vector<adouble> controls(num_controls);
        for (int i_control = 0; i_control < num_controls; ++i_control) {
            controls[i_control] = x[control_index(0, i_control)];
        }
        // Evaluate integral cost at the initial time.
        adouble integrand_value = 0;
        // TODO avoid duplication here. Use lambda function?
        m_problem->integral_cost(m_initial_time, states, controls,
                                 integrand_value);
        obj_value = integrand_value;

        for (int i_mesh = 1; i_mesh < m_num_mesh_points; ++i_mesh) {
            for (int i_state = 0; i_state < num_states; ++i_state) {
                states[i_state] = x[state_index(i_mesh, i_state)];
            }
            std::vector<adouble> controls(num_controls);
            for (int i_control = 0; i_control < num_controls; ++i_control) {
                controls[i_control] = x[control_index(i_mesh, i_control)];
            }
            integrand_value = 0;
            m_problem->integral_cost(step_size * i_mesh + m_initial_time,
                                     states, controls, integrand_value);
            obj_value += step_size * integrand_value;
            // TODO use more intelligent quadrature.
        }
    }
    void constraints(const std::vector<adouble>& x,
                     std::vector<adouble>& constraints) const override {
        // TODO parallelize.
        int num_states = m_problem->num_states();
        int num_controls = m_problem->num_controls();
        double step_size = (m_final_time - m_initial_time) /
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
                                       std::vector<adouble>(num_states));
        //std::vector<std::vector<adouble>>
        //        states_trajectory(m_num_mesh_points, {num_states});
        for (int i_mesh_point = 0; i_mesh_point < m_num_mesh_points;
                ++i_mesh_point) {
            // Get the states and controls for this mesh point.
            // TODO prefer having a view, not copying.
            std::vector<adouble> states(num_states);
            //const auto& states = states_trajectory[i_mesh_point];
            for (int i_state = 0; i_state < num_states; ++i_state) {
                states[i_state] = x[state_index(i_mesh_point, i_state)];
            }
            std::vector<adouble> controls(num_controls);
            for (int i_control = 0; i_control < num_controls; ++i_control) {
                controls[i_control] = x[control_index(i_mesh_point, i_control)];
            }
            auto& derivatives = derivatives_trajectory[i_mesh_point];
            m_problem->dynamics(states, controls, derivatives);
        }

        // Bounds on initial and final states and controls.
        // ------------------------------------------------
        for (int i_state = 0; i_state < num_states; ++i_state) {
            constraints[constraint_bound_index(InitialStates, i_state)] =
                    x[state_index(0, i_state)];
        }
        // TODO separate loops might help avoid cache misses, based on the
        // orer of the constraint indices.
        for (int i_state = 0; i_state < num_states; ++i_state) {
            constraints[constraint_bound_index(FinalStates, i_state)] =
                    x[state_index(m_num_mesh_points - 1, i_state)];
        }
        for (int i_control = 0; i_control < num_controls; ++i_control) {
            constraints[constraint_bound_index(InitialControls, i_control)] =
                    x[control_index(0, i_control)];
        }
        for (int i_control = 0; i_control < num_controls; ++i_control) {
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
            assert(derivatives_i.size() == (unsigned)num_states); // TODO temporary.
            for (int i_state = 0; i_state < num_states; ++i_state) {
                const auto& state_i =  x[state_index(i_mesh, i_state)];
                const auto& state_im1 = x[state_index(i_mesh - 1, i_state)];
                constraints[constraint_index(i_mesh, i_state)] = 
                    state_i - (state_im1 + step_size * derivatives_i[i_state]);
            }
            // TODO this would be so much easier with a matrix library.
        }
    }
    void finalize_solution(Ipopt::SolverReturn /*TODO status*/,
                           Index /*num_variables*/,
                           const Number* x,
                           const Number* /*z_L*/, const Number* /*z_U*/,
                           Index /*num_constraints*/,
                           const Number* /*g*/, const Number* /*lambda*/,
                           Number obj_value, const Ipopt::IpoptData* /*ip_data*/,
                           Ipopt::IpoptCalculatedQuantities* /*ip_cq*/) override {
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
private:
    // TODO uh oh how to have generic interface?? the solver should also be
    // templatized??
    std::shared_ptr<Problem> m_problem;
};

class SlidingMass : public OptimalControlProblem<adouble> {
    // TODO difficult... virtual void initial_guess()
    // TODO really want to declare each state variable individually, and give
    // each one a name.
    int num_states() const override { return 2; }
    int num_controls() const override { return 1; }
    void bounds(double& initial_time, double& final_time,
                std::vector<double>& states_lower,
                std::vector<double>& states_upper,
                std::vector<double>& initial_states_lower,
                std::vector<double>& initial_states_upper,
                std::vector<double>& final_states_lower,
                std::vector<double>& final_states_upper,
                std::vector<double>& controls_lower,
                std::vector<double>& controls_upper,
                std::vector<double>& initial_controls_lower,
                std::vector<double>& initial_controls_upper,
                std::vector<double>& final_controls_lower,
                std::vector<double>& final_controls_upper) const override {
        // TODO turn into bounds on time.
        initial_time = 0.0;
        final_time = 2.0;
        states_lower           = {0, -10};
        states_upper           = {2,  10};
        initial_states_lower   = {0, 0};
        initial_states_upper   = initial_states_lower;
        final_states_lower     = {1, 0};
        final_states_upper     = final_states_lower;
        controls_lower         = {-50};
        controls_upper         = {50};
        initial_controls_lower = controls_lower;
        initial_controls_upper = controls_upper;
        final_controls_lower   = controls_lower;
        final_controls_upper   = controls_upper;
    }
    const double mass = 10.0;
    void dynamics(const std::vector<adouble>& states,
                  const std::vector<adouble>& controls,
                  std::vector<adouble>& derivatives) const override {
        derivatives[0] = states[1];
        derivatives[1] = controls[0] / mass;
    }
    // TODO alternate form that takes a matrix; state at every time.
    //virtual void continuous(const MatrixXd& x, MatrixXd& xdot) const = 0;
    //void endpoint_cost(const T& final_time,
    //                   const std::vector<T>& final_states) const override {

    //}
    void integral_cost(const double& /*time*/,
                       const std::vector<adouble>& /*states*/,
                       const std::vector<adouble>& controls,
                       adouble& integrand) const override {
        integrand = controls[0] * controls[0]; 
    }
};

//double g = 9.81;
//
//class MyProb : public Problem {
//    void ode(const VectorXd& x, const VectorXd& u, VectorXd& xdot) const
//            override {
//        xdot[0] = x[1];
//        xdot[1] = u[0];
//        //xdot[1] = -g * sin(x[0]);
//    }
//    int num_states() const override { return 2; }
//    int num_controls() const override { return 1; }
//};

int main() {
    {
        // Ipopt::SmartPtr<ToyProblem> mynlp = new ToyProblem();
        // mynlp->set_variable_bounds({-5, -5}, {5, 5});
        // mynlp->set_constraint_bounds({-0.1}, {0.1});
        // mynlp->set_initial_guess({0, 0});
        Ipopt::SmartPtr<DirectCollocationSolver> mynlp =
                new DirectCollocationSolver();
        std::shared_ptr<OptimalControlProblem<adouble>> problem(
                new SlidingMass());
        mynlp->set_problem(problem);
        // TODO return 0;
        Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
        app->Options()->SetNumericValue("tol", 1e-9);
        app->Options()->SetStringValue("mu_strategy", "adaptive");
        app->Options()->SetStringValue("output_file", "ipopt.out");
        // TODO temporary:
        app->Options()->SetStringValue("derivative_test", "second-order");
        Ipopt::ApplicationReturnStatus status;
        status = app->Initialize();
        if (status != Ipopt::Solve_Succeeded) {
            printf("\n\n*** Error during initialization!\n");
            return (int) status;
        }
        status = app->OptimizeTNLP(mynlp);
        if (status == Ipopt::Solve_Succeeded) {
            printf("\n\n*** The problem solved!\n");
        } else {
            printf("\n\n*** The problem FAILED!\n");
        }
        //ToyProblem toy;
        //adouble f;
        //toy.objective({1.5, -2.0}, f);
        //std::cout << "DEBUG " << f << std::endl;
        return (int) status;
    }

    //int num_points = 100;
    //std::unique_ptr<Problem> problem(new MyProb());
    //const int num_states = problem->num_states();
    //const int num_controls = problem->num_controls();
    ////MatrixXd xdot(num_states, num_points);
    ////MatrixXd x(num_states, num_points);
    ////MatrixXd u(num_controls, num_points);

    //// TODO forward integration.
    //const double step_size = 0.001;
    //const int num_steps = 10000;
    //const double final_time = step_size * num_steps;

    //VectorXd xdot(num_states);
    //VectorXd u(num_controls);
    //u[0] = 1.0;
    //VectorXd initial_x(num_states);
    //initial_x[0] = 0;
    //initial_x[1] = 0;
    //VectorXd current_x = initial_x;
    //for (int itime = 0; itime < num_steps; ++itime) {
    //    problem->ode(current_x, u/*.col(itime)*/, xdot);
    //    current_x = current_x + step_size * xdot;
    //    std::cout << current_x << std::endl << std::endl;
    //}
    //std::cout << "Final time: " << final_time << std::endl;

    /*
    for (int ipt = 0; ipt < num_points; ++ipt) {
        VectorXd this_xdot = xdot.col(0); // TODO unnecessary copy?
        problem->ode(x.col(ipt), u.col(ipt), this_xdot);
        std::cout << this_xdot << std::endl;
        //problem->ode(x[ipt], xdot[ipt]);
    }
    */
}
