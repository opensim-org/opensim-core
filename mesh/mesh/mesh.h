#ifndef MESH_MESH_H
#define MESH_MESH_H

#include <iostream>
#include <Eigen/Dense>
#include <IpTNLP.hpp>
#include <adolc/adolc.h>
// TODO should not have using declarations in a header file.

// http://www.coin-or.org/Ipopt/documentation/node23.html

// TODO create my own "NonnegativeIndex" or Count type.

// TODO use faster linear solvers from Sherlock cluster.

// TODO want to abstract optimization problem away from IPOPT.


// TODO provide a C interface?

namespace mesh {

using VectorXa = Eigen::Matrix<adouble, Eigen::Dynamic, 1>;
using MatrixXa = Eigen::Matrix<adouble, Eigen::Dynamic, Eigen::Dynamic>;

template <typename T>
using VectorX = Eigen::Matrix<T, Eigen::Dynamic, 1>;

// TODO consider namespace opt for generic NLP stuff.

template <typename T>
class OptimizationProblem {
public:
    virtual ~OptimizationProblem() = default;
    OptimizationProblem() = default;
    OptimizationProblem(unsigned num_variables, unsigned num_constraints)
            : m_num_variables(num_variables),
              m_num_constraints(num_constraints) {}

    virtual void objective(const VectorX<T>& x, T& obj_value) const;
    virtual void constraints(const VectorX<T>& x,
            Eigen::Ref<VectorX<T>> constr) const;
    // TODO can override to provide custom derivatives.
    //virtual void gradient(const std::vector<T>& x, std::vector<T>& grad) const;
    //virtual void jacobian(const std::vector<T>& x, TODO) const;
    //virtual void hessian() const;

    unsigned get_num_variables() const { return m_num_variables; }

    unsigned get_num_constraints() const { return m_num_constraints; }

    const Eigen::VectorXd& get_variable_lower_bounds() const {
        return m_variable_lower_bounds;
    }
    const Eigen::VectorXd& get_variable_upper_bounds() const {
        return m_variable_upper_bounds;
    }
    const Eigen::VectorXd& get_constraint_lower_bounds() const {
        return m_constraint_lower_bounds;
    }
    const Eigen::VectorXd& get_constraint_upper_bounds() const {
        return m_constraint_upper_bounds;
    }

protected:
    void set_num_variables(unsigned num_variables) {
        // TODO if set, invalidate variable bounds.
        m_num_variables = num_variables;
    }
    void set_num_constraints(unsigned num_constraints) {
        m_num_constraints = num_constraints;
    }
    // TODO eigen wants these to be more generic to avoid temporaries.
    void set_variable_bounds(const Eigen::VectorXd& lower,
                             const Eigen::VectorXd& upper) {
        // TODO make sure num_variables has been set.
        // TODO can only call this if m_num_variables etc are already set.
        assert(lower.size() == m_num_variables);
        assert(upper.size() == m_num_variables);
        m_variable_lower_bounds = lower;
        m_variable_upper_bounds = upper;
    }
    void set_constraint_bounds(const Eigen::VectorXd& lower,
                               const Eigen::VectorXd& upper) {
        assert(lower.size() == m_num_constraints);
        assert(upper.size() == m_num_constraints);
        m_constraint_lower_bounds = lower;
        m_constraint_upper_bounds = upper;
    }
private:
    // TODO use safer types that will give exceptions for improper values.
    unsigned m_num_variables;
    unsigned m_num_constraints;
    Eigen::VectorXd m_variable_lower_bounds;
    Eigen::VectorXd m_variable_upper_bounds;
    Eigen::VectorXd m_constraint_lower_bounds;
    Eigen::VectorXd m_constraint_upper_bounds;
};

template <typename T>
void OptimizationProblem<T>::objective(const VectorX<T>&, T&) const {
    // TODO proper error messages.
    throw std::runtime_error("Not implemented.");
}
template <typename T>
void OptimizationProblem<T>::constraints(const VectorX<T>&,
        Eigen::Ref<VectorX<T>>) const {
    // TODO throw std::runtime_error("Not implemented.");
}

// TODO Specialize OptimizationProblem<adouble> with implementations
// for gradient, jacobian, hessian.

// TODO templatized?
class OptimizationSolver {
public:
    // TODO do not force adouble in the future.
    OptimizationSolver(const OptimizationProblem<adouble>& problem)
            : m_problem(problem) {}
    virtual double optimize(Eigen::Ref<Eigen::VectorXd> variables) const = 0;
protected:
    const OptimizationProblem<adouble>& m_problem;
};

// TODO perhaps NewtonSolver {}; ?
// TODO for now, assume that it requires using adolc.
class IpoptSolver : public OptimizationSolver {
public:
    IpoptSolver(const OptimizationProblem<adouble>& problem)
            : OptimizationSolver(problem) {}
    // TODO explain what happens if initial guess is omitted.
    double optimize(Eigen::Ref<Eigen::VectorXd> variables) const override;
private:
    // TODO come up with a better name; look at design patterns book?
    class TNLP;
};

class IpoptSolver::TNLP : public Ipopt::TNLP {
public:
    using Index = Ipopt::Index;
    using Number = Ipopt::Number;
    TNLP(const OptimizationProblem<adouble>& problem) : m_problem(problem) {
        m_num_variables = m_problem.get_num_variables();
        m_num_constraints = m_problem.get_num_constraints();
    }
    void initialize(const Eigen::VectorXd& guess);
    const Eigen::VectorXd& get_solution() const {
        return m_solution;
    }
private:
    // TODO move to OptimizationProblem if more than one solver would need this.
    // TODO should use fancy arguments to avoid temporaries and to exploit
    // expression templates.
    void lagrangian(double obj_factor, const VectorXa& x,
            const Eigen::VectorXd& lambda,
            adouble& result) const;
    // TODO should move to OptimizationProblem<adouble>
    double trace_objective(short int tag, Index num_variables, const Number* x);
    void trace_constraints(short int tag, Index num_variables, const Number* x,
            Index num_constraints, Number* g);
    bool get_nlp_info(Index& num_variables, Index& num_constraints,
            Index& num_nonzeros_jacobian, Index& num_nonzeros_hessian,
            IndexStyleEnum& index_style) override {
        num_variables = m_problem.get_num_variables();
        num_constraints = m_problem.get_num_constraints();
        num_nonzeros_jacobian = m_jacobian_num_nonzeros;
        num_nonzeros_hessian = m_hessian_num_nonzeros;
        index_style = TNLP::C_STYLE;
        return true;
    }
    bool get_bounds_info(Index num_variables,
            Number* x_lower, Number* x_upper,
            Index num_constraints,
            Number* g_lower, Number* g_upper) override;

    // z: multipliers for bound constraints on x.
    // warmstart will require giving initial values for the multipliers.
    bool get_starting_point(Index num_variables, bool init_x, Number* x,
            bool init_z, Number* z_L, Number* z_U,
            Index num_constraints, bool init_lambda,
            Number* lambda) override;

    bool eval_f(Index num_variables, const Number* x, bool new_x,
            Number& obj_value) override;

    bool eval_grad_f(Index num_variables, const Number* x, bool new_x,
            Number* grad_f) override;

    bool eval_g(Index num_variables, const Number* x, bool new_x,
            Index num_constraints, Number* g) override;

    // TODO can Ipopt do finite differencing for us?
    bool eval_jac_g(Index num_variables, const Number* x, bool new_x,
            Index num_constraints, Index num_nonzeros_jacobian,
            Index* iRow, Index *jCol, Number* values) override;

    bool eval_h(Index num_variables, const Number* x, bool new_x,
            Number obj_factor, Index num_constraints, const Number* lambda,
            bool new_lambda, Index num_nonzeros_hessian,
            Index* iRow, Index *jCol, Number* values) override;

    void finalize_solution(Ipopt::SolverReturn status,
            Index num_variables,
            const Number* x,
            const Number* z_L, const Number* z_U,
            Index num_constraints,
            const Number* g, const Number* lambda,
            Number obj_value, const Ipopt::IpoptData* ip_data,
            Ipopt::IpoptCalculatedQuantities* ip_cq) override;

    // Members.
    const OptimizationProblem<adouble>& m_problem;

    unsigned m_num_variables = std::numeric_limits<unsigned>::max();
    unsigned m_num_constraints = std::numeric_limits<unsigned>::max();

    // TODO Don't need to store a copy here...?
    Eigen::VectorXd m_initial_guess;
    Eigen::VectorXd m_solution;

    unsigned m_hessian_num_nonzeros = -1;
    std::vector<unsigned int> m_hessian_row_indices;
    std::vector<unsigned int> m_hessian_col_indices;
    unsigned m_jacobian_num_nonzeros = -1;
    std::vector<unsigned int> m_jacobian_row_indices;
    std::vector<unsigned int> m_jacobian_col_indices;

    //double m_cached_obj_value = std::nan(nullptr);
    const short int m_objective_tag = 1;
    const short int m_constraint_tag = 2;
    // TODO what about for lagrangian??
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
            Eigen::Ref<Eigen::VectorXd> states_lower,
            Eigen::Ref<Eigen::VectorXd> states_upper,
            Eigen::Ref<Eigen::VectorXd> initial_states_lower,
            Eigen::Ref<Eigen::VectorXd> initial_states_upper,
            Eigen::Ref<Eigen::VectorXd> final_states_upper,
            Eigen::Ref<Eigen::VectorXd> final_states_lower,
            Eigen::Ref<Eigen::VectorXd> controls_lower,
            Eigen::Ref<Eigen::VectorXd> controls_upper,
            Eigen::Ref<Eigen::VectorXd> initial_controls_lower,
            Eigen::Ref<Eigen::VectorXd> initial_controls_upper,
            Eigen::Ref<Eigen::VectorXd> final_controls_lower,
            Eigen::Ref<Eigen::VectorXd> final_controls_upper) const = 0;

    // TODO use Eigen, not std::vector.
    virtual void dynamics(const VectorX<T>& states,
            const VectorX<T>& controls,
            Eigen::Ref<VectorX<T>> derivative) const = 0;
    // TODO alternate form that takes a matrix; state at every time.
    //virtual void continuous(const MatrixX<T>& x, MatrixX<T>& xdot) const = 0;
    //virtual void endpoint_cost(const T& final_time,
    //                           const VectorX<T>& final_states) const = 0;
    // TODO change time to T.
    virtual void integral_cost(const double& time,
            const VectorX<T>& states,
            const VectorX<T>& controls,
            T& integrand) const = 0;
};

// TODO template <typename T>
class EulerTranscription : public OptimizationProblem<adouble> {
    // TODO should this *BE* an OptimizationProblem, or should it just
    // contain one?
public:
    typedef OptimalControlProblem<adouble> Problem;

    // TODO why would we want a shared_ptr? A copy would use the same Problem.
    EulerTranscription(std::shared_ptr<Problem> problem) {
        set_problem(problem);
    }
    void set_problem(std::shared_ptr<Problem> problem);

    void objective(const VectorXa& x,
            adouble& obj_value) const override;
    void constraints(const VectorXa& x,
            Eigen::Ref<VectorXa> constr) const override;

    void interpret_iterate(const Eigen::VectorXd& x,
            Eigen::Ref<Eigen::MatrixXd> states_trajectory,
            Eigen::Ref<Eigen::MatrixXd> controls_trajectory) const;
private:
    int state_index(int i_mesh_point, int i_state) const {
        return i_mesh_point * m_num_continuous_variables + i_state;
    }
    int control_index(int i_mesh_point, int i_control) const {
        return i_mesh_point * m_num_continuous_variables
                + i_control + m_num_states;
    }
    int constraint_index(int i_mesh, int i_state) const {
        const int num_bound_constraints = 2 * m_num_continuous_variables;
        return num_bound_constraints + (i_mesh - 1) * m_num_states + i_state;
    }
    enum BoundsCategory {
        InitialStates   = 0,
        FinalStates     = 1,
        InitialControls = 2,
        FinalControls   = 3,
    };
    int constraint_bound_index(BoundsCategory category, int index) const {
        if (category <= 1) {
            assert(index < m_num_states);
            return category * m_num_states + index;
        }
        assert(index < m_num_controls);
        return 2 * m_num_states + (category - 2) * m_num_controls + index;
    }

    std::shared_ptr<Problem> m_problem;
    int m_num_mesh_points = 20;
    int m_num_states = -1;
    int m_num_controls = -1;
    int m_num_continuous_variables = -1;
    // TODO these should go eventually:
    double m_initial_time = -1;
    double m_final_time = -1;
};

} // namespace mesh

// TODO templatize Problem.

// TODO interface 0: (inheritance)
// derive from Problem, implement virtual functions
// interface 1: (composition)
// composed of Variables, Controls, Goals, etc.
/*
std::unordered_map<std::string, Goal> m_goals;
std::unordered_map<std::string
 * */

#endif // MESH_MESH_H
