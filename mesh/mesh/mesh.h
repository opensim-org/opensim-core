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

// http://www.coin-or.org/Ipopt/documentation/node23.html

// TODO create my own "NonnegativeIndex" or Count type.

// TODO use faster linear solvers from Sherlock cluster.

// TODO want to abstract optimization problem away from IPOPT.


// TODO provide a C interface?

namespace mesh {

// TODO consider namespace opt for generic NLP stuff.

template <typename T>
class OptimizationProblem {
public:
    OptimizationProblem(unsigned num_variables, unsigned num_constraints)
            : m_num_variables(num_variables),
              m_num_constraints(num_constraints) {}

    virtual void objective(const std::vector<T>& x, T& obj_value) const;
    virtual void constraints(const std::vector<T>& x,
            std::vector<T>& constr) const;
    // TODO can override to provide custom derivatives.
    //virtual void gradient(const std::vector<T>& x, std::vector<T>& grad) const;
    //virtual void jacobian(const std::vector<T>& x, TODO) const;
    //virtual void hessian() const;

    unsigned get_num_variables() const { return m_num_variables; }

    unsigned get_num_constraints() const { return m_num_constraints; }

    const std::vector<double>& get_variable_lower_bounds() const {
        return m_variable_lower_bounds;
    }
    const std::vector<double>& get_variable_upper_bounds() const {
        return m_variable_upper_bounds;
    }
    const std::vector<double>& get_constraint_lower_bounds() const {
        return m_constraint_lower_bounds;
    }
    const std::vector<double>& get_constraint_upper_bounds() const {
        return m_constraint_upper_bounds;
    }

protected:
    void set_variable_bounds(const std::vector<double>& lower,
                             const std::vector<double>& upper) {
        // TODO can only call this if m_num_variables etc are already set.
        assert(lower.size() == m_num_variables);
        assert(upper.size() == m_num_variables);
        m_variable_lower_bounds = lower;
        m_variable_upper_bounds = upper;
    }
    void set_constraint_bounds(const std::vector<double>& lower,
                               const std::vector<double>& upper) {
        assert(lower.size() == m_num_constraints);
        assert(upper.size() == m_num_constraints);
        m_constraint_lower_bounds = lower;
        m_constraint_upper_bounds = upper;
    }
private:
    // TODO use safer types that will give exceptions for improper values.
    unsigned m_num_variables;
    unsigned m_num_constraints;
    std::vector<double> m_variable_lower_bounds;
    std::vector<double> m_variable_upper_bounds;
    std::vector<double> m_constraint_lower_bounds;

private:
    // TODO
    std::vector<double> m_constraint_upper_bounds;
};

template <typename T>
void OptimizationProblem<T>::objective(const std::vector<T>&, T&) const {
    // TODO proper error messages.
    throw std::runtime_error("Not implemented.");
}
template <typename T>
void OptimizationProblem<T>::constraints(const std::vector<T>&,
        std::vector<T>&) const {
    // TODO throw std::runtime_error("Not implemented.");
}

// TODO Specialize OptimizationProblem<adouble> with implementations
// for gradient, jacobian, hessian.

template <typename T>
class OptimalControlProblem : public OptimizationProblem<T> {

};

// TODO templatized?
class OptimizationSolver {
public:
    // TODO do not force adouble in the future.
    OptimizationSolver(const OptimizationProblem<adouble>& problem)
            : m_problem(problem) {}
    virtual double optimize(std::vector<double>& variables) const = 0;
protected:
    const OptimizationProblem<adouble>& m_problem;
};

// TODO perhaps NewtonSolver {}; ?
// TODO for now, assume that it requires using adolc.
class IpoptSolver : public OptimizationSolver {
public:
    IpoptSolver(const OptimizationProblem<adouble>& problem)
            : OptimizationSolver(problem) {}
    double optimize(std::vector<double>& variables) const override;
private:
    // TODO come up with a better name; look at design patterns book?
    class TNLP;
};

class IpoptSolver::TNLP : public Ipopt::TNLP {
public:
    TNLP(const OptimizationProblem<adouble>& problem) : m_problem(problem) {
        m_num_variables = m_problem.get_num_variables();
        m_num_constraints = m_problem.get_num_constraints();
    }
    void initialize(const std::vector<double>& guess);
    const std::vector<double>& get_solution() const {
        return m_solution;
    }
private:
    // TODO move to OptimizationProblem if more than one solver would need this.
    void lagrangian(double obj_factor, const std::vector<adouble>& x,
            const std::vector<double>& lambda,
            adouble& result) const;
    // TODO should move to OptimizationProblem<adouble>
    double trace_objective(short int tag, Index num_variables, const Number* x);
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

    unsigned m_num_variables = -1;
    unsigned m_num_constraints = -1;

    // TODO Don't need to store a copy here...?
    std::vector<double> m_initial_guess;
    std::vector<double> m_solution;

    unsigned m_hessian_num_nonzeros = -1;
    std::vector<unsigned int> m_hessian_row_indices;
    std::vector<unsigned int> m_hessian_col_indices;
    unsigned m_jacobian_num_nonzeros = -1;
    std::vector<unsigned int> m_jacobian_row_indices;
    std::vector<unsigned int> m_jacobian_col_indices;

    //double m_cached_obj_value = std::nan(nullptr);
    const short int objective_tag = 1;
    const short int constraint_tag = 1;
    // TODO what about for lagrangian??
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

class IpoptADOLC_OptimizationProblem : public Ipopt::TNLP {
private:
    unsigned m_num_variables = -1;
    unsigned m_num_constraints = -1;
    std::vector<double> m_variable_lower_bounds;
    std::vector<double> m_variable_upper_bounds;
    std::vector<double> m_constraint_lower_bounds;
    std::vector<double> m_constraint_upper_bounds;
    std::vector<double> m_initial_guess;
    unsigned m_hessian_num_nonzeros = -1;
    std::vector<unsigned int> m_hessian_row_indices;
    std::vector<unsigned int> m_hessian_col_indices;
    unsigned m_jacobian_num_nonzeros = -1;
    std::vector<unsigned int> m_jacobian_row_indices;
    std::vector<unsigned int> m_jacobian_col_indices;
    std::vector<double> m_solution;
public:
    IpoptADOLC_OptimizationProblem() = default;
    IpoptADOLC_OptimizationProblem(int num_variables, int num_constraints) :
            m_num_variables(num_variables), m_num_constraints(num_constraints) {}

    void set_num_variables(unsigned num_variables)
    { m_num_variables = num_variables; }
    void set_num_constraints(unsigned num_constraints)
    { m_num_constraints = num_constraints; }

    virtual void objective(const std::vector<adouble>& x,
                           adouble& obj_value) const = 0;
    virtual void constraints(const std::vector<adouble>& x,
                             std::vector<adouble>& constraints) const = 0;
    void lagrangian(double obj_factor, const std::vector<adouble>& x,
                    const std::vector<double>& lambda,
                    adouble& result) const {
        assert(x.size() == m_num_variables);
        assert(lambda.size() == m_num_constraints);

        result = 0;
        //if (obj_factor != 0) {
        //    objective(x, result);
        //    result *= obj_factor;
        //}
        objective(x, result); // TODO should not compute obj if obj_factor = 0.
        // but that optimization makes ADOLC unhappy.
        result *= obj_factor;
        std::vector<adouble> constr(m_num_constraints);
        constraints(x, constr);
        for (unsigned icon = 0; icon < m_num_constraints; ++icon) {
            result += lambda[icon] * constr[icon];
        }
    }
    void set_variable_bounds(const std::vector<double>& lower,
                             const std::vector<double>& upper) {
        assert(lower.size() == m_num_variables);
        assert(upper.size() == m_num_variables);
        m_variable_lower_bounds = lower;
        m_variable_upper_bounds = upper;
        // TODO check their sizes.
    }
    void set_constraint_bounds(const std::vector<double>& lower,
                               const std::vector<double>& upper) {
        assert(lower.size() == m_num_constraints);
        assert(upper.size() == m_num_constraints);
        m_constraint_lower_bounds = lower;
        m_constraint_upper_bounds = upper;
    }

    void set_initial_guess(const std::vector<double>& guess);

    std::vector<double> get_solution() const { return m_solution; }
private:
    bool get_nlp_info(Index& num_variables, Index& num_constraints,
                      Index& num_nonzeros_jacobian, Index& num_nonzeros_hessian,
                      IndexStyleEnum& index_style) override {
        num_variables = m_num_variables;
        num_constraints = m_num_constraints;
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
                           const Number* x, const Number* z_L, const Number* z_U,
                           Index num_constraints,
                           const Number* g, const Number* lambda,
                           Number obj_value, const Ipopt::IpoptData* ip_data,
                           Ipopt::IpoptCalculatedQuantities* ip_cq) override;
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
    void set_problem(std::shared_ptr<Problem> problem);
    int state_index(int i_mesh_point, int i_state) const {
        return i_mesh_point * m_num_continuous_variables + i_state;
    }
    int control_index(int i_mesh_point, int i_control) const{
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
    void objective(const std::vector<adouble>& x,
                   adouble& obj_value) const override;
    void constraints(const std::vector<adouble>& x,
                     std::vector<adouble>& constraints) const override;
    void finalize_solution(Ipopt::SolverReturn /*TODO status*/,
                           Index /*num_variables*/,
                           const Number* x,
                           const Number* /*z_L*/, const Number* /*z_U*/,
                           Index /*num_constraints*/,
                           const Number* /*g*/, const Number* /*lambda*/,
                           Number obj_value, const Ipopt::IpoptData* /*ip_data*/,
                           Ipopt::IpoptCalculatedQuantities* /*ip_cq*/) override;
private:
    // TODO uh oh how to have generic interface?? the solver should also be
    // templatized??
    std::shared_ptr<Problem> m_problem;
};
