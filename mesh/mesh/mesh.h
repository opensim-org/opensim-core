#ifndef MESH_MESH_H
#define MESH_MESH_H

// TODO temp
#include "legacy.h"

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include <adolc/adolc.h>
#include <adolc/sparse/sparsedrivers.h>
// TODO should not have using declarations in a header file.
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
    const short int m_objective_tag = 1;
    const short int m_constraint_tag = 2;
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

#endif // MESH_MESH_H
