#ifndef MESH_OPTIMIZATIONSOLVER_H
#define MESH_OPTIMIZATIONSOLVER_H

#include "common.h"
#include <Eigen/Dense>
#include <adolc/adolc.h>
#include <IpTNLP.hpp>

namespace mesh {

template<typename T>
class OptimizationProblem;

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
    TNLP(const OptimizationProblem<adouble>& problem);
    void initialize(const Eigen::VectorXd& guess);
    const Eigen::VectorXd& get_solution() const
    {
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
            IndexStyleEnum& index_style) override;
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

} // namespace mesh

#endif // MESH_OPTIMIZATIONSOLVER_H
