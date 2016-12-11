#ifndef MESH_OPTIMIZATIONSOLVER_H
#define MESH_OPTIMIZATIONSOLVER_H

#include "common.h"
#include <Eigen/Dense>
// TODO should be able to remove dependenc on adolc in this file.
#include <adolc/adolc.h>
#include <IpTNLP.hpp>

namespace mesh {

template<typename T>
class OptimizationProblem;

class OptimizationProblemProxy;

// TODO templatized?
class OptimizationSolver {
public:
    // TODO do not force adouble in the future.
    OptimizationSolver(const OptimizationProblem<adouble>& problem);
    // TODO must be an lvalue??
    // TODO might want to change this interface.
    double optimize(Eigen::VectorXd& variables) const;
protected:
    virtual double optimize_impl(Eigen::VectorXd& variables) const = 0;
    std::shared_ptr<const OptimizationProblemProxy> m_problem;
    // TODO rename m_problem to m_proxy? m_probproxy?
//    const OptimizationProblemProxy& m_problem;
};

// TODO perhaps NewtonSolver {}; ?
// TODO for now, assume that it requires using adolc.
class IpoptSolver : public OptimizationSolver {
public:
    // TODO this means the IpoptSolver *would* get access to the Problem,
    // and we don't want that.
    IpoptSolver(const OptimizationProblem<adouble>& problem)
            : OptimizationSolver(problem) {}
    // TODO explain what happens if initial guess is omitted.
    // TODO cannot use temporary.
protected:
    double optimize_impl(Eigen::VectorXd& variables) const override;
private:
    // TODO come up with a better name; look at design patterns book?
    class TNLP;
};

class IpoptSolver::TNLP : public Ipopt::TNLP {
public:
    using Index = Ipopt::Index;
    using Number = Ipopt::Number;
    TNLP(std::shared_ptr<const OptimizationProblemProxy> problem);
    void initialize(const Eigen::VectorXd& guess);
    const Eigen::VectorXd& get_solution() const
    {
        return m_solution;
    }
private:
    // TODO move to OptimizationProblem if more than one solver would need this.
    // TODO should use fancy arguments to avoid temporaries and to exploit
    // expression templates.
//    void lagrangian(double obj_factor, const VectorXa& x,
//            const Eigen::VectorXd& lambda,
//            adouble& result) const;
    // TODO should move to OptimizationProblem<adouble>
//    double trace_objective(short int tag, Index num_variables, const Number* x);
//    void trace_constraints(short int tag, Index num_variables, const Number* x,
//            Index num_constraints, Number* g);
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
//    const OptimizationProblemProxy& m_problem;
    // TODO reconsider the type of this variable:
    std::shared_ptr<const OptimizationProblemProxy> m_problem;

    unsigned m_num_variables = std::numeric_limits<unsigned>::max();
    unsigned m_num_constraints = std::numeric_limits<unsigned>::max();

    // TODO Don't need to store a copy here...?
    Eigen::VectorXd m_initial_guess;
    Eigen::VectorXd m_solution;

    unsigned m_hessian_num_nonzeros = std::numeric_limits<unsigned>::max();
    std::vector<unsigned int> m_hessian_row_indices;
    std::vector<unsigned int> m_hessian_col_indices;
    unsigned m_jacobian_num_nonzeros = std::numeric_limits<unsigned>::max();
    std::vector<unsigned int> m_jacobian_row_indices;
    std::vector<unsigned int> m_jacobian_col_indices;

    //double m_cached_obj_value = std::nan(nullptr);
    // TODO what about for lagrangian??
};

} // namespace mesh

#endif // MESH_OPTIMIZATIONSOLVER_H
