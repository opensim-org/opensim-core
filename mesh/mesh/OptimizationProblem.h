#ifndef MESH_OPTIMIZATIONPROBLEM_H
#define MESH_OPTIMIZATIONPROBLEM_H

#include "common.h"

namespace mesh {

// TODO Specialize OptimizationProblem<adouble> with implementations
// for gradient, jacobian, hessian.

template<typename T>
class OptimizationProblem {
public:
    class Proxy;

    virtual ~OptimizationProblem() = default;

    OptimizationProblem() = default;

    OptimizationProblem(unsigned num_variables, unsigned num_constraints)
            :m_num_variables(num_variables),
             m_num_constraints(num_constraints) { }

    // TODO can override to provide custom derivatives.
    //virtual void gradient(const std::vector<T>& x, std::vector<T>& grad) const;
    //virtual void jacobian(const std::vector<T>& x, TODO) const;
    //virtual void hessian() const;

    unsigned get_num_variables() const { return m_num_variables; }

    unsigned get_num_constraints() const { return m_num_constraints; }

    void objective(const Eigen::VectorXd& variables, double& obj_value) const;

    void constraints(const Eigen::VectorXd& variables,
            Eigen::Ref<Eigen::VectorXd> constr) const;

//    void objective(const VectorX<T>& x, T& obj_value) const;
//
//    void constraints(const VectorX<T>& x,
//            Eigen::Ref<VectorX<T>> constr) const;

    // TODO for both hessian and jacobian.
    // TODO what about multiple points used to determine sparsity?
    // TODO this would move to a proxy class.
    void determine_sparsity(const Eigen::VectorXd& variables,
            std::vector<unsigned int>& jacobian_row_indices,
            std::vector<unsigned int>& jacobian_col_indices,
            std::vector<unsigned int>& hessian_row_indices,
            std::vector<unsigned int>& hessian_col_indices) const;

    const Eigen::VectorXd& get_variable_lower_bounds() const
    {
        return m_variable_lower_bounds;
    }

    const Eigen::VectorXd& get_variable_upper_bounds() const
    {
        return m_variable_upper_bounds;
    }

    const Eigen::VectorXd& get_constraint_lower_bounds() const
    {
        return m_constraint_lower_bounds;
    }

    const Eigen::VectorXd& get_constraint_upper_bounds() const
    {
        return m_constraint_upper_bounds;
    }
protected:
    // TODO it might be weird for users to use/implement a method called "_impl"
    virtual void objective_impl(const VectorX<T>& x, T& obj_value) const;

    virtual void constraints_impl(const VectorX<T>& x,
            Eigen::Ref<VectorX<T>> constr) const;

    void set_num_variables(unsigned num_variables)
    {
        // TODO if set, invalidate variable bounds.
        m_num_variables = num_variables;
    }

    void set_num_constraints(unsigned num_constraints)
    {
        m_num_constraints = num_constraints;
    }

    // TODO eigen wants these to be more generic to avoid temporaries.
    void set_variable_bounds(const Eigen::VectorXd& lower,
            const Eigen::VectorXd& upper)
    {
        // TODO make sure num_variables has been set.
        // TODO can only call this if m_num_variables etc are already set.
        assert(lower.size()==m_num_variables);
        assert(upper.size()==m_num_variables);
        m_variable_lower_bounds = lower;
        m_variable_upper_bounds = upper;
    }

    void set_constraint_bounds(const Eigen::VectorXd& lower,
            const Eigen::VectorXd& upper)
    {
        assert(lower.size()==m_num_constraints);
        assert(upper.size()==m_num_constraints);
        m_constraint_lower_bounds = lower;
        m_constraint_upper_bounds = upper;
    }

private:

    // TODO this feels too Ipopt-specific..obj_factor?
    void lagrangian(double obj_factor, const VectorX<T>& x,
            const Eigen::VectorXd& lambda, T& result) const;


    // TODO use safer types that will give exceptions for improper values.
    unsigned m_num_variables;
    unsigned m_num_constraints;
    Eigen::VectorXd m_variable_lower_bounds;
    Eigen::VectorXd m_variable_upper_bounds;
    Eigen::VectorXd m_constraint_lower_bounds;
    Eigen::VectorXd m_constraint_upper_bounds;
};

template<typename T>
void OptimizationProblem<T>::objective_impl(const VectorX<T>&, T&) const
{
    // TODO proper error messages.
    throw std::runtime_error("Not implemented.");
}

template<typename T>
void OptimizationProblem<T>::constraints_impl(const VectorX<T>&,
        Eigen::Ref<VectorX<T>>) const
{
// TODO throw std::runtime_error("Not implemented.");
}

template<typename T>
void OptimizationProblem<T>::lagrangian(double obj_factor, const VectorX<T>& x,
        const Eigen::VectorXd& lambda, T& result) const
{
    assert(x.size() == m_num_variables);
    assert(lambda.size() == m_num_constraints);

    result = 0;
    // TODO should not compute obj if obj_factor = 0 but this messes up with
    // ADOL-C.
    //if (obj_factor != 0) {
    //    objective(x, result);
    //    result *= obj_factor;
    //}
    objective_impl(x, result);
    result *= obj_factor;

    // TODO if (!m_num_constraints) return;
    VectorXa constr(m_num_constraints);
    constraints_impl(x, constr);
    // TODO it's highly unlikely that this works:
    // TODO result += lambda.dot(constr);
    for (unsigned icon = 0; icon<m_num_constraints; ++icon) {
        result += lambda[icon]*constr[icon];
    }
}

} // namespace mesh

#endif // MESH_OPTIMIZATIONPROBLEM_H
