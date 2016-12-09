#ifndef MESH_OPTIMIZATIONPROBLEM_H
#define MESH_OPTIMIZATIONPROBLEM_H

#include "common.h"

namespace mesh {

// TODO Specialize OptimizationProblem<adouble> with implementations
// for gradient, jacobian, hessian.

template<typename T>
class OptimizationProblem {
public:
    virtual ~OptimizationProblem() = default;

    OptimizationProblem() = default;

    OptimizationProblem(unsigned num_variables, unsigned num_constraints)
            :m_num_variables(num_variables),
             m_num_constraints(num_constraints) { }

    virtual void objective(const VectorX <T>& x, T& obj_value) const;

    virtual void constraints(const VectorX <T>& x,
            Eigen::Ref <VectorX<T>> constr) const;
    // TODO can override to provide custom derivatives.
    //virtual void gradient(const std::vector<T>& x, std::vector<T>& grad) const;
    //virtual void jacobian(const std::vector<T>& x, TODO) const;
    //virtual void hessian() const;

    unsigned get_num_variables() const { return m_num_variables; }

    unsigned get_num_constraints() const { return m_num_constraints; }

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
    // TODO use safer types that will give exceptions for improper values.
    unsigned m_num_variables;
    unsigned m_num_constraints;
    Eigen::VectorXd m_variable_lower_bounds;
    Eigen::VectorXd m_variable_upper_bounds;
    Eigen::VectorXd m_constraint_lower_bounds;
    Eigen::VectorXd m_constraint_upper_bounds;
};

template<typename T>
void OptimizationProblem<T>::objective(const VectorX <T>&, T&) const
{
    // TODO proper error messages.
    throw std::runtime_error("Not implemented.");
}

template<typename T>
void OptimizationProblem<T>::constraints(const VectorX <T>&,
        Eigen::Ref <VectorX<T>>) const
{
    // TODO throw std::runtime_error("Not implemented.");
}

} // namespace mesh

#endif // MESH_OPTIMIZATIONPROBLEM_H
