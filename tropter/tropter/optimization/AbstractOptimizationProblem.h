#ifndef TROPTER_ABSTRACTOPTIMIZATIONPROBLEM_H
#define TROPTER_ABSTRACTOPTIMIZATIONPROBLEM_H

#include <tropter/common.h>
#include <memory>

namespace tropter {

class OptimizationProblemProxy;

class AbstractOptimizationProblem {
public:

    AbstractOptimizationProblem() = default;
    AbstractOptimizationProblem(
            unsigned num_variables, unsigned num_constraints)
            :
            m_num_variables(num_variables),
            m_num_constraints(num_constraints) { }
    unsigned get_num_variables() const { return m_num_variables; }
    unsigned get_num_constraints() const { return m_num_constraints; }
    const Eigen::VectorXd&
    get_variable_lower_bounds() const { return m_variable_lower_bounds; }
    const Eigen::VectorXd&
    get_variable_upper_bounds() const { return m_variable_upper_bounds; }
    const Eigen::VectorXd&
    get_constraint_lower_bounds() const { return m_constraint_lower_bounds; }
    const Eigen::VectorXd&
    get_constraint_upper_bounds() const { return m_constraint_upper_bounds; }
    virtual std::shared_ptr<OptimizationProblemProxy> make_proxy() const = 0;

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
    // TODO allow specifying these as std::vector<std::pair<double>>;
    // this is a more logical way to specify bounds for users.
    void set_variable_bounds(const Eigen::VectorXd& lower,
            const Eigen::VectorXd& upper)
    {
        // TODO make sure num_variables has been set.
        // TODO can only call this if m_num_variables etc are already set.
        assert(lower.size() == m_num_variables);
        assert(upper.size() == m_num_variables);
        // TODO where should this ordering check go?
        // TODO make sure that this check works (test it).
        assert((lower.array() <= upper.array()).all());
        m_variable_lower_bounds = lower;
        m_variable_upper_bounds = upper;
    }
    void set_constraint_bounds(const Eigen::VectorXd& lower,
            const Eigen::VectorXd& upper)
    {
        assert(lower.size() == m_num_constraints);
        assert(upper.size() == m_num_constraints);
        // TODO assert(lower <= upper);
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

} // namespace tropter

#endif // TROPTER_ABSTRACTOPTIMIZATIONPROBLEM_H
