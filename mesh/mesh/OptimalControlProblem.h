#ifndef MESH_OPTIMALCONTROLPROBLEM_H
#define MESH_OPTIMALCONTROLPROBLEM_H

#include "common.h"
#include <Eigen/Dense>

namespace mesh {

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

} // namespace mesh

#endif // MESH_OPTIMALCONTROLPROBLEM_H
