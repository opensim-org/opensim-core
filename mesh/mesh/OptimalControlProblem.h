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
            Eigen::Ref<Eigen::VectorXd> final_states_lower,
            Eigen::Ref<Eigen::VectorXd> final_states_upper,
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
    // TODO endpoint or terminal cost?
    //virtual void endpoint_cost(const T& final_time,
    //        const VectorX<T>& final_states) const = 0;
    virtual void endpoint_cost(const double& final_time,
                               const VectorX<T>& final_states,
                               T& cost) const;
    // TODO change time to T.
    virtual void integral_cost(const double& time,
            const VectorX<T>& states,
            const VectorX<T>& controls,
            T& integrand) const;
};

template<typename T>
void OptimalControlProblem<T>::endpoint_cost(const double&,
        const VectorX<T>&, T&) const {}

template<typename T>
void OptimalControlProblem<T>::integral_cost(const double&,
        const VectorX<T>&, const VectorX<T>&, T&) const {}

struct Bounds {
    Bounds() = default;
    Bounds(double value) : lower(value), upper(value) {}
    // TODO check here that lower <= upper.
    Bounds(double lower_bound, double upper_bound)
    {
        assert(lower_bound <= upper_bound);
        lower = lower_bound;
        upper = upper_bound;
    }
    bool is_set() const
    {
        return !std::isnan(lower) && !std::isnan(upper);
    }
    // TODO create nicer name for NaN;
    double lower = std::numeric_limits<double>::quiet_NaN();
    double upper = std::numeric_limits<double>::quiet_NaN();
};
struct InitialBounds : public Bounds {
    using Bounds::Bounds;
};
struct FinalBounds : public Bounds {
    using Bounds::Bounds;
};

template<typename T>
class OptimalControlProblemNamed : public OptimalControlProblem<T> {
public:
private:
    struct ContinuousVariableInfo {
        std::string name;
        Bounds bounds;
        InitialBounds initial_bounds;
        FinalBounds final_bounds;
    };
public:
    void set_time(const double& initial_time, const double& final_time)
    {
        m_initial_time = initial_time;
        m_final_time = final_time;
    }
    void add_state(const std::string& name, const Bounds& bounds,
            const InitialBounds& initial_bounds = InitialBounds(),
            const FinalBounds& final_bounds = FinalBounds())
    {
        m_state_infos.push_back({name, bounds, initial_bounds, final_bounds});
    }
    void add_control(const std::string& name, const Bounds& bounds,
            const InitialBounds& initial_bounds = InitialBounds(),
            const FinalBounds& final_bounds = FinalBounds())
    {
        m_control_infos.push_back({name, bounds, initial_bounds, final_bounds});
    }
    int num_states() const override final { return m_state_infos.size(); }
    int num_controls() const override final { return m_control_infos.size(); }
    void bounds(double& initial_time, double& final_time,
            Eigen::Ref<Eigen::VectorXd> states_lower,
            Eigen::Ref<Eigen::VectorXd> states_upper,
            Eigen::Ref<Eigen::VectorXd> initial_states_lower,
            Eigen::Ref<Eigen::VectorXd> initial_states_upper,
            Eigen::Ref<Eigen::VectorXd> final_states_lower,
            Eigen::Ref<Eigen::VectorXd> final_states_upper,
            Eigen::Ref<Eigen::VectorXd> controls_lower,
            Eigen::Ref<Eigen::VectorXd> controls_upper,
            Eigen::Ref<Eigen::VectorXd> initial_controls_lower,
            Eigen::Ref<Eigen::VectorXd> initial_controls_upper,
            Eigen::Ref<Eigen::VectorXd> final_controls_lower,
            Eigen::Ref<Eigen::VectorXd> final_controls_upper)
                const override final
    {
        initial_time = m_initial_time;
        final_time = m_final_time;
        for (unsigned is = 0; is < m_state_infos.size(); ++is) {
            const auto& info = m_state_infos[is];
            states_lower[is]         = info.bounds.lower;
            states_upper[is]         = info.bounds.upper;
            // TODO do not create initial/final bounds constraints if they
            // are not necessary.
            if (info.initial_bounds.is_set()) {
                initial_states_lower[is] = info.initial_bounds.lower;
                initial_states_upper[is] = info.initial_bounds.upper;
            } else {
                initial_states_lower[is] = info.bounds.lower;
                initial_states_upper[is] = info.bounds.upper;
            }
            if (info.final_bounds.is_set()) {
                final_states_lower[is]   = info.final_bounds.lower;
                final_states_upper[is]   = info.final_bounds.upper;
            } else {
                final_states_lower[is]   = info.bounds.lower;
                final_states_upper[is]   = info.bounds.upper;
            }
        }
        for (unsigned ic = 0; ic < m_control_infos.size(); ++ic) {
            const auto& info = m_control_infos[ic];
            // TODO if (!info.bounds.is_set()), give error.
            controls_lower[ic]         = info.bounds.lower;
            controls_upper[ic]         = info.bounds.upper;
            if (info.initial_bounds.is_set()) {
                initial_controls_lower[ic] = info.initial_bounds.lower;
                initial_controls_upper[ic] = info.initial_bounds.upper;
            } else {
                initial_controls_lower[ic] = info.bounds.lower;
                initial_controls_upper[ic] = info.bounds.upper;
            }
            if (info.final_bounds.is_set()) {
                final_controls_lower[ic]   = info.final_bounds.lower;
                final_controls_upper[ic]   = info.final_bounds.upper;
            } else {
                final_controls_lower[ic]   = info.bounds.lower;
                final_controls_upper[ic]   = info.bounds.upper;
            }
        }
    }
private:
    double m_initial_time;
    double m_final_time;
    std::vector<ContinuousVariableInfo> m_state_infos;
    std::vector<ContinuousVariableInfo> m_control_infos;
};





} // namespace mesh

#endif // MESH_OPTIMALCONTROLPROBLEM_H

