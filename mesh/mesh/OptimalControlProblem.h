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
    // TODO assume 0?
    virtual int num_path_constraints() const { return 0; }
    virtual std::vector<std::string> get_state_names() const { return {}; }
    virtual std::vector<std::string> get_control_names() const { return {}; }

    virtual void print_description() const;

    virtual void bounds(
            double& initial_time_lower, double& initial_time_upper,
            double& final_time_lower, double& final_time_upper,
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
            Eigen::Ref<Eigen::VectorXd> final_controls_upper,
            Eigen::Ref<Eigen::VectorXd> path_constraints_lower,
            Eigen::Ref<Eigen::VectorXd> path_constraints_upper) const = 0;

    /// Perform any precalculations or caching (e.g., interpolating data)
    /// necessary before the optimal control problem is solved. This is
    /// invoked every time there's a new mesh (e.g., in each mesh refinement
    /// iteration).
    /// To perform any caching in member variables, you have to use a
    /// const_cast:
    /// @code{.cpp}
    /// const_cast<MyOCP*>(this)->data = ...;
    /// @endcode
    /// @param mesh The length of mesh is the number of mesh points. The mesh
    ///             points are normalized and are thus within [0, 1].
    virtual void initialize_on_mesh(const Eigen::VectorXd& mesh) const;

    // TODO use Eigen, not std::vector.
    // TODO must pass in the time.
    virtual void dynamics(const VectorX<T>& states,
                          const VectorX<T>& controls,
                          Eigen::Ref<VectorX<T>> derivative) const;
    virtual void path_constraints(unsigned index,
                                  const T& time,
                                  const VectorX<T>& states,
                                  const VectorX<T>& controls,
                                  Eigen::Ref<VectorX<T>> constraints) const;
    // TODO alternate form that takes a matrix; state at every time.
    //virtual void continuous(const MatrixX<T>& x, MatrixX<T>& xdot) const = 0;
    // TODO endpoint or terminal cost?
    //virtual void endpoint_cost(const T& final_time,
    //        const VectorX<T>& final_states) const = 0;
    virtual void endpoint_cost(const T& final_time,
                               const VectorX<T>& final_states,
                               T& cost) const;
    // TODO change time to T.
    virtual void integral_cost(const T& time,
            const VectorX<T>& states,
            const VectorX<T>& controls,
            T& integrand) const;
};

template<typename T>
void OptimalControlProblem<T>::print_description() const {
    std::cout << "Description of this optimal control problem:" << std::endl;
    std::cout << "Number of states: "
              << this->num_states() << std::endl;
    std::cout << "Number of controls: "
              << this->num_controls() << std::endl;
    std::cout << "Number of path constraints: "
              << this->num_path_constraints() << std::endl;
}

template<typename T>
void OptimalControlProblem<T>::initialize_on_mesh(const Eigen::VectorXd&) const
{}

template<typename T>
void OptimalControlProblem<T>::dynamics(const VectorX<T>&,
                                        const VectorX<T>&,
                                        Eigen::Ref<VectorX<T>>) const
{}

template<typename T>
void OptimalControlProblem<T>::path_constraints(unsigned,
                                                const T&,
                                                const VectorX <T>&,
                                                const VectorX <T>&,
                                                Eigen::Ref<VectorX<T>>) const {}

template<typename T>
void OptimalControlProblem<T>::endpoint_cost(const T&,
                                             const VectorX<T>&, T&) const {}

template<typename T>
void OptimalControlProblem<T>::integral_cost(const T&,
                                             const VectorX<T>&,
                                             const VectorX<T>&, T&) const {}

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
private:
    struct ContinuousVariableInfo {
        std::string name;
        Bounds bounds;
        InitialBounds initial_bounds;
        FinalBounds final_bounds;
    };
    struct PathConstraintInfo {
        std::string name;
        Bounds bounds;
    };
public:
    OptimalControlProblemNamed() = default;
    OptimalControlProblemNamed(const std::string& name) : m_name(name) {}

    void set_time(const InitialBounds& initial_time,
            const FinalBounds& final_time)
    {
        m_initial_time_bounds = initial_time;
        m_final_time_bounds = final_time;
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
        // TODO proper handling of bounds: if initial_bounds is omitted,
        // then its values come from bounds.
        // TODO want to avoid adding unnecessary constraints to the optimization
        // problem if initial/final bounds are omitted. Use the Bounds'
        // objects' "is_set()" function.
        m_control_infos.push_back({name, bounds, initial_bounds, final_bounds});
    }
    void add_path_constraint(const std::string& name, const Bounds& bounds) {
        m_path_constraint_infos.push_back({name, bounds});
    }
    int num_states() const override final { return m_state_infos.size(); }
    int num_controls() const override final { return m_control_infos.size(); }
    int num_path_constraints() const override final
    {
        return m_path_constraint_infos.size();
    }
    std::vector<std::string> get_state_names() const override {
        std::vector<std::string> names;
        for (const auto& info : m_state_infos) {
            names.push_back(info.name);
        }
        return names;
    }
    std::vector<std::string> get_control_names() const override {
        std::vector<std::string> names;
        for (const auto& info : m_control_infos) {
            names.push_back(info.name);
        }
        return names;
    }
    void print_description() const override final
    {
        using std::cout;
        using std::endl;
        auto print_continuous_var_info = [](
                const std::string& var_type,
                const std::vector<ContinuousVariableInfo>& infos) {
            cout << var_type << ":";
            if (!infos.empty())
                cout << " (total number: " << infos.size() << ")" << endl;
            else
                cout << " none" << endl;
            for (const auto& info : infos) {
                cout << "  " << info.name << ". bounds: ["
                        << info.bounds.lower << ", "
                        << info.bounds.upper << "] ";
                if (info.initial_bounds.is_set()) {
                    cout << " initial bounds: ["
                            << info.initial_bounds.lower << ", "
                            << info.initial_bounds.upper << "].";
                }
                if (info.initial_bounds.is_set()) {
                    cout << " final bounds: ["
                            << info.final_bounds.lower << ", "
                            << info.final_bounds.upper << "]";
                }
                cout << endl;
            }
        };

        cout << "Description of optimal control problem "
             << m_name << ":" << endl;

        print_continuous_var_info("States", m_state_infos);
        print_continuous_var_info("Controls", m_control_infos);

        cout << "Path constraints: (total number: "
             << this->num_path_constraints() << ")" << endl;
        for (const auto& info : m_path_constraint_infos) {
            cout << "  " << info.name << ". bounds: ["
                 << info.bounds.lower << ", "
                 << info.bounds.upper << "]"
                 << endl;
        }
    }
    void bounds(double& initial_time_lower, double& initial_time_upper,
                double& final_time_lower, double& final_time_upper,
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
                Eigen::Ref<Eigen::VectorXd> final_controls_upper,
                Eigen::Ref<Eigen::VectorXd> path_constraints_lower,
                Eigen::Ref<Eigen::VectorXd> path_constraints_upper)
    const override final
    {
        initial_time_lower = m_initial_time_bounds.lower;
        initial_time_upper = m_initial_time_bounds.upper;
        final_time_lower = m_final_time_bounds.lower;
        final_time_upper = m_final_time_bounds.upper;
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
        for (unsigned ip = 0; ip < m_path_constraint_infos.size(); ++ip) {
            const auto& info = m_path_constraint_infos[ip];
            path_constraints_lower[ip] = info.bounds.lower;
            path_constraints_upper[ip] = info.bounds.upper;
        }
    }
private:
    std::string m_name = "<unnamed>";
    InitialBounds m_initial_time_bounds;
    FinalBounds m_final_time_bounds;
    std::vector<ContinuousVariableInfo> m_state_infos;
    std::vector<ContinuousVariableInfo> m_control_infos;
    std::vector<PathConstraintInfo> m_path_constraint_infos;
};

} // namespace mesh

#endif // MESH_OPTIMALCONTROLPROBLEM_H

