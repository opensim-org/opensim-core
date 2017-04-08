#ifndef MESH_OPTIMALCONTROLPROBLEM_H
#define MESH_OPTIMALCONTROLPROBLEM_H

#include "common.h"
#include <Eigen/Dense>

namespace mesh {

struct OptimalControlIterate {
    Eigen::RowVectorXd time;
    Eigen::MatrixXd states;
    Eigen::MatrixXd controls;
    std::vector<std::string> state_names;
    std::vector<std::string> control_names;
    virtual void write(const std::string& filepath) const {
        std::ofstream f(filepath);

        // Column headers.
        f << "time";
        for (int i_state = 0; i_state < states.rows(); ++i_state) {
            f << "," << state_names[i_state];
        }
        for (int i_control = 0; i_control < controls.rows(); ++i_control) {
            f << "," << control_names[i_control];
        }
        f << std::endl;

        // Data.
        for (int i_mesh = 0; i_mesh < time.size(); ++i_mesh) {
            f << time[i_mesh];
            for (int i_state = 0; i_state< states.rows(); ++i_state) {
                f << "," << states(i_state, i_mesh);
            }
            for (int i_control = 0; i_control< controls.rows(); ++i_control) {
                f << "," << controls(i_control, i_mesh);
            }
            f << std::endl;
        }
        f.close();
    }
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
    // TODO assume 0?
    virtual int num_path_constraints() const { return 0; }
    /// If no names were provided, these will be "state0", "state1", etc.
    virtual std::vector<std::string> get_state_names() const;
    /// If no names were provided, these will be "control0", "control1", etc.
    virtual std::vector<std::string> get_control_names() const;
    /// If no names were provided, these will be "pathcon0", "pathcon1", etc.
    virtual std::vector<std::string> get_path_constraint_names() const;

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
    /// iteration). Implementing this function probably only makes sense if
    /// your problem has fixed initial and final times.
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
    virtual void integral_cost(const T& time,
            const VectorX<T>& states,
            const VectorX<T>& controls,
            T& integrand) const;
};

template<typename T>
std::vector<std::string>
OptimalControlProblem<T>::get_state_names() const
{
    size_t N = num_states();
    std::vector<std::string> names(N);
    for (size_t i = 0; i < N; ++i) names[i] = "state" + std::to_string(i);
    return names;
}

template<typename T>
std::vector<std::string>
OptimalControlProblem<T>::get_control_names() const
{
    size_t N = num_controls();
    std::vector<std::string> names(N);
    for (size_t i = 0; i < N; ++i) names[i] = "control" + std::to_string(i);
    return names;

}

template<typename T>
std::vector<std::string>
OptimalControlProblem<T>::get_path_constraint_names() const
{
    size_t N = num_path_constraints();
    std::vector<std::string> names(N);
    for (size_t i = 0; i < N; ++i) names[i] = "pathcon" + std::to_string(i);
    return names;

}

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

    /// @name Assemble the problem
    /// @{
    void set_time(const InitialBounds& initial_time,
            const FinalBounds& final_time)
    {
        m_initial_time_bounds = initial_time;
        m_final_time_bounds = final_time;
    }
    // TODO make sure initial and final bounds are within the bounds.
    /// This returns an index that can be used to access this specific state
    /// variable within `dynamics()` , `path_constraints()`, etc.
    int add_state(const std::string& name, const Bounds& bounds,
            const InitialBounds& initial_bounds = InitialBounds(),
            const FinalBounds& final_bounds = FinalBounds())
    {
        m_state_infos.push_back({name, bounds, initial_bounds, final_bounds});
        return m_state_infos.size() - 1;
    }
    /// This returns an index that can be used to access this specific control
    /// variable within `dynamics()` , `path_constraints()`, etc.
    int add_control(const std::string& name, const Bounds& bounds,
            const InitialBounds& initial_bounds = InitialBounds(),
            const FinalBounds& final_bounds = FinalBounds())
    {
        // TODO proper handling of bounds: if initial_bounds is omitted,
        // then its values come from bounds.
        // TODO want to avoid adding unnecessary constraints to the optimization
        // problem if initial/final bounds are omitted. Use the Bounds'
        // objects' "is_set()" function.
        m_control_infos.push_back({name, bounds, initial_bounds, final_bounds});
        return m_control_infos.size() - 1;
    }
    /// This returns an index that can be used to access this specific path
    /// constraint element within `path_constraints()`.
    int add_path_constraint(const std::string& name, const Bounds& bounds) {
        m_path_constraint_infos.push_back({name, bounds});
        return m_path_constraint_infos.size() - 1;
    }
    /// @}

    /// @name Helpers for setting an initial guess
    /// @{
    /// Set a guess for the trajectory of a single state variable with name
    /// `name` to `value`. This function relieves you of the need to know the
    /// index of a state variable. The `guess` must already have its `time`
    /// vector filled out. If `guess.states` is empty, this function will set
    /// its dimensions appropriately according to the provided `guess.time`;
    /// otherwise, `guess.states` must have the correct dimensions (number of
    /// states x mesh points).
    /// @param[in,out] guess
    ///     The row of the states matrix associated with the provided name
    ///     is set to value.
    /// @param[in] name
    ///     Name of the state variable (provided in add_state()).
    /// @param[in] value
    ///     This must have the same number of columns as `guess.time` and
    ///     `guess.states`.
    void set_state_guess(OptimalControlIterate& guess,
                         const std::string& name,
                         const Eigen::VectorXd& value) {
        // Check for errors.
        if (guess.time.size() == 0) {
            throw std::runtime_error("[mesh] guess.time is empty.");
        }
        if (value.size() != guess.time.size()) {
            throw std::runtime_error("[mesh] Expected value to have " +
                    std::to_string(guess.time.size()) + " elements, but it has"
                    " " + std::to_string(value.size()) + ".");
        }
        if (guess.states.rows() == 0) {
            guess.states.resize(m_state_infos.size(), guess.time.size());
        } else if (size_t(guess.states.rows()) != m_state_infos.size() ||
                   guess.states.cols() != guess.time.size()) {
           throw std::runtime_error("[mesh] Expected guess.states to have "
                   "dimensions " + std::to_string(m_state_infos.size()) + " x "
                   + std::to_string(guess.time.size()) + ", but dimensions are "
                   + std::to_string(guess.states.rows()) + " x "
                   + std::to_string(guess.states.cols()) + ".");
        }

        // Find the state index.
        size_t state_index = 0;
        // TODO store state infos in a map.
        for (const auto& info : m_state_infos) {
            if (info.name == name) break;
            state_index++;
        }
        if (state_index == m_state_infos.size()) {
            throw std::runtime_error(
                    "[mesh] State " + name + " does not exist.");
        }

        // Set the guess.
        guess.states.row(state_index) = value;
    }
    /// Set a guess for the trajectory of a single control variable with name
    /// `name` to `value`. This function relieves you of the need to know the
    /// index of a control variable. The `guess` must already have its `time`
    /// vector filled out. If `guess.control` is empty, this function will set
    /// its dimensions appropriately according to the provided `guess.time`;
    /// otherwise, `guess.control` must have the correct dimensions (number of
    /// controls x mesh points).
    /// @param[in,out] guess
    ///     The row of the controls matrix associated with the provided name
    ///     is set to value.
    /// @param[in] name
    ///     Name of the control variable (provided in add_control()).
    /// @param[in] value
    ///     This must have the same number of columns as `guess.time` and
    ///     `guess.controls`.
    void set_control_guess(OptimalControlIterate& guess,
                         const std::string& name,
                         const Eigen::VectorXd& value) {
        // Check for errors.
        if (guess.time.size() == 0) {
            throw std::runtime_error("[mesh] guess.time is empty.");
        }
        if (value.size() != guess.time.size()) {
            throw std::runtime_error("[mesh] Expected value to have " +
                    std::to_string(guess.time.size()) + " elements, but it has"
                    " " + std::to_string(value.size()) + ".");
        }
        if (guess.controls.rows() == 0) {
            guess.controls.resize(m_control_infos.size(), guess.time.size());
        } else if (size_t(guess.controls.rows()) != m_control_infos.size() ||
                guess.controls.cols() != guess.time.size()) {
            throw std::runtime_error("[mesh] Expected guess.controls to have "
                "dimensions " + std::to_string(m_control_infos.size()) + " x "
                + std::to_string(guess.time.size()) + ", but dimensions are "
                + std::to_string(guess.controls.rows()) + " x "
                + std::to_string(guess.controls.cols()) + ".");
        }

        // Find the control index.
        size_t control_index = 0;
        // TODO store control infos in a map.
        for (const auto& info : m_control_infos) {
            if (info.name == name) break;
            control_index++;
        }
        if (control_index == m_control_infos.size()) {
            throw std::runtime_error(
                    "[mesh] Control " + name + " does not exist.");
        }

        // Set the guess.
        guess.controls.row(control_index) = value;
    }
    /// @}

    /// @name Get information about the problem
    /// @{
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
    std::vector<std::string> get_path_constraint_names() const override {
        std::vector<std::string> names;
        for (const auto& info : m_path_constraint_infos) {
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
    /// @}
private:
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

