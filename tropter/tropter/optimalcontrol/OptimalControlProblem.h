#ifndef TROPTER_OPTIMALCONTROLPROBLEM_H
#define TROPTER_OPTIMALCONTROLPROBLEM_H

#include "OptimalControlIterate.h"
#include <tropter/common.h>
#include <Eigen/Dense>

namespace tropter {

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

template <typename T>
class OptimalControlProblem {
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
    OptimalControlProblem() = default;
    OptimalControlProblem(const std::string& name) : m_name(name) {}
    virtual ~OptimalControlProblem() = default;
    /// @name Get information about the problem
    /// @{
    int num_states() const
    {   return (int)m_state_infos.size(); }
    int num_controls() const
    {   return (int)m_control_infos.size(); }
    int num_path_constraints() const
    {   return (int)m_path_constraint_infos.size(); }
    std::vector<std::string> get_state_names() const {
        std::vector<std::string> names;
        for (const auto& info : m_state_infos) {
            names.push_back(info.name);
        }
        return names;
    }
    std::vector<std::string> get_control_names() const {
        std::vector<std::string> names;
        for (const auto& info : m_control_infos) {
            names.push_back(info.name);
        }
        return names;
    }
    std::vector<std::string> get_path_constraint_names() const {
        std::vector<std::string> names;
        for (const auto& info : m_path_constraint_infos) {
            names.push_back(info.name);
        }
        return names;
    }
    void print_description() const;
    /// @}

    /// @name Assemble the problem
    /// @{
    void set_time(const InitialBounds& initial_time,
            const FinalBounds& final_time) {
        m_initial_time_bounds = initial_time;
        m_final_time_bounds = final_time;
    }
    // TODO make sure initial and final bounds are within the bounds.
    /// This returns an index that can be used to access this specific state
    /// variable within `dynamics()` , `path_constraints()`, etc.
    /// TODO check if a state with the provided name already exists.
    int add_state(const std::string& name, const Bounds& bounds,
            const InitialBounds& initial_bounds = InitialBounds(),
            const FinalBounds& final_bounds = FinalBounds()) {
        m_state_infos.push_back({name, bounds, initial_bounds, final_bounds});
        return (int)m_state_infos.size() - 1;
    }
    /// This returns an index that can be used to access this specific control
    /// variable within `dynamics()` , `path_constraints()`, etc.
    /// TODO check if a control with the provided name already exists.
    int add_control(const std::string& name, const Bounds& bounds,
            const InitialBounds& initial_bounds = InitialBounds(),
            const FinalBounds& final_bounds = FinalBounds()) {
        // TODO proper handling of bounds: if initial_bounds is omitted,
        // then its values come from bounds.
        // TODO want to avoid adding unnecessary constraints to the optimization
        // problem if initial/final bounds are omitted. Use the Bounds'
        // objects' "is_set()" function.
        m_control_infos.push_back({name, bounds, initial_bounds, final_bounds});
        return (int)m_control_infos.size() - 1;
    }
    /// This returns an index that can be used to access this specific path
    /// constraint element within `path_constraints()`.
    /// TODO check if a path constraint with the provided name already exists.
    int add_path_constraint(const std::string& name, const Bounds& bounds) {
        m_path_constraint_infos.push_back({name, bounds});
        return (int)m_path_constraint_infos.size() - 1;
    }
    /// @}

    /// @name Implement these functions
    /// These are the virtual functions you implement to define your optimal
    /// control problem.
    /// @{
    /// Perform any precalculations or caching (e.g., interpolating data)
    /// necessary before the optimal control problem is solved. This is
    /// invoked every time there's a new mesh (e.g., in each mesh refinement
    /// iteration). Implementing this function probably only makes sense if
    /// your problem has fixed initial and final times.
    /// To perform any caching in member variables, you have to use *mutable*
    /// member variables or a const_cast:
    /// @code{.cpp}
    /// const_cast<MyOCP*>(this)->data = ...;
    /// @endcode
    /// @param mesh The length of mesh is the number of mesh points. The mesh
    ///             points are normalized and are thus within [0, 1].
    virtual void initialize_on_mesh(const Eigen::VectorXd& mesh) const;

    // TODO must pass in the time.
    // The initial values in `derivative` are arbitrary and cannot be assumed
    // to be 0, etc. You must set entries to 0 explicitly if you want that.
    virtual void dynamics(const VectorX<T>& states,
                          const VectorX<T>& controls,
                          Eigen::Ref<VectorX<T>> derivative) const;
    // The initial values in `constraints` are arbitrary and cannot be assumed
    // to be 0, etc. You must set entries to 0 explicitly if you want that.
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
            throw std::runtime_error("[tropter] guess.time is empty.");
        }
        if (value.size() != guess.time.size()) {
            throw std::runtime_error("[tropter] Expected value to have " +
                    std::to_string(guess.time.size()) + " elements, but it has"
                    " " + std::to_string(value.size()) + ".");
        }
        if (guess.states.rows() == 0) {
            guess.states.resize(m_state_infos.size(), guess.time.size());
        } else if (size_t(guess.states.rows()) != m_state_infos.size() ||
                guess.states.cols() != guess.time.size()) {
            throw std::runtime_error("[tropter] Expected guess.states to have "
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
                    "[tropter] State " + name + " does not exist.");
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
            throw std::runtime_error("[tropter] guess.time is empty.");
        }
        if (value.size() != guess.time.size()) {
            throw std::runtime_error("[tropter] Expected value to have " +
                    std::to_string(guess.time.size()) + " elements, but it has"
                    " " + std::to_string(value.size()) + ".");
        }
        if (guess.controls.rows() == 0) {
            guess.controls.resize(m_control_infos.size(), guess.time.size());
        } else if (size_t(guess.controls.rows()) != m_control_infos.size() ||
                guess.controls.cols() != guess.time.size()) {
            throw std::runtime_error("[tropter] Expected guess.controls to have "
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
                    "[tropter] Control " + name + " does not exist.");
        }

        // Set the guess.
        guess.controls.row(control_index) = value;
    }
    /// @}

    // TODO move to "getter" portion.
    /// @name For use by direct collocation solver
    /// @{
    /// This function provides the bounds on time, states, and controls in a
    /// format that is easy for the direct collocation classes to use.
    void all_bounds(
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
            Eigen::Ref<Eigen::VectorXd> path_constraints_upper) const;
    /// @}
private:
    std::string m_name = "<unnamed>";
    InitialBounds m_initial_time_bounds;
    FinalBounds m_final_time_bounds;
    std::vector<ContinuousVariableInfo> m_state_infos;
    std::vector<ContinuousVariableInfo> m_control_infos;
    std::vector<PathConstraintInfo> m_path_constraint_infos;
};

} // namespace tropter

#endif // TROPTER_OPTIMALCONTROLPROBLEM_H

