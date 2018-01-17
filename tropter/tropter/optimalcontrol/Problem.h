#ifndef TROPTER_OPTIMALCONTROL_PROBLEM_H
#define TROPTER_OPTIMALCONTROL_PROBLEM_H
// ----------------------------------------------------------------------------
// tropter: Problem.h
// ----------------------------------------------------------------------------
// Copyright (c) 2017 tropter authors
//
// Licensed under the Apache License, Version 2.0 (the "License"); you may
// not use this file except in compliance with the License. You may obtain a
// copy of the License at http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ----------------------------------------------------------------------------

#include "Iterate.h"
#include <tropter/common.h>
#include <Eigen/Dense>

namespace tropter {

/// @ingroup optimalcontrol
struct Bounds {
    Bounds() = default;
    Bounds(double value) : lower(value), upper(value) {}
    // TODO check here that lower <= upper.
    Bounds(double lower_bound, double upper_bound);
    bool is_set() const
    {
        return !std::isnan(lower) && !std::isnan(upper);
    }
    // TODO create nicer name for NaN;
    double lower = std::numeric_limits<double>::quiet_NaN();
    double upper = std::numeric_limits<double>::quiet_NaN();
};
/// @ingroup optimalcontrol
struct InitialBounds : public Bounds {
    using Bounds::Bounds;
};
/// @ingroup optimalcontrol
struct FinalBounds : public Bounds {
    using Bounds::Bounds;
};

/// This struct holds inputs to
/// OptimalControlProblem::calc_differntial_algebraic_equations().
/// @ingroup optimalcontrol
template<typename T>
struct DAEInput {
    /// This index may be helpful for using a cache computed in
    /// OptimalControlProblem::initialize_on_mesh().
    const int mesh_index;
    /// The current time for the provided states and controls.
    const T& time;
    /// The vector of states at time `time`. This stores a reference to an
    /// Eigen matrix, thereby avoiding copying the memory.
    /// @note If you pass this reference into multiple functions that take an
    /// Eigen Vector as input, this could create multiple copies unnecessarily;
    /// in this case, create one copy and pass that to the other functions:
    /// @code{.cpp}
    /// const tropter::VectorX<T> states = input.states;
    /// calc_something(states);
    /// calc_something_else(states);
    /// @endcode
    const Eigen::Ref<const VectorX<T>>& states;
    /// The vector of controls at time `time`.
    /// @note If you pass this into functions that take an Eigen Vector as
    /// input, see the note above for the `states` variable.
    const Eigen::Ref<const VectorX<T>>& controls;
    /// The vector of time-invariant parameter values.
    /// @note If you pass this into functions that take an Eigen Vector as
    /// input, see the note above for the `states` variable.
    const Eigen::Ref<const VectorX<T>>& parameters;
};
/// This struct holds the outputs of
/// OptimalControlProblem::calc_differntial_algebraic_equations().
/// @ingroup optimalcontrol
template<typename T>
struct DAEOutput {
    /// Store the right-hand-side of the differential equations in this
    /// variable. The length of this vector is num_states.
    /// This is a reference to memory that is managed by the direct
    /// collocation solver. If you want to create a shorter variable name,
    /// you could do something like this:
    /// @code{.cpp}
    /// auto& xdot = output.dynamics;
    /// @endcode
    /// Do *not* copy this variable into a new Eigen Vector; the direct
    /// collocation solver will not be able to obtain your output.
    /// @code{.cpp}
    /// tropter::VectorX<T> xdot = output.dynamics;
    /// @endcode
    Eigen::Ref<VectorX<T>> dynamics;
    /// Store the path constraint errors in this variable. The length of this
    /// vector is num_path_constraints. See the notes above about using this
    /// reference.
    Eigen::Ref<VectorX<T>> path;
};

/// We use the following terms to describe an optimal control problem:
/// - *state*: a single state variable.
/// - *states*:  a vector of all state variables at a given time.
/// - *states trajectory*: a trajectory through time of states (state vectors).
/// - *control*: a single control variable.
/// - *controls*:  a vector of all control variables at a given time.
/// - *controls trajectory*: a trajectory through time of controls
///   (control vectors).
/// - *parameter*: a single parameter variable.
/// - *parameters*: a vector of all parameter variables.
/// @ingroup optimalcontrol
template <typename T>
class Problem {
private:
    struct ContinuousVariableInfo {
        ContinuousVariableInfo(std::string, Bounds, InitialBounds, FinalBounds);
        const std::string name;
        const Bounds bounds;
        const InitialBounds initial_bounds;
        const FinalBounds final_bounds;
    };
    struct PathConstraintInfo {
        const std::string name;
        const Bounds bounds;
    };
    struct ParameterInfo {
        const std::string name;
        const Bounds bounds;
    };
public:

    Problem() = default;
    Problem(const std::string& name) : m_name(name) {}
    virtual ~Problem() = default;
    /// @name Get information about the problem
    /// @{
    int get_num_states() const
    {   return (int)m_state_infos.size(); }
    int get_num_controls() const
    {   return (int)m_control_infos.size(); }
    int get_num_parameters() const
    {   return (int)m_parameter_infos.size(); }
    int get_num_path_constraints() const
    {   return (int)m_path_constraint_infos.size(); }
    /// Get the names of all the states in the order they appear in the
    /// `states` input to calc_differential_algebraic_equations(), etc.
    /// Note: this function is not free to call.
    std::vector<std::string> get_state_names() const {
        std::vector<std::string> names;
        for (const auto& info : m_state_infos) {
            names.push_back(info.name);
        }
        return names;
    }
    /// Get the names of all the controls in the order they appear in
    /// the `controls` input to calc_differential_algebraic_equations(), etc.
    /// Note: this function is not free to call.
    std::vector<std::string> get_control_names() const {
        std::vector<std::string> names;
        for (const auto& info : m_control_infos) {
            names.push_back(info.name);
        }
        return names;
    }
    /// Get the names of all the parameters in the order they appear in
    /// the `parameters` input to calc_differential_algebraic_equations(), etc.
    /// Note: this function is not free to call.
    std::vector<std::string> get_parameter_names() const {
        std::vector<std::string> names;
        for (const auto& info : m_parameter_infos) {
            names.push_back(info.name);
        }
        return names;
    }
    /// Get the names of all the path constraints in the order they appear in
    /// the 'path' output to calc_differential_algebraic_equations(), etc.
    /// Note: this function is not free to call.
    std::vector<std::string> get_path_constraint_names() const {
        std::vector<std::string> names;
        for (const auto& info : m_path_constraint_infos) {
            names.push_back(info.name);
        }
        return names;
    }
    /// Print (to std::cout) the number, names, and bounds of the states,
    /// controls, and path constraints.
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
    // TODO no initial, final bounds for controls.
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
    /// This returns an index that can be used to access this specific parameter
    /// variable within `dynamics()` , `path_constraints()`, etc.
    /// TODO check if a parameter with the provided name already exists.
    int add_parameter(const std::string& name, const Bounds& bounds) {
        m_parameter_infos.push_back({name, bounds});
        return (int)m_parameter_infos.size() - 1;
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

    /// Compute the right-hand side of the differntial algebraic equations
    /// (DAE) for the system you want to optimize. This is the function that
    /// provides the dynamics and path constraints.
    /// The initial values in `derivatives` and `constraints` are arbitrary and
    /// cannot be assumed to be 0, etc. You must set entries to 0 explicitly if
    /// you want that.
    virtual void calc_differential_algebraic_equations(
            const DAEInput<T>& in, DAEOutput<T> out) const;
    // TODO alternate form that takes a matrix; state at every time.
    //virtual void continuous(const MatrixX<T>& x, MatrixX<T>& xdot) const = 0;
    // TODO Maybe this one signature could be used for both the "continuous,
    // entire trajectory" mode or the "one time" mode.
    //    var.states[0];
    //    var.states_traj
    //    var.controls[0];
    //    out.derivatives[0] =
    //    out.constraints[0] = ...
    //}
    // TODO endpoint or terminal cost?
    virtual void calc_endpoint_cost(const T& final_time,
            const VectorX<T>& final_states,
            const VectorX<T>& parameters,
            T& cost) const;
    virtual void calc_integral_cost(const T& time,
            const VectorX<T>& states,
            const VectorX<T>& controls,
            const VectorX<T>& parameters,
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
    void set_state_guess(Iterate& guess,
            const std::string& name,
            const Eigen::VectorXd& value);
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
    void set_control_guess(Iterate& guess,
            const std::string& name,
            const Eigen::VectorXd& value);
    /// Set a guess for the value of a single parameter variable with name
    /// `name` to `value`. This function relieves you of the need to know the
    /// index of a parameter variable.
    /// @param[in,out] guess
    ///     The element of the parameters vector associated with the provided 
    ///     name is set to value.
    /// @param[in] name
    ///     Name of the parameter variable (provided in add_parameter()).
    /// @param[in] value
    ///     This value is time-invariant in the iterate `guess`.
    void set_parameter_guess(Iterate& guess,
            const std::string& name,
            const double& value);
    /// @}

    // TODO move to "getter" portion.
    /// @name For use by direct collocation solver
    /// @{

    /// This function provides the bounds on time, states, and controls in a
    /// format that is easy for the direct collocation classes to use.
    void get_all_bounds(
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
            Eigen::Ref<Eigen::VectorXd> parameters_lower,
            Eigen::Ref<Eigen::VectorXd> parameters_upper,
            Eigen::Ref<Eigen::VectorXd> path_constraints_lower,
            Eigen::Ref<Eigen::VectorXd> path_constraints_upper) const;
    /// @}
private:
    std::string m_name = "<unnamed>";
    InitialBounds m_initial_time_bounds;
    FinalBounds m_final_time_bounds;
    std::vector<ContinuousVariableInfo> m_state_infos;
    std::vector<ContinuousVariableInfo> m_control_infos;
    std::vector<ParameterInfo> m_parameter_infos;
    std::vector<PathConstraintInfo> m_path_constraint_infos;
};

} // namespace tropter

#endif // TROPTER_OPTIMALCONTROL_PROBLEM_H

