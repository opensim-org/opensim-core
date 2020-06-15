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
/// Problem::calc_differntial_algebraic_equations() and
/// Problem::calc_integral_cost().
/// @ingroup optimalcontrol
template<typename T>
struct Input {
    /// This index may be helpful for using a cache computed in
    /// OptimalControlProblem::initialize_on_mesh().
    const int time_index;
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
    /// The vector of adjuncts at time `time`.
    /// @note If you pass this into functions that take an Eigen Vector as
    /// input, see the note above for the `states` variable.
    const Eigen::Ref<const VectorX<T>>& adjuncts;
    /// The vector of adjuncts at time `time`.
    /// @note If you pass this into functions that take an Eigen Vector as
    /// input, see the note above for the `states` variable.
    /// @note An empty reference indicates that the diffuses are not defined
    /// for the time point associated with this struct.
    const Eigen::Ref<const VectorX<T>>& diffuses;
    /// The vector of time-invariant parameter values.
    /// @note If you pass this into functions that take an Eigen Vector as
    /// input, see the note above for the `states` variable.
    const Eigen::Ref<const VectorX<T>>& parameters;
};
/// This is the input for Problem::calc_cost(). Refer to the documentation for
/// Input above for details. Diffuse variables do not exist at phase endpoints.
template<typename T>
struct CostInput {
    const int initial_time_index;
    const T& initial_time;
    const Eigen::Ref<const VectorX<T>>& initial_states;
    const Eigen::Ref<const VectorX<T>>& initial_controls;
    const Eigen::Ref<const VectorX<T>>& initial_adjuncts;
    const int final_time_index;
    const T& final_time;
    const Eigen::Ref<const VectorX<T>>& final_states;
    const Eigen::Ref<const VectorX<T>>& final_controls;
    const Eigen::Ref<const VectorX<T>>& final_adjuncts;
    const Eigen::Ref<const VectorX<T>>& parameters;
    const T& integral;
};
/// This struct holds the outputs of
/// Problem::calc_differntial_algebraic_equations().
/// @ingroup optimalcontrol
template<typename T>
struct Output {
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
    /// @note An empty reference indicates that the path constraint is not 
    /// defined for the time point represented by this struct.
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
/// - *adjunct*: a single supplemental continuous variable. 
/// - *adjuncts*: a vector of all adjunct variables at a given time.
/// - *adjuncts trajectory*: a trajectory through time of adjuncts
///   (adjunct vectors).
/// - *diffuse*: a single variable only defined within a mesh interval.
/// - *diffuses*: a vector of all diffuses at a given time.
/// - *diffuses trajectory*: a trajectory through time of diffuses
///   (diffuse vectors).
/// - *parameter*: a single parameter variable.
/// - *parameters*: a vector of all parameter variables.
/// @ingroup optimalcontrol
template <typename T>
class Problem {
private:
    struct ContinuousVariableInfo {
        ContinuousVariableInfo(std::string, Bounds, InitialBounds, FinalBounds);
        std::string name;
        Bounds bounds;
        InitialBounds initial_bounds;
        FinalBounds final_bounds;
    };
    struct ParameterInfo {
        std::string name;
        Bounds bounds;
    };
    struct DiffuseInfo {
        std::string name;
        Bounds bounds;
    };
    struct CostInfo {
        std::string name;
        bool requires_integral;
    };
    struct PathConstraintInfo {
        std::string name;
        Bounds bounds;
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
    int get_num_adjuncts() const
    {   return (int)m_adjunct_infos.size(); }
    int get_num_diffuses() const
    {   return (int)m_diffuse_infos.size(); }
    int get_num_parameters() const
    {   return (int)m_parameter_infos.size(); }
    int get_num_costs() const
    {   return (int)m_cost_infos.size(); }
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
    /// Get the names of all the adjuncts in the order they appear in the 
    /// `adjuncts` input to calc_differential_algebraic_equations(), etc.
    /// Note: this function is not free to call.
    std::vector<std::string> get_adjunct_names() const {
        std::vector<std::string> names;
        for (const auto& info : m_adjunct_infos) {
            names.push_back(info.name);
        }
        return names;
    }
    /// Get the names of all the diffuses in the order they appear in the 
    /// `diffuses` input to calc_differential_algebraic_equations(), etc.
    /// Note: this function is not free to call.
    std::vector<std::string> get_diffuse_names() const {
        std::vector<std::string> names;
        for (const auto& info : m_diffuse_infos) {
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
    /// Get the names of all the cost terms in the order they were added
    /// to the problem (and the order in which they are evaluated).
    /// Note: this function is not free to call.
    std::vector<std::string> get_cost_names() const {
        std::vector<std::string> names;
        for (const auto& info : m_cost_infos) {
            names.push_back(info.name);
        }
        return names;
    }
    /// Obtain whether the cost at the provided index requires an integral. That
    /// is, must the problem override calc_cost_integral()?
    bool get_cost_requires_integral(int index) const {
        return m_cost_infos[index].requires_integral;
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
            const FinalBounds& final_time);
    /// This returns an index that can be used to access this specific state
    /// variable within `dynamics()`, `path_constraints()`, etc.
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
    /// This returns an index that can be used to access this specific adjunct
    /// variable within `dynamics()`, `path_constraints()`, etc.
    /// TODO check if an adjunct with the provided name already exists.
    int add_adjunct(const std::string& name, const Bounds& bounds,
            const InitialBounds& initial_bounds = InitialBounds(),
            const FinalBounds& final_bounds = FinalBounds()) {

        m_adjunct_infos.push_back({name, bounds, initial_bounds, final_bounds});
        return (int)m_adjunct_infos.size() - 1;
    }
    /// This returns an index that can be used to access this specific diffuse
    /// variable within `dynamics()`, `path_constraints()`, etc.
    /// TODO check if an diffuse with the provided name already exists.
    int add_diffuse(const std::string& name, const Bounds& bounds) {

        m_diffuse_infos.push_back({name, bounds});
        return (int)m_diffuse_infos.size() - 1;
    }
    /// This returns an index that can be used to access this specific parameter
    /// variable within `dynamics()` , `path_constraints()`, etc.
    /// TODO check if a parameter with the provided name already exists.
    int add_parameter(const std::string& name, const Bounds& bounds) {
        m_parameter_infos.push_back({name, bounds});
        return (int)m_parameter_infos.size() - 1;
    }
    /// Add a cost term to the problem. Implement the cost via
    /// calc_cost_integrand() and calc_cost(). The index of the cost term is
    /// passed into these functions. The cost can depend only on endpoints of
    /// the phase (e.g., initial and final state) or also on an integral across
    /// the phase.
    /// Use num_integrals to specify whether or not the cost involves an
    /// integral (in which case you must implement calc_cost_integrand()).
    /// num_integrals can be either 0 or 1. In the future, we may support
    /// multiple integrals per cost.
    int add_cost(const std::string& name, int num_integrals);
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

    /// This function is invoked every time there is a new iterate (perhaps
    /// multiple times), and allows you to perform any initialization or caching
    /// based on the constant parameter values from the iterate, in case this is
    /// expensive. Implementing this function is optional, even if your problem
    /// has parameters.
    virtual void initialize_on_iterate(const VectorX<T>& parameters) const;

    /// Compute the right-hand side of the differntial algebraic equations
    /// (DAE) for the system you want to optimize. This is the function that
    /// provides the dynamics and path constraints.
    /// The initial values in `dynamics` and `path` are arbitrary and cannot be 
    /// assumed to be 0, etc. You must set entries to 0 explicitly if you want 
    /// that.
    /// It is not always necessary to compute path constraint values at every 
    /// time point. This is determined by the underlying transcription scheme,
    /// and return values for the `path` structure are only accepted if it is of
    /// non-zero size for a given time index. Therefore, implementations of this
    /// function require detection of the `path` structure size if path
    /// constraints exist in your problem.
    virtual void calc_differential_algebraic_equations(
            const Input<T>& in, Output<T> out) const;
    // TODO alternate form that takes a matrix; state at every time.
    //virtual void continuous(const MatrixX<T>& x, MatrixX<T>& xdot) const = 0;
    // TODO Maybe this one signature could be used for both the "continuous,
    // entire trajectory" mode or the "one time" mode.
    //    var.states[0];
    //    var.states_traj
    //    var.controls[0];
    //    out.dynamics[0] =
    //    out.path[0] = ...
    //}
    /// Compute cost terms as a function of the initial and final states/controls,
    /// and perhaps integrals (for cost terms involving integrals).
    /// Use cost_index to ensure determine which cost to compute.
    virtual void calc_cost(
            int cost_index, const CostInput<T>& in, T& cost) const;
    /// For cost terms with integrals, compute the integrand. Use cost_index
    /// to ensure determine which cost to compute.
    virtual void calc_cost_integrand(
            int cost_index, const Input<T>& in, T& integrand) const;
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
    /// vector filled out. If `guess.controls` is empty, this function will set
    /// its dimensions appropriately according to the provided `guess.time`;
    /// otherwise, `guess.controls` must have the correct dimensions (number of
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
    /// Set a guess for the trajectory of a single adjunct variable with name
    /// `name` to `value`. This function relieves you of the need to know the
    /// index of a adjunct variable. The `guess` must already have its `time`
    /// vector filled out. If `guess.adjuncts` is empty, this function will set
    /// its dimensions appropriately according to the provided `guess.time`;
    /// otherwise, `guess.adjuncts` must have the correct dimensions (number of
    /// adjuncts x mesh points).
    /// @param[in,out] guess
    ///     The row of the adjuncts matrix associated with the provided name
    ///     is set to value.
    /// @param[in] name
    ///     Name of the adjunct variable (provided in add_adjunct()).
    /// @param[in] value
    ///     This must have the same number of columns as `guess.time` and
    ///     `guess.adjuncts`.
    void set_adjunct_guess(Iterate& guess,
            const std::string& name,
            const Eigen::VectorXd& value);
    /// Set a guess for the trajectory of a single diffuse variable with name
    /// `name` to `value`. This function relieves you of the need to know the
    /// index of a diffuse variable. The `guess` must already have its `time`
    /// vector filled out. If `guess.diffuses` is empty, this function will 
    /// set its dimensions appropriately according to the provided `guess.time`;
    /// otherwise, `guess.diffuses` must have the correct dimensions (number 
    /// of diffuses x mesh points).
    /// @param[in,out] guess
    ///     The row of the diffuse matrix associated with the provided name
    ///     is set to value.
    /// @param[in] name
    ///     Name of the diffuse variable (provided in add_diffuse()).
    /// @param[in] value
    ///     This must have the same number of columns as `guess.time` and
    ///     `guess.diffuses`.
    /// @note For convenience, this method has the same behavior as the previous
    /// set_*_guess() methods. However, depending on the transcription scheme 
    /// used to solve the optimal control problem, the guess provided for the 
    /// diffuse variable will be invalid for certain time points. The values at 
    /// these time points will not be included in the transcribed NLP. 
    void set_diffuse_guess(Iterate& guess,
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
            Eigen::Ref<Eigen::VectorXd> adjuncts_lower,
            Eigen::Ref<Eigen::VectorXd> adjuncts_upper,
            Eigen::Ref<Eigen::VectorXd> initial_adjuncts_lower,
            Eigen::Ref<Eigen::VectorXd> initial_adjuncts_upper,
            Eigen::Ref<Eigen::VectorXd> final_adjuncts_lower,
            Eigen::Ref<Eigen::VectorXd> final_adjuncts_upper,
            Eigen::Ref<Eigen::VectorXd> diffuses_lower,
            Eigen::Ref<Eigen::VectorXd> diffuses_upper,
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
    std::vector<ContinuousVariableInfo> m_adjunct_infos;
    std::vector<DiffuseInfo> m_diffuse_infos;
    std::vector<ParameterInfo> m_parameter_infos;
    std::vector<CostInfo> m_cost_infos;
    std::vector<PathConstraintInfo> m_path_constraint_infos;
};

} // namespace tropter

#endif // TROPTER_OPTIMALCONTROL_PROBLEM_H

