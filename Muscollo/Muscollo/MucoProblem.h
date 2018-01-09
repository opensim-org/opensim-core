#ifndef MUSCOLLO_MUCOPROBLEM_H
#define MUSCOLLO_MUCOPROBLEM_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoProblem.h                                            *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia, Nicholas Bianco                             *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "MucoCost.h"
#include "MucoParameter.h"

#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

class MucoPhase;
class MucoVariableInfo;
class MucoParameter;

/// Small struct to handle bounds.
/// You can access the bound values directly (.lower, .upper).
struct OSIMMUSCOLLO_API MucoBounds {
    /// The bounds are NaN, which means (-inf, inf).
    MucoBounds() = default;
    /// The lower and upper bound are equal (the variable is constrained to this
    /// single value).
    MucoBounds(double value) : lower(value), upper(value) {}
    /// The variable is constrained to be within [lower, upper].
    MucoBounds(double lower, double upper) {
        OPENSIM_THROW_IF(lower > upper, Exception,
                "Expected lower <= upper, but lower=" + std::to_string(lower)
                + " and upper=" + std::to_string(upper) + ".");
        this->lower = lower;
        this->upper = upper;
    }
    /// True if the lower and upper bounds are both not NaN.
    bool isSet() const {
        return !SimTK::isNaN(lower) && !SimTK::isNaN(upper);
    }
    /// The returned array has either 0, 1, or 2 elements.
    /// - 0 elements: bounds are not set.
    /// - 1 element: equality constraint
    /// - 2 elements: range (inequality constraint).
    Array<double> getAsArray() const {
        Array<double> vec;
        if (isSet()) {
            vec.append(lower);
            if (lower != upper) vec.append(upper);
        }
        return vec;
    }
    double lower = SimTK::NTraits<double>::getNaN();
    double upper = SimTK::NTraits<double>::getNaN();
protected:
    /// Used internally to create Bounds from a list property.
    /// The list property must have either 0, 1 or 2 elements.
    MucoBounds(const Property<double>& p) {
        assert(p.size() <= 2);
        if (p.size() >= 1) {
            lower = p[0];
            if (p.size() == 2) upper = p[1];
            else               upper = p[0];
        }
    }

    friend MucoPhase;
    friend MucoVariableInfo;
    friend MucoParameter;
};
/// Used for specifying the bounds on a variable at the start of a phase.
struct OSIMMUSCOLLO_API MucoInitialBounds : public MucoBounds {
    using MucoBounds::MucoBounds;
    friend MucoPhase;
    friend MucoVariableInfo;
};
/// Used for specifying the bounds on a variable at the end of a phase.
struct OSIMMUSCOLLO_API MucoFinalBounds : public MucoBounds {
    using MucoBounds::MucoBounds;
    friend MucoPhase;
    friend MucoVariableInfo;
};

class MucoPhase;


// ============================================================================
// MucoVariableInfo
// ============================================================================

/// Bounds on continuous variables (states, controls). The name should
/// correspond to path of a state variable or an actuator in the model.
class OSIMMUSCOLLO_API MucoVariableInfo : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoVariableInfo, Object);
public:
    MucoVariableInfo();
    MucoVariableInfo(const std::string& name, const MucoBounds&,
            const MucoInitialBounds&, const MucoFinalBounds&);

    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    MucoBounds getBounds() const
    {   return MucoBounds(getProperty_bounds()); }
    /// @copydoc getBounds()
    MucoInitialBounds getInitialBounds() const
    {   return MucoInitialBounds(getProperty_initial_bounds()); }
    /// @copydoc getBounds()
    MucoFinalBounds getFinalBounds() const
    {   return MucoFinalBounds(getProperty_final_bounds()); }

protected:

    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(bounds, double, 2,
            "1 value: required value over all time. "
            "2 values: lower, upper bounds on value over all time.");
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(initial_bounds, double, 2,
            "1 value: required initial value. "
            "2 values: lower, upper bounds on initial value.");
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(final_bounds, double, 2,
            "1 value: required final value. "
            "2 values: lower, upper bounds on final value.");

private:
    void constructProperties();
};


// ============================================================================
// MucoPhase
// ============================================================================

/// The states, controls, dynamics, parameters, and costs for a phase of the 
/// problem.
/// The dynamics are provided by the %OpenSim Model.
///
/// Workflow
/// --------
/// 1. Set the model (setModel()).
/// 2. Set time bounds, state and control information.
/// 3. Add parameter and cost terms.
///
/// Supported %Model Component%s
/// ----------------------------
/// Muscollo does not support all types of models. Specifically, the
/// following components are not supported:
///   - Constraint%s
///   - Actuator%s with multiple controls (non-ScalarActuator%s).
// TODO documentation for properties.
class OSIMMUSCOLLO_API MucoPhase : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoPhase, Object);
public:
    MucoPhase();

    /// Set the Model whose dynamics should be used for this phase.
    /// The model is copied into the MucoPhase; further changes made to the
    /// passed-in model will have no effect on this MucoPhase.
    void setModel(const Model&);
    /// Set the bounds on the initial and final time for this phase.
    /// If you want to constrain the initial time to a single value, pass
    /// that value to the constructor of MucoInitialBounds. If you want the
    /// initial time to fall within a range, pass the lower and upper bounds
    /// to the constructor of MucoInitialBounds. Likewise for MucoFinalBounds.
    /// This will overwrite bounds that were set previously, if any.
    void setTimeBounds(const MucoInitialBounds&, const MucoFinalBounds&);
    /// Set information about a single state variable in this phase.
    /// @param name
    ///     The name must match the path of a state variable in the
    ///     model (e.g., `hip/flexion/value` or `hip/flexion/speed`).
    /// @param bounds
    ///     The bounds on this state variable over the entire phase. If
    ///     default-constructed (`{}`), then either no bounds are applied or
    ///     bounds are taken from the model (depending on the type of state
    ///     variable).
    /// @param init
    ///     The bounds on this state variable at the start of the phase.
    ///     By default, there are no additional bounds on the initial value
    ///     (though the `bounds` over the entire phase still apply to the
    ///     initial value).
    /// @param final
    ///     Similar to `init` but for the value at the end of the phase.
    ///
    /// For all bounds arguments: if you want to constrain to a single value,
    /// pass that single value. If you want to constrain to a range, pass
    /// the lower and upper bounds to the constructor as two arguments.
    ///
    /// ### Examples
    /// Set bounds over the entire phase, but do not specify additional
    /// bounds on the value at the start and end of the phase.
    /// @code{.cpp}
    /// phase.setStateInfo("knee/flexion/value", {-1.5*SimTK::Pi, 0});
    /// @endcode
    ///
    /// Allow any value throughout the phase (within the coordinate's range,
    /// if clamped), but the initial value is 5.
    /// @code{.cpp}
    /// phase.setStateInfo("ankle/flexion/value", {}, 5);
    /// @endcode
    ///
    /// Constrain the initial and final state to a single value of 0.
    /// @code{.cpp}
    /// phase.setStateInfo("ankle/flexion/speed", {}, 0, 0);
    /// @endcode
    ///
    /// This function will overwrite any info that has previously been set for
    /// this state variable.
    ///
    /// @precondition
    ///     The completed model must be set.
    void setStateInfo(const std::string& name, const MucoBounds& bounds,
            const MucoInitialBounds& init = {},
            const MucoFinalBounds& final = {});
    /// Set information about a single control variable in this phase.
    /// Similar to setStateInfo(). The name for a control is the path to the
    /// associated ScalarActuator (e.g., "soleus_r").
    ///
    /// @precondition
    ///     The completed model must be set.
    // TODO by default, use the actuator's control_min and control_max.
    void setControlInfo(const std::string& name, const MucoBounds&,
            const MucoInitialBounds& = {}, const MucoFinalBounds& = {});
    /// Add a parameter to this phase. The passed-in parameter is copied, and
    /// thus any subsequent edits have not effect.
    /// Parameter variables must have a name (MucoParameter::setName()), and it
    /// must be unique. Note that parameters have the name "parameter" by 
    /// default, but choosing a more appropriate name is recommended.
    ///
    /// @precondition
    ///     The completed model must be set.
    void addParameter(const MucoParameter&);
    /// Add a cost term to this phase. The passed-in cost is copied, and thus
    /// any subsequent edits have no effect.
    /// Cost terms must have a name (MucoCost::setName()), and it must be
    /// unique. Note that costs have the name "cost" by default, so if you
    /// only have one cost, you don't need to set its name manually.
    ///
    /// @precondition
    ///     The completed model must be set.
    void addCost(const MucoCost&);

    const Model& getModel() const { return get_model(); }
    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    MucoInitialBounds getTimeInitialBounds() const;
    /// @copydoc getTimeInitialBounds()
    MucoFinalBounds getTimeFinalBounds() const;
    /// Get the state names of for all the state infos.
    std::vector<std::string> createStateInfoNames() const;
    /// Get the control names of for all the control infos.
    std::vector<std::string> createControlInfoNames() const;
    const MucoVariableInfo& getStateInfo(const std::string& name) const;
    const MucoVariableInfo& getControlInfo(const std::string& name) const;

    // TODO add getCost() and/or updCost().


    /// @name Interface for solvers
    /// These functions are for use by MucoSolver%s, but can also be called
    /// by users for debugging. Make sure to call initialize() before invoking
    /// any other functions in this group.
    /// @{

    /// Invoked by the solver in preparation for solving the problem.
    // TODO make private and make MucoProblem a friend.
    void initialize(const Model&) const;
    /// Calculate the sum of integrand over all the integral cost terms in this
    /// phase for the provided state. That is, the returned value is *not* an
    /// integral over time.
    SimTK::Real calcIntegralCost(const SimTK::State& state) const {
        SimTK::Real integrand = 0;
        for (int i = 0; i < getProperty_costs().size(); ++i) {
            integrand += get_costs(i).calcIntegralCost(state);
        }
        return integrand;
    }
    /// Calculate the sum of all the endpoint cost terms in this phase.
    SimTK::Real calcEndpointCost(const SimTK::State& finalState) const {
        SimTK::Real cost = 0;
        // TODO cannot use controls.
        for (int i = 0; i < getProperty_costs().size(); ++i) {
            cost = get_costs(i).calcEndpointCost(finalState);
        }
        return cost;
    }

    /// @}

    // TODO print list of state and control names.
    //void printDescription();


protected: // Protected so that doxygen shows the properties.
    OpenSim_DECLARE_PROPERTY(model, Model,
            "OpenSim Model to provide dynamics.");
    // TODO error if not provided.
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(time_initial_bounds, double, 2,
            "1 value: required initial time. "
            "2 values: lower, upper bounds on initial time.");
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(time_final_bounds, double, 2,
            "1 value: required final time. "
            "2 values: lower, upper bounds on final time.");
    OpenSim_DECLARE_LIST_PROPERTY(state_infos, MucoVariableInfo,
            "The state variables' bounds.");
    OpenSim_DECLARE_LIST_PROPERTY(control_infos, MucoVariableInfo,
            "The control variables' bounds.");
    OpenSim_DECLARE_LIST_PROPERTY(parameters, MucoParameter,
            "Parameter variables (model property values) to optimize.");
    OpenSim_DECLARE_LIST_PROPERTY(costs, MucoCost,
            "Quantities to minimize in the cost functional.");

private:
    void constructProperties();
};


// ============================================================================
// MucoProblem
// ============================================================================

/// A description of an optimal control problem, backed by %OpenSim Model%s.
/// A MucoProblem is composed as follows:
///   - 1 or more MucoPhase%s (only 1 phase supported currently).
///   - OpenSim Model
///   - state and control variable info (e.g., bounds)
///   - parameter and cost terms
/// Most problems only have 1 phase. This class has convenience methods to
/// configure the first (0-th) phase.
class OSIMMUSCOLLO_API MucoProblem : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoProblem, Object);
public:
    MucoProblem();

    /// @name Convenience methods for phase 0.
    /// These methods allow you to conveniently edit phase 0 of the problem.
    /// See MucoPhase's documentation for more information.
    /// @{

    /// Set the model to use for phase 0.
    void setModel(const Model&);
    /// Set time bounds for phase 0.
    void setTimeBounds(const MucoInitialBounds&, const MucoFinalBounds&);
    /// Set bounds for a state variable for phase 0.
    void setStateInfo(const std::string& name, const MucoBounds&,
            const MucoInitialBounds& = {}, const MucoFinalBounds& = {});
    /// Set bounds for a control variable for phase 0.
    void setControlInfo(const std::string& name, const MucoBounds&,
            const MucoInitialBounds& = {}, const MucoFinalBounds& = {});
    /// Add a parameter term for phase 0.
    void addParameter(const MucoParameter&);
    /// Add a cost term for phase 0.
    void addCost(const MucoCost&);
    /// @}

    // TODO access phase by name

    /// Get a modifiable phase of the problem by index (starting index of 0).
    /// This accesses the internal phases property.
    MucoPhase& updPhase(int index = 0)
    {   return upd_phases(index); }
    /// Get a modifiable phase of the problem by index (starting index of 0).
    /// This accesses the internal phases property.
    const MucoPhase& getPhase(int index = 0) const
    {   return get_phases(index); }

    /// @name Interface for solvers
    /// These functions are for use by MucoSolver%s, but can also be called
    /// by users for debugging.
    /// @{

    /// Invoked by the solver in preparation for solving the problem.
    /// This also performs error checks on the Problem.
    // TODO create intermediate classes so that the solvers can have a
    // non-const intermediate that contains a const MucoProblem; the
    // intermediate can determine what parts of the MucoProblem to
    // reveal/allow changing.
    void initialize(const Model&) const;

    /// @}

    // TODO
    // TODO check that
    //void checkWellPosed();
protected: // We'd prefer private, but protected means it shows up in Doxygen.
    // TODO OpenSim_DECLARE_LIST_PROPERTY_ATLEAST(phases, MucoPhase, 1,
    OpenSim_DECLARE_LIST_PROPERTY_SIZE(phases, MucoPhase, 1,
            "List of 1 or more MucoPhases.");

private:
    void constructProperties();
};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOPROBLEM_H
