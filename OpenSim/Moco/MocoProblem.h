#ifndef OPENSIM_MOCOPROBLEM_H
#define OPENSIM_MOCOPROBLEM_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoProblem.h                                                     *
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

#include "MocoProblemRep.h"
#include "OpenSim/Actuators/ModelProcessor.h"

namespace OpenSim {

// ============================================================================
// MocoPhase
// ============================================================================

/** The states, controls, dynamics, parameters, goals, and constraints for a
phase of the problem.
The dynamics are provided by the %OpenSim Model.

This class allows you to define your problem, but does not let you do
anything with your problem (this class only contains user input).
Use MocoProblem::createRep() to create an instance of MocoProblemRep,
which provides additional functionality. */
class OSIMMOCO_API MocoPhase : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoPhase, Object);

public:
    MocoPhase();

    /// Set the Model whose dynamics should be used for this phase.
    /// The phase takes ownership of the passed-in model. This function
    /// returns a pointer to the model stored in the phase (identical to the
    /// passed-in model).
    Model* setModel(std::unique_ptr<Model> model);
    /// The model is copied into the MocoPhase; further changes made to the
    /// passed-in model will have no effect on this MocoPhase.
    /// This function returns a pointer to the model stored in the phase
    /// (the copy).
    Model* setModelAsCopy(Model model);
    /// Set a model processor for creating the model for this phase. Use this
    /// to provide a model as a .osim file.
    void setModelProcessor(ModelProcessor model);
    /// Get a mutable reference to the internal ModelProcessor. Use this to
    /// set the processor's base model or to add operators to the processor.
    ModelProcessor& updModelProcessor();
    /// Set the bounds on the initial and final time for this phase.
    /// If you want to constrain the initial time to a single value, pass
    /// that value to the constructor of MocoInitialBounds. If you want the
    /// initial time to fall within a range, pass the lower and upper bounds
    /// to the constructor of MocoInitialBounds. Likewise for MocoFinalBounds.
    /// This will overwrite bounds that were set previously, if any.
    void setTimeBounds(const MocoInitialBounds&, const MocoFinalBounds&);
    /// Find and print the names of all state variables containing a substring.
    void printStateNamesWithSubstring (const std::string& name);
    /// Set information about a single state variable in this phase.
    /// @param name
    ///     The name must match the path of a state variable in the
    ///     model (e.g., `/hip/flexion/value` or `/hip/flexion/speed`).
    /// @param bounds
    ///     The bounds on this state variable over the entire phase. If
    ///     default-constructed (`{}`), then either the variable is
    ///     unconstrained or default bounds are used (see below).
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
    /// Default bounds
    /// 1. Coordinate values: the Coordinate's range is used (regardless of
    ///     whether the coordinate is clamped).
    /// 2. Coordinate speeds: this class's default_speed_bounds property.
    /// 3. All other states: if a component has a SimTK::Vec2 output named
    ///    `statebounds_<state-name>`, then this output is used to set default
    ///    bounds for the state variable `<state-name>` for that component.
    ///    The first element of the Vec2 is the lower bound and the second is
    ///    the upper bound.
    ///
    /// These defaults are also used if you completely omit state info for a
    /// state variable.
    ///
    /// For states with default bounds, if you actually want a variable to
    /// be unconstrained, pass in MocoBounds::unconstrained().
    ///
    /// Examples
    /// Set bounds over the entire phase, but do not specify additional
    /// bounds on the value at the start and end of the phase.
    /// @code{.cpp}
    /// phase.setStateInfo("/knee/flexion/value", {-1.5*SimTK::Pi, 0});
    /// @endcode
    ///
    /// Allow any value throughout the phase (within the coordinate's range),
    /// but the initial value is 5.
    /// @code{.cpp}
    /// phase.setStateInfo("/ankle/flexion/value", {}, 5);
    /// @endcode
    ///
    /// Constrain the initial and final state to a single value of 0, but
    /// use default speed bounds elsewhere.
    /// @code{.cpp}
    /// phase.setStateInfo("/ankle/flexion/speed", {}, 0, 0);
    /// @endcode
    ///
    /// Make a coordinate value unconstrained.
    /// @code{.cpp}
    /// phase.setStateInfo("/ankle/flexion/value", MocoBounds::unconstrained());
    /// @endcode
    ///
    /// This function will overwrite any info that has previously been set for
    /// this state variable.
    void setStateInfo(const std::string& name, const MocoBounds& bounds,
            const MocoInitialBounds& init = {},
            const MocoFinalBounds& final = {});
    /// Set information for state variables whose names match the provided
    /// regular expression. You can use this to set bounds for all muscle
    /// activations, etc. Infos provided via setStateInfoPattern() take
    /// precedence over the default values from the model. Infos provided via
    /// setStateInfo() take precedence over infos provided with
    /// setStateInfoPattern().  If a state variable name matches multiple
    /// patterns, the info provided with the last pattern is used for that state
    /// variable.
    void setStateInfoPattern(const std::string& pattern,
            const MocoBounds& bounds, const MocoInitialBounds& init = {},
            const MocoFinalBounds& final = {});
    /// Find and print the names of all control variables containing a substring.
    void printControlNamesWithSubstring(const std::string& name);
    /// Set information about a single control variable in this phase.
    /// Similar to setStateInfo(). The name for a control is the path to the
    /// associated actuator (e.g., "/forceset/soleus_r"). If setting a control
    /// info for an actuator with multiple controls, the name should be the
    /// actuator path appended by the control index (e.g. "/actuator_0");
    /// If info is not specified for a ScalarActuator (or if only the initial
    /// and/or final bounds are provided), the actuator's min and max control
    /// are used for the bounds over the phase. By default, non-ScalarActuators
    /// are unconstrained.
    void setControlInfo(const std::string& name, const MocoBounds&,
            const MocoInitialBounds& = {}, const MocoFinalBounds& = {});

    /// Set the bounds on generalized speed state variables
    /// for which explicit bounds are not set.
    void setDefaultSpeedBounds(const MocoBounds& bounds) {
        set_default_speed_bounds(bounds);
    }
    /// Set information for control variables whose names match the provided
    /// regular expression. You can use this to set bounds for all muscle
    /// activations, etc. Infos provided via setControlInfoPattern() take
    /// precedence over the default values from the model. Infos provided via
    /// setControlInfo() take precedence over infos provided with
    /// setControlInfoPattern().  If a state variable name matches multiple
    /// patterns, the info provided with the last pattern is used for that
    /// control variable.
    void setControlInfoPattern(const std::string& pattern, const MocoBounds&,
            const MocoInitialBounds& = {}, const MocoFinalBounds& = {});
    /// For muscles without explicit activation bounds, set the bounds for
    /// muscle activation (if activation dynamics are enabled) from the bounds
    /// for muscle control (excitation), using min/max control if explicit
    /// control bounds are not provided. Default: true.
    void setBoundActivationFromExcitation(bool tf) {
        set_bound_activation_from_excitation(tf);
    }
    /// Set the bounds on *all* of the kinematic constraint equations in this
    /// phase. When creating a MocoProblemRep, these bounds are used to create
    /// MocoConstraintInfo's for each kinematic constraint equation in the
    /// phase.
    void setKinematicConstraintBounds(const MocoBounds& bounds) {
        set_kinematic_constraint_bounds(bounds);
    }
    /// Set the bounds on *all* of the Lagrange multipliers in this phase.
    /// When creating a MocoProblemRep, these bounds are used to create
    /// MocoVariableInfo%s for each Lagrange multiplier in the phase.
    void setMultiplierBounds(const MocoBounds& bounds) {
        set_multiplier_bounds(bounds);
    }
    /// Add a parameter to this phase.
    /// Parameter variables must have a name (MocoParameter::setName()), and the
    /// name must be unique. Note that parameters have the name "parameter" by
    /// default, but choosing a more appropriate name is recommended.
    /// C++ example:
    /// @code{.cpp}
    /// // Using the base MocoParameter directly.
    /// auto param0Ptr = phase.addParameter("mass", "body", "mass", {0, 10});
    /// // Using a custom MocoParameter.
    /// auto param1Ptr = phase.addParameter<MyCustomParameter>(...);
    /// @endcode
    /// You can edit the parameter using the returned pointer.
    /// Python example:
    /// @code{.py}
    /// param = opensim.MocoParameter()
    /// phase.addParameter(param)
    /// @endcode
    /// Matlab example:
    /// @code
    /// param = org.opensim.modeling.MocoParameter();
    /// phase.addParameter(param);
    /// @endcode
    /// In both Python and Matlab, changes to `param` will affect your problem.
    template <typename MocoParamType, typename... Args>
    MocoParamType* addParameter(Args&&... args) {
        return addParameter(std::unique_ptr<MocoParamType>(
                new MocoParamType(std::forward<Args>(args)...)));
    }
    template <typename MocoParamType>
    MocoParamType* addParameter(std::unique_ptr<MocoParamType> param) {
        MocoParamType* ptr = param.get();
        updProperty_parameters().adoptAndAppendValue(param.release());
        return ptr;
    }
    /// Add a goal to this phase.
    /// Goal must have a name (MocoGoal::setName()), and the name must be
    /// unique. Note that goals have the name "goal" by default, so if you
    /// only have one goal, you don't need to set its name manually.
    /// C++ example:
    /// @code{.cpp}
    /// auto goal0Ptr = phase.addGoal<MocoFinalTimeGoal>();
    /// auto goal1Ptr = phase.addGoal<MocoFinalTimeGoal>("final_time");
    /// @endcode
    /// You can edit the goal using the returned pointer.
    /// Python example:
    /// @code{.py}
    /// goal = opensim.MocoFinalTimeGoal()
    /// phase.addGoal(goal)
    /// @endcode
    /// Matlab example:
    /// @code
    /// goal = org.opensim.modeling.MocoFinalTimeGoal();
    /// phase.addGoal(goal);
    /// @endcode
    /// In both Python and Matlab, changes to `goal` will affect your problem.
    template <typename MocoGoalType, typename... Args>
    MocoGoalType* addGoal(Args&&... args) {
        return addGoal(std::unique_ptr<MocoGoalType>(
                new MocoGoalType(std::forward<Args>(args)...)));
    }
    /// Add a goal term to this phase.
    /// Similar to above.
    template <typename MocoGoalType>
    MocoGoalType* addGoal(std::unique_ptr<MocoGoalType> goal) {
        MocoGoalType* ptr = goal.get();
        updProperty_goals().adoptAndAppendValue(goal.release());
        return ptr;
    }

    /// Add a path constraint to this phase.
    /// Path constraints must have a name (MocoPathConstraint::setName()), and
    /// the name must be unique. Note that path constraints have the name
    /// "path_constraint" by default, so if you only have one path constraint,
    /// you don't need to set its name manually.
    /// C++ example:
    /// @code{.cpp}
    /// auto pcPtr = phase.addPathConstraint<MyPathConstraint>();
    /// @endcode
    /// You can edit the constraint using the returned pointer.
    /// Python example:
    /// @code{.py}
    /// pc = opensim.MyPathConstraint()
    /// phase.addPathConstraint(pc)
    /// @endcode
    /// Matlab example:
    /// @code
    /// pc = MyPathConstraint();
    /// phase.addPathConstraint(pc);
    /// @endcode
    /// In both Python and Matlab, changes to `pc` will affect your problem.
    template <typename MocoPCType, typename... Args>
    MocoPCType* addPathConstraint(Args&&... args) {
        return addPathConstraint(std::unique_ptr<MocoPCType>(
                new MocoPCType(std::forward<Args>(args)...)));
    }
    /// Add a path constraint to this phase.
    /// Similar to above.
    template <typename MocoPCType>
    MocoPCType* addPathConstraint(std::unique_ptr<MocoPCType> pc) {
        MocoPCType* ptr = pc.get();
        updProperty_path_constraints().adoptAndAppendValue(pc.release());
        return ptr;
    }

    /// Get the base model in the internal ModelProcessor. This throws an
    /// exception if the ModelProcessor does not have a base model. By default,
    /// the model is an empty model.
    const Model& getModel() const { return get_model().getModel(); }
    /// Get a mutable reference to the base model in the internal
    /// ModelProcessor. This throws an exception if the ModelProcessor does not
    /// have a base model. By default, the model is an empty model.
    Model& updModel() { return upd_model().updModel(); }
    /// Get the ModelProcessor.
    const ModelProcessor& getModelProcessor() const { return get_model(); }

    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    MocoInitialBounds getTimeInitialBounds() const;
    /// @copydoc getTimeInitialBounds()
    MocoFinalBounds getTimeFinalBounds() const;
    /// Access explicit state infos provided to this phase. For some state
    /// variables, default bounds are obtained from the model.
    /// This function does *not* provide such automatically-populated bounds
    /// from the model. For that, use see MocoProblemRep::getStateInfo().
    const MocoVariableInfo& getStateInfo(const std::string& name) const;
    /// Access explicit control infos provided to this phase.
    /// Default bounds are obtained from the model.
    /// This function does *not* provide such automatically-populated bounds
    /// from the model. For that, use see MocoProblemRep::getControlInfo().
    const MocoVariableInfo& getControlInfo(const std::string& name) const;

    const MocoBounds& getDefaultSpeedBounds() const {
        return get_default_speed_bounds();
    }
    bool getBoundActivationFromExcitation() const {
        return get_bound_activation_from_excitation();
    }
    const MocoBounds& getKinematicConstraintBounds() const {
        return get_kinematic_constraint_bounds();
    }
    const MocoBounds& getMultiplierBounds() const {
        return get_multiplier_bounds();
    }

    const MocoParameter& getParameter(const std::string& name) const;
    MocoParameter& updParameter(const std::string& name);

    const MocoGoal& getGoal(const std::string& name) const;
    MocoGoal& updGoal(const std::string& name);

    /// Get a MocoPathConstraint from this MocoPhase. Note: this does not
    /// include MocoKinematicConstraints, use getKinematicConstraint() instead.
    const MocoPathConstraint& getPathConstraint(const std::string& name) const;
    MocoPathConstraint& updPathConstraint(const std::string& name);

protected: // Protected so that doxygen shows the properties.
    OpenSim_DECLARE_PROPERTY(
            model, ModelProcessor, "OpenSim Model to provide dynamics.");
    OpenSim_DECLARE_PROPERTY(
            time_initial_bounds, MocoInitialBounds, "Bounds on initial value.");
    OpenSim_DECLARE_PROPERTY(
            time_final_bounds, MocoFinalBounds, "Bounds on final value.");
    OpenSim_DECLARE_PROPERTY(default_speed_bounds, MocoBounds,
            "Bounds for coordinate speeds if not specified in "
            "state_infos (default: [-50, 50]).");
    OpenSim_DECLARE_PROPERTY(bound_activation_from_excitation, bool,
            "For muscles without explicit activation bounds, set the bounds "
            "for muscle activation (if activation dynamics are enabled) from " 
            "the bounds for muscle control (excitation), using "             
            "min/max control if explicit control bounds are not "            
            "provided. (default: true).");
    OpenSim_DECLARE_LIST_PROPERTY(
            state_infos, MocoVariableInfo, "The state variables' bounds.");
    OpenSim_DECLARE_LIST_PROPERTY(state_infos_pattern, MocoVariableInfo,
            "Set state variable bounds for all states matching a regular "
            "expression.");
    OpenSim_DECLARE_LIST_PROPERTY(
            control_infos, MocoVariableInfo, "The control variables' bounds.");
    OpenSim_DECLARE_LIST_PROPERTY(control_infos_pattern, MocoVariableInfo,
            "Set control variable bounds for all controls matching a regular "
            "expression.");
    OpenSim_DECLARE_LIST_PROPERTY(parameters, MocoParameter,
            "Parameter variables (model properties) to optimize.");
    OpenSim_DECLARE_LIST_PROPERTY(goals, MocoGoal,
            "Integral/endpoint quantities to minimize or constrain.");
    OpenSim_DECLARE_LIST_PROPERTY(path_constraints, MocoPathConstraint,
            "Path constraints to enforce in the optimal control problem.");
    // TODO make this a list property of MocoConstraintInfos when we are able to
    // map OpenSim constraint names to Simbody constraints.
    OpenSim_DECLARE_PROPERTY(kinematic_constraint_bounds, MocoBounds,
            "The bounds on all the kinematic constraints in the model to be "
            "enforced. By default the constraints are strictly enforced (zero "
            "bounds).");
    OpenSim_DECLARE_PROPERTY(multiplier_bounds, MocoBounds,
            "Variable info to apply to all Lagrange multipliers in the "
            "problem. "
            "The default bounds are [-1000 1000].");

private:
    void constructProperties();

    friend MocoProblemRep;
};

// ============================================================================
// MocoProblem
// ============================================================================

/** A description of an optimal control problem, backed by %OpenSim Model%s.
A MocoProblem is a series of phases, each of which contains the following:
  - OpenSim Model
  - state and control variable info (e.g., bounds)
  - parameter variables (model properties)
  - goals (costs and endpoint constraints)
  - path constraints

Currently, only single-phase problems are supported.
This class has convenience methods to configure the first (0-th) phase.

This class allows you to define your problem, but does not let you do
anything with your problem (this class only contains user input).
Use createRep() to create an instance of MocoProblemRep,
which provides additional functionality. */
class OSIMMOCO_API MocoProblem : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoProblem, Object);

public:
    MocoProblem();

    /// @name Convenience methods for phase 0.
    /// These methods allow you to conveniently edit phase 0 of the problem.
    /// See MocoPhase's documentation for more information.
    /// @{

    /// Set the model to use for phase 0.
    /// @see MocoPhase::setModel().
    Model* setModel(std::unique_ptr<Model> model);
    /// Set the model to use for phase 0.
    /// @see MocoPhase::setModelAsCopy().
    Model* setModelAsCopy(Model model);
    /// Update the model in phase 0.
    Model& updModel() { return upd_phases(0).updModel(); }
    /// Set a model processor for phase 0.
    /// @see MocoPhase::setModelProcessor().
    void setModelProcessor(ModelProcessor model);
    /// Set time bounds for phase 0.
    void setTimeBounds(const MocoInitialBounds&, const MocoFinalBounds&);
    /// Find and print the names of all state variables containing a substring.
    void printStateNamesWithSubstring(const std::string& name);
    /// Set bounds for a state variable for phase 0.
    void setStateInfo(const std::string& name, const MocoBounds&,
            const MocoInitialBounds& = {}, const MocoFinalBounds& = {});
    /// Set bounds for all state variables for phase 0 whose path matches
    /// the provided pattern.
    // TODO: We tried to give an example regex but it had characters that caused
    // doxygen to not produce documentation for this entire file.
    void setStateInfoPattern(const std::string& pattern,
            const MocoBounds& bounds, const MocoInitialBounds& init = {},
            const MocoFinalBounds& final = {});
    /// Find and print the names of all state variables containing a substring.
    void printControlNamesWithSubstring(const std::string& name);
    /// Set bounds for a control variable for phase 0.
    void setControlInfo(const std::string& name, const MocoBounds&,
            const MocoInitialBounds& = {}, const MocoFinalBounds& = {});
    /// Set bounds for a control variable using a regular expression.
    void setControlInfoPattern(const std::string& pattern, const MocoBounds&,
            const MocoInitialBounds& = {}, const MocoFinalBounds& = {});
    /// Set bounds for the kinematic constraints in phase 0.
    void setKinematicConstraintBounds(const MocoBounds& bounds);
    /// Set bounds for the Lagrange multipliers in phase 0.
    void setMultiplierBounds(const MocoBounds& bounds);
    /// Add a parameter variable for phase 0.
    /// @see MocoPhase::addParameter()
    template <typename MocoParamType = MocoParameter, typename... Args>
    MocoParamType* addParameter(Args&&... args) {
        return upd_phases(0).addParameter(std::unique_ptr<MocoParamType>(
                new MocoParamType(std::forward<Args>(args)...)));
    }
    /// Add a parameter variable for phase 0.
    template <typename MocoParamType = MocoParameter>
    MocoParamType* addParameter(std::unique_ptr<MocoParamType> param) {
        return upd_phases(0).addParameter(std::move(param));
    }
    /// Add a goal for phase 0.
    /// @see MocoPhase::addGoal()
    template <typename MocoGoalType, typename... Args>
    MocoGoalType* addGoal(Args&&... args) {
        return upd_phases(0).addGoal(std::unique_ptr<MocoGoalType>(
                new MocoGoalType(std::forward<Args>(args)...)));
    }
    /// Add a goal for phase 0.
    template <typename MocoGoalType>
    MocoGoalType* addGoal(std::unique_ptr<MocoGoalType> goal) {
        return upd_phases(0).addGoal(std::move(goal));
    }
    /// Returns a reference to the goal with name "name" in phase 0.
    MocoGoal& updGoal(const std::string& name);
    /// Add a constraint for phase 0.
    /// @see MocoPhase::addPathConstraint()
    template <typename MocoPCType, typename... Args>
    MocoPCType* addPathConstraint(Args&&... args) {
        return upd_phases(0).addPathConstraint(std::unique_ptr<MocoPCType>(
                new MocoPCType(std::forward<Args>(args)...)));
    }
    /// Add a constraint for phase 0.
    template <typename MocoPCType>
    MocoPCType* addPathConstraint(std::unique_ptr<MocoPCType> pc) {
        return upd_phases(0).addPathConstraint(std::move(pc));
    }
    /// @}

    /// Get a modifiable phase of the problem by index (starting index of 0).
    /// This accesses the internal phases property.
    MocoPhase& updPhase(int index = 0) { return upd_phases(index); }
    /// Get a modifiable phase of the problem by index (starting index of 0).
    /// This accesses the internal phases property.
    const MocoPhase& getPhase(int index = 0) const { return get_phases(index); }

#ifndef SWIG // MocoProblemRep() is not copyable.
    /// Create an instance of MocoProblemRep, which fills in additional
    /// state and control bounds, and allows you to apply parameter values
    /// and evaluate the goals.
    ///
    /// This function will check your problem for various errors.
    MocoProblemRep createRep() const { return MocoProblemRep(*this); }
#endif
    /// Use this variant of createRep() if you require the MocoProblemRep to be
    /// dynamically-allocated MocoProblemRep.
    std::unique_ptr<MocoProblemRep> createRepHeap() const {
        return std::unique_ptr<MocoProblemRep>(new MocoProblemRep(*this));
    }

    friend MocoProblemRep;

protected: // We'd prefer private, but protected means it shows up in Doxygen.
    // TODO OpenSim_DECLARE_LIST_PROPERTY_ATLEAST(phases, MocoPhase, 1,
    OpenSim_DECLARE_LIST_PROPERTY_SIZE(
            phases, MocoPhase, 1, "List of 1 or more MocoPhases.");

private:
    void constructProperties();
};

} // namespace OpenSim

#endif // OPENSIM_MOCOPROBLEM_H
