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
#include "MucoBounds.h"
#include "MucoParameter.h"
#include "MucoConstraint.h"

#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

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

    MucoBounds getBounds() const
    {   return get_bounds(); }
    MucoInitialBounds getInitialBounds() const
    {   return get_initial_bounds(); }
    MucoFinalBounds getFinalBounds() const
    {   return get_final_bounds(); }

    /// Throws an exception if initial and final bounds are not within bounds.
    // TODO Move to finalizeFromProperties() and cache MucoBounds.
    void validate() const;

    /// Print the bounds on this variable.
    void printDescription(std::ostream& stream = std::cout) const;

protected:

    OpenSim_DECLARE_UNNAMED_PROPERTY(bounds, MucoBounds,
            "Bounds over all time.");
    OpenSim_DECLARE_UNNAMED_PROPERTY(initial_bounds, MucoInitialBounds,
            "Bounds on initial value.");
    OpenSim_DECLARE_UNNAMED_PROPERTY(final_bounds, MucoFinalBounds,
            "Bounds on final value.");

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
    /// Set the bounds on *all* of the multibody constraint equations in this
    /// phase. During initialization, these bounds are used to create 
    /// MucoConstraintInfo's for each multibody constraint equation in the 
    /// phase.
    void setMultibodyConstraintBounds(const MucoBounds& bounds)
    {   set_multibody_constraint_bounds(bounds); }
    /// Set the bounds on *all* of the Lagrange multipliers in this phase. 
    /// During initialization, these bounds are used to create 
    /// MucoVariableInfo's for each Lagrange multiplier in the phase.
    void setMultiplierBounds(const MucoBounds& bounds)
    {   set_multiplier_bounds(bounds); }
    /// Add a parameter to this phase. The passed-in parameter is copied, and
    /// thus any subsequent edits have no effect.
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
    /// Add a path constraint to this phase. The passed-in constriant is copied, 
    /// and thus any subsequent edits have not effect.
    /// Path constraints must have a name (MucoPathConstraint::setName()), and 
    /// it must be unique. Note that path constraints have the name 
    /// "path_constraint" by default, so if you only have one path constraint, 
    /// you don't need to set its name manually.
    ///
    /// @precondition
    ///     The completed model must be set.
    void addPathConstraint(const MucoPathConstraint&);

    const Model& getModel() const { return get_model(); }
    Model& updModel() { return upd_model(); }

    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    MucoInitialBounds getTimeInitialBounds() const;
    /// @copydoc getTimeInitialBounds()
    MucoFinalBounds getTimeFinalBounds() const;
    /// Get the state names of all the state infos.
    std::vector<std::string> createStateInfoNames() const;
    /// Get the control names of all the control infos.
    std::vector<std::string> createControlInfoNames() const;
    /// Get the names of all the Lagrange multiplier infos;
    std::vector<std::string> createMultiplierInfoNames() const;
    /// Get the names of all the parameters.
    std::vector<std::string> createParameterNames() const;
    /// Get the names of all the MucoPathConstraints.
    std::vector<std::string> createPathConstraintNames() const;
    /// Get the constraint names of all the multibody constraints. Note: this
    /// should only be called after initialization().
    std::vector<std::string> createMultibodyConstraintNames() const;
    const MucoVariableInfo& getStateInfo(const std::string& name) const;
    const MucoVariableInfo& getControlInfo(const std::string& name) const;
    const MucoParameter& getParameter(const std::string& name) const;
    MucoParameter& updParameter(const std::string& name);
    /// Get a MucoPathConstraint from this MucoPhase. Note: this does not 
    /// include MucoMultibodyConstraints, use getMultibodyConstraint() instead.
    const MucoPathConstraint& getPathConstraint(const std::string& name) const;
    /// Get the number of scalar path constraints in the MucoProblem. This does 
    /// not include multibody constraints equations, and is only available after 
    /// initialization. 
    int getNumPathConstraintEquations() const {   
        OPENSIM_THROW_IF(m_num_path_constraint_eqs == -1, Exception,
            "The number of scalar path constraint equations is not available "
            "until after initialization.");
        return m_num_path_constraint_eqs;
    }
    /// Get a MucoMultibodyConstraint from this MucoPhase. Note: this does not 
    /// include MucoPathConstraints, use getPathConstraint() instead. Since 
    /// these are created directly from model information, this should only be 
    /// called after initialization.
    const MucoMultibodyConstraint& 
    getMultibodyConstraint(const std::string& name) const;
    /// Get the number of scalar multibody constraints in the MucoProblem. This 
    /// does not include path constraints equations, and is only available after 
    /// initialization. 
    int getNumMultibodyConstraintEquations() const {
        OPENSIM_THROW_IF(m_num_multibody_constraint_eqs == -1, Exception,
            "The number of scalar multibody constraint equations is not "
            "available until after initialization.");
        return m_num_multibody_constraint_eqs;
    }
    /// Given a multibody constraint name, get a vector of MucoVariableInfos 
    /// corresponding to the Lagrange multipliers for that multibody constraint.
    /// Note: Since these are created directly from model constraint
    /// information, this should only be called after initialization.
    const std::vector<MucoVariableInfo>& 
    getMultiplierInfos(const std::string& multibodyConstraintInfoName) const;

    // TODO add getCost() and/or updCost().

    /// Print a brief description of the costs and variables in this phase.
    void printDescription(std::ostream& stream = std::cout) const;

    /// @name Interface for solvers
    /// These functions are for use by MucoSolver%s, but can also be called
    /// by users for debugging. Make sure to call initialize() before invoking
    /// any other functions in this group.
    /// @{

    /// Invoked by the solver in preparation for solving the problem.
    /// The passed-in model is a non-const reference because MucoParameter needs
    /// the ability to make changes to the model.
    void initialize(Model&) const;
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
    /// Calculate the errors in all the scalar path constraint equations in this
    /// phase.
    void calcPathConstraintErrors(const SimTK::State& state, 
        SimTK::Vector& errors) const {
        
        OPENSIM_THROW_IF_FRMOBJ(
            errors.size() != getNumPathConstraintEquations(), Exception,
            "The size of the errors vector passed is not consistent with the "
            "number of scalar path constraint equations in this MucoProblem.");

        for (int i = 0; i < getProperty_path_constraints().size(); ++i) {
            get_path_constraints(i).calcPathConstraintErrors(state, errors);
        }
    }
    /// Calculate the errors in all the scalar multibody constraint equations in
    /// this phase. This may not be the most efficient solution for solvers, but 
    /// is rather intended as a convenience method for a quick implementation or 
    /// for debugging model constraints causing issues in an optimal control 
    /// problem.
    SimTK::Vector calcMultibodyConstraintErrors(const SimTK::State& state) const 
    {
        SimTK::Vector errors(getNumMultibodyConstraintEquations(), 0.0);
        int index = 0;
        int thisConstraintNumEqs;
        for (int i = 0; i < m_multibody_constraints.size(); ++i) {
            thisConstraintNumEqs = 
            m_multibody_constraints[i].getConstraintInfo().getNumEquations();

            SimTK::Vector theseErrors(thisConstraintNumEqs, 
                errors.getContiguousScalarData() + index, true);
            m_multibody_constraints[i].calcMultibodyConstraintErrors(getModel(), 
                state, theseErrors);

            index += thisConstraintNumEqs;
        }
        return errors;
    }

    /// Apply paramater values to the model passed to initialize() within the
    /// current MucoProblem. Values must be consistent with the order of 
    /// parameters returned from createParameterNames().
    ///
    /// Note: initSystem() must be called on the model after calls to this
    /// method in order for provided parameter values to be applied to the 
    /// model.
    void applyParametersToModel(const SimTK::Vector& parameterValues) const;

    /// @}

protected: // Protected so that doxygen shows the properties.
    OpenSim_DECLARE_PROPERTY(model, Model,
            "OpenSim Model to provide dynamics.");
    // TODO error if not provided.
    OpenSim_DECLARE_UNNAMED_PROPERTY(time_initial_bounds, MucoInitialBounds,
            "Bounds on initial value.");
    OpenSim_DECLARE_UNNAMED_PROPERTY(time_final_bounds, MucoFinalBounds,
            "Bounds on final value.");
    OpenSim_DECLARE_LIST_PROPERTY(state_infos, MucoVariableInfo,
            "The state variables' bounds.");
    OpenSim_DECLARE_LIST_PROPERTY(control_infos, MucoVariableInfo,
            "The control variables' bounds.");
    OpenSim_DECLARE_LIST_PROPERTY(parameters, MucoParameter,
            "Parameter variables (model properties) to optimize.");
    OpenSim_DECLARE_LIST_PROPERTY(costs, MucoCost,
            "Quantities to minimize in the cost functional.");
    OpenSim_DECLARE_LIST_PROPERTY(path_constraints, MucoPathConstraint,
            "Path constraints to enforce in the optimal control problem.");
    // TODO make this a list property of MucoConstraintInfos when we are able to
    // map OpenSim constraint names to Simbody constraints.
    OpenSim_DECLARE_PROPERTY(multibody_constraint_bounds, MucoBounds,
        "The bounds on all the multibody constraints in the model to be "
        "enforced. By default the constraints are strictly enforced (zero "
        "bounds).");
    OpenSim_DECLARE_PROPERTY(multiplier_bounds, MucoBounds,
        "Variable info to apply to all Lagrange multipliers in the problem. "
        "The default bounds are [-1000 1000].");

private:
    void constructProperties();
    mutable int m_num_path_constraint_eqs = -1;
    mutable int m_num_multibody_constraint_eqs = -1;
    mutable std::vector<MucoMultibodyConstraint> 
        m_multibody_constraints;
    mutable std::map<std::string, std::vector<MucoVariableInfo>>
        m_multiplier_infos_map;

};


// ============================================================================
// MucoProblem
// ============================================================================

/// A description of an optimal control problem, backed by %OpenSim Model%s.
/// A MucoProblem is composed as follows:
///   - 1 or more MucoPhase%s (only 1 phase supported currently).
///   - OpenSim Model
///   - state and control variable info (e.g., bounds)
///   - parameter variables (model properties)
///   - cost terms
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
    /// Set bounds for the multibody constraints in phase 0.
    void setMultibodyConstraintBounds(const MucoBounds& bounds);
    /// Set bounds for the Lagrange multipliers in phase 0.
    void setMultiplierBounds(const MucoBounds& bounds);
    /// Add a parameter variable for phase 0.
    void addParameter(const MucoParameter&);
    /// Add a cost term for phase 0.
    void addCost(const MucoCost&);
    /// Add a constraint for phase 0.
    void addPathConstraint(const MucoPathConstraint&);
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

    /// Print a description of this problem, including costs and variable
    /// bounds. By default, the description is printed to the console (cout),
    /// but you can provide your own stream.
    void printDescription(std::ostream& stream = std::cout) const;

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
    void initialize(Model&) const;

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
