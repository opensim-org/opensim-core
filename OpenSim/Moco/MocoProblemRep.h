#ifndef OPENSIM_MOCOPROBLEMREP_H
#define OPENSIM_MOCOPROBLEMREP_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoProblemRep.h                                                  *
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

#include "MocoConstraint.h"
#include "MocoGoal/MocoGoal.h"
#include "MocoParameter.h"
#include "MocoVariableInfo.h"
#include "osimMocoDLL.h"

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Actuators/DeGrooteFregly2016Muscle.h>

namespace OpenSim {

class MocoProblem;
class DiscreteController;
class DiscreteForces;
class PositionMotion;
class AccelerationMotion;

/// The primary intent of this class is for use by MocoSolver%s, but users
/// can also use this class to apply parameter values to the model
/// and evaluate cost terms.
/// This class also checks the MocoProblem for various errors.
/// To get an instance of this class, use MocoProblem::createRep().
/// This interface currently supports only single-phase problems.
/// This class stores a reference (not a copy) to the original MocoProblem
/// from which it was created.
///
/// @par ModelBase and ModelDisabledConstraints
/// This class provides access to two models: ModelBase is obtained by
/// processing the ModelProcessor that the user gives to MocoProblem.
/// ModelDisabledConstraints is a copy of ModelBase in which all kinematic
/// constraints are disabled.
/// ModelDisabledConstraints contains a DiscreteForces component, which
/// is used to apply constraint forces computed using ModelBase.
/// If kinematics are not prescribed (with PositionMotion),
/// ModelDisabledConstraints also contains an AccelerationMotion component,
/// which is used by solvers that rely on implicit multibody dynamics.
/// The initialize() function adds a DiscreteController
/// to both models; this controller is used by a solver to set the control
/// signals for actuators to use.
/// To learn the need for and use of these two models, see @ref impldiverse.

class OSIMMOCO_API MocoProblemRep {
public:
    MocoProblemRep() = default;
    MocoProblemRep(const MocoProblemRep&) = delete;
    MocoProblemRep& operator=(const MocoProblemRep&) = delete;
    MocoProblemRep(MocoProblemRep&& source)
            : m_problem(std::move(source.m_problem)) {
        if (m_problem) initialize();
    }
    MocoProblemRep& operator=(MocoProblemRep&& source) {
        m_problem = std::move(source.m_problem);
        if (m_problem) initialize();
        return *this;
    }

    const std::string& getName() const;

    /// Get a reference to the copy of the model being used by this
    /// MocoProblemRep. This model is obtained by processing the ModelProcessor
    /// the user gives to MocoProblem. This model is *not* the model given to
    /// MocoGoal or MocoPathConstraint, but can be used within solvers to
    /// compute constraint forces and constraint errors (see
    /// getModelDisabledConstraints() for more details). Any parameter updates
    /// via a MocoParameter added to the problem will be applied to this model.
    const Model& getModelBase() const { return m_model_base; }
    /// This is a state object that solvers can use along with ModelBase.
    SimTK::State& updStateBase() const { return m_state_base; }
    /// This is a component inside ModelBase that you can use to
    /// set the value of control signals.
    const DiscreteController& getDiscreteControllerBase() const {
        return m_discrete_controller_base.getRef();
    }
    /// Get a reference to a copy of the model being used by this
    /// MocoProblemRep, but with all constraints disabled and an additional
    /// DiscreteForces component. This new component can be used to apply
    /// constraint forces computed from the base model to this model, which
    /// updates the discrete variables in the state associated with these
    /// forces. You should use this model to compute accelerations via
    /// getModelDisabledConstraints().realizeAccleration(state), making sure to
    /// add any constraint forces to the model preceding the realization. This
    /// model is the same instance as that given to MocoGoal and
    /// MocoPathConstraint, ensuring that realizing to Stage::Acceleration
    /// in these classes produces the same accelerations computed by the solver.
    /// Any parameter updates via a MocoParameter added to the problem
    /// will be applied to this model.
    const Model& getModelDisabledConstraints() const {
        return m_model_disabled_constraints;
    }
    /// This is a state object that solvers can use with
    /// ModelDisabledConstraints. Some solvers may need to use 2 state objects
    /// at once; you can supply an index of 1 to get a second state object.
    SimTK::State& updStateDisabledConstraints(int index = 0) const {
        assert(index <= 1);
        return m_state_disabled_constraints[index];
    }
    /// This is a component inside ModelDisabledConstraints that you can use to
    /// set the value of control signals.
    const DiscreteController& getDiscreteControllerDisabledConstraints() const {
        return m_discrete_controller_disabled_constraints.getRef();
    }
    /// This is a component inside ModelDisabledConstraints that you can use
    /// to set the value of discrete forces, intended to hold the constraint
    /// forces obtained from ModelBase.
    const DiscreteForces& getConstraintForces() const {
        return m_constraint_forces.getRef();
    }
    /// This is a component inside ModelDisabledConstraints that you can use
    /// to set the value of generalized accelerations UDot, for use in
    /// implicit dynamics formulations. The motion is not necessarily enabled.
    const AccelerationMotion& getAccelerationMotion() const {
        return m_acceleration_motion.getRef();
    }
    int getNumStates() const { return (int)m_state_infos.size(); }
    int getNumControls() const { return (int)m_control_infos.size(); }
    int getNumParameters() const { return (int)m_parameters.size(); }
    /// Get the number of goals in cost mode.
    int getNumCosts() const { return (int)m_costs.size(); }
    /// Get the number of goals in endpoint constraint mode.
    int getNumEndpointConstraints() const {
        return (int)m_endpoint_constraints.size();
    }
    int getNumKinematicConstraints() const {
        return (int)m_kinematic_constraints.size();
    }
    /// Does the model contain a PositionMotion to prescribe all generalized
    /// coordinates, speeds, and accelerations?
    bool isPrescribedKinematics() const { return m_prescribedKinematics; }
    int getNumImplicitAuxiliaryResiduals() const {
        return (int)m_implicit_residual_refs.size();
    }
    /// This excludes generalized coordinate and speed states if
    /// isPrescribedKinematics() is true.
    std::vector<std::string> createStateVariableNamesInSystemOrder(
            std::unordered_map<int, int>& yIndexMap) const;
    /// Get the state names of all the state infos.
    std::vector<std::string> createStateInfoNames() const;
    /// Get the control names of all the control infos.
    std::vector<std::string> createControlInfoNames() const;
    /// Get the names of all the parameters.
    std::vector<std::string> createParameterNames() const;
    /// Get the names of all the goals in cost mode.
    std::vector<std::string> createCostNames() const;
    /// Get the names of all the goals in endpoint constraint mode.
    std::vector<std::string> createEndpointConstraintNames() const;
    /// Get the names of all the MocoPathConstraint%s.
    std::vector<std::string> createPathConstraintNames() const;
    /// Get the names of all the Lagrange multiplier infos.
    std::vector<std::string> createMultiplierInfoNames() const;
    /// Get the constraint names of all the kinematic constraints. Note: this
    /// should only be called after initialize().
    std::vector<std::string> createKinematicConstraintNames() const;
    /// Get a vector of names for all kinematic constraint equations.
    /// Kinematic constraint equations are ordered as so:
    /// - position-level constraints
    /// - velocity-level constraints
    /// - acceleration-level constraints
    /// If includeDerivatives is true, the ordering is:
    /// - position-level constraints
    /// - first derivative of position-level constraints (denoted by suffix "d")
    /// - velocity-level constraints
    /// - second derivative of position-level constraints (suffix "dd")
    /// - first derivative of velocity-level constraints (suffix "d")
    /// - acceleration-level constraints
    std::vector<std::string> getKinematicConstraintEquationNames(
            bool includeDerivatives) const;
    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    MocoInitialBounds getTimeInitialBounds() const;
    /// @copydoc getTimeInitialBounds()
    MocoFinalBounds getTimeFinalBounds() const;
    /// Get information for state variables. See MocoPhase::setStateInfo().
    const MocoVariableInfo& getStateInfo(const std::string& name) const;
    /// Get information for actuator controls.
    /// If the control is associated with a non-scalar actuator (i.e. uses
    /// multiple control variables), then the control name will be the actuator
    /// path appended by the control index (e.g. "/actuator_0");
    /// See MocoPhase::setControlInfo().
    const MocoVariableInfo& getControlInfo(const std::string& name) const;
    const MocoParameter& getParameter(const std::string& name) const;
    /// Get a cost by name. This returns a MocoGoal in cost mode.
    const MocoGoal& getCost(const std::string& name) const;
    /// Get a cost by index. The order is the same as in getCostNames().
    /// Note: this does not perform a bounds check.
    const MocoGoal& getCostByIndex(int index) const;
    /// Get an endpoint constraint by name. This returns a MocoGoal in endpoint
    /// constraint mode.
    const MocoGoal& getEndpointConstraint(const std::string& name) const;
    /// Get an endpoint constraint by index.
    /// The order is the same as in getEndpointConstraintNames().
    /// Note: this does not perform a bounds check.
    const MocoGoal& getEndpointConstraintByIndex(int index) const;
    /// Get a MocoPathConstraint. Note: this does not
    /// include MocoKinematicConstraints, use getKinematicConstraint() instead.
    const MocoPathConstraint& getPathConstraint(const std::string& name) const;
    /// Get a path constraint by index. The order is the same as
    /// in getPathConstraintNames(). Note: this does not perform a bounds check.
    const MocoPathConstraint& getPathConstraintByIndex(int index) const;

    /// Get the number of scalar path constraints in the MocoProblem. This does
    /// not include kinematic constraints equations.
    int getNumPathConstraintEquations() const {
        OPENSIM_THROW_IF(m_num_path_constraint_equations == -1, Exception,
                "The number of scalar path constraint equations is not "
                "available until after initialization.");
        return m_num_path_constraint_equations;
    }
    /// Given a kinematic constraint name, get a vector of MocoVariableInfos
    /// corresponding to the Lagrange multipliers for that kinematic constraint.
    const std::vector<MocoVariableInfo>& getMultiplierInfos(
            const std::string& kinematicConstraintInfoName) const;
    /// Get a MocoKinematicConstraint from this MocoPhase. Note: this does not
    /// include MocoPathConstraints, use getPathConstraint() instead.
    const MocoKinematicConstraint& getKinematicConstraint(
            const std::string& name) const;
    /// Get the number of scalar kinematic constraints in the MocoProblem. This
    /// does not include path constraints equations.
    int getNumKinematicConstraintEquations() const {
        OPENSIM_THROW_IF(m_num_kinematic_constraint_equations == -1, Exception,
                "The number of scalar kinematic constraint equations is not "
                "available until after initialization.");
        return m_num_kinematic_constraint_equations;
    }

    /// Print a description of this problem, including costs and variable
    /// bounds. Printing is done using OpenSim::log_cout().
    void printDescription() const;

    /// @name Interface for solvers
    /// These functions are for use by MocoSolver%s, but can also be called
    /// by users for debugging.
    /// @{
    /// Calculate the errors in all the scalar path constraint equations in this
    /// phase.
    void calcPathConstraintErrors(
            const SimTK::State& state, SimTK::Vector& errors) const {

        OPENSIM_THROW_IF(errors.size() != getNumPathConstraintEquations(),
                Exception,
                "The size of the errors vector passed is not consistent with "
                "the "
                "number of scalar path constraint equations in this "
                "MocoProblem.");

        for (const auto& pc : m_path_constraints) {
            pc->calcPathConstraintErrorsView(state, errors);
        }
    }
    /// Calculate the errors in all the scalar kinematic constraint equations in
    /// this phase. This may not be the most efficient solution for solvers, but
    /// is rather intended as a convenience method for a quick implementation or
    /// for debugging model constraints causing issues in an optimal control
    /// problem.
    SimTK::Vector calcKinematicConstraintErrors(
            const SimTK::State& state) const {
        SimTK::Vector errors(getNumKinematicConstraintEquations(), 0.0);
        int index = 0;
        int thisConstraintNumEquations;
        for (int i = 0; i < (int)m_kinematic_constraints.size(); ++i) {
            thisConstraintNumEquations = m_kinematic_constraints[i]
                                                 .getConstraintInfo()
                                                 .getNumEquations();

            SimTK::Vector theseErrors(thisConstraintNumEquations,
                    errors.getContiguousScalarData() + index, true);
            m_kinematic_constraints[i].calcKinematicConstraintErrors(
                    getModelBase(), state, theseErrors);

            index += thisConstraintNumEquations;
        }
        return errors;
    }

    /// Apply paramater values to the models created from the model passed to
    /// initialize() within the current MocoProblem. Values must be consistent
    /// with the order of parameters returned from createParameterNames().
    ///
    /// Note: initSystem() must be called on each model after calls to this
    /// method in order for provided parameter values to be applied to the
    /// model. You can pass `true` to have initSystem() called for you, and to
    /// also re-disable any constraints re-enabled by the initSystem() call
    /// (see getModelDisabledConstraints()).
    void applyParametersToModelProperties(const SimTK::Vector& parameterValues,
            bool initSystemAndDisableConstraints = false) const;

    /// Get a vector of reference pointers to model outputs that return residual
    /// values for any components with dynamics in implicit forms. The 
    /// references returned are from the model returned by 
    /// getModelDisabledConstraints(). 
    const std::vector<SimTK::ReferencePtr<const Output<double>>>&
    getImplicitResidualReferencePtrs() const {
        return m_implicit_residual_refs;
    }

    /// Get reference pointers to components that enforce dynamics in implicit 
    /// form. This returns a vector of pairs including the name of the discrete
    /// derivative variable and the component reference pointer.
    const 
    std::vector<std::pair<std::string, SimTK::ReferencePtr<const Component>>>&
    getImplicitComponentReferencePtrs() const {
        return m_implicit_component_refs;
    }
    /// @}

private:
    explicit MocoProblemRep(const MocoProblem& problem);
    friend MocoProblem;

    void initialize();

    /// Get a list of reference pointers to all outputs whose names (not paths)
    /// match a substring defined by a provided regex string pattern. The regex
    /// string pattern could be the full name of the output. Only Output%s that
    /// match the template argument type will be returned (double is the default
    /// type). Set the argument 'includeDescendents' to true to include outputs
    /// from all descendents (subcomponents) from the provided component.
    template <typename T = double>
    std::vector<SimTK::ReferencePtr<const Output<T>>>
    getModelOutputReferencePtrs(const Component& component,
            const std::string& pattern, bool includeDescendents = false) {
        // Create regex.
        std::regex regex(pattern);
        // Initialize outputs array.
        std::vector<SimTK::ReferencePtr<const Output<T>>> outputs;

        std::function<void(const Component&, const std::regex&, bool,
                std::vector<SimTK::ReferencePtr<const Output<T>>>&)> helper;
        helper = [&helper](const Component& component, const std::regex& regex,
                bool includeDescendents,
                std::vector<SimTK::ReferencePtr<const Output<T>>>& outputs) {
          // Store a reference to outputs that match the template
          // parameter type and whose names contain the provided
          // substring.
          for (const auto& entry : component.getOutputs()) {
              const std::string& name = entry.first;
              const auto foundSubstring = std::regex_match(name, regex);
              const auto* output =
                      dynamic_cast<const Output<T>*>(entry.second.get());
              if (output && foundSubstring) {
                  outputs.emplace_back(output);
              }
          }

          // Repeat for all subcomponents.
          if (includeDescendents) {
              for (const Component& thisComp :
                      component.getComponentList<Component>()) {
                  if (&thisComp == &component) { continue; }
                  helper(thisComp, regex, false, outputs);
              }
          }
        };

        helper(component, regex, includeDescendents, outputs);
        return outputs;
    }

    const MocoProblem* m_problem;

    Model m_model_base;
    mutable SimTK::State m_state_base;
    SimTK::ReferencePtr<const DiscreteController> m_discrete_controller_base;
    SimTK::ReferencePtr<const PositionMotion> m_position_motion_base;

    Model m_model_disabled_constraints;
    mutable std::array<SimTK::State, 2> m_state_disabled_constraints;
    SimTK::ReferencePtr<const DiscreteController>
            m_discrete_controller_disabled_constraints;
    SimTK::ReferencePtr<const PositionMotion>
            m_position_motion_disabled_constraints;
    SimTK::ReferencePtr<DiscreteForces> m_constraint_forces;
    SimTK::ReferencePtr<AccelerationMotion> m_acceleration_motion;

    bool m_prescribedKinematics = false;

    std::unordered_map<std::string, MocoVariableInfo> m_state_infos;
    std::unordered_map<std::string, MocoVariableInfo> m_control_infos;

    std::vector<std::unique_ptr<MocoParameter>> m_parameters;
    std::vector<std::unique_ptr<MocoGoal>> m_costs;
    std::vector<std::unique_ptr<MocoGoal>> m_endpoint_constraints;
    std::vector<std::unique_ptr<MocoPathConstraint>> m_path_constraints;
    int m_num_path_constraint_equations = -1;
    int m_num_kinematic_constraint_equations = -1;
    std::vector<MocoKinematicConstraint> m_kinematic_constraints;
    std::map<std::string, std::vector<MocoVariableInfo>> m_multiplier_infos_map;
    std::vector<std::string> m_kinematic_constraint_eq_names_with_derivatives;
    std::vector<std::string>
            m_kinematic_constraint_eq_names_without_derivatives;

    std::vector<SimTK::ReferencePtr<const Output<double>>>
            m_implicit_residual_refs;
    std::vector<std::pair<std::string, SimTK::ReferencePtr<const Component>>>
            m_implicit_component_refs;

    static const std::vector<std::string> m_disallowedJoints;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOPROBLEMREP_H
