#ifndef MUSCOLLO_MUCOPROBLEMREP_H
#define MUSCOLLO_MUCOPROBLEMREP_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoProblemRep.h                                         *
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

#include "osimMuscolloDLL.h"
#include "MucoVariableInfo.h"
#include "MucoCost.h"
#include "MucoConstraint.h"
#include "MucoParameter.h"
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

class MucoProblem;

/// The primary intent of this class is for use by MucoSolver%s, but users
/// can also use this class to apply parameter values to the model
/// and evaluate cost terms.
/// This class also checks the MucoProblem for various errors.
/// To get an instance of this class, use MucoProblem::createRep().
/// This interface currently supports only single-phase problems.
/// This class stores a reference (not a copy) to the original MucoProblem
/// from which it was created.
class OSIMMUSCOLLO_API MucoProblemRep {
public:
    MucoProblemRep() = default;
    MucoProblemRep(const MucoProblemRep&) = delete;
    MucoProblemRep& operator=(const MucoProblemRep&) = delete;
    MucoProblemRep(MucoProblemRep&& source)
            : m_problem(std::move(source.m_problem)) {
        if (m_problem) initialize();
    }
    MucoProblemRep& operator=(MucoProblemRep&& source) {
        m_problem = std::move(source.m_problem);
        if (m_problem) initialize();
        return *this;
    }

    const std::string& getName() const;

    /// Get a reference to the copy of the model being used by this MucoProblemRep.
    /// This model is the same instance as that given to MucoCost,
    /// MucoParameter, and MucoPathConstraint.
    const Model& getModel() const { return m_model; }
    /// Get the state names of all the state infos.
    std::vector<std::string> createStateInfoNames() const;
    /// Get the control names of all the control infos.
    std::vector<std::string> createControlInfoNames() const;
    /// Get the names of all the parameters.
    std::vector<std::string> createParameterNames() const;
    /// Get the names of all the MucoPathConstraints.
    std::vector<std::string> createPathConstraintNames() const;
    /// Get the names of all the Lagrange multiplier infos.
    std::vector<std::string> createMultiplierInfoNames() const;
    /// Get the constraint names of all the multibody constraints. Note: this
    /// should only be called after initialize().
    std::vector<std::string> createMultibodyConstraintNames() const;
    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    MucoInitialBounds getTimeInitialBounds() const;
    /// @copydoc getTimeInitialBounds()
    MucoFinalBounds getTimeFinalBounds() const;
    /// Get information for state variables. If info was not specified for
    /// a coordinate value, the coordinate range is used for the bounds.
    /// If info was not specified for a coordinate speed, the
    /// default_speed_bounds property is used.
    const MucoVariableInfo& getStateInfo(const std::string& name) const;
    /// Get information for actuator controls. If info was not specified for
    /// an actuator, the actuator's min and max control are used for the bounds.
    const MucoVariableInfo& getControlInfo(const std::string& name) const;
    const MucoParameter& getParameter(const std::string& name) const;
    /// Get a MucoPathConstraint from this MucoPhase. Note: this does not
    /// include MucoMultibodyConstraints, use getMultibodyConstraint() instead.
    const MucoPathConstraint& getPathConstraint(const std::string& name) const;

    /// Get the number of scalar path constraints in the MucoProblem. This does
    /// not include multibody constraints equations.
    int getNumPathConstraintEquations() const {
        OPENSIM_THROW_IF(m_num_path_constraint_eqs == -1, Exception,
                "The number of scalar path constraint equations is not available "
                "until after initialization.");
        return m_num_path_constraint_eqs;
    }
    /// Given a multibody constraint name, get a vector of MucoVariableInfos
    /// corresponding to the Lagrange multipliers for that multibody constraint.
    /// Note: Since these are created directly from model constraint
    /// information, this should only be called after initialization. TODO
    const std::vector<MucoVariableInfo>&
    getMultiplierInfos(const std::string& multibodyConstraintInfoName) const;
    /// Get a MucoMultibodyConstraint from this MucoPhase. Note: this does not
    /// include MucoPathConstraints, use getPathConstraint() instead. Since
    /// these are created directly from model information, this should only be
    /// called after initialization. TODO
    const MucoMultibodyConstraint&
    getMultibodyConstraint(const std::string& name) const;
    /// Get the number of scalar multibody constraints in the MucoProblem. This
    /// does not include path constraints equations.
    int getNumMultibodyConstraintEquations() const {
        OPENSIM_THROW_IF(m_num_multibody_constraint_eqs == -1, Exception,
                "The number of scalar multibody constraint equations is not "
                "available until after initialization.");
        return m_num_multibody_constraint_eqs;
    }

    /// Print a description of this problem, including costs and variable
    /// bounds. By default, the description is printed to the console (cout),
    /// but you can provide your own stream.
    void printDescription(std::ostream& stream = std::cout) const;

    /// @name Interface for solvers
    /// These functions are for use by MucoSolver%s, but can also be called
    /// by users for debugging.
    /// @{
    /// Calculate the sum of integrand over all the integral cost terms in this
    /// phase for the provided state. That is, the returned value is *not* an
    /// integral over time.
    SimTK::Real calcIntegralCost(const SimTK::State& state) const {
        SimTK::Real integrand = 0;
        for (const auto& cost : m_costs) {
            integrand += cost->calcIntegralCost(state);
        }
        return integrand;
    }
    /// Calculate the sum of all the endpoint cost terms in this phase.
    SimTK::Real calcEndpointCost(const SimTK::State& finalState) const {
        SimTK::Real sum = 0;
        // TODO cannot use controls.
        for (const auto& cost : m_costs) {
            sum += cost->calcEndpointCost(finalState);
        }
        return sum;
    }
    /// Calculate the errors in all the scalar path constraint equations in this
    /// phase.
    void calcPathConstraintErrors(const SimTK::State& state,
            SimTK::Vector& errors) const {

        OPENSIM_THROW_IF(
                errors.size() != getNumPathConstraintEquations(), Exception,
                "The size of the errors vector passed is not consistent with the "
                "number of scalar path constraint equations in this MucoProblem.");

        for (const auto& pc : m_path_constraints) {
            pc->calcPathConstraintErrors(state, errors);
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
        for (int i = 0; i < (int)m_multibody_constraints.size(); ++i) {
            thisConstraintNumEqs =
                    m_multibody_constraints[i].getConstraintInfo().getNumEquations();

            SimTK::Vector theseErrors(thisConstraintNumEqs,
                    errors.getContiguousScalarData() + index, true);
            m_multibody_constraints[i].calcMultibodyConstraintErrors(
                    getModel(), state, theseErrors);

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

private:
    explicit MucoProblemRep(const MucoProblem& problem);
    friend MucoProblem;

    void initialize();

    const MucoProblem* m_problem;

    Model m_model;

    std::unordered_map<std::string, MucoVariableInfo> m_state_infos;
    std::unordered_map<std::string, MucoVariableInfo> m_control_infos;

    std::vector<std::unique_ptr<MucoParameter>> m_parameters;
    std::vector<std::unique_ptr<MucoCost>> m_costs;
    std::vector<std::unique_ptr<MucoPathConstraint>> m_path_constraints;
    int m_num_path_constraint_eqs = -1;
    int m_num_multibody_constraint_eqs = -1;
    std::vector<MucoMultibodyConstraint> m_multibody_constraints;
    std::map<std::string, std::vector<MucoVariableInfo>> m_multiplier_infos_map;
};

} // namespace OpenSim


#endif // MUSCOLLO_MUCOPROBLEMREP_H
