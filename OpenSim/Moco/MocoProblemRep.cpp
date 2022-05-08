/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoProblemRep.cpp                                           *
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

#include "Components/AccelerationMotion.h"
#include "Components/DiscreteController.h"
#include "Components/DiscreteForces.h"
#include "MocoProblem.h"
#include "MocoProblemInfo.h"
#include "MocoScaleFactor.h"
#include <regex>
#include <unordered_set>

#include <OpenSim/Simulation/PositionMotion.h>
#include <OpenSim/Simulation/SimulationUtilities.h>

using namespace OpenSim;

const std::vector<std::string> MocoProblemRep::m_disallowedJoints(
        {"FreeJoint", "BallJoint", "EllipsoidJoint", "ScapulothoracicJoint"});

MocoProblemRep::MocoProblemRep(const MocoProblem& problem)
        : m_problem(&problem) {
    initialize();
}
void MocoProblemRep::initialize() {

    // Clear member variables.
    m_model_base = Model();
    m_model_base.updDisplayHints().disableVisualization();
    m_state_base.clear();
    m_position_motion_base.reset();
    m_model_disabled_constraints = Model();
    m_model_disabled_constraints.updDisplayHints().disableVisualization();
    m_position_motion_disabled_constraints.reset();
    m_constraint_forces.reset();
    m_acceleration_motion.reset();
    m_state_infos.clear();
    m_control_infos.clear();
    m_parameters.clear();
    m_costs.clear();
    m_endpoint_constraints.clear();
    m_path_constraints.clear();
    m_kinematic_constraints.clear();
    m_multiplier_infos_map.clear();
    m_kinematic_constraint_eq_names_with_derivatives.clear();
    m_kinematic_constraint_eq_names_without_derivatives.clear();
    m_implicit_component_refs.clear();
    m_implicit_residual_refs.clear();

    if (!getTimeInitialBounds().isSet() && !getTimeFinalBounds().isSet()) {
        log_warn("No time bounds set.");
    }

    const auto& ph0 = m_problem->getPhase(0);
    // TODO: Provide directory from which to load model file.
    m_model_base = ph0.getModelProcessor().process();

    auto discreteControllerBaseUPtr = make_unique<DiscreteController>();
    m_discrete_controller_base.reset(discreteControllerBaseUPtr.get());
    m_model_base.addController(discreteControllerBaseUPtr.release());

    // Scale factors
    // -------------
    int numScaleFactors = 0;
    std::unordered_set<std::string> scaleFactorNames;
    for (int i = 0; i < ph0.getProperty_goals().size(); ++i) {
        const auto& goal = ph0.get_goals(i);
        std::vector<MocoScaleFactor> scaleFactors = goal.getScaleFactors();
        for (const auto& scaleFactor : scaleFactors) {
            OPENSIM_THROW_IF(scaleFactor.getName().empty(), Exception,
                    "All scale factors must have a name.");
            OPENSIM_THROW_IF(scaleFactorNames.count(scaleFactor.getName()),
                     Exception, "A scale factor with name '{}' already exists.",
                     scaleFactor.getName());
            scaleFactorNames.insert(scaleFactor.getName());
            MocoScaleFactor* thisScaleFactor = new MocoScaleFactor(
                    scaleFactor.getName(),
                    scaleFactor.getBounds());
            m_model_base.addComponent(thisScaleFactor);
            ++numScaleFactors;
        }
    }
    m_model_base.finalizeFromProperties();

    int countMotion = 0;
    for (const auto& comp : m_model_base.getComponentList<PositionMotion>()) {
        // Next line exists only to avoid an "unused variable" compiler warning.
        comp.getName();
        if (comp.getDefaultEnabled()) {
            ++countMotion;
            OPENSIM_THROW_IF(countMotion > 1, Exception,
                    "The model cannot contain more than 1 PositionMotion.");
            m_prescribedKinematics = true;
        }
    }

    // We disable the PrescribedMotion by default so that,
    // if there are constraints, the AssemblySolver does not complain about
    // having 0 parameters with which to satisfy the constraints. After
    // we're done with the assembly in initSystem(), we can re-enable the
    // prescribed motion.
    if (m_prescribedKinematics) {
        auto& posmotBase =
                *m_model_base.updComponentList<PositionMotion>().begin();
        posmotBase.setDefaultEnabled(false);
    }

    m_state_base = m_model_base.initSystem();

    if (m_prescribedKinematics) {
        m_position_motion_base.reset(
                &*m_model_base.getComponentList<PositionMotion>().begin());
        m_position_motion_base->setEnabled(m_state_base, true);
    }

    // Disallow joints where the derivative of the generalized coordinates does
    // not equal the generalized speeds.
    for (const auto& joint : m_model_base.getComponentList<Joint>()) {
        const std::string& jointType = joint.getConcreteClassName();
        if (std::find(m_disallowedJoints.begin(), m_disallowedJoints.end(),
                      jointType) != m_disallowedJoints.end()) {
            OPENSIM_THROW(Exception, "{} with name '{}' detected, but "
                                     "{}s are not yet supported in Moco "
                                     "(since qdot != u). Consider replacing "
                                     "with a CustomJoint.",
                          jointType, joint.getName(), jointType);
        }
    }

    // We would like to eventually compute the model accelerations through
    // realizing to Stage::Acceleration. However, if the model has constraints,
    // realizing to Stage::Acceleration will cause Simbody to compute it's own
    // Lagrange multipliers which will not necessarily be consistent with the
    // multipliers provided by a solver. Therefore, we'll create a copy of the
    // original model, disable its constraints, and apply the constraint
    // forces equivalent to the solver's Lagrange multipliers before computing
    // the accelerations.
    // If there's a PrescribedMotion in the model, it's disabled by default
    // in this copied model.
    m_model_disabled_constraints = Model(m_model_base);

    // The constraint forces will be applied to the copied model via an
    // OpenSim::DiscreteForces component, a thin wrapper to Simbody's
    // DiscreteForces class, which adds discrete variables to the state.
    auto constraintForcesUPtr = make_unique<DiscreteForces>();
    constraintForcesUPtr->setName("constraint_forces");
    m_constraint_forces.reset(constraintForcesUPtr.get());
    m_model_disabled_constraints.addComponent(constraintForcesUPtr.release());

    m_model_disabled_constraints.finalizeFromProperties();
    m_discrete_controller_disabled_constraints.reset(
            &*m_model_disabled_constraints.getComponentList<DiscreteController>().begin());


    if (!m_prescribedKinematics) {
        // The Acceleration motion is always added if there is no
        // PositionMotion, but is only enabled by solvers if using an implicit
        // dynamics mode. We use this motion to ensure that joint reaction
        // forces can be computed correctly from the solver-supplied UDot
        // (otherwise, Simbody will compute its own "incorrect" UDot using
        // forward dynamics).
        auto accelMotionUPtr = make_unique<AccelerationMotion>("motion");
        m_acceleration_motion.reset(accelMotionUPtr.get());
        m_model_disabled_constraints.addModelComponent(
                accelMotionUPtr.release());
    }

    // Grab a writable state from the copied model -- we'll use this to disable
    // its constraints below.
    m_state_disabled_constraints[0] = m_model_disabled_constraints.initSystem();
    m_state_disabled_constraints[1] = m_state_disabled_constraints[0];

    // See comment above for m_position_motion_base.
    if (m_prescribedKinematics) {
        m_position_motion_disabled_constraints.reset(
                &*m_model_disabled_constraints
                          .getComponentList<PositionMotion>()
                          .begin());
        for (auto& stateDisCon : m_state_disabled_constraints) {
            m_position_motion_disabled_constraints->setEnabled(
                    stateDisCon, true);
        }
    }

    // Get property values for constraints and Lagrange multipliers.
    const auto& kcBounds = ph0.get_kinematic_constraint_bounds();
    const MocoBounds& multBounds = ph0.get_multiplier_bounds();
    MocoInitialBounds multInitBounds(
            multBounds.getLower(), multBounds.getUpper());
    MocoFinalBounds multFinalBounds(
            multBounds.getLower(), multBounds.getUpper());
    // Get model information to loop through constraints.
    const auto& matter = m_model_base.getMatterSubsystem();
    auto& matterDisabledConstraints =
            m_model_disabled_constraints.updMatterSubsystem();
    const auto NC = matter.getNumConstraints();
    const auto& state = m_model_base.getWorkingState();
    int mp, mv, ma;
    m_num_kinematic_constraint_equations = 0;
    std::vector<std::string> kc_perr_names;
    std::vector<std::string> kc_verr_names;
    std::vector<std::string> kc_aerr_names;
    for (SimTK::ConstraintIndex cid(0); cid < NC; ++cid) {
        const SimTK::Constraint& constraint = matter.getConstraint(cid);
        SimTK::Constraint& constraintToDisable =
                matterDisabledConstraints.updConstraint(cid);
        if (!constraint.isDisabled(state)) {
            constraint.getNumConstraintEquationsInUse(state, mp, mv, ma);
            MocoKinematicConstraint kc(cid, mp, mv, ma);

            // Set the bounds for this kinematic constraint based on the
            // property.
            MocoConstraintInfo kcInfo = kc.getConstraintInfo();
            std::vector<MocoBounds> kcBoundVec(
                    kc.getConstraintInfo().getNumEquations(), kcBounds);
            kcInfo.setBounds(kcBoundVec);
            kc.setConstraintInfo(kcInfo);

            // Update number of scalar kinematic constraint equations.
            m_num_kinematic_constraint_equations +=
                    kc.getConstraintInfo().getNumEquations();

            // Append this kinematic constraint to the internal vector variable.
            // TODO: Avoid copies when the vector needs to be resized.
            m_kinematic_constraints.push_back(kc);

            // Add variable infos for all Lagrange multipliers in the problem.
            // Multipliers are only added based on the number of holonomic,
            // nonholonomic, or acceleration kinematic constraints and are *not*
            // based on the number for derivatives of holonomic or nonholonomic
            // constraint equations.
            // TODO how to name multiplier variables?
            std::vector<MocoVariableInfo> multInfos;
            for (int i = 0; i < mp; ++i) {
                std::string name = fmt::format("cid{}_p{}", cid, i);
                kc_perr_names.push_back(name);
                MocoVariableInfo info("lambda_" + name, multBounds,
                        multInitBounds, multFinalBounds);
                multInfos.push_back(info);
            }
            for (int i = 0; i < mv; ++i) {
                std::string name = fmt::format("cid{}_v{}", cid, i);
                MocoVariableInfo info("lambda_" + name, multBounds,
                        multInitBounds, multFinalBounds);
                kc_verr_names.push_back(name);
                multInfos.push_back(info);
            }
            for (int i = 0; i < ma; ++i) {
                std::string name = fmt::format("cid{}_a{}", cid, i);
                MocoVariableInfo info("lambda_" + name, multBounds,
                        multInitBounds, multFinalBounds);
                kc_aerr_names.push_back(name);
                multInfos.push_back(info);
            }
            m_multiplier_infos_map.insert({kcInfo.getName(), multInfos});

            // Disable this constraint in the copied model.
            for (auto& stateDisCon : m_state_disabled_constraints) {
                constraintToDisable.disable(stateDisCon);
            }
        }
    }

    // Create kinematic constraint equation names.
    // Solvers use "..._without_derivatives" when not enforcing constraint
    // derivatives, and use "..._with_derivatives" otherwise.
    if (!m_prescribedKinematics) {
        for (const auto& name : kc_perr_names) {
            m_kinematic_constraint_eq_names_without_derivatives.push_back(name);
            m_kinematic_constraint_eq_names_with_derivatives.push_back(name);
        }
        for (const auto& name : kc_perr_names) {
            m_kinematic_constraint_eq_names_with_derivatives.push_back(
                    name + "d");
        }
        for (const auto& name : kc_verr_names) {
            m_kinematic_constraint_eq_names_without_derivatives.push_back(name);
            m_kinematic_constraint_eq_names_with_derivatives.push_back(name);
        }
        for (const auto& name : kc_perr_names) {
            m_kinematic_constraint_eq_names_with_derivatives.push_back(
                    name + "dd");
        }
        for (const auto& name : kc_verr_names) {
            m_kinematic_constraint_eq_names_with_derivatives.push_back(
                    name + "d");
        }
        for (const auto& name : kc_aerr_names) {
            m_kinematic_constraint_eq_names_without_derivatives.push_back(name);
            m_kinematic_constraint_eq_names_with_derivatives.push_back(name);
        }
    }

    // Verify that the constraint error vectors in the state associated with the
    // copied model are empty.
    for (const auto& stateDisCon : m_state_disabled_constraints) {
        m_model_disabled_constraints.getSystem().realize(
                stateDisCon, SimTK::Stage::Instance);
        OPENSIM_THROW_IF(stateDisCon.getNQErr() != 0 ||
                                 stateDisCon.getNUErr() != 0 ||
                                 stateDisCon.getNUDotErr() != 0,
                Exception, "Internal error.");
    }

    // State infos.
    // ------------
    // Set the regex pattern states first.
    const auto stateNames = m_model_base.getStateVariableNames();
    for (int i = 0; i < ph0.getProperty_state_infos_pattern().size(); ++i) {
        const auto& pattern = ph0.get_state_infos_pattern(i).getName();
        auto regexPattern = std::regex(pattern);
        for (int j = 0; j < stateNames.size(); ++j) {
            if (std::regex_match(stateNames[j], regexPattern)) {
                m_state_infos[stateNames[j]] = ph0.get_state_infos_pattern(i);
                m_state_infos[stateNames[j]].setName(stateNames[j]);
            }
        }
    }
    for (int i = 0; i < ph0.getProperty_state_infos().size(); ++i) {
        const auto& name = ph0.get_state_infos(i).getName();
        OPENSIM_THROW_IF(stateNames.findIndex(name) == -1, Exception,
                "State info provided for nonexistent state '{}'.", name);
    }

    // Create internal record of state infos, automatically populated from
    // coordinates and actuators. This could override state infos set using a
    // regex pattern above.
    for (int i = 0; i < ph0.getProperty_state_infos().size(); ++i) {
        const auto& name = ph0.get_state_infos(i).getName();
        m_state_infos[name] = ph0.get_state_infos(i);
    }

    // Components can provide default state bounds via an output starting with
    // "statebounds_".
    for (const auto& component : m_model_base.getComponentList()) {
        const auto outputsBound = getModelOutputReferencePtrs<SimTK::Vec2>(
                component, "^statebounds_.*");
        for (const auto& output : outputsBound) {
            const auto nameStart = output->getName().find("_") + 1;
            const auto stateName = output->getName().substr(nameStart);
            const auto statePath = fmt::format(
                    "{}/{}", component.getAbsolutePathString(), stateName);
            // If this is indeed a state and no info has been provided for it,
            // use the state bounds from the output.
            if (stateNames.findIndex(statePath) != -1) {
                if (m_state_infos.count(statePath) == 0) {
                    const auto info = MocoVariableInfo(statePath, {}, {}, {});
                    m_state_infos[statePath] = info;
                }
                if (!m_state_infos[statePath].getBounds().isSet()) {
                    const auto& bounds =
                            output->getValue(m_state_base);
                    m_state_infos[statePath].setBounds({bounds[0], bounds[1]});
                }
            }
        }
    }

    if (!m_prescribedKinematics) {
        for (const auto& coord : m_model_base.getComponentList<Coordinate>()) {
            const auto stateVarNames = coord.getStateVariableNames();
            {
                const std::string coordValueName = stateVarNames[0];
                // TODO document: Range used even if not clamped.
                if (m_state_infos.count(coordValueName) == 0) {
                    const auto info =
                            MocoVariableInfo(coordValueName, {}, {}, {});
                    m_state_infos[coordValueName] = info;
                }
                if (!m_state_infos[coordValueName].getBounds().isSet()) {
                    m_state_infos[coordValueName].setBounds(
                            {coord.getRangeMin(), coord.getRangeMax()});
                }
            }
            {
                const std::string coordSpeedName = stateVarNames[1];
                if (m_state_infos.count(coordSpeedName) == 0) {
                    const auto info =
                            MocoVariableInfo(coordSpeedName, {}, {}, {});
                    m_state_infos[coordSpeedName] = info;
                }
                if (!m_state_infos[coordSpeedName].getBounds().isSet()) {
                    m_state_infos[coordSpeedName].setBounds(
                            ph0.get_default_speed_bounds());
                }
            }
        }
    }

    // Control infos.
    // --------------
    auto controlNames = createControlNamesFromModel(m_model_base);
    for (int i = 0; i < ph0.getProperty_control_infos_pattern().size(); ++i) {
        const auto& pattern = ph0.get_control_infos_pattern(i).getName();
        auto regexPattern = std::regex(pattern);
        for (int j = 0; j < (int)controlNames.size(); ++j) {
            if (std::regex_match(controlNames[j], regexPattern)) {
                m_control_infos[controlNames[j]] =
                        ph0.get_control_infos_pattern(i);
                m_control_infos[controlNames[j]].setName(controlNames[j]);
            }
        }
    }

    for (int i = 0; i < ph0.getProperty_control_infos().size(); ++i) {
        const auto& name = ph0.get_control_infos(i).getName();
        auto it = std::find(controlNames.begin(), controlNames.end(), name);
        OPENSIM_THROW_IF(it == controlNames.end(), Exception,
                "Control info provided for nonexistent or disabled actuator "
                "'{}'.",
                name);
    }

    for (int i = 0; i < ph0.getProperty_control_infos().size(); ++i) {
        const auto& name = ph0.get_control_infos(i).getName();
        m_control_infos[name] = ph0.get_control_infos(i);
    }

    // Loop through all the actuators in the model and create control infos
    // for the associated actuator control variables.
    for (const auto& actu : m_model_base.getComponentList<Actuator>()) {
        const std::string actuName = actu.getAbsolutePathString();
        if (actu.numControls() == 1) {
            // No control info exists; add one.
            if (m_control_infos.count(actuName) == 0) {
                const auto info = MocoVariableInfo(actuName, {}, {}, {});
                m_control_infos[actuName] = info;
            }
            if (!m_control_infos[actuName].getBounds().isSet()) {
                // If this scalar actuator derives from OpenSim::ScalarActuator,
                // use the getMinControl() and getMaxControl() methods to set
                // the bounds. Otherwise, set the bounds to (-inf, inf).
                if (const auto* scalarActu =
                                dynamic_cast<const ScalarActuator*>(&actu)) {
                    m_control_infos[actuName].setBounds(
                            {scalarActu->getMinControl(),
                                    scalarActu->getMaxControl()});
                } else {
                    m_control_infos[actuName].setBounds(
                            MocoBounds::unconstrained());
                }
            }

            if (ph0.get_bound_activation_from_excitation()) {
                const auto* muscle = dynamic_cast<const Muscle*>(&actu);
                if (muscle && !muscle->get_ignore_activation_dynamics()) {
                    const std::string stateName = actuName + "/activation";
                    auto& info = m_state_infos[stateName];
                    if (info.getName().empty()) { info.setName(stateName); }
                    if (!info.getBounds().isSet()) {
                        info.setBounds(m_control_infos[actuName].getBounds());
                    }
                }
            }

        } else {
            // This is a non-scalar actuator, so we need to add multiple
            // control infos.
            for (int idx = 0; idx < actu.numControls(); ++idx) {
                std::string controlName = actuName + "_" + std::to_string(idx);
                if (m_control_infos.count(controlName) == 0) {
                    const auto info = MocoVariableInfo(controlName, {}, {}, {});
                    m_control_infos[controlName] = info;
                }
                if (!m_control_infos[controlName].getBounds().isSet()) {
                    m_control_infos[controlName].setBounds(
                            MocoBounds::unconstrained());
                }
            }
        }
    }

    // Auxiliary state implicit residual outputs.
    const auto allImplicitResiduals = getModelOutputReferencePtrs<double>(
            m_model_disabled_constraints, "^implicitresidual_.*", true);
    for (const auto& output : allImplicitResiduals) {
        const auto& component = output->getOwner();
        const auto nameStart = output->getName().find("_") + 1;
        const std::string stateName = output->getName().substr(nameStart);
        bool enabled = component.getOutputValue<bool>(
                m_state_disabled_constraints[0],
                "implicitenabled_" + stateName);
        if (enabled) {
            m_implicit_residual_refs.emplace_back(output.get());
            m_implicit_component_refs.emplace_back(
                    "implicitderiv_" + stateName, &component);
        }
    }

    // Parameters.
    // -----------
    int numParametersFromPhase = (int)ph0.getProperty_parameters().size();
    m_parameters.resize(ph0.getProperty_parameters().size() + numScaleFactors);
    // Construct MocoParameters added to the MocoProblem via addParameter().
    std::unordered_set<std::string> paramNames;
    for (int i = 0; i < numParametersFromPhase; ++i) {
        const auto& param = ph0.get_parameters(i);
        OPENSIM_THROW_IF(param.getName().empty(), Exception,
                "All parameters must have a name.");
        OPENSIM_THROW_IF(paramNames.count(param.getName()), Exception,
                "A parameter with name '{}' already exists.", param.getName());
        paramNames.insert(param.getName());
        m_parameters[i] = std::unique_ptr<MocoParameter>(param.clone());
        // We must initialize on both models so that they are consistent
        // when parameters are updated when applyParameterToModel() is
        // called. Calling initializeOnModel() twice here is fine since the
        // models are identical aside from disabled Simbody constraints. The
        // property references to the parameters in both models are added to
        // the MocoParameter's internal vector of property references.
        m_parameters[i]->initializeOnModel(m_model_base);
        m_parameters[i]->initializeOnModel(m_model_disabled_constraints);
    }
    // Add MocoParameters based on MocoScaleFactors added to the model. We use
    // the name of the MocoScaleFactor for the MocoParameter, which is already
    // guaranteed to be unique based on the checks we made above.
    int iparam = numParametersFromPhase;
    const auto& scaleFactors =
            m_model_disabled_constraints.getComponentList<MocoScaleFactor>();
    for (const auto& scaleFactor : scaleFactors) {
        m_parameters[iparam] = std::unique_ptr<MocoParameter>(
                new MocoParameter(
                        scaleFactor.getName(),
                        scaleFactor.getAbsolutePathString(),
                        "scale_factor",
                        scaleFactor.getBounds()));
        m_parameters[iparam]->initializeOnModel(m_model_base);
        m_parameters[iparam]->initializeOnModel(
                m_model_disabled_constraints);
        ++iparam;
    }

    // Goals.
    // ------
    std::unordered_set<std::string> goalNames;
    for (int i = 0; i < ph0.getProperty_goals().size(); ++i) {
        const auto& goal = ph0.get_goals(i);
        OPENSIM_THROW_IF(goal.getName().empty(), Exception,
                "All goals must have a name.");
        OPENSIM_THROW_IF(goalNames.count(goal.getName()), Exception,
                "A goal with name '{}' already exists.", goal.getName());
        goalNames.insert(goal.getName());
        if (goal.getEnabled()) {
            std::unique_ptr<MocoGoal> item(goal.clone());
            item->initializeOnModel(m_model_disabled_constraints);
            if (item->getModeIsEndpointConstraint()) {
                m_endpoint_constraints.push_back(std::move(item));
            } else {
                m_costs.push_back(std::move(item));
            }
        }
    }

    MocoProblemInfo problemInfo;
    problemInfo.minInitialTime = getTimeInitialBounds().getLower();
    problemInfo.maxFinalTime = getTimeFinalBounds().getUpper();

    // Auxiliary path constraints.
    // ---------------------------
    m_num_path_constraint_equations = 0;
    m_path_constraints.resize(ph0.getProperty_path_constraints().size());
    std::unordered_set<std::string> pcNames;
    for (int i = 0; i < ph0.getProperty_path_constraints().size(); ++i) {
        const auto& pc = ph0.get_path_constraints(i);
        OPENSIM_THROW_IF(pc.getName().empty(), Exception,
                "All path constraints must have a name.");
        OPENSIM_THROW_IF(pcNames.count(pc.getName()), Exception,
                "A path constraint with name '{}' already exists.",
                pc.getName());
        pcNames.insert(pc.getName());
        m_path_constraints[i] = std::unique_ptr<MocoPathConstraint>(pc.clone());
        m_path_constraints[i]->initializeOnModel(m_model_disabled_constraints,
                problemInfo, m_num_path_constraint_equations);
        m_num_path_constraint_equations +=
                m_path_constraints[i]->getConstraintInfo().getNumEquations();
    }
}

const std::string& MocoProblemRep::getName() const {
    return m_problem->getName();
}
MocoInitialBounds MocoProblemRep::getTimeInitialBounds() const {
    return m_problem->getPhase(0).get_time_initial_bounds();
}
MocoFinalBounds MocoProblemRep::getTimeFinalBounds() const {
    return m_problem->getPhase(0).get_time_final_bounds();
}
std::vector<std::string> MocoProblemRep::createStateVariableNamesInSystemOrder(
        std::unordered_map<int, int>& yIndexMap) const {
    auto stateNames = OpenSim::createStateVariableNamesInSystemOrder(
            m_model_base, yIndexMap);
    auto out = stateNames;
    if (m_prescribedKinematics) {
        for (int i = 0; i < (int)stateNames.size(); ++i) {
            if (IO::EndsWith(stateNames[i], "/value") ||
                    IO::EndsWith(stateNames[i], "/speed")) {
                out.erase(std::find(out.begin(), out.end(), stateNames[i]));
            }
        }
    }
    return out;
}
std::vector<std::string> MocoProblemRep::createStateInfoNames() const {
    std::vector<std::string> names(m_state_infos.size());
    int i = 0;
    for (const auto& info : m_state_infos) {
        names[i] = info.first;
        ++i;
    }
    return names;
}
std::vector<std::string> MocoProblemRep::createControlInfoNames() const {
    std::vector<std::string> names(m_control_infos.size());
    int i = 0;
    for (const auto& info : m_control_infos) {
        names[i] = info.first;
        ++i;
    }
    return names;
}
std::vector<std::string> MocoProblemRep::createMultiplierInfoNames() const {
    std::vector<std::string> names;
    for (const auto& kc : m_kinematic_constraints) {
        const auto& infos =
                m_multiplier_infos_map.at(kc.getConstraintInfo().getName());
        for (const auto& info : infos) { names.push_back(info.getName()); }
    }
    return names;
}
std::vector<std::string>
MocoProblemRep::createKinematicConstraintNames() const {
    std::vector<std::string> names(m_kinematic_constraints.size());
    // Kinematic constraint names are stored in the internal constraint
    // info.
    for (int i = 0; i < (int)m_kinematic_constraints.size(); ++i) {
        names[i] = m_kinematic_constraints[i].getConstraintInfo().getName();
    }
    return names;
}
std::vector<std::string> MocoProblemRep::getKinematicConstraintEquationNames(
        bool includeDerivatives) const {
    if (includeDerivatives)
        return m_kinematic_constraint_eq_names_with_derivatives;
    return m_kinematic_constraint_eq_names_without_derivatives;
}
std::vector<std::string> MocoProblemRep::createParameterNames() const {
    std::vector<std::string> names(m_parameters.size());
    int i = 0;
    for (const auto& param : m_parameters) {
        names[i] = param->getName();
        ++i;
    }
    return names;
}
std::vector<std::string> MocoProblemRep::createCostNames() const {
    std::vector<std::string> names(m_costs.size());
    int i = 0;
    for (const auto& cost : m_costs) {
        names[i] = cost->getName();
        ++i;
    }
    return names;
}
std::vector<std::string> MocoProblemRep::createEndpointConstraintNames() const {
    std::vector<std::string> names(m_endpoint_constraints.size());
    int i = 0;
    for (const auto& endpoint_constraint : m_endpoint_constraints) {
        names[i] = endpoint_constraint->getName();
        ++i;
    }
    return names;
}
std::vector<std::string> MocoProblemRep::createPathConstraintNames() const {
    std::vector<std::string> names(m_path_constraints.size());
    int i = 0;
    for (const auto& pc : m_path_constraints) {
        names[i] = pc->getName();
        ++i;
    }
    return names;
}
const MocoVariableInfo& MocoProblemRep::getStateInfo(
        const std::string& name) const {
    OPENSIM_THROW_IF(m_state_infos.count(name) == 0, Exception,
            "No info available for state '{}'.", name);
    return m_state_infos.at(name);
}
const MocoVariableInfo& MocoProblemRep::getControlInfo(
        const std::string& name) const {
    OPENSIM_THROW_IF(m_control_infos.count(name) == 0, Exception,
            "No info available for control '{}'.", name);
    return m_control_infos.at(name);
}
const MocoParameter& MocoProblemRep::getParameter(
        const std::string& name) const {

    for (const auto& param : m_parameters) {
        if (param->getName() == name) { return *param.get(); }
    }
    OPENSIM_THROW(Exception, "No parameter with name '{}' found.", name);
}
const MocoGoal& MocoProblemRep::getCost(const std::string& name) const {

    for (const auto& c : m_costs) {
        if (c->getName() == name) { return *c.get(); }
    }
    OPENSIM_THROW(Exception, "No cost with name '{}' found.", name);
}
const MocoGoal& MocoProblemRep::getCostByIndex(int index) const {
    return *m_costs[index];
}
const MocoGoal& MocoProblemRep::getEndpointConstraint(
        const std::string& name) const {

    for (const auto& c : m_endpoint_constraints) {
        if (c->getName() == name) { return *c.get(); }
    }
    OPENSIM_THROW(
            Exception, "No endpoint constraint with name '{}' found.", name);
}
const MocoGoal& MocoProblemRep::getEndpointConstraintByIndex(int index) const {
    return *m_endpoint_constraints[index];
}
const MocoPathConstraint& MocoProblemRep::getPathConstraint(
        const std::string& name) const {

    for (const auto& pc : m_path_constraints) {
        if (pc->getName() == name) { return *pc.get(); }
    }
    OPENSIM_THROW(Exception, "No path constraint with name '{}' found.", name);
}
const MocoPathConstraint& MocoProblemRep::getPathConstraintByIndex(
        int index) const {
    return *m_path_constraints[index];
}
const MocoKinematicConstraint& MocoProblemRep::getKinematicConstraint(
        const std::string& name) const {

    // Kinematic constraint names are stored in the internal constraint
    // info.
    for (const auto& kc : m_kinematic_constraints) {
        if (kc.getConstraintInfo().getName() == name) { return kc; }
    }
    OPENSIM_THROW(
            Exception, "No kinematic constraint with name '{}' found.", name);
}
const std::vector<MocoVariableInfo>& MocoProblemRep::getMultiplierInfos(
        const std::string& kinematicConstraintInfoName) const {

    auto search = m_multiplier_infos_map.find(kinematicConstraintInfoName);
    if (search != m_multiplier_infos_map.end()) {
        return m_multiplier_infos_map.at(kinematicConstraintInfoName);
    } else {
        OPENSIM_THROW(Exception,
                "No variable infos for kinematic constraint info with name "
                "'{}' found.",
                kinematicConstraintInfoName);
    }
}

void MocoProblemRep::applyParametersToModelProperties(
        const SimTK::Vector& parameterValues,
        bool initSystemAndDisableConstraints) const {
    OPENSIM_THROW_IF(parameterValues.size() != (int)m_parameters.size(),
            Exception,
            "There are {} parameters in this MocoProblem, but {} values were "
            "provided.",
            m_parameters.size(), parameterValues.size());
    for (int i = 0; i < (int)m_parameters.size(); ++i) {
        m_parameters[i]->applyParameterToModelProperties(parameterValues(i));
    }
    if (initSystemAndDisableConstraints) {
        // TODO: Avoid these const_casts.

        // Model base.
        // -----------
        const_cast<Model&>(m_model_base).initSystem();
        // The PrescribedMotion is disabled by default in the model so that,
        // if there are constraints, the AssemblySolver does not complain about
        // having 0 parameters with which to satisfy the constraints. After
        // we're done with the assembly in initSystem(), we can re-enable the
        // prescribed motion.
        if (m_position_motion_base) {
            m_position_motion_base->setEnabled(m_state_base, true);
        }

        // Model disable constraints.
        // --------------------------
        Model& m_model_disabled_constraints_const_cast =
                const_cast<Model&>(m_model_disabled_constraints);

        m_state_disabled_constraints[0] =
                m_model_disabled_constraints_const_cast.initSystem();
        m_state_disabled_constraints[1] = m_state_disabled_constraints[0];
        // See comment above for m_position_motion_base.
        if (m_position_motion_disabled_constraints) {
            for (auto& stateDisCon : m_state_disabled_constraints) {
                m_position_motion_disabled_constraints->setEnabled(
                        stateDisCon, true);
            }
        }

        // Re-disable constraints if they were enabled by the previous
        // initSystem() call.
        auto& matterDisabledConstraints =
                m_model_disabled_constraints_const_cast.updMatterSubsystem();
        const auto NC = matterDisabledConstraints.getNumConstraints();
        for (SimTK::ConstraintIndex cid(0); cid < NC; ++cid) {
            SimTK::Constraint& constraintToDisable =
                    matterDisabledConstraints.updConstraint(cid);
            for (auto& stateDisCon : m_state_disabled_constraints) {
                if (!constraintToDisable.isDisabled(stateDisCon)) {
                    constraintToDisable.disable(stateDisCon);
                }
            }
        }
    }
}

void MocoProblemRep::printDescription() const {

    auto printHeaderLine = [&](const std::string& label, size_t size) {
        std::stringstream ss;
        ss << label << ": ";
        if (size == 0) {
            ss << "none";
        } else {
            ss << "(total: " << size << ")";
        }
        log_cout(ss.str());
    };

    printHeaderLine("Costs", m_costs.size());
    for (const auto& cost : m_costs) {
        cost->printDescription();
    }

    printHeaderLine("Endpoint constraints", m_endpoint_constraints.size());
    for (const auto& endpoint_constraint : m_endpoint_constraints) {
        endpoint_constraint->printDescription();
    }

    printHeaderLine("Kinematic constraints", m_kinematic_constraints.size());
    for (int i = 0; i < (int)m_kinematic_constraints.size(); ++i) {
        m_kinematic_constraints[i].getConstraintInfo().printDescription();
    }

    printHeaderLine("Path constraints", m_path_constraints.size());
    for (const auto& pc : m_path_constraints) {
        pc->printDescription();
    }

    printHeaderLine("States", m_state_infos.size());
    // TODO want to loop through the model's state variables and controls,
    // not just the infos.
    for (const auto& info : m_state_infos) {
        info.second.printDescription();
    }

    printHeaderLine("Controls", m_control_infos.size());
    for (const auto& info : m_control_infos) {
        info.second.printDescription();
    }

    printHeaderLine("Parameters", m_parameters.size());
    for (const auto& param : m_parameters) {
        param->printDescription();
    }
}
