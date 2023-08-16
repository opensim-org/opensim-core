/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoCasOCProblem.cpp                                         *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2018 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
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

#include "MocoCasOCProblem.h"

#include "MocoCasADiSolver.h"

#include <OpenSim/Simulation/SimulationUtilities.h>

using namespace OpenSim;

thread_local SimTK::Vector_<SimTK::SpatialVec>
        MocoCasOCProblem::m_constraintBodyForces;
thread_local SimTK::Vector MocoCasOCProblem::m_constraintMobilityForces;
thread_local SimTK::Vector MocoCasOCProblem::m_pvaerr;

MocoCasOCProblem::MocoCasOCProblem(const MocoCasADiSolver& mocoCasADiSolver,
        const MocoProblemRep& problemRep,
        std::unique_ptr<ThreadsafeJar<const MocoProblemRep>> jar,
        std::string dynamicsMode, std::string kinematicConstraintMethod)
        : m_jar(std::move(jar)),
          m_paramsRequireInitSystem(
                  mocoCasADiSolver.get_parameters_require_initsystem()),
          m_formattedTimeString(getFormattedDateTime(true)) {

    setDynamicsMode(dynamicsMode);
    setKinematicConstraintMethod(kinematicConstraintMethod);
    const auto& model = problemRep.getModelBase();

    // Ensure the model does not have user-provided controllers.
    int numControllers = 0;
    for (const auto& controller : model.getComponentList<Controller>()) {
        // Avoid unused variable warning.
        (void)&controller;
        ++numControllers;
    }
    // The model has a DiscreteController added by MocoProblemRep; any other
    // controllers were added by the user.
    OPENSIM_THROW_IF(numControllers > 1, Exception,
            "MocoCasADiSolver does not support models with Controllers.");

    if (problemRep.isPrescribedKinematics()) {
        setPrescribedKinematics(true, model.getWorkingState().getNU());
    }

    auto stateNames =
            problemRep.createStateVariableNamesInSystemOrder(m_yIndexMap);
    setTimeBounds(convertBounds(problemRep.getTimeInitialBounds()),
            convertBounds(problemRep.getTimeFinalBounds()));
    for (const auto& stateName : stateNames) {
        const auto& info = problemRep.getStateInfo(stateName);
        CasOC::StateType stateType;
        if (IO::EndsWith(stateName, "/value"))
            stateType = CasOC::StateType::Coordinate;
        else if (IO::EndsWith(stateName, "/speed"))
            stateType = CasOC::StateType::Speed;
        else
            stateType = CasOC::StateType::Auxiliary;
        addState(stateName, stateType, convertBounds(info.getBounds()),
                convertBounds(info.getInitialBounds()),
                convertBounds(info.getFinalBounds()));
    }

    auto controlNames =
            createControlNamesFromModel(model, m_modelControlIndices);
    for (const auto& controlName : controlNames) {
        const auto& info = problemRep.getControlInfo(controlName);
        addControl(controlName, convertBounds(info.getBounds()),
                convertBounds(info.getInitialBounds()),
                convertBounds(info.getFinalBounds()));
    }

    // Set the number of residual equations to be enforced for components with
    // dynamics in implicit form.
    const auto& implicitRefs = problemRep.getImplicitComponentReferencePtrs();
    std::vector<std::string> derivativeNames;
    for (const auto& implicitRef : implicitRefs) {
        derivativeNames.push_back(implicitRef.second->getAbsolutePathString() +
                                  "/" + implicitRef.first);
    }

    setAuxiliaryDerivativeNames(derivativeNames);

    // Add any scalar constraints associated with kinematic constraints in
    // the model as path constraints in the problem.
    // Whether or not enabled kinematic constraints exist in the model,
    // check that optional solver properties related to constraints are
    // set properly.
    const auto kcNames = problemRep.createKinematicConstraintNames();
    if (kcNames.empty()) {
        OPENSIM_THROW_IF(mocoCasADiSolver.get_minimize_lagrange_multipliers(),
                Exception,
                "Solver property 'minimize_lagrange_multipliers' "
                "was enabled but no enabled kinematic constraints exist in the "
                "model.");
    } else {

        int cid, mv, ma;
        int multIndexThisConstraint;
        std::vector<KinematicLevel> kinLevels;
        const bool enforceConstraintDerivs =
                mocoCasADiSolver.get_enforce_constraint_derivatives();
        for (const auto& kcName : kcNames) {
            const auto& kc = problemRep.getKinematicConstraint(kcName);
            const auto& multInfos = problemRep.getMultiplierInfos(kcName);
            cid = kc.getSimbodyConstraintIndex();
            mv = kc.getNumVelocityEquations();
            ma = kc.getNumAccelerationEquations();
            kinLevels = kc.getKinematicLevels();

            if (isKinematicConstraintMethodProjection()) {
                OPENSIM_THROW_IF(!enforceConstraintDerivs, Exception,
                         "The projection method for enforcing kinematic "
                         "constraints requires that the solver property "
                         "'enforce_constraint_derivatives' be set to true.");
            } else {
                OPENSIM_THROW_IF(mv != 0, Exception,
                         "The PKT method is not compatible with velocity-level "
                         "kinematic constraints. There are {} velocity-level "
                         "scalar constraints associated with the model "
                         "Constraint at ConstraintIndex {}.",
                         mv, cid);
                OPENSIM_THROW_IF(ma != 0, Exception,
                         "The PKT method is not compatible with acceleration-"
                         "level kinematic constraints. There are {} "
                         "acceleration-level scalar constraints associated with "
                         "the model Constraint at ConstraintIndex {}.",
                         ma, cid);
            }

            // Loop through all scalar constraints associated with the model
            // constraint and corresponding path constraints to the optimal
            // control problem.
            //
            // We need a different index for the Lagrange multipliers since
            // they are only added if the current constraint equation is not a
            // derivative of a position- or velocity-level equation.
            multIndexThisConstraint = 0;
            for (int i = 0; i < kc.getConstraintInfo().getNumEquations(); ++i) {

                // If the index for this path constraint represents
                // a non-derivative scalar constraint equation, add a
                // Lagrange multiplier to the problem.
                if (kinLevels[i] == KinematicLevel::Position ||
                        kinLevels[i] == KinematicLevel::Velocity ||
                        kinLevels[i] == KinematicLevel::Acceleration) {

                    // Grab the constraint information from the MocoProblem.
                    const auto& multInfo = multInfos[multIndexThisConstraint];
                    CasOC::KinematicLevel kinLevel;
                    if (kinLevels[i] == KinematicLevel::Position)
                        kinLevel = CasOC::KinematicLevel::Position;
                    else if (kinLevels[i] == KinematicLevel::Velocity)
                        kinLevel = CasOC::KinematicLevel::Velocity;
                    else if (kinLevels[i] == KinematicLevel::Acceleration)
                        kinLevel = CasOC::KinematicLevel::Acceleration;
                    else {
                        OPENSIM_THROW(OpenSim::Exception,
                                      "Unrecognized KinematicLevel");
                    }

                    // This adds a Lagrange multiplier to the current problem,
                    // and, if kinematics are not prescribed, increments the
                    // number of scalar kinematic constraint equations added.
                    addKinematicConstraint(multInfo.getName(),
                           convertBounds(multInfo.getBounds()),
                           convertBounds(multInfo.getInitialBounds()),
                           convertBounds(multInfo.getFinalBounds()), kinLevel);

                    // If kinematics are not prescribed and we are enforcing
                    // kinematic constraints explicitly, we need to add
                    // additional "slack" variables to ensure the problem is not
                    // over constrained.
                    if (!isPrescribedKinematics()) {
                        OPENSIM_THROW_IF(
                                multInfo.getName().substr(0, 6) != "lambda",
                                Exception,
                                "Expected the multiplier name for this "
                                "constraint to begin with 'lambda' but it "
                                "begins with '{}'.",
                                multInfo.getName().substr(0, 6));
                        if (isKinematicConstraintMethodProjection()) {
                            // Add "mu" variables for the projection method by
                            // Bordalba et al. (2023).
                            const auto muBounds = convertBounds(
                                    mocoCasADiSolver
                                            .get_projection_variable_bounds());
                            if (kinLevel == CasOC::KinematicLevel::Position) {
                                // For holonomic constraints, we need to add
                                // two slack variables: one for the position
                                // error and one for the derivative of the
                                // position error.
                                addSlack(std::string(multInfo.getName())
                                                 .replace(0, 6, "mu"),
                                         muBounds);
                                addSlack(std::string(multInfo.getName())
                                                 .replace(0, 6, "mu") + "_dot",
                                         muBounds);
                            } else if (kinLevel ==
                                       CasOC::KinematicLevel::Velocity) {
                                addSlack(std::string(multInfo.getName())
                                                 .replace(0, 6, "mu"),
                                         muBounds);
                            }
                        } else if (enforceConstraintDerivs) {
                            // Add "gamma" variables for the method by
                            // Posa et al. (2015).
                            const auto vcBounds = convertBounds(
                                    mocoCasADiSolver
                                            .get_velocity_correction_bounds());
                            addSlack(std::string(multInfo.getName())
                                             .replace(0, 6, "gamma"),
                                     vcBounds);
                        }
                    }
                    ++multIndexThisConstraint;
                }
            }
        }

        // Set kinematic constraint information on the CasOC::Problem.
        setEnforceConstraintDerivatives(enforceConstraintDerivs);
        // The bounds are the same for all kinematic constraints in the
        // MocoProblem, so just grab the bounds from the first constraint.
        // TODO: This behavior may be unexpected for users.
        const auto& kc = problemRep.getKinematicConstraint(kcNames.at(0));
        std::vector<MocoBounds> bounds = kc.getConstraintInfo().getBounds();
        setKinematicConstraintBounds(convertBounds(bounds.at(0)));
    }

    for (const auto& paramName : problemRep.createParameterNames()) {
        const auto& param = problemRep.getParameter(paramName);
        addParameter(paramName, convertBounds(param.getBounds()));
    }

    const auto costNames = problemRep.createCostNames();
    for (const auto& name : costNames) {
        const auto& cost = problemRep.getCost(name);
        addCost(name, cost.getNumIntegrals(), cost.getNumOutputs());
    }

    const auto endpointConNames =
            problemRep.createEndpointConstraintNames();
    for (const auto& name : endpointConNames) {
        const auto& ec = problemRep.getEndpointConstraint(name);
        std::vector<CasOC::Bounds> casBounds;
        for (const auto& bounds : ec.getConstraintInfo().getBounds()) {
            casBounds.push_back(convertBounds(bounds));
        }
        addEndpointConstraint(name, ec.getNumIntegrals(), casBounds);
    }

    const auto pathConstraintNames = problemRep.createPathConstraintNames();
    for (const auto& name : pathConstraintNames) {
        const auto& pathCon = problemRep.getPathConstraint(name);
        std::vector<CasOC::Bounds> casBounds;
        for (const auto& bounds : pathCon.getConstraintInfo().getBounds()) {
            casBounds.push_back(convertBounds(bounds));
        }
        addPathConstraint(name, casBounds);
    }

    m_fileDeletionThrower = OpenSim::make_unique<FileDeletionThrower>(
            fmt::format("delete_this_to_stop_optimization_{}_{}.txt",
                    problemRep.getName(), m_formattedTimeString));
}
