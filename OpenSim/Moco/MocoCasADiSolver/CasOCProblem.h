#ifndef OPENSIM_CASOCPROBLEM_H
#define OPENSIM_CASOCPROBLEM_H
/* -------------------------------------------------------------------------- *
 * OpenSim: CasOCProblem.h                                                    *
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

#include <OpenSim/Moco/MocoUtilities.h>
#include "CasOCFunction.h"
#include <casadi/casadi.hpp>
#include <string>
#include <unordered_map>

namespace OpenSim {
class MocoCasADiSolver;
} // namespace OpenSim

/// CasOC is a namespace containing classes for solving multibody optimal
/// control problems with CasADi. CasOC is not designed to solve generic optimal
/// control problems. For example, CasOC does not require the user to provide a
/// system of first-order differential equations.
///
/// CasOC does not conceptually depend on OpenSim or Moco, though CasOC may use
/// OpenSim/Moco utilities (e.g., exception handling).
/// CasADi Optimal Control.
namespace CasOC {

struct Bounds {
    Bounds() = default;
    Bounds(double lower, double upper) : lower(lower), upper(upper) {}
    double lower = std::numeric_limits<double>::quiet_NaN();
    double upper = std::numeric_limits<double>::quiet_NaN();
    bool isSet() const { return !std::isnan(lower) && !std::isnan(upper); }
};

/// This enum is used to categorize a state variable as a generalized
/// coordinate, as a generalized speed, or as an auxiliary state variable (e.g.,
/// muscle activity).
enum class StateType { Coordinate, Speed, Auxiliary };
enum class KinematicLevel { Position, Velocity, Acceleration };
struct StateInfo {
    std::string name;
    StateType type;
    Bounds bounds;
    Bounds initialBounds;
    Bounds finalBounds;
};
struct ControlInfo {
    std::string name;
    Bounds bounds;
    Bounds initialBounds;
    Bounds finalBounds;
};
struct MultiplierInfo {
    std::string name;
    Bounds bounds;
    Bounds initialBounds;
    Bounds finalBounds;
    KinematicLevel level;
};
struct SlackInfo {
    std::string name;
    Bounds bounds;
};
struct ParameterInfo {
    std::string name;
    Bounds bounds;
};

struct EndpointInfo {
    EndpointInfo(std::string name, int num_outputs,
            std::unique_ptr<Integrand> ifunc, std::unique_ptr<Endpoint> efunc)
            : name(std::move(name)), num_outputs(num_outputs),
              integrand_function(std::move(ifunc)),
              endpoint_function(std::move(efunc)) {}
    std::string name;
    int num_outputs;
    std::unique_ptr<Integrand> integrand_function;
    std::unique_ptr<Endpoint> endpoint_function;
};

struct CostInfo : EndpointInfo {
    CostInfo(std::string name, int num_outputs,
            std::unique_ptr<Integrand> ifunc, std::unique_ptr<Endpoint> efunc)
            : EndpointInfo(std::move(name), num_outputs, std::move(ifunc),
                      std::move(efunc)) {}
};

struct EndpointConstraintInfo : EndpointInfo {
    EndpointConstraintInfo(std::string name, int num_outputs,
            std::unique_ptr<Integrand> ifunc, std::unique_ptr<Endpoint> efunc,
            casadi::DM lowerBounds, casadi::DM upperBounds)
            : EndpointInfo(std::move(name), num_outputs, std::move(ifunc),
                      std::move(efunc)),
              lowerBounds(std::move(lowerBounds)),
              upperBounds(std::move(upperBounds)) {}
    // The number of rows in these bounds must be num_outputs.
    casadi::DM lowerBounds;
    casadi::DM upperBounds;
};

/// The number outputs in the function must match the size of
/// lowerBounds and upperBounds.
struct PathConstraintInfo {
    std::string name;
    int size() const { return (int)lowerBounds.numel(); }
    casadi::DM lowerBounds;
    casadi::DM upperBounds;
    std::unique_ptr<PathConstraint> function;
};

class Solver;
class TrapezoidalSolver;

class Problem {
public:
    virtual ~Problem() = default;

    struct ContinuousInput {
        const double& time;
        const casadi::DM& states;
        const casadi::DM& controls;
        const casadi::DM& multipliers;
        const casadi::DM& derivatives;
        const casadi::DM& parameters;
    };
    struct CostInput {
        const double& initial_time;
        const casadi::DM& initial_states;
        const casadi::DM& initial_controls;
        const casadi::DM& initial_multipliers;
        const casadi::DM& initial_derivatives;
        const double& final_time;
        const casadi::DM& final_states;
        const casadi::DM& final_controls;
        const casadi::DM& final_multipliers;
        const casadi::DM& final_derivatives;
        const casadi::DM& parameters;
        const double& integral;
    };
    struct MultibodySystemExplicitOutput {
        casadi::DM& multibody_derivatives;
        casadi::DM& auxiliary_derivatives;
        casadi::DM& auxiliary_residuals;
        casadi::DM& kinematic_constraint_errors;
    };
    struct MultibodySystemImplicitOutput {
        casadi::DM& multibody_residuals;
        casadi::DM& auxiliary_derivatives;
        casadi::DM& auxiliary_residuals;
        casadi::DM& kinematic_constraint_errors;
    };

protected:
    /// @name Interface for the user building the problem.
    /// Call the add/set functions in the constructor for your problem.
    /// @{
    void setTimeBounds(Bounds initial, Bounds final) {
        m_timeInitialBounds = std::move(initial);
        m_timeFinalBounds = std::move(final);
    }
    /// Add a differential state. The MultibodySystem function must provide
    /// differential equations for Speed and Auxiliary states. Currently, CasOC
    /// internally handles the differential equations for the generalized
    /// coordinates. The state variables must be added in the order Coordinate,
    /// Speed, Auxiliary.
    // TODO: Create separate addDegreeOfFreedom() and addAuxiliaryState()?
    void addState(std::string name, StateType type, Bounds bounds,
            Bounds initialBounds, Bounds finalBounds) {
        clipEndpointBounds(bounds, initialBounds);
        clipEndpointBounds(bounds, finalBounds);
        m_stateInfos.push_back({std::move(name), type, std::move(bounds),
                std::move(initialBounds), std::move(finalBounds)});
        if (type == StateType::Coordinate)
            ++m_numCoordinates;
        else if (type == StateType::Speed)
            ++m_numSpeeds;
        else if (type == StateType::Auxiliary)
            ++m_numAuxiliaryStates;
    }
    /// Add an algebraic variable/"state" to the problem.
    void addControl(std::string name, Bounds bounds, Bounds initialBounds,
            Bounds finalBounds) {
        clipEndpointBounds(bounds, initialBounds);
        clipEndpointBounds(bounds, finalBounds);
        m_controlInfos.push_back({std::move(name), std::move(bounds),
                std::move(initialBounds), std::move(finalBounds)});
    }
    void addKinematicConstraint(std::string multName, Bounds multbounds,
            Bounds multInitialBounds, Bounds multFinalBounds,
            KinematicLevel kinLevel) {
        clipEndpointBounds(multbounds, multInitialBounds);
        clipEndpointBounds(multbounds, multFinalBounds);
        m_multiplierInfos.push_back({std::move(multName), std::move(multbounds),
                std::move(multInitialBounds), std::move(multFinalBounds),
                kinLevel});

        if (kinLevel == KinematicLevel::Position)
            ++m_numHolonomicConstraintEquations;
        else if (kinLevel == KinematicLevel::Velocity)
            ++m_numNonHolonomicConstraintEquations;
        else if (kinLevel == KinematicLevel::Acceleration)
            ++m_numAccelerationConstraintEquations;
    }
    /// Add a slack velocity correction variable to the problem associated with
    /// a kinematic constraint in the model.
    void addSlack(std::string name, Bounds bounds) {
        m_slackInfos.push_back({std::move(name), std::move(bounds)});
    }
    /// Set if all kinematics are prescribed. In this case, do not add state
    /// variables for coordinates or speeds. The number of multibody dynamics
    /// equations is equal to the number of speeds in the original system. But
    /// if kinematics are prescribed, you must provide the number of multibody
    /// dynamics equations directly. This is because no speed state variables
    /// are added and CasOCProblem can't obtain the number of multibody
    /// equations by counting the number of speed state variables.
    void setPrescribedKinematics(bool tf, int numMultibodyDynamicsEquations) {
        m_prescribedKinematics = tf;
        m_numMultibodyDynamicsEquationsIfPrescribedKinematics =
                numMultibodyDynamicsEquations;
    }
    /// Set whether not constraint derivatives are to be enforced.
    void setEnforceConstraintDerivatives(bool tf) {
        m_enforceConstraintDerivatives = tf;
    }
    /// Set the bounds for *all* kinematic constraints in the problem.
    void setKinematicConstraintBounds(Bounds bounds) {
        m_kinematicConstraintBounds = std::move(bounds);
    }
    /// Add a constant (time-invariant) variable to the optimization problem.
    void addParameter(std::string name, Bounds bounds) {
        m_paramInfos.push_back({std::move(name), std::move(bounds)});
    }
    /// Add a cost term to the problem.
    void addCost(std::string name, int numIntegrals, int numOutputs) {
        OPENSIM_THROW_IF(numIntegrals < 0 || numIntegrals > 1,
                OpenSim::Exception, "numIntegrals must be 0 or 1.");
        std::unique_ptr<CostIntegrand> integrand_function;
        if (numIntegrals) {
            integrand_function = OpenSim::make_unique<CostIntegrand>();
        }
        m_costInfos.emplace_back(std::move(name), numOutputs,
                std::move(integrand_function),
                OpenSim::make_unique<Cost>());
    }
    /// Add an endpoint constraint to the problem.
    void addEndpointConstraint(
            std::string name, int numIntegrals, std::vector<Bounds> bounds) {
        OPENSIM_THROW_IF(numIntegrals < 0 || numIntegrals > 1,
                OpenSim::Exception, "numIntegrals must be 0 or 1.");
        std::unique_ptr<EndpointConstraintIntegrand> integrand_function;
        if (numIntegrals) {
            integrand_function =
                    OpenSim::make_unique<EndpointConstraintIntegrand>();
        }
        casadi::DM lower(bounds.size(), 1);
        casadi::DM upper(bounds.size(), 1);
        for (int ibound = 0; ibound < (int)bounds.size(); ++ibound) {
            lower(ibound, 0) = bounds[ibound].lower;
            upper(ibound, 0) = bounds[ibound].upper;
        }
        m_endpointConstraintInfos.emplace_back(std::move(name),
                (int)bounds.size(), std::move(integrand_function),
                OpenSim::make_unique<EndpointConstraint>(), std::move(lower),
                std::move(upper));
    }
    /// The size of bounds must match the number of outputs in the function.
    void addPathConstraint(std::string name, std::vector<Bounds> bounds) {
        casadi::DM lower(bounds.size(), 1);
        casadi::DM upper(bounds.size(), 1);
        for (int ibound = 0; ibound < (int)bounds.size(); ++ibound) {
            lower(ibound, 0) = bounds[ibound].lower;
            upper(ibound, 0) = bounds[ibound].upper;
        }
        m_pathInfos.push_back({std::move(name), std::move(lower),
                std::move(upper), OpenSim::make_unique<PathConstraint>()});
    }
    void setDynamicsMode(std::string dynamicsMode) {
        OPENSIM_THROW_IF(
                dynamicsMode != "explicit" && dynamicsMode != "implicit",
                OpenSim::Exception, "Invalid dynamics mode.");
        m_dynamicsMode = std::move(dynamicsMode);
        m_isDynamicsModeImplicit = m_dynamicsMode == "implicit";
    }
    void setAuxiliaryDerivativeNames(const std::vector<std::string>& names) {
        m_auxiliaryDerivativeNames = names;
        m_numAuxiliaryResiduals = (int)names.size();
    }

public:
    /// Kinematic constraint errors should be ordered as so:
    /// - position-level constraints
    /// - first derivative of position-level constraints
    /// - velocity-level constraints
    /// - second derivative of position-level constraints
    /// - first derivative of velocity-level constraints
    /// - acceleration-level constraints
    virtual void calcMultibodySystemExplicit(const ContinuousInput& input,
            bool calcKCErrors, MultibodySystemExplicitOutput& output) const = 0;
    virtual void calcMultibodySystemImplicit(const ContinuousInput& input,
            bool calcKCErrors, MultibodySystemImplicitOutput& output) const = 0;
    virtual void calcVelocityCorrection(const double& time,
            const casadi::DM& multibody_states, const casadi::DM& slacks,
            const casadi::DM& parameters,
            casadi::DM& velocity_correction) const = 0;

    virtual void calcCostIntegrand(int /*costIndex*/,
            const ContinuousInput& /*input*/, double& /*integrand*/) const {}
    virtual void calcCost(int /*costIndex*/, const CostInput& /*input*/,
            casadi::DM& /*cost*/) const {}
    virtual void calcEndpointConstraintIntegrand(int /*index*/,
            const ContinuousInput& /*input*/, double& /*integrand*/) const {}
    virtual void calcEndpointConstraint(int /*index*/,
            const CostInput& /*input*/, casadi::DM& /*values*/) const {}
    virtual void calcPathConstraint(int /*constraintIndex*/,
            const ContinuousInput& /*input*/,
            casadi::DM& /*path_constraint*/) const {}

    virtual std::vector<std::string>
    createKinematicConstraintEquationNamesImpl() const;

    void intermediateCallback() const { intermediateCallbackImpl(); }
    void intermediateCallbackWithIterate(const CasOC::Iterate& it) const {
        intermediateCallbackWithIterateImpl(it);
    }
    /// This is invoked once for each iterate in the optimization process.
    virtual void intermediateCallbackImpl() const {}
    /// Process an intermediate iterate. The frequency with which this is
    /// evaluated is governed by Solver::getOutputInterval().
    virtual void intermediateCallbackWithIterateImpl(
            const CasOC::Iterate&) const {}
    /// @}

public:
    /// Create an iterate with the variable names populated according to the
    /// variables added to this problem.
    template <typename IterateType = Iterate>
    IterateType createIterate() const {
        IterateType it;
        for (const auto& info : m_stateInfos)
            it.state_names.push_back(info.name);
        for (const auto& info : m_controlInfos)
            it.control_names.push_back(info.name);
        for (const auto& info : m_multiplierInfos)
            it.multiplier_names.push_back(info.name);
        for (const auto& info : m_slackInfos)
            it.slack_names.push_back(info.name);
        if (isDynamicsModeImplicit()) {
            for (const auto& info : m_stateInfos) {
                if (info.type == StateType::Speed) {
                    auto name = info.name;
                    auto leafpos = name.find("speed");
                    OPENSIM_THROW_IF(leafpos == std::string::npos,
                            OpenSim::Exception, "Internal error.");
                    name.replace(leafpos, name.size(), "accel");
                    it.derivative_names.push_back(name);
                }
            }
        }
        for (const auto& auxDerivName : m_auxiliaryDerivativeNames) {
            it.derivative_names.push_back(auxDerivName);
        }
            
        for (const auto& info : m_paramInfos)
            it.parameter_names.push_back(info.name);
        return it;
    }

    void initialize(const std::string& finiteDiffScheme,
            std::shared_ptr<const std::vector<VariablesDM>>
                    pointsForSparsityDetection) const {
        auto* mutThis = const_cast<Problem*>(this);

        {
            int index = 0;
            for (const auto& costInfo : mutThis->m_costInfos) {
                costInfo.endpoint_function->constructFunction(this,
                        "cost_" + costInfo.name + "_endpoint", index,
                        costInfo.num_outputs, finiteDiffScheme,
                        pointsForSparsityDetection);
                if (costInfo.integrand_function) {
                    costInfo.integrand_function->constructFunction(this,
                            "cost_" + costInfo.name + "_integrand", index,
                            finiteDiffScheme, pointsForSparsityDetection);
                }
                ++index;
            }
        }
        {
            int index = 0;
            for (const auto& info : mutThis->m_endpointConstraintInfos) {
                info.endpoint_function->constructFunction(this,
                        "endpoint_constraint_" + info.name + "_endpoint", index,
                        info.num_outputs, finiteDiffScheme,
                        pointsForSparsityDetection);
                if (info.integrand_function) {
                    info.integrand_function->constructFunction(this,
                            "endpoint_constraint_" + info.name + "_integrand", index,
                            finiteDiffScheme, pointsForSparsityDetection);
                }
                ++index;
            }
        }
        {
            int index = 0;
            for (const auto& pathInfo : mutThis->m_pathInfos) {
                pathInfo.function->constructFunction(this,
                        "path_constraint_" + pathInfo.name, index,
                        (int)pathInfo.lowerBounds.size1(), finiteDiffScheme,
                        pointsForSparsityDetection);
                ++index;
            }
        }

        if (m_dynamicsMode == "implicit") {
            // Construct a full implicit multibody system (i.e. including
            // kinematic constraints).
            mutThis->m_implicitMultibodyFunc =
                    OpenSim::make_unique<MultibodySystemImplicit<true>>();
            mutThis->m_implicitMultibodyFunc->constructFunction(this,
                    "implicit_multibody_system", finiteDiffScheme,
                    pointsForSparsityDetection);

            // Construct an implicit multibody system ignoring kinematic
            // constraints.
            mutThis->m_implicitMultibodyFuncIgnoringConstraints =
                    OpenSim::make_unique<MultibodySystemImplicit<false>>();
            mutThis->m_implicitMultibodyFuncIgnoringConstraints
                    ->constructFunction(this,
                            "implicit_multibody_system_ignoring_constraints",
                            finiteDiffScheme, pointsForSparsityDetection);
        } else {
            mutThis->m_multibodyFunc =
                    OpenSim::make_unique<MultibodySystemExplicit<true>>();
            mutThis->m_multibodyFunc->constructFunction(this,
                    "explicit_multibody_system", finiteDiffScheme,
                    pointsForSparsityDetection);

            mutThis->m_multibodyFuncIgnoringConstraints =
                    OpenSim::make_unique<MultibodySystemExplicit<false>>();
            mutThis->m_multibodyFuncIgnoringConstraints->constructFunction(this,
                    "multibody_system_ignoring_constraints", finiteDiffScheme,
                    pointsForSparsityDetection);
        }

        if (m_enforceConstraintDerivatives) {
            mutThis->m_velocityCorrectionFunc =
                    OpenSim::make_unique<VelocityCorrection>();
            mutThis->m_velocityCorrectionFunc->constructFunction(this,
                    "velocity_correction", finiteDiffScheme,
                    pointsForSparsityDetection);
        }
    }

    /// @name Interface for CasOC::Transcription.
    /// @{
    int getNumStates() const { return (int)m_stateInfos.size(); }
    int getNumControls() const { return (int)m_controlInfos.size(); }
    int getNumParameters() const { return (int)m_paramInfos.size(); }
    int getNumMultipliers() const { return (int)m_multiplierInfos.size(); }
    std::string getDynamicsMode() const { return m_dynamicsMode; }
    bool isDynamicsModeImplicit() const { return m_isDynamicsModeImplicit; }
    int getNumDerivatives() const {
        return getNumAccelerations() + getNumAuxiliaryResidualEquations();
    }
    int getNumSlacks() const { return (int)m_slackInfos.size(); }
    /// This is the number of generalized coordinates, which may be greater
    /// than the number of generalized speeds.
    int getNumCoordinates() const { return m_numCoordinates; }
    int getNumSpeeds() const { return m_numSpeeds; }
    int getNumAccelerations() const {
        if (isDynamicsModeImplicit() && !isPrescribedKinematics()) {
            return getNumSpeeds();
        } else {
            return 0;
        }
    }
    int getNumAuxiliaryStates() const { return m_numAuxiliaryStates; }
    int getNumCosts() const { return (int)m_costInfos.size(); }
    bool isPrescribedKinematics() const { return m_prescribedKinematics; }
    /// If the coordinates are prescribed, then the number of multibody dynamics
    /// equations is not the same as the number of speeds.
    int getNumMultibodyDynamicsEquations() const {
        if (m_prescribedKinematics) {
            return m_numMultibodyDynamicsEquationsIfPrescribedKinematics;
        }
        return getNumSpeeds();
    }
    const std::vector<std::string>& getAuxiliaryDerivativeNames() const {
        return m_auxiliaryDerivativeNames;
    }
    int getNumAuxiliaryResidualEquations() const {
        return m_numAuxiliaryResiduals;
    }
    int getNumKinematicConstraintEquations() const {
        // If all kinematics are prescribed, we assume that the prescribed
        // kinematics obey any kinematic constraints. Therefore, the kinematic
        // constraints would be redundant, and we need not enforce them.
        if (m_prescribedKinematics) return 0;
        if (m_enforceConstraintDerivatives) {
            return 3 * m_numHolonomicConstraintEquations +
                   2 * m_numNonHolonomicConstraintEquations +
                   m_numAccelerationConstraintEquations;
        }
        return m_numHolonomicConstraintEquations +
               m_numNonHolonomicConstraintEquations +
               m_numAccelerationConstraintEquations;
    }
    /// Create a vector of names for scalar kinematic constraint equations.
    /// The length of the vector is getNumKinematicConstraintEquations().
    /// `includeDerivatives` determines if names for derivatives of
    /// position-level and velocity-level constraints should be included.
    std::vector<std::string> createKinematicConstraintEquationNames() const {
        std::vector<std::string> names =
                createKinematicConstraintEquationNamesImpl();
        OPENSIM_THROW_IF(
                (int)names.size() != getNumKinematicConstraintEquations(),
                OpenSim::Exception, "Internal error.");
        return names;
    }
    int getNumHolonomicConstraintEquations() const {
        return m_numHolonomicConstraintEquations;
    }
    int getNumNonHolonomicConstraintEquations() const {
        return m_numNonHolonomicConstraintEquations;
    }
    int getNumAccelerationConstraintEquations() const {
        return m_numAccelerationConstraintEquations;
    }
    bool getEnforceConstraintDerivatives() const {
        return m_enforceConstraintDerivatives;
    }
    const Bounds& getKinematicConstraintBounds() const {
        return m_kinematicConstraintBounds;
    }
    const Bounds& getTimeInitialBounds() const { return m_timeInitialBounds; }
    const Bounds& getTimeFinalBounds() const { return m_timeFinalBounds; }
    const std::vector<StateInfo>& getStateInfos() const { return m_stateInfos; }
    const std::vector<ControlInfo>& getControlInfos() const {
        return m_controlInfos;
    }
    const std::vector<MultiplierInfo>& getMultiplierInfos() const {
        return m_multiplierInfos;
    }
    const std::vector<SlackInfo>& getSlackInfos() const { return m_slackInfos; }
    const std::vector<ParameterInfo>& getParameterInfos() const {
        return m_paramInfos;
    }
    const std::vector<CostInfo>& getCostInfos() const { return m_costInfos; }
    const std::vector<EndpointConstraintInfo>&
    getEndpointConstraintInfos() const {
        return m_endpointConstraintInfos;
    }
    const std::vector<PathConstraintInfo>& getPathConstraintInfos() const {
        return m_pathInfos;
    }
    /// Get a function to the full multibody system (i.e. including kinematic
    /// constraints errors).
    const casadi::Function& getMultibodySystem() const {
        return *m_multibodyFunc;
    }
    /// Get a function to the multibody system that does *not* compute kinematic
    /// constraint errors (if they exist). This may be necessary for computing
    /// state derivatives at grid points where we do not want to enforce
    /// kinematic constraint errors.
    const casadi::Function& getMultibodySystemIgnoringConstraints() const {
        return *m_multibodyFuncIgnoringConstraints;
    }
    /// Get a function to compute the velocity correction to qdot when enforcing
    /// kinematic constraints and their derivatives. We require a separate
    /// function for this since we don't actually compute qdot within the
    /// multibody system.
    const casadi::Function& getVelocityCorrection() const {
        return *m_velocityCorrectionFunc;
    }
    const casadi::Function& getImplicitMultibodySystem() const {
        return *m_implicitMultibodyFunc;
    }
    const casadi::Function&
    getImplicitMultibodySystemIgnoringConstraints() const {
        return *m_implicitMultibodyFuncIgnoringConstraints;
    }
    /// @}

private:
    /// Clip endpoint to be as strict as b.
    void clipEndpointBounds(const Bounds& b, Bounds& endpoint) {
        endpoint.lower = std::max(b.lower, endpoint.lower);
        endpoint.upper = std::min(b.upper, endpoint.upper);
    }

    Bounds m_timeInitialBounds;
    Bounds m_timeFinalBounds;
    std::vector<StateInfo> m_stateInfos;
    int m_numCoordinates = 0;
    int m_numSpeeds = 0;
    int m_numAuxiliaryStates = 0;
    int m_numAuxiliaryResiduals = 0;
    int m_numHolonomicConstraintEquations = 0;
    int m_numNonHolonomicConstraintEquations = 0;
    int m_numAccelerationConstraintEquations = 0;
    bool m_enforceConstraintDerivatives = false;
    std::string m_dynamicsMode = "explicit";
    std::vector<std::string> m_auxiliaryDerivativeNames;
    bool m_isDynamicsModeImplicit = false;
    bool m_prescribedKinematics = false;
    int m_numMultibodyDynamicsEquationsIfPrescribedKinematics = 0;
    Bounds m_kinematicConstraintBounds;
    std::vector<ControlInfo> m_controlInfos;
    std::vector<MultiplierInfo> m_multiplierInfos;
    std::vector<SlackInfo> m_slackInfos;
    std::vector<ParameterInfo> m_paramInfos;
    std::vector<CostInfo> m_costInfos;
    std::vector<EndpointConstraintInfo> m_endpointConstraintInfos;
    std::vector<PathConstraintInfo> m_pathInfos;
    std::unique_ptr<MultibodySystemExplicit<true>> m_multibodyFunc;
    std::unique_ptr<MultibodySystemExplicit<false>>
            m_multibodyFuncIgnoringConstraints;
    std::unique_ptr<MultibodySystemImplicit<true>> m_implicitMultibodyFunc;
    std::unique_ptr<MultibodySystemImplicit<false>>
            m_implicitMultibodyFuncIgnoringConstraints;
    std::unique_ptr<VelocityCorrection> m_velocityCorrectionFunc;
};

} // namespace CasOC

#endif // OPENSIM_CASOCPROBLEM_H
