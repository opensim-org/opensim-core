#ifndef MOCO_CASOCPROBLEM_H
#define MOCO_CASOCPROBLEM_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: CasOCProblem.h                                               *
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

#include "../MocoUtilities.h"
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
};
struct SlackInfo {
    std::string name;
    Bounds bounds;
};
struct ParameterInfo {
    std::string name;
    Bounds bounds;
};

/// The number outputs in the function must match the size of
/// lowerBounds and upperBounds.
struct PathConstraintInfo {
    std::string name;
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
    struct EndpointInput {
        const double& final_time;
        const casadi::DM& final_states;
        const casadi::DM& final_controls;
        const casadi::DM& final_multipliers;
        const casadi::DM& final_derivatives;
        const casadi::DM& parameters;
    };
    struct MultibodySystemExplicitOutput {
        casadi::DM& multibody_derivatives;
        casadi::DM& auxiliary_derivatives;
        casadi::DM& kinematic_constraint_errors;
    };
    struct MultibodySystemImplicitOutput {
        casadi::DM& multibody_residuals;
        casadi::DM& auxiliary_derivatives;
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
                std::move(multInitialBounds), std::move(multFinalBounds)});

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
    /// The size of bounds must match the number of outputs in the function.
    /// Use variadic template arguments to pass arguments to the constructor of
    /// FunctionType.
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
    }

public:
    virtual void calcIntegralCostIntegrand(
            const ContinuousInput&, double& integrand) const {
        integrand = 0;
    }
    virtual void calcEndpointCost(
            const EndpointInput&, double& cost) const {
        cost = 0;
    }
    virtual void calcMultibodySystemExplicit(const ContinuousInput& input,
            bool calcKCErrors, MultibodySystemExplicitOutput& output) const = 0;
    virtual void calcMultibodySystemImplicit(const ContinuousInput& input,
            bool calcKCErrors, MultibodySystemImplicitOutput& output) const = 0;
    virtual void calcVelocityCorrection(const double& time,
            const casadi::DM& multibody_states, const casadi::DM& slacks,
            const casadi::DM& parameters,
            casadi::DM& velocity_correction) const = 0;

    virtual void calcPathConstraint(int,
            const ContinuousInput&, casadi::DM&) const {}

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
        // We do not know whether this problem will be solved using implicit
        // or explicit dynamics mode, so we populate the derivative_names
        // always.
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
        for (const auto& info : m_paramInfos)
            it.parameter_names.push_back(info.name);
        return it;
    }

    void constructFunctions(const std::string& finiteDiffScheme,
            std::shared_ptr<const std::vector<VariablesDM>>
                    pointsForSparsityDetection) const {
        auto* mutThis = const_cast<Problem*>(this);

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
        mutThis->m_integralCostFunc =
                OpenSim::make_unique<IntegralCostIntegrand>();
        mutThis->m_integralCostFunc->constructFunction(this,
                "integral_cost_integrand", finiteDiffScheme,
                pointsForSparsityDetection);

        mutThis->m_endpointCostFunc = OpenSim::make_unique<EndpointCost>();
        mutThis->m_endpointCostFunc->constructFunction(this, "endpoint_cost",
                finiteDiffScheme, pointsForSparsityDetection);

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
    // TODO: Skip over empty slots for quaternions.
    int getNumStates() const { return (int)m_stateInfos.size(); }
    int getNumControls() const { return (int)m_controlInfos.size(); }
    int getNumParameters() const { return (int)m_paramInfos.size(); }
    int getNumMultipliers() const { return (int)m_multiplierInfos.size(); }
    std::string getDynamicsMode() const { return m_dynamicsMode; }
    int getNumDerivatives() const {
        if (m_dynamicsMode == "implicit") {
            return getNumSpeeds();
        } else {
            return 0;
        }
    }
    int getNumSlacks() const { return (int)m_slackInfos.size(); }
    /// This is the number of generalized coordinates, which may be greater
    /// than the number of generalized speeds.
    int getNumCoordinates() const { return m_numCoordinates; }
    int getNumSpeeds() const { return m_numSpeeds; }
    int getNumAuxiliaryStates() const { return m_numAuxiliaryStates; }
    /// If the coordinates are prescribed, then the number of multibody dynamics
    /// equations is not the same as the number of speeds.
    int getNumMultibodyDynamicsEquations() const {
        if (m_prescribedKinematics) {
            return m_numMultibodyDynamicsEquationsIfPrescribedKinematics;
        }
        return getNumSpeeds();
    }
    int getNumKinematicConstraintEquations() const {
        if (m_enforceConstraintDerivatives) {
            return 3 * m_numHolonomicConstraintEquations +
                   2 * m_numNonHolonomicConstraintEquations +
                   m_numAccelerationConstraintEquations;
        } else {
            return m_numHolonomicConstraintEquations +
                   m_numNonHolonomicConstraintEquations +
                   m_numAccelerationConstraintEquations;
        }
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
    const std::vector<PathConstraintInfo>& getPathConstraintInfos() const {
        return m_pathInfos;
    }
    const casadi::Function& getIntegralCostIntegrand() const {
        return *m_integralCostFunc;
    }
    const casadi::Function& getEndpointCost() const {
        return *m_endpointCostFunc;
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
    int m_numHolonomicConstraintEquations = 0;
    int m_numNonHolonomicConstraintEquations = 0;
    int m_numAccelerationConstraintEquations = 0;
    bool m_enforceConstraintDerivatives = false;
    std::string m_dynamicsMode = "explicit";
    bool m_prescribedKinematics = false;
    int m_numMultibodyDynamicsEquationsIfPrescribedKinematics = 0;
    Bounds m_kinematicConstraintBounds;
    std::vector<ControlInfo> m_controlInfos;
    std::vector<MultiplierInfo> m_multiplierInfos;
    std::vector<SlackInfo> m_slackInfos;
    std::vector<PathConstraintInfo> m_pathInfos;
    std::vector<ParameterInfo> m_paramInfos;
    std::unique_ptr<IntegralCostIntegrand> m_integralCostFunc;
    std::unique_ptr<EndpointCost> m_endpointCostFunc;
    std::unique_ptr<MultibodySystemExplicit<true>> m_multibodyFunc;
    std::unique_ptr<MultibodySystemExplicit<false>>
            m_multibodyFuncIgnoringConstraints;
    std::unique_ptr<MultibodySystemImplicit<true>> m_implicitMultibodyFunc;
    std::unique_ptr<MultibodySystemImplicit<false>>
            m_implicitMultibodyFuncIgnoringConstraints;
    std::unique_ptr<VelocityCorrection> m_velocityCorrectionFunc;
};

} // namespace CasOC

#endif // MOCO_CASOCPROBLEM_H
