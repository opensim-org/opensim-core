#ifndef MOCO_CASOCPROBLEM_H
#define MOCO_CASOCPROBLEM_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoCasOCProblem.h                                           *
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

/// This enum describes the different types of optimization variables, and
/// are the keys for the Variables map.
enum Var {
    initial_time,
    final_time,
    /// Differential variables.
    states,
    /// Algebraic variables.
    controls,
    /// Used for kinematic constraints.
    multipliers,
    /// Used for certain methods of solving kinematic constraints.
    slacks,
    /// Used in implicit dynamics mode.
    derivatives, // TODO: Rename to accelerations?
    /// Constant in time.
    parameters
};

template <typename T>
using Variables = std::unordered_map<Var, T, std::hash<int>>;

/// Numeric variables for initial guesses and solutions.
using VariablesDM = Variables<casadi::DM>;
/// Symbolic variables, used to define the problem.
using VariablesMX = Variables<casadi::MX>;

/// This struct is used to obtain initial guesses.
struct Iterate {
    VariablesDM variables;
    casadi::DM times;
    std::vector<std::string> state_names;
    std::vector<std::string> control_names;
    std::vector<std::string> multiplier_names;
    std::vector<std::string> slack_names;
    std::vector<std::string> derivative_names;
    std::vector<std::string> parameter_names;
    /// Return a new iterate in which the data is resampled at the times in
    /// newTimes.
    Iterate resample(const casadi::DM& newTimes) const;
};

/// This struct is used to return a solution to a problem. Use `stats`
/// to check if the problem converged.
struct Solution : public Iterate {
    casadi::Dict stats;
};

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
    /// @name Interface for the user building the problem.
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
    /// Add a Lagrange multiplier variable to the problem associated with a 
    /// kinematic constraint in the model.
    void addMultiplier(std::string name, Bounds bounds, Bounds initialBounds,
            Bounds finalBounds) {
        clipEndpointBounds(bounds, initialBounds);
        clipEndpointBounds(bounds, finalBounds);
        m_multiplierInfos.push_back({std::move(name), std::move(bounds),
            std::move(initialBounds), std::move(finalBounds)});
    }
    /// Add a slack velocity correction variable to the problem associated with
    /// a kinematic constraint in the model.
    void addSlack(std::string name, Bounds bounds) {
        m_slackInfos.push_back({std::move(name), std::move(bounds)});
    }
    /// Set the total number of kinematic constraint equations (including 
    /// derivatives of holonomic and non-holonomic constraint equations) in the
    /// multibody system.
    void setNumKinematicConstraintEquations(int numEqs) {
        m_numKinematicConstraintEquations = numEqs;
    }
    /// Set the number of holonomic constraint equations in the multibody 
    /// system.
    /// @note This does *not* include holonomic constraint equation derivatives.
    void setNumHolonomicConstraintEquations(int numHolo) {
        m_numHolonomicConstraintEquations = numHolo;
    }
    /// Set the number of non-holonomic constraint equations in the multibody
    /// system.
    /// @note This does *not* include non-holonomic constraint equation 
    /// derivatives.
    void setNumNonHolonomicConstraintEquations(int numNonHolo) {
        m_numNonHolonomicConstraintEquations = numNonHolo;
    }
    /// Set the number of acceleration constraint equations in the multibody
    /// system.
    void setNumAccelerationConstraintEquations(int numAccel) {
        m_numAccelerationConstraintEquations = numAccel;
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
    /// FunctionType must derive from PathConstraints.
    /// The size of bounds must match the number of outputs in the function.
    /// Use variadic template arguments to pass arguments to the constructor of
    /// FunctionType.
    template <typename FunctionType, typename... Args>
    void addPathConstraint(
            std::string name, std::vector<Bounds> bounds, Args&&... args) {
        casadi::DM lower(bounds.size(), 1);
        casadi::DM upper(bounds.size(), 1);
        for (int ibound = 0; ibound < (int)bounds.size(); ++ibound) {
            lower(ibound, 0) = bounds[ibound].lower;
            upper(ibound, 0) = bounds[ibound].upper;
        }
        m_pathInfos.push_back(
                {std::move(name), std::move(lower), std::move(upper),
                        OpenSim::make_unique<FunctionType>(
                                std::forward<Args>(args)...)});
        m_pathInfos.back().function->constructFunction(
                this, "path_constraint_" + name, (int)bounds.size());
    }
    /// FunctionType must derive from IntegralCostIntegrand.
    /// Set a functon that computes the integrand of the integral cost.
    template <typename FunctionType, typename... Args>
    void setIntegralCost(Args&&... args) {
        m_integralCostFunc =
                OpenSim::make_unique<FunctionType>(std::forward<Args>(args)...);
        m_integralCostFunc->constructFunction(this, "integral_cost_integrand");
    }
    /// FunctionType must derive from EndpointCost.
    template <typename FunctionType, typename... Args>
    void setEndpointCost(Args&&... args) {
        m_endpointCostFunc =
                OpenSim::make_unique<FunctionType>(std::forward<Args>(args)...);
        m_endpointCostFunc->constructFunction(this, "endpoint_cost");
    }
    /// FunctionType must derive from MultibodySystem.
    template <template <bool> class FunctionType, typename... Args>
    void setMultibodySystem(Args&&... args) {
        // Construct an unconstrained multibody system.
        m_unconstrainedMultibodyFunc = 
                OpenSim::make_unique<FunctionType<false>>(
                    std::forward<Args>(args)...);
        m_unconstrainedMultibodyFunc->constructFunction(this,
                "unconstrainted_multibody_system");
        // Constraint a full multibody system (i.e. including kinematic
        // constraints).
        m_multibodyFunc = OpenSim::make_unique<FunctionType<true>>(
                    std::forward<Args>(args)...);
        m_multibodyFunc->constructFunction(this, "multibody_system");
    }
    /// FunctionType must derive from VelocityCorrection.
    template <typename FunctionType, typename... Args>
    void setVelocityCorrection(Args&&... args) {
        m_velocityCorrectionFunc =
                OpenSim::make_unique<FunctionType>(std::forward<Args>(args)...);
        m_velocityCorrectionFunc->constructFunction(this, 
                "velocity_correction");
    }
    template <typename FunctionType, typename... Args>
    void setImplicitMultibodySystem(Args&&... args) {
        m_implicitMultibodyFunc =
                OpenSim::make_unique<FunctionType>(std::forward<Args>(args)...);
        m_implicitMultibodyFunc->constructFunction(this,
                "implicit_multibody_system");
    }

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

    /// @}

    /// @name Interface for CasOC::Transcription.
    /// @{
    // TODO: Skip over empty slots for quaternions.
    int getNumStates() const { return (int)m_stateInfos.size(); }
    int getNumControls() const { return (int)m_controlInfos.size(); }
    int getNumParameters() const { return (int)m_paramInfos.size(); }
    int getNumMultipliers() const { return (int)m_multiplierInfos.size(); }
    int getNumSlacks() const { return (int)m_slackInfos.size(); }
    /// This is the number of generalized coordinates, which may be greater
    /// than the number of generalized speeds.
    int getNumCoordinates() const { return m_numCoordinates; }
    int getNumSpeeds() const { return m_numSpeeds; }
    int getNumAuxiliaryStates() const { return m_numAuxiliaryStates; }
    int getNumKinematicConstraintEquations() const { 
        return m_numKinematicConstraintEquations; 
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
    const std::vector<SlackInfo>& getSlackInfos() const {
        return m_slackInfos;
    }
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
    /// Get a function the full multibody system (i.e. including kinematic 
    /// constraints errors).
    const casadi::Function& getMultibodySystem() const {
        return *m_multibodyFunc;
    }
    /// Get a function to the multibody system that does *not* compute kinematic
    /// constraint errors (if they exist). This may be necessary for computing
    /// state derivatives at grid points where we do not want to enforce 
    /// kinematic constraint errors.
    const casadi::Function& getUnconstrainedMultibodySystem() const {
        return *m_unconstrainedMultibodyFunc;
    }
    const casadi::Function& getVelocityCorrection() const {
        return *m_velocityCorrectionFunc;
    }
    const casadi::Function& getImplicitMultibodySystem() const {
        return *m_implicitMultibodyFunc;
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
    int m_numKinematicConstraintEquations = 0;
    int m_numHolonomicConstraintEquations = 0;
    int m_numNonHolonomicConstraintEquations = 0;
    int m_numAccelerationConstraintEquations = 0;
    bool m_enforceConstraintDerivatives = false;
    Bounds m_kinematicConstraintBounds;
    std::vector<ControlInfo> m_controlInfos;
    std::vector<MultiplierInfo> m_multiplierInfos;
    std::vector<SlackInfo> m_slackInfos;
    std::vector<PathConstraintInfo> m_pathInfos;
    std::vector<ParameterInfo> m_paramInfos;
    std::unique_ptr<IntegralCostIntegrand> m_integralCostFunc;
    std::unique_ptr<EndpointCost> m_endpointCostFunc;
    std::unique_ptr<MultibodySystem<true>> m_multibodyFunc;
    std::unique_ptr<MultibodySystem<false>> m_unconstrainedMultibodyFunc;
    std::unique_ptr<MultibodySystemImplicit> m_implicitMultibodyFunc;
    std::unique_ptr<VelocityCorrection> m_velocityCorrectionFunc;
};

} // namespace CasOC

#endif // MOCO_CASOCPROBLEM_H
