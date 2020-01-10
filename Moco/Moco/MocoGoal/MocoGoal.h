#ifndef MOCO_MOCOGOAL_H
#define MOCO_MOCOGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoGoal.h                                                   *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
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

#include "../MocoBounds.h"
#include "../MocoConstraintInfo.h"
#include "../osimMocoDLL.h"

#include <SimTKcommon/internal/State.h>

#include <OpenSim/Common/Object.h>

namespace OpenSim {

class Model;

// TODO give option to specify gradient and Hessian analytically.

/// A goal is term in the cost functional to be minimized, or a set of endpoint
/// constraints that must lie within provided bounds. Goals depend on the
/// phase's initial and final states and controls, and optionally on the
/// integral of a quantity over the phase.
/// Not all goals support endpoint constraint mode; see
/// getSupportsEndpointConstraint(). If a goal does support endpoint constraint
/// mode, then the default mode is available via getMode(). Use endpoint
/// constraint mode if you require the goal to be met strictly and do not want
/// to allow a trade-off between this goal and other goals.
/// The calculation of the goal may differ between cost and endpoint constraint
/// modes; cost mode may require that outputs are squared, for example.
///
/// @par For developers
/// Every time the problem is solved, a copy of this goal is used. An individual
/// instance of a goal is only ever used in a single problem. Therefore, there
/// is no need to clear cache variables that you create in initializeImpl().
/// Also, information stored in this goal does not persist across multiple
/// solves.
/// @ingroup mocogoal
class OSIMMOCO_API MocoGoal : public Object {
    OpenSim_DECLARE_ABSTRACT_OBJECT(MocoGoal, Object);

public:
    MocoGoal();

    MocoGoal(std::string name);

    MocoGoal(std::string name, double weight);

    /// %Set whether this goal is used in the problem.
    void setEnabled(bool enabled) { set_enabled(enabled); }
    bool getEnabled() const { return get_enabled(); }

    /// In cost mode, the goal is multiplied by this weight. Use the weight to
    /// control the relative importance of terms in the cost functional.
    void setWeight(double weight) { set_weight(weight); }
    double getWeight() const { return get_weight(); }

    enum class Mode { Cost, EndpointConstraint };
    /// Set the mode property to either 'cost' or 'endpoint_constraint'. This
    /// should be set before initializing. Setting to 'endpoint_constraint' if
    /// getSupportsEndpointConstraint() is false causes an exception during
    /// initializing.
    void setMode(std::string mode) { set_mode(mode); }
    /// This returns the default mode of the goal, unless the user overrode
    /// the default using setMode().
    std::string getModeAsString() const {
        return getMode() == Mode::Cost ? "cost" : "endpoint_constraint";
    }
    Mode getMode() const {
        OPENSIM_THROW_IF_FRMOBJ(
                !m_model, Exception, "Getting the mode requires initializing.");
        return m_modeToUse;
    }
    bool getModeIsCost() const { return getMode() == Mode::Cost; }
    bool getModeIsEndpointConstraint() const {
        return getMode() == Mode::EndpointConstraint;
    }

    /// Types of goals have a class-level default for whether they are enforced
    /// as a cost or endpoint constraint.
    Mode getDefaultMode() const { return getDefaultModeImpl(); }

    /// Can this constraint be used in endpoint constraint mode?
    bool getSupportsEndpointConstraint() const {
        return getSupportsEndpointConstraintImpl();
    }

    /// Get bounds for the constraints when using this goal in endpoint
    /// constraint mode.
    const MocoConstraintInfo& getConstraintInfo() const {
        return get_MocoConstraintInfo();
    }
    MocoConstraintInfo& updConstraintInfo() { return upd_MocoConstraintInfo(); }

    /// Get the length of the return value of calcGoal().
    int getNumOutputs() const {
        OPENSIM_THROW_IF_FRMOBJ(!m_model, Exception,
                "Getting the number of outputs requires initializing.");
        return getConstraintInfo().getNumEquations();
    }

    /// Get the number of integrals required by this cost.
    /// This returns either 0 (for a strictly-endpoint cost) or 1.
    /// @precondition This goal must be initialized.
    int getNumIntegrals() const {
        OPENSIM_THROW_IF_FRMOBJ(m_numIntegrals == -1, Exception,
                "The goal must be initialized for the number of integrals to "
                "be available.");
        return m_numIntegrals;
    }
    /// Calculate the integrand that should be integrated and passed to
    /// calcCost(). If getNumIntegrals() is not zero, this must be implemented.
    SimTK::Real calcIntegrand(const SimTK::State& state) const {
        double integrand = 0;
        if (!get_enabled()) { return integrand; }
        calcIntegrandImpl(state, integrand);
        return integrand;
    }
    struct GoalInput {
        const SimTK::State& initial_state;
        const SimTK::State& final_state;
        /// This is computed by integrating calcIntegrand().
        const double& integral;
    };
    /// In cost mode, the returned cost includes the weight, and the elements of
    /// the returned vector should be summed by the caller to obtain the total
    /// cost. In endpoint constraint mode, each element of the vector is a
    /// different scalar equation to enforce as a constraint.
    /// The length of the returned vector is getNumOutputs().
    void calcGoal(const GoalInput& input, SimTK::Vector& goal) const {
        goal.resize(getNumOutputs());
        goal = 0;
        if (!get_enabled()) { return; }
        calcGoalImpl(input, goal);
        goal *= m_weightToUse;
    }
    /// For use by solvers. This also performs error checks on the Problem.
    void initializeOnModel(const Model& model) const {
        m_model.reset(&model);
        if (!get_enabled()) { return; }

        // Set mode.
        Mode mode;
        if (getProperty_mode().empty()) {
            mode = getDefaultMode();
        } else {
            checkMode(get_mode());
            mode = (get_mode() == "cost") ? Mode::Cost
                                          : Mode::EndpointConstraint;
        }
        OPENSIM_THROW_IF_FRMOBJ(mode == Mode::EndpointConstraint &&
                                        !getSupportsEndpointConstraint(),
                Exception,
                "Endpoint constraint mode not supported by this goal.");
        m_modeToUse = mode;
        if (m_modeToUse == Mode::EndpointConstraint) {
            m_weightToUse = 1;
        } else {
            m_weightToUse = get_weight();
        }

        initializeOnModelImpl(model);

        OPENSIM_THROW_IF_FRMOBJ(m_numIntegrals == -1, Exception,
                "Expected setNumIntegralsAndOutputs() to be invoked, "
                "but it was not.");
    }

    /// Print the name type and mode of this goal. In cost mode, this prints the
    /// weight.
    void printDescription(std::ostream& stream = std::cout) const;

protected:
    /// Perform any caching before the problem is solved.
    /// You must override this function and invoke setNumIntegralsAndOutputs().
    /// @precondition The model is initialized (initSystem()) and getModel()
    /// is available.
    /// The passed-in model is equivalent to getModel().
    /// Use this opportunity to check for errors in user input.
    virtual void initializeOnModelImpl(const Model&) const = 0;

    /// Set the number of integral terms required by this goal and the length
    /// of the vector passed into calcGoalImpl().
    /// This must be set within initializeOnModelImpl(), otherwise an exception
    /// is thrown during initialization.
    /// The number of integrals must be either 0 or 1.
    void setNumIntegralsAndOutputs(int numIntegrals, int numOutputs) const {
        OPENSIM_THROW_IF(numIntegrals < 0 || numIntegrals > 1, Exception,
                "Number of integrals must be 0 or 1.");
        OPENSIM_THROW_IF(numOutputs < 0, Exception,
                "Number of outputs must be non-negative.");
        m_numIntegrals = numIntegrals;
        const_cast<MocoGoal*>(this)->upd_MocoConstraintInfo().setNumEquations(
                numOutputs);
    }

    virtual Mode getDefaultModeImpl() const { return Mode::Cost; }
    virtual bool getSupportsEndpointConstraintImpl() const { return false; }
    /// @precondition The state is realized to SimTK::Stage::Position.
    /// If you need access to the controls, you must realize to Velocity:
    /// @code
    /// getModel().realizeVelocity(state);
    /// @endcode
    /// The Lagrange multipliers for kinematic constraints are not available.
    virtual void calcIntegrandImpl(
            const SimTK::State& state, double& integrand) const;
    /// The Lagrange multipliers for kinematic constraints are not available.
    virtual void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& goal) const = 0;
    /// Print a more detailed description unique to each goal.
    virtual void printDescriptionImpl(
            std::ostream& stream = std::cout) const {};
    /// For use within virtual function implementations.
    const Model& getModel() const {
        OPENSIM_THROW_IF_FRMOBJ(!m_model, Exception,
                "Model is not available until the start of initializing.");
        return m_model.getRef();
    }

    double calcSystemDisplacement(
            const SimTK::State& initial, const SimTK::State& final) const;

private:
    OpenSim_DECLARE_PROPERTY(
            enabled, bool, "This bool indicates whether this goal is enabled.");
    OpenSim_DECLARE_PROPERTY(weight, double,
            "In cost mode, the goal is multiplied by this weight (default: "
            "1).");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(mode, std::string,
            "'cost' to enforce as a penalty, 'endpoint_constraint' to enforce "
            "as a constraint.");
    OpenSim_DECLARE_UNNAMED_PROPERTY(MocoConstraintInfo,
            "The bounds and labels for this MocoGoal, if applied as an "
            "endpoint constraint.");

    void constructProperties();

    void checkMode(const std::string& mode) const {
        OPENSIM_THROW_IF_FRMOBJ(mode != "cost" && mode != "endpoint_constraint",
                Exception,
                format("Expected mode to be 'cost' or 'endpoint_constraint' "
                       "but got %s.",
                        mode));
    }

    mutable SimTK::ReferencePtr<const Model> m_model;
    mutable double m_weightToUse;
    mutable Mode m_modeToUse;
    mutable int m_numIntegrals = -1;
};

inline void MocoGoal::calcIntegrandImpl(
        const SimTK::State& /*state*/, double& /*integrand*/) const {}

/// Endpoint cost for final time.
/// @ingroup mocogoal
class OSIMMOCO_API MocoFinalTimeGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoFinalTimeGoal, MocoGoal);

public:
    MocoFinalTimeGoal() = default;
    MocoFinalTimeGoal(std::string name) : MocoGoal(std::move(name)) {}
    MocoFinalTimeGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {}

protected:
    void initializeOnModelImpl(const Model&) const override {
        setNumIntegralsAndOutputs(0, 1);
    }
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
        cost[0] = input.final_state.getTime();
    }
};

/// This goal requires the average speed of the system to match a desired
/// average speed. The average speed of the system is the displacement of the
/// system's center of mass divided by the duration of the phase.
/// @ingroup mocogoal
class MocoAverageSpeedGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoAverageSpeedGoal, MocoGoal);

public:
    OpenSim_DECLARE_PROPERTY(desired_average_speed, double,
            "The desired average speed of the system (m/s). Default: 0.");
    MocoAverageSpeedGoal() { constructProperties(); }
    MocoAverageSpeedGoal(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }

protected:
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    Mode getDefaultModeImpl() const override {
        return Mode::EndpointConstraint;
    }
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& values) const override {
        SimTK::Real timeInitial = input.initial_state.getTime();
        SimTK::Real timeFinal = input.final_state.getTime();
        SimTK::Real duration = timeFinal - timeInitial;

        SimTK::Vec3 comInitial =
                getModel().calcMassCenterPosition(input.initial_state);
        SimTK::Vec3 comFinal =
                getModel().calcMassCenterPosition(input.final_state);
        // TODO: Use distance squared for convexity.
        SimTK::Real displacement = (comFinal - comInitial).norm();
        // Calculate average gait speed.
        values[0] = get_desired_average_speed() - (displacement / duration);
        if (getModeIsCost()) { values[0] = SimTK::square(values[0]); }
    }
    void initializeOnModelImpl(const Model&) const override {
        setNumIntegralsAndOutputs(0, 1);
    }

private:
    void constructProperties() { constructProperty_desired_average_speed(0); }
};

} // namespace OpenSim

#endif // MOCO_MOCOGOAL_H
