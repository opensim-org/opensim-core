#ifndef OPENSIM_MOCOGOAL_H
#define OPENSIM_MOCOGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoGoal.h                                                        *
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

#include <SimTKcommon/internal/State.h>

#include <OpenSim/Common/Object.h>
#include <OpenSim/Moco/MocoBounds.h>
#include <OpenSim/Moco/MocoConstraintInfo.h>
#include <OpenSim/Moco/MocoScaleFactor.h>
#include <OpenSim/Moco/osimMocoDLL.h>

namespace OpenSim {

class Model;

// TODO give option to specify gradient and Hessian analytically.

/** A goal is a term in the cost functional to be minimized, or a set of endpoint
constraints that must lie within provided bounds. Goals depend on the
phase's initial and final states and controls, and optionally on the
integral of a quantity over the phase.
Not all goals support endpoint constraint mode; see
getSupportsEndpointConstraint(). If a goal does support endpoint constraint
mode, then the default mode is available via getMode(). Use endpoint
constraint mode if you require the goal to be met strictly and do not want
to allow a trade-off between this goal and other goals.
The calculation of the goal may differ between cost and endpoint constraint
modes; cost mode may require that outputs are squared, for example.

# Stage dependency
Some goals require less of IntegrandInput and GoalInput than others. To
ensure goals are computed efficiently, goals can specify a stage dependency,
which tells solvers what to do when preparing IntegrandInput and GoalInput.
Here are the expectations for each SimTK::Stage:

- SimTK::Stage::Topology: the time field of IntegrandInput and
    initial_time and final_time fields of GoalInput are available.
- SimTK::Stage::Model: controls fields of IntegrandInput and
    GoalInput are available.
- SimTK::Stage::Instance: MocoParameters are applied to the model.
- SimTK::Stage::Time: state variables (SimTK::State::getY(), etc.) are
    available, and SimTK::Stage::getTime() is updated.
- SimTK::Stage::Position: state, initial_state, and final_state can be used
    to compute position-dependent quantities.
- SimTK::Stage::Velocity: state, initial_state, and final_state can be used
    to compute velocity-dependent quantities.
- SimTK::Stage::Dynamics: state, initial_state, and final_state can be used
    to compute force-dependent quantities.
- SimTK::Stage::Acceleration: state, initial_state, and final_state can be
    used to compute acceleration-dependent quantities, such as body
    accelerations and joint reactions.

## Scale factors
Goals may include an option to add scale factors to the MocoProblem using
`appendScaleFactor()`, which takes a MocoScaleFactor object for its argument.
A copy of this component is added to the Model internal to MocoProblemRep and
its property value is optimized via a MocoParameter. Scale factor usage is
specific to each MocoGoal (if used at all).

@par For developers
Every time the problem is solved, a copy of this goal is used. An individual
instance of a goal is only ever used in a single problem. Therefore, there
is no need to clear cache variables that you create in initializeImpl().
Also, information stored in this goal does not persist across multiple
solves.
@ingroup mocogoal */
class OSIMMOCO_API MocoGoal : public Object {
    OpenSim_DECLARE_ABSTRACT_OBJECT(MocoGoal, Object);

public:
    MocoGoal();

    MocoGoal(std::string name);

    MocoGoal(std::string name, double weight);

    /// Set whether this goal is used in the problem.
    void setEnabled(bool enabled) { set_enabled(enabled); }
    bool getEnabled() const { return get_enabled(); }

    /// In cost mode, the goal is multiplied by this weight. Use the weight to
    /// control the relative importance of terms in the cost functional. The
    /// weight is ignored (e.g., set to 1) in endpoint constraint mode.
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

    /// Can this goal be used in endpoint constraint mode?
    bool getSupportsEndpointConstraint() const {
        return getSupportsEndpointConstraintImpl();
    }

    /// Get bounds for the constraints that are enforced when using this goal in
    /// endpoint constraint mode.
    /// This info is ignored if getSupportsEndpointConstraint() is false.
    const MocoConstraintInfo& getConstraintInfo() const {
        return get_MocoConstraintInfo();
    }
    MocoConstraintInfo& updConstraintInfo() { return upd_MocoConstraintInfo(); }

    /// Set the vector of endpoint constraint bounds for this MocoGoal. This
    /// vector must have length equal to the number of outputs for this goal,
    /// otherwise an exception is thrown.
    /// This info is ignored if getSupportsEndpointConstraint() is false.
    void setEndpointConstraintBounds(const std::vector<MocoBounds>& bounds) {
        updConstraintInfo().setBounds(bounds);
    }
    /// Get the vector of the endpoint constraint bounds for this MocoGoal.
    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    std::vector<MocoBounds> getEndpointConstraintBounds() const {
        std::vector<MocoBounds> bounds(getConstraintInfo().getBounds());
        return bounds;
    }

    /// Get the length of the return value of calcGoal().
    int getNumOutputs() const {
        OPENSIM_THROW_IF_FRMOBJ(!m_model, Exception,
                "Getting the number of outputs requires initializing.");
        return getConstraintInfo().getNumEquations();
    }

    /// Get the number of integrals required by this cost.
    /// This returns either 0 (for a strictly-endpoint cost) or 1.
    /// @precondition initializeOnModel() has been invoked.
    int getNumIntegrals() const {
        OPENSIM_THROW_IF_FRMOBJ(m_numIntegrals == -1, Exception,
                "The goal must be initialized for the number of integrals to "
                "be available.");
        return m_numIntegrals;
    }

    /// Obtain the stage that this goal depends on. Solvers can use this to more
    /// efficiently decide how to set the IntegrandInput and GoalInput. See the
    /// MocoGoal class description for details about the different stages. The
    /// stage dependency is the same for both modes of the goal (cost and
    /// endpoint constraint).
    // TODO: create separate getIntegrandStageDependency() and
    // getEndpointStageDependency() rather than using the same stage dependency
    // for both (as is the case now).
    SimTK::Stage getStageDependency() const {
        return m_stageDependency;
    }

    struct IntegrandInput {
        /// Time is available regardless of the stage dependency.
        const SimTK::Real& time;
        /// The time in the state is only updated if the stage requirement is
        /// SimTK::Stage::Time or greater.
        /// If you need access to the state variables, you must set a stage
        /// requirement of SimTK::Stage::Time.
        const SimTK::State& state;
        /// Controls are available with a stage requirement of
        /// SimTK::Stage::Model.
        /// If you only need to access the controls, use this field.
        /// This vector has length Model::getNumControls(), and has slots for
        /// disabled actuators. The function createControlNamesForModel()
        /// provides data structures that can be indexed in the same way as this
        /// field. Use this field rather than Model::getControls().
        const SimTK::Vector& controls;
    };
    /// Calculate the integrand that should be integrated and passed to
    /// calcCost(). If getNumIntegrals() is not zero, this must be implemented.
    /// @precondition initializeOnModel() has been invoked.
    SimTK::Real calcIntegrand(const IntegrandInput& input) const {
        double integrand = 0;
        if (!get_enabled()) { return integrand; }
        const SimTK::Stage stageBefore = input.state.getSystemStage();

        calcIntegrandImpl(input, integrand);

        if (input.state.getSystemStage() > stageBefore) {
            SimTK_ERRCHK2_ALWAYS(
                    input.state.getSystemStage() <= m_stageDependency,
                    (getConcreteClassName() + "::calcIntegrand()").c_str(),
                    "This goal has a stage dependency of %s, but "
                    "calcIntegrandImpl() exceeded this stage by realizing "
                    "to %s.",
                    m_stageDependency.getName().c_str(),
                    input.state.getSystemStage().getName().c_str());
        }
        return integrand;
    }

    /// @see IntegrandInput.
    struct GoalInput {
        const SimTK::Real& initial_time;
        const SimTK::State& initial_state;
        const SimTK::Vector& initial_controls;
        const SimTK::Real& final_time;
        const SimTK::State& final_state;
        const SimTK::Vector& final_controls;
        /// The solver computes the integral by integrating calcIntegrand().
        const double& integral;
    };
    /// In cost mode, the returned cost includes the weight, and the elements of
    /// the returned vector should be summed by the caller to obtain the total
    /// cost. In endpoint constraint mode, each element of the vector is a
    /// different scalar equation to enforce as a constraint.
    /// The length of the returned vector is getNumOutputs().
    /// @precondition initializeOnModel() has been invoked.
    void calcGoal(const GoalInput& input, SimTK::Vector& goal) const {
        goal.resize(getNumOutputs());
        goal = 0;
        if (!get_enabled()) { return; }
        const SimTK::Stage initialStageBefore =
                input.initial_state.getSystemStage();
        const SimTK::Stage finalStageBefore =
                input.final_state.getSystemStage();

        calcGoalImpl(input, goal);

        if (input.initial_state.getSystemStage() > initialStageBefore) {
            SimTK_ERRCHK2_ALWAYS(
                    input.initial_state.getSystemStage() <= m_stageDependency,
                    (getConcreteClassName() + "::calcGoal()").c_str(),
                    "This goal has a stage dependency of %s, but "
                    "calcGoalImpl() exceeded this stage by realizing "
                    "initial_state to %s.",
                    m_stageDependency.getName().c_str(),
                    input.initial_state.getSystemStage().getName().c_str());
        }
        if (input.final_state.getSystemStage() > finalStageBefore) {
            SimTK_ERRCHK2_ALWAYS(
                    input.final_state.getSystemStage() <= m_stageDependency,
                    (getConcreteClassName() + "::calcGoal()").c_str(),
                    "This goal has a stage dependency of %s, but "
                    "calcGoalImpl() exceeded this stage by realizing "
                    "final_state to %s.",
                    m_stageDependency.getName().c_str(),
                    input.final_state.getSystemStage().getName().c_str());
        }
        goal *= m_weightToUse;
    }

    /// Perform error checks on user input for this goal, and cache
    /// quantities needed when computing the goal value.
    /// This function must be invoked before invoking calcIntegrand() or
    /// calcGoal().
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
                "Expected setRequirements() to be invoked, "
                "but it was not.");
    }

    /// Get a vector of the MocoScaleFactors added to this MocoGoal.
    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    std::vector<MocoScaleFactor> getScaleFactors() const {
        std::vector<MocoScaleFactor> scaleFactors;
        for (int i = 0; i < getProperty_scale_factors().size(); ++i) {
            scaleFactors.push_back(get_scale_factors(i));
        }
        return scaleFactors;
    }

    /// Print the name type and mode of this goal. In cost mode, this prints the
    /// weight.
    void printDescription() const;

protected:
    /// Perform any caching before the problem is solved.
    /// You must override this function and invoke setRequirements().
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
    /// The stageDependency can be set to lower stages to avoid unnecessary
    /// calculations. For example, if the goal does not depend on forces,
    /// then the solver does not need to prepare the
    /// IntegrandInput and GoalInput for force calculations.
    /// See the MocoGoal class description for help with choosing the stage
    /// dependency.
    /// If you are not sure what your stageDependency is, leave it as
    /// SimTK::Stage::Acceleration to be safe.
    ///
    /// You must still realize to the appropriate stage within the
    /// integrand and goal functions. Setting the stageDependency to stage X
    /// does not mean that the SimTK::State is realized to stage X as a
    /// precondition of calcIntegrandImpl() and calcGoalImpl().
    void setRequirements(int numIntegrals, int numOutputs,
            SimTK::Stage stageDependency = SimTK::Stage::Acceleration) const {
        OPENSIM_THROW_IF(numIntegrals < 0 || numIntegrals > 1, Exception,
                "Number of integrals must be 0 or 1.");
        OPENSIM_THROW_IF(numOutputs < 0, Exception,
                "Number of outputs must be non-negative.");
        m_numIntegrals = numIntegrals;
        const_cast<MocoGoal*>(this)->upd_MocoConstraintInfo().setNumEquations(
                numOutputs);
        m_stageDependency = stageDependency;
    }

    virtual Mode getDefaultModeImpl() const { return Mode::Cost; }
    virtual bool getSupportsEndpointConstraintImpl() const { return false; }
    /// You may need to realize the state to the stage required for your
    /// calculations.
    /// Do NOT realize to a stage higher than the goal's stage dependency;
    /// doing so will cause an exception to be thrown.
    /// The Lagrange multipliers for kinematic constraints are not available.
    virtual void calcIntegrandImpl(
            const IntegrandInput& input, SimTK::Real& integrand) const;
    /// You may need to realize the state to the stage required for your
    /// calculations.
    /// Do NOT realize to a stage higher than the goal's stage dependency;
    /// doing so will cause an exception to be thrown.
    /// The Lagrange multipliers for kinematic constraints are not available.
    virtual void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& goal) const = 0;
    /// Print a more detailed description unique to each goal.
    virtual void printDescriptionImpl() const {};
    /// For use within virtual function implementations.
    const Model& getModel() const {
        OPENSIM_THROW_IF_FRMOBJ(!m_model, Exception,
                "Model is not available until the start of initializing.");
        return m_model.getRef();
    }

    double calcSystemDisplacement(
            const SimTK::State& initial, const SimTK::State& final) const;

    /// Append a MocoScaleFactor to this MocoGoal.
    void appendScaleFactor(const MocoScaleFactor& scaleFactor) {
        append_scale_factors(scaleFactor);
    }

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
    OpenSim_DECLARE_LIST_PROPERTY(scale_factors, MocoScaleFactor,
            "Scale factors added by derived MocoGoal classes that are optimized "
            "via a MocoParameter. A copy of each MocoScaleFactor component is "
            "added to the model internal to MocoProblem, which makes the scale "
            "factors values available when computing the cost function for each "
            "MocoGoal.")

    void constructProperties();

    void checkMode(const std::string& mode) const {
        OPENSIM_THROW_IF_FRMOBJ(mode != "cost" && mode != "endpoint_constraint",
                Exception,
                fmt::format(
                        "Expected mode to be 'cost' or 'endpoint_constraint' "
                        "but got {}.",
                        mode));
    }

    mutable SimTK::ReferencePtr<const Model> m_model;
    mutable double m_weightToUse;
    mutable Mode m_modeToUse;
    mutable SimTK::Stage m_stageDependency = SimTK::Stage::Acceleration;
    mutable int m_numIntegrals = -1;
};

inline void MocoGoal::calcIntegrandImpl(
        const IntegrandInput&, SimTK::Real&) const {}

/** Endpoint cost for final time.
@ingroup mocogoal */
class OSIMMOCO_API MocoFinalTimeGoal : public MocoGoal {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoFinalTimeGoal, MocoGoal);

public:
    MocoFinalTimeGoal() = default;
    MocoFinalTimeGoal(std::string name) : MocoGoal(std::move(name)) {}
    MocoFinalTimeGoal(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {}

protected:
    void initializeOnModelImpl(const Model&) const override {
        setRequirements(0, 1, SimTK::Stage::Topology);
    }
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& cost) const override {
        cost[0] = input.final_time;
    }
};

/** This goal requires the average speed of the system to match a desired
average speed. The average speed of the system is the displacement of the
system's center of mass divided by the duration of the phase.

In endpoint constraint mode, the goal is computed as follows:

\f[
 v_\mathrm{des} - \frac{r_\mathrm{com}(t_f) - r_\mathrm{com}(t_i)}{t_f - t_i}
\f]

We use the following notation:
- \f$ v_\mathrm{des} \f$: desired average speed.
- \f$ r_\mathrm{com}(t) \f$: mass center position.
- \f$ t_i \f$: initial time.
- \f$ t_f \f$: final time.

In cost mode, the value of the goal is the above quantity squared.
@ingroup mocogoal */
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
        setRequirements(0, 1);
    }

private:
    void constructProperties() { constructProperty_desired_average_speed(0); }
};

} // namespace OpenSim

#endif // OPENSIM_MOCOGOAL_H
