#ifndef OPENSIM_MOCOOUTPUTGOAL_H
#define OPENSIM_MOCOOUTPUTGOAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoOutputGoal.h                                                  *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2022 Stanford University and the Authors                     *
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

#include "MocoGoal.h"

namespace OpenSim {

/** This abstract base class provides convenience methods and common interfaces
for all Output-related MocoGoal's. All MocoGoal's deriving from this class
include the 'setOutputPath()', 'setOutputIndex()', and 'setExponent()' methods
and their corresponding Object properties. The convenience method
'initializeOnModelBase()' should be called at the top of
'initializeOnModelImpl()' within each derived class. Similarly,
'calcOutputValue()' can be used to retrieve the Output value with
'calcGoalImpl()' and/or 'calcIntegrandImpl()', as needed for each derived class.
The method 'getDependsOnStage()' returns the SimTK::Stage that should be realized
to to calculate Output values. The method 'setValueToExponent()' can be used to
raise a value to the exponent provided via 'setExponent()'.

We support the following Output types:
- double
- SimTK::Vec3
- SimTK::SpatialVec

When using vector types, 'setOutputIndex()' may be used to select a specific
element of the Output vector. If not specified, the norm of the vector is
returned when calling 'calcOutputValue()'.

@ingroup mocogoal */
class OSIMMOCO_API MocoOutputBase : public MocoGoal {
    OpenSim_DECLARE_ABSTRACT_OBJECT(MocoOutputBase, MocoGoal);

public:
    MocoOutputBase() { constructProperties(); }
    MocoOutputBase(std::string name) : MocoGoal(std::move(name)) {
        constructProperties();
    }
    MocoOutputBase(std::string name, double weight)
            : MocoGoal(std::move(name), weight) {
        constructProperties();
    }

    /** Set the absolute path to the output in the model to use as the integrand
    for this goal. The format is "/path/to/component|output_name". */
    void setOutputPath(std::string path) { set_output_path(std::move(path)); }
    const std::string& getOutputPath() const { return get_output_path(); }

    /** Set the exponent applied to the output value in the integrand. This
    exponent is applied when minimizing the norm of a vector type output. The
    default exponent is set to 1, meaning that the output can take on negative
    values in the integrand. When the exponent is set to a value greater than
    1, the absolute value function is applied to the output (before the
    exponent is applied), meaning that odd numbered exponents (greater than 1)
    do not take on negative values. */
    void setExponent(int exponent) { set_exponent(exponent); }
    int getExponent() const { return get_exponent(); }

    /** Set the index to the value to be minimized when a vector type Output is
    specified. For SpatialVec Outputs, indices 0, 1, and 2 refer to the
    rotational components and indices 3, 4, and 5 refer to the translational
    components. A value of -1 indicates to minimize the vector norm (which is the
    default setting). If an index for a type double Output is provided, an
    exception is thrown. */
    void setOutputIndex(int index) { set_output_index(index); }
    int getOutputIndex() const { return get_output_index(); }

protected:

    /** Get a reference to the Output at the specified Output path and store its
    data type. This also creates a function based on the exponent set via
    'setExponent()', which can be accessed with 'setValueToExponent()'. Finally,
    this also sets the "depends-on stage", which can be accessed with
    'getDependsOnStage()'. Call this function at the top of
    'initializeOnModelImpl()' in each derived class. */
    void initializeOnModelBase() const;

    /** Calculate the Output value for the provided SimTK::State. If using a
    vector Output, either the vector norm or vector element will be returned,
    depending on whether an index was provided via 'setOutputIndex()'. Do not
    call this function until 'initializeOnModelBase()' has been called. */
    double calcOutputValue(const SimTK::State&) const;

    /** Raise a value to the exponent set via 'setExponent()'. Do not call this
    function until 'initializeOnModelBase()' has been called. */
    double setValueToExponent(double value) const {
        return m_power_function(value);
    }

    /** Get the "depends-on stage", or the SimTK::Stage we need to realize the
    system to in order to calculate the Output value. */
    const SimTK::Stage& getDependsOnStage() const {
        return m_dependsOnStage;
    }

    void printDescriptionImpl() const override;


private:
    OpenSim_DECLARE_PROPERTY(output_path, std::string,
            "The absolute path to the output in the model to use as the "
            "integrand for this goal.");
    OpenSim_DECLARE_PROPERTY(exponent, int,
            "The exponent applied to the output value in the integrand. "
            "The output can take on negative values in the integrand when the "
            "exponent is set to 1 (the default value). When the exponent is "
            "set to a value greater than 1, the absolute value function is "
            "applied to the output (before the exponent is applied), meaning "
            "that odd numbered exponents (greater than 1) do not take on "
            "negative values.");
    OpenSim_DECLARE_PROPERTY(output_index, int,
            "The index to the value to be minimized when a vector type "
            "Output is specified. For SpatialVec Outputs, indices 0, 1, "
            "and 2 refer to the rotational components and indices 3, 4, "
            "and 5 refer to the translational components. A value of -1 "
            "indicates to minimize the vector norm (default: -1).");
    void constructProperties();

    enum DataType {
        Type_double,
        Type_Vec3,
        Type_SpatialVec,
    };
    mutable DataType m_data_type;
    mutable SimTK::ReferencePtr<const AbstractOutput> m_output;
    mutable std::function<double(const double&)> m_power_function;
    mutable int m_index1;
    mutable int m_index2;
    mutable bool m_minimizeVectorNorm;
    mutable SimTK::Stage m_dependsOnStage = SimTK::Stage::Acceleration;
};

/** This goal allows you to use model Outputs of type double, SimTK::Vec3, and
SimTK::SpatialVec in the integrand of a goal. By default, when using vector type
Outputs, the norm of the vector is minimized, but you can also minimize a
specific element of a vector Output via `setOutputIndex()`. You can also specify
the exponent of the value in the integrand via `setExponent()`.

This goal supports both "Cost" (default) and "EndpointConstraint" modes. In
"EndpointConstraint" mode, the integral of the Output value is constrained
between user-specified bounds. By default, these bounds constrain the integral to
zero; use 'updConstraintInfo()' to set custom bounds.
@ingroup mocogoal */
class OSIMMOCO_API MocoOutputGoal : public MocoOutputBase {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoOutputGoal, MocoOutputBase);

public:
    MocoOutputGoal() { constructProperties(); }
    MocoOutputGoal(std::string name) : MocoOutputBase(std::move(name)) {
        constructProperties();
    }
    MocoOutputGoal(std::string name, double weight)
            : MocoOutputBase(std::move(name), weight) {
        constructProperties();
    }

    /** Set if the goal should be divided by the displacement of the system's
    center of mass over the phase. */
    void setDivideByDisplacement(bool tf) { set_divide_by_displacement(tf); }
    bool getDivideByDisplacement() const {
        return get_divide_by_displacement();
    }

    /** Set if the goal should be divided by the total mass of the model. */
    void setDivideByMass(bool tf) { set_divide_by_mass(tf); }
    bool getDivideByMass() const {
        return get_divide_by_mass();
    }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& state, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& values) const override;
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    Mode getDefaultModeImpl() const override {
        return Mode::Cost;
    }

private:
    OpenSim_DECLARE_PROPERTY(divide_by_displacement, bool,
            "Divide by the model's displacement over the phase (default: "
            "false)");
    OpenSim_DECLARE_PROPERTY(divide_by_mass, bool,
            "Divide by the model's total mass (default: false)");
    void constructProperties();
};

/** This goal permits the integration of only positive or negative values from a
model Output. This goal allows you to use model Outputs of type double, or a 
single specified element from an Output of type SimTK::Vec3 or 
SimTK::SpatialVec in the integrand of a goal. You can also specify the exponent
of the value in the integrand via 'setExponent()'. 

This goal performs a smooth approximation of the common 'minimum' or 'maximum'
extremum functions and then calculates the resulting integral. 
The goal is computed as follows:

\f[
\frac{1}{dm} \int_{t_i}^{t_f} 
    w_v\beta((\frac{1}{s} (\ln (1 + \exp (s\betav))))^p) ~dt
\f]
We use the following notation:
- \f$ d \f$: displacement of the system, if `divide_by_displacement` is
  true; 1 otherwise.
- \f$ m \f$: mass of the system, if `divide_by_mass` is
  true; 1 otherwise.
- \f$ v \f$: the output variable of choice.
- \f$ w_v \f$: the weight for output variable \f$ v \f$.
- \f$ \beta \f$: the approximate extremum to be taken (\beta == -1 for
  minimum; \beta == 1 for maximum).
- \f$ s \f$: the smoothing factor for approximating the extremum. With
  \f$ s \f$ == 1 the approximation is closer to the true extremum taken.
  For \f$ v \f$ with potentially large magnitudes (> 2000) during a simulation
  it is recommended to set this property as 0.2 to avoid Inf.
- \f$ p \f$: the `exponent`.
*/
class OSIMMOCO_API MocoOutputExtremumGoal : public MocoOutputBase {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoOutputExtremumGoal, MocoOutputBase);

public:
    MocoOutputExtremumGoal() { constructProperties(); }
    MocoOutputExtremumGoal(std::string name) : MocoOutputBase(std::move(name)) {
        constructProperties();
    }
    MocoOutputExtremumGoal(std::string name, double weight)
            : MocoOutputBase(std::move(name), weight) {
        constructProperties();
    }

    /** Set if the goal should be divided by the displacement of the system's
    center of mass over the phase. */
    void setDivideByDisplacement(bool tf) { set_divide_by_displacement(tf); }
    bool getDivideByDisplacement() const {
        return get_divide_by_displacement();
    }

    /** Set if the goal should be divided by the total mass of the model. */
    void setDivideByMass(bool tf) { set_divide_by_mass(tf); }
    bool getDivideByMass() const { return get_divide_by_mass(); }

    /** Set the type of extremum ('minimum' or 'maximum') to be applied to the 
    output variable of choice. */
    void setExtremumType(std::string extremum_type) {
        set_extremum_type(extremum_type);
    }
    std::string getExtremumType() const { return get_extremum_type(); }

    /** Set the smoothing factor used for the extremum approximation
    (default = 1.0). This property can be set between [0.2, 1.0]. For Outputs
    that may take on large values (> ~2000) during a simulation, it is
    recommended to set this property closer to 0.2.*/
    void setSmoothingFactor(double smoothing_factor) {
        set_smoothing_factor(smoothing_factor);
    }
    double getSmoothingFactor() const { return get_smoothing_factor(); }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& state, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& values) const override;
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    Mode getDefaultModeImpl() const override { return Mode::Cost; }

private:
    OpenSim_DECLARE_PROPERTY(divide_by_displacement, bool,
            "Divide by the model's displacement over the phase (default: "
            "false)");
    OpenSim_DECLARE_PROPERTY(divide_by_mass, bool,
            "Divide by the model's total mass (default: false)");
    OpenSim_DECLARE_PROPERTY(extremum_type, std::string,
            "The type of extremum to be taken in the goal."
            "'minimum' or 'maximum'"
            "(Default extremum type = 'minimum').");
    OpenSim_DECLARE_PROPERTY(smoothing_factor, double,
            "The smoothing factor applied in the approximation of the "
            "extremum function (default = 1.0). This property can be set "
            "between [0.2, 1.0]. For Outputs that may take on large values "
            "(> ~2000) during a simulation, it is recommended to set this "
            "property closer to 0.2.");

    enum DataType {
        Type_double,
        Type_Vec3,
        Type_SpatialVec,
    };
    mutable DataType m_data_type;
    mutable int m_beta;
    mutable bool m_minimizeVectorNorm; 

    void constructProperties();
};

/** This goal allows you to minimize or constrain a Model Output value at the
beginning of a trajectory. Outputs of type double, SimTK::Vec3, and
SimTK::SpatialVec are supported. By default, when using vector type Outputs, the
norm of the vector is minimized, but you can also minimize a specific element of
a vector Output via `setOutputIndex()`. You can also specify the exponent of the
value in the integrand via `setExponent()`.

This goal supports both "Cost" (default) and "EndpointConstraint" modes. In
"EndpointConstraint" mode, the Output value is constrained between user-specified
bounds. By default, these bounds constrain the initial value to zero; use
'updConstraintInfo()' to set custom bounds.
@ingroup mocogoal */
class OSIMMOCO_API MocoInitialOutputGoal : public MocoOutputBase {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoInitialOutputGoal, MocoOutputBase);

public:
    MocoInitialOutputGoal() { constructProperties(); }
    MocoInitialOutputGoal(std::string name) : MocoOutputBase(std::move(name)) {
        constructProperties();
    }
    MocoInitialOutputGoal(std::string name, double weight)
            : MocoOutputBase(std::move(name), weight) {
        constructProperties();
    }
protected:
    void initializeOnModelImpl(const Model&) const override {
        initializeOnModelBase();
        setRequirements(0, 1, getDependsOnStage());
    }
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& values) const override {
        values[0] = setValueToExponent(calcOutputValue(input.initial_state));
    }
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    Mode getDefaultModeImpl() const override {
        return Mode::Cost;
    }

private:
    void constructProperties() {};
};

/** This goal allows you to minimize or constrain a Model Output value at the
end of a trajectory. Outputs of type double, SimTK::Vec3, and SimTK::SpatialVec
are supported. By default, when using vector type Outputs, the norm of the vector
is minimized, but you can also minimize a specific element of a vector Output via
`setOutputIndex()`. You can also specify the exponent of the value in the
integrand via `setExponent()`.

This goal supports both "Cost" (default) and "EndpointConstraint" modes. In
"EndpointConstraint" mode, the Output value is constrained between user-specified
bounds. By default, these bounds constrain the final value to zero; use
'updConstraintInfo()' to set custom bounds.
@ingroup mocogoal */
class OSIMMOCO_API MocoFinalOutputGoal : public MocoOutputBase {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoFinalOutputGoal, MocoOutputBase);

public:
    MocoFinalOutputGoal() { constructProperties(); }
    MocoFinalOutputGoal(std::string name) : MocoOutputBase(std::move(name)) {
        constructProperties();
    }
    MocoFinalOutputGoal(std::string name, double weight)
            : MocoOutputBase(std::move(name), weight) {
        constructProperties();
    }
protected:
    void initializeOnModelImpl(const Model&) const override {
        initializeOnModelBase();
        setRequirements(0, 1, getDependsOnStage());
    }
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& values) const override {
        values[0] = setValueToExponent(calcOutputValue(input.final_state));
    }
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    Mode getDefaultModeImpl() const override {
        return Mode::Cost;
    }

private:
    void constructProperties() {};
};

/** This goal allows you to minimize or constrain the difference of values from
a Model Output from the beginning and end of a trajectory. Outputs of type
double, SimTK::Vec3, and SimTK::SpatialVec are supported. By default, when using
vector type Outputs, the norm of the vector is minimized, but you can also
minimize a specific element of a vector Output via `setOutputIndex()`. You can
also specify the exponent of the value in the integrand via `setExponent()`.

This goal supports both "Cost" (default) and "EndpointConstraint" modes. In
"EndpointConstraint" mode, the difference in Output values is constrained between
user-specified bounds. By default, these bounds constrain the final value to
zero; use 'updConstraintInfo()' to set custom bounds.

@note The exponent provided via 'setExponent()' is applied to the difference
      between final and initial Output values.
@ingroup mocogoal */
class OSIMMOCO_API MocoOutputPeriodicityGoal : public MocoOutputBase {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoOutputPeriodicityGoal, MocoOutputBase);

public:
    MocoOutputPeriodicityGoal() { constructProperties(); }
    MocoOutputPeriodicityGoal(std::string name) :
            MocoOutputBase(std::move(name)) {
        constructProperties();
    }
    MocoOutputPeriodicityGoal(std::string name, double weight)
            : MocoOutputBase(std::move(name), weight) {
        constructProperties();
    }
protected:
    void initializeOnModelImpl(const Model&) const override {
        initializeOnModelBase();
        setRequirements(0, 1, getDependsOnStage());
    }
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& values) const override {
        values[0] = setValueToExponent(
                calcOutputValue(input.final_state) -
                calcOutputValue(input.initial_state));
    }
    bool getSupportsEndpointConstraintImpl() const override { return true; }
    Mode getDefaultModeImpl() const override {
        return Mode::Cost;
    }

private:
    void constructProperties() {};
};

/** This goal allows you to minimize the squared difference between a Model
Output value and a user-defined function. Outputs of type double, SimTK::Vec3,
and SimTK::SpatialVec are supported. By default, when using vector type Outputs,
the norm of the vector is tracked, but you can also track a specific element of a
vector Output via `setOutputIndex()`.
@note The exponent provided via 'setExponent()' is applied to the difference
      between Output value and the tracking function.
@ingroup mocogoal */
class OSIMMOCO_API MocoOutputTrackingGoal : public MocoOutputBase {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoOutputTrackingGoal, MocoOutputBase);

public:
    MocoOutputTrackingGoal() { constructProperties(); }
    MocoOutputTrackingGoal(std::string name) : MocoOutputBase(std::move(name)) {
        constructProperties();
    }
    MocoOutputTrackingGoal(std::string name, double weight)
            : MocoOutputBase(std::move(name), weight) {
        constructProperties();
    }

    /// The function of time that the Output value will track in the integrand.
    void setTrackingFunction(const Function& f) {
        set_tracking_function(f);
    }
    const Function& getTrackingFunction() const {
        return get_tracking_function();
    }

protected:
    void initializeOnModelImpl(const Model&) const override;
    void calcIntegrandImpl(
            const IntegrandInput& input, double& integrand) const override;
    void calcGoalImpl(
            const GoalInput& input, SimTK::Vector& values) const override;

private:
    OpenSim_DECLARE_OPTIONAL_PROPERTY(tracking_function, Function,
            "A function of time that the model Output value will track in the "
            "integrand.");
    void constructProperties();
};

} // namespace OpenSim

#endif // OPENSIM_MOCOOUTPUTGOAL_H