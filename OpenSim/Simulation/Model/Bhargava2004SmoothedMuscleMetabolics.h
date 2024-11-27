#ifndef BHARGAVA2004SMOOTHEDMUSCLEMETABOLICS_H
#define BHARGAVA2004SMOOTHEDMUSCLEMETABOLICS_H
/* -------------------------------------------------------------------------- *
 *                 Bhargava2004SmoothedMuscleMetabolics.h                     *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Antoine Falisse, Christopher Dembia, Nicholas Bianco            *
 * Contributors: Tim Dorn, Thomas Uchida                                      *
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <unordered_map>

#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/Muscle.h>

namespace OpenSim {

/** Object class that holds the metabolic parameters required to calculate
metabolic power for a single muscle. */
class OSIMSIMULATION_API Bhargava2004SmoothedMuscleMetabolics_MuscleParameters 
        : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            Bhargava2004SmoothedMuscleMetabolics_MuscleParameters, Component);
public:
    OpenSim_DECLARE_PROPERTY(specific_tension, double,
        "The specific tension of the muscle (Pascals (N/m^2), default is "
        "0.25e6).");
    OpenSim_DECLARE_PROPERTY(density, double,
        "The density of the muscle (kg/m^3, default is 1059.7).");
    OpenSim_DECLARE_PROPERTY(ratio_slow_twitch_fibers, double,
        "Ratio of slow twitch fibers in the muscle "
        "(must be between 0 and 1, default is 0.5).");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(use_provided_muscle_mass, bool,
        "An optional flag that allows the user to explicitly specify a muscle "
        "mass. If set to true, the 'provided_muscle_mass' property must be "
        "specified (default is false).");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(provided_muscle_mass, double,
        "The user specified muscle mass (kg, default is NaN). When this "
        "property is NaN, the muscle mass is calculated as follows: "
        "(volume * density) / specific_tension) where "
        "volume = maximal_isometric_force * optimal_fiber_length.");
    OpenSim_DECLARE_PROPERTY(activation_constant_slow_twitch, double,
        "Activation constant for slow twitch fibers (W/kg, default is 40.0).");
    OpenSim_DECLARE_PROPERTY(activation_constant_fast_twitch, double,
        "Activation constant for fast twitch fibers (W/kg, default is "
        "133.0).");
    OpenSim_DECLARE_PROPERTY(maintenance_constant_slow_twitch, double,
        "Maintenance constant for slow twitch fibers (W/kg, default is "
        "74.0).");
    OpenSim_DECLARE_PROPERTY(maintenance_constant_fast_twitch, double,
        "Maintenance constant for fast twitch fibers (W/kg, default is "
        "111.0).");

    OpenSim_DECLARE_SOCKET(muscle, Muscle,
            "The muscle to which the Bhargava2004SmoothedMuscleMetabolics is "
            "connected.");

    Bhargava2004SmoothedMuscleMetabolics_MuscleParameters();

    double getMuscleMass() const { return muscleMass; }
    void setMuscleMass();

    const Muscle& getMuscle() const { return getConnectee<Muscle>("muscle"); }

private:
    void constructProperties();
    mutable double muscleMass;
};

/** This class implements the metabolic energy model of Bhargava et al (2004)
and provides an option to use smooth (i.e., twice continuously
differentiable) approximations. These approximations might be better suited
for gradient-based optimization algorithms.

We propose two smooth implementations.

In the first implementation, conditional if statements were approximated by
using hyperbolic tangent functions (tanh). For example, the following if
statement:
<pre>     y = a, if x <= d </pre>
<pre>     y = b, if x > d </pre>
can be approximated by:
<pre>     f = 0.5 + 0.5 tanh(b(x-d)) </pre>
<pre>     y = a + (-a + b) f </pre>
where b is a parameter that determines the smoothness of the transition.

In the second implementation, conditional if statements were approximated
by using Huber loss functions, which have the following form:
<pre>     L(f(x)) = 0.5 f(x)^2, if f(x) <= delta </pre>
<pre>     L(f(x)) = delta(f(x) - 0.5 delta), otherwise. </pre>
The Huber loss function is quadratic for f(x) <= delta and linear
otherwise, with equal value and slopes of the different sections at the
points where f(x) = delta (https://en.wikipedia.org/wiki/Huber_loss). In
our implementation, we scaled this function with a parameter b that
determines the smootheness of the transition between the quadratic and
linear parts. Note that this approximation is piecewise but still
continuous.

The metabolic energy model includes components for activation heat rate,
maintenance heat rate, shortening heat rate, and mechanical work rate.

The shortening heat rate model differs between concentric contractions and
eccentric contractions. We smoothed the transition between both contraction
types using our smoothing functions. Note that when using the force
dependent shortening proportional constant, we only provide the tanh
smoothing option for approximating the shortening heat rate. This is
motivated by the fact that the shortening heat rate is defined by linear
functions but with different non-zero constants of proportionality for
concentric and eccentric contractions. It is therefore easier to smooth the
transition between both contraction types with a tanh function than with a
Huber loss function. The difference between the original (non-smooth) and the
smooth implementations is illustrated in the following figure:

\htmlonly <style>div.image img[src="SmoothShorteningHeatRate.png"]{width:750px;}</style> \endhtmlonly
@image html SmoothShorteningHeatRate.png "Curves produced using isometricTotalActiveForce=350, fiberForceTotal=250, velocity_smoothing=10"

The mechanical work rate model includes negative mechanical work rate
(i.e., work rate resulting from eccentric contraction) by default. However,
if specified by the user, the model only takes positive mechanical work
rate (i.e., work rate resulting from concentric contraction) into account.
In such case, we smoothed the transition between positive rate and zero
using our smoothing functions. The difference between the original
(non-smooth) and the smooth implementations is illustrated in the following
figure:

\htmlonly <style>div.image img[src="SmoothMechanicalWorkRate.png"]{width:750px;}</style> \endhtmlonly
@image html SmoothMechanicalWorkRate.png "Curves produced using fiber_force_active=250, velocity_smoothing=10"

The metabolic energy model implementation includes an optional clamping
that prevents the total metabolic rate (i.e., total metabolic power) to be
negative. This clamping is done by increasing the shortening heat rate. We
smoothed the transition between positive and negative total metabolic rate
using our smoothing functions. The difference between the original
(non-smooth) and the smooth implementations is illustrated in the following
figure:

\htmlonly <style>div.image img[src="ClampingTotalMetabolicRate.png"]{width:750px;}</style> \endhtmlonly
@image html ClampingTotalMetabolicRate.png "Curves produced using shorteningHeatRate=totalRate/4, power_smoothing=10"

The metabolic energy model implementation includes an optional clamping
(see Umberger et al (2003), page 104) that prevents the total heat rate
(i.e., activation heat rate + maintenance heat rate + shortening heat rate)
for a given muscle to fall below 1.0 W/kg. Note that, if active, this
clamping will cause the sum of the reported individual heat rates and work
rate to differ from the reported metabolic rate. We smoothed the transition
between total heat rate higher and lower than 1.0 W/kg using our smoothing
functions. The difference between the original (non-smooth) and the smooth
implementations is illustrated in the following figure:

\htmlonly <style>div.image img[src="ClampingTotalHeatRate.png"]{width:750px;}</style> \endhtmlonly
@image html ClampingTotalHeatRate.png "Curves produced using muscle_mass=0.4, heat_rate_smoothing=10"

Note that the maintenance heat rate implementation relies on a
PiecewiseLinearFunction. The first and second order derivatives of this
function can be evaluated but they are discontinuous. This might cause
issues with gradient-based optimization algorithms. Problems using this
discontinuous function have successfully converged; therefore, we have
included it in this implementation of the model.

You can enable smoothing via the `use_smoothing` property. The smoothing type
('tanh' or 'huber') can be chosen via the `smoothing_type` property, and the
level of smoothing can be controlled by the `velocity_smoothing`,
`power_smoothing`, and `heat_rate_smoothing` properties.

Muscles to be included when computing the total metabolic rate should be
specified using one of the three `addMuscle()` function overloads. See the
properties of `Bhargava2004SmoothedMuscleMetabolics_MuscleParameters()` for the
default parameter values used when not specified via the second or third
`addMuscle()` overload.

@code
Bhargava2004SmoothedMuscleMetabolics* metabolics =
    new Bhargava2004SmoothedMuscleMetabolics();
metabolics->setName("metabolics");
metabolics->set_use_smoothing(true);

// The simplest way to add the muscle to the metabolics model: just provide the
// name of the muscle and a reference to Muscle component.
metabolics->addMuscle("soleus_r", model.getComponent<Muscle>("soleus_r"));

// Provide the ratio of slow to fast twitch fibers and the specific tension of
// muscle when adding it to the metabolics model. The values shown are also the 
// default values.
double ratio_slow_twitch_fibers = 0.5;
double specific_tension = 0.25e6;
metabolics->addMuscle("gastroc_r", model.getComponent<Muscle>("gastroc_r"), 
        ratio_slow_twitch_fibers, specific_tension);

// Provide the slow and fast twitch fiber constants used to compute the 
// activation and maintenance heat rates. The values shown are also the default
// values.
double activation_constant_slow_twitch = 40.0;
double activation_constant_fast_twitch = 133.0;
double maintenance_constant_slow_twitch = 74.0;
double maintenance_constant_fast_twitch = 111.0;
metabolics->addMuscle("tibant_r", model.getComponent<Muscle>("tibant_r"),
        ratio_slow_twitch_fibers, specific_tension,
        activation_constant_slow_twitch, activation_constant_fast_twitch,
        maintenance_constant_slow_twitch, maintenance_constant_fast_twitch);

model.addComponent(metabolics);
model.finalizeConnections();
@endcode

The total metabolic rate output can be obtained using `getTotalMetabolicRate()`,
which takes a SimTK::State as a argument. You can similarly obtain the
individual heat rate and mechanical work rate components of the total metabolic
cost via `getTotalActivationRate()`, `getTotalMaintenanceRate()`,
`getTotalShorteningRate()`, and `getTotalMechanicalWorkRate()`. All outputs
require realizing the passed SimTK::State to SimTK::Stage::Dynamics.

@code
const auto& metabolics = 
    model.getComponent<Bhargava2004SmoothedMuscleMetabolics>("metabolics");
model.realizeDynamics(state);
double totalMetabolicRate = metabolics.getTotalMetabolicRate(state);
double activationHeatRate = metabolics.getTotalActivationRate(state);
@endcode

Bhargava et al. 2004: https://doi.org/10.1016/s0021-9290(03)00239-2 */
class OSIMSIMULATION_API Bhargava2004SmoothedMuscleMetabolics
        : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            Bhargava2004SmoothedMuscleMetabolics, ModelComponent);

public:
    OpenSim_DECLARE_PROPERTY(enforce_minimum_heat_rate_per_muscle, bool,
            "Specify whether the total heat rate for a muscle will be clamped "
            "to a minimum value of 1.0 W/kg (default is true). When set to "
            "true, the sum of the reported individual heat rates + work rate "
            "will not equal the reported total metabolic rate if the total "
            "heat rate falls below 1.0 W/kg.");
    OpenSim_DECLARE_PROPERTY(use_force_dependent_shortening_prop_constant,
            bool, "Specify whether to use a force dependent shortening "
            "proportionality constant (default is false).");
    OpenSim_DECLARE_PROPERTY(basal_coefficient, double, "Basal metabolic "
            "coefficient (default is 1.2).");
    OpenSim_DECLARE_PROPERTY(basal_exponent, double, "Basal metabolic "
            "exponent (default is 1).");
    OpenSim_DECLARE_PROPERTY(muscle_effort_scaling_factor, double,
            "Scale the excitation and activation values to compensate for "
            "solutions with excessive coactivation (e.g., when a suboptimal "
            "tracking strategy is used) (default is 1).");
    OpenSim_DECLARE_PROPERTY(include_negative_mechanical_work, bool,
            "Specify whether negative mechanical work will be included in "
            "mechanicalWorkRate (default is true).");
    OpenSim_DECLARE_PROPERTY(forbid_negative_total_power, bool,
            "Specify whether the total power for each muscle must remain  "
            "positive (default is true).");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(use_smoothing, bool,
            "An optional flag that allows the user to explicitly specify "
            "whether a smooth approximation of the metabolic energy model "
            "should be used (default is false).");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(smoothing_type, std::string,
            "An optional flag that allows the user to explicitly specify "
            "what type of smoothing to use ('tanh' or 'huber'; default is "
            "'tanh').");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(velocity_smoothing, double,
            "The parameter that determines the smoothness of the transition "
            "of the tanh or Huber loss function used to smooth the conditions "
            "related to contraction type (concentric or eccentric). Note that "
            "when computing the shortening heat rate while using the force "
            "dependent shortening proportionality constant, a tanh "
            "approximation is used even when using the Huber loss smoothing "
            "approach. The larger the steeper the transition but the worse "
            "for optimization (default is 10).");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(power_smoothing, double,
            "The parameter that determines the smoothness of the transition "
            "of the tanh or Huber loss function used to smooth the condition "
            "enforcing non-negative total power. The larger the steeper the "
            "transition but the worse for optimization (default is 10).");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(heat_rate_smoothing, double,
            "The parameter that determines the smoothness of the transition "
            "of the tanh or Huber loss function used to smooth the condition "
            "enforcing total heat rate larger than 1 (W/kg) for a give muscle "
            ". The larger the steeper the transition but the worse for "
            "optimization (default is 10).");
    OpenSim_DECLARE_LIST_PROPERTY(muscle_parameters,
            Bhargava2004SmoothedMuscleMetabolics_MuscleParameters,
            "Metabolic parameters for each muscle.");

    OpenSim_DECLARE_OUTPUT(total_metabolic_rate, double, getTotalMetabolicRate,
            SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(total_activation_rate, double,
            getTotalActivationRate, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(total_maintenance_rate, double,
            getTotalMaintenanceRate, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(total_shortening_rate, double,
            getTotalShorteningRate, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(total_mechanical_work_rate, double,
            getTotalMechanicalWorkRate, SimTK::Stage::Dynamics);

    OpenSim_DECLARE_LIST_OUTPUT(muscle_metabolic_rate, double,
            getMuscleMetabolicRate, SimTK::Stage::Dynamics);

    Bhargava2004SmoothedMuscleMetabolics();

    /** Get the number of muscles added to the metabolics model by one of the
    `addMuscle()` overloads. */
    int getNumMetabolicMuscles() const;

    /** Specify a muscle that should be included when computing the total
    metabolic rate. If the `muscle_mass` argument is not provided, it is
    estimated based on the max isometric force, specific tension,
    muscle density, and optimal fiber length. */
    void addMuscle(const std::string& name, const Muscle& muscle,
            double muscle_mass = SimTK::NaN);

    /** Specify a muscle that should be included when computing the total
    metabolic rate, as well as its ratio of slow to fast twitch fibers and
    specific tension. If the `muscle_mass` argument is not provided, it is
    estimated based on the max isometric force, specific tension,
    muscle density, and optimal fiber length. */
    void addMuscle(const std::string& name, const Muscle& muscle,
            double ratio_slow_twitch_fibers, double specific_tension,
            double muscle_mass = SimTK::NaN);

    /** Specify a muscle that should be included when computing the total
    metabolic rate, as well as its ratio of slow to fast twitch fibers and
    specific tension. This overload also allows you to specify the slow and fast
    twitch fiber constants used to compute the activation and maintenance heat
    rates. If the `muscle_mass` argument is not provided, it is estimated based
    on the max isometric force, specific tension, muscle density, and optimal
    fiber length. */
    void addMuscle(const std::string& name, const Muscle& muscle,
            double ratio_slow_twitch_fibers, double specific_tension,
            double activation_constant_slow_twitch,
            double activation_constant_fast_twitch,
            double maintenance_constant_slow_twitch,
            double maintenance_constant_fast_twitch,
            double muscle_mass = SimTK::NaN);

    double getTotalMetabolicRate(const SimTK::State& s) const;
    double getTotalActivationRate(const SimTK::State& s) const;
    double getTotalMaintenanceRate(const SimTK::State& s) const;
    double getTotalShorteningRate(const SimTK::State& s) const;
    double getTotalMechanicalWorkRate(const SimTK::State& s) const;
    double getMuscleMetabolicRate(
            const SimTK::State& s, const std::string& channel) const;

private:
    void constructProperties();
    void extendFinalizeFromProperties() override;
    void extendConnectToModel(Model& model) override;
    void extendRealizeTopology(SimTK::State&) const override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void calcMetabolicRateForCache(const SimTK::State& s) const;
    const SimTK::Vector& getMetabolicRate(const SimTK::State& s) const;
    const SimTK::Vector& getActivationRate(const SimTK::State& s) const;
    const SimTK::Vector& getMaintenanceRate(const SimTK::State& s) const;
    const SimTK::Vector& getShorteningRate(const SimTK::State& s) const;
    const SimTK::Vector& getMechanicalWorkRate(const SimTK::State& s) const;
    void calcMetabolicRate(const SimTK::State& s,
            SimTK::Vector& totalRatesForMuscles,
            SimTK::Vector& activationRatesForMuscles,
            SimTK::Vector& maintenanceRatesForMuscles,
            SimTK::Vector& shorteningRatesForMuscles,
            SimTK::Vector& mechanicalWorkRatesForMuscles) const;
    mutable std::unordered_map<std::string, int> m_muscleIndices;
    using ConditionalFunction =
            double(const double&, const double&, const double&, const double&,
                    const int&);
    PiecewiseLinearFunction m_fiberLengthDepCurve;
    mutable std::function<ConditionalFunction> m_conditional;
    mutable std::function<ConditionalFunction> m_tanh_conditional;
};

} // namespace OpenSim

#endif // BHARGAVA2004SMOOTHEDMUSCLEMETABOLICS_H
