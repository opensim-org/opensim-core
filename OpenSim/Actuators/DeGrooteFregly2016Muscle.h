#ifndef DEGROOTEFREGLY2016MUSCLE_H
#define DEGROOTEFREGLY2016MUSCLE_H
/* -------------------------------------------------------------------------- *
 *                 OpenSim:  DeGrooteFregly2016Muscle.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
 * Author(s): Christopher Dembia, Nicholas Bianco                             *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Actuators/osimActuatorsDLL.h>

#include <OpenSim/Common/DataTable.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Muscle.h>

namespace OpenSim {

// TODO avoid checking ignore_tendon_compliance() in each function;
//       might be slow.
// TODO prohibit fiber length from going below 0.2.

/** This muscle model was published in De Groote et al. 2016. 

The parameters of the active force-length and force-velocity curves have
been slightly modified from what was published to ensure the curves go
through key points:
- Active force-length curve goes through (1, 1).
- Force-velocity curve goes through (-1, 0) and (0, 1).
The default tendon force curve parameters are modified from that in De
Groote et al., 2016: the curve is parameterized by the strain at 1 norm
force (rather than "kT"), and the default value for this parameter is
0.049 (same as in TendonForceLengthCurve) rather than 0.0474.

This implementation introduces the property 'active_force_width_scale' as 
an addition to the original model, which allows users to effectively make 
the active force-length curve wider. This property may be useful for 
improving the force-generating capacity of a muscle without increasing 
maximum isometric force. This property works by scaling the normalized
fiber length when the active force-length curve is computed. For example, 
a scale factor of 2 means that the fiber muscle traverses half as far 
along the force-length curve in either direction.

This implementation adds fiber damping as an addition to the original model. 
Users can specify this via the 'fiber_damping' property, and damping force
along the fiber is computed by multiplying the property value by the 
normalized fiber velocity and max isometric force. If using this muscle for
optimization, fiber damping is recommended as it can improve convergence.

@note If converting from Thelen2003Muscles via replaceMuscles(), fiber 
   damping will be set to zero since there is no damping in that muscle
   model.

This class supports tendon compliance dynamics in both explicit and implicit 
form (formulations 1 and 3 from De Groote et al. 2016). Both forms of the 
dynamics use normalized tendon force as the state variable (rather than the 
typical fiber length state). The explicit form is handled through the usual 
Component dynamics interface. The implicit form introduces an additional 
discrete state variable and cache variable in the SimTK::State for the 
derivative of normalized tendon force and muscle-tendon equilibrium residual 
respectively. In general, it is preferable to use the implicit form in 
optimization since it can be robust to arbitrary initial guesses (see De 
Groote et al. 2016). However, the implicit form is only for use with solvers 
that support implicit dynamics (i.e. Moco) and cannot be used to perform a 
time-stepping forward simulation with Manager; use explicit mode for 
time-stepping.

@section property Property Bounds
The acceptable bounds for each property are enforced at model initialization.
These bounds are:
 - activation_time_constant: (0, inf]
 - deactivation_time_constant: (0, inf]
 - active_force_width_scale: [1, inf]
 - fiber_damping: [0, inf]
 - passive_fiber_strain_at_one_norm_force: (0, inf]
 - tendon_strain_at_one_norm_force: (0, inf]
 - pennation_angle_at_optimal: [0, Pi/2)
 - default_activation: (0, inf]
 - default_normalized_tendon_force: [0, 5]

@note The methods getMinNormalizedTendonForce() and
   getMaxNormalizedTendonForce() provide these bounds for use in custom solvers.

@note Muscle properties can be optimized using MocoParameter. The acceptable
bounds for each property are **not** enforced during parameter optimization, so
the user must supply these bounds to MocoParameter.

@note The properties `default_activation` and `default_normalized_tendon_force`
cannot be optimized because they are applied during model initialization only.

@section departures Departures from the Muscle base class

The documentation for Muscle::MuscleLengthInfo states that the
optimalFiberLength of a muscle is also its resting length, but this is not
true for this muscle: there is a non-zero passive fiber force at the
optimal fiber length.

In the Muscle class, setIgnoreTendonCompliance() and
setIngoreActivationDynamics() control modeling options, meaning these
settings could theoretically be changed. However, for this class, the
modeling option is ignored and the values of the ignore_tendon_compliance
and ignore_activation_dynamics properties are used directly.

De Groote, F., Kinney, A. L., Rao, A. V., & Fregly, B. J. (2016). Evaluation
of Direct Collocation Optimal Control Problem Formulations for Solving the
Muscle Redundancy Problem. Annals of Biomedical Engineering, 44(10), 1–15.
http://doi.org/10.1007/s10439-016-1591-9 */
class OSIMACTUATORS_API DeGrooteFregly2016Muscle : public Muscle {
    OpenSim_DECLARE_CONCRETE_OBJECT(DeGrooteFregly2016Muscle, Muscle);

public:
    OpenSim_DECLARE_PROPERTY(activation_time_constant, double,
            "Smaller value means activation can increase more rapidly. "
            "Default: 0.015 seconds. Bounds: (0, inf]");
    OpenSim_DECLARE_PROPERTY(deactivation_time_constant, double,
            "Smaller value means activation can decrease more rapidly. "
            "Default: 0.060 seconds. Bounds: (0, inf]");
    OpenSim_DECLARE_PROPERTY(default_activation, double,
            "Value of activation in the default state returned by "
            "initSystem(). Default: 0.5. Bounds: (0, inf]");
    OpenSim_DECLARE_PROPERTY(default_normalized_tendon_force, double,
            "Value of normalized tendon force in the default state returned by "
            "initSystem(). Default: 0.5. Bounds: [0, 5].");
    OpenSim_DECLARE_PROPERTY(active_force_width_scale, double,
            "Scale factor for the width of the active force-length curve. "
            "Larger values make the curve wider. Default: 1.0. Bounds: [1, inf]");
    OpenSim_DECLARE_PROPERTY(fiber_damping, double,
            "Use this property to define the linear damping force that is "
            "added to the total muscle fiber force. It is computed by "
            "multiplying this damping parameter by the normalized fiber "
            "velocity and the max isometric force. Default: 0. Bounds: [0, inf]");
    OpenSim_DECLARE_PROPERTY(ignore_passive_fiber_force, bool,
            "Make the passive fiber force 0. Default: false.");
    OpenSim_DECLARE_PROPERTY(passive_fiber_strain_at_one_norm_force, double,
            "Fiber strain when the passive fiber force is 1 normalized force. "
            "Default: 0.6. Bounds: (0, inf]");
    OpenSim_DECLARE_PROPERTY(tendon_strain_at_one_norm_force, double,
            "Tendon strain at a tension of 1 normalized force. "
            "Default: 0.049. Bounds: (0, inf]");
    OpenSim_DECLARE_PROPERTY(tendon_compliance_dynamics_mode, std::string,
            "The dynamics method used to enforce tendon compliance dynamics. "
            "Options: 'explicit' or 'implicit'. Default: 'explicit'. ");

    OpenSim_DECLARE_OUTPUT(passive_fiber_elastic_force, double,
            getPassiveFiberElasticForce, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(passive_fiber_elastic_force_along_tendon, double,
            getPassiveFiberElasticForceAlongTendon, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(passive_fiber_damping_force, double,
            getPassiveFiberDampingForce, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(passive_fiber_damping_force_along_tendon, double,
            getPassiveFiberDampingForceAlongTendon, SimTK::Stage::Dynamics);

    OpenSim_DECLARE_OUTPUT(implicitresidual_normalized_tendon_force, double,
            getImplicitResidualNormalizedTendonForce, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(implicitenabled_normalized_tendon_force, bool,
            getImplicitEnabledNormalizedTendonForce, SimTK::Stage::Model);

    OpenSim_DECLARE_OUTPUT(statebounds_normalized_tendon_force, SimTK::Vec2,
            getBoundsNormalizedTendonForce, SimTK::Stage::Model);

    DeGrooteFregly2016Muscle() { constructProperties(); }


protected:
    //--------------------------------------------------------------------------
    // COMPONENT INTERFACE
    //--------------------------------------------------------------------------
    /// @name Component interface
    /// @{
    void extendFinalizeFromProperties() override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendInitStateFromProperties(SimTK::State& s) const override;
    void extendSetPropertiesFromState(const SimTK::State& s) override;
    void computeStateVariableDerivatives(const SimTK::State& s) const override;
    /// @}

    //--------------------------------------------------------------------------
    // ACTUATOR INTERFACE
    //--------------------------------------------------------------------------
    /// @name Actuator interface
    /// @{
    double computeActuation(const SimTK::State& s) const override;
    /// @}

public:
    //--------------------------------------------------------------------------
    // MUSCLE INTERFACE
    //--------------------------------------------------------------------------
    /// @name Muscle interface
    /// @{

    /// If ignore_activation_dynamics is true, this gets excitation instead.
    double getActivation(const SimTK::State& s) const override {
        // We override the Muscle's implementation because Muscle requires
        // realizing to Dynamics to access activation from MuscleDynamicsInfo,
        // which is unnecessary if the activation is a state.
        if (get_ignore_activation_dynamics()) {
            return getControl(s);
        } else {
            return getStateVariableValue(s, STATE_ACTIVATION_NAME);
        }
    }

    /// If ignore_activation_dynamics is true, this sets excitation instead.
    void setActivation(SimTK::State& s, double activation) const override {
        if (get_ignore_activation_dynamics()) {
            SimTK::Vector& controls(getModel().updControls(s));
            setControls(SimTK::Vector(1, activation), controls);
            getModel().setControls(s, controls);
        } else {
            setStateVariableValue(s, STATE_ACTIVATION_NAME, activation);
        }
        markCacheVariableInvalid(s, "velInfo");
        markCacheVariableInvalid(s, "dynamicsInfo");
    }

protected:
    double calcInextensibleTendonActiveFiberForce(
        SimTK::State&, double) const override;
    void calcMuscleLengthInfo(
            const SimTK::State& s, MuscleLengthInfo& mli) const override;
    void calcFiberVelocityInfo(
            const SimTK::State& s, FiberVelocityInfo& fvi) const override;
    void calcMuscleDynamicsInfo(
            const SimTK::State& s, MuscleDynamicsInfo& mdi) const override;
    void calcMusclePotentialEnergyInfo(const SimTK::State& s,
            MusclePotentialEnergyInfo& mpei) const override;

public:
    /// In this method, calcEquilibriumResidual() is used to find a value of the
    /// normalized tendon force state variable that produces muscle-tendon
    /// equilibrium. This relies on the implicit form of tendon compliance since
    /// the explicit form uses the normalized tendon force state variable
    /// directly to compute fiber force, which always produces a zero
    /// muscle-tendon equilibrium residual. The derivative of normalized tendon
    /// force is set to zero since a value is required for the implicit form of
    /// the model.  
    void computeInitialFiberEquilibrium(SimTK::State& s) const override;
    /// @}

    /// @name Get methods.
    /// @{

    /// Get the portion of the passive fiber force generated by the elastic
    /// element only (N).
    double getPassiveFiberElasticForce(const SimTK::State& s) const;
    /// Get the portion of the passive fiber force generated by the elastic
    /// element only, projected onto the tendon direction (N).
    double getPassiveFiberElasticForceAlongTendon(const SimTK::State& s) const;
    /// Get the portion of the passive fiber force generated by the damping
    /// element only (N).
    double getPassiveFiberDampingForce(const SimTK::State& s) const;
    /// Get the portion of the passive fiber force generated by the damping
    /// element only, projected onto the tendon direction (N).
    double getPassiveFiberDampingForceAlongTendon(const SimTK::State& s) const;

    /// Get whether fiber dynamics is in implicit dynamics mode when using 
    /// normalized tendon force as the state. This is useful to indicate to 
    /// solvers to handle the normalized tendon force derivative and 
    /// muscle-tendon equilibrium variables, which are added to the State as 
    /// discrete and cache variables, respectively.
    /// This function is intended primarily for the model Output 
    /// 'implicitenabled_normalized_tendon_force'. We don't need the state, but 
    /// the state parameter is a requirement of Output functions.
    bool getImplicitEnabledNormalizedTendonForce(const SimTK::State&) const {
        return !get_ignore_tendon_compliance() && !m_isTendonDynamicsExplicit;
    }
    /// Compute the muscle-tendon force equilibrium residual value when using
    /// implicit contraction dynamics with normalized tendon force as the
    /// state.
    /// This function is intended primarily for the model Output 
    /// 'implicitresidual_normalized_tendon_force'. 
    double getImplicitResidualNormalizedTendonForce(
            const SimTK::State& s) const;

    /// If ignore_tendon_compliance is true, this gets normalized fiber force
    /// along the tendon instead.
    double getNormalizedTendonForce(const SimTK::State& s) const {
        if (get_ignore_tendon_compliance()) {
            return getTendonForce(s) / get_max_isometric_force();
        } else {
            return getStateVariableValue(s, STATE_NORMALIZED_TENDON_FORCE_NAME);
        }
    }

    /// Obtain the time derivative of the normalized tendon force.
    /// - If ignore_tendon_compliance is false, this returns zero.
    /// - If tendon_compliance_dynamics_mode is 'implicit', this gets the
    /// discrete variable normalized tendon force derivative value.
    /// - If tendon_compliance_dynamics_mode is 'explicit', this gets the value
    /// returned by getStateVariableDerivativeValue() for the
    /// 'normalized_tendon_force' state.
    double getNormalizedTendonForceDerivative(const SimTK::State& s) const {
        if (get_ignore_tendon_compliance()) { return 0.0; }

        if (m_isTendonDynamicsExplicit) {
            return getStateVariableDerivativeValue(
                s, STATE_NORMALIZED_TENDON_FORCE_NAME);
        } else {
            return getDiscreteVariableValue(
                s, DERIVATIVE_NORMALIZED_TENDON_FORCE_NAME);
        }
    }

    /// @copydoc calcEquilibriumResidual()
    /// This calls calcEquilibriumResidual() using values from the provided 
    /// SimTK::State as arguments. While is computed using implicit mode, the 
    /// value of normalized tendon force derivative used *is* consistent with 
    /// the property `tendon_compliance_dynamics_mode` (see
    /// getNormalizedTendonForceDerivative()).
    double getEquilibriumResidual(const SimTK::State& s) const {
        return calcEquilibriumResidual(getLength(s), getLengtheningSpeed(s),
                getActivation(s), getNormalizedTendonForce(s),
                getNormalizedTendonForceDerivative(s));
    }

    /// @copydoc calcLinearizedEquilibriumResidualDerivative()
    double getLinearizedEquilibriumResidualDerivative(
            const SimTK::State& s) const {
        return calcLinearizedEquilibriumResidualDerivative(getLength(s),
                getLengtheningSpeed(s), getActivation(s),
                getNormalizedTendonForce(s),
                getNormalizedTendonForceDerivative(s));
    }

    static std::string getActivationStateName() {
        return STATE_ACTIVATION_NAME;
    }
    static std::string getNormalizedTendonForceStateName() {
        return STATE_NORMALIZED_TENDON_FORCE_NAME;
    }
    static std::string getImplicitDynamicsDerivativeName() {
        return DERIVATIVE_NORMALIZED_TENDON_FORCE_NAME;
    }
    static std::string getImplicitDynamicsResidualName() {
        return RESIDUAL_NORMALIZED_TENDON_FORCE_NAME;
    }
    static double getMinNormalizedTendonForce() { return m_minNormTendonForce; }
    static double getMaxNormalizedTendonForce() { return m_maxNormTendonForce; }
    /// The first element of the Vec2 is the lower bound, and the second is the
    /// upper bound.
    /// This function is intended primarily for the model Output 
    /// 'statebounds_normalized_tendon_force'. We don't need the state, but the 
    /// state parameter is a requirement of Output functions.
    SimTK::Vec2 getBoundsNormalizedTendonForce(const SimTK::State&) const
    { return {getMinNormalizedTendonForce(), getMaxNormalizedTendonForce()}; }

    static double getMinNormalizedFiberLength() { return m_minNormFiberLength; }
    static double getMaxNormalizedFiberLength() { return m_maxNormFiberLength; }
    /// The first element of the Vec2 is the lower bound, and the second is the
    /// upper bound.
    /// Note that since fiber length is not used as a state variable, these
    /// bounds cannot be enforced directly. It is upon the user to ensure the
    /// muscle fiber is operating within the specified domain.
    SimTK::Vec2 getBoundsNormalizedFiberLength() const {
        return {getMinNormalizedTendonForce(), getMaxNormalizedTendonForce()};
    }
    /// @}

    /// @name Set methods.
    /// @{
    /// If ignore_tendon_compliance is true, this sets nothing.
    void setNormalizedTendonForce(
            SimTK::State& s, double normTendonForce) const {
        if (!get_ignore_tendon_compliance()) {
            setStateVariableValue(
                    s, STATE_NORMALIZED_TENDON_FORCE_NAME, normTendonForce);
            markCacheVariableInvalid(s, "lengthInfo");
            markCacheVariableInvalid(s, "velInfo");
            markCacheVariableInvalid(s, "dynamicsInfo");
        }
    }
    /// @}

    /// @name Calculation methods.
    /// These functions compute the values of normalized/dimensionless curves,
    /// their derivatives and integrals, and other quantities of the muscle.
    /// These do not depend on a SimTK::State.
    /// @{

    /// The active force-length curve is the sum of 3 Gaussian-like curves. The
    /// width of the curve can be adjusted via the 'active_force_width_scale'
    /// property.
    SimTK::Real calcActiveForceLengthMultiplier(
            const SimTK::Real& normFiberLength) const {
        const double& scale = get_active_force_width_scale();
        // Shift the curve so its peak is at the origin, scale it
        // horizontally, then shift it back so its peak is still at x = 1.0.
        const double x = (normFiberLength - 1.0) / scale + 1.0;
        return calcGaussianLikeCurve(x, b11, b21, b31, b41) +
               calcGaussianLikeCurve(x, b12, b22, b32, b42) +
               calcGaussianLikeCurve(x, b13, b23, b33, b43);
    }

    /// The derivative of the active force-length curve with respect to
    /// normalized fiber length. This curve is based on the derivative of the
    /// Gaussian-like curve used in calcActiveForceLengthMultiplier(). The
    /// 'active_force_width_scale' property also affects the value of the
    /// derivative curve.
    SimTK::Real calcActiveForceLengthMultiplierDerivative(
            const SimTK::Real& normFiberLength) const {
        const double& scale = get_active_force_width_scale();
        // Shift the curve so its peak is at the origin, scale it
        // horizontally, then shift it back so its peak is still at x = 1.0.
        const double x = (normFiberLength - 1.0) / scale + 1.0;
        return (1.0 / scale) *
               (calcGaussianLikeCurveDerivative(x, b11, b21, b31, b41) +
                       calcGaussianLikeCurveDerivative(x, b12, b22, b32, b42) +
                       calcGaussianLikeCurveDerivative(x, b13, b23, b33, b43));
    }

    /// The parameters of this curve are not modifiable, so this function is
    /// static.
    /// Domain: [-1, 1]
    /// Range: [0, 1.794]
    /// @note It is upon the user to check that the muscle fiber is acting 
    ///       within the specified domain. Force computations outside this range
    ///       may be incorrect.
    static SimTK::Real calcForceVelocityMultiplier(
            const SimTK::Real& normFiberVelocity) {
        using SimTK::square;
        const SimTK::Real tempV = d2 * normFiberVelocity + d3;
        const SimTK::Real tempLogArg = tempV + sqrt(square(tempV) + 1.0);
        return d1 * log(tempLogArg) + d4;
    }

    /// This is the inverse of the force-velocity multiplier function, and
    /// returns the normalized fiber velocity (in [-1, 1]) as a function of
    /// the force-velocity multiplier.
    static SimTK::Real calcForceVelocityInverseCurve(
            const SimTK::Real& forceVelocityMult) {
        // The version of this equation in the supplementary materials of De
        // Groote et al., 2016 has an error (it's missing a "-d3" before
        // dividing by "d2").
        return (sinh(1.0 / d1 * (forceVelocityMult - d4)) - d3) / d2;
    }

    /// This is the passive force-length curve. The curve becomes negative below
    /// the minNormFiberLength.
    ///
    /// We modified this equation from that in the supplementary materials of De
    /// Groote et al., 2016, which is the same function used in
    /// Thelen2003Muscle. The version in the supplementary materials passes
    /// through y = 0 at x = 1.0 and allows for negative forces. We do not want
    /// negative forces within the allowed range of fiber lengths, so we
    /// modified the equation to pass through y = 0 at x = minNormFiberLength. 
    /// (This is not an issue for Thelen2003Muscle because the curve is not 
    /// smooth and returns 0 for lengths less than optimal fiber length.)
    SimTK::Real calcPassiveForceMultiplier(
            const SimTK::Real& normFiberLength) const {
        if (get_ignore_passive_fiber_force()) return 0;

        const double& e0 = get_passive_fiber_strain_at_one_norm_force();

        const double offset =
                exp(kPE * (m_minNormFiberLength - 1.0) / e0);
        const double denom = exp(kPE) - offset;

        return (exp(kPE * (normFiberLength - 1.0) / e0) - offset) / denom;
    }

    /// This is the derivative of the passive force-length curve with respect to
    /// the normalized fiber length.
    SimTK::Real calcPassiveForceMultiplierDerivative(
            const SimTK::Real& normFiberLength) const {
        if (get_ignore_passive_fiber_force()) return 0;

        const double& e0 = get_passive_fiber_strain_at_one_norm_force();

        const double offset = exp(kPE * (m_minNormFiberLength - 1) / e0);

        return (kPE * exp((kPE * (normFiberLength - 1)) / e0)) /
               (e0 * (exp(kPE) - offset));
    }

    /// This is the integral of the passive force-length curve with respect to
    /// the normalized fiber length over the domain
    /// [minNormFiberLength normFiberLength], where minNormFiberLength is the
    /// value return by getMinNormalizedFiberLength().
    SimTK::Real calcPassiveForceMultiplierIntegral(
            const SimTK::Real& normFiberLength) const {
        if (get_ignore_passive_fiber_force()) return 0;

        const double& e0 = get_passive_fiber_strain_at_one_norm_force();

        const double temp1 = 
                e0 + kPE * normFiberLength - kPE * m_minNormFiberLength;
        const double temp2 = exp(kPE * (normFiberLength - 1.0) / e0);
        const double temp3 = exp(kPE * (m_minNormFiberLength - 1.0) / e0);
        const double numer = exp(kPE) * temp1 - e0 * temp2;
        const double denom = kPE * (temp3 - exp(kPE));
        return (temp1 / kPE) + (numer / denom);
    }

    /// The normalized tendon force as a function of normalized tendon length.
    /// Note that this curve does not go through (1, 0); when
    /// normTendonLength=1, this function returns a slightly negative number.
    // TODO: In explicit mode, do not allow negative tendon forces?
    SimTK::Real calcTendonForceMultiplier(
            const SimTK::Real& normTendonLength) const {
        return c1 * exp(getTendonStiffnessParameter() * (normTendonLength - c2)) - c3;
    }

    /// This is the derivative of the tendon-force length curve with respect to
    /// normalized tendon length.
    SimTK::Real calcTendonForceMultiplierDerivative(
            const SimTK::Real& normTendonLength) const {
        return c1 * getTendonStiffnessParameter() * exp(getTendonStiffnessParameter() * (normTendonLength - c2));
    }

    /// This is the integral of the tendon-force length curve with respect to
    /// normalized tendon length over the domain
    /// [minNormTendonLength normTendonLength]. The lower bound on the domain
    /// is computed by passing the value return by getMinNormalizedTendonForce()
    /// to calcTendonForceLengthInverseCurve(). 
    SimTK::Real calcTendonForceMultiplierIntegral(
            const SimTK::Real& normTendonLength) const {
        const double minNormTendonLength =
                calcTendonForceLengthInverseCurve(m_minNormTendonForce);
        const double temp1 =
                exp(getTendonStiffnessParameter() * normTendonLength)
                - exp(getTendonStiffnessParameter() * minNormTendonLength);
        const double temp2 = c1 * exp(-c2 * getTendonStiffnessParameter())
                                / getTendonStiffnessParameter();
        const double temp3 = c3 * (normTendonLength - minNormTendonLength);
        return temp1 * temp2 - temp3;
    }

    /// This is the inverse of the tendon force-length curve, and returns the
    /// normalized tendon length as a function of the normalized tendon force.
    SimTK::Real calcTendonForceLengthInverseCurve(
            const SimTK::Real& normTendonForce) const {
        return log((1.0 / c1) * (normTendonForce + c3))
                / getTendonStiffnessParameter() + c2;
    }

    /// This returns normalized tendon velocity given the derivative of 
    /// normalized tendon force and normalized tendon length. This is derived
    /// by taking the derivative of the tendon force multiplier curve with 
    /// respect to time and then solving for normalized fiber velocity (see
    /// supplementary information for De Groote et al. 2016).
    SimTK::Real calcTendonForceLengthInverseCurveDerivative(
            const SimTK::Real& derivNormTendonForce,
            const SimTK::Real& normTendonLength) const {
        return derivNormTendonForce / (c1 * getTendonStiffnessParameter()
            * exp(getTendonStiffnessParameter() * (normTendonLength - c2)));
    }

    /// This computes both the total fiber force and the individual components
    /// of fiber force (active, conservative passive, and non-conservative
    /// passive).
    /// @note based on Millard2012EquilibriumMuscle::calcFiberForce().
    void calcFiberForce(const SimTK::Real& activation,
            const SimTK::Real& activeForceLengthMultiplier,
            const SimTK::Real& forceVelocityMultiplier,
            const SimTK::Real& normPassiveFiberForce,
            const SimTK::Real& normFiberVelocity, 
            SimTK::Real& activeFiberForce,
            SimTK::Real& conPassiveFiberForce,
            SimTK::Real& nonConPassiveFiberForce,
            SimTK::Real& totalFiberForce) const {
        const auto& maxIsometricForce = get_max_isometric_force();
        // active force
        activeFiberForce =
                maxIsometricForce * (activation * activeForceLengthMultiplier *
                                            forceVelocityMultiplier);
        // conservative passive force
        conPassiveFiberForce = maxIsometricForce * normPassiveFiberForce;
        // non-conservative passive force
        nonConPassiveFiberForce =
                maxIsometricForce * get_fiber_damping() * normFiberVelocity;
        // total force
        totalFiberForce = activeFiberForce + conPassiveFiberForce +
                          nonConPassiveFiberForce;
    }

    /// The stiffness of the fiber in the direction of the fiber. This includes
    /// both active and passive force contributions to stiffness from the muscle
    /// fiber.
    /// @note based on Millard2012EquilibriumMuscle::calcFiberStiffness().
    SimTK::Real calcFiberStiffness(const SimTK::Real& activation,
            const SimTK::Real& normFiberLength,
            const SimTK::Real& fiberVelocityMultiplier) const {

        const SimTK::Real partialNormFiberLengthPartialFiberLength =
                1.0 / get_optimal_fiber_length();
        const SimTK::Real partialNormActiveForcePartialFiberLength =
                partialNormFiberLengthPartialFiberLength *
                calcActiveForceLengthMultiplierDerivative(normFiberLength);
        const SimTK::Real partialNormPassiveForcePartialFiberLength =
                partialNormFiberLengthPartialFiberLength *
                calcPassiveForceMultiplierDerivative(normFiberLength);

        // fiberStiffness = d_fiberForce / d_fiberLength
        return get_max_isometric_force() *
               (activation * partialNormActiveForcePartialFiberLength *
                               fiberVelocityMultiplier +
                       partialNormPassiveForcePartialFiberLength);
    }

    /// The stiffness of the tendon in the direction of the tendon.
    /// @note based on Millard2012EquilibriumMuscle.
    SimTK::Real calcTendonStiffness(const SimTK::Real& normTendonLength) const {

        if (get_ignore_tendon_compliance()) return SimTK::Infinity;
        return (get_max_isometric_force() / get_tendon_slack_length()) *
               calcTendonForceMultiplierDerivative(normTendonLength);
    }

    /// The stiffness of the whole musculotendon unit in the direction of the
    /// tendon.
    /// @note based on Millard2012EquilibriumMuscle.
    SimTK::Real calcMuscleStiffness(const SimTK::Real& tendonStiffness,
            const SimTK::Real& fiberStiffnessAlongTendon) const {

        if (get_ignore_tendon_compliance()) return fiberStiffnessAlongTendon;
        // TODO Millard2012EquilibriumMuscle includes additional checks that
        // the stiffness is non-negative and that the denominator is non-zero.
        // Checks are omitted here to preserve continuity and smoothness for
        // optimization (see #3685).
        return (fiberStiffnessAlongTendon * tendonStiffness) /
               (fiberStiffnessAlongTendon + tendonStiffness);
    }

    virtual double calcMuscleStiffness(const SimTK::State& s) const override
    {
        const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
        return calcMuscleStiffness(
                mdi.tendonStiffness,
                mdi.fiberStiffnessAlongTendon);
    }

    /// The derivative of pennation angle with respect to fiber length.
    /// @note based on
    /// MuscleFixedWidthPennationModel::calc_DPennationAngle_DFiberLength().
    SimTK::Real calcPartialPennationAnglePartialFiberLength(
            const SimTK::Real& fiberLength) const {

        using SimTK::square;
        // pennationAngle = asin(fiberWidth/fiberLength)
        // d_pennationAngle/d_fiberLength =
        //          d/d_fiberLength (asin(fiberWidth/fiberLength))
        return (-getFiberWidth() / square(fiberLength)) /
               sqrt(1.0 - square(getFiberWidth() / fiberLength));
    }

    /// The derivative of the fiber force along the tendon with respect to fiber
    /// length.
    /// @note based on
    /// Millard2012EquilibriumMuscle::calc_DFiberForceAT_DFiberLength().
    SimTK::Real calcPartialFiberForceAlongTendonPartialFiberLength(
            const SimTK::Real& fiberForce, const SimTK::Real& fiberStiffness,
            const SimTK::Real& sinPennationAngle,
            const SimTK::Real& cosPennationAngle,
            const SimTK::Real& partialPennationAnglePartialFiberLength) const {

        const SimTK::Real partialCosPennationAnglePartialFiberLength =
                -sinPennationAngle * partialPennationAnglePartialFiberLength;

        // The stiffness of the fiber along the direction of the tendon. For
        // small changes in length parallel to the fiber, this quantity is
        // d_fiberForceAlongTendon / d_fiberLength =
        //      d/d_fiberLength(fiberForce * cosPennationAngle)
        return fiberStiffness * cosPennationAngle +
               fiberForce * partialCosPennationAnglePartialFiberLength;
    }

    /// The derivative of the fiber force along the tendon with respect to the
    /// fiber length along the tendon.
    /// @note based on
    /// Millard2012EquilibriumMuscle::calc_DFiberForceAT_DFiberLengthAT.
    SimTK::Real calcFiberStiffnessAlongTendon(const SimTK::Real& fiberLength,
            const SimTK::Real& partialFiberForceAlongTendonPartialFiberLength,
            const SimTK::Real& sinPennationAngle,
            const SimTK::Real& cosPennationAngle,
            const SimTK::Real& partialPennationAnglePartialFiberLength) const {

        // The change in length of the fiber length along the tendon.
        // fiberLengthAlongTendon = fiberLength * cosPennationAngle
        const SimTK::Real partialFiberLengthAlongTendonPartialFiberLength =
                cosPennationAngle -
                fiberLength * sinPennationAngle *
                        partialPennationAnglePartialFiberLength;

        // fiberStiffnessAlongTendon
        //    = d_fiberForceAlongTendon / d_fiberLengthAlongTendon
        //    = (d_fiberForceAlongTendon / d_fiberLength) *
        //      (1 / (d_fiberLengthAlongTendon / d_fiberLength))
        return partialFiberForceAlongTendonPartialFiberLength *
               (1.0 / partialFiberLengthAlongTendonPartialFiberLength);
    }

    SimTK::Real calcPartialTendonLengthPartialFiberLength(
            const SimTK::Real& fiberLength,
            const SimTK::Real& sinPennationAngle,
            const SimTK::Real& cosPennationAngle,
            const SimTK::Real& partialPennationAnglePartialFiberLength) const {

        return fiberLength * sinPennationAngle *
                       partialPennationAnglePartialFiberLength -
               cosPennationAngle;
    }

    SimTK::Real calcPartialTendonForcePartialFiberLength(
            const SimTK::Real& tendonStiffness, const SimTK::Real& fiberLength, 
            const SimTK::Real& sinPennationAngle, 
            const SimTK::Real& cosPennationAngle) const {
        const SimTK::Real partialPennationAnglePartialFiberLength =
                calcPartialPennationAnglePartialFiberLength(fiberLength);

        const SimTK::Real partialTendonLengthPartialFiberLength =
                calcPartialTendonLengthPartialFiberLength(fiberLength, 
                    sinPennationAngle, cosPennationAngle,
                        partialPennationAnglePartialFiberLength);

        return tendonStiffness * partialTendonLengthPartialFiberLength;
    }

    /// The residual (i.e. error) in the muscle-tendon equilibrium equation:
    ///     residual = normTendonForce - normFiberForce * cosPennationAngle
    /// The residual is unitless (units of normalized force).
    /// This is computed using the muscle in implicit mode, since explicit mode
    /// uses the normalized tendon force state variable directly
    /// to compute fiber force, which always produces a zero muscle-tendon
    /// equilibrium residual. 
    SimTK::Real calcEquilibriumResidual(const SimTK::Real& muscleTendonLength,
            const SimTK::Real& muscleTendonVelocity,
            const SimTK::Real& activation, const SimTK::Real& normTendonForce,
            const SimTK::Real& normTendonForceDerivative) const {

        MuscleLengthInfo mli;
        FiberVelocityInfo fvi;
        MuscleDynamicsInfo mdi;
        calcMuscleLengthInfoHelper(
                muscleTendonLength, false, mli, normTendonForce);
        calcFiberVelocityInfoHelper(muscleTendonVelocity, activation, false,
                false, mli, fvi, normTendonForce, normTendonForceDerivative);
        calcMuscleDynamicsInfoHelper(activation, false, mli, fvi, mdi,
                normTendonForce);

        return mdi.normTendonForce -
               mdi.fiberForceAlongTendon / get_max_isometric_force();
    }

    /// The residual (i.e. error) in the time derivative of the linearized
    /// muscle-tendon equilibrium equation (Millard et al. 2013, equation A6):
    ///     residual = fiberStiffnessAlongTendon * fiberVelocityAlongTendon -
    ///                tendonStiffness *
    ///                    (muscleTendonVelocity - fiberVelocityAlongTendon)
    /// This may be useful for finding equilibrium when there is velocity in the
    /// muscle-tendon actuator. Velocity is divided between the muscle and
    /// tendon based on their relative stiffnesses. 
    SimTK::Real calcLinearizedEquilibriumResidualDerivative(
            const SimTK::Real& muscleTendonLength,
            const SimTK::Real& muscleTendonVelocity,
            const SimTK::Real& activation, const SimTK::Real& normTendonForce,
            const SimTK::Real& normTendonForceDerivative) const {

        MuscleLengthInfo mli;
        FiberVelocityInfo fvi;
        MuscleDynamicsInfo mdi;
        calcMuscleLengthInfoHelper(
                muscleTendonLength, false, mli, normTendonForce);
        calcFiberVelocityInfoHelper(muscleTendonVelocity, activation, false,
                m_isTendonDynamicsExplicit, mli, fvi, normTendonForce,
                normTendonForceDerivative);
        calcMuscleDynamicsInfoHelper(activation, false, mli, fvi, mdi,
                normTendonForce);

        return mdi.fiberStiffnessAlongTendon * fvi.fiberVelocityAlongTendon -
               mdi.tendonStiffness *
                       (muscleTendonVelocity - fvi.fiberVelocityAlongTendon);
    }
    /// @}

    /// @name Utilities
    /// @{

    /// Export the active force-length multiplier and passive force multiplier
    /// curves to a DataTable. If the normFiberLengths argument is omitted, we
    /// use createVectorLinspace(200, minNormFiberLength, maxNormFiberLength).
    DataTable exportFiberLengthCurvesToTable(
            const SimTK::Vector& normFiberLengths = SimTK::Vector()) const;
    /// Export the fiber force-velocity multiplier curve to a DataTable. If
    /// the normFiberVelocities argument is omitted, we use
    /// createVectorLinspace(200, -1.1, 1.1).
    DataTable exportFiberVelocityMultiplierToTable(
            const SimTK::Vector& normFiberVelocities = SimTK::Vector()) const;
    /// Export the fiber tendon force multiplier curve to a DataTable. If
    /// the normFiberVelocities argument is omitted, we use
    /// createVectorLinspace(200, 0.95, 1 + <strain at 1 norm force>)
    DataTable exportTendonForceMultiplierToTable(
            const SimTK::Vector& normTendonLengths = SimTK::Vector()) const;
    /// Print the muscle curves to STO files. The files will be named as
    /// `<muscle-name>_<curve_type>.sto`.
    ///
    /// @param directory
    ///     The directory to which the data files should be written. Do NOT
    ///     include the filename. By default, the files are printed to the
    ///     current working directory.
    void printCurvesToSTOFiles(const std::string& directory = ".") const;

    /// Replace muscles of other types in the model with muscles of this type.
    /// Currently, only Millard2012EquilibriumMuscles and Thelen2003Muscles
    /// are replaced. For these two muscle classes, we copy property values into
    /// equivalent properties of the newly created DeGrooteFregly2016Muscle. 
    /// If the model has muscles of other types, an exception is
    /// thrown unless allowUnsupportedMuscles is true, in which a
    /// DeGrooteFregly2016Muscle is created using only the base Muscle class 
    /// property values. 
    /// Since the DeGrooteFregly2016Muscle implements tendon compliance dynamics
    /// with normalized tendon force as the state variable, this function
    /// ignores the 'default_fiber_length' property in replaced muscles.
    static void replaceMuscles(
            Model& model, bool allowUnsupportedMuscles = false);
    /// @}

    /// @name Scaling
    /// @{
    /// Adjust the properties of the muscle after the model has been scaled. The
    /// optimal fiber length and tendon slack length are each multiplied by the
    /// ratio of the current path length and the path length before scaling.
    void extendPostScale(
            const SimTK::State& s, const ScaleSet& scaleSet) override;
    /// @}

private:
    void constructProperties();

    void calcMuscleLengthInfoHelper(const SimTK::Real& muscleTendonLength,
            const bool& ignoreTendonCompliance, MuscleLengthInfo& mli,
            const SimTK::Real& normTendonForce = SimTK::NaN) const;
    /// `normTendonForce` is required if and only if `isTendonDynamicsExplicit`
    /// is true. `normTendonForceDerivative` is required if and only if
    /// `isTendonDynamicsExplicit` is false.
    void calcFiberVelocityInfoHelper(const SimTK::Real& muscleTendonVelocity,
            const SimTK::Real& activation, const bool& ignoreTendonCompliance,
            const bool& isTendonDynamicsExplicit, const MuscleLengthInfo& mli,
            FiberVelocityInfo& fvi,
            const SimTK::Real& normTendonForce = SimTK::NaN,
            const SimTK::Real& normTendonForceDerivative = SimTK::NaN) const;
    void calcMuscleDynamicsInfoHelper(const SimTK::Real& activation,
            const bool& ignoreTendonCompliance, const MuscleLengthInfo& mli,
            const FiberVelocityInfo& fvi, MuscleDynamicsInfo& mdi,
            const SimTK::Real& normTendonForce = SimTK::NaN) const;
    void calcMusclePotentialEnergyInfoHelper(const bool& ignoreTendonCompliance,
            const MuscleLengthInfo& mli, MusclePotentialEnergyInfo& mpei) const;

    SimTK::Real getFiberWidth() const {
        const auto normFiberWidth = sin(get_pennation_angle_at_optimal());
        return get_optimal_fiber_length() * normFiberWidth;
    }
    SimTK::Real getSquareFiberWidth() const {
        return SimTK::square(getFiberWidth());
    }
    SimTK::Real getMaxContractionVelocityInMetersPerSecond() const {
        return get_max_contraction_velocity() * get_optimal_fiber_length();
    }
    /// Tendon stiffness parameter kT from De Groote et al., 2016. Instead of
    /// kT, users specify tendon strain at 1 norm force, which is more intuitive.
    SimTK::Real getTendonStiffnessParameter() const {
        return log((1.0 + c3) / c1) /
           (1.0 + get_tendon_strain_at_one_norm_force() - c2);
    }

    /// This is a Gaussian-like function used in the active force-length curve.
    /// A proper Gaussian function does not have the variable in the denominator
    /// of the exponent.
    /// The supplement for De Groote et al., 2016 has a typo:
    /// the denominator should be squared.
    static SimTK::Real calcGaussianLikeCurve(const SimTK::Real& x,
            const double& b1, const double& b2, const double& b3,
            const double& b4) {
        using SimTK::square;
        return b1 * exp(-0.5 * square(x - b2) / square(b3 + b4 * x));
    }

    /// The derivative of the curve defined in calcGaussianLikeCurve() with
    /// respect to 'x' (usually normalized fiber length).
    static SimTK::Real calcGaussianLikeCurveDerivative(const SimTK::Real& x,
            const double& b1, const double& b2, const double& b3,
            const double& b4) {
        using SimTK::cube;
        using SimTK::square;
        return (b1 * exp(-square(b2 - x) / (2 * square(b3 + b4 * x))) *
                       (b2 - x) * (b3 + b2 * b4)) /
               cube(b3 + b4 * x);
    }

    //enum StatusFromEstimateMuscleFiberState {
    //    Success_Converged,
    //    Warning_FiberAtLowerBound,
    //    Warning_FiberAtUpperBound,
    //    Failure_MaxIterationsReached
    //};

    //struct ValuesFromEstimateMuscleFiberState {
    //    int iterations;
    //    double solution_error;
    //    double fiber_length;
    //    double fiber_velocity;
    //    double normalized_tendon_force;
    //};

    //std::pair<StatusFromEstimateMuscleFiberState,
    //        ValuesFromEstimateMuscleFiberState>
    //estimateMuscleFiberState(const double activation, 
    //        const double muscleTendonLength, const double muscleTendonVelocity, 
    //        const double normTendonForceDerivative, const double tolerance,
    //        const int maxIterations) const;

    // Curve parameters.
    // Notation comes from De Groote et al., 2016 (supplement).

    // Parameters for the active fiber force-length curve.
    // ---------------------------------------------------
    // Values are taken from https://simtk.org/projects/optcntrlmuscle
    // rather than the paper supplement. b11 was modified to ensure that
    // f(1) = 1.
    constexpr static double b11 = 0.8150671134243542;
    constexpr static double b21 = 1.055033428970575;
    constexpr static double b31 = 0.162384573599574;
    constexpr static double b41 = 0.063303448465465;
    constexpr static double b12 = 0.433004984392647;
    constexpr static double b22 = 0.716775413397760;
    constexpr static double b32 = -0.029947116970696;
    constexpr static double b42 = 0.200356847296188;
    constexpr static double b13 = 0.1;
    constexpr static double b23 = 1.0;
    constexpr static double b33 = 0.353553390593274; // 0.5 * sqrt(0.5)
    constexpr static double b43 = 0.0;

    // Parameters for the passive fiber force-length curve.
    // ---------------------------------------------------
    // Exponential shape factor.
    constexpr static double kPE = 4.0;

    // Parameters for the tendon force curve.
    // --------------------------------------
    constexpr static double c1 = 0.200;
    // Horizontal asymptote as x -> -inf is -c3.
    // Normalized force at 0 strain is c1 * exp(-c2) - c3.
    // This parameter is 0.995 in De Groote et al., which causes the y-value at
    // 0 strain to be negative. We use 1.0 so that the y-value at 0 strain is 0
    // (since c2 == c3).
    constexpr static double c2 = 1.0;
    // This parameter is 0.250 in De Groote et al., which causes
    // lim(x->-inf) = -0.25 instead of -0.20.
    constexpr static double c3 = 0.200;

    // Parameters for the force-velocity curve.
    // ----------------------------------------
    // The parameters from the paper supplement are rounded/truncated and cause
    // the curve to not go through the points (-1, 0) and (0, 1). We solved for
    // different values of d1 and d4 so that the curve goes through (-1, 0) and
    // (0, 1).
    // The values from the code at https://simtk.org/projects/optcntrlmuscle
    // also do not go through (-1, 0) and (0, 1).
    constexpr static double d1 = -0.3211346127989808;
    constexpr static double d2 = -8.149;
    constexpr static double d3 = -0.374;
    constexpr static double d4 = 0.8825327733249912;

    constexpr static double m_minNormFiberLength = 0.2;
    constexpr static double m_maxNormFiberLength = 1.8;

    constexpr static double m_minNormTendonForce = 0.0;
    constexpr static double m_maxNormTendonForce = 5.0;

    static const std::string STATE_ACTIVATION_NAME;
    static const std::string STATE_NORMALIZED_TENDON_FORCE_NAME;
    static const std::string DERIVATIVE_NORMALIZED_TENDON_FORCE_NAME;
    static const std::string RESIDUAL_NORMALIZED_TENDON_FORCE_NAME;

    // Computed from properties.
    // -------------------------
    bool m_isTendonDynamicsExplicit = true;

    // Indices for MuscleDynamicsInfo::userDefinedDynamicsExtras.
    constexpr static int m_mdi_passiveFiberElasticForce = 0;
    constexpr static int m_mdi_passiveFiberDampingForce = 1;
    constexpr static int m_mdi_partialPennationAnglePartialFiberLength = 2;
    constexpr static int m_mdi_partialFiberForceAlongTendonPartialFiberLength =
            3;
    constexpr static int m_mdi_partialTendonForcePartialFiberLength = 4;
};

} // namespace OpenSim

#endif // DEGROOTEFREGLY2016MUSCLE_H
