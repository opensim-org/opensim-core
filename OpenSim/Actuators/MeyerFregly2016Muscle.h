#ifndef OPENSIM_MEYERFREGLY2016MUSCLE_H
#define OPENSIM_MEYERFREGLY2016MUSCLE_H
/* -------------------------------------------------------------------------- *
 *                    OpenSim:  MeyerFregly2016Muscle.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco, Spencer Williams                               *
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

/** This muscle model was published in Meyer et al. 2016. The implementation is
based heavily on the previously implemented DeGrooteFregly2016Muscle.

This is a rigid-tendon muscle model. Therefore, the ignore_tendon_compliance
property is ignored, and a warnign is issued if it is set to false. In the
Muscle class setIngoreActivationDynamics() controls a modeling option, meaning
this settings could theoretically be changed. However, for this class, the
modeling option is ignored and the value of the ignore_activation_dynamics
property is used directly.

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

@section property Property Bounds
The acceptable bounds for each property are enforced at model initialization.
These bounds are:
 - activation_time_constant: (0, inf]
 - deactivation_time_constant: (0, inf]
 - activation_dynamics_smoothing: (0, inf]
 - active_force_width_scale: [1, inf]
 - fiber_damping: [0, inf]
 - pennation_angle_at_optimal: [0, Pi/2)
 - default_activation: (0, inf]

@note The default value for activation_dynamics_smoothing is set to 0.1 to
      match the originally published model, but a value of 10 is recommended
      to achieve activation and deactivation speeds closer to the intended
      time constants.

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

Meyer A. J., Eskinazi, I., Jackson, J. N., Rao, A. V., Patten, C., & Fregly,
B. J. (2016). Muscle Synergies Facilitate Computational Prediction of
Subject-Specific Walking Motions. Frontiers in Bioengineering and
Biotechnology, 4, 1055–27. http://doi.org/10.3389/fbioe.2016.00077 */
class OSIMACTUATORS_API MeyerFregly2016Muscle : public Muscle {
    OpenSim_DECLARE_CONCRETE_OBJECT(MeyerFregly2016Muscle, Muscle);

public:
    OpenSim_DECLARE_PROPERTY(activation_time_constant, double,
            "Smaller value means activation can increase more rapidly. "
            "Default: 0.015 seconds.");
    OpenSim_DECLARE_PROPERTY(deactivation_time_constant, double,
            "Smaller value means activation can decrease more rapidly. "
            "Default: 0.060 seconds.");
    OpenSim_DECLARE_PROPERTY(default_activation, double,
            "Value of activation in the default state returned by "
            "initSystem(). Default: 0.5.");
    OpenSim_DECLARE_PROPERTY(active_force_width_scale, double,
            "Scale factor for the width of the active force-length curve. "
            "Larger values make the curve wider. Default: 1.0.");
    OpenSim_DECLARE_PROPERTY(fiber_damping, double,
            "Use this property to define the linear damping force that is "
            "added to the total muscle fiber force. It is computed by "
            "multiplying this damping parameter by the normalized fiber "
            "velocity and the max isometric force. Default: 0.");
    OpenSim_DECLARE_PROPERTY(ignore_passive_fiber_force, bool,
            "Make the passive fiber force 0. Default: false.");
    OpenSim_DECLARE_PROPERTY(activation_dynamics_smoothing, double,
            "The parameter that determines the smoothness of the transition "
            "of the tanh function used to smooth the switch between muscle "
            "activation (a-e < 0) and deactivation (a-e > 0). The default "
            "value is 0.1 to match the originally published model, but a value "
            "of 10 is recommended to to achieve activation and deactivation "
            "speeds closer to the intended time constants. Larger values "
            "steepen the transition, but may slow optimization convergence.");

    OpenSim_DECLARE_OUTPUT(passive_fiber_elastic_force, double,
            getPassiveFiberElasticForce, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(passive_fiber_elastic_force_along_tendon, double,
            getPassiveFiberElasticForceAlongTendon, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(passive_fiber_damping_force, double,
            getPassiveFiberDampingForce, SimTK::Stage::Dynamics);
    OpenSim_DECLARE_OUTPUT(passive_fiber_damping_force_along_tendon, double,
            getPassiveFiberDampingForceAlongTendon, SimTK::Stage::Dynamics);

    MeyerFregly2016Muscle();

protected:
    /// @name Component interface
    /// @{
    void extendFinalizeFromProperties() override;
    void extendAddToSystem(SimTK::MultibodySystem&) const override;
    void extendInitStateFromProperties(SimTK::State&) const override;
    void extendSetPropertiesFromState(const SimTK::State&) override;
    void computeStateVariableDerivatives(const SimTK::State&) const override;
    /// @}

    /// @name ModelComponent interface
    /// @{
    void extendPostScale(const SimTK::State&, const ScaleSet&) override;
    /// @}

    /// @name Actuator interface
    /// @{
    double computeActuation(const SimTK::State&) const override;
    /// @}

public:
    /// @name Muscle interface
    /// @{

    /// If ignore_activation_dynamics is true, this gets excitation instead.
    double getActivation(const SimTK::State&) const override;

    /// If ignore_activation_dynamics is true, this sets excitation instead.
    void setActivation(SimTK::State&, double) const override;

protected:
    double calcInextensibleTendonActiveFiberForce(
            SimTK::State&, double) const override;
    void calcMuscleLengthInfo(
            const SimTK::State&, MuscleLengthInfo&) const override;
    void calcFiberVelocityInfo(
            const SimTK::State&, FiberVelocityInfo&) const override;
    void calcMuscleDynamicsInfo(
            const SimTK::State&, MuscleDynamicsInfo&) const override;
    void calcMusclePotentialEnergyInfo(
            const SimTK::State&, MusclePotentialEnergyInfo&) const override;

public:
    void computeInitialFiberEquilibrium(SimTK::State&) const override;
    /// @}

    /// @name Accessors.
    /// @{

    /// Get the portion of the passive fiber force generated by the elastic
    /// element only (N).
    double getPassiveFiberElasticForce(const SimTK::State&) const;
    /// Get the portion of the passive fiber force generated by the elastic
    /// element only, projected onto the tendon direction (N).
    double getPassiveFiberElasticForceAlongTendon(const SimTK::State&) const;
    /// Get the portion of the passive fiber force generated by the damping
    /// element only (N).
    double getPassiveFiberDampingForce(const SimTK::State&) const;
    /// Get the portion of the passive fiber force generated by the damping
    /// element only, projected onto the tendon direction (N).
    double getPassiveFiberDampingForceAlongTendon(const SimTK::State&) const;

    /// The first element of the Vec2 is the lower bound, and the second is the
    /// upper bound.
    /// Note that since fiber length is not used as a state variable, these
    /// bounds cannot be enforced directly. It is upon the user to ensure the
    /// muscle fiber is operating within the specified domain.
    SimTK::Vec2 getBoundsNormalizedFiberLength() const;
    /// @}

    /// @name Calculation methods.
    /// These functions compute the values of normalized/dimensionless curves,
    /// their derivatives and integrals, and other quantities of the muscle.
    /// These do not depend on a SimTK::State.
    /// @{

    /// The active force-length curve is the sum of 3 Gaussian-like curves. The
    /// width of the curve can be adjusted via the 'active_force_width_scale'
    /// property.
    SimTK::Real calcActiveForceLengthMultiplier(SimTK::Real) const;

    /// The derivative of the active force-length curve with respect to
    /// normalized fiber length. This curve is based on the derivative of the
    /// Gaussian-like curve used in calcActiveForceLengthMultiplier(). The
    /// 'active_force_width_scale' property also affects the value of the
    /// derivative curve.
    SimTK::Real calcActiveForceLengthMultiplierDerivative(SimTK::Real) const;

    /// The parameters of this curve are not modifiable, so this function is
    /// static.
    /// @note It is upon the user to check that the muscle fiber is acting
    ///       within the specified domain. Force computations outside this range
    ///       may be incorrect.
    static SimTK::Real calcForceVelocityMultiplier(SimTK::Real);

    /// This is the inverse of the force-velocity multiplier function, and
    /// returns the normalized fiber velocity (in [-1, 1]) as a function of
    /// the force-velocity multiplier.
    static SimTK::Real calcForceVelocityInverseCurve(SimTK::Real);

    /// This is the passive force-length curve. The curve becomes negative below
    /// the minNormFiberLength.
    SimTK::Real calcPassiveForceMultiplier(SimTK::Real) const;

    /// This is the derivative of the passive force-length curve with respect to
    /// the normalized fiber length.
    SimTK::Real calcPassiveForceMultiplierDerivative(SimTK::Real) const;

    /// This is the integral of the passive force-length curve with respect to
    /// the normalized fiber length over the domain
    /// [minNormFiberLength normFiberLength], where minNormFiberLength is the
    /// value return by getMinNormalizedFiberLength().
    ///
    /// This placeholder implementation returns zero.
    SimTK::Real calcPassiveForceMultiplierIntegral(
            const SimTK::Real& normFiberLength) const {
        return 0;
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

    /// The stiffness of the whole musculotendon unit in the direction of the
    /// tendon.
    ///
    /// @note The default implementation includes additional checks that
    /// the stiffness is non-negative and that the denominator is non-zero.
    /// Checks are omitted here to preserve continuity and smoothness for
    /// optimization (see #3685).
    SimTK::Real calcMuscleStiffness(const SimTK::Real& tendonStiffness,
            const SimTK::Real& fiberStiffnessAlongTendon) const {

        if (get_ignore_tendon_compliance()) return fiberStiffnessAlongTendon;
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
        const auto normFiberWidth = sin(get_pennation_angle_at_optimal());
        const auto fiberWidth = get_optimal_fiber_length() * normFiberWidth;
        return (-fiberWidth / square(fiberLength)) /
               sqrt(1.0 - square(fiberWidth / fiberLength));
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
        calcMuscleLengthInfoHelper(muscleTendonLength, mli);
        calcFiberVelocityInfoHelper(muscleTendonVelocity, activation, mli, fvi);
        calcMuscleDynamicsInfoHelper(activation, mli, fvi, mdi);

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
            const SimTK::Real& activation) const {

        MuscleLengthInfo mli;
        FiberVelocityInfo fvi;
        MuscleDynamicsInfo mdi;
        calcMuscleLengthInfoHelper(muscleTendonLength, mli);
        calcFiberVelocityInfoHelper(muscleTendonVelocity, activation, mli, fvi);
        calcMuscleDynamicsInfoHelper(activation, mli, fvi, mdi);

        return mdi.fiberStiffnessAlongTendon * fvi.fiberVelocityAlongTendon -
               mdi.tendonStiffness *
                       (muscleTendonVelocity - fvi.fiberVelocityAlongTendon);
    }
    /// @}

private:
    void constructProperties();

    // HELPER FUNCTIONS
    void calcMuscleLengthInfoHelper(SimTK::Real, MuscleLengthInfo&) const;
    void calcFiberVelocityInfoHelper(SimTK::Real, SimTK::Real,
            const MuscleLengthInfo&, FiberVelocityInfo&) const;
    void calcMuscleDynamicsInfoHelper(SimTK::Real, const MuscleLengthInfo&,
            const FiberVelocityInfo&, MuscleDynamicsInfo&) const;
    void calcMusclePotentialEnergyInfoHelper(
            const MuscleLengthInfo&, MusclePotentialEnergyInfo&) const;

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
};

} // namespace OpenSim

#endif // OPENSIM_MEYERFREGLY2016MUSCLE_H