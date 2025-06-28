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

    MeyerFregly2016Muscle() { constructProperties(); }

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
    void computeInitialFiberEquilibrium(SimTK::State& s) const override {
        // This is a rigid tendon model.
        return;
    }
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

    static std::string getActivationStateName() {
        return STATE_ACTIVATION_NAME;
    }

    static double getMinNormalizedFiberLength() { return m_minNormFiberLength; }
    static double getMaxNormalizedFiberLength() { return m_maxNormFiberLength; }
    /// The first element of the Vec2 is the lower bound, and the second is the
    /// upper bound.
    /// Note that since fiber length is not used as a state variable, these
    /// bounds cannot be enforced directly. It is upon the user to ensure the
    /// muscle fiber is operating within the specified domain.
    SimTK::Vec2 getBoundsNormalizedFiberLength() const {
        return {getMinNormalizedFiberLength(), getMaxNormalizedFiberLength()};
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
    /// @note It is upon the user to check that the muscle fiber is acting
    ///       within the specified domain. Force computations outside this range
    ///       may be incorrect.
    static SimTK::Real calcForceVelocityMultiplier(
            const SimTK::Real& normFiberVelocity) {
        return d1 + d2 * atan(d3 + d4 * atan(d5 + d6 * normFiberVelocity));
    }

    /// This is the inverse of the force-velocity multiplier function, and
    /// returns the normalized fiber velocity (in [-1, 1]) as a function of
    /// the force-velocity multiplier.
    static SimTK::Real calcForceVelocityInverseCurve(
            const SimTK::Real& forceVelocityMult) {
        // The version of this equation in the supplementary materials of De
        // Groote et al., 2016 has an error (it's missing a "-d3" before
        // dividing by "d2").

        return (tan((tan((forceVelocityMult - d1) / d2) - d3) / d4) - d5) / d6;
    }

    /// This is the passive force-length curve. The curve becomes negative below
    /// the minNormFiberLength.
    SimTK::Real calcPassiveForceMultiplier(
            const SimTK::Real& normFiberLength) const {
        if (get_ignore_passive_fiber_force()) return 0;

        return e1 * log(exp(e2 * (normFiberLength - e3)) + 1);
    }

    /// This is the derivative of the passive force-length curve with respect to
    /// the normalized fiber length.
    SimTK::Real calcPassiveForceMultiplierDerivative(
            const SimTK::Real& normFiberLength) const {
        if (get_ignore_passive_fiber_force()) return 0;

        return e1 / (exp(e2 * (normFiberLength - e3)) + 1) *
                exp(e2 * (normFiberLength - e3)) * e2;
    }

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
    /// equivalent properties of the newly created MeyerFregly2016Muscle.
    /// If the model has muscles of other types, an exception is
    /// thrown unless allowUnsupportedMuscles is true, in which a
    /// MeyerFregly2016Muscle is created using only the base Muscle class
    /// property values.
    /// @note MeyerFregly2016Muscle does not have a property to set the fiber
    ///       strain at which the passive fiber reaches one normalized force.
    ///       Therefore, converted muscles will use the passive fiber force
    ///       curves in this muscle and ignore such properties in the original
    ///       muscle.
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
            MuscleLengthInfo& mli) const;
    /// `normTendonForce` is required if and only if `isTendonDynamicsExplicit`
    /// is true. `normTendonForceDerivative` is required if and only if
    /// `isTendonDynamicsExplicit` is false.
    void calcFiberVelocityInfoHelper(const SimTK::Real& muscleTendonVelocity,
            const SimTK::Real& activation, const MuscleLengthInfo& mli,
            FiberVelocityInfo& fvi) const;
    void calcMuscleDynamicsInfoHelper(const SimTK::Real& activation,
            const MuscleLengthInfo& mli, const FiberVelocityInfo& fvi,
            MuscleDynamicsInfo& mdi) const;
    void calcMusclePotentialEnergyInfoHelper(
            const MuscleLengthInfo& mli, MusclePotentialEnergyInfo& mpei) const;

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

    // Curve parameters.
    // Notation comes from De Groote et al., 2016 (supplement).

    // Parameters for the active fiber force-length curve.
    // ---------------------------------------------------
    constexpr static double b11 = 0.8174335195120225;
    constexpr static double b21 = 1.054348561163096;
    constexpr static double b31 = 0.16194288662761705;
    constexpr static double b41 = 0.06381565266097716;
    constexpr static double b12 = 0.43130780147182907;
    constexpr static double b22 = 0.7163004817144202;
    constexpr static double b32 = -0.029060905806803296;
    constexpr static double b42 = 0.19835014521987723;
    constexpr static double b13 = 0.1;
    constexpr static double b23 = 1.0;
    constexpr static double b33 = 0.353553390593274; // 0.5 * sqrt(0.5)
    constexpr static double b43 = 0.0;

    // Parameters for the passive fiber force-length curve.
    // ---------------------------------------------------
    constexpr static double e1 = 0.232000797810576;
    constexpr static double e2 = 12.438535493526128;
    constexpr static double e3 = 1.329470475731338;

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
    constexpr static double d1 = -32.51401019139919;
    constexpr static double d2 = 22.160392466960214;
    constexpr static double d3 = 18.7932134796918;
    constexpr static double d4 = 6.320952269683997;
    constexpr static double d5 = -0.27671677680513945;
    constexpr static double d6 = 8.053304562566995;

    constexpr static double m_minNormFiberLength = 0.2;
    constexpr static double m_maxNormFiberLength = 1.8;

    static const std::string STATE_ACTIVATION_NAME;


    // Indices for MuscleDynamicsInfo::userDefinedDynamicsExtras.
    constexpr static int m_mdi_passiveFiberElasticForce = 0;
    constexpr static int m_mdi_passiveFiberDampingForce = 1;
    constexpr static int m_mdi_partialPennationAnglePartialFiberLength = 2;
    constexpr static int m_mdi_partialFiberForceAlongTendonPartialFiberLength =
            3;
    constexpr static int m_mdi_partialTendonForcePartialFiberLength = 4;
};

} // namespace OpenSim

#endif // OPENSIM_MEYERFREGLY2016MUSCLE_H