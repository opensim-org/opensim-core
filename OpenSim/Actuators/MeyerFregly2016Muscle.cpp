/* -------------------------------------------------------------------------- *
 *                    OpenSim:  MeyerFregly2016Muscle.cpp                     *
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

#include "MeyerFregly2016Muscle.h"

#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>
#include <OpenSim/Actuators/Thelen2003Muscle.h>
#include <OpenSim/Common/CommonUtilities.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "OpenSim/Common/STOFileAdapter.h"

using namespace OpenSim;

//=============================================================================
// CONSTANTS
//=============================================================================

// Notation for the following curve parameters comes from the supplemental
// material of the De Groote et al., 2016 paper, which the MeyerFregly2016Muscle
// implementation is based on:
//
// De Groote, F., Kinney, A. L., Rao, A. V., & Fregly, B. J. (2016). Evaluation
// of Direct Collocation Optimal Control Problem Formulations for Solving the
// Muscle Redundancy Problem. Annals of Biomedical Engineering, 44(10), 1–15.
// http://doi.org/10.1007/s10439-016-1591-9

// Parameters for the active fiber force-length curve.
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
constexpr static double e1 = 0.232000797810576;
constexpr static double e2 = 12.438535493526128;
constexpr static double e3 = 1.329470475731338;

// Exponential shape factor.
constexpr static double kPE = 4.0;

// Parameters for the tendon force curve.
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
constexpr static double d1 = -32.51401019139919;
constexpr static double d2 = 22.160392466960214;
constexpr static double d3 = 18.7932134796918;
constexpr static double d4 = 6.320952269683997;
constexpr static double d5 = -0.27671677680513945;
constexpr static double d6 = 8.053304562566995;

constexpr static double m_minNormFiberLength = 0.2;
constexpr static double m_maxNormFiberLength = 1.8;

// Indices for MuscleDynamicsInfo::userDefinedDynamicsExtras.
constexpr static int m_mdi_passiveFiberElasticForce = 0;
constexpr static int m_mdi_passiveFiberDampingForce = 1;
constexpr static int m_mdi_partialPennationAnglePartialFiberLength = 2;
constexpr static int m_mdi_partialFiberForceAlongTendonPartialFiberLength = 3;
constexpr static int m_mdi_partialTendonForcePartialFiberLength = 4;

static const std::string STATE_ACTIVATION_NAME = "activation";

//=============================================================================
// CONSTRUCTION
//=============================================================================
MeyerFregly2016Muscle::MeyerFregly2016Muscle() {
    constructProperties();
}

void MeyerFregly2016Muscle::constructProperties() {
    constructProperty_activation_time_constant(0.015);
    constructProperty_deactivation_time_constant(0.060);
    constructProperty_default_activation(0.5);
    constructProperty_active_force_width_scale(1.0);
    constructProperty_fiber_damping(0.0);
    constructProperty_ignore_passive_fiber_force(false);
    constructProperty_activation_dynamics_smoothing(0.1);
}

//=============================================================================
// COMPONENT INTERFACE
//=============================================================================
void MeyerFregly2016Muscle::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();
    if (!get_ignore_tendon_compliance()) {
        log_warn("The ignore_tendon_compliance property is ignored for this "
                "Muscle, but it is currently set to 'false'. "
                "Setting to 'true'.");
        set_ignore_tendon_compliance(true);
    }

    OPENSIM_THROW_IF_FRMOBJ(!get_ignore_tendon_compliance(),
            Exception,
            "The ignore_tendon_compliance property must be 'true' for this "
            "Muscle, but it is 'false'.");

    OPENSIM_THROW_IF_FRMOBJ(!getProperty_optimal_force().getValueIsDefault(),
            Exception,
            "The optimal_force property is ignored for this Force; "
            "use max_isometric_force instead.");

    SimTK_ERRCHK2_ALWAYS(get_activation_time_constant() > 0,
            "MeyerFregly2016Muscle::extendFinalizeFromProperties",
            "%s: activation_time_constant must be greater than zero, "
            "but it is %g.",
            getName().c_str(), get_activation_time_constant());

    SimTK_ERRCHK2_ALWAYS(get_deactivation_time_constant() > 0,
            "MeyerFregly2016Muscle::extendFinalizeFromProperties",
            "%s: deactivation_time_constant must be greater than zero, "
            "but it is %g.",
            getName().c_str(), get_deactivation_time_constant());

    SimTK_ERRCHK2_ALWAYS(get_default_activation() > 0,
            "MeyerFregly2016Muscle::extendFinalizeFromProperties",
            "%s: default_activation must be greater than zero, "
            "but it is %g.",
            getName().c_str(), get_default_activation());

    SimTK_ERRCHK2_ALWAYS(get_active_force_width_scale() >= 1,
            "MeyerFregly2016Muscle::extendFinalizeFromProperties",
            "%s: active_force_width_scale must be greater than or equal to "
            "1.0, "
            "but it is %g.",
            getName().c_str(), get_active_force_width_scale());

    SimTK_ERRCHK2_ALWAYS(get_fiber_damping() >= 0,
            "MeyerFregly2016Muscle::extendFinalizeFromProperties",
            "%s: fiber_damping must be greater than or equal to zero, "
            "but it is %g.",
            getName().c_str(), get_fiber_damping());

    SimTK_ERRCHK2_ALWAYS(get_activation_dynamics_smoothing() > 0,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: activation_dynamics_smoothing must be greater than zero, "
            "but it is %g.",
            getName().c_str(), get_activation_dynamics_smoothing());

    if (get_activation_dynamics_smoothing() <= 0.1) {
        log_debug("The activation_dynamics_smoothing property is set to {}, "
                "which is equal or less than the original default value of "
                "the model, but may produce activation and deactivation times "
                "that are inconsistent with the activation dynamics time "
                "constants. A value of 10 is recommended to achieve activation "
                "and deactivation speeds closer to the intended time constants. ",
                get_activation_dynamics_smoothing());
    }

    OPENSIM_THROW_IF_FRMOBJ(
            get_pennation_angle_at_optimal() < 0 ||
                    get_pennation_angle_at_optimal() >
                            SimTK::Pi / 2.0 - SimTK::SignificantReal,
            InvalidPropertyValue,
            getProperty_pennation_angle_at_optimal().getName(),
            "Pennation angle at optimal fiber length must be in the range [0, "
            "Pi/2).");
}

void MeyerFregly2016Muscle::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    if (!get_ignore_activation_dynamics()) {
        addStateVariable(STATE_ACTIVATION_NAME, SimTK::Stage::Dynamics);
    }
}

void MeyerFregly2016Muscle::extendInitStateFromProperties(
        SimTK::State& s) const {
    Super::extendInitStateFromProperties(s);
    if (!get_ignore_activation_dynamics()) {
        setActivation(s, get_default_activation());
    }
}

void MeyerFregly2016Muscle::extendSetPropertiesFromState(
        const SimTK::State& s) {
    Super::extendSetPropertiesFromState(s);
    if (!get_ignore_activation_dynamics()) {
        set_default_activation(getActivation(s));
    }
}

void MeyerFregly2016Muscle::computeStateVariableDerivatives(
        const SimTK::State& s) const {

    // Activation dynamics.
    // --------------------
    if (!get_ignore_activation_dynamics()) {
        const auto& activation = getActivation(s);
        const auto& excitation = getControl(s);
        static const double actTimeConst = get_activation_time_constant();
        static const double deactTimeConst = get_deactivation_time_constant();
        static const double tanhSteepness = get_activation_dynamics_smoothing();
        //     f = 0.5 tanh(b(e - a))
        //     z = 0.5 + 1.5a
        // da/dt = [(f + 0.5)/(tau_a * z) + (-f + 0.5)*z/tau_d] * (e - a)
        const SimTK::Real timeConstFactor = 0.5 + 1.5 * activation;
        const SimTK::Real tempAct = 1.0 / (actTimeConst * timeConstFactor);
        const SimTK::Real tempDeact = timeConstFactor / deactTimeConst;
        const SimTK::Real f =
                0.5 * tanh(tanhSteepness * (excitation - activation));
        const SimTK::Real timeConst =
                tempAct * (f + 0.5) + tempDeact * (-f + 0.5);
        const SimTK::Real derivative = timeConst * (excitation - activation);
        setStateVariableDerivativeValue(s, STATE_ACTIVATION_NAME, derivative);
    }
}

//=============================================================================
// MODEL COMPONENT INTERFACE
//=============================================================================
void MeyerFregly2016Muscle::extendPostScale(
        const SimTK::State& s, const ScaleSet& scaleSet) {
    Super::extendPostScale(s, scaleSet);

    AbstractGeometryPath& path = updPath();
    if (path.getPreScaleLength(s) > 0.0) {
        double scaleFactor = path.getLength(s) / path.getPreScaleLength(s);
        upd_optimal_fiber_length() *= scaleFactor;
        upd_tendon_slack_length() *= scaleFactor;

        // Clear the pre-scale length that was stored in the path.
        path.setPreScaleLength(s, 0.0);
    }
}

//=============================================================================
// ACTUATOR INTERFACE
//=============================================================================
double MeyerFregly2016Muscle::computeActuation(const SimTK::State& s) const {
    const auto& mdi = getMuscleDynamicsInfo(s);
    setActuation(s, mdi.tendonForce);
    return mdi.tendonForce;
}

//=============================================================================
// MUSCLE INTERFACE
//=============================================================================
 double MeyerFregly2016Muscle::getActivation(const SimTK::State& s) const {
    // We override the Muscle's implementation because Muscle requires
    // realizing to Dynamics to access activation from MuscleDynamicsInfo,
    // which is unnecessary if the activation is a state.
    if (get_ignore_activation_dynamics()) {
        return getControl(s);
    } else {
        return getStateVariableValue(s, STATE_ACTIVATION_NAME);
    }
}

void MeyerFregly2016Muscle::setActivation(SimTK::State& s,
        double activation) const {
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

double MeyerFregly2016Muscle::calcInextensibleTendonActiveFiberForce(
        SimTK::State& s, double activation) const {
    MuscleLengthInfo mli;
    FiberVelocityInfo fvi;
    MuscleDynamicsInfo mdi;
    const double muscleTendonLength = getLength(s);
    const double muscleTendonVelocity = getLengtheningSpeed(s);
    calcMuscleLengthInfoHelper(muscleTendonLength, mli);
    calcFiberVelocityInfoHelper(
            muscleTendonVelocity, activation, mli, fvi);
    calcMuscleDynamicsInfoHelper(activation, mli, fvi, mdi);

    return mdi.activeFiberForce;
}

void MeyerFregly2016Muscle::calcMuscleLengthInfo(
        const SimTK::State& s, MuscleLengthInfo& mli) const {

    const auto& muscleTendonLength = getLength(s);
    calcMuscleLengthInfoHelper(muscleTendonLength, mli);

    if (mli.tendonLength < get_tendon_slack_length()) {
        log_info("MeyerFregly2016Muscle '{}' is buckling (length < "
                 "tendon_slack_length) at time {} s.",
                getName(), s.getTime());
    }
}

void MeyerFregly2016Muscle::calcFiberVelocityInfo(
        const SimTK::State& s, FiberVelocityInfo& fvi) const {

    const auto& mli = getMuscleLengthInfo(s);
    const auto& muscleTendonVelocity = getLengtheningSpeed(s);
    const auto& activation = getActivation(s);

    calcFiberVelocityInfoHelper(muscleTendonVelocity, activation, mli, fvi);

    if (fvi.normFiberVelocity < -1.0) {
        log_info("MeyerFregly2016Muscle '{}' is exceeding maximum "
                 "contraction velocity at time {} s.",
                getName(), s.getTime());
    }
}

void MeyerFregly2016Muscle::calcMuscleDynamicsInfo(
        const SimTK::State& s, MuscleDynamicsInfo& mdi) const {
    const auto& activation = getActivation(s);
    const auto& mli = getMuscleLengthInfo(s);
    const auto& fvi = getFiberVelocityInfo(s);

    calcMuscleDynamicsInfoHelper(activation, mli, fvi, mdi);
}

void MeyerFregly2016Muscle::calcMusclePotentialEnergyInfo(
        const SimTK::State& s, MusclePotentialEnergyInfo& mpei) const {
    const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
    calcMusclePotentialEnergyInfoHelper(mli, mpei);
}

void MeyerFregly2016Muscle::computeInitialFiberEquilibrium(SimTK::State&) const {
    // This is a rigid tendon model, so no computation is needed here.
    return;
}

//=============================================================================
// ACCESSORS
//=============================================================================

double MeyerFregly2016Muscle::getPassiveFiberElasticForce(
        const SimTK::State& s) const {
    return getMuscleDynamicsInfo(s)
            .userDefinedDynamicsExtras[m_mdi_passiveFiberElasticForce];
}
double MeyerFregly2016Muscle::getPassiveFiberElasticForceAlongTendon(
        const SimTK::State& s) const {
    return getMuscleDynamicsInfo(s)
               .userDefinedDynamicsExtras[m_mdi_passiveFiberElasticForce] *
           getMuscleLengthInfo(s).cosPennationAngle;
}
double MeyerFregly2016Muscle::getPassiveFiberDampingForce(
        const SimTK::State& s) const {
    return getMuscleDynamicsInfo(s)
               .userDefinedDynamicsExtras[m_mdi_passiveFiberDampingForce];
}
double MeyerFregly2016Muscle::getPassiveFiberDampingForceAlongTendon(
        const SimTK::State& s) const {
    return getMuscleDynamicsInfo(s)
               .userDefinedDynamicsExtras[m_mdi_passiveFiberDampingForce] *
           getMuscleLengthInfo(s).cosPennationAngle;
}

SimTK::Vec2 MeyerFregly2016Muscle::getBoundsNormalizedFiberLength() const {
    return {m_minNormFiberLength, m_maxNormFiberLength};
}

//=============================================================================
// CALCULATION METHODS
//=============================================================================
SimTK::Real MeyerFregly2016Muscle::calcActiveForceLengthMultiplier(
        SimTK::Real normFiberLength) const {
    const double& scale = get_active_force_width_scale();
    // Shift the curve so its peak is at the origin, scale it
    // horizontally, then shift it back so its peak is still at x = 1.0.
    const double x = (normFiberLength - 1.0) / scale + 1.0;
    return calcGaussianLikeCurve(x, b11, b21, b31, b41) +
           calcGaussianLikeCurve(x, b12, b22, b32, b42) +
           calcGaussianLikeCurve(x, b13, b23, b33, b43);
}

SimTK::Real MeyerFregly2016Muscle::calcActiveForceLengthMultiplierDerivative(
        SimTK::Real normFiberLength) const {
    const double& scale = get_active_force_width_scale();
    // Shift the curve so its peak is at the origin, scale it
    // horizontally, then shift it back so its peak is still at x = 1.0.
    const double x = (normFiberLength - 1.0) / scale + 1.0;
    return (1.0 / scale) *
           (calcGaussianLikeCurveDerivative(x, b11, b21, b31, b41) +
            calcGaussianLikeCurveDerivative(x, b12, b22, b32, b42) +
            calcGaussianLikeCurveDerivative(x, b13, b23, b33, b43));
}

SimTK::Real MeyerFregly2016Muscle::calcForceVelocityMultiplier(
        SimTK::Real normFiberVelocity) {
    return d1 + d2 * atan(d3 + d4 * atan(d5 + d6 * normFiberVelocity));
}

SimTK::Real MeyerFregly2016Muscle::calcForceVelocityInverseCurve(
        SimTK::Real forceVelocityMult) {
    // The version of this equation in the supplementary materials of De
    // Groote et al., 2016 has an error (it's missing a "-d3" before
    // dividing by "d2").

    return (tan((tan((forceVelocityMult - d1) / d2) - d3) / d4) - d5) / d6;
}

SimTK::Real MeyerFregly2016Muscle::calcPassiveForceMultiplier(
        SimTK::Real normFiberLength) const {
    if (get_ignore_passive_fiber_force()) return 0;

    return e1 * log(exp(e2 * (normFiberLength - e3)) + 1);
}

SimTK::Real MeyerFregly2016Muscle::calcPassiveForceMultiplierDerivative(
        SimTK::Real normFiberLength) const {
    if (get_ignore_passive_fiber_force()) return 0;

    return e1 / (exp(e2 * (normFiberLength - e3)) + 1) *
            exp(e2 * (normFiberLength - e3)) * e2;
}

//=============================================================================
// HELPER FUNCTIONS
//=============================================================================
void MeyerFregly2016Muscle::calcMuscleLengthInfoHelper(
        SimTK::Real muscleTendonLength, MuscleLengthInfo& mli) const {

    // Tendon.
    // -------
    mli.normTendonLength = 1.0;
    mli.tendonStrain = 0.0;
    mli.tendonLength = get_tendon_slack_length();

    // Fiber.
    // ------
    using SimTK::square;
    const auto normFiberWidth = sin(get_pennation_angle_at_optimal());
    const auto fiberWidth = get_optimal_fiber_length() * normFiberWidth;
    const auto squareFiberWidth = square(fiberWidth);
    mli.fiberLengthAlongTendon = muscleTendonLength - mli.tendonLength;
    mli.fiberLength = sqrt(
            SimTK::square(mli.fiberLengthAlongTendon) + squareFiberWidth);
    mli.normFiberLength = mli.fiberLength / get_optimal_fiber_length();

    // Pennation.
    // ----------
    mli.cosPennationAngle = mli.fiberLengthAlongTendon / mli.fiberLength;
    mli.sinPennationAngle = fiberWidth / mli.fiberLength;
    mli.pennationAngle = asin(mli.sinPennationAngle);

    // Multipliers.
    // ------------
    mli.fiberPassiveForceLengthMultiplier =
            calcPassiveForceMultiplier(mli.normFiberLength);
    mli.fiberActiveForceLengthMultiplier =
            calcActiveForceLengthMultiplier(mli.normFiberLength);
}

void MeyerFregly2016Muscle::calcFiberVelocityInfoHelper(
        SimTK::Real muscleTendonVelocity, SimTK::Real activation,
        const MuscleLengthInfo& mli, FiberVelocityInfo& fvi) const {

    fvi.normTendonVelocity = 0.0;
    fvi.tendonVelocity = 0.0;
    fvi.fiberVelocityAlongTendon = muscleTendonVelocity;
    fvi.fiberVelocity =
            fvi.fiberVelocityAlongTendon * mli.cosPennationAngle;
    const auto maxContractionVelocityInMetersPerSecond =
            get_max_contraction_velocity() * get_optimal_fiber_length();
    fvi.normFiberVelocity =
            fvi.fiberVelocity / maxContractionVelocityInMetersPerSecond;
    fvi.fiberForceVelocityMultiplier =
            calcForceVelocityMultiplier(fvi.normFiberVelocity);
    const auto normFiberWidth = sin(get_pennation_angle_at_optimal());
    const auto fiberWidth = get_optimal_fiber_length() * normFiberWidth;
    const SimTK::Real tanPennationAngle =
            fiberWidth / mli.fiberLengthAlongTendon;
    fvi.pennationAngularVelocity =
            -fvi.fiberVelocity / mli.fiberLength * tanPennationAngle;
}

void MeyerFregly2016Muscle::calcMuscleDynamicsInfoHelper(
        SimTK::Real activation, const MuscleLengthInfo& mli,
        const FiberVelocityInfo& fvi, MuscleDynamicsInfo& mdi) const {

    mdi.activation = activation;

    SimTK::Real activeFiberForce;
    SimTK::Real conPassiveFiberForce;
    SimTK::Real nonConPassiveFiberForce;
    SimTK::Real totalFiberForce;
    calcFiberForce(mdi.activation, mli.fiberActiveForceLengthMultiplier,
            fvi.fiberForceVelocityMultiplier,
            mli.fiberPassiveForceLengthMultiplier, fvi.normFiberVelocity,
            activeFiberForce, conPassiveFiberForce, nonConPassiveFiberForce,
            totalFiberForce);

    SimTK::Real passiveFiberForce =
            conPassiveFiberForce + nonConPassiveFiberForce;

    // TODO revisit this if compressive forces become an issue.
    //// When using a rigid tendon, avoid generating compressive fiber forces by
    //// saturating the damping force produced by the parallel element.
    //// Based on Millard2012EquilibriumMuscle::calcMuscleDynamicsInfo().
    // if (get_ignore_tendon_compliance()) {
    //    if (totalFiberForce < 0) {
    //        totalFiberForce = 0.0;
    //        nonConPassiveFiberForce = -activeFiberForce -
    //        conPassiveFiberForce; passiveFiberForce = conPassiveFiberForce +
    //        nonConPassiveFiberForce;
    //    }
    //}

    // Compute force entries.
    // ----------------------
    const auto maxIsometricForce = get_max_isometric_force();
    mdi.fiberForce = totalFiberForce;
    mdi.activeFiberForce = activeFiberForce;
    mdi.passiveFiberForce = passiveFiberForce;
    mdi.normFiberForce = mdi.fiberForce / maxIsometricForce;
    mdi.fiberForceAlongTendon = mdi.fiberForce * mli.cosPennationAngle;
    mdi.normTendonForce = mdi.normFiberForce * mli.cosPennationAngle;
    mdi.tendonForce = mdi.fiberForceAlongTendon;

    // Compute stiffness entries.
    // --------------------------
    mdi.fiberStiffness = calcFiberStiffness(mdi.activation, mli.normFiberLength,
            fvi.fiberForceVelocityMultiplier);
    const auto& partialPennationAnglePartialFiberLength =
            calcPartialPennationAnglePartialFiberLength(mli.fiberLength);
    const auto& partialFiberForceAlongTendonPartialFiberLength =
            calcPartialFiberForceAlongTendonPartialFiberLength(mdi.fiberForce,
                    mdi.fiberStiffness, mli.sinPennationAngle,
                    mli.cosPennationAngle,
                    partialPennationAnglePartialFiberLength);
    mdi.fiberStiffnessAlongTendon = calcFiberStiffnessAlongTendon(
            mli.fiberLength, partialFiberForceAlongTendonPartialFiberLength,
            mli.sinPennationAngle, mli.cosPennationAngle,
            partialPennationAnglePartialFiberLength);
    mdi.tendonStiffness = SimTK::Infinity;

    const auto& partialTendonForcePartialFiberLength =
            calcPartialTendonForcePartialFiberLength(mdi.tendonStiffness,
                    mli.fiberLength, mli.sinPennationAngle,
                    mli.cosPennationAngle);

    // Compute power entries.
    // ----------------------
    // In order for the fiberPassivePower to be zero work, the non-conservative
    // passive fiber force is lumped into active fiber power. This is based on
    // the implementation in Millard2012EquilibriumMuscle (and verified over
    // email with Matt Millard).
    mdi.fiberActivePower = -(mdi.activeFiberForce + nonConPassiveFiberForce) *
                           fvi.fiberVelocity;
    mdi.fiberPassivePower = -conPassiveFiberForce * fvi.fiberVelocity;
    mdi.tendonPower = -mdi.tendonForce * fvi.tendonVelocity;

    mdi.userDefinedDynamicsExtras.resize(5);
    mdi.userDefinedDynamicsExtras[m_mdi_passiveFiberElasticForce] =
            conPassiveFiberForce;
    mdi.userDefinedDynamicsExtras[m_mdi_passiveFiberDampingForce] =
            nonConPassiveFiberForce;
    mdi.userDefinedDynamicsExtras
            [m_mdi_partialPennationAnglePartialFiberLength] =
            partialPennationAnglePartialFiberLength;
    mdi.userDefinedDynamicsExtras
            [m_mdi_partialFiberForceAlongTendonPartialFiberLength] =
            partialFiberForceAlongTendonPartialFiberLength;
    mdi.userDefinedDynamicsExtras[m_mdi_partialTendonForcePartialFiberLength] =
            partialTendonForcePartialFiberLength;
}

void MeyerFregly2016Muscle::calcMusclePotentialEnergyInfoHelper(
        const MuscleLengthInfo& mli, MusclePotentialEnergyInfo& mpei) const {

    // Fiber potential energy.
    // -----------------------
    mpei.fiberPotentialEnergy =
            calcPassiveForceMultiplierIntegral(mli.normFiberLength) *
            get_optimal_fiber_length() * get_max_isometric_force();

    // Tendon potential energy.
    // ------------------------
    mpei.tendonPotentialEnergy = 0;

    // Total potential energy.
    // -----------------------
    mpei.musclePotentialEnergy =
            mpei.fiberPotentialEnergy + mpei.tendonPotentialEnergy;
}