/* -------------------------------------------------------------------------- *
 *                 OpenSim:  DeGrooteFregly2016Muscle.cpp                     *
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

#include "DeGrooteFregly2016Muscle.h"

#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>
#include <OpenSim/Actuators/Thelen2003Muscle.h>
#include <OpenSim/Common/CommonUtilities.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "OpenSim/Common/STOFileAdapter.h"

using namespace OpenSim;

const std::string DeGrooteFregly2016Muscle::STATE_ACTIVATION_NAME("activation");
const std::string DeGrooteFregly2016Muscle::STATE_NORMALIZED_TENDON_FORCE_NAME(
        "normalized_tendon_force");
const std::string
        DeGrooteFregly2016Muscle::DERIVATIVE_NORMALIZED_TENDON_FORCE_NAME(
                "implicitderiv_normalized_tendon_force");
const std::string
        DeGrooteFregly2016Muscle::RESIDUAL_NORMALIZED_TENDON_FORCE_NAME(
                "implicitresidual_normalized_tendon_force");

// We must define these variables in some compilation unit (pre-C++17).
// https://stackoverflow.com/questions/40690260/undefined-reference-error-for-static-constexpr-member?noredirect=1&lq=1
constexpr double DeGrooteFregly2016Muscle::b11;
constexpr double DeGrooteFregly2016Muscle::b21;
constexpr double DeGrooteFregly2016Muscle::b31;
constexpr double DeGrooteFregly2016Muscle::b41;
constexpr double DeGrooteFregly2016Muscle::b12;
constexpr double DeGrooteFregly2016Muscle::b22;
constexpr double DeGrooteFregly2016Muscle::b32;
constexpr double DeGrooteFregly2016Muscle::b42;
constexpr double DeGrooteFregly2016Muscle::b13;
constexpr double DeGrooteFregly2016Muscle::b23;
constexpr double DeGrooteFregly2016Muscle::b33;
constexpr double DeGrooteFregly2016Muscle::b43;
constexpr double DeGrooteFregly2016Muscle::m_minNormFiberLength;
constexpr double DeGrooteFregly2016Muscle::m_maxNormFiberLength;
constexpr double DeGrooteFregly2016Muscle::m_minNormTendonForce;
constexpr double DeGrooteFregly2016Muscle::m_maxNormTendonForce;
constexpr int DeGrooteFregly2016Muscle::m_mdi_passiveFiberElasticForce;
constexpr int DeGrooteFregly2016Muscle::m_mdi_passiveFiberDampingForce;
constexpr int
        DeGrooteFregly2016Muscle::m_mdi_partialPennationAnglePartialFiberLength;
constexpr int DeGrooteFregly2016Muscle::
        m_mdi_partialFiberForceAlongTendonPartialFiberLength;
constexpr int
        DeGrooteFregly2016Muscle::m_mdi_partialTendonForcePartialFiberLength;

void DeGrooteFregly2016Muscle::constructProperties() {
    constructProperty_activation_time_constant(0.015);
    constructProperty_deactivation_time_constant(0.060);
    constructProperty_default_activation(0.5);
    constructProperty_default_normalized_tendon_force(0.5);
    constructProperty_active_force_width_scale(1.0);
    constructProperty_fiber_damping(0.0);
    constructProperty_passive_fiber_strain_at_one_norm_force(0.6);
    constructProperty_tendon_strain_at_one_norm_force(0.049);
    constructProperty_ignore_passive_fiber_force(false);
    constructProperty_tendon_compliance_dynamics_mode("explicit");
}

void DeGrooteFregly2016Muscle::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();
    OPENSIM_THROW_IF_FRMOBJ(!getProperty_optimal_force().getValueIsDefault(),
            Exception,
            "The optimal_force property is ignored for this Force; "
            "use max_isometric_force instead.");

    SimTK_ERRCHK2_ALWAYS(get_activation_time_constant() > 0,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: activation_time_constant must be greater than zero, "
            "but it is %g.",
            getName().c_str(), get_activation_time_constant());

    SimTK_ERRCHK2_ALWAYS(get_deactivation_time_constant() > 0,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: deactivation_time_constant must be greater than zero, "
            "but it is %g.",
            getName().c_str(), get_deactivation_time_constant());

    SimTK_ERRCHK2_ALWAYS(get_default_activation() > 0,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: default_activation must be greater than zero, "
            "but it is %g.",
            getName().c_str(), get_default_activation());

    SimTK_ERRCHK2_ALWAYS(get_default_normalized_tendon_force() >= 0,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: default_normalized_tendon_force must be greater than or equal "
            "to zero, but it is %g.",
            getName().c_str(), get_default_normalized_tendon_force());

    SimTK_ERRCHK2_ALWAYS(get_default_normalized_tendon_force() <= 5,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: default_normalized_tendon_force must be less than or equal to "
            "5.0, but it is %g.",
            getName().c_str(), get_default_normalized_tendon_force());

    SimTK_ERRCHK2_ALWAYS(get_active_force_width_scale() >= 1,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: active_force_width_scale must be greater than or equal to "
            "1.0, "
            "but it is %g.",
            getName().c_str(), get_active_force_width_scale());

    SimTK_ERRCHK2_ALWAYS(get_fiber_damping() >= 0,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: fiber_damping must be greater than or equal to zero, "
            "but it is %g.",
            getName().c_str(), get_fiber_damping());

    SimTK_ERRCHK2_ALWAYS(get_passive_fiber_strain_at_one_norm_force() > 0,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: passive_fiber_strain_at_one_norm_force must be greater "
            "than zero, but it is %g.",
            getName().c_str(), get_passive_fiber_strain_at_one_norm_force());

    SimTK_ERRCHK2_ALWAYS(get_tendon_strain_at_one_norm_force() > 0,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: tendon_strain_at_one_norm_force must be greater than zero, "
            "but it is %g.",
            getName().c_str(), get_tendon_strain_at_one_norm_force());

    OPENSIM_THROW_IF_FRMOBJ(
            get_pennation_angle_at_optimal() < 0 ||
                    get_pennation_angle_at_optimal() >
                            SimTK::Pi / 2.0 - SimTK::SignificantReal,
            InvalidPropertyValue,
            getProperty_pennation_angle_at_optimal().getName(),
            "Pennation angle at optimal fiber length must be in the range [0, "
            "Pi/2).");

    using SimTK::square;
    const auto normFiberWidth = sin(get_pennation_angle_at_optimal());
    m_fiberWidth = get_optimal_fiber_length() * normFiberWidth;
    m_squareFiberWidth = square(m_fiberWidth);
    m_maxContractionVelocityInMetersPerSecond =
            get_max_contraction_velocity() * get_optimal_fiber_length();
    m_kT = log((1.0 + c3) / c1) /
           (1.0 + get_tendon_strain_at_one_norm_force() - c2);
    m_isTendonDynamicsExplicit =
            get_tendon_compliance_dynamics_mode() == "explicit";
}

void DeGrooteFregly2016Muscle::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    if (!get_ignore_activation_dynamics()) {
        addStateVariable(STATE_ACTIVATION_NAME, SimTK::Stage::Dynamics);
    }
    if (!get_ignore_tendon_compliance()) {
        addStateVariable(
                STATE_NORMALIZED_TENDON_FORCE_NAME, SimTK::Stage::Dynamics);
        if (!m_isTendonDynamicsExplicit) {
            addDiscreteVariable(DERIVATIVE_NORMALIZED_TENDON_FORCE_NAME,
                    SimTK::Stage::Dynamics);
            addCacheVariable(RESIDUAL_NORMALIZED_TENDON_FORCE_NAME, double(0),
                    SimTK::Stage::Dynamics);
        }
    }
}

void DeGrooteFregly2016Muscle::extendInitStateFromProperties(
        SimTK::State& s) const {
    Super::extendInitStateFromProperties(s);
    if (!get_ignore_activation_dynamics()) {
        setActivation(s, get_default_activation());
    }
    if (!get_ignore_tendon_compliance()) {
        setNormalizedTendonForce(s, get_default_normalized_tendon_force());
    }
}

void DeGrooteFregly2016Muscle::extendSetPropertiesFromState(
        const SimTK::State& s) {
    Super::extendSetPropertiesFromState(s);
    if (!get_ignore_activation_dynamics()) {
        set_default_activation(getActivation(s));
    }
    if (!get_ignore_tendon_compliance()) {
        set_default_normalized_tendon_force(getNormalizedTendonForce(s));
    }
}

void DeGrooteFregly2016Muscle::computeStateVariableDerivatives(
        const SimTK::State& s) const {

    // Activation dynamics.
    // --------------------
    if (!get_ignore_activation_dynamics()) {
        const auto& activation = getActivation(s);
        const auto& excitation = getControl(s);
        static const double actTimeConst = get_activation_time_constant();
        static const double deactTimeConst = get_deactivation_time_constant();
        static const double tanhSteepness = 0.1;
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

    // Tendon compliance dynamics.
    // ---------------------------
    if (!get_ignore_tendon_compliance()) {
        double normTendonForceDerivative;
        if (m_isTendonDynamicsExplicit) {
            const auto& mli = getMuscleLengthInfo(s);
            const auto& fvi = getFiberVelocityInfo(s);
            // calcTendonForceMultiplerDerivative() is with respect to
            // normalized tendon length, so using the chain rule, to get
            // normalized tendon force derivative with respect to time, we
            // multiply by normalized fiber velocity.
            normTendonForceDerivative =
                    fvi.normTendonVelocity *
                    calcTendonForceMultiplierDerivative(mli.normTendonLength);
        } else {
            normTendonForceDerivative = getDiscreteVariableValue(
                    s, DERIVATIVE_NORMALIZED_TENDON_FORCE_NAME);
        }

        setStateVariableDerivativeValue(s, STATE_NORMALIZED_TENDON_FORCE_NAME,
                normTendonForceDerivative);
    }
}

double DeGrooteFregly2016Muscle::computeActuation(const SimTK::State& s) const {
    const auto& mdi = getMuscleDynamicsInfo(s);
    setActuation(s, mdi.tendonForce);
    return mdi.tendonForce;
}

void DeGrooteFregly2016Muscle::calcMuscleLengthInfoHelper(
        const SimTK::Real& muscleTendonLength,
        const bool& ignoreTendonCompliance, MuscleLengthInfo& mli,
        const SimTK::Real& normTendonForce) const {

    // Tendon.
    // -------
    if (ignoreTendonCompliance) {
        mli.normTendonLength = 1.0;
    } else {
        mli.normTendonLength =
                calcTendonForceLengthInverseCurve(normTendonForce);
    }
    mli.tendonStrain = mli.normTendonLength - 1.0;
    mli.tendonLength = get_tendon_slack_length() * mli.normTendonLength;

    // Fiber.
    // ------
    mli.fiberLengthAlongTendon = muscleTendonLength - mli.tendonLength;
    mli.fiberLength = sqrt(
            SimTK::square(mli.fiberLengthAlongTendon) + m_squareFiberWidth);
    mli.normFiberLength = mli.fiberLength / get_optimal_fiber_length();

    // Pennation.
    // ----------
    mli.cosPennationAngle = mli.fiberLengthAlongTendon / mli.fiberLength;
    mli.sinPennationAngle = m_fiberWidth / mli.fiberLength;
    mli.pennationAngle = asin(mli.sinPennationAngle);

    // Multipliers.
    // ------------
    mli.fiberPassiveForceLengthMultiplier =
            calcPassiveForceMultiplier(mli.normFiberLength);
    mli.fiberActiveForceLengthMultiplier =
            calcActiveForceLengthMultiplier(mli.normFiberLength);
}

void DeGrooteFregly2016Muscle::calcFiberVelocityInfoHelper(
        const SimTK::Real& muscleTendonVelocity, const SimTK::Real& activation,
        const bool& ignoreTendonCompliance,
        const bool& isTendonDynamicsExplicit, const MuscleLengthInfo& mli,
        FiberVelocityInfo& fvi, const SimTK::Real& normTendonForce,
        const SimTK::Real& normTendonForceDerivative) const {

    if (isTendonDynamicsExplicit && !ignoreTendonCompliance) {
        const auto& normFiberForce = normTendonForce / mli.cosPennationAngle;
        fvi.fiberForceVelocityMultiplier =
                (normFiberForce - mli.fiberPassiveForceLengthMultiplier) /
                (activation * mli.fiberActiveForceLengthMultiplier);
        fvi.normFiberVelocity =
                calcForceVelocityInverseCurve(fvi.fiberForceVelocityMultiplier);
        fvi.fiberVelocity = fvi.normFiberVelocity *
                            m_maxContractionVelocityInMetersPerSecond;
        fvi.fiberVelocityAlongTendon =
                fvi.fiberVelocity / mli.cosPennationAngle;
        fvi.tendonVelocity =
                muscleTendonVelocity - fvi.fiberVelocityAlongTendon;
        fvi.normTendonVelocity = fvi.tendonVelocity / get_tendon_slack_length();
    } else {
        if (ignoreTendonCompliance) {
            fvi.normTendonVelocity = 0.0;
        } else {
            fvi.normTendonVelocity =
                    calcTendonForceLengthInverseCurveDerivative(
                            normTendonForceDerivative, mli.normTendonLength);
        }
        fvi.tendonVelocity = get_tendon_slack_length() * fvi.normTendonVelocity;
        fvi.fiberVelocityAlongTendon =
                muscleTendonVelocity - fvi.tendonVelocity;
        fvi.fiberVelocity =
                fvi.fiberVelocityAlongTendon * mli.cosPennationAngle;
        fvi.normFiberVelocity =
                fvi.fiberVelocity / m_maxContractionVelocityInMetersPerSecond;
        fvi.fiberForceVelocityMultiplier =
                calcForceVelocityMultiplier(fvi.normFiberVelocity);
    }

    const SimTK::Real tanPennationAngle =
            m_fiberWidth / mli.fiberLengthAlongTendon;
    fvi.pennationAngularVelocity =
            -fvi.fiberVelocity / mli.fiberLength * tanPennationAngle;
}

void DeGrooteFregly2016Muscle::calcMuscleDynamicsInfoHelper(
        const SimTK::Real& activation, const SimTK::Real& muscleTendonVelocity,
        const bool& ignoreTendonCompliance, const MuscleLengthInfo& mli,
        const FiberVelocityInfo& fvi, MuscleDynamicsInfo& mdi,
        const SimTK::Real& normTendonForce) const {

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

    if (ignoreTendonCompliance) {
        mdi.normTendonForce = mdi.normFiberForce * mli.cosPennationAngle;
        mdi.tendonForce = mdi.fiberForceAlongTendon;
    } else {
        mdi.normTendonForce = normTendonForce;
        mdi.tendonForce = maxIsometricForce * mdi.normTendonForce;
    }

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
    mdi.tendonStiffness = calcTendonStiffness(mli.normTendonLength);
    mdi.muscleStiffness = calcMuscleStiffness(
            mdi.tendonStiffness, mdi.fiberStiffnessAlongTendon);

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
    mdi.musclePower = -mdi.tendonForce * muscleTendonVelocity;

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

void DeGrooteFregly2016Muscle::calcMusclePotentialEnergyInfoHelper(
        const bool& ignoreTendonCompliance, const MuscleLengthInfo& mli,
        MusclePotentialEnergyInfo& mpei) const {

    // Based on Millard2012EquilibriumMuscle::calcMusclePotentialEnergyInfo().

    // Fiber potential energy.
    // -----------------------
    mpei.fiberPotentialEnergy =
            calcPassiveForceMultiplierIntegral(mli.normFiberLength) *
            get_optimal_fiber_length() * get_max_isometric_force();

    // Tendon potential energy.
    // ------------------------
    mpei.tendonPotentialEnergy = 0;
    if (!ignoreTendonCompliance) {
        mpei.tendonPotentialEnergy =
                calcTendonForceMultiplierIntegral(mli.normTendonLength) *
                get_tendon_slack_length() * get_max_isometric_force();
    }

    // Total potential energy.
    // -----------------------
    mpei.musclePotentialEnergy =
            mpei.fiberPotentialEnergy + mpei.tendonPotentialEnergy;
}

void DeGrooteFregly2016Muscle::calcMuscleLengthInfo(
        const SimTK::State& s, MuscleLengthInfo& mli) const {

    const auto& muscleTendonLength = getLength(s);
    SimTK::Real normTendonForce = SimTK::NaN;
    if (!get_ignore_tendon_compliance()) {
        normTendonForce = getNormalizedTendonForce(s);
    }
    calcMuscleLengthInfoHelper(muscleTendonLength,
            get_ignore_tendon_compliance(), mli, normTendonForce);

    if (mli.tendonLength < get_tendon_slack_length()) {
        // TODO the Millard model sets fiber velocity to zero when the
        //       tendon is buckling, but this may create a discontinuity.
        log_info("DeGrooteFregly2016Muscle '{}' is buckling (length < "
                 "tendon_slack_length) at time {} s.",
                getName(), s.getTime());
    }
}

void DeGrooteFregly2016Muscle::calcFiberVelocityInfo(
        const SimTK::State& s, FiberVelocityInfo& fvi) const {

    const auto& mli = getMuscleLengthInfo(s);
    const auto& muscleTendonVelocity = getLengtheningSpeed(s);
    const auto& activation = getActivation(s);

    SimTK::Real normTendonForce = SimTK::NaN;
    SimTK::Real normTendonForceDerivative = SimTK::NaN;
    if (!get_ignore_tendon_compliance()) {
        if (m_isTendonDynamicsExplicit) {
            normTendonForce = getNormalizedTendonForce(s);
        } else {
            normTendonForceDerivative = getNormalizedTendonForceDerivative(s);
        }
    }

    calcFiberVelocityInfoHelper(muscleTendonVelocity, activation,
            get_ignore_tendon_compliance(), m_isTendonDynamicsExplicit, mli,
            fvi, normTendonForce, normTendonForceDerivative);

    if (fvi.normFiberVelocity < -1.0) {
        log_info("DeGrooteFregly2016Muscle '{}' is exceeding maximum "
                 "contraction velocity at time {} s.",
                getName(), s.getTime());
    }
}

void DeGrooteFregly2016Muscle::calcMuscleDynamicsInfo(
        const SimTK::State& s, MuscleDynamicsInfo& mdi) const {
    const auto& activation = getActivation(s);
    SimTK::Real normTendonForce = SimTK::NaN;
    if (!get_ignore_tendon_compliance()) {
        normTendonForce = getNormalizedTendonForce(s);
    }
    const auto& muscleTendonVelocity = getLengtheningSpeed(s);
    const auto& mli = getMuscleLengthInfo(s);
    const auto& fvi = getFiberVelocityInfo(s);

    calcMuscleDynamicsInfoHelper(activation, muscleTendonVelocity,
            get_ignore_tendon_compliance(), mli, fvi, mdi, normTendonForce);
}

void DeGrooteFregly2016Muscle::calcMusclePotentialEnergyInfo(
        const SimTK::State& s, MusclePotentialEnergyInfo& mpei) const {
    const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
    calcMusclePotentialEnergyInfoHelper(
            get_ignore_tendon_compliance(), mli, mpei);
}

double
OpenSim::DeGrooteFregly2016Muscle::calcInextensibleTendonActiveFiberForce(
        SimTK::State& s, double activation) const {
    MuscleLengthInfo mli;
    FiberVelocityInfo fvi;
    MuscleDynamicsInfo mdi;
    const double muscleTendonLength = getLength(s);
    const double muscleTendonVelocity = getLengtheningSpeed(s);

    // We set the boolean arguments to ignore tendon compliance in all three
    // function calls to true since we are computing rigid tendon fiber force.
    calcMuscleLengthInfoHelper(muscleTendonLength, true, mli);
    // The second boolean argument is to specify how to compute velocity
    // information for either explicit or implicit tendon compliance dynamics.
    // However, since we are already ignoring tendon compliance, velocity
    // information will be computed whether this argument is true or false.
    calcFiberVelocityInfoHelper(
            muscleTendonVelocity, activation, true, true, mli, fvi);
    calcMuscleDynamicsInfoHelper(
            activation, muscleTendonVelocity, true, mli, fvi, mdi);

    return mdi.activeFiberForce;
}

void DeGrooteFregly2016Muscle::computeInitialFiberEquilibrium(
        SimTK::State& s) const {
    if (get_ignore_tendon_compliance()) return;

    getModel().realizeVelocity(s);

    const auto& muscleTendonLength = getLength(s);
    const auto& muscleTendonVelocity = getLengtheningSpeed(s);
    const auto& activation = getActivation(s);

    // We have to use the implicit form of the model since the explicit form
    // will produce a zero residual for any guess of normalized tendon force.
    // The implicit form requires a value for normalized tendon force
    // derivative, so we'll set it to zero for simplicity.
    const SimTK::Real normTendonForceDerivative = 0.0;

    // Wrap residual function so it is a function of normalized tendon force
    // only.
    const auto calcResidual = [this, &muscleTendonLength, &muscleTendonVelocity,
                                      &normTendonForceDerivative, &activation](
                                      const SimTK::Real& normTendonForce) {
        return calcEquilibriumResidual(muscleTendonLength, muscleTendonVelocity,
                activation, normTendonForce, normTendonForceDerivative);
    };

    const auto equilNormTendonForce = solveBisection(calcResidual,
            m_minNormTendonForce, m_maxNormTendonForce, 1e-10, 100);

    setNormalizedTendonForce(s, equilNormTendonForce);

    // TODO not working as well as bisection.
    // const double tolerance = std::max(
    //        1e-8 * get_max_isometric_force(), SimTK::SignificantReal * 10);
    // int maxIterations = 1000;

    // try {
    //    auto result = estimateMuscleFiberState(activation,
    //            muscleTendonLength, muscleTendonVelocity,
    //            normTendonForceDerivative, tolerance, maxIterations);

    //    switch (result.first) {

    //    case Success_Converged:
    //        setNormalizedTendonForce(s, result.second["norm_tendon_force"]);
    //        setActuation(s, get_max_isometric_force() *
    //                                result.second["norm_tendon_force"]);
    //        break;

    //    case Warning_FiberAtLowerBound:
    //        printf("\n\nDeGrooteFregly2016Muscle static solution:"
    //               " %s is at its minimum fiber length of %f\n",
    //                getName().c_str(), result.second["fiber_length"]);
    //        setNormalizedTendonForce(s, result.second["norm_tendon_force"]);
    //        setActuation(s, get_max_isometric_force() *
    //                                result.second["norm_tendon_force"]);
    //        break;

    //    case Warning_FiberAtUpperBound:
    //        printf("\n\nDeGrooteFregly2016Muscle static solution:"
    //               " %s is at its maximum fiber length of %f\n",
    //                getName().c_str(), result.second["fiber_length"]);
    //        setNormalizedTendonForce(s, result.second["norm_tendon_force"]);
    //        setActuation(s, get_max_isometric_force() *
    //                                result.second["norm_tendon_force"]);
    //        break;

    //    case Failure_MaxIterationsReached:
    //        std::ostringstream ss;
    //        ss << "\n  Solution error " <<
    //        abs(result.second["solution_error"])
    //           << " exceeds tolerance of " << tolerance << "\n"
    //           << "  Newton iterations reached limit of " << maxIterations
    //           << "\n"
    //           << "  Activation is " << activation << "\n"
    //           << "  Fiber length is " << result.second["fiber_length"] <<
    //           "\n";
    //        OPENSIM_THROW_FRMOBJ(MuscleCannotEquilibrate, ss.str());
    //    }

    //} catch (const std::exception& x) {
    //    OPENSIM_THROW_FRMOBJ(MuscleCannotEquilibrate,
    //            "Internal exception encountered.\n" + std::string{x.what()});
    //}
}

// std::pair<DeGrooteFregly2016Muscle::StatusFromEstimateMuscleFiberState,
//        DeGrooteFregly2016Muscle::ValuesFromEstimateMuscleFiberState>
// DeGrooteFregly2016Muscle::estimateMuscleFiberState(const double activation,
//        const double muscleTendonLength, const double muscleTendonVelocity,
//        const double normTendonForceDerivative, const double tolerance,
//        const int maxIterations) const {
//
//    MuscleLengthInfo mli;
//    FiberVelocityInfo fvi;
//    MuscleDynamicsInfo mdi;
//
//    double normTendonForce = get_default_normalized_tendon_force();
//    calcMuscleLengthInfoHelper(muscleTendonLength,
//            get_ignore_tendon_compliance(), mli, normTendonForce);
//
//    double fiberLength = mli.fiberLength;
//    double residual = SimTK::MostPositiveReal;
//    double partialFiberForceAlongTendonPartialFiberLength = 0.0;
//    double partialTendonForcePartialFiberLength = 0.0;
//    double partialResidualPartialFiberLength = 0.0;
//    double deltaFiberLength = 0.0;
//
//    // Helper functions
//    // ----------------
//    // Update position level quantities.
//    auto positionFunc = [&] {
//        const auto& fiberLengthAlongTendon =
//                sqrt(SimTK::square(fiberLength) - m_squareFiberWidth);
//        const auto& tendonLength = muscleTendonLength -
//        fiberLengthAlongTendon; const auto& normTendonLength = tendonLength /
//        get_tendon_slack_length(); normTendonForce =
//        calcTendonForceMultiplier(normTendonLength);
//        calcMuscleLengthInfoHelper(muscleTendonLength,
//                get_ignore_tendon_compliance(), mli, normTendonForce);
//    };
//    // Update velocity and dynamics level quantities and compute residual.
//    auto dynamicsFunc = [&] {
//        calcFiberVelocityInfoHelper(muscleTendonVelocity, activation, false,
//                false, mli, fvi, normTendonForce, normTendonForceDerivative);
//        calcMuscleDynamicsInfoHelper(activation, muscleTendonVelocity, false,
//                mli, fvi, mdi, normTendonForce);
//
//        partialFiberForceAlongTendonPartialFiberLength =
//                mdi.userDefinedDynamicsExtras[
//                        m_mdi_partialFiberForceAlongTendonPartialFiberLength];
//        partialTendonForcePartialFiberLength = mdi.userDefinedDynamicsExtras[
//                m_mdi_partialTendonForcePartialFiberLength];
//
//        residual = calcEquilibriumResidual(
//                mdi.tendonForce, mdi.fiberForceAlongTendon);
//    };
//
//    // Initialize the loop.
//    int iter = 0;
//    positionFunc();
//    dynamicsFunc();
//    double residualPrev = residual;
//    double fiberLengthPrev = fiberLength;
//    double h = 1.0;
//
//    while ((abs(residual) > tolerance) && (iter < maxIterations)) {
//        // Compute the search direction.
//        partialResidualPartialFiberLength =
//                partialFiberForceAlongTendonPartialFiberLength -
//                partialTendonForcePartialFiberLength;
//        h = 1.0;
//
//        while (abs(residual) >= abs(residualPrev)) {
//            // Compute the Newton step.
//            deltaFiberLength =
//                    -h * residualPrev / partialResidualPartialFiberLength;
//
//            // Take a Newton step if the step is nonzero.
//            if (abs(deltaFiberLength) > SimTK::SignificantReal)
//                fiberLength = fiberLengthPrev + deltaFiberLength;
//            else {
//                // We've stagnated or hit a limit; assume we are hitting local
//                // minimum and attempt to approach from the other direction.
//                fiberLength = fiberLengthPrev -
//                              SimTK::sign(deltaFiberLength) * SimTK::SqrtEps;
//                h = 0;
//            }
//
//            if (fiberLength / get_optimal_fiber_length() <
//                    m_minNormFiberLength) {
//                fiberLength = m_minNormFiberLength *
//                get_optimal_fiber_length();
//            }
//            if (fiberLength / get_optimal_fiber_length() >
//                    m_maxNormFiberLength) {
//                fiberLength = m_maxNormFiberLength *
//                get_optimal_fiber_length();
//            }
//
//            positionFunc();
//            dynamicsFunc();
//
//            if (h <= SimTK::SqrtEps) { break; }
//            h = 0.5 * h;
//        }
//
//        residualPrev = residual;
//        fiberLengthPrev = fiberLength;
//
//        iter++;
//    }
//
//    // Populate the result map.
//    ValuesFromEstimateMuscleFiberState resultValues;
//
//    if (abs(residual) < tolerance) { // The solution converged.
//
//        resultValues.iterations = iter;
//        resultValues.solution_error = residual;
//        resultValues.fiber_length = fiberLength;
//        resultValues.fiber_velocity = fvi.fiberVelocity;
//        resultValues.normalized_tendon_force = mdi.normTendonForce;
//
//        return std::pair<StatusFromEstimateMuscleFiberState,
//                ValuesFromEstimateMuscleFiberState>(
//                Success_Converged, resultValues);
//    }
//
//    // Fiber length is at or exceeds its lower bound.
//    if (fiberLength / get_optimal_fiber_length() <= m_minNormFiberLength) {
//
//        fiberLength = m_minNormFiberLength * get_optimal_fiber_length();
//        positionFunc();
//        normTendonForce = calcTendonForceMultiplier(mli.normTendonLength);
//
//        resultValues.iterations = iter;
//        resultValues.solution_error = residual;
//        resultValues.fiber_length = fiberLength;
//        resultValues.fiber_velocity = 0;
//        resultValues.normalized_tendon_force = normTendonForce;
//
//        return std::pair<StatusFromEstimateMuscleFiberState,
//                ValuesFromEstimateMuscleFiberState>(
//                Warning_FiberAtLowerBound, resultValues);
//    }
//
//    // Fiber length is at or exceeds its upper bound.
//    if (fiberLength / get_optimal_fiber_length() >= m_maxNormFiberLength) {
//
//        fiberLength = m_maxNormFiberLength * get_optimal_fiber_length();
//        positionFunc();
//        normTendonForce = calcTendonForceMultiplier(mli.normTendonLength);
//
//        resultValues.iterations = iter;
//        resultValues.solution_error = residual;
//        resultValues.fiber_length = fiberLength;
//        resultValues.fiber_velocity = 0;
//        resultValues.normalized_tendon_force = normTendonForce;
//
//        return std::pair<StatusFromEstimateMuscleFiberState,
//                ValuesFromEstimateMuscleFiberState>(
//                Warning_FiberAtUpperBound, resultValues);
//    }
//
//    // Max iterations reached.
//    resultValues.iterations = iter;
//    resultValues.solution_error = residual;
//    resultValues.fiber_length = SimTK::NaN;
//    resultValues.fiber_velocity = SimTK::NaN;
//    resultValues.normalized_tendon_force = SimTK::NaN;
//
//    return std::pair<StatusFromEstimateMuscleFiberState,
//            ValuesFromEstimateMuscleFiberState>(
//            Failure_MaxIterationsReached, resultValues);
//}

double DeGrooteFregly2016Muscle::getPassiveFiberElasticForce(
        const SimTK::State& s) const {
    return getMuscleDynamicsInfo(s)
            .userDefinedDynamicsExtras[m_mdi_passiveFiberElasticForce];
}
double DeGrooteFregly2016Muscle::getPassiveFiberElasticForceAlongTendon(
        const SimTK::State& s) const {
    return getMuscleDynamicsInfo(s)
                   .userDefinedDynamicsExtras[m_mdi_passiveFiberElasticForce] *
           getMuscleLengthInfo(s).cosPennationAngle;
}
double DeGrooteFregly2016Muscle::getPassiveFiberDampingForce(
        const SimTK::State& s) const {
    return getMuscleDynamicsInfo(s)
            .userDefinedDynamicsExtras[m_mdi_passiveFiberDampingForce];
}
double DeGrooteFregly2016Muscle::getPassiveFiberDampingForceAlongTendon(
        const SimTK::State& s) const {
    return getMuscleDynamicsInfo(s)
                   .userDefinedDynamicsExtras[m_mdi_passiveFiberDampingForce] *
           getMuscleLengthInfo(s).cosPennationAngle;
}

double DeGrooteFregly2016Muscle::getImplicitResidualNormalizedTendonForce(
        const SimTK::State& s) const {
    if (get_ignore_tendon_compliance()) { return 0; }
    if (m_isTendonDynamicsExplicit) { return SimTK::NaN; }

    // Recompute residual if cache is invalid.
    if (!isCacheVariableValid(s, RESIDUAL_NORMALIZED_TENDON_FORCE_NAME)) {
        // Compute muscle-tendon equilibrium residual value to update the
        // cache variable.
        setCacheVariableValue(s, RESIDUAL_NORMALIZED_TENDON_FORCE_NAME,
                getEquilibriumResidual(s));
        markCacheVariableValid(s, RESIDUAL_NORMALIZED_TENDON_FORCE_NAME);
    }

    return getCacheVariableValue<double>(
            s, RESIDUAL_NORMALIZED_TENDON_FORCE_NAME);
}

DataTable DeGrooteFregly2016Muscle::exportFiberLengthCurvesToTable(
        const SimTK::Vector& normFiberLengths) const {
    SimTK::Vector def;
    const SimTK::Vector* x = nullptr;
    if (normFiberLengths.nrow()) {
        x = &normFiberLengths;
    } else {
        def = createVectorLinspace(
                200, m_minNormFiberLength, m_maxNormFiberLength);
        x = &def;
    }

    DataTable table;
    table.setColumnLabels(
            {"active_force_length_multiplier", "passive_force_multiplier"});
    SimTK::RowVector row(2);
    for (int irow = 0; irow < x->nrow(); ++irow) {
        const auto& normFiberLength = x->get(irow);
        row[0] = calcActiveForceLengthMultiplier(normFiberLength);
        row[1] = calcPassiveForceMultiplier(normFiberLength);
        table.appendRow(normFiberLength, row);
    }
    return table;
}

DataTable DeGrooteFregly2016Muscle::exportTendonForceMultiplierToTable(
        const SimTK::Vector& normTendonLengths) const {
    SimTK::Vector def;
    const SimTK::Vector* x = nullptr;
    if (normTendonLengths.nrow()) {
        x = &normTendonLengths;
    } else {
        // Evaluate the inverse of the tendon curve at y = 1.
        def = createVectorLinspace(
                200, 0.95, 1.0 + get_tendon_strain_at_one_norm_force());
        x = &def;
    }

    DataTable table;
    table.setColumnLabels({"tendon_force_multiplier"});
    SimTK::RowVector row(1);
    for (int irow = 0; irow < x->nrow(); ++irow) {
        const auto& normTendonLength = x->get(irow);
        row[0] = calcTendonForceMultiplier(normTendonLength);
        table.appendRow(normTendonLength, row);
    }
    return table;
}

DataTable DeGrooteFregly2016Muscle::exportFiberVelocityMultiplierToTable(
        const SimTK::Vector& normFiberVelocities) const {
    SimTK::Vector def;
    const SimTK::Vector* x = nullptr;
    if (normFiberVelocities.nrow()) {
        x = &normFiberVelocities;
    } else {
        def = createVectorLinspace(200, -1.1, 1.1);
        x = &def;
    }

    DataTable table;
    table.setColumnLabels({"force_velocity_multiplier"});
    SimTK::RowVector row(1);
    for (int irow = 0; irow < x->nrow(); ++irow) {
        const auto& normFiberVelocity = x->get(irow);
        row[0] = calcForceVelocityMultiplier(normFiberVelocity);
        table.appendRow(normFiberVelocity, row);
    }
    return table;
}

void DeGrooteFregly2016Muscle::printCurvesToSTOFiles(
        const std::string& directory) const {
    const std::string prefix =
            directory + SimTK::Pathname::getPathSeparator() + getName();
    STOFileAdapter::write(exportFiberLengthCurvesToTable(),
            prefix + "_fiber_length_curves.sto");
    STOFileAdapter::write(exportFiberVelocityMultiplierToTable(),
            prefix + "_fiber_velocity_multiplier.sto");
    STOFileAdapter::write(exportTendonForceMultiplierToTable(),
            prefix + "_tendon_force_multiplier.sto");
}

void DeGrooteFregly2016Muscle::replaceMuscles(
        Model& model, bool allowUnsupportedMuscles) {

    model.finalizeConnections();

    // Create path actuators from muscle properties and add to the model. Save
    // a list of pointers of the muscles to delete.
    std::vector<Muscle*> musclesToDelete;
    auto& muscleSet = model.updMuscles();
    for (int im = 0; im < muscleSet.getSize(); ++im) {
        auto& muscBase = muscleSet.get(im);

        // Pre-emptively create a default DeGrooteFregly2016Muscle
        // (TODO: not ideal to do this).
        auto actu = OpenSim::make_unique<DeGrooteFregly2016Muscle>();

        // Perform muscle-model-specific mappings or throw exception if the
        // muscle is not supported.
        if (auto musc = dynamic_cast<Millard2012EquilibriumMuscle*>(
                    &muscBase)) {

            actu->set_default_activation(musc->get_default_activation());
            actu->set_activation_time_constant(
                    musc->get_activation_time_constant());
            actu->set_deactivation_time_constant(
                    musc->get_deactivation_time_constant());
            actu->set_fiber_damping(musc->get_fiber_damping());
            actu->set_passive_fiber_strain_at_one_norm_force(
                    musc->get_FiberForceLengthCurve()
                            .get_strain_at_one_norm_force());
            actu->set_tendon_strain_at_one_norm_force(
                    musc->get_TendonForceLengthCurve()
                            .get_strain_at_one_norm_force());

        } else if (auto musc = dynamic_cast<Thelen2003Muscle*>(&muscBase)) {

            actu->set_default_activation(musc->getDefaultActivation());
            actu->set_activation_time_constant(
                    musc->get_activation_time_constant());
            actu->set_deactivation_time_constant(
                    musc->get_deactivation_time_constant());
            // Fiber damping needs to be hardcoded at zero since it is not a
            // property of the Thelen2003 muscle.
            actu->set_fiber_damping(0);
            actu->set_passive_fiber_strain_at_one_norm_force(
                    musc->get_FmaxMuscleStrain());

            actu->set_tendon_strain_at_one_norm_force(
                    musc->get_FmaxTendonStrain());

        } else {
            OPENSIM_THROW_IF(!allowUnsupportedMuscles, Exception,
                    "Muscle '{}' of type {} is unsupported and "
                    "allowUnsupportedMuscles=false.",
                    muscBase.getName(), muscBase.getConcreteClassName());
            continue;
        }

        // Perform all the common mappings at base class level (OpenSim::Muscle)
        actu->setName(muscBase.getName());
        muscBase.setName(muscBase.getName() + "_delete");
        actu->set_appliesForce(muscBase.get_appliesForce());
        actu->setMinControl(muscBase.getMinControl());
        actu->setMaxControl(muscBase.getMaxControl());

        actu->setMaxIsometricForce(muscBase.getMaxIsometricForce());
        actu->setOptimalFiberLength(muscBase.getOptimalFiberLength());
        actu->setTendonSlackLength(muscBase.getTendonSlackLength());
        actu->setPennationAngleAtOptimalFiberLength(
                muscBase.getPennationAngleAtOptimalFiberLength());
        actu->setMaxContractionVelocity(muscBase.getMaxContractionVelocity());
        actu->set_ignore_tendon_compliance(
                muscBase.get_ignore_tendon_compliance());
        actu->set_ignore_activation_dynamics(
                muscBase.get_ignore_activation_dynamics());

        const auto& pathPointSet = muscBase.getGeometryPath().getPathPointSet();
        auto& geomPath = actu->updGeometryPath();
        for (int ipp = 0; ipp < pathPointSet.getSize(); ++ipp) {
            auto* pathPoint = pathPointSet.get(ipp).clone();
            const auto& socketNames = pathPoint->getSocketNames();
            for (const auto& socketName : socketNames) {
                pathPoint->updSocket(socketName)
                        .connect(pathPointSet.get(ipp)
                                         .getSocket(socketName)
                                         .getConnecteeAsObject());
            }
            geomPath.updPathPointSet().adoptAndAppend(pathPoint);
        }

        const auto& pathWrapSet = muscBase.getGeometryPath().getWrapSet();
        for (int ipw = 0; ipw < pathWrapSet.getSize(); ++ipw) {
            auto* pathWrap = pathWrapSet.get(ipw).clone();
            const auto& socketNames = pathWrap->getSocketNames();
            for (const auto& socketName : socketNames) {
                pathWrap->updSocket(socketName)
                        .connect(pathWrapSet.get(ipw)
                                .getSocket(socketName)
                                .getConnecteeAsObject());
            }
            geomPath.updWrapSet().adoptAndAppend(pathWrap);
        }

        std::string actname = actu->getName();
        model.addForce(actu.release());

        musclesToDelete.push_back(&muscBase);
    }

    // Delete the muscles.
    for (const auto* musc : musclesToDelete) {
        int index = model.getForceSet().getIndex(musc, 0);
        OPENSIM_THROW_IF(index == -1, Exception,
                "Muscle with name {} not found in ForceSet.", musc->getName());
        bool success = model.updForceSet().remove(index);
        OPENSIM_THROW_IF(!success, Exception,
                "Attempt to remove muscle with "
                "name {} was unsuccessful.",
                musc->getName());
    }

    model.finalizeFromProperties();
    model.finalizeConnections();
}

void DeGrooteFregly2016Muscle::extendPostScale(
        const SimTK::State& s, const ScaleSet& scaleSet) {
    Super::extendPostScale(s, scaleSet);

    GeometryPath& path = upd_GeometryPath();
    if (path.getPreScaleLength(s) > 0.0)
    {
        double scaleFactor = path.getLength(s) / path.getPreScaleLength(s);
        upd_optimal_fiber_length() *= scaleFactor;
        upd_tendon_slack_length() *= scaleFactor;

        // Clear the pre-scale length that was stored in the GeometryPath.
        path.setPreScaleLength(s, 0.0);
    }
}
