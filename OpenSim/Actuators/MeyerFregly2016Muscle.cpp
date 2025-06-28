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

const std::string MeyerFregly2016Muscle::STATE_ACTIVATION_NAME("activation");

// We must define these variables in some compilation unit (pre-C++17).
// https://stackoverflow.com/questions/40690260/undefined-reference-error-for-static-constexpr-member?noredirect=1&lq=1
constexpr double MeyerFregly2016Muscle::b11;
constexpr double MeyerFregly2016Muscle::b21;
constexpr double MeyerFregly2016Muscle::b31;
constexpr double MeyerFregly2016Muscle::b41;
constexpr double MeyerFregly2016Muscle::b12;
constexpr double MeyerFregly2016Muscle::b22;
constexpr double MeyerFregly2016Muscle::b32;
constexpr double MeyerFregly2016Muscle::b42;
constexpr double MeyerFregly2016Muscle::b13;
constexpr double MeyerFregly2016Muscle::b23;
constexpr double MeyerFregly2016Muscle::b33;
constexpr double MeyerFregly2016Muscle::b43;
constexpr double MeyerFregly2016Muscle::m_minNormFiberLength;
constexpr double MeyerFregly2016Muscle::m_maxNormFiberLength;
constexpr int MeyerFregly2016Muscle::m_mdi_passiveFiberElasticForce;
constexpr int MeyerFregly2016Muscle::m_mdi_passiveFiberDampingForce;
constexpr int
        MeyerFregly2016Muscle::m_mdi_partialPennationAnglePartialFiberLength;
constexpr int MeyerFregly2016Muscle::
        m_mdi_partialFiberForceAlongTendonPartialFiberLength;
constexpr int
        MeyerFregly2016Muscle::m_mdi_partialTendonForcePartialFiberLength;

void MeyerFregly2016Muscle::constructProperties() {
    constructProperty_activation_time_constant(0.015);
    constructProperty_deactivation_time_constant(0.060);
    constructProperty_default_activation(0.5);
    constructProperty_active_force_width_scale(1.0);
    constructProperty_fiber_damping(0.0);
    constructProperty_ignore_passive_fiber_force(false);
    constructProperty_activation_dynamics_smoothing(0.1);
}

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

double MeyerFregly2016Muscle::computeActuation(const SimTK::State& s) const {
    const auto& mdi = getMuscleDynamicsInfo(s);
    setActuation(s, mdi.tendonForce);
    return mdi.tendonForce;
}

void MeyerFregly2016Muscle::calcMuscleLengthInfoHelper(
        const SimTK::Real& muscleTendonLength, MuscleLengthInfo& mli) const {

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
        const SimTK::Real& muscleTendonVelocity, const SimTK::Real& activation,
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
        const SimTK::Real& activation, const MuscleLengthInfo& mli,
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

    // Based on Millard2012EquilibriumMuscle::calcMusclePotentialEnergyInfo().

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

void MeyerFregly2016Muscle::calcMuscleLengthInfo(
        const SimTK::State& s, MuscleLengthInfo& mli) const {

    const auto& muscleTendonLength = getLength(s);
    calcMuscleLengthInfoHelper(muscleTendonLength, mli);

    if (mli.tendonLength < get_tendon_slack_length()) {
        // TODO the Millard model sets fiber velocity to zero when the
        //       tendon is buckling, but this may create a discontinuity.
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

double
OpenSim::MeyerFregly2016Muscle::calcInextensibleTendonActiveFiberForce(
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

DataTable MeyerFregly2016Muscle::exportFiberLengthCurvesToTable(
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

DataTable MeyerFregly2016Muscle::exportFiberVelocityMultiplierToTable(
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

void MeyerFregly2016Muscle::printCurvesToSTOFiles(
        const std::string& directory) const {
    const std::string prefix =
            directory + SimTK::Pathname::getPathSeparator() + getName();
    STOFileAdapter::write(exportFiberLengthCurvesToTable(),
            prefix + "_fiber_length_curves.sto");
    STOFileAdapter::write(exportFiberVelocityMultiplierToTable(),
            prefix + "_fiber_velocity_multiplier.sto");
}

void MeyerFregly2016Muscle::replaceMuscles(
        Model& model, bool allowUnsupportedMuscles) {

    model.finalizeConnections();

    // Create path actuators from muscle properties and add to the model. Save
    // a list of pointers of the muscles to delete.
    std::vector<Muscle*> musclesToDelete;
    auto& muscleSet = model.updMuscles();
    for (int im = 0; im < muscleSet.getSize(); ++im) {
        auto& muscBase = muscleSet.get(im);

        // Pre-emptively create a default MeyerFregly2016Muscle
        // (TODO: not ideal to do this).
        auto actu = std::make_unique<MeyerFregly2016Muscle>();

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

        } else if (auto musc = dynamic_cast<Thelen2003Muscle*>(&muscBase)) {

            actu->set_default_activation(musc->getDefaultActivation());
            actu->set_activation_time_constant(
                    musc->get_activation_time_constant());
            actu->set_deactivation_time_constant(
                    musc->get_deactivation_time_constant());
            // Fiber damping needs to be hardcoded at zero since it is not a
            // property of the Thelen2003 muscle.
            actu->set_fiber_damping(0);

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
        actu->updProperty_path().assign(muscBase.getProperty_path());
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

void MeyerFregly2016Muscle::extendPostScale(
        const SimTK::State& s, const ScaleSet& scaleSet) {
    Super::extendPostScale(s, scaleSet);

    AbstractGeometryPath& path = updPath();
    if (path.getPreScaleLength(s) > 0.0)
    {
        double scaleFactor = path.getLength(s) / path.getPreScaleLength(s);
        upd_optimal_fiber_length() *= scaleFactor;
        upd_tendon_slack_length() *= scaleFactor;

        // Clear the pre-scale length that was stored in the path.
        path.setPreScaleLength(s, 0.0);
    }
}