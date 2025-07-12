/* -------------------------------------------------------------------------- *
 *               Bhargava2004SmoothedMuscleMetabolics.cpp                     *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2020 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Antoine Falisse, Christopher Dembia, Nick Bianco                *
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

#include "Bhargava2004SmoothedMuscleMetabolics.h"

#include <SimTKcommon/internal/State.h>

#include <OpenSim/Common/Component.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

//=============================================================================
//  Bhargava2004Metabolics_MuscleParameters
//=============================================================================

Bhargava2004SmoothedMuscleMetabolics_MuscleParameters::
        Bhargava2004SmoothedMuscleMetabolics_MuscleParameters() {
    constructProperties();
}

// Set the muscle mass internal member variable muscleMass based on
// whether the use_provided_muscle_mass property is true or false.
void Bhargava2004SmoothedMuscleMetabolics_MuscleParameters::setMuscleMass() {
    if (get_use_provided_muscle_mass())
        muscleMass = get_provided_muscle_mass();
    else {
        muscleMass =
            (getMuscle().getMaxIsometricForce() / get_specific_tension())
            * get_density() * getMuscle().getOptimalFiberLength();
        }
}

void Bhargava2004SmoothedMuscleMetabolics_MuscleParameters::
        constructProperties() {

    // Specific tension of mammalian muscle (Pascals (N/m^2)).
    constructProperty_specific_tension(0.25e6);
    // Density of mammalian muscle (kg/m^3).
    constructProperty_density(1059.7);

    constructProperty_ratio_slow_twitch_fibers(0.5);

    constructProperty_use_provided_muscle_mass(false);
    constructProperty_provided_muscle_mass(SimTK::NaN);

    // Defaults (W/kg) from Bhargava et al (2004).
    constructProperty_activation_constant_slow_twitch(40.0);
    constructProperty_activation_constant_fast_twitch(133.0);
    constructProperty_maintenance_constant_slow_twitch(74.0);
    constructProperty_maintenance_constant_fast_twitch(111.0);
}

//=============================================================================
//  Bhargava2004Metabolics
//=============================================================================
Bhargava2004SmoothedMuscleMetabolics::Bhargava2004SmoothedMuscleMetabolics() {
    constructProperties();
    const int curvePoints = 5;
    const double curveX[] = {0.0, 0.5, 1.0, 1.5, 10.0};
    const double curveY[] = {0.5, 0.5, 1.0, 0.0, 0.0};
    m_fiberLengthDepCurve = PiecewiseLinearFunction(curvePoints, curveX,
            curveY, "defaultCurve");
}

// Add a muscle with default Bhargava2004Metabolics_MuscleParameters so that it
// can be included in the metabolics analysis. If no muscle mass is provided
// (or if set to NaN) a default approximation is used to calculate it.
void Bhargava2004SmoothedMuscleMetabolics::addMuscle(
        const std::string& name,
        const Muscle& muscle, double muscle_mass) {
    append_muscle_parameters(
            Bhargava2004SmoothedMuscleMetabolics_MuscleParameters());
    auto& mp = upd_muscle_parameters(
            getProperty_muscle_parameters().size() - 1);
    mp.setName(name);
    if (SimTK::isNaN(muscle_mass)) {
        mp.set_use_provided_muscle_mass(false);
    } else {
        mp.set_use_provided_muscle_mass(true);
        mp.set_provided_muscle_mass(muscle_mass);
    }
    mp.connectSocket_muscle(muscle);
    mp.setMuscleMass();
}

// Add a muscle with default Bhargava2004Metabolics_MuscleParameters except for
// slow twitch fibers ratio and specific tension so that it can be included in
// the metabolics analysis. If no muscle mass is provided (or if set to NaN) a
// default approximation is used to calculate it.
void Bhargava2004SmoothedMuscleMetabolics::addMuscle(const std::string& name,
        const Muscle& muscle, double ratio_slow_twitch_fibers,
        double specific_tension, double muscle_mass) {
    append_muscle_parameters(
            Bhargava2004SmoothedMuscleMetabolics_MuscleParameters());
    auto& mp = upd_muscle_parameters(
            getProperty_muscle_parameters().size() - 1);
    mp.setName(name);
    mp.set_ratio_slow_twitch_fibers(ratio_slow_twitch_fibers);
    mp.set_specific_tension(specific_tension);
    if (SimTK::isNaN(muscle_mass)) {
        mp.set_use_provided_muscle_mass(false);
    } else {
        mp.set_use_provided_muscle_mass(true);
        mp.set_provided_muscle_mass(muscle_mass);
    }
    mp.connectSocket_muscle(muscle);
    mp.setMuscleMass();
}

// Add a muscle and specifiy all Bhargava2004Metabolics_MuscleParameters. If
// no muscle mass is provided (or if set to NaN) a default approximation is
// used to calculate it.
void Bhargava2004SmoothedMuscleMetabolics::addMuscle(const std::string& name,
        const Muscle& muscle, double ratio_slow_twitch_fibers,
        double specific_tension, double activation_constant_slow_twitch,
        double activation_constant_fast_twitch,
        double maintenance_constant_slow_twitch,
        double maintenance_constant_fast_twitch,
        double muscle_mass) {
    append_muscle_parameters(
            Bhargava2004SmoothedMuscleMetabolics_MuscleParameters());
    auto& mp = upd_muscle_parameters(
            getProperty_muscle_parameters().size() - 1);
    mp.setName(name);
    mp.set_ratio_slow_twitch_fibers(ratio_slow_twitch_fibers);
    mp.set_specific_tension(specific_tension);
    mp.set_activation_constant_slow_twitch(activation_constant_slow_twitch);
    mp.set_activation_constant_fast_twitch(activation_constant_fast_twitch);
    mp.set_maintenance_constant_slow_twitch(maintenance_constant_slow_twitch);
    mp.set_maintenance_constant_fast_twitch(maintenance_constant_fast_twitch);
    if (SimTK::isNaN(muscle_mass)) {
        mp.set_use_provided_muscle_mass(false);
    } else {
        mp.set_use_provided_muscle_mass(true);
        mp.set_provided_muscle_mass(muscle_mass);
    }
    mp.connectSocket_muscle(muscle);
    mp.setMuscleMass();
}

void Bhargava2004SmoothedMuscleMetabolics::constructProperties() {
    constructProperty_muscle_parameters();

    constructProperty_enforce_minimum_heat_rate_per_muscle(true);

    constructProperty_use_force_dependent_shortening_prop_constant(false);
    constructProperty_basal_coefficient(1.2);
    constructProperty_basal_exponent(1.0);
    constructProperty_muscle_effort_scaling_factor(1.0);
    constructProperty_include_negative_mechanical_work(true);
    constructProperty_forbid_negative_total_power(true);

    constructProperty_use_smoothing(false);
    constructProperty_smoothing_type("tanh");
    constructProperty_velocity_smoothing(10);
    constructProperty_power_smoothing(10);
    constructProperty_heat_rate_smoothing(10);
}

void Bhargava2004SmoothedMuscleMetabolics::extendFinalizeFromProperties() {
    if (get_use_smoothing()) {
        m_tanh_conditional = [](const double& cond, const double& left,
                const double& right, const double& smoothing, const int&) {
            const double smoothed_binary = 0.5 + 0.5 * tanh(smoothing * cond);
            return left + (-left + right) * smoothed_binary;
        };
        if (get_smoothing_type() == "tanh") {
            m_conditional = m_tanh_conditional;

        } else if (get_smoothing_type() == "huber") {
            m_conditional = [](const double& cond, const double& left,
                    const double& right, const double& smoothing,
                    const int& direction) {
                const double offset = (direction == 1) ? left : right;
                const double scale = (right - left) / cond;
                const double delta = 1.0;
                const double state = direction * cond;
                const double shift = 0.5 * (1 / smoothing);
                const double y = smoothing * (state + shift);
                double f = 0;
                if (y < 0) f = offset;
                else if (y <= delta) f = 0.5 * y * y + offset;
                else  f = delta * (y - 0.5 * delta) + offset;
                return scale * (f/smoothing + offset * (1.0 - 1.0/smoothing));
            };
        }
    } else {
        m_conditional = [](const double& cond, const double& left,
                const double& right, const double& smoothing, const int&) {
            if (cond <= 0) {
                return left;
            } else {
                return right;
            }
        };
        m_tanh_conditional = m_conditional;
    }
}

void Bhargava2004SmoothedMuscleMetabolics::extendConnectToModel(
        Model& model) {
    Super::extendConnectToModel(model);
    const auto& muscleParameters = getProperty_muscle_parameters();
    for (int i = 0; i < muscleParameters.size(); ++i) {
        auto& mp = upd_muscle_parameters(i);
        mp.setMuscleMass();
    }
}

double Bhargava2004SmoothedMuscleMetabolics::getTotalMetabolicRate(
        const SimTK::State& s) const {
    // BASAL METABOLIC RATE (W) (based on whole body mass, not muscle mass).
    // ---------------------------------------------------------------------
    double Bdot = get_basal_coefficient()
            * pow(getModel().getMatterSubsystem().calcSystemMass(s),
                    get_basal_exponent());
    return getMetabolicRate(s).sum() + Bdot;
}

double Bhargava2004SmoothedMuscleMetabolics::getTotalActivationRate(
        const SimTK::State& s) const {
    return getActivationRate(s).sum();
}

double Bhargava2004SmoothedMuscleMetabolics::getTotalMaintenanceRate(
        const SimTK::State& s) const {
    return getMaintenanceRate(s).sum();
}

double Bhargava2004SmoothedMuscleMetabolics::getTotalShorteningRate(
        const SimTK::State& s) const {
    return getShorteningRate(s).sum();
}

double Bhargava2004SmoothedMuscleMetabolics::getTotalMechanicalWorkRate(
        const SimTK::State& s) const {
    return getMechanicalWorkRate(s).sum();
}

double Bhargava2004SmoothedMuscleMetabolics::getMuscleMetabolicRate(
        const SimTK::State& s, const std::string& channel) const {
    return getMetabolicRate(s).get(m_muscleIndices.at(channel));
}

void Bhargava2004SmoothedMuscleMetabolics::extendRealizeTopology(
        SimTK::State& state) const {
    Super::extendRealizeTopology(state);
    m_muscleIndices.clear();
    for (int i = 0; i < getProperty_muscle_parameters().size(); ++i) {
        const auto& muscle = get_muscle_parameters(i).getMuscle();
        if (muscle.get_appliesForce()) {
            m_muscleIndices[muscle.getAbsolutePathString()] = i;
        }
    }
}

void Bhargava2004SmoothedMuscleMetabolics::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    SimTK::Vector rates = SimTK::Vector((int)m_muscleIndices.size(), 0.0);
    addCacheVariable<SimTK::Vector>("metabolic_rate", rates,
            SimTK::Stage::Dynamics);
    addCacheVariable<SimTK::Vector>("activation_rate", rates,
            SimTK::Stage::Dynamics);
    addCacheVariable<SimTK::Vector>("maintenance_rate", rates,
            SimTK::Stage::Dynamics);
    addCacheVariable<SimTK::Vector>("shortening_rate", rates,
            SimTK::Stage::Dynamics);
    addCacheVariable<SimTK::Vector>("mechanical_work_rate", rates,
            SimTK::Stage::Dynamics);
}

void Bhargava2004SmoothedMuscleMetabolics::calcMetabolicRateForCache(
    const SimTK::State& s) const {
    calcMetabolicRate(s,
            updCacheVariableValue<SimTK::Vector>(s, "metabolic_rate"),
            updCacheVariableValue<SimTK::Vector>(s, "activation_rate"),
            updCacheVariableValue<SimTK::Vector>(s, "maintenance_rate"),
            updCacheVariableValue<SimTK::Vector>(s, "shortening_rate"),
            updCacheVariableValue<SimTK::Vector>(s, "mechanical_work_rate")
            );
    markCacheVariableValid(s, "metabolic_rate");
    markCacheVariableValid(s, "activation_rate");
    markCacheVariableValid(s, "maintenance_rate");
    markCacheVariableValid(s, "shortening_rate");
    markCacheVariableValid(s, "mechanical_work_rate");
}

const SimTK::Vector& Bhargava2004SmoothedMuscleMetabolics::getMetabolicRate(
        const SimTK::State& s) const {
    if (!isCacheVariableValid(s, "metabolic_rate")) {
        calcMetabolicRateForCache(s);
    }
    return getCacheVariableValue<SimTK::Vector>(s, "metabolic_rate");
}

const SimTK::Vector& Bhargava2004SmoothedMuscleMetabolics::getActivationRate(
        const SimTK::State& s) const {
    if (!isCacheVariableValid(s, "activation_rate")) {
        calcMetabolicRateForCache(s);
    }
    return getCacheVariableValue<SimTK::Vector>(s, "activation_rate");
}

const SimTK::Vector& Bhargava2004SmoothedMuscleMetabolics::getMaintenanceRate(
        const SimTK::State& s) const {
    if (!isCacheVariableValid(s, "maintenance_rate")) {
        calcMetabolicRateForCache(s);
    }
    return getCacheVariableValue<SimTK::Vector>(s, "maintenance_rate");
}

const SimTK::Vector& Bhargava2004SmoothedMuscleMetabolics::getShorteningRate(
        const SimTK::State& s) const {
    if (!isCacheVariableValid(s, "shortening_rate")) {
        calcMetabolicRateForCache(s);
    }
    return getCacheVariableValue<SimTK::Vector>(s, "shortening_rate");
}

const SimTK::Vector&
Bhargava2004SmoothedMuscleMetabolics::getMechanicalWorkRate(
        const SimTK::State& s) const {
    if (!isCacheVariableValid(s, "mechanical_work_rate")) {
        calcMetabolicRateForCache(s);
    }
    return getCacheVariableValue<SimTK::Vector>(s, "mechanical_work_rate");
}

void Bhargava2004SmoothedMuscleMetabolics::calcMetabolicRate(
        const SimTK::State& s, SimTK::Vector& totalRatesForMuscles,
        SimTK::Vector& activationRatesForMuscles,
        SimTK::Vector& maintenanceRatesForMuscles,
        SimTK::Vector& shorteningRatesForMuscles,
        SimTK::Vector& mechanicalWorkRatesForMuscles) const {
    totalRatesForMuscles.resize((int)m_muscleIndices.size());
    activationRatesForMuscles.resize((int)m_muscleIndices.size());
    maintenanceRatesForMuscles.resize((int)m_muscleIndices.size());
    shorteningRatesForMuscles.resize((int)m_muscleIndices.size());
    mechanicalWorkRatesForMuscles.resize((int)m_muscleIndices.size());
    double activationHeatRate, maintenanceHeatRate, shorteningHeatRate;
    double mechanicalWorkRate;
    activationHeatRate = maintenanceHeatRate = shorteningHeatRate =
        mechanicalWorkRate = 0;

    for (const auto& muscleIndex : m_muscleIndices) {

        const auto& index = muscleIndex.second;
        const auto& muscleParameter = get_muscle_parameters(index);
        const auto& muscle = muscleParameter.getMuscle();

        const double maximalIsometricForce = muscle.getMaxIsometricForce();
        const double activation =
            get_muscle_effort_scaling_factor() * muscle.getActivation(s);
        const double excitation =
            get_muscle_effort_scaling_factor() * muscle.getControl(s);
        const double fiberForcePassive =  muscle.getPassiveFiberForce(s);
        const double fiberForceActive =
            get_muscle_effort_scaling_factor() * muscle.getActiveFiberForce(s);
        const double fiberForceTotal =
            fiberForceActive + fiberForcePassive;
        const double fiberLengthNormalized =
            muscle.getNormalizedFiberLength(s);
        const double fiberVelocity = muscle.getFiberVelocity(s);
        const double slowTwitchExcitation =
            muscleParameter.get_ratio_slow_twitch_fibers()
            * sin(SimTK::Pi/2 * excitation);
        const double fastTwitchExcitation =
            (1 - muscleParameter.get_ratio_slow_twitch_fibers())
            * (1 - cos(SimTK::Pi/2 * excitation));
        // This small constant is added to the fiber velocity to prevent
        // dividing by 0 (in case the actual fiber velocity is null) when using
        // the Huber loss smoothing approach, thereby preventing singularities.
        const double eps = 1e-16;

        // Get the unnormalized total active force, isometricTotalActiveForce
        // that 'would' be developed at the current activation and fiber length
        // under isometric conditions (i.e., fiberVelocity=0).
        const double isometricTotalActiveForce =
            activation * muscle.getActiveForceLengthMultiplier(s)
            * maximalIsometricForce;

        // ACTIVATION HEAT RATE (W).
        // -------------------------
        // This value is set to 1.0, as used by Anderson & Pandy (1999),
        // however, in Bhargava et al., (2004) they assume a function here.
        // We will ignore this function and use 1.0 for now.
        const double decay_function_value = 1.0;
        activationHeatRate =
            muscleParameter.getMuscleMass() * decay_function_value
            * ( (muscleParameter.get_activation_constant_slow_twitch()
                        * slowTwitchExcitation)
                + (muscleParameter.get_activation_constant_fast_twitch()
                        * fastTwitchExcitation) );

        // MAINTENANCE HEAT RATE (W).
        // --------------------------
        const double fiber_length_dependence = m_fiberLengthDepCurve.calcValue(
                    SimTK::Vector(1, fiberLengthNormalized));
        maintenanceHeatRate =
            muscleParameter.getMuscleMass() * fiber_length_dependence
                * ( (muscleParameter.get_maintenance_constant_slow_twitch()
                            * slowTwitchExcitation)
                + (muscleParameter.get_maintenance_constant_fast_twitch()
                            * fastTwitchExcitation) );

        // SHORTENING HEAT RATE (W).
        // --> note that we define fiberVelocity<0 as shortening and
        //     fiberVelocity>0 as lengthening.
        // ---------------------------------------------------------
        double alpha;
        if (get_use_force_dependent_shortening_prop_constant()) {
            // Even when using the Huber loss smoothing approach, we still rely
            // on a tanh approximation for the shortening heat rate when using
            // the force dependent shortening proportional constant. This is
            // motivated by the fact that the shortening heat rate is defined
            // by linear functions but with different non-zero constants of
            // proportionality for concentric and eccentric contractions. It is
            // therefore easier to smooth the transition between both
            // contraction types with a tanh function than with a Huber loss
            // function.
            alpha = m_tanh_conditional(fiberVelocity + eps,
                    (0.16 * isometricTotalActiveForce)
                    + (0.18 * fiberForceTotal),
                    0.157 * fiberForceTotal,
                    get_velocity_smoothing(),
                    -1);
        } else {
            // This simpler value of alpha comes from Frank Anderson's 1999
            // dissertation "A Dynamic Optimization Solution for a Complete
            // Cycle of Normal Gait".
            alpha = m_conditional(fiberVelocity + eps,
                    0.25 * fiberForceTotal,
                    0,
                    get_velocity_smoothing(),
                    -1);
        }
        shorteningHeatRate = -alpha * (fiberVelocity + eps);

        // MECHANICAL WORK RATE for the contractile element of the muscle (W).
        // --> note that we define fiberVelocity<0 as shortening and
        //     fiberVelocity>0 as lengthening.
        // -------------------------------------------------------------------
        if (get_include_negative_mechanical_work())
        {
            mechanicalWorkRate = -fiberForceActive * fiberVelocity;
        } else {
            mechanicalWorkRate = m_conditional(fiberVelocity + eps,
                    -fiberForceActive * fiberVelocity,
                    0,
                    get_velocity_smoothing(),
                    -1);
        }

        // NAN CHECKING
        // ------------------------------------------
        if (SimTK::isNaN(activationHeatRate))
            std::cout << "WARNING::" << getName() << ": activationHeatRate ("
                    << muscleParameter.getName() << ") = NaN!" << std::endl;
        if (SimTK::isNaN(maintenanceHeatRate))
            std::cout << "WARNING::" << getName() << ": maintenanceHeatRate ("
                    << muscleParameter.getName() << ") = NaN!" << std::endl;
        if (SimTK::isNaN(shorteningHeatRate))
            std::cout << "WARNING::" << getName() << ": shorteningHeatRate ("
                    << muscleParameter.getName() << ") = NaN!" << std::endl;
        if (SimTK::isNaN(mechanicalWorkRate))
            std::cout << "WARNING::" << getName() << ": mechanicalWorkRate ("
                    <<  muscleParameter.getName() << ") = NaN!" << std::endl;

        // If necessary, increase the shortening heat rate so that the total
        // power is non-negative.
        if (get_forbid_negative_total_power()) {
            const double Edot_W_beforeClamp = activationHeatRate
                + maintenanceHeatRate + shorteningHeatRate
                + mechanicalWorkRate;
            if (get_use_smoothing()) {
                const double Edot_W_beforeClamp_smoothed = m_conditional(
                        -Edot_W_beforeClamp,
                        0,
                        Edot_W_beforeClamp,
                        get_power_smoothing(),
                        1);
                shorteningHeatRate -= Edot_W_beforeClamp_smoothed;
            } else {
                if (Edot_W_beforeClamp < 0)
                    shorteningHeatRate -= Edot_W_beforeClamp;
            }
        }

        // This check is adapted from Umberger(2003), page 104: the total heat
        // rate (i.e., activationHeatRate + maintenanceHeatRate
        // + shorteningHeatRate) for a given muscle cannot fall below 1.0 W/kg.
        // If the total heat rate falls below 1.0 W/kg, the sum of the reported
        // individual heat rates and work rate does not equal the reported
        // metabolic rate.
        // --------------------------------------------------------------------
        double totalHeatRate = activationHeatRate + maintenanceHeatRate
            + shorteningHeatRate;
        if (get_use_smoothing()) {
            if (get_enforce_minimum_heat_rate_per_muscle())
            {
                totalHeatRate = m_conditional(
                        -totalHeatRate + 1.0 * muscleParameter.getMuscleMass(),
                        totalHeatRate,
                        1.0 * muscleParameter.getMuscleMass(),
                        get_heat_rate_smoothing(),
                        1);
            }
        } else {
            if (get_enforce_minimum_heat_rate_per_muscle()
                    && totalHeatRate < 1.0 * muscleParameter.getMuscleMass())
            {
                totalHeatRate = 1.0 * muscleParameter.getMuscleMass();
            }
        }

        // TOTAL METABOLIC ENERGY RATE (W).
        // --------------------------------
        double Edot = totalHeatRate + mechanicalWorkRate;

        totalRatesForMuscles[index] = Edot;
        activationRatesForMuscles[index] = activationHeatRate;
        maintenanceRatesForMuscles[index] = maintenanceHeatRate;
        shorteningRatesForMuscles[index] = shorteningHeatRate;
        mechanicalWorkRatesForMuscles[index] = mechanicalWorkRate;
    }
}

int Bhargava2004SmoothedMuscleMetabolics::getNumMetabolicMuscles() const {
    return getProperty_muscle_parameters().size();
}
