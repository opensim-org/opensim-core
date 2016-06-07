/* -------------------------------------------------------------------------- *
 *                 OpenSim:  UchidaUmbergerMuscleMetabolics.cpp               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
 * Author(s): Chris Dembia                                                    *
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

#include "UchidaUmbergerMuscleMetabolics.h"
#include "Model.h"
#include "Umberger2010MuscleMetabolicsProbe.h"

using namespace OpenSim;

// ============================================================================
// UchidaUmbergerMuscleMetabolics
// ============================================================================
UchidaUmbergerMuscleMetabolics::UchidaUmbergerMuscleMetabolics() {
    constructInfrastructure();
}
void UchidaUmbergerMuscleMetabolics::constructProperties() {
    constructProperty_muscle_reps();
    
    constructProperty_enforce_minimum_heat_rate_per_muscle(true);
    
    constructProperty_aerobic_factor(1.5);
    constructProperty_basal_coefficient(1.2);
    constructProperty_basal_exponent(1.0);
    
    constructProperty_use_Bhargava_recruitment_model(true);
    constructProperty_include_negative_mechanical_work(true);
    constructProperty_forbid_negative_total_power(true);
}

double UchidaUmbergerMuscleMetabolics::
calcBasalRate(const SimTK::State& s) const {
    const auto& basalCoeff = get_basal_coefficient();
    const auto& sysMass = getModel().getMatterSubsystem().calcSystemMass(s);
    const auto& basalExponent = get_basal_exponent();
    return basalCoeff * pow(sysMass, basalExponent);
}
double UchidaUmbergerMuscleMetabolics::
calcWholeBodyRate(const SimTK::State& s) const {
    return calcBasalRate(s) + calcSumTotalRate(s);
}
double UchidaUmbergerMuscleMetabolics::
calcSumTotalRate(const SimTK::State& s) const {
    double result = 0;
    for (int i = 0; i < getProperty_muscle_reps().size(); ++i) {
        result += get_muscle_reps(i).calcTotalRate(s);
    }
    return result;
}
double UchidaUmbergerMuscleMetabolics::
calcSumActivationMaintenanceRate(const SimTK::State& s) const {
    double result = 0;
    for (int i = 0; i < getProperty_muscle_reps().size(); ++i) {
        result += get_muscle_reps(i).calcActivationMaintenanceRate(s);
    }
    return result;
}
double UchidaUmbergerMuscleMetabolics::
calcSumShorteningRate(const SimTK::State& s) const {
    double result = 0;
    for (int i = 0; i < getProperty_muscle_reps().size(); ++i) {
        result += get_muscle_reps(i).calcShorteningRate(s);
    }
    return result;
}
double UchidaUmbergerMuscleMetabolics::
calcSumMechanicalWorkRate(const SimTK::State& s) const {
    double result = 0;
    for (int i = 0; i < getProperty_muscle_reps().size(); ++i) {
        result += get_muscle_reps(i).calcMechanicalWorkRate(s);
    }
    return result;
}
double UchidaUmbergerMuscleMetabolics::
calcInstantaneousEfficiency(const SimTK::State& s) const {
    // TODO
    return SimTK::NaN;
}

/* static */ UchidaUmbergerMuscleMetabolics UchidaUmbergerMuscleMetabolics::
createFromProbe(const Umberger2010MuscleMetabolicsProbe& probe) {
    UchidaUmbergerMuscleMetabolics met;
    
    // Aggregate properties.
    met.set_enforce_minimum_heat_rate_per_muscle(
            probe.get_enforce_minimum_heat_rate_per_muscle());
    met.set_aerobic_factor(probe.get_aerobic_factor());
    met.set_basal_coefficient(probe.get_basal_coefficient());
    met.set_basal_exponent(probe.get_basal_exponent());
    met.set_use_Bhargava_recruitment_model(probe.get_use_Bhargava_recruitment_model());
    met.set_include_negative_mechanical_work(probe.get_include_negative_mechanical_work());
    met.set_forbid_negative_total_power(probe.get_forbid_negative_total_power());
    
    // Individual muscles.
    const auto& paramSet =
        probe.get_Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameterSet();
    for (int i = 0; i < paramSet.getSize(); ++i) {
        const auto& params = paramSet[i];
    
        met.append_muscle_reps(MuscleRep());
        // Get the MuscleRep that we just appended.
        auto& rep = met.upd_muscle_reps(i);
        
        // The name of the muscle to connect to is just the name of the
        // Umberger2010MuscleMetabolicsProbe_MetabolicMuscleParameter object.
        // TODO this should in the future be a valid path name, not just
        // a muscle name.
        rep.updConnector<Muscle>("muscle").setConnecteeName(params.getName());
        rep.set_specific_tension(params.get_specific_tension());
        rep.set_density(params.get_density());
        rep.set_ratio_slow_twitch_fibers(params.get_specific_tension());
        rep.set_use_provided_muscle_mass(params.get_use_provided_muscle_mass());
        rep.set_provided_muscle_mass(params.get_provided_muscle_mass());
    }
    return met;
}

// ============================================================================
// MuscleRep
// ============================================================================
UchidaUmbergerMuscleMetabolics::MuscleRep::MuscleRep() {
    constructInfrastructure();
}

void UchidaUmbergerMuscleMetabolics::MuscleRep::constructProperties() {
    // (Pascals (N/m^2)), specific tension of mammalian muscle.
    constructProperty_specific_tension(0.25e6);
    // (kg/m^3), density of mammalian muscle.
    constructProperty_density(1059.7);
    constructProperty_ratio_slow_twitch_fibers(0.5);
    constructProperty_use_provided_muscle_mass(false);
    constructProperty_provided_muscle_mass(SimTK::NaN);
}

void UchidaUmbergerMuscleMetabolics::MuscleRep::
extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();
    
    // TODO Cannot get the actual muscle yet since we have not connected to it yet.
    // TODO check if we should use the muscle connector to wire up the inputs.
    const auto& muscPath = getConnector<Muscle>("muscle").getConnecteeName();
    
    if (!muscPath.empty()) {
        std::vector<std::string> muscleInputNames {"activation", "excitation",
                "active_fiber_force", "normalized_fiber_length",
                "fiber_velocity", "active_force_length_multiplier" };
        for (const auto& inputName : muscleInputNames) {
            updInput(inputName).setConnecteeName(muscPath + "/" + inputName);
        }
    }
}

void UchidaUmbergerMuscleMetabolics::MuscleRep::
extendConnectToModel(OpenSim::Model& model) {
    OPENSIM_THROW_IF_FRMOBJ(
        !dynamic_cast<const UchidaUmbergerMuscleMetabolics*>(&getParent()),
        Exception, "This compoment must be a subcomponent of "
                   "UchidaUmbergerMuscleMetabolics.");
        
    if (get_use_provided_muscle_mass()) {
        _muscMass = get_provided_muscle_mass();
    } else {
        const auto& muscle = getConnectee<Muscle>("muscle");
        _muscMass = (muscle.getMaxIsometricForce() / get_specific_tension())
                    * get_density() * muscle.getOptimalFiberLength();
    }
}

double UchidaUmbergerMuscleMetabolics::MuscleRep::getMuscleMass() const {
    return _muscMass;
}

void UchidaUmbergerMuscleMetabolics::MuscleRep::extendAddToSystem(
        SimTK::MultibodySystem& sys) const {
    Super::extendAddToSystem(sys);
    // TODO add cache variables.
}

double UchidaUmbergerMuscleMetabolics::MuscleRep::calcTotalRate(
        const SimTK::State& s) const {
    return calcTotalHeatRate(s) + calcMechanicalWorkRate(s);
}

double UchidaUmbergerMuscleMetabolics::MuscleRep::calcTotalHeatRate(
        const SimTK::State& s) const {
    double sumHeatRate = calcActivationMaintenanceRate(s) +
                         calcShorteningRate(s);
    
    const double minHeatRate = 1.0 * getMuscleMass(); // that is, 1.0 W/kg.
    
    if (getParent().get_enforce_minimum_heat_rate_per_muscle() &&
            sumHeatRate < minHeatRate) {
        return minHeatRate;
    }
    return sumHeatRate;
}

double UchidaUmbergerMuscleMetabolics::MuscleRep::calcActivationMaintenanceRate(
        const SimTK::State& s) const {
    // TODO
    
    // Constants.
    const double& mass = getMuscleMass();
    
    // Inputs.
    const double excitation = getInput<double>("excitation").getValue(s);
    const double activation = getInput<double>("activation").getValue(s);
    const double normalized_fiber_length =
                getInput<double>("normalized_fiber_length").getValue(s);
    const double F_iso =
                getInput<double>("active_force_length_multiplier").getValue(s);
    
    // Compute intermediate quantities.
    const double A = findActivationDependenceScalingFactor(excitation,
                                                           activation);
    const double slowTwitchRatio = findSlowTwitchRatio(excitation, activation);
    
    const double A_AM = std::pow(A, 0.6);
    const double A_AM_S = A_AM * getParent().get_aerobic_factor();
    const double unscaledAMdot = 128 * (1 - slowTwitchRatio) + 25;
    
    // Umberger does not have a name for this term in his paper.
    double fiberLengthScalingFactor = 1;
    if (normalized_fiber_length > 1.0) {
        fiberLengthScalingFactor = 0.4 + 0.6 * F_iso;
    }
    
    return mass * (unscaledAMdot * fiberLengthScalingFactor * A_AM_S);
}

double UchidaUmbergerMuscleMetabolics::MuscleRep::calcShorteningRate(
        const SimTK::State& s) const {
    double unclampedShorteningRate = calcUnclampedShorteningRate(s);
    double totalUnclampedRate = calcActivationMaintenanceRate(s) +
                                unclampedShorteningRate +
                                calcMechanicalWorkRate(s);
    if (getParent().get_forbid_negative_total_power()
            && totalUnclampedRate < 0) {
        return -totalUnclampedRate;
    }
    return unclampedShorteningRate;
}

double UchidaUmbergerMuscleMetabolics::MuscleRep::calcUnclampedShorteningRate(
        const SimTK::State& s) const {
    // TODO
    const auto& muscle = getConnectee<Muscle>("muscle");
    
    // Constants.
    const double& maxShorteningVelocity_Lopt_per_sec = muscle.getMaxContractionVelocity();
    const double Vmax_fastTwitch = maxShorteningVelocity_Lopt_per_sec;
    const double Vmax_slowTwitch = maxShorteningVelocity_Lopt_per_sec / 2.5;
    const double alpha_shortening_fastTwitch = 153 / Vmax_fastTwitch;
    const double alpha_shortening_slowTwitch = 100 / Vmax_slowTwitch;
    const double& mass = getMuscleMass();
    // This parameter is different in the 2003 and 2010 models.
    const double alpha_lengthening =
            (getParent().get_include_negative_mechanical_work() ? 4.0 : 0.3)
            * alpha_shortening_slowTwitch;
    
    // Inputs.
    const double excitation = getInput<double>("excitation").getValue(s);
    const double activation = getInput<double>("activation").getValue(s);
    const double normalized_fiber_length =
                getInput<double>("normalized_fiber_length").getValue(s);
    const double F_iso =
                getInput<double>("active_force_length_multiplier").getValue(s);
    const double fiberVel = getInput<double>("fiber_velocity").getValue(s);
    
    const double fiberVel_Lopt_per_sec = fiberVel / muscle.getOptimalFiberLength();
    
    // Intermediate quantities.
    const double A = findActivationDependenceScalingFactor(excitation,
                                                           activation);
    const double slowTwitchRatio = findSlowTwitchRatio(excitation, activation);
    
    // Fiber velocity dependence.
    double specificRate = getParent().get_aerobic_factor();
    if (fiberVel_Lopt_per_sec <= 0) {
        // concentric contraction; Vm <= 0
        const double maxShorteningRate = 100.0;
        
        const double rate_slowTwitch = std::min(maxShorteningRate,
                -alpha_shortening_slowTwitch * fiberVel_Lopt_per_sec);
        const double rate_fastTwitch =
                alpha_shortening_fastTwitch * fiberVel_Lopt_per_sec
                                            * (1 - slowTwitchRatio);
        
        specificRate *= (rate_slowTwitch * slowTwitchRatio) - rate_fastTwitch;
        specificRate *= std::pow(A, 2.0);
        
    } else {
        specificRate *= alpha_lengthening * fiberVel_Lopt_per_sec;
        specificRate *= A;
    }
    
    // Fiber length dependence.
    if (normalized_fiber_length > 1.0) {
        specificRate *= F_iso;
    }
    
    return mass * specificRate;
}

double UchidaUmbergerMuscleMetabolics::MuscleRep::calcMechanicalWorkRate(
        const SimTK::State& s) const {
    // TODO change getInput calls to return references.
    const double fiberVel = getInput<double>("fiber_velocity").getValue(s);
    // TODO consider muscle effort scaling factor.
    const double activeFiberForce =
            getInput<double>("active_fiber_force").getValue(s);
    
    // We define Vm < 0 as shortening.
    if (getParent().get_include_negative_mechanical_work() || fiberVel <= 0) {
        return -activeFiberForce * fiberVel;
    }
    return 0;
}

double UchidaUmbergerMuscleMetabolics::MuscleRep::
calcInstantaneousEfficiency(const SimTK::State& s) const {
    // TODO the numerator should be the actual work done, not just
    // the mechanical work rate used to compute heat.
    return SimTK::NaN; // calcMechanicalWorkRate(s) / calcTotalRate(s);
    // return getInput<double>("fiber_work_rate").getValue(s) / calcTotalRate(s);
}

const UchidaUmbergerMuscleMetabolics& UchidaUmbergerMuscleMetabolics::MuscleRep::
getParent() const {
    return dynamic_cast<const UchidaUmbergerMuscleMetabolics&>(
            Component::getParent());
}

double UchidaUmbergerMuscleMetabolics::MuscleRep::
findActivationDependenceScalingFactor(const double& excitation,
                                      const double& activation) const {
    if (excitation > activation) { return excitation; }
    else { return 0.5 * (excitation + activation); }
}

double UchidaUmbergerMuscleMetabolics::MuscleRep::
findSlowTwitchRatio(const double& excitation, const double& activation) const {
    using namespace SimTK;
    double slowTwitchRatio = get_ratio_slow_twitch_fibers();
    if (getParent().get_use_Bhargava_recruitment_model()) {
        const double uSlow = slowTwitchRatio * sin(0.5 * Pi * excitation);
        const double uFast = (1 - slowTwitchRatio)
                             * (1 - cos(0.5 * Pi * excitation));
        slowTwitchRatio = (excitation == 0) ? 1.0 : uSlow / (uSlow + uFast);
    }
    return slowTwitchRatio;
}