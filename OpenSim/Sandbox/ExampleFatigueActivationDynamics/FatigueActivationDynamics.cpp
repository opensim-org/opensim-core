/* -------------------------------------------------------------------------- *
 *              OpenSim:  FatigueMuscleActivationDynamics.cpp              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
 * Author(s): Jennifer Yong, Apoorva Rajagopal                       *
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

#include "FatigueMuscleActivationDynamics.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

const std::string FatigueMuscleActivationDynamics::STATE_NAME_ACTIVATION = "fatigue_activation";
const std::string FatigueMuscleActivationDynamics::STATE_NAME_TARGETACTIVATION = "target_activation";
const std::string FatigueMuscleActivationDynamics::STATE_NAME_FATIGUEMOTORUNITS = "fatigue_motor_units";
const std::string FatigueMuscleActivationDynamics::STATE_NAME_ACTIVEMOTORUNITS = "active_motor_units";
const std::string FatigueMuscleActivationDynamics::STATE_NAME_RESTINGMOTORUNITS = "resting_motor_units";


//==============================================================================
// CONSTRUCTORS
//==============================================================================
FatigueMuscleActivationDynamics::FatigueMuscleActivationDynamics() :
MuscleActivationDynamics()
{
    setNull();
    constructProperties();
    setName("default_FatigueMuscleActivationDynamics");
}

FatigueMuscleActivationDynamics::
FatigueMuscleActivationDynamics(const std::string& name,
                                   ExcitationGetter* getter) :
MuscleActivationDynamics(name, getter)
{
    setNull();
    constructProperties();
}

FatigueMuscleActivationDynamics::FatigueMuscleActivationDynamics(double fatigueFactor, double recoveryFactor, double recruitmentFromRestingTimeConstant) :
MuscleActivationDynamics()
{
    setNull();
    constructProperties();
    setName("FatigueMuscleActivationDynamics_setFactors");
    setFatigueFactor(fatigueFactor);
    setRecoveryFactor(recoveryFactor);
    setRecruitmentFromRestingTimeConstant(recruitmentFromRestingTimeConstant);
    
}


//==============================================================================
// ACCESSORS AND MUTATORS
//==============================================================================
double FatigueMuscleActivationDynamics::getActivationTimeConstant() const
{   return get_activation_time_constant(); }
double FatigueMuscleActivationDynamics::getDeactivationTimeConstant() const
{   return get_deactivation_time_constant(); }
double FatigueMuscleActivationDynamics::getFatigueFactor() const
{   return get_fatigue_factor(); }
double FatigueMuscleActivationDynamics::getRecoveryFactor() const
{   return get_recovery_factor(); }
double FatigueMuscleActivationDynamics::getRecruitmentFromRestingTimeConstant() const
{   return get_recruitment_from_resting_time_constant(); }
double FatigueMuscleActivationDynamics::getDefaultActiveMotorUnits() const
{   return get_default_active_motor_units(); }
double FatigueMuscleActivationDynamics::getDefaultFatigueMotorUnits() const
{   return get_default_fatigue_motor_units(); }
double FatigueMuscleActivationDynamics::getDefaultRestingMotorUnits() const
{   return get_default_resting_motor_units(); }

void FatigueMuscleActivationDynamics::
setActivationTimeConstant(double activationTimeConstant)
{
    set_activation_time_constant(max(SimTK::SignificantReal,
                                     activationTimeConstant));
}

void FatigueMuscleActivationDynamics::
setDeactivationTimeConstant(double deactivationTimeConstant)
{
    set_deactivation_time_constant(max(SimTK::SignificantReal,
                                       deactivationTimeConstant));
}

void FatigueMuscleActivationDynamics::setFatigueFactor(double fatigueFactor)
{
    set_fatigue_factor(max(SimTK::SignificantReal, fatigueFactor));
}

void FatigueMuscleActivationDynamics::setRecoveryFactor(double recoveryFactor)
{
    set_recovery_factor(max(SimTK::SignificantReal, recoveryFactor));
}

void FatigueMuscleActivationDynamics::setRecruitmentFromRestingTimeConstant(double recruitmentFromRestingTimeConstant)
{
    set_recruitment_from_resting_time_constant(max(SimTK::SignificantReal,recruitmentFromRestingTimeConstant));
}

void FatigueMuscleActivationDynamics::setDefaultActiveMotorUnits(double defaultActiveMotorUnits)
{
    set_default_active_motor_units(min(max(SimTK::SignificantReal, defaultActiveMotorUnits),1));
}

void FatigueMuscleActivationDynamics::setDefaultFatigueMotorUnits(double defaultFatigueMotorUnits)
{
    set_default_fatigue_motor_units(min(max(SimTK::SignificantReal, defaultActiveMotorUnits),1));
}

void FatigueMuscleActivationDynamics::setDefaultRestingMotorUnits(double defaultRestingMotorUnits)
{
    set_default_resting_motor_units(min(max(SimTK::SignificantReal, defaultRestingMotorUnits),1));
}


//==============================================================================
// MODELCOMPONENT INTERFACE REQUIREMENTS
//==============================================================================
void FatigueMuscleActivationDynamics::
extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
    addStateVariable(STATE_NAME_ACTIVATION, SimTK::Stage::Dynamics);
    addStateVariable(STATE_NAME_TARGETACTIVATION, SimTK::Stage::Dynamics);
    addStateVariable(STATE_NAME_FATIGUEMOTORUNITS, SimTK::Stage::Dynamics);
    addStateVariable(STATE_NAME_ACTIVEMOTORUNITS, SimTK::Stage::Dynamics);
    addStateVariable(STATE_NAME_RESTINGMOTORUNITS, SimTK::Stage::Dynamics);
}

void FatigueMuscleActivationDynamics::
extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);
    setTargetActivation(s, getDefaultActivation());
    setActivation(s, getDefaultActivation()*(1 - getDefaultFatigueMotorUnits));
    setActiveMotorUnits(s, getDefaultActiveMotorUnits());
    setFatigueMotorUnits(s, getDefaultFatigueMotorUnits());
    setRestingMotorUnits(s, getDefaultRestingMotorUnits());
}

void FatigueMuscleActivationDynamics::
extendSetPropertiesFromState(const SimTK::State& s)
{
    Super::extendSetPropertiesFromState(s);
    setDefaultActivation(getActivation(s));
    setDefaultActiveMotorUnits(getActiveMotorUnits(s));
    setDefaultFatigueMotorUnits(getFatigueMotorUnits(s));
    setDefaultRestingMotorUnits(getRestingMotorUnits(s));
}

void FatigueMuscleActivationDynamics::
computeStateVariableDerivatives(const SimTK::State& s) const
{
    double aTarget_dot = calcTargetActivationDerivative(getExcitation(s), getTargetActivation(s));
    double a_dot = calcActivationDerivative(getExcitation(s), getTargetActivation(s), getFatigueMotorUnits(s), getActiveMotorUnits(s));
    double fatigueMU_dot = calcFatigueMotorUnitsDeriv(getFatigueMotorUnits(s), getActiveMotorUnits(s));
    double activeMU_dot = calcActiveMotorUnitsDeriv(getExcitation(s), getTargetActivation(s), getActiveMotorUnits(s), getFatigueMotorUnits(s));
    double restingMU_dot = calcRestingMotorUnitsDeriv(fatigueMU_dot,activeMU_dot);
    
    setStateVariableDerivativeValue(s, STATE_NAME_TARGETACTIVATION, aTarget_dot);
    setStateVariableDerivativeValue(s, STATE_NAME_ACTIVATION, a_dot);
    setStateVariableDerivativeValue(s, STATE_NAME_FATIGUEMOTORUNITS, fatigueMU_dot);
    setStateVariableDerivativeValue(s, STATE_NAME_ACTIVEMOTORUNITS, activeMU_dot);
    setStateVariableDerivativeValue(s, STATE_NAME_RESTINGMOTORUNITS, restingMU_dot);
}

//==============================================================================
// STATE-DEPENDENT METHODS
//==============================================================================
double FatigueMuscleActivationDynamics::
getActivation(const SimTK::State& s) const
{
    return clampToValidInterval(getStateVariableValue(s, STATE_NAME_ACTIVATION));
}

void FatigueMuscleActivationDynamics::setActivation(SimTK::State& s,
                                                       double activation) const
{
    setStateVariableValue(s, STATE_NAME_ACTIVATION,
                          clampToValidInterval(activation));
}

// TO DO (*#&$(*&#Q$()*&@#()$*&@#(*$&@)#(*&$(*)@#&$*(
// SERIOUSLY
// JENNY NEEDS TO DO THIS :D

//==============================================================================
// PRIVATE METHODS
//==============================================================================
void FatigueMuscleActivationDynamics::setNull()
{
    setAuthors("Jennifer Yong, Apoorva Rajagopal");
}

void FatigueMuscleActivationDynamics::constructProperties()
{
    constructProperty_activation_time_constant(0.010);
    constructProperty_deactivation_time_constant(0.040);
    constructProperty_fatigue_factor(0.00505);
    constructProperty_recovery_factor(0.01725);
    constructProperty_recruitment_from_resting_time_constant(10.0);
    constructProperty_default_active_motor_units(0.0);
    constructProperty_default_fatigue_motor_units(0.0);
    constructProperty_default_resting_motor_units(1.0);
}

double FatigueMuscleActivationDynamics::
calcTargetActivationDerivative(double excitation, double targetActivation) const
{
    excitation = clampToValidInterval(excitation);
    targetActivation = clampToValidInterval(targetActivation);
    double tau = (excitation > targetActivation)
    ? getActivationTimeConstant() * (0.5 + 1.5*targetActivation)
    : getDeactivationTimeConstant() / (0.5 + 1.5*targetActivation);
    return (excitation-targetActivation)/tau;
}

double FatigueMuscleActivationDynamics::calcActivationDerivative(double excitation, double targetActivation, double fatigueMotorUnits, double activeMotorUnits) const
{
    excitation = clampToValidInterval(excitation);
    targetActivation = clamptoValidInterval(targetActivation);
    double targetActivationDeriv = calcTargetActivationDerivative(excitation, targetActivation);
    double fatigueMotorUnitsDeriv = calcFatigueMotorUnitsDeriv(fatigueMotorUnits, activeMotorUnits);
    return (targetActivationDeriv * (1 - fatigueMotorUnits) - (targetActivation * fatigueMotorUnitsDeriv));
}

double FatigueMuscleActivationDynamics::calcFatigueMotorUnitsDeriv(double fatigueMotorUnits, double activeMotorUnits) const
{
    double F = getFatigueFactor();
    double R = getRecoveryFactor();
    double restingMotorUnits = calcRestingMotorUnits(fatigueMotorUnits, activeMotorUnits);
    return (F * activeMotorUnits * restingMotorUnits - R * fatigueMotorUnits);
    
}

double FatigueMuscleActivationDynamics::calcActiveMotorUnitsDeriv(double excitation, double targetActivation, double activeMotorUnits, double fatigueMotorUnits) const
{
    double restingMotorUnits = calcRestingMotorUnits(fatigueMotorUnits, activeMotorUnits);
    double F = getFatigueFactor();
    double recruitmentOfResting = calcRecruitmentOfResting(excitation, targetActivation, activeMotorUnits, fatigueMotorUnits);
    double tau = getRecruitmentFromRestingTimeConstant();
    return (restingMotorUnits * ((recruitmentOfResting/tau) - (F*activeMotorUnits)));
}

double FatigueMuscleActivationDynamics::calcRestingMotorUnitsDeriv(double fatigueMotorUnitsDeriv, activeMotorUnitsDeriv) const
{
    return (-fatigueMotorUnitsDeriv-activeMotorUnitsDeriv);
}

double FatigueMuscleActivationDyamics::calcRestingMotorUnits(double fatigueMotorUnits, double activeMotorUnits) const
{
    return (1 - fatigueMotorUnits - activeMotorUnits);
}

double FatigueMuscleActivationDynamics::calcRecruitmentOfResting(double excitation, double targetActivation, double activeMotorUnits, double fatigueMotorUnits) const
{
    double restingMotorUnits = calcRestingMotorUnits(fatigueMotorUnits, activeMotorUnits);
    if ((excitation >= targetActivation) && ((excitation - activeMotorUnits) < restingMotorUnits)) {
        return (excitation - activeMotorUnits);
    }
    else if ((excitation >= targetActivation) && ((excitation - activeMotorUnits) >= restingMotorUnits)) {
        return restingMotorUnits;
    }
    else
    {
        return (excitation - activeMotorUnits);
    }
}

