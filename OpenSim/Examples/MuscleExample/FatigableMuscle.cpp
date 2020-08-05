/* -------------------------------------------------------------------------- *
 *                      OpenSim:  FatigableMuscle.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan, Ajay Seth                                           *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//=============================================================================
// INCLUDES
//=============================================================================
#include "FatigableMuscle.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/*
 * Default constructor
 */
FatigableMuscle::FatigableMuscle()
{
    constructProperties();
}

//_____________________________________________________________________________
/*
 * Constructor.
 */
FatigableMuscle::FatigableMuscle(const std::string &name, 
                    double maxIsometricForce, double optimalFiberLength, 
                    double tendonSlackLength, double pennationAngle, 
                    double fatigueFactor, double recoveryFactor) : 
    Super(name, maxIsometricForce, optimalFiberLength, tendonSlackLength, 
          pennationAngle)
{
    constructProperties();
    setFatigueFactor(fatigueFactor);
    setRecoveryFactor(recoveryFactor);
}

//_____________________________________________________________________________
/*
 * Construct and initialize properties.
 * All properties are added to the property set. Once added, they can be
 * read in and written to files.
 */
void FatigableMuscle::constructProperties()
{
    setAuthors("Ajay Seth");
    constructProperty_fatigue_factor(0.5);
    constructProperty_recovery_factor(0.5);
    constructProperty_default_target_activation(0.01);
    constructProperty_default_active_motor_units(1.0);
    constructProperty_default_fatigued_motor_units(0.0);
}

// Define new states and their derivatives in the underlying system
void FatigableMuscle::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    // Allow Millard2012EquilibriumMuscle to add its states, before extending
    Super::extendAddToSystem(system);

    // Now add the states necessary to implement the fatigable behavior
    addStateVariable("target_activation");
    addStateVariable("active_motor_units");
    addStateVariable("fatigued_motor_units");
}

void FatigableMuscle::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);
    setTargetActivation(s, getDefaultTargetActivation());
    setActiveMotorUnits(s, getDefaultActiveMotorUnits());
    setFatiguedMotorUnits(s, getDefaultFatiguedMotorUnits());
}

void FatigableMuscle::extendSetPropertiesFromState(const SimTK::State& s)
{
    Super::extendSetPropertiesFromState(s);
    setDefaultTargetActivation(getTargetActivation(s));
    setDefaultActiveMotorUnits(getActiveMotorUnits(s));
    setDefaultFatiguedMotorUnits(getFatiguedMotorUnits(s));
}

//--------------------------------------------------------------------------
// GET & SET Properties
//--------------------------------------------------------------------------
void FatigableMuscle::setFatigueFactor(double aFatigueFactor)
{
    set_fatigue_factor(aFatigueFactor);
}

void FatigableMuscle::setRecoveryFactor(double aRecoveryFactor)
{
    set_recovery_factor(aRecoveryFactor);
}


void FatigableMuscle::setDefaultTargetActivation(double targetActivation)
{
    set_default_target_activation(targetActivation);
}

double FatigableMuscle::getDefaultActiveMotorUnits() const
{
    return get_default_active_motor_units();
}

void FatigableMuscle::setDefaultActiveMotorUnits(double activeMotorUnits) {
    set_default_active_motor_units(activeMotorUnits);
}

double FatigableMuscle::getDefaultFatiguedMotorUnits() const
{
    return get_default_fatigued_motor_units();
}

void FatigableMuscle::setDefaultFatiguedMotorUnits(double fatiguedMotorUnits) {
    set_default_fatigued_motor_units(fatiguedMotorUnits);
}

//--------------------------------------------------------------------------
// GET & SET States and their derivatives
//--------------------------------------------------------------------------
double FatigableMuscle::getTargetActivation(const SimTK::State& s) const 
{   return getStateVariableValue(s, "target_activation"); }

void FatigableMuscle::setTargetActivation(SimTK::State& s,
                                          double fatiguedAct) const
{   setStateVariableValue(s, "target_activation", fatiguedAct); }

double FatigableMuscle::getTargetActivationDeriv(const SimTK::State& s) const 
{   return getStateVariableDerivativeValue(s, "target_activation"); }

void FatigableMuscle::setTargetActivationDeriv(const SimTK::State& s,
                                        double fatiguedActDeriv) const 
{   setStateVariableDerivativeValue(s, "target_activation", fatiguedActDeriv); }


double FatigableMuscle::getActiveMotorUnits(const SimTK::State& s) const 
{   return getStateVariableValue(s, "active_motor_units"); }

void FatigableMuscle::setActiveMotorUnits(SimTK::State& s,
                                          double activeMotorUnits) const
{   setStateVariableValue(s, "active_motor_units", activeMotorUnits); }

double FatigableMuscle::getActiveMotorUnitsDeriv(const SimTK::State& s) const 
{   return getStateVariableDerivativeValue(s, "active_motor_units"); }

void FatigableMuscle::setActiveMotorUnitsDeriv(const SimTK::State& s,
                                        double activeMotorUnitsDeriv) const 
{   setStateVariableDerivativeValue(s, "active_motor_units", activeMotorUnitsDeriv); }

double FatigableMuscle::getFatiguedMotorUnits(const SimTK::State& s) const
{   return getStateVariableValue(s, "fatigued_motor_units"); }

void FatigableMuscle::setFatiguedMotorUnits(SimTK::State& s,
                                            double fatiguedMotorUnits) const
{   setStateVariableValue(s, "fatigued_motor_units", fatiguedMotorUnits); }

double FatigableMuscle::getFatiguedMotorUnitsDeriv(const SimTK::State& s) const 
{    return getStateVariableDerivativeValue(s, "fatigued_motor_units"); }

void FatigableMuscle::setFatiguedMotorUnitsDeriv(const SimTK::State& s,
                                        double fatiguedMotorUnitsDeriv) const
{   setStateVariableDerivativeValue(s, "fatigued_motor_units", fatiguedMotorUnitsDeriv); }


//=============================================================================
// COMPUTATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the derivatives of the muscle states.
 *
 * @param s  system state
 */
void FatigableMuscle::computeStateVariableDerivatives(const SimTK::State& s) const
{
    // Allow Super to assign any state derivative values for states it allocated
    Super::computeStateVariableDerivatives(s);

    // int nd = getNumStateVariables();

    SimTK_ASSERT1(getNumStateVariables() == 5, 
                  "FatigableMuscle: Expected 5 state variables"
                  " but encountered  %f.", getNumStateVariables());

    // compute the rates at which motor units are converted to/from active
    // and fatigued states based on Liu et al. 2008
    double activeMotorUnitsDeriv = - getFatigueFactor()*getActiveMotorUnits(s) 
            + getRecoveryFactor() * getFatiguedMotorUnits(s);

    double fatigueMotorUnitsDeriv = getFatigueFactor()* getActiveMotorUnits(s) 
            - getRecoveryFactor() * getFatiguedMotorUnits(s);

    // compute the target activation rate
    double excitation = getExcitation(s);
    double targetActivation =
        getActivationModel().clampActivation(getTargetActivation(s));
    double targetActivationRate =
        getActivationModel().calcDerivative(targetActivation, excitation);

    // specify the activation derivative based on the amount of active motor 
    // units and the rate at which motor units are becoming active.
    // we assume that the actual activation = Ma*a       then,
    // activationRate = dMa/dt*a + Ma*da/dt  where a is the target activation
    double activationRate = activeMotorUnitsDeriv*targetActivation + 
                            getActiveMotorUnits(s)*targetActivationRate;

    // cache the results for fast access by reporting, etc...
    Super::setStateVariableDerivativeValue(s, "activation", activationRate);
    setTargetActivationDeriv(s, targetActivationRate);
    setActiveMotorUnitsDeriv(s, activeMotorUnitsDeriv);
    setFatiguedMotorUnitsDeriv(s, fatigueMotorUnitsDeriv);
}
