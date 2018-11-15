/* -------------------------------------------------------------------------- *
 *            OpenSim:  ActivationFiberLengthMuscle_Deprecated.cpp            *
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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//==============================================================================
// INCLUDES
//==============================================================================
#include "ActivationFiberLengthMuscle_Deprecated.h"
#include "Model.h"

using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

//==============================================================================
// STATICS
//==============================================================================

const int ActivationFiberLengthMuscle_Deprecated::STATE_ACTIVATION = 0;
const int ActivationFiberLengthMuscle_Deprecated::STATE_FIBER_LENGTH = 1;

const string ActivationFiberLengthMuscle_Deprecated::STATE_ACTIVATION_NAME = "activation";
const string ActivationFiberLengthMuscle_Deprecated::STATE_FIBER_LENGTH_NAME = "fiber_length";


//==============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//==============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy 
// assignment operator.

//_____________________________________________________________________________
// Default constructor.
ActivationFiberLengthMuscle_Deprecated::ActivationFiberLengthMuscle_Deprecated() 
{
    setNull();
    constructProperties();
}

//==============================================================================
// CONSTRUCTION METHODS
//==============================================================================
//_____________________________________________________________________________
// Set the data members of this ActivationFiberLengthMuscle_Deprecated to 
// their null values.
void ActivationFiberLengthMuscle_Deprecated::setNull()
{
    _defaultActivation = 0;
    _defaultFiberLength = 0;
}

//_____________________________________________________________________________
// Allocate and initialize properties. (There aren't any here.)
void ActivationFiberLengthMuscle_Deprecated::constructProperties()
{
}

//_____________________________________________________________________________
/**
 * allocate and initialize the SimTK state for this actuator.
 */
 void ActivationFiberLengthMuscle_Deprecated::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
    const string& className = getConcreteClassName();
    const string& suffix = " flag is not currently implemented.";

    if(get_ignore_activation_dynamics()){
        string errMsg = className + "::ignore_activation_dynamics" + suffix;
        throw Exception(errMsg);
    }

    if(get_ignore_tendon_compliance()){
        string errMsg = className + "::ignore_tendon_compliance" + suffix;
        throw Exception(errMsg);
    }

    addStateVariable(STATE_ACTIVATION_NAME);
    addStateVariable(STATE_FIBER_LENGTH_NAME);

    // Cache the computed active and passive muscle force
    // note the total muscle force is the tendon force and is already a cached variable of the actuator
    addCacheVariable<double>("activeForce", 0.0, SimTK::Stage::Velocity);
    addCacheVariable<double>("passiveForce", 0.0, SimTK::Stage::Velocity);
 }

 void ActivationFiberLengthMuscle_Deprecated::extendInitStateFromProperties( SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);

    setActivation(s, _defaultActivation);
    setFiberLength(s, _defaultFiberLength);
}

void ActivationFiberLengthMuscle_Deprecated::extendSetPropertiesFromState(const SimTK::State& state)
{
    Super::extendSetPropertiesFromState(state);

    _defaultActivation = getActivation(state);
    _defaultFiberLength = getFiberLength(state);
}

double ActivationFiberLengthMuscle_Deprecated::getDefaultActivation() const {
    return _defaultActivation;
}
void ActivationFiberLengthMuscle_Deprecated::setDefaultActivation(double activation) {
    _defaultActivation = activation;
}
double ActivationFiberLengthMuscle_Deprecated::getDefaultFiberLength() const {
    return _defaultFiberLength;
}
void ActivationFiberLengthMuscle_Deprecated::setDefaultFiberLength(double length) {
    _defaultFiberLength = length;
}


// STATES
//_____________________________________________________________________________
/**
 * Set the derivative of an actuator state, specified by index
 *
 * @param aStateName The name of the state to set.
 * @param aValue The value to set the state to.
 */
void ActivationFiberLengthMuscle_Deprecated::setStateVariableDeriv(const SimTK::State& s, const std::string &aStateName, double aValue) const
{
    double& cacheVariable = updCacheVariableValue<double>(s, aStateName + "_deriv");
    cacheVariable = aValue;
    markCacheVariableValid(s, aStateName + "_deriv");
}

//_____________________________________________________________________________
/**
 * Get the derivative of an actuator state, by index.
 *
 * @param aStateName the name of the state to get.
 * @return The value of the state.
 */
double ActivationFiberLengthMuscle_Deprecated::getStateVariableDeriv(const SimTK::State& s, const std::string &aStateName) const
{
    return getCacheVariableValue<double>(s, aStateName + "_deriv");
}

//_____________________________________________________________________________
/**
 * Compute the derivatives of the muscle states.
 *
 * @param s  system state
 */
void ActivationFiberLengthMuscle_Deprecated::
    computeStateVariableDerivatives(const SimTK::State &s) const
{
    double adot = 0;
    double ldot = 0;
    if (appliesForce(s)) {
        adot = getActivationDeriv(s);
        ldot = getFiberVelocity(s);
    }

    setStateVariableDerivativeValue(s, STATE_ACTIVATION_NAME, adot);
    setStateVariableDerivativeValue(s, STATE_FIBER_LENGTH_NAME, ldot);
}

//==============================================================================
// GET
//==============================================================================
//-----------------------------------------------------------------------------
// LENGTH
//-----------------------------------------------------------------------------
double ActivationFiberLengthMuscle_Deprecated::getFiberLength(const SimTK::State& s) const {
    return getStateVariableValue(s, STATE_FIBER_LENGTH_NAME);
}
void ActivationFiberLengthMuscle_Deprecated::setFiberLength(SimTK::State& s, double fiberLength) const {
    setStateVariableValue(s, STATE_FIBER_LENGTH_NAME, fiberLength);
}
double ActivationFiberLengthMuscle_Deprecated::getFiberLengthDeriv(const SimTK::State& s) const {
    return getStateVariableDeriv(s, STATE_FIBER_LENGTH_NAME);
}
void ActivationFiberLengthMuscle_Deprecated::setFiberLengthDeriv(const SimTK::State& s, double fiberLengthDeriv) const {
    setStateVariableDeriv(s, STATE_FIBER_LENGTH_NAME, fiberLengthDeriv);
}
//_____________________________________________________________________________
/**
 * Get the normalized length of the muscle fiber(s).  This is the current
 * fiber length(s) divided by the optimal fiber length.
 *
 * @param Current length of the muscle fiber(s).
 */
double ActivationFiberLengthMuscle_Deprecated::getNormalizedFiberLength(const SimTK::State& s) const
{
    return getFiberLength(s) / getOptimalFiberLength();
}
//_____________________________________________________________________________
/**
 * Get the length of the muscle fiber(s) along the tendon. This method
 * accounts for the pennation angle. 
 *
 * @return Current length of the muscle fiber(s) along the direction of
 * the tendon.
 */
double ActivationFiberLengthMuscle_Deprecated::getFiberLengthAlongTendon(const SimTK::State& s) const
{
    return getFiberLength(s) * cos(getPennationAngle(s));
}
//_____________________________________________________________________________
/**
 * Get the length of the tendon.
 *
 * @return Current length of the tendon.
 */
double ActivationFiberLengthMuscle_Deprecated::getTendonLength(const SimTK::State& s) const
{
    return getLength(s) - getFiberLengthAlongTendon(s);
}

//-----------------------------------------------------------------------------
// FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the force generated by the muscle fibers. This accounts for
 * pennation angle. That is, the fiber force is computed by dividing the
 * actuator force by the cosine of the pennation angle.
 *
 * @return Force in the muscle fibers.
 */
double ActivationFiberLengthMuscle_Deprecated::getFiberForce(const SimTK::State& s) const
{
    double force;
    double cos_penang = cos(getPennationAngle(s));
    if(fabs(cos_penang) < SimTK::Zero) {
        force = SimTK::NaN;
    } else {
        force = getActuation(s) / cos_penang;
    }

    return force;
}
//_____________________________________________________________________________
/**
 * Get the active force generated by the muscle fibers.
 *
 * @return Current active force of the muscle fibers.
 */
double ActivationFiberLengthMuscle_Deprecated::getActiveFiberForce(const SimTK::State& s) const
{
    return getFiberForce(s) - getPassiveFiberForce(s);
}
/**
 * Get the passive force generated by the muscle fibers.
 *
 * @param Current passive force of the muscle fiber(s).
 */
double ActivationFiberLengthMuscle_Deprecated::getPassiveFiberForce(const SimTK::State& s) const
{
    return getPassiveForce(s);
}
//_____________________________________________________________________________
/**
 * Get the active force generated by the muscle fibers along the direction
 * of the tendon.
 *
 * @return Current active force of the muscle fibers along tendon.
 */
double ActivationFiberLengthMuscle_Deprecated::getActiveFiberForceAlongTendon(const SimTK::State& s) const
{
    return getActiveFiberForce(s) * cos(getPennationAngle(s));
}
//_____________________________________________________________________________
/**
 * Get the passive force generated by the muscle fibers along the direction
 * of the tendon.
 *
 * @return Current passive force of the muscle fibers along tendon.
 */
double ActivationFiberLengthMuscle_Deprecated::getPassiveFiberForceAlongTendon(const SimTK::State& s) const
{
    return getPassiveFiberForce(s) * cos(getPennationAngle(s));
}
double ActivationFiberLengthMuscle_Deprecated::getPassiveForce( const SimTK::State& s) const {
    return getCacheVariableValue<double>(s, "passiveForce");
}
void ActivationFiberLengthMuscle_Deprecated::setPassiveForce(const SimTK::State& s, double force ) const {
    setCacheVariableValue<double>(s, "passiveForce", force);
}

double ActivationFiberLengthMuscle_Deprecated::getTendonForce(const SimTK::State& s) const {
    return getActuation(s);
}
void ActivationFiberLengthMuscle_Deprecated::setTendonForce(const SimTK::State& s, double force) const {
    setActuation(s, force);
}
double ActivationFiberLengthMuscle_Deprecated::getActivation(const SimTK::State& s) const {
    return getStateVariableValue(s, STATE_ACTIVATION_NAME);
}
void ActivationFiberLengthMuscle_Deprecated::setActivation(SimTK::State& s, double activation) const {
    setStateVariableValue(s, STATE_ACTIVATION_NAME, activation);
}
double ActivationFiberLengthMuscle_Deprecated::getActivationDeriv(const SimTK::State& s) const {
    return getStateVariableDeriv(s, STATE_ACTIVATION_NAME);
}
void ActivationFiberLengthMuscle_Deprecated::setActivationDeriv(const SimTK::State& s, double activationDeriv) const {
    setStateVariableDeriv(s, STATE_ACTIVATION_NAME, activationDeriv);
}
//_____________________________________________________________________________
//**
// * get the excitation value for this ActivationFiberLengthMuscle_Deprecated 
// */
double ActivationFiberLengthMuscle_Deprecated::getExcitation( const SimTK::State& s) const {
    return( getControl(s) );
}
//_____________________________________________________________________________
/**
 * Get the stress in this actuator.  It is calculated as the force divided
 * by the maximum isometric force (which is proportional to its area).
 */
double ActivationFiberLengthMuscle_Deprecated::getStress(const SimTK::State& s) const
{
    return getActuation(s) / get_max_isometric_force();
}

//==============================================================================
// SCALING
//==============================================================================
void ActivationFiberLengthMuscle_Deprecated::
extendPostScale(const SimTK::State& s, const ScaleSet& scaleSet)
{
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

//--------------------------------------------------------------------------
// COMPUTATIONS
//--------------------------------------------------------------------------

//==============================================================================
// FORCE APPLICATION
//==============================================================================
//_____________________________________________________________________________
/**
 * Apply the muscle's force at its points of attachment to the bodies.
 */
void ActivationFiberLengthMuscle_Deprecated::computeForce(const SimTK::State& s, 
                              SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                              SimTK::Vector& generalizedForces) const
{
    Super::computeForce(s, bodyForces, generalizedForces);

    if (isActuationOverridden(s)) {
        // Also define the state derivatives, since realize acceleration will
        // ask for muscle derivatives, which will be integrated
        // in the case the force is being overridden, the states aren't being used
        // but a valid derivative cache entry is still required
        setStateVariableDeriv(s, STATE_ACTIVATION_NAME, 0.0);
        setStateVariableDeriv(s, STATE_FIBER_LENGTH_NAME, 0.0);
    } 
}

//_____________________________________________________________________________
/**
 * Compute the equilibrium states.  This method computes a fiber length
 * for the muscle that is consistent with the muscle's activation level.
 */
void ActivationFiberLengthMuscle_Deprecated::computeInitialFiberEquilibrium(SimTK::State& s) const
{
    /*double force = */computeIsometricForce(s, getActivation(s));

    //cout<<getName()<<": isometric force = "<<force<<endl;
    //cout<<getName()<<": fiber length = "<<getFiberLength(s)<<endl;
}


double ActivationFiberLengthMuscle_Deprecated::
    calcInextensibleTendonActiveFiberForce(SimTK::State& s, double aActivation) const
{
    double isometricForce = computeIsometricForce(s, aActivation);

    const double &optimalFiberLength = get_optimal_fiber_length();
    const double &pennationAngleAtOptimal = get_pennation_angle_at_optimal();
    const double &maxContractionVelocity = get_max_contraction_velocity();

    //double normalizedLength = getFiberLength(s) / optimalFiberLength;
    double normalizedVelocity = -cos(pennationAngleAtOptimal) * getLengtheningSpeed(s) / (maxContractionVelocity * optimalFiberLength);
    double normalizedForceVelocity = evaluateForceLengthVelocityCurve(1.0,1.0,normalizedVelocity);

    return isometricForce * normalizedForceVelocity;
}

//==============================================================================
// GENERIC NORMALIZED FORCE-LENGTH-VELOCITY PROPERTIES
//==============================================================================
//_____________________________________________________________________________
/**
 * Evaluate the normalized force-length-velocity curve for the Muscle_Deprecated.
 * A simple generic implementation is used here.  Derived classes should
 * override this method for more precise evaluation of the
 * force-length-velocity curve.
 *
 * @param aActivation Activation level of the Muscle_Deprecated.  1.0 is full activation;
 * 0.0 is no activation.
 * @param aNormalizedLength Normalized length of the Muscle_Deprecated fibers.  1.0 indicates
 * the Muscle_Deprecated fibers are at their optimal length.  Lnorm = L / Lo.
 * @param aNormalizedVelocity Normalized shortening velocity of the Muscle_Deprecated fibers.
 * Positive values indicate concentric contraction (shortening); negative values
 * indicate eccentric contraction (lengthening).  Normalized velocity is
 * the fiber shortening velocity divided by the maximum shortening velocity times
 * the optimal fiber length.  Vnorm = V / (Vmax*Lo).
 * @return Force normalized by the optimal force.
 */
double ActivationFiberLengthMuscle_Deprecated::evaluateForceLengthVelocityCurve(double aActivation, double aNormalizedLength, double aNormalizedVelocity) const
{
    // force-length
    double fLength = exp(-17.33 * fabs(pow(aNormalizedLength-1.0,3)));

    // force-velocity
    double fVelocity = 1.8  -  1.8 / (1.0 + exp( (0.04 - aNormalizedVelocity)/0.18) );

    return aActivation * fLength * fVelocity;
}


//_____________________________________________________________________________
/**
 * Utility function to calculate the current pennation angle in a
 * Muscle_Deprecated. Pennation angle increases as Muscle_Deprecated fibers shorten. The implicit
 * modeling assumption is that Muscle_Deprecateds have constant width.
 *
 * @param aFiberLength Current fiber length of Muscle_Deprecated.
 * @param aOptimalFiberLength Optimal fiber length of Muscle_Deprecated.
 * @param aInitialPennationAngle Pennation angle at optimal fiber length (in radians).
 * @return Current pennation angle (in radians).
 */
double ActivationFiberLengthMuscle_Deprecated::calcPennation( double aFiberLength, double aOptimalFiberLength,
                                                double aInitialPennationAngle) const
{
    if (aFiberLength < SimTK::Eps)
        return 0.0;

   double value = aOptimalFiberLength * sin(aInitialPennationAngle) / aFiberLength;

   if ( SimTK::isNaN(value)  ) 
       return 0.0;
   else if (value <= 0.0 )
      return 0.0;
   else if (value >= 1.0)
        return SimTK_PI/2.0;
   else
      return asin(value);
}

//==============================================================================
// CALCULATIONS
//==============================================================================
/* calculate muscle's position related values such fiber and tendon lengths,
    normalized lengths, pennation angle, etc... */
void ActivationFiberLengthMuscle_Deprecated::calcMuscleLengthInfo(const SimTK::State& s, MuscleLengthInfo& mli) const
{
    double norm_muscle_tendon_length = getLength(s) / getOptimalFiberLength();
    
    mli.fiberLength = getStateVariableValue(s, STATE_FIBER_LENGTH_NAME);
    
    mli.pennationAngle = calcPennation(mli.fiberLength, getOptimalFiberLength(), getPennationAngleAtOptimalFiberLength());

    mli.cosPennationAngle = cos(mli.pennationAngle);
    mli.tendonLength = getLength(s)-mli.fiberLength*mli.cosPennationAngle;
    
    mli.normFiberLength = mli.fiberLength/getOptimalFiberLength();
    mli.normTendonLength = norm_muscle_tendon_length - mli.normFiberLength * mli.cosPennationAngle;
    mli.tendonStrain = (mli.tendonLength/getTendonSlackLength()-1.0);

    mli.fiberActiveForceLengthMultiplier = calcActiveForce(s, mli.normFiberLength);
    mli.fiberPassiveForceLengthMultiplier = calcPassiveForce(s, mli.normFiberLength);
}

/* calculate muscle's velocity related values such fiber and tendon velocities,
    normalized velocities, pennation angular velocity, etc... */
void ActivationFiberLengthMuscle_Deprecated::calcFiberVelocityInfo(const SimTK::State& s, FiberVelocityInfo& fvi) const
{
    fvi.fiberVelocity = getFiberLengthDeriv(s);
    fvi.normFiberVelocity = fvi.fiberVelocity/(getOptimalFiberLength()*getMaxContractionVelocity());
}

/* calculate muscle's active and passive force-length, force-velocity, 
    tendon force, relationships and their related values */
void ActivationFiberLengthMuscle_Deprecated::calcMuscleDynamicsInfo(const SimTK::State& s, MuscleDynamicsInfo& mdi) const
{
    const MuscleLengthInfo &mli = getMuscleLengthInfo(s);
    const double &maxIsometricForce = getMaxIsometricForce();

    double tendonForce = getActuation(s);
    mdi.normTendonForce = tendonForce/maxIsometricForce;
    
    mdi.passiveFiberForce = mli.fiberPassiveForceLengthMultiplier * maxIsometricForce;
    
    mdi.activation = getStateVariableValue(s, STATE_ACTIVATION_NAME);

    mdi.activeFiberForce =  tendonForce/mli.cosPennationAngle - mdi.passiveFiberForce;
}
