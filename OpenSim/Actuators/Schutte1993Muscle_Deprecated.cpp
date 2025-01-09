/* -------------------------------------------------------------------------- *
 *                 OpenSim:  Schutte1993Muscle_Deprecated.cpp                 *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan, Jeffrey A. Reinbolt                                 *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "Schutte1993Muscle_Deprecated.h"
#include <OpenSim/Common/SimmSpline.h>
#include <OpenSim/Common/SimmMacros.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Schutte1993Muscle_Deprecated::Schutte1993Muscle_Deprecated()
{
    constructProperties();
}

//_____________________________________________________________________________
/**
 * Constructor.
 */
Schutte1993Muscle_Deprecated::Schutte1993Muscle_Deprecated
   (const std::string& aName, double aMaxIsometricForce,
    double aOptimalFiberLength, double aTendonSlackLength,
    double aPennationAngle)
{
    constructProperties();
    setName(aName);
    setMaxIsometricForce(aMaxIsometricForce);
    setOptimalFiberLength(aOptimalFiberLength);
    setTendonSlackLength(aTendonSlackLength);
    setPennationAngleAtOptimalFiberLength(aPennationAngle);
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
// Allocate and initialize properties.
void Schutte1993Muscle_Deprecated::constructProperties()
{
    constructProperty_time_scale(0.1);
    constructProperty_activation1(7.667);
    constructProperty_activation2(1.459854);
    constructProperty_damping(0.1);

    int tendonForceLengthCurvePoints = 17;
    double tendonForceLengthCurveX[] = {-10.00000000, -0.00200000, -0.00100000,  0.00000000,  0.00131000,  0.00281000,  0.00431000,  0.00581000,  0.00731000,  0.00881000,  0.01030000,  0.01180000,  0.01230000,  9.20000000,  9.20100000,  9.20200000, 20.00000000};
    double tendonForceLengthCurveY[] = {0.00000000,  0.00000000,  0.00000000,  0.00000000,  0.01080000,  0.02570000,  0.04350000,  0.06520000,  0.09150000,  0.12300000,  0.16100000,  0.20800000,  0.22700000,  345.00000000,  345.00000000,  345.00000000,  345.00000000};
    SimmSpline tendonForceLengthCurve
       (tendonForceLengthCurvePoints, tendonForceLengthCurveX, 
        tendonForceLengthCurveY);
    constructProperty_tendon_force_length_curve(tendonForceLengthCurve);

    int activeForceLengthCurvePoints = 21;
    double activeForceLengthCurveX[] = {-5.30769200, -4.30769200, -1.92307700, -0.88461500, -0.26923100,  0.23076900,  0.46153800,  0.52725000,  0.62875000,  0.71875000,  0.86125000,  1.04500000,  1.21750000,  1.43875000,  1.50000000,  1.61538500,  2.00000000,  2.96153800,  3.69230800,  5.46153800,  9.90190200};
    double activeForceLengthCurveY[] = {0.01218800,  0.02189900,  0.03646600,  0.05249300,  0.07531200,  0.11415800,  0.15785900,  0.22666700,  0.63666700,  0.85666700,  0.95000000,  0.99333300,  0.77000000,  0.24666700,  0.19382100,  0.13325200,  0.07268300,  0.04441700,  0.03634100,  0.02189900,  0.00733200};
    SimmSpline activeForceLengthCurve
       (activeForceLengthCurvePoints, activeForceLengthCurveX, 
        activeForceLengthCurveY);
    constructProperty_active_force_length_curve(activeForceLengthCurve);

    int passiveForceLengthCurvePoints = 13;
    double passiveForceLengthCurveX[] = {-5.00000000,  0.99800000,  0.99900000,  1.00000000,  1.10000000,  1.20000000,  1.30000000,  1.40000000,  1.50000000,  1.60000000,  1.60100000,  1.60200000,  5.00000000};
    double passiveForceLengthCurveY[] = {0.00000000,  0.00000000,  0.00000000,  0.00000000,  0.03500000,  0.12000000,  0.26000000,  0.55000000,  1.17000000,  2.00000000,  2.00000000,  2.00000000,  2.00000000};
    SimmSpline passiveForceLengthCurve
       (passiveForceLengthCurvePoints, passiveForceLengthCurveX, 
        passiveForceLengthCurveY);
    constructProperty_passive_force_length_curve(passiveForceLengthCurve);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this Schutte1993Muscle_Deprecated.
 */
void Schutte1993Muscle_Deprecated::extendConnectToModel(Model& aModel)
{
    // Base class
    Super::extendConnectToModel(aModel);
}


//=============================================================================
// GET
//=============================================================================
//-----------------------------------------------------------------------------
// TIME SCALE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the scale factor for normalizing time.
 *
 * @param aTimeScale The scale factor for normalizing time.
 * @return Whether the scale factor was successfully changed.
 */
bool Schutte1993Muscle_Deprecated::setTimeScale(double aTimeScale)
{
    set_time_scale(aTimeScale);
    return true;
}

//-----------------------------------------------------------------------------
// ACTIVATION 1
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the time constant of ramping up of muscle force.
 *
 * @param aActivation1 The time constant of ramping up of muscle force.
 * @return Whether the time constant was successfully changed.
 */
bool Schutte1993Muscle_Deprecated::setActivation1(double aActivation1)
{
    set_activation1(aActivation1);
    return true;
}

//-----------------------------------------------------------------------------
// ACTIVATION 2
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the time constant of ramping up and ramping down of muscle force.
 *
 * @param aActivation1 The time constant of ramping up and ramping down of muscle force.
 * @return Whether the time constant was successfully changed.
 */
bool Schutte1993Muscle_Deprecated::setActivation2(double aActivation2)
{
    set_activation2(aActivation2);
    return true;
}



//-----------------------------------------------------------------------------
// DAMPING
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the damping factor related to maximum contraction velocity.
 *
 * @param aDamping The damping factor related to maximum contraction velocity.
 * @return Whether the damping factor was successfully changed.
 */
bool Schutte1993Muscle_Deprecated::setDamping(double aDamping)
{
    set_damping(aDamping);
    return true;
}




//_____________________________________________________________________________
/**
 * Compute the actuation for the muscle. This function assumes
 * that computeDerivatives has already been called.
 */
double Schutte1993Muscle_Deprecated::computeActuation(const SimTK::State& s) const
{
    double tendonForce;
    double passiveForce;
    double activeForce;
    double activationDeriv;
    double fiberLengthDeriv;

    double norm_tendon_length, ca;
    double norm_muscle_tendon_length, pennation_angle;
    double excitation = getExcitation(s);

    /* Normalize the muscle states */
    double activation = getActivation(s);
    double normFiberLength = getFiberLength(s) / _optimalFiberLength;

    /* Compute normalized muscle state derivatives */
    if (excitation >= activation) 
       activationDeriv = (excitation - activation) * (get_activation1() * excitation + get_activation2());
    else
      activationDeriv = (excitation - activation) * get_activation2();

    pennation_angle = calcPennation(normFiberLength, 1.0, _pennationAngleAtOptimal);
    ca = cos(pennation_angle);
    norm_muscle_tendon_length = getLength(s) / _optimalFiberLength;
    norm_tendon_length = norm_muscle_tendon_length - normFiberLength * ca;

    tendonForce = calcTendonForce(s,norm_tendon_length);
    passiveForce =  calcNonzeroPassiveForce(s,normFiberLength, 0.0);
    activeForce = getActiveForceLengthCurve().calcValue(SimTK::Vector(1, normFiberLength) );
    if (activeForce < 0.0) activeForce = 0.0;

   /* If pennation equals 90 degrees, fiber length equals muscle width and fiber
    * velocity goes to zero.  Pennation will stay at 90 until tendon starts to
    * pull, then "stiff tendon" approximation is used to calculate approximate
    * fiber velocity.
    */
    if (EQUAL_WITHIN_ERROR(ca, 0.0)) {
      if (EQUAL_WITHIN_ERROR(tendonForce, 0.0)) {
         fiberLengthDeriv = 0.0;;
      } else {
         double h = norm_muscle_tendon_length - _tendonSlackLength;
         double w = _optimalFiberLength * sin(_pennationAngleAtOptimal);
         double new_fiber_length = sqrt(h*h + w*w) / _optimalFiberLength;
         double new_pennation_angle = calcPennation(new_fiber_length, 1.0, _pennationAngleAtOptimal);
         double new_ca = cos(new_pennation_angle);
         fiberLengthDeriv = getLengtheningSpeed(s) * get_time_scale() / _optimalFiberLength * new_ca;
      }
    } else {
      double velocity_dependent_force = tendonForce / ca - passiveForce;
      if (velocity_dependent_force < 0.0) velocity_dependent_force = 0.0;
      fiberLengthDeriv  = calcFiberVelocity(s,activation, activeForce, velocity_dependent_force);
    }

    /* Un-normalize the muscle state derivatives and forces. */
    setActivationDeriv(s,  activationDeriv / get_time_scale());
    setFiberLengthDeriv(s, fiberLengthDeriv * _optimalFiberLength / get_time_scale());

    tendonForce = tendonForce * _maxIsometricForce;
    setActuation(s, tendonForce);
    setTendonForce(s, tendonForce);
    setPassiveForce(s, passiveForce * _maxIsometricForce);

    return( tendonForce );
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the active force-length curve.
 *
 * @return Pointer to the active force-length curve (Function).
 */
const Function& Schutte1993Muscle_Deprecated::getActiveForceLengthCurve() const
{
    return get_active_force_length_curve();
}

//_____________________________________________________________________________
/**
 * Set the active force-length curve.
 *
 * @param aActiveForceLengthCurve Pointer to an active force-length curve (Function).
 * @return Whether active force-length curve was successfully changed.
 */
bool Schutte1993Muscle_Deprecated::
setActiveForceLengthCurve(const Function& aActiveForceLengthCurve)
{
    set_active_force_length_curve(aActiveForceLengthCurve);
    return true;
}

//_____________________________________________________________________________
/**
 * Get the passive force-length curve.
 *
 * @return Pointer to the passive force-length curve (Function).
 */
const Function& Schutte1993Muscle_Deprecated::getPassiveForceLengthCurve() const
{
    return get_passive_force_length_curve();
}

//_____________________________________________________________________________
/**
 * Get the passive force-length curve.
 *
 * @param aPassiveForceLengthCurve Pointer to a passive force-length curve (Function).
 * @return Whether passive force-length curve was successfully changed.
 */
bool Schutte1993Muscle_Deprecated::
setPassiveForceLengthCurve(const Function& aPassiveForceLengthCurve)
{
    set_passive_force_length_curve(aPassiveForceLengthCurve);
    return true;
}

//_____________________________________________________________________________
/**
 * Get the tendon force-length curve.
 *
 * @return Pointer to the tendon force-length curve (Function).
 */
const Function& Schutte1993Muscle_Deprecated::getTendonForceLengthCurve() const
{
    return get_tendon_force_length_curve();
}

//_____________________________________________________________________________
/**
 * Get the tendon force-length curve.
 *
 * @param aTendonForceLengthCurve Pointer to a tendon force-length curve (Function).
 * @return Whether tendon force-length curve was successfully changed.
 */
bool Schutte1993Muscle_Deprecated::
setTendonForceLengthCurve(const Function& aTendonForceLengthCurve)
{
    set_tendon_force_length_curve(aTendonForceLengthCurve);
    return true;
}

//_____________________________________________________________________________
/**
 * Calculate the force in tendon by finding tendon strain
 * and using it to interpolate the tendon force-length curve.
 *
 * @param aNormTendonLength Normalized length of the tendon.
 * @return The force in the tendon.
 */
double Schutte1993Muscle_Deprecated::calcTendonForce(const SimTK::State& s, double aNormTendonLength) const
{
   double tendon_force;
   double norm_resting_length = _tendonSlackLength / _optimalFiberLength;
   double tendon_strain =  (aNormTendonLength - norm_resting_length) / norm_resting_length;

   if (tendon_strain < 0.0)
      tendon_force = 0.0;
   else
      tendon_force = getTendonForceLengthCurve().calcValue(SimTK::Vector(1, tendon_strain));

   return tendon_force;
}

//_____________________________________________________________________________
/**
 * calcNonzeroPassiveForce: written by Chris Raasch and Lisa Schutte.
 * This function calculates the passive force in the muscle fibers using
 * an exponential instead of cubic splines. This results in non-zero passive
 * force for any fiber length (and thus prevents "slack" muscle/tendon problems).
 * It includes the contribution of an exponential passive force-length curve
 * (which equals 1.0 at norm_fiber_length = 1.5) as well as the damping effects
 * due to contraction velocity. It should someday be replaced by a new
 * passive-force spline in the muscle input file, but for now it includes
 * constants as Chris and Lisa derived them for their specific muscle model.
 *
 * @param aNormTendonLength Normalized length of the tendon.
 * @return The passive force in the muscle fibers.
 */
double Schutte1993Muscle_Deprecated::calcNonzeroPassiveForce(const SimTK::State& s, double aNormFiberLength, double aNormFiberVelocity) const
{
   double flcomponent =   0.0;
   if (getProperty_passive_force_length_curve().getValueIsDefault())
       flcomponent = exp(8.0*(aNormFiberLength - 1.0)) / exp(4.0);
   else
       flcomponent = getPassiveForceLengthCurve().calcValue(SimTK::Vector(1, aNormFiberLength) );
   return flcomponent + get_damping() * aNormFiberVelocity;
}

//_________ ____________________________________________________________________
/**
 * calcFiberVelocity: written by Chris Raasch and Lisa Schutte.
 * This function calculates the fiber velocity using an inverse
 * muscle force-velocity relationship with damping. It should
 * someday be replaced by a new force-velocity spline in the muscle input
 * file, but for now it includes constants as Chris and Lisa derived them
 * for their specific muscle model.
 *
 * @param aActivation Activation of the muscle.
 * @param aActiveForce Active force in the muscle fibers.
 * @param aVelocityDependentForce Force value that depends on fiber velocity.
 * @return The velocity of the muscle fibers.
 */
double Schutte1993Muscle_Deprecated::calcFiberVelocity(const SimTK::State& s, double aActivation, double aActiveForce, double aVelocityDependentForce) const
{
    double b, c, fiber_velocity;
    double kv = 0.15, slope_k = 0.13, fmax = 1.4;

   if (aVelocityDependentForce < -get_damping())
    {
      fiber_velocity = aVelocityDependentForce / get_damping();
    }
   else if (aVelocityDependentForce < aActivation * aActiveForce)
   {
      c = kv * (aVelocityDependentForce - aActivation * aActiveForce) / get_damping();
      b = -kv * (aVelocityDependentForce / kv + aActivation * aActiveForce +
            get_damping()) / get_damping();
      fiber_velocity = (-b - sqrt(b * b - 4 * c)) / 2.0;
   }
   else
   {
      c = -(slope_k * kv / ((get_damping() * (kv + 1)))) *
          (aVelocityDependentForce - aActivation * aActiveForce);
      b = -(aVelocityDependentForce / get_damping()
            -fmax * aActivation * aActiveForce / get_damping() - slope_k * kv / (kv + 1));
        fiber_velocity = (-b + sqrt(b * b - 4 * c)) / 2.0;
    }

    return fiber_velocity;
}

//_____________________________________________________________________________
/**
 * computeIsometricForce: this function finds the force in a muscle, assuming
 * static equilibrium. Using the total muscle-tendon length, it finds the
 * fiber and tendon lengths so that the forces in each match. This routine
 * takes pennation angle into account, so its definition of static equilibrium
 * is when tendon_force = fiber_force * cos(pennation_angle). This function
 * will modify the object's values for length, fiberLength,
 * and passiveForce.
 *
 * @param aActivation Activation of the muscle.
 * @return The isometric force in the muscle.
 */
double Schutte1993Muscle_Deprecated::computeIsometricForce(SimTK::State& s, double aActivation) const
{
#define MAX_ITERATIONS 100
#define ERROR_LIMIT 0.01

   int i;
   double length,tendon_length, fiber_force, tmp_fiber_length, min_tendon_stiffness;
   double cos_factor, fiber_stiffness;
   double old_fiber_length{SimTK::NaN}, length_change, tendon_stiffness, percent;
   double error_force = 0.0, old_error_force, tendon_force, tendon_strain;
   double passiveForce, activeForce, tendonForce, fiberLength;

   if (_optimalFiberLength < ROUNDOFF_ERROR) {
      setStateVariableValue(s, STATE_FIBER_LENGTH_NAME, 0.0);
      setPassiveForce(s, 0.0);
      setActuation(s, 0.0);
      setTendonForce(s, 0.0);
      return 0.0;
   }
 
    length = getLength(s);

   // Make first guess of fiber and tendon lengths. Make fiber length equal to
   // optimal_fiber_length so that you start in the middle of the active+passive
   // force-length curve. Muscle_width is the width, or thickness, of the
   // muscle-tendon unit. It is the shortest allowable fiber length because if
   // the muscle-tendon length is very short, the pennation angle will be 90
   // degrees and the fibers will be vertical (assuming the tendon is horizontal).
   // When this happens, the fibers are as long as the muscle is wide.
   // If the resting tendon length is zero, then set the fiber length equal to
   // the muscle tendon length / cosine_factor, and find its force directly.

   double muscle_width = _optimalFiberLength * sin(_pennationAngleAtOptimal);

   if (_tendonSlackLength < ROUNDOFF_ERROR) {
      tendon_length = 0.0;
      cos_factor = cos(atan(muscle_width / length));
      fiberLength = length / cos_factor;

        activeForce =  getActiveForceLengthCurve().calcValue(SimTK::Vector(1, fiberLength / _optimalFiberLength)) * aActivation * _maxIsometricForce;
       if (activeForce < 0.0) activeForce = 0.0;

        passiveForce = calcNonzeroPassiveForce(s, fiberLength / _optimalFiberLength, 0.0) * _maxIsometricForce;

        setPassiveForce(s, passiveForce );
        setStateVariableValue(s, STATE_FIBER_LENGTH_NAME,  fiberLength);
        tendonForce = (activeForce + passiveForce) * cos_factor;
        setActuation(s, tendonForce);
        setTendonForce(s, tendonForce);
      return tendonForce;
   } else if (length < _tendonSlackLength) {
        setStateVariableValue(s, STATE_FIBER_LENGTH_NAME,  muscle_width);
        setPassiveForce(s, 0.0);
        setActuation(s, 0.0);
        setTendonForce(s, 0.0);
      return 0.0;
   } else {
      fiberLength = _optimalFiberLength;
      cos_factor = cos(calcPennation(fiberLength, _optimalFiberLength, _pennationAngleAtOptimal));  
      tendon_length = length - fiberLength * cos_factor;

      /* Check to make sure tendon is not shorter than its slack length. If it
       * is, set the length to its slack length and re-compute fiber length.
       */
      if (tendon_length < _tendonSlackLength) {
         tendon_length = _tendonSlackLength;
         cos_factor = cos(atan(muscle_width / (length - tendon_length)));
         fiberLength = (length - tendon_length) / cos_factor;
         if (fiberLength < muscle_width) fiberLength = muscle_width;
      }

   }

   // Muscle-tendon force is found using an iterative method. First, you guess
   // the length of the muscle fibers and the length of the tendon, and
   // calculate their respective forces. If the forces match (are within
   // ERROR_LIMIT of each other), stop; else change the length guesses based
   // on the error and try again.
   for (i = 0; i < MAX_ITERATIONS; i++) {
        activeForce = getActiveForceLengthCurve().calcValue(SimTK::Vector(1, fiberLength / _optimalFiberLength)) * aActivation;
      if (activeForce < 0.0) activeForce = 0.0;

        passiveForce = calcNonzeroPassiveForce(s, fiberLength / _optimalFiberLength, 0.0);
      if (passiveForce < 0.0) passiveForce = 0.0;

      fiber_force = (activeForce + passiveForce ) * _maxIsometricForce * cos_factor;

      tendon_strain = (tendon_length / _tendonSlackLength - 1.0);
      if (tendon_strain < 0.0)
         tendon_force = 0.0;
      else
         tendon_force = getTendonForceLengthCurve().calcValue(SimTK::Vector(1, tendon_strain)) * _maxIsometricForce;
      setActuation(s, tendon_force);
      setTendonForce(s, tendon_force);

      old_error_force = error_force;
 
      error_force = tendon_force - fiber_force;

      if (DABS(error_force) <= ERROR_LIMIT) // muscle-tendon force found!
         break;

      if (i == 0)
         old_error_force = error_force;

      if (DSIGN(error_force) != DSIGN(old_error_force)) {
         percent = DABS(error_force) / (DABS(error_force) + DABS(old_error_force));
         tmp_fiber_length = old_fiber_length;
         old_fiber_length = fiberLength;
         fiberLength += percent * (tmp_fiber_length - fiberLength);
      } else {
         // Estimate the stiffnesses of the tendon and the fibers. If tendon
         // stiffness is too low, then the next length guess will overshoot
         // the equilibrium point. So we artificially raise it using the
         // normalized muscle force. (_activeForce+_passiveForce) is the
         // normalized force for the current fiber length, and we assume that
         // the equilibrium length is close to this current length. So we want
         // to get force = (_activeForce+_passiveForce) from the tendon as well.
         // We hope this will happen by setting the tendon stiffness to
         // (_activeForce+_passiveForce) times its maximum stiffness.
            double tendon_elastic_modulus = 1200.0;
            double tendon_max_stress = 32.0;

         tendon_stiffness = getTendonForceLengthCurve().calcValue(SimTK::Vector(1, tendon_strain)) *
                _maxIsometricForce / _tendonSlackLength;

         min_tendon_stiffness = (activeForce + passiveForce) *
             tendon_elastic_modulus * _maxIsometricForce /
             (tendon_max_stress * _tendonSlackLength);

         if (tendon_stiffness < min_tendon_stiffness)
            tendon_stiffness = min_tendon_stiffness;

         fiber_stiffness = _maxIsometricForce / _optimalFiberLength *
             (getActiveForceLengthCurve().calcValue(SimTK::Vector(1, fiberLength / _optimalFiberLength))  +
            calcNonzeroPassiveForce(s, fiberLength / _optimalFiberLength, 0.0));

         // determine how much the fiber and tendon lengths have to
         // change to make the error_force zero. But don't let the
          // length change exceed half the optimal fiber length because
          // that's too big a change to make all at once.
         length_change = fabs(error_force/(fiber_stiffness / cos_factor + tendon_stiffness));

         if (fabs(length_change / _optimalFiberLength) > 0.5)
            length_change = 0.5 * _optimalFiberLength;

         // now change the fiber length depending on the sign of the error
         // and the sign of the fiber stiffness (which equals the sign of
         // the slope of the muscle's force-length curve).
         old_fiber_length = fiberLength;


         if (error_force > 0.0)
             fiberLength += length_change;
         else
             fiberLength -= length_change;


      }

      cos_factor = cos(calcPennation(fiberLength, _optimalFiberLength, _pennationAngleAtOptimal));
      tendon_length = length - fiberLength * cos_factor;

      // Check to make sure tendon is not shorter than its slack length. If it is,
      // set the length to its slack length and re-compute fiber length.
      if (tendon_length < _tendonSlackLength) {
         tendon_length = _tendonSlackLength;
         cos_factor = cos(atan(muscle_width / (length - tendon_length)));
         fiberLength = (length - tendon_length) / cos_factor;
      }
   }

   setStateVariableValue(s, STATE_FIBER_LENGTH_NAME,  fiberLength);
   setPassiveForce(s, passiveForce * _maxIsometricForce);

    return tendon_force;
}

// Satisfy the ActivationFiberLengthMuscle_Deprecated interface
double Schutte1993Muscle_Deprecated::calcPassiveForce(const SimTK::State& s, double aNormFiberLength) const
{
    return calcNonzeroPassiveForce(s,aNormFiberLength, 0.0);
}
double Schutte1993Muscle_Deprecated::calcActiveForce(const SimTK::State& s, double aNormFiberLength) const
{
    return getActiveForceLengthCurve().calcValue(SimTK::Vector(1, aNormFiberLength) );
}
