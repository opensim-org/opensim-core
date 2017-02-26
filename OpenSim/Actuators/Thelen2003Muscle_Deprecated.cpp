/* -------------------------------------------------------------------------- *
 *                 OpenSim:  Thelen2003Muscle_Deprecated.cpp                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include "Thelen2003Muscle_Deprecated.h"
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Simulation/Model/Model.h>

//==============================================================================
// STATICS
//==============================================================================
using namespace std;
using namespace OpenSim;

//==============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//==============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Thelen2003Muscle_Deprecated::Thelen2003Muscle_Deprecated()
{
    constructProperties();
}

//_____________________________________________________________________________
/**
 * Constructor.
 */
Thelen2003Muscle_Deprecated::Thelen2003Muscle_Deprecated
   (const std::string& name, double maxIsometricForce,
    double optimalFiberLength, double tendonSlackLength,
    double pennationAngle) 
:   Super()
{
    constructProperties();

    setName(name);
    setMaxIsometricForce(maxIsometricForce);
    setOptimalFiberLength(optimalFiberLength);
    setTendonSlackLength(tendonSlackLength);
    setPennationAngleAtOptimalFiberLength(pennationAngle);
}


//_____________________________________________________________________________
// Allocate and initialize properties.
void Thelen2003Muscle_Deprecated::constructProperties()
{
    constructProperty_activation_time_constant(0.01);
    constructProperty_deactivation_time_constant(0.04);
    constructProperty_Vmax(10.0);
    constructProperty_Vmax0(5.0);
    constructProperty_FmaxTendonStrain(0.033);
    constructProperty_FmaxMuscleStrain(0.6);
    constructProperty_KshapeActive(0.5);
    constructProperty_KshapePassive(4.0);
    constructProperty_damping(0.05);
    constructProperty_Af(0.3);
    constructProperty_Flen(1.8);
}


//==============================================================================
// GET
//==============================================================================

//==============================================================================
// COMPUTATION
//==============================================================================
//_____________________________________________________________________________
/**
 * Compute the actuation for the muscle. This function assumes
 * that computeDerivatives has already been called.
 *
 * This function is based on muscle_deriv_func9 from derivs.c (old pipeline code)
 */
double Thelen2003Muscle_Deprecated::
computeActuation(const SimTK::State& s) const
{
    double tendonForce;
    double passiveForce;
    double activationDeriv;
    double fiberLengthDeriv;

    double norm_tendon_length, ca;
    double norm_muscle_tendon_length, pennation_angle;
    double excitation = getExcitation(s);

    /* Normalize the muscle states */
    double activation = getActivation(s);
    double normFiberLength = getFiberLength(s) / _optimalFiberLength;

    // Maximum contraction velocity is an activation scaled value
    double Vmax = getVmax();
    if (activation < 1.0) {
        Vmax = getVmax0() + activation*(Vmax-getVmax0());
    }
    Vmax = Vmax*_optimalFiberLength;

    /* Compute normalized muscle state derivatives */
    if (excitation >= activation) {
        activationDeriv = (excitation - activation) / getActivationTimeConstant();
    } else {
        activationDeriv = (excitation - activation) / getDeactivationTimeConstant();
    }

    pennation_angle = calcPennation( normFiberLength, 1.0, _pennationAngleAtOptimal);
    ca = cos(pennation_angle);

    norm_muscle_tendon_length = getLength(s) / _optimalFiberLength;
    norm_tendon_length = norm_muscle_tendon_length - normFiberLength * ca;

    tendonForce = calcTendonForce(s,norm_tendon_length);
    passiveForce = calcPassiveForce(s,normFiberLength);
    double activeForce = calcActiveForce(s,normFiberLength);

    // NOTE: SimmZajacMuscle has this check, but Darryl's muscle didn't seem to
    // if (activeForce < 0.0) activeForce = 0.0;

    /* If pennation equals 90 degrees, fiber length equals muscle width and fiber
    * velocity goes to zero.  Pennation will stay at 90 until tendon starts to
    * pull, then "stiff tendon" approximation is used to calculate approximate
    * fiber velocity.
    */
    if (EQUAL_WITHIN_ERROR(ca, 0.0)) {
      if (EQUAL_WITHIN_ERROR(tendonForce, 0.0)) {
          fiberLengthDeriv = 0.0;
            // ms->fiber_velocity = 0.0;
      } else {
         double h = norm_muscle_tendon_length - _tendonSlackLength;
         double w = _optimalFiberLength * sin(_pennationAngleAtOptimal);
         double new_fiber_length = sqrt(h*h + w*w) / _optimalFiberLength;
            double new_pennation_angle = calcPennation( new_fiber_length, 1.0, _pennationAngleAtOptimal);
         double new_ca = cos(new_pennation_angle);
         fiberLengthDeriv = getLengtheningSpeed(s) / (Vmax * new_ca);
        }
    } else {

        double velocity_dependent_force = tendonForce / ca - passiveForce;
    //  if (velocity_dependent_force < 0.0)  velocity_dependent_force = 0.0;

        fiberLengthDeriv = calcFiberVelocity(s,activation,activeForce,velocity_dependent_force);
    }


    /* Un-normalize the muscle state derivatives and forces. */
    /* Note: Do not need to Un-Normalize activation dynamics equation since activation, deactivation parameters
     specified in muscle file are now independent of time scale */

    setActivationDeriv(s, activationDeriv );
    setFiberLengthDeriv(s, fiberLengthDeriv * Vmax );

    tendonForce = tendonForce *  _maxIsometricForce;
    setActuation(s, tendonForce);
    setTendonForce(s, tendonForce);
    setPassiveForce( s, passiveForce * _maxIsometricForce);

    //cout << "ThelenMuscle computeActuation " << getName() << "  t=" << s.getTime() << " force = " << tendonForce << endl;

    return( tendonForce );
}

//_____________________________________________________________________________
/**
 * From cmg_dt.c - calc_tendon_force_dt
 *
 * CALC_TENDON_FORCE_DT: this routine calculates the force in tendon by finding
 * tendon strain and using it in an exponential function (JBME 2003 - Thelen)
 * FmaxTendonStrain - Function is parameterized by the tendon strain due to maximum isometric muscle force
 *     This should be specified as a dynamic parameter in the muscle file
 *
 * @param aNormTendonLength Normalized length of the tendon.
 * @return The force in the tendon.
 */
double Thelen2003Muscle_Deprecated::
calcTendonForce(const SimTK::State& s, double aNormTendonLength) const
{
    double norm_resting_length = _tendonSlackLength / _optimalFiberLength;
    double tendon_strain =  (aNormTendonLength - norm_resting_length) / norm_resting_length;

    double KToe = 3;
    double ToeStrain = 0.609*getFmaxTendonStrain();
    double ToeForce = 0.333333;
    double klin = 1.712/getFmaxTendonStrain();

    double tendon_force;
    if (tendon_strain>ToeStrain)
        tendon_force = klin*(tendon_strain-ToeStrain)+ToeForce;
    else if (tendon_strain>0) 
        tendon_force = ToeForce*(exp(KToe*tendon_strain/ToeStrain)-1.0)/(exp(KToe)-1);
    else
        tendon_force=0.;

    // Add on a small stiffness so that tendon never truly goes slack for non-zero tendon lengths
    tendon_force+=0.001*(1.+tendon_strain);

    return tendon_force;
}

//_____________________________________________________________________________
/**
 * From gmc.dt.c - calc_passive_fiber_force_dt
 *
 * CALC_PASSIVE_FIBER_FORCE_DT: written by Darryl Thelen
 * this routine calculates the passive force in the muscle fibers using
 * an exponential-linear function instead of cubic splines.
 * It always returns a non-zero force for all muscle lengths
 * This equation is parameterized using the following dynamic parameters
 * which must be specified in the muscle file
 * Dynamic Parameters:
 *   FmaxMuscleStrain - passive muscle strain due to the application of 
 *                      maximum isometric muscle force
 *   KshapePassive - exponential shape factor
 *
 *  The normalized force due to passive stretch is given by
 *  For L < (1+maxStrain)*Lo
 *      f/f0 = exp(ks*(L-1)/maxStrain) / exp(ks)
 *
 * @param aNormFiberLength Normalized length of the muscle fiber.
 * @return The passive force in the muscle fibers.
 */
double Thelen2003Muscle_Deprecated::
calcPassiveForce(const SimTK::State& s, double aNormFiberLength) const
{
    double passive_force;

    if (aNormFiberLength>(1+getFmaxMuscleStrain())) { // Switch to a linear model at large forces
        double slope=(getKshapePassive()/getFmaxMuscleStrain())*(exp(getKshapePassive()*(1.0+getFmaxMuscleStrain()-1.0)/getFmaxMuscleStrain())) / (exp(getKshapePassive()));
        passive_force=1.0+slope*(aNormFiberLength-(1.0+getFmaxMuscleStrain()));
    }
    else
        passive_force = (exp(getKshapePassive()*(aNormFiberLength-1.0)/getFmaxMuscleStrain())) / (exp(getKshapePassive()));

    return passive_force;
}

//_____________________________________________________________________________
/**
 * From gmc.dt.c - calc_active_force_dt
 *
 * CALC_ACTIVE_FORCE_DT: this routine calculates the active component of force
 * in the muscle fibers. It uses the current fiber length to interpolate the
 * active force-length curve - described by Gaussian curve as in Thelen, JBME 2003
 * *
 * @param aNormFiberLength Normalized length of the muscle fiber.
 * @return The active force in the muscle fibers.
 */
double Thelen2003Muscle_Deprecated::
calcActiveForce(const SimTK::State& s, double aNormFiberLength) const
{
    double x=-(aNormFiberLength-1.)*(aNormFiberLength-1.)/getKshapeActive();
    return exp(x);
}

//_____________________________________________________________________________
/**
 * From gmc_dt.c - calc_norm_fiber_velocity_dt
 *
 * CALC_NORM_FIBER_VELOCITY_DT: written by Darryl Thelen
 * this routine calculates the normalized fiber velocity (scaled to Vmax) by inverting the
 * muscle force-velocity-activation relationship (Thelen, JBME 2003)
 * This equation is parameterized using the following dynamic parameters
 * which must be specified in the muscle file
 * Dynamic Parameters:
 *   damping - normalized passive damping in parallel with contractile element
 *   Af - velocity shape factor from Hill's equation
 *   Flen   - Maximum normalized force when muscle is lengthening
 *
 * @param aActivation Activation of the muscle.
 * @param aActiveForce Active force in the muscle fibers.
 * @param aVelocityDependentForce Force value that depends on fiber velocity.
 * @return The velocity of the muscle fibers.
 */
double Thelen2003Muscle_Deprecated::
calcFiberVelocity(const SimTK::State& s, double aActivation, double aActiveForce, double aVelocityDependentForce) const
{
    double epsilon=1.e-6;

    // Don't allow zero activation
    if (aActivation<epsilon) 
        aActivation=epsilon;

    double Fa = aActivation*aActiveForce;
    double Fv = aVelocityDependentForce;

    double norm_fiber_velocity;
    if (Fv<Fa) {        // Muscle shortening
        if (Fv<0) { // Extend the force-velocity curve for negative forces using linear extrapolation
            double F0=0;
            double b=Fa+F0/getAf();
            double fv0 = (F0-Fa)/(b+getDamping());
            double F1=epsilon;
            b=Fa+F1/getAf();
            double fv1 = (F1-Fa)/(b+getDamping());
            b = (F1-F0)/(fv1-fv0);
            norm_fiber_velocity = fv0 + (Fv-F0)/b;
        }
        else {
            double b=Fa+Fv/getAf();
            norm_fiber_velocity = (Fv-Fa)/(b+getDamping());
        }
    }
    else if (Fv<(.95*Fa*getFlen())) {
        double b=(2+2./getAf())*(Fa*getFlen()-Fv)/(getFlen()-1.);
        norm_fiber_velocity = (Fv-Fa)/(b+getDamping());
    }
    else {  // Extend the force-velocity curve for forces that exceed maximum using linear extrapolation
        double F0=.95*Fa*getFlen();
        double b=(2+2./getAf())*(Fa*getFlen()-F0)/(getFlen()-1.);
        double fv0 = (F0-Fa)/(b+getDamping());
        double F1=(.95+epsilon)*Fa*getFlen();
        b=(2+2./getAf())*(Fa*getFlen()-F1)/(getFlen()-1.);
        double fv1 = (F1-Fa)/(b+getDamping());
        b = (fv1-fv0)/(F1-F0);
        norm_fiber_velocity = fv0 + b*(Fv-F0);
    }

    return norm_fiber_velocity;
}

//_____________________________________________________________________________
/**
 * Find the force produced by an actuator (the musculotendon unit), assuming
 * static equilibrium. Using the total muscle-tendon length, it finds the
 * fiber and tendon lengths so that the forces in each match. This routine
 * takes pennation angle into account, so its definition of static equilibrium
 * is when tendon_force = fiber_force * cos(pennation_angle). This function
 * will modify the object's values for length, fiberLength, activeForce, 
 * and passiveForce.
 *
 * @param aActivation Activation of the muscle.
 * @return The isometric force in the muscle.
 */
double Thelen2003Muscle_Deprecated::
computeIsometricForce(SimTK::State& s, double aActivation) const
{
#define MAX_ITERATIONS 100
#define ERROR_LIMIT 0.01

   int i;
   double length,tendon_length, fiber_force, tmp_fiber_length, min_tendon_stiffness;
   double cos_factor, fiber_stiffness;
   double old_fiber_length{}, length_change, tendon_stiffness, percent;
   double error_force = 0.0, old_error_force, tendon_force, norm_tendon_length;
   double passiveForce;
   double fiberLength;

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

        double activeForce = calcActiveForce(s, fiberLength / _optimalFiberLength)  * aActivation;
        if (activeForce < 0.0) activeForce = 0.0;

        passiveForce = calcPassiveForce(s, fiberLength / _optimalFiberLength);
        if (passiveForce < 0.0) passiveForce = 0.0;

        setPassiveForce(s, passiveForce );
        setStateVariableValue(s, STATE_FIBER_LENGTH_NAME, fiberLength);
        tendon_force = (activeForce + passiveForce) * _maxIsometricForce * cos_factor;
        setActuation(s, tendon_force);
        setTendonForce(s, tendon_force);
        return tendon_force;
   } else if (length < _tendonSlackLength) {
        setPassiveForce(s, 0.0);
      setStateVariableValue(s, STATE_FIBER_LENGTH_NAME, muscle_width);
      _model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);
      setActuation(s, 0.0);
      setTendonForce(s, 0.0);
      return 0.0;
   } else {
      fiberLength = _optimalFiberLength;
      cos_factor = cos(calcPennation( fiberLength, _optimalFiberLength,  _pennationAngleAtOptimal ));  
      tendon_length = length - fiberLength * cos_factor;

      /* Check to make sure tendon is not shorter than its slack length. If it
       * is, set the length to its slack length and re-compute fiber length.
       */

      if (tendon_length < _tendonSlackLength) {
         tendon_length = _tendonSlackLength;
         cos_factor = cos(atan(muscle_width / (length - tendon_length)));
         fiberLength = (length - tendon_length) / cos_factor;
         if (fiberLength < muscle_width)
           fiberLength = muscle_width;
      }

   }

   // Muscle-tendon force is found using an iterative method. First, you guess
   // the length of the muscle fibers and the length of the tendon, and
   // calculate their respective forces. If the forces match (are within
   // ERROR_LIMIT of each other), stop; else change the length guesses based
   // on the error and try again.
   for (i = 0; i < MAX_ITERATIONS; i++) {
        double activeForce = calcActiveForce(s, fiberLength/ _optimalFiberLength) * aActivation;
      if( activeForce <  0.0) activeForce = 0.0;

      passiveForce =  calcPassiveForce(s, fiberLength / _optimalFiberLength);
      if (passiveForce < 0.0) passiveForce = 0.0;

      fiber_force = (activeForce + passiveForce) * _maxIsometricForce * cos_factor;

      norm_tendon_length = tendon_length / _optimalFiberLength;
      tendon_force = calcTendonForce(s, norm_tendon_length) * _maxIsometricForce;
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
         fiberLength = fiberLength + percent * (tmp_fiber_length - fiberLength);
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

         tendon_stiffness = calcTendonForce(s, norm_tendon_length) *
                _maxIsometricForce / _tendonSlackLength;

         min_tendon_stiffness = (activeForce + passiveForce) *
             tendon_elastic_modulus * _maxIsometricForce /
             (tendon_max_stress * _tendonSlackLength);

         if (tendon_stiffness < min_tendon_stiffness)
            tendon_stiffness = min_tendon_stiffness;

         fiber_stiffness = _maxIsometricForce / _optimalFiberLength *
            (calcActiveForce(s, fiberLength / _optimalFiberLength)  +
            calcPassiveForce(s, fiberLength / _optimalFiberLength));

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
      cos_factor = cos(calcPennation(fiberLength, _optimalFiberLength, _pennationAngleAtOptimal ));
      tendon_length = length - fiberLength * cos_factor;

      // Check to make sure tendon is not shorter than its slack length. If it is,
      // set the length to its slack length and re-compute fiber length.
      if (tendon_length < _tendonSlackLength) {
         tendon_length = _tendonSlackLength;
         cos_factor = cos(atan(muscle_width / (length - tendon_length)));
         fiberLength = (length - tendon_length) / cos_factor ;
      }
    }

    setPassiveForce(s, passiveForce );
    _model->getMultibodySystem().realize(s, SimTK::Stage::Position);
    setStateVariableValue(s, STATE_FIBER_LENGTH_NAME,  fiberLength);

//cout << "ThelenMuscle computeIsometricForce " << getName() << "  t=" << s.getTime() << " force = " << tendon_force << endl;

   return tendon_force;
}
