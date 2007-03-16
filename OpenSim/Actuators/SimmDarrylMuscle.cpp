// SimmDarrylMuscle.cpp
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

//=============================================================================
// INCLUDES
//=============================================================================
#include "SimmDarrylMuscle.h"
#include "SimmMacros.h"
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/DebugUtilities.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

const int SimmDarrylMuscle::STATE_ACTIVATION = 0;
const int SimmDarrylMuscle::STATE_FIBER_LENGTH = 1;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmDarrylMuscle::SimmDarrylMuscle() :
   AbstractSimmMuscle(),
	_maxIsometricForce(_maxIsometricForceProp.getValueDbl()),
	_optimalFiberLength(_optimalFiberLengthProp.getValueDbl()),
	_tendonSlackLength(_tendonSlackLengthProp.getValueDbl()),
	_pennationAngle(_pennationAngleProp.getValueDbl()),
	_activationTimeConstant(_activationTimeConstantProp.getValueDbl()),
	_deactivationTimeConstant(_deactivationTimeConstantProp.getValueDbl()),
	_vmax(_vmaxProp.getValueDbl()),
	_vmax0(_vmax0Prop.getValueDbl()),
	_fmaxTendonStrain(_fmaxTendonStrainProp.getValueDbl()),
	_fmaxMuscleStrain(_fmaxMuscleStrainProp.getValueDbl()),
	_kShapeActive(_kShapeActiveProp.getValueDbl()),
	_kShapePassive(_kShapePassiveProp.getValueDbl()),
	_damping(_dampingProp.getValueDbl()),
	_af(_afProp.getValueDbl()),
	_flen(_flenProp.getValueDbl())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmDarrylMuscle::~SimmDarrylMuscle()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMuscle SimmDarrylMuscle to be copied.
 */
SimmDarrylMuscle::SimmDarrylMuscle(const SimmDarrylMuscle &aMuscle) :
   AbstractSimmMuscle(aMuscle),
	_maxIsometricForce(_maxIsometricForceProp.getValueDbl()),
	_optimalFiberLength(_optimalFiberLengthProp.getValueDbl()),
	_tendonSlackLength(_tendonSlackLengthProp.getValueDbl()),
	_pennationAngle(_pennationAngleProp.getValueDbl()),
	_activationTimeConstant(_activationTimeConstantProp.getValueDbl()),
	_deactivationTimeConstant(_deactivationTimeConstantProp.getValueDbl()),
	_vmax(_vmaxProp.getValueDbl()),
	_vmax0(_vmax0Prop.getValueDbl()),
	_fmaxTendonStrain(_fmaxTendonStrainProp.getValueDbl()),
	_fmaxMuscleStrain(_fmaxMuscleStrainProp.getValueDbl()),
	_kShapeActive(_kShapeActiveProp.getValueDbl()),
	_kShapePassive(_kShapePassiveProp.getValueDbl()),
	_damping(_dampingProp.getValueDbl()),
	_af(_afProp.getValueDbl()),
	_flen(_flenProp.getValueDbl())
{
	setNull();
	setupProperties();
	copyData(aMuscle);
}

//_____________________________________________________________________________
/**
 * Copy this muscle point and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmDarrylMuscle.
 */
Object* SimmDarrylMuscle::copy() const
{
	SimmDarrylMuscle *musc = new SimmDarrylMuscle(*this);
	return(musc);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SimmDarrylMuscle to another.
 *
 * @param aMuscle SimmDarrylMuscle to be copied.
 */
void SimmDarrylMuscle::copyData(const SimmDarrylMuscle &aMuscle)
{
	_maxIsometricForce = aMuscle._maxIsometricForce;
	_optimalFiberLength = aMuscle._optimalFiberLength;
	_tendonSlackLength = aMuscle._tendonSlackLength;
	_pennationAngle = aMuscle._pennationAngle;
	_activationTimeConstant = aMuscle._activationTimeConstant;
	_deactivationTimeConstant = aMuscle._deactivationTimeConstant;
	_vmax = aMuscle._vmax;
	_vmax0 = aMuscle._vmax0;
	_fmaxTendonStrain = aMuscle._fmaxTendonStrain;
	_fmaxMuscleStrain = aMuscle._fmaxMuscleStrain;
	_kShapeActive = aMuscle._kShapeActive;
	_kShapePassive = aMuscle._kShapePassive;
	_damping = aMuscle._damping;
	_af = aMuscle._af;
	_flen = aMuscle._flen;
}

//_____________________________________________________________________________
/**
 * Set the data members of this SimmDarrylMuscle to their null values.
 */
void SimmDarrylMuscle::setNull()
{
	setType("SimmDarrylMuscle");

	setNumControls(1); setNumStates(2); setNumPseudoStates(0);
	bindControl(0, _excitation, "excitation");
	bindState(STATE_ACTIVATION, _activation, "activation");
	bindState(STATE_FIBER_LENGTH, _fiberLength, "fiber_length");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmDarrylMuscle::setupProperties()
{
	_maxIsometricForceProp.setName("max_isometric_force");
	_maxIsometricForceProp.setValue(0.0);
	_propertySet.append(&_maxIsometricForceProp);

	_optimalFiberLengthProp.setName("optimal_fiber_length");
	_optimalFiberLengthProp.setValue(0.0);
	_propertySet.append(&_optimalFiberLengthProp);

	_tendonSlackLengthProp.setName("tendon_slack_length");
	_tendonSlackLengthProp.setValue(0.0);
	_propertySet.append(&_tendonSlackLengthProp);

	_pennationAngleProp.setName("pennation_angle");
	_pennationAngleProp.setValue(0.0);
	_propertySet.append(&_pennationAngleProp);

	_activationTimeConstantProp.setName("activation_time_constant");
	_activationTimeConstantProp.setValue(0.0);
	_propertySet.append(&_activationTimeConstantProp);

	_deactivationTimeConstantProp.setName("deactivation_time_constant");
	_deactivationTimeConstantProp.setValue(0.0);
	_propertySet.append(&_deactivationTimeConstantProp);

	_vmaxProp.setName("Vmax");
	_vmaxProp.setValue(0.0);
	_propertySet.append(&_vmaxProp);

	_vmax0Prop.setName("Vmax0");
	_vmax0Prop.setValue(0.0);
	_propertySet.append(&_vmax0Prop);

	_fmaxTendonStrainProp.setName("FmaxTendonStrain");
	_fmaxTendonStrainProp.setValue(0.0);
	_propertySet.append(&_fmaxTendonStrainProp);

	_fmaxMuscleStrainProp.setName("FmaxMuscleStrain");
	_fmaxMuscleStrainProp.setValue(0.0);
	_propertySet.append(&_fmaxMuscleStrainProp);

	_kShapeActiveProp.setName("KshapeActive");
	_kShapeActiveProp.setValue(0.0);
	_propertySet.append(&_kShapeActiveProp);

	_kShapePassiveProp.setName("KshapePassive");
	_kShapePassiveProp.setValue(0.0);
	_propertySet.append(&_kShapePassiveProp);

	_dampingProp.setName("damping");
	_dampingProp.setValue(0.0);
	_propertySet.append(&_dampingProp);

	_afProp.setName("Af");
	_afProp.setValue(0.0);
	_propertySet.append(&_afProp);

	_flenProp.setName("Flen");
	_flenProp.setValue(0.0);
	_propertySet.append(&_flenProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this SimmDarrylMuscle.
 */
void SimmDarrylMuscle::setup(AbstractModel* aModel)
{
	// Base class
	AbstractSimmMuscle::setup(aModel);

	// Reasonable initial activation value
	_activation = 0.01;

	// Compute isometric force to get starting value
	// of _fiberLength.
	computeIsometricForce(_activation);
	//_fiberLength = 1.4*_optimalFiberLength;
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
SimmDarrylMuscle& SimmDarrylMuscle::operator=(const SimmDarrylMuscle &aMuscle)
{
	// BASE CLASS
	AbstractSimmMuscle::operator=(aMuscle);

	copyData(aMuscle);

	return(*this);
}

//=============================================================================
// TYPE REGISTRATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Register the types used by this class.
 */
void SimmDarrylMuscle::registerTypes()
{
	// Base class
	AbstractSimmMuscle::registerTypes();
}

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Scale the muscle.
 *
 * @param aScaleSet XYZ scale factors for the bodies
 * @return Whether or not the muscle was scaled successfully
 */
void SimmDarrylMuscle::scale(const ScaleSet& aScaleSet)
{
	AbstractSimmMuscle::scale(aScaleSet);

	// some force-generating parameters are scaled in postScale(),
	// so as of now there is nothing else to do here...
}

//_____________________________________________________________________________
/**
 * Perform computations that need to happen after the muscle is scaled.
 * For this object, that entails comparing the musculotendon length
 * before and after scaling, and scaling some of the force-generating
 * properties a proportional amount.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 */
void SimmDarrylMuscle::postScale(const ScaleSet& aScaleSet)
{
	// Base class
	AbstractSimmMuscle::postScale(aScaleSet);

	if (_preScaleLength > 0.0)
	{
		double scaleFactor = getLength() / _preScaleLength;

		_optimalFiberLength *= scaleFactor;
		_tendonSlackLength *= scaleFactor;
		//_maxIsometricForce *= scaleFactor;

		_preScaleLength = 0.0;
	}
}

//_____________________________________________________________________________
/**
 * Compute the derivatives of the muscle states.
 *
 * @param rDYDT the state derivatives are returned here.
 */
void SimmDarrylMuscle::computeStateDerivatives(double rDYDT[])
{
	if (!rDYDT)
		return;

	rDYDT[STATE_ACTIVATION] = _activationDeriv;
	rDYDT[STATE_FIBER_LENGTH] = _fiberLengthDeriv;
}

//_____________________________________________________________________________
/**
 * Compute the actuation for the muscle. This function assumes
 * that computeDerivatives has already been called.
 *
 * This function is based on muscle_deriv_func9 from derivs.c (old pipeline code)
 */
void SimmDarrylMuscle::computeActuation()
{
	// Base Class (to calculate speed)
	AbstractSimmMuscle::computeActuation();

   double normState[2], normStateDeriv[2], norm_tendon_length, ca;
   double norm_muscle_tendon_length, pennation_angle;

   /* Normalize the muscle states */
   normState[STATE_ACTIVATION] = _activation;
   normState[STATE_FIBER_LENGTH] = _fiberLength / _optimalFiberLength;

	// Maximum contraction velocity is an activation scaled value
	double Vmax = _vmax;
	if (normState[STATE_ACTIVATION]<1.0)
		Vmax = _vmax0 + normState[STATE_ACTIVATION]*(Vmax-_vmax0);
	Vmax = Vmax*_optimalFiberLength;

   /* Compute normalized muscle state derivatives */
   if (_excitation >= normState[STATE_ACTIVATION])
      normStateDeriv[STATE_ACTIVATION] = (_excitation - normState[STATE_ACTIVATION]) / _activationTimeConstant;
   else
      normStateDeriv[STATE_ACTIVATION] = (_excitation - normState[STATE_ACTIVATION]) / _deactivationTimeConstant;

	pennation_angle = AbstractSimmMuscle::calcPennation(normState[STATE_FIBER_LENGTH], 1.0, _pennationAngle);
   ca = cos(pennation_angle);

   norm_muscle_tendon_length = getLength() / _optimalFiberLength;
   norm_tendon_length = norm_muscle_tendon_length - normState[STATE_FIBER_LENGTH] * ca;

   _tendonForce = calcTendonForce(norm_tendon_length);
   _passiveForce = calcPassiveForce(normState[STATE_FIBER_LENGTH]);
	_activeForce = calcActiveForce(normState[STATE_FIBER_LENGTH]);
	
	// NOTE: SimmZajacMuscle has this check, but Darryl's muscle didn't seem to
	// if (_activeForce < 0.0) _activeForce = 0.0;
 
   /* If pennation equals 90 degrees, fiber length equals muscle width and fiber
    * velocity goes to zero.  Pennation will stay at 90 until tendon starts to
    * pull, then "stiff tendon" approximation is used to calculate approximate
    * fiber velocity.
    */
   if (EQUAL_WITHIN_ERROR(ca, 0.0))
   {
      if (EQUAL_WITHIN_ERROR(_tendonForce, 0.0))
      {
         normStateDeriv[STATE_FIBER_LENGTH] = 0.0;
			// ms->fiber_velocity = 0.0;
		}
		else 
		{
         double h = norm_muscle_tendon_length - _tendonSlackLength;
         double w = _optimalFiberLength * sin(_pennationAngle);
         double new_fiber_length = sqrt(h*h + w*w) / _optimalFiberLength;
			double new_pennation_angle = AbstractSimmMuscle::calcPennation(new_fiber_length, 1.0, _pennationAngle);
         double new_ca = cos(new_pennation_angle);
         normStateDeriv[STATE_FIBER_LENGTH] = getSpeed() / (Vmax * new_ca);
		}
	}
   else
   {
      double velocity_dependent_force = _tendonForce / ca - _passiveForce;
      normStateDeriv[STATE_FIBER_LENGTH] = calcFiberVelocity(normState[STATE_ACTIVATION], _activeForce, velocity_dependent_force);
   }

   /* Un-normalize the muscle state derivatives and forces. */
   /* Note: Do not need to Un-Normalize activation dynamics equation since activation, deactivation parameters
     specified in muscle file are now independent of time scale */
   _activationDeriv = normStateDeriv[STATE_ACTIVATION];
   _fiberLengthDeriv = normStateDeriv[STATE_FIBER_LENGTH] * Vmax;

	_tendonForce *= _maxIsometricForce;
	_passiveForce *= _maxIsometricForce;
	_activeForce *= _maxIsometricForce;
	//ms->tendon_length = norm_tendon_length*(*(ms->optimal_fiber_length));

	setForce(_tendonForce);
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
double SimmDarrylMuscle::calcTendonForce(double aNormTendonLength) const
{
   double norm_resting_length = _tendonSlackLength / _optimalFiberLength;
   double tendon_strain =  (aNormTendonLength - norm_resting_length) / norm_resting_length;

	double KToe = 3;
	double ToeStrain = 0.609*_fmaxTendonStrain;
	double ToeForce = 0.333333;
	double klin = 1.712/_fmaxTendonStrain;

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
 *	 KshapePassive - exponential shape factor
 *
 *  The normalized force due to passive stretch is given by
 *  For L < (1+maxStrain)*Lo
 *		f/f0 = exp(ks*(L-1)/maxStrain) / exp(ks)
 *
 * @param aNormFiberLength Normalized length of the muscle fiber.
 * @return The passive force in the muscle fibers.
 */
double SimmDarrylMuscle::calcPassiveForce(double aNormFiberLength) const
{
	double passive_force;

	if (aNormFiberLength>(1+_fmaxMuscleStrain)) { // Switch to a linear model at large forces
		double slope=(_kShapePassive/_fmaxMuscleStrain)*(exp(_kShapePassive*(1.0+_fmaxMuscleStrain-1.0)/_fmaxMuscleStrain)) / (exp(_kShapePassive));
		passive_force=1.0+slope*(aNormFiberLength-(1.0+_fmaxMuscleStrain));
	}
	else
		passive_force = (exp(_kShapePassive*(aNormFiberLength-1.0)/_fmaxMuscleStrain)) / (exp(_kShapePassive));

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
double SimmDarrylMuscle::calcActiveForce(double aNormFiberLength) const
{
	double x=-(aNormFiberLength-1.)*(aNormFiberLength-1.)/_kShapeActive;
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
 *   Flen	- Maximum normalized force when muscle is lengthening
 *
 * @param aActivation Activation of the muscle.
 * @param aActiveForce Active force in the muscle fibers.
 * @param aVelocityDependentForce Force value that depends on fiber velocity.
 * @return The velocity of the muscle fibers.
 */
double SimmDarrylMuscle::calcFiberVelocity(double aActivation, double aActiveForce, double aVelocityDependentForce) const
{
   double epsilon=1.e-6;

   // Don't allow zero activation
	if (aActivation<epsilon) 
		aActivation=epsilon;

	double Fa = aActivation*aActiveForce;
	double Fv = aVelocityDependentForce;

   double norm_fiber_velocity;
	if (Fv<Fa) {		// Muscle shortening
		if (Fv<0) {	// Extend the force-velocity curve for negative forces using linear extrapolation
			double F0=0;
			double b=Fa+F0/_af;
        	double fv0 = (F0-Fa)/(b+_damping);
			double F1=epsilon;
			b=Fa+F1/_af;
        	double fv1 = (F1-Fa)/(b+_damping);
			b = (F1-F0)/(fv1-fv0);
        	norm_fiber_velocity = fv0 + (Fv-F0)/b;
		}
		else {
			double b=Fa+Fv/_af;
			norm_fiber_velocity = (Fv-Fa)/(b+_damping);
		}
	}
	else if (Fv<(.95*Fa*_flen)) {
		double b=(2+2./_af)*(Fa*_flen-Fv)/(_flen-1.);
		norm_fiber_velocity = (Fv-Fa)/(b+_damping);
	}
	else {  // Extend the force-velocity curve for forces that exceed maximum using linear extrapolation
			double F0=.95*Fa*_flen;
			double b=(2+2./_af)*(Fa*_flen-F0)/(_flen-1.);
        	double fv0 = (F0-Fa)/(b+_damping);
			double F1=(.95+epsilon)*Fa*_flen;
			b=(2+2./_af)*(Fa*_flen-F1)/(_flen-1.);
        	double fv1 = (F1-Fa)/(b+_damping);
			b = (fv1-fv0)/(F1-F0);
        	norm_fiber_velocity = fv0 + b*(Fv-F0);
    }

    return norm_fiber_velocity;
}
//_____________________________________________________________________________
/**
 * Compute stress
 */
double SimmDarrylMuscle::getStress() const
{
	return _force / _maxIsometricForce;
}

//_____________________________________________________________________________
/**
 * computeIsometricForce: this function finds the force in a muscle, assuming
 * static equilibrium. Using the total muscle-tendon length, it finds the
 * fiber and tendon lengths so that the forces in each match. This routine
 * takes pennation angle into account, so its definition of static equilibrium
 * is when tendon_force = fiber_force * cos(pennation_angle). This funcion
 * will modify the object's values for _length, _fiberLength, _activeForce, 
 * and _passiveForce.
 *
 * @param aActivation Activation of the muscle.
 * @return The isometric force in the muscle.
 */
double SimmDarrylMuscle::computeIsometricForce(double aActivation)
{
#define MAX_ITERATIONS 100
#define ERROR_LIMIT 0.01

   int i;
   double tendon_length, fiber_force, tmp_fiber_length, min_tendon_stiffness;
   double cos_factor, fiber_stiffness;
   double old_fiber_length, length_change, tendon_stiffness, percent;
   double error_force = 0.0, old_error_force, tendon_force, tendon_strain;
   
   // If the muscle has no fibers, then treat it as a ligament.
   if (_optimalFiberLength < ROUNDOFF_ERROR) {
		// ligaments should be a separate class, so _optimalFiberLength should
		// never be zero.
      return 0.0;
   }

   calculateLength();

   // Make first guess of fiber and tendon lengths. Make fiber length equal to
   // optimal_fiber_length so that you start in the middle of the active+passive
   // force-length curve. Muscle_width is the width, or thickness, of the
   // muscle-tendon unit. It is the shortest allowable fiber length because if
   // the muscle-tendon length is very short, the pennation angle will be 90
   // degrees and the fibers will be vertical (assuming the tendon is horizontal).
   // When this happens, the fibers are as long as the muscle is wide.
   // If the resting tendon length is zero, then set the fiber length equal to
   // the muscle tendon length / cosine_factor, and find its force directly.

   double muscle_width = _optimalFiberLength * sin(_pennationAngle);

   if (_tendonSlackLength < ROUNDOFF_ERROR) {
      tendon_length = 0.0;
      cos_factor = cos(atan(muscle_width / _length));
      _fiberLength = _length / cos_factor;

		_activeForce = calcActiveForce(_fiberLength / _optimalFiberLength) * aActivation;
		if (_activeForce < 0.0)
			_activeForce = 0.0;

		_passiveForce = calcPassiveForce(_fiberLength / _optimalFiberLength);
		if (_passiveForce < 0.0)
			_passiveForce = 0.0;

      return (_activeForce + _passiveForce) * _maxIsometricForce * cos_factor;
   } else if (_length < _tendonSlackLength) {
      _fiberLength = muscle_width;
      tendon_length = _length;
      return 0.0;
   } else {
      _fiberLength = _optimalFiberLength;
      cos_factor = cos(calcPennation(_fiberLength, _optimalFiberLength, _pennationAngle));  
      tendon_length = _length - _fiberLength * cos_factor;

      /* Check to make sure tendon is not shorter than its slack length. If it
       * is, set the length to its slack length and re-compute fiber length.
       */
      if (tendon_length < _tendonSlackLength) {
         tendon_length = _tendonSlackLength;
         cos_factor = cos(atan(muscle_width / (_length - tendon_length)));
         _fiberLength = (_length - tendon_length) / cos_factor;
         if (_fiberLength < muscle_width)
            _fiberLength = muscle_width;
      }
   }

   // Muscle-tendon force is found using an iterative method. First, you guess
   // the length of the muscle fibers and the length of the tendon, and
   // calculate their respective forces. If the forces match (are within
   // ERROR_LIMIT of each other), stop; else change the length guesses based
   // on the error and try again.
   for (i = 0; i < MAX_ITERATIONS; i++) {
		_activeForce = calcActiveForce(_fiberLength / _optimalFiberLength) * aActivation;
      if (_activeForce < 0.0)
         _activeForce = 0.0;

		_passiveForce = calcPassiveForce(_fiberLength / _optimalFiberLength);
      if (_passiveForce < 0.0)
         _passiveForce = 0.0;

      fiber_force = (_activeForce + _passiveForce) * _maxIsometricForce * cos_factor;

      tendon_strain = (tendon_length / _tendonSlackLength - 1.0);
      if (tendon_strain < 0.0)
         tendon_force = 0.0;
      else
         tendon_force = calcTendonForce(tendon_strain) * _maxIsometricForce;

      old_error_force = error_force;
 
      error_force = tendon_force - fiber_force;

      if (DABS(error_force) <= ERROR_LIMIT) // muscle-tendon force found!
         break;

      if (i == 0)
         old_error_force = error_force;

      if (DSIGN(error_force) != DSIGN(old_error_force)) {
         percent = DABS(error_force) / (DABS(error_force) + DABS(old_error_force));
         tmp_fiber_length = old_fiber_length;
         old_fiber_length = _fiberLength;
         _fiberLength += percent * (tmp_fiber_length - _fiberLength);
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

         tendon_stiffness = calcTendonForce(tendon_strain) *
				_maxIsometricForce / _tendonSlackLength;

         min_tendon_stiffness = (_activeForce + _passiveForce) *
	         tendon_elastic_modulus * _maxIsometricForce /
	         (tendon_max_stress * _tendonSlackLength);

         if (tendon_stiffness < min_tendon_stiffness)
            tendon_stiffness = min_tendon_stiffness;

         fiber_stiffness = _maxIsometricForce / _optimalFiberLength *
            (calcActiveForce(_fiberLength / _optimalFiberLength)  +
            calcPassiveForce(_fiberLength / _optimalFiberLength));

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
         old_fiber_length = _fiberLength;

         if (error_force > 0.0)
            _fiberLength += length_change;
         else
            _fiberLength -= length_change;
      }

      cos_factor = cos(calcPennation(_fiberLength, _optimalFiberLength, _pennationAngle));
      tendon_length = _length - _fiberLength * cos_factor;

      // Check to make sure tendon is not shorter than its slack length. If it is,
      // set the length to its slack length and re-compute fiber length.
      if (tendon_length < _tendonSlackLength) {
         tendon_length = _tendonSlackLength;
         cos_factor = cos(atan(muscle_width / (_length - tendon_length)));
         _fiberLength = (_length - tendon_length) / cos_factor;
      }
   }

   return tendon_force;
}

void SimmDarrylMuscle::peteTest() const
{
	int i;

	cout << "Muscle: " << getName() << endl;
	for (i = 0; i < _attachmentSet.getSize(); i++)
		_attachmentSet.get(i)->peteTest();
	cout << "   groups: ";
	for (i = 0; i < _groupNames.getSize(); i++)
		cout << _groupNames[i] << " ";
	cout << endl;
#if 0
	cout << "   timeScale: " << _timeScale << endl;
	cout << "   activationTimeConstant: " << _activationTimeConstant << endl;
	cout << "   deactivationTimeConstant: " << _deactivationTimeConstant << endl;
	cout << "   maxIsometricForce: " << _maxIsometricForce << endl;
	cout << "   optimalFiberLength: " << _optimalFiberLength << endl;
	cout << "   tendonSlackLength: " << _tendonSlackLength << endl;
	cout << "   pennationAngle: " << _pennationAngle << endl;
	cout << "   maxContractionVelocityFullAct: " << _maxContractionVelocityFullAct << endl;
	cout << "   maxContractionVelocityLowAct: " << _maxContractionVelocityLowAct << endl;
	cout << "   current length: " << getLength() << endl;
#endif
}
