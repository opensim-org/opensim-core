// SimmZajacHill.cpp
// Author: Peter Loan
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
#include "SimmZajacHill.h"
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Common/rdMath.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

const int SimmZajacHill::STATE_ACTIVATION = 0;
const int SimmZajacHill::STATE_FIBER_LENGTH = 1;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmZajacHill::SimmZajacHill() :
   AbstractMuscle(),
	_timeScale(_timeScaleProp.getValueDbl()),
	_activation1(_activation1Prop.getValueDbl()),
	_activation2(_activation2Prop.getValueDbl()),
	_maxIsometricForce(_maxIsometricForceProp.getValueDbl()),
	_optimalFiberLength(_optimalFiberLengthProp.getValueDbl()),
	_tendonSlackLength(_tendonSlackLengthProp.getValueDbl()),
	_pennationAngle(_pennationAngleProp.getValueDbl()),
	_maxContractionVelocity(_maxContractionVelocityProp.getValueDbl()),
	_damping(_dampingProp.getValueDbl()),
	_tendonForceLengthCurve(_tendonForceLengthCurveProp.getValueObjPtrRef()),
	_activeForceLengthCurve(_activeForceLengthCurveProp.getValueObjPtrRef()),
	_passiveForceLengthCurve(_passiveForceLengthCurveProp.getValueObjPtrRef()),
	_forceVelocityCurve(_forceVelocityCurveProp.getValueObjPtrRef())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmZajacHill::~SimmZajacHill()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMuscle SimmZajacHill to be copied.
 */
SimmZajacHill::SimmZajacHill(const SimmZajacHill &aMuscle) :
   AbstractMuscle(aMuscle),
	_timeScale(_timeScaleProp.getValueDbl()),
	_activation1(_activation1Prop.getValueDbl()),
	_activation2(_activation2Prop.getValueDbl()),
	_maxIsometricForce(_maxIsometricForceProp.getValueDbl()),
	_optimalFiberLength(_optimalFiberLengthProp.getValueDbl()),
	_tendonSlackLength(_tendonSlackLengthProp.getValueDbl()),
	_pennationAngle(_pennationAngleProp.getValueDbl()),
	_maxContractionVelocity(_maxContractionVelocityProp.getValueDbl()),
	_damping(_dampingProp.getValueDbl()),
	_tendonForceLengthCurve(_tendonForceLengthCurveProp.getValueObjPtrRef()),
	_activeForceLengthCurve(_activeForceLengthCurveProp.getValueObjPtrRef()),
	_passiveForceLengthCurve(_passiveForceLengthCurveProp.getValueObjPtrRef()),
	_forceVelocityCurve(_forceVelocityCurveProp.getValueObjPtrRef())
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
 * @return Pointer to a copy of this SimmZajacHill.
 */
Object* SimmZajacHill::copy() const
{
	SimmZajacHill *musc = new SimmZajacHill(*this);
	return(musc);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SimmZajacHill to another.
 *
 * @param aMuscle SimmZajacHill to be copied.
 */
void SimmZajacHill::copyData(const SimmZajacHill &aMuscle)
{
	_timeScale = aMuscle._timeScale;
	_activation1 = aMuscle._activation1;
	_activation2 = aMuscle._activation2;
	_maxIsometricForce = aMuscle._maxIsometricForce;
	_optimalFiberLength = aMuscle._optimalFiberLength;
	_tendonSlackLength = aMuscle._tendonSlackLength;
	_pennationAngle = aMuscle._pennationAngle;
	_maxContractionVelocity = aMuscle._maxContractionVelocity;
	_damping = aMuscle._damping;
	_tendonForceLengthCurve = (Function*)Object::SafeCopy(aMuscle._tendonForceLengthCurve);
	_activeForceLengthCurve = (Function*)Object::SafeCopy(aMuscle._activeForceLengthCurve);
	_passiveForceLengthCurve = (Function*)Object::SafeCopy(aMuscle._passiveForceLengthCurve);
	_forceVelocityCurve = (Function*)Object::SafeCopy(aMuscle._forceVelocityCurve);
}

//_____________________________________________________________________________
/**
 * Set the data members of this SimmZajacHill to their null values.
 */
void SimmZajacHill::setNull()
{
	setType("SimmZajacHill");

	setNumControls(1); setNumStates(2); setNumPseudoStates(0);
	bindControl(0, _excitation, "excitation");
	bindState(STATE_ACTIVATION, _activation, "activation");
	bindState(STATE_FIBER_LENGTH, _fiberLength, "fiber_length");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmZajacHill::setupProperties()
{
	PropertyGroup* forceGroup = _propertySet.addGroup("force");
	PropertyGroup* dynamicGroup = _propertySet.addGroup("dynamic");
	PropertyGroup* functionsGroup = _propertySet.addGroup("functions");
	PropertyGroup* extraGroup = _propertySet.addGroup("extra");

	_timeScaleProp.setName("time_scale");
	_timeScaleProp.setValue(0.0);
	_propertySet.append(&_timeScaleProp);
	_propertySet.addPropertyToGroup(dynamicGroup, &_timeScaleProp);

	_activation1Prop.setName("activation1");
	_activation1Prop.setValue(0.0);
	_propertySet.append(&_activation1Prop);
	_propertySet.addPropertyToGroup(extraGroup, &_activation1Prop);

	_activation2Prop.setName("activation2");
	_activation2Prop.setValue(0.0);
	_propertySet.append(&_activation2Prop);
	_propertySet.addPropertyToGroup(extraGroup, &_activation2Prop);

	_maxIsometricForceProp.setName("max_isometric_force");
	_maxIsometricForceProp.setValue(0.0);
	_propertySet.append(&_maxIsometricForceProp);
	_propertySet.addPropertyToGroup(forceGroup, &_maxIsometricForceProp);

	_optimalFiberLengthProp.setName("optimal_fiber_length");
	_optimalFiberLengthProp.setValue(0.0);
	_propertySet.append(&_optimalFiberLengthProp);
	_propertySet.addPropertyToGroup(forceGroup, &_optimalFiberLengthProp);

	_tendonSlackLengthProp.setName("tendon_slack_length");
	_tendonSlackLengthProp.setValue(0.0);
	_propertySet.append(&_tendonSlackLengthProp);
	_propertySet.addPropertyToGroup(forceGroup, &_tendonSlackLengthProp);

	_pennationAngleProp.setName("pennation_angle");
	_pennationAngleProp.setValue(0.0);
	_propertySet.append(&_pennationAngleProp);
	_propertySet.addPropertyToGroup(forceGroup, &_pennationAngleProp);

	_maxContractionVelocityProp.setName("max_contraction_velocity");
	_maxContractionVelocityProp.setValue(0.0);
	_propertySet.append(&_maxContractionVelocityProp);
	_propertySet.addPropertyToGroup(dynamicGroup, &_maxContractionVelocityProp);

	_dampingProp.setName("damping");
	_dampingProp.setValue(0.0);
	_propertySet.append(&_dampingProp);
	_propertySet.addPropertyToGroup(dynamicGroup, &_dampingProp);

	_tendonForceLengthCurveProp.setName("tendon_force_length_curve");
	_propertySet.append(&_tendonForceLengthCurveProp);
	_propertySet.addPropertyToGroup(functionsGroup, &_tendonForceLengthCurveProp);

	_activeForceLengthCurveProp.setName("active_force_length_curve");
	_propertySet.append(&_activeForceLengthCurveProp);
	_propertySet.addPropertyToGroup(functionsGroup, &_activeForceLengthCurveProp);

	_passiveForceLengthCurveProp.setName("passive_force_length_curve");
	_propertySet.append(&_passiveForceLengthCurveProp);
	_propertySet.addPropertyToGroup(functionsGroup, &_passiveForceLengthCurveProp);

	_forceVelocityCurveProp.setName("force_velocity_curve");
	_propertySet.append(&_forceVelocityCurveProp);
	_propertySet.addPropertyToGroup(functionsGroup, &_forceVelocityCurveProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this SimmZajacHill.
 */
void SimmZajacHill::setup(Model* aModel)
{
	// Base class
	AbstractMuscle::setup(aModel);

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
SimmZajacHill& SimmZajacHill::operator=(const SimmZajacHill &aMuscle)
{
	// BASE CLASS
	AbstractMuscle::operator=(aMuscle);

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
void SimmZajacHill::registerTypes()
{
	// Base class
	AbstractMuscle::registerTypes();
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
void SimmZajacHill::scale(const ScaleSet& aScaleSet)
{
	AbstractMuscle::scale(aScaleSet);

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
void SimmZajacHill::postScale(const ScaleSet& aScaleSet)
{
	AbstractMuscle::postScale(aScaleSet);

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
void SimmZajacHill::computeStateDerivatives(double rDYDT[])
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
 */
void SimmZajacHill::computeActuation()
{
	// Base Class (to calculate speed)
	AbstractMuscle::computeActuation();

   double normState[2], normStateDeriv[2], norm_tendon_length, ca;
   double norm_muscle_tendon_length, pennation_angle;

   /* Normalize the muscle states */
   normState[STATE_ACTIVATION] = _activation;
   normState[STATE_FIBER_LENGTH] = _fiberLength / _optimalFiberLength;

   /* Compute normalized muscle state derivatives */
   if (_excitation >= normState[STATE_ACTIVATION])
      normStateDeriv[STATE_ACTIVATION] = (_excitation - normState[STATE_ACTIVATION]) * (_activation1 * _excitation + _activation2);
   else
      normStateDeriv[STATE_ACTIVATION] = (_excitation - normState[STATE_ACTIVATION]) * _activation2;

	pennation_angle = calcPennation(normState[STATE_FIBER_LENGTH], 1.0, _pennationAngle);
   ca = cos(pennation_angle);
   norm_muscle_tendon_length = getLength() / _optimalFiberLength;
   norm_tendon_length = norm_muscle_tendon_length - normState[STATE_FIBER_LENGTH] * ca;
   _tendonForce = calcTendonForce(norm_tendon_length);
   _passiveForce = calcNonzeroPassiveForce(normState[STATE_FIBER_LENGTH], 0.0);
	_activeForce = getActiveForceLengthCurve()->evaluate(0, normState[STATE_FIBER_LENGTH]);
	if (_activeForce < 0.0)
		_activeForce = 0.0;

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
      }
      else
      {
         double h = norm_muscle_tendon_length - _tendonSlackLength;
         double w = _optimalFiberLength * sin(_pennationAngle);
         double new_fiber_length = sqrt(h*h + w*w) / _optimalFiberLength;
         double new_pennation_angle = calcPennation(new_fiber_length, 1.0, _pennationAngle);
         double new_ca = cos(new_pennation_angle);
         normStateDeriv[STATE_FIBER_LENGTH] = getSpeed() * _timeScale / _optimalFiberLength * new_ca;
      }
   }
   else
   {
      double velocity_dependent_force = _tendonForce / ca - _passiveForce;
      normStateDeriv[STATE_FIBER_LENGTH] = calcFiberVelocity(normState[STATE_ACTIVATION], _activeForce, velocity_dependent_force);
   }

   /* Un-normalize the muscle state derivatives and forces. */
   _activationDeriv = normStateDeriv[STATE_ACTIVATION] / _timeScale;
   _fiberLengthDeriv = normStateDeriv[STATE_FIBER_LENGTH] * _optimalFiberLength / _timeScale;

	_tendonForce *= _maxIsometricForce;
	_passiveForce *= _maxIsometricForce;
	_activeForce *= _maxIsometricForce;

	setForce(_tendonForce);
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
Function* SimmZajacHill::getActiveForceLengthCurve() const
{
	return _activeForceLengthCurve;
}

//_____________________________________________________________________________
/**
 * Get the passive force-length curve.
 *
 * @return Pointer to the passive force-length curve (Function).
 */
Function* SimmZajacHill::getPassiveForceLengthCurve() const
{
	return _passiveForceLengthCurve;
}

//_____________________________________________________________________________
/**
 * Get the tendon force-length curve.
 *
 * @return Pointer to the tendon force-length curve (Function).
 */
Function* SimmZajacHill::getTendonForceLengthCurve() const
{
	return _tendonForceLengthCurve;
}

//_____________________________________________________________________________
/**
 * Get the force-velocity curve.
 *
 * @return Pointer to the force-velocity curve (Function).
 */
Function* SimmZajacHill::getForceVelocityCurve() const
{
	return _forceVelocityCurve;
}

//_____________________________________________________________________________
/**
 * Calculate the force in tendon by finding tendon strain
 * and using it to interpolate the tendon force-length curve.
 *
 * @param aNormTendonLength Normalized length of the tendon.
 * @return The force in the tendon.
 */
double SimmZajacHill::calcTendonForce(double aNormTendonLength) const
{
   double tendon_force;
   double norm_resting_length = _tendonSlackLength / _optimalFiberLength;
   double tendon_strain =  (aNormTendonLength - norm_resting_length) / norm_resting_length;

   if (tendon_strain < 0.0)
      tendon_force = 0.0;
   else
      tendon_force = getTendonForceLengthCurve()->evaluate(0, tendon_strain);

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
double SimmZajacHill::calcNonzeroPassiveForce(double aNormFiberLength, double aNormFiberVelocity) const
{
   double flcomponent = exp(8.0*(aNormFiberLength - 1.0)) / exp(4.0);

   return flcomponent + _damping * aNormFiberVelocity;
}

//_____________________________________________________________________________
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
double SimmZajacHill::calcFiberVelocity(double aActivation, double aActiveForce, double aVelocityDependentForce) const
{
   double b, c, fiber_velocity;
   double kv = 0.15, slope_k = 0.13, fmax = 1.4;

   if (aVelocityDependentForce < -_damping)
	{
      fiber_velocity = aVelocityDependentForce / _damping;
	}
   else if (aVelocityDependentForce < aActivation * aActiveForce)
   {
      c = kv * (aVelocityDependentForce - aActivation * aActiveForce) / _damping;
      b = -kv * (aVelocityDependentForce / kv + aActivation * aActiveForce +
			_damping) / _damping;
      fiber_velocity = (-b - sqrt(b * b - 4 * c)) / 2.0;
   }
   else
   {
      c = -(slope_k * kv / ((_damping * (kv + 1)))) *
	      (aVelocityDependentForce - aActivation * aActiveForce);
      b = -(aVelocityDependentForce / _damping
			-fmax * aActivation * aActiveForce / _damping - slope_k * kv / (kv + 1));
      fiber_velocity = (-b + sqrt(b * b - 4 * c)) / 2.0;
   }

   return fiber_velocity;
}
//_____________________________________________________________________________
/**
 * Compute stress
 */
double SimmZajacHill::getStress() const
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
double SimmZajacHill::computeIsometricForce(double aActivation)
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

		_activeForce = getActiveForceLengthCurve()->evaluate(0, _fiberLength / _optimalFiberLength) * aActivation;
      if (_activeForce < 0.0)
         _activeForce = 0.0;

		_passiveForce = getPassiveForceLengthCurve()->evaluate(0, _fiberLength / _optimalFiberLength);
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
		_activeForce = getActiveForceLengthCurve()->evaluate(0, _fiberLength / _optimalFiberLength) * aActivation;
      if (_activeForce < 0.0)
         _activeForce = 0.0;

		_passiveForce = getPassiveForceLengthCurve()->evaluate(0, _fiberLength / _optimalFiberLength);
      if (_passiveForce < 0.0)
         _passiveForce = 0.0;

      fiber_force = (_activeForce + _passiveForce) * _maxIsometricForce * cos_factor;

      tendon_strain = (tendon_length / _tendonSlackLength - 1.0);
      if (tendon_strain < 0.0)
         tendon_force = 0.0;
      else
         tendon_force = getTendonForceLengthCurve()->evaluate(0, tendon_strain) * _maxIsometricForce;

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

         tendon_stiffness = getTendonForceLengthCurve()->evaluate(0, tendon_strain) *
				_maxIsometricForce / _tendonSlackLength;

         min_tendon_stiffness = (_activeForce + _passiveForce) *
	         tendon_elastic_modulus * _maxIsometricForce /
	         (tendon_max_stress * _tendonSlackLength);

         if (tendon_stiffness < min_tendon_stiffness)
            tendon_stiffness = min_tendon_stiffness;

         fiber_stiffness = _maxIsometricForce / _optimalFiberLength *
            (getActiveForceLengthCurve()->evaluate(0, _fiberLength / _optimalFiberLength)  +
            getPassiveForceLengthCurve()->evaluate(0, _fiberLength / _optimalFiberLength));

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

void SimmZajacHill::peteTest() const
{
	AbstractMuscle::peteTest();

	cout << "   timeScale: " << _timeScale << endl;
	cout << "   activation1: " << _activation1 << endl;
	cout << "   activation2: " << _activation2 << endl;
	cout << "   maxIsometricForce: " << _maxIsometricForce << endl;
	cout << "   optimalFiberLength: " << _optimalFiberLength << endl;
	cout << "   tendonSlackLength: " << _tendonSlackLength << endl;
	cout << "   pennationAngle: " << _pennationAngle << endl;
	cout << "   maxContractionVelocity: " << _maxContractionVelocity << endl;
	cout << "   damping: " << _damping << endl;
	if (_tendonForceLengthCurve) cout << "   tendonForceLengthCurve: " << *_tendonForceLengthCurve << endl;
	if (_activeForceLengthCurve) cout << "   activeForceLengthCurve: " << *_activeForceLengthCurve << endl;
	if (_passiveForceLengthCurve) cout << "   passiveForceLengthCurve: " << *_passiveForceLengthCurve << endl;
	if (_forceVelocityCurve) cout << "   forceVelocityCurve: " << *_forceVelocityCurve << endl;
	cout << "   current length: " << _length << endl;
}
