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
#include "SimmMacros.h"
#include <OpenSim/Tools/rdMath.h>

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
   AbstractSimmMuscle(),
	_timeScale(_timeScaleProp.getValueDbl()),
	_activation1(_activation1Prop.getValueDbl()),
	_activation2(_activation2Prop.getValueDbl()),
	_maxIsometricForce(_maxIsometricForceProp.getValueDbl()),
	_optimalFiberLength(_optimalFiberLengthProp.getValueDbl()),
	_tendonSlackLength(_tendonSlackLengthProp.getValueDbl()),
	_pennationAngle(_pennationAngleProp.getValueDbl()),
	_maxContractionVelocity(_maxContractionVelocityProp.getValueDbl()),
	_damping(_dampingProp.getValueDbl()),
	_tendonForceLengthCurve((ArrayPtrs<Function>&)_tendonForceLengthCurveProp.getValueObjArray()),
	_activeForceLengthCurve((ArrayPtrs<Function>&)_activeForceLengthCurveProp.getValueObjArray()),
	_passiveForceLengthCurve((ArrayPtrs<Function>&)_passiveForceLengthCurveProp.getValueObjArray()),
	_forceVelocityCurve((ArrayPtrs<Function>&)_forceVelocityCurveProp.getValueObjArray())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmZajacHill::SimmZajacHill(DOMElement *aElement) :
   AbstractSimmMuscle(aElement),
	_timeScale(_timeScaleProp.getValueDbl()),
	_activation1(_activation1Prop.getValueDbl()),
	_activation2(_activation2Prop.getValueDbl()),
	_maxIsometricForce(_maxIsometricForceProp.getValueDbl()),
	_optimalFiberLength(_optimalFiberLengthProp.getValueDbl()),
	_tendonSlackLength(_tendonSlackLengthProp.getValueDbl()),
	_pennationAngle(_pennationAngleProp.getValueDbl()),
	_maxContractionVelocity(_maxContractionVelocityProp.getValueDbl()),
	_damping(_dampingProp.getValueDbl()),
	_tendonForceLengthCurve((ArrayPtrs<Function>&)_tendonForceLengthCurveProp.getValueObjArray()),
	_activeForceLengthCurve((ArrayPtrs<Function>&)_activeForceLengthCurveProp.getValueObjArray()),
	_passiveForceLengthCurve((ArrayPtrs<Function>&)_passiveForceLengthCurveProp.getValueObjArray()),
	_forceVelocityCurve((ArrayPtrs<Function>&)_forceVelocityCurveProp.getValueObjArray())
{
	setNull();
	setupProperties();
	updateFromXMLNode();
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
   AbstractSimmMuscle(aMuscle),
	_timeScale(_timeScaleProp.getValueDbl()),
	_activation1(_activation1Prop.getValueDbl()),
	_activation2(_activation2Prop.getValueDbl()),
	_maxIsometricForce(_maxIsometricForceProp.getValueDbl()),
	_optimalFiberLength(_optimalFiberLengthProp.getValueDbl()),
	_tendonSlackLength(_tendonSlackLengthProp.getValueDbl()),
	_pennationAngle(_pennationAngleProp.getValueDbl()),
	_maxContractionVelocity(_maxContractionVelocityProp.getValueDbl()),
	_damping(_dampingProp.getValueDbl()),
	_tendonForceLengthCurve((ArrayPtrs<Function>&)_tendonForceLengthCurveProp.getValueObjArray()),
	_activeForceLengthCurve((ArrayPtrs<Function>&)_activeForceLengthCurveProp.getValueObjArray()),
	_passiveForceLengthCurve((ArrayPtrs<Function>&)_passiveForceLengthCurveProp.getValueObjArray()),
	_forceVelocityCurve((ArrayPtrs<Function>&)_forceVelocityCurveProp.getValueObjArray())
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

//_____________________________________________________________________________
/**
 * Copy this SimmZajacHill and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmZajacHill::SimmZajacHill(DOMElement*) in order to establish the
 * relationship of the SimmZajacHill object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmZajacHill object. Finally, the data members of the copy are
 * updated using SimmZajacHill::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmZajacHill.
 */
Object* SimmZajacHill::copy(DOMElement *aElement) const
{
	SimmZajacHill *pt = new SimmZajacHill(aElement);
	*pt = *this;
	pt->updateFromXMLNode();
	return(pt);
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
	_tendonForceLengthCurve = aMuscle._tendonForceLengthCurve;
	_activeForceLengthCurve = aMuscle._activeForceLengthCurve;
	_passiveForceLengthCurve = aMuscle._passiveForceLengthCurve;
	_forceVelocityCurve = aMuscle._forceVelocityCurve;
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
	_timeScaleProp.setName("time_scale");
	_timeScaleProp.setValue(0.0);
	_propertySet.append(&_timeScaleProp);

	_activation1Prop.setName("activation1");
	_activation1Prop.setValue(0.0);
	_propertySet.append(&_activation1Prop);

	_activation2Prop.setName("activation2");
	_activation2Prop.setValue(0.0);
	_propertySet.append(&_activation2Prop);

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

	_maxContractionVelocityProp.setName("max_contraction_velocity");
	_maxContractionVelocityProp.setValue(0.0);
	_propertySet.append(&_maxContractionVelocityProp);

	_dampingProp.setName("damping");
	_dampingProp.setValue(0.0);
	_propertySet.append(&_dampingProp);

	ArrayPtrs<Object> func;

	_tendonForceLengthCurveProp.setName("tendon_force_length_curve");
	_tendonForceLengthCurveProp.setValue(func);
	_propertySet.append(&_tendonForceLengthCurveProp);

	_activeForceLengthCurveProp.setName("active_force_length_curve");
	_activeForceLengthCurveProp.setValue(func);
	_propertySet.append(&_activeForceLengthCurveProp);

	_passiveForceLengthCurveProp.setName("passive_force_length_curve");
	_passiveForceLengthCurveProp.setValue(func);
	_propertySet.append(&_passiveForceLengthCurveProp);

	_forceVelocityCurveProp.setName("force_velocity_curve");
	_forceVelocityCurveProp.setValue(func);
	_propertySet.append(&_forceVelocityCurveProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this SimmZajacHill.
 */
void SimmZajacHill::setup(AbstractModel* aModel)
{
	// Base class
	AbstractSimmMuscle::setup(aModel);

	// Reasonable initial state values
	_activation = 0.01;
	_fiberLength = 1.4*_optimalFiberLength;
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
void SimmZajacHill::registerTypes()
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
void SimmZajacHill::scale(const ScaleSet& aScaleSet)
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
void SimmZajacHill::postScale(const ScaleSet& aScaleSet)
{
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
	AbstractSimmMuscle::computeActuation();

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

	pennation_angle = AbstractSimmMuscle::calcPennation(normState[STATE_FIBER_LENGTH], 1.0, _pennationAngle);
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
         double new_pennation_angle = AbstractSimmMuscle::calcPennation(new_fiber_length, 1.0, _pennationAngle);
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
	if (_activeForceLengthCurve.getSize() > 0)
		return _activeForceLengthCurve.get(0);

	return NULL;
}

//_____________________________________________________________________________
/**
 * Get the passive force-length curve.
 *
 * @return Pointer to the passive force-length curve (Function).
 */
Function* SimmZajacHill::getPassiveForceLengthCurve() const
{
	if (_passiveForceLengthCurve.getSize() > 0)
		return _passiveForceLengthCurve.get(0);

	return NULL;
}

//_____________________________________________________________________________
/**
 * Get the tendon force-length curve.
 *
 * @return Pointer to the tendon force-length curve (Function).
 */
Function* SimmZajacHill::getTendonForceLengthCurve() const
{
	if (_tendonForceLengthCurve.getSize() > 0)
		return _tendonForceLengthCurve.get(0);

	return NULL;
}

//_____________________________________________________________________________
/**
 * Get the force-velocity curve.
 *
 * @return Pointer to the force-velocity curve (Function).
 */
Function* SimmZajacHill::getForceVelocityCurve() const
{
	if (_forceVelocityCurve.getSize() > 0)
		return _forceVelocityCurve.get(0);

	return NULL;
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

void SimmZajacHill::peteTest() const
{
	AbstractSimmMuscle::peteTest();

	cout << "   timeScale: " << _timeScale << endl;
	cout << "   activation1: " << _activation1 << endl;
	cout << "   activation2: " << _activation2 << endl;
	cout << "   maxIsometricForce: " << _maxIsometricForce << endl;
	cout << "   optimalFiberLength: " << _optimalFiberLength << endl;
	cout << "   tendonSlackLength: " << _tendonSlackLength << endl;
	cout << "   pennationAngle: " << _pennationAngle << endl;
	cout << "   maxContractionVelocity: " << _maxContractionVelocity << endl;
	cout << "   damping: " << _damping << endl;
	if (_tendonForceLengthCurve.getSize() > 0)
		cout << "   tendonForceLengthCurve: " << *(_tendonForceLengthCurve[0]) << endl;
	if (_activeForceLengthCurve.getSize() > 0)
		cout << "   activeForceLengthCurve: " << *(_activeForceLengthCurve[0]) << endl;
	if (_passiveForceLengthCurve.getSize() > 0)
		cout << "   passiveForceLengthCurve: " << *(_passiveForceLengthCurve[0]) << endl;
	if (_forceVelocityCurve.getSize() > 0)
		cout << "   forceVelocityCurve: " << *(_forceVelocityCurve[0]) << endl;
	cout << "   current length: " << _length << endl;
}
