// Schutte1993Muscle.cpp
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//=============================================================================
// INCLUDES
//=============================================================================
#include "Schutte1993Muscle.h"
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Common/rdMath.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

const int Schutte1993Muscle::STATE_ACTIVATION = 0;
const int Schutte1993Muscle::STATE_FIBER_LENGTH = 1;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Schutte1993Muscle::Schutte1993Muscle() :
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
Schutte1993Muscle::~Schutte1993Muscle()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMuscle Schutte1993Muscle to be copied.
 */
Schutte1993Muscle::Schutte1993Muscle(const Schutte1993Muscle &aMuscle) :
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
	setup(aMuscle.getModel());
}

//_____________________________________________________________________________
/**
 * Copy this muscle point and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this Schutte1993Muscle.
 */
Object* Schutte1993Muscle::copy() const
{
	Schutte1993Muscle *musc = new Schutte1993Muscle(*this);
	return(musc);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one Schutte1993Muscle to another.
 *
 * @param aMuscle Schutte1993Muscle to be copied.
 */
void Schutte1993Muscle::copyData(const Schutte1993Muscle &aMuscle)
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
 * Set the data members of this Schutte1993Muscle to their null values.
 */
void Schutte1993Muscle::setNull()
{
	setType("Schutte1993Muscle");

	setNumControls(1); setNumStates(2); setNumPseudoStates(0);
	bindControl(0, _excitation, "excitation");
	bindState(STATE_ACTIVATION, _activation, "activation");
	bindState(STATE_FIBER_LENGTH, _fiberLength, "fiber_length");

	_excitation = 0.0;
	_activation = 0.0;
	_fiberLength = 0.0;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Schutte1993Muscle::setupProperties()
{
	_timeScaleProp.setName("time_scale");
	_timeScaleProp.setComment("Scale factor for normalizing time");
	_timeScaleProp.setValue(0.0);
	_propertySet.append(&_timeScaleProp, "Parameters");

	_activation1Prop.setName("activation1");
	_timeScaleProp.setComment("Parameter used in time constant of ramping up of muscle force");
	_activation1Prop.setValue(0.0);
	_propertySet.append(&_activation1Prop, "Parameters");

	_activation2Prop.setName("activation2");
	_timeScaleProp.setComment("Parameter used in time constant of ramping up and ramping down of muscle force");
	_activation2Prop.setValue(0.0);
	_propertySet.append(&_activation2Prop, "Parameters");

	_maxIsometricForceProp.setName("max_isometric_force");
	_timeScaleProp.setComment("Maximum isometric force that the fibers can generate");
	_maxIsometricForceProp.setValue(0.0);
	_propertySet.append(&_maxIsometricForceProp, "Parameters");

	_optimalFiberLengthProp.setName("optimal_fiber_length");
	_timeScaleProp.setComment("Optimal length of the muscle fibers");
	_optimalFiberLengthProp.setValue(0.0);
	_propertySet.append(&_optimalFiberLengthProp, "Parameters");

	_tendonSlackLengthProp.setName("tendon_slack_length");
	_timeScaleProp.setComment("Resting length of the tendon");
	_tendonSlackLengthProp.setValue(0.0);
	_propertySet.append(&_tendonSlackLengthProp, "Parameters");

	_pennationAngleProp.setName("pennation_angle");
	_timeScaleProp.setComment("Angle between tendon and fibers at optimal fiber length");
	_pennationAngleProp.setValue(0.0);
	_propertySet.append(&_pennationAngleProp, "Parameters");

	_maxContractionVelocityProp.setName("max_contraction_velocity");
	_timeScaleProp.setComment("Maximum contraction velocity of the fibers, in optimal fiberlengths per second");
	_maxContractionVelocityProp.setValue(0.0);
	_propertySet.append(&_maxContractionVelocityProp, "Parameters");

	_dampingProp.setName("damping");
	_timeScaleProp.setComment("Damping factor related to maximum contraction velocity");
	_dampingProp.setValue(0.05);

	_propertySet.append(&_dampingProp, "Parameters");

	_tendonForceLengthCurveProp.setName("tendon_force_length_curve");
	_timeScaleProp.setComment("Function representing force-length behavior of tendon");
	_propertySet.append(&_tendonForceLengthCurveProp, "Functions");

	_activeForceLengthCurveProp.setName("active_force_length_curve");
	_timeScaleProp.setComment("Function representing active force-length behavior of muscle fibers");
	_propertySet.append(&_activeForceLengthCurveProp, "Functions");

	_passiveForceLengthCurveProp.setName("passive_force_length_curve");
	_timeScaleProp.setComment("Function representing passive force-length behavior of muscle fibers");
	_propertySet.append(&_passiveForceLengthCurveProp, "Functions");

	_forceVelocityCurveProp.setName("force_velocity_curve");
	_timeScaleProp.setComment("Function representing force-velocity behavior of muscle fibers");
	_propertySet.append(&_forceVelocityCurveProp, "Functions");
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this Schutte1993Muscle.
 */
void Schutte1993Muscle::setup(Model* aModel)
{
	// Base class
	AbstractMuscle::setup(aModel);

	// aModel will be NULL when objects are being registered.
	if (aModel == NULL)
		return;

	if(!getActiveForceLengthCurve()) 
		throw Exception("Schutte1993Muscle.setup: ERROR- No active force length curve specified for muscle '"+getName()+"'",__FILE__,__LINE__);
	else if(!getPassiveForceLengthCurve())
		throw Exception("Schutte1993Muscle.setup: ERROR- No passive force length curve specified for muscle '"+getName()+"'",__FILE__,__LINE__);
	else if(!getTendonForceLengthCurve())
		throw Exception("Schutte1993Muscle.setup: ERROR- No tendon force length curve specified for muscle '"+getName()+"'",__FILE__,__LINE__);

	// Reasonable initial activation value
	_activation = 0.01;

	// Compute isometric force to get starting value
	// of _fiberLength.
	computeEquilibrium();
}

//_____________________________________________________________________________
/**
 * Copy the property values from another actuator, which may not be
 * a Schutte1993Muscle.
 *
 * @param aActuator Actuator to copy property values from.
 */
void Schutte1993Muscle::copyPropertyValues(AbstractActuator& aActuator)
{
	AbstractMuscle::copyPropertyValues(aActuator);

	const Property* prop = aActuator.getPropertySet().contains("time_scale");
	if (prop) _timeScaleProp.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("activation1");
	if (prop) _activation1Prop.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("activation2");
	if (prop) _activation2Prop.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("max_isometric_force");
	if (prop) _maxIsometricForceProp.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("optimal_fiber_length");
	if (prop) _optimalFiberLengthProp.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("tendon_slack_length");
	if (prop) _tendonSlackLengthProp.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("pennation_angle");
	if (prop) _pennationAngleProp.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("max_contraction_velocity");
	if (prop) _maxContractionVelocityProp.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("damping");
	if (prop) _dampingProp.setValue(prop->getValueDbl());

	Property* prop2 = aActuator.getPropertySet().contains("tendon_force_length_curve");
	if (prop2) {
	   Object* obj = prop2->getValueObjPtr();
		if (obj) _tendonForceLengthCurveProp.setValue(obj);
	}

	prop2 = aActuator.getPropertySet().contains("active_force_length_curve");
	if (prop2) {
	   Object* obj = prop2->getValueObjPtr();
		if (obj) _activeForceLengthCurveProp.setValue(obj);
	}

	prop2 = aActuator.getPropertySet().contains("passive_force_length_curve");
	if (prop2) {
	   Object* obj = prop2->getValueObjPtr();
		if (obj) _passiveForceLengthCurveProp.setValue(obj);
	}

	prop2 = aActuator.getPropertySet().contains("force_velocity_curve");
	if (prop2) {
	   Object* obj = prop2->getValueObjPtr();
		if (obj) _forceVelocityCurveProp.setValue(obj);
	}
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
Schutte1993Muscle& Schutte1993Muscle::operator=(const Schutte1993Muscle &aMuscle)
{
	// BASE CLASS
	AbstractMuscle::operator=(aMuscle);

	copyData(aMuscle);

	setup(aMuscle.getModel());

	return(*this);
}


//=============================================================================
// GET
//=============================================================================
//-----------------------------------------------------------------------------
// PENNATION ANGLE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the current pennation angle of the muscle fiber(s).
 *
 * @param Pennation angle.
 */
double Schutte1993Muscle::getPennationAngle()
{
	return calcPennation(_fiberLength,_optimalFiberLength,_pennationAngle);
}

//-----------------------------------------------------------------------------
// LENGTH
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the length of the muscle fiber(s).
 *
 * @param Current length of the muscle fiber(s).
 */
double Schutte1993Muscle::getFiberLength()
{
	return _fiberLength;
}
//_____________________________________________________________________________
/**
 * Get the normalized length of the muscle fiber(s).  This is the current
 * fiber length(s) divided by the optimal fiber length.
 *
 * @param Current length of the muscle fiber(s).
 */
double Schutte1993Muscle::getNormalizedFiberLength()
{
	return _fiberLength / getOptimalFiberLength();
}

//-----------------------------------------------------------------------------
// FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the passive force generated by the muscle fibers.
 *
 * @param Current active force of the muscle fiber(s).
 */
double Schutte1993Muscle::getPassiveFiberForce()
{
	return _passiveForce;
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
void Schutte1993Muscle::scale(const ScaleSet& aScaleSet)
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
void Schutte1993Muscle::postScale(const ScaleSet& aScaleSet)
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
void Schutte1993Muscle::computeStateDerivatives(double rDYDT[])
{
	if (!rDYDT)
		return;

	rDYDT[STATE_ACTIVATION] = _activationDeriv;
	rDYDT[STATE_FIBER_LENGTH] = _fiberLengthDeriv;
}

//_____________________________________________________________________________
/**
 * Compute the equilibrium states.  This method computes a fiber length
 * for the muscle that is consistent with the muscle's activation level.
 */
void Schutte1993Muscle::computeEquilibrium()
{
	double force = computeIsometricForce(_activation);

	//cout<<getName()<<": isometric force = "<<force<<endl;
	//cout<<getName()<<": fiber length = "<<_fiberLength<<endl;
}

//_____________________________________________________________________________
/**
 * Compute the actuation for the muscle. This function assumes
 * that computeDerivatives has already been called.
 */
void Schutte1993Muscle::computeActuation()
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
Function* Schutte1993Muscle::getActiveForceLengthCurve() const
{
	return _activeForceLengthCurve;
}

//_____________________________________________________________________________
/**
 * Get the passive force-length curve.
 *
 * @return Pointer to the passive force-length curve (Function).
 */
Function* Schutte1993Muscle::getPassiveForceLengthCurve() const
{
	return _passiveForceLengthCurve;
}

//_____________________________________________________________________________
/**
 * Get the tendon force-length curve.
 *
 * @return Pointer to the tendon force-length curve (Function).
 */
Function* Schutte1993Muscle::getTendonForceLengthCurve() const
{
	return _tendonForceLengthCurve;
}

//_____________________________________________________________________________
/**
 * Get the force-velocity curve.
 *
 * @return Pointer to the force-velocity curve (Function).
 */
Function* Schutte1993Muscle::getForceVelocityCurve() const
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
double Schutte1993Muscle::calcTendonForce(double aNormTendonLength) const
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
double Schutte1993Muscle::calcNonzeroPassiveForce(double aNormFiberLength, double aNormFiberVelocity) const
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
double Schutte1993Muscle::calcFiberVelocity(double aActivation, double aActiveForce, double aVelocityDependentForce) const
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
double Schutte1993Muscle::getStress() const
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
double Schutte1993Muscle::computeIsometricForce(double aActivation)
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

   calcLengthAfterPathComputation();

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

//_____________________________________________________________________________
/**
 * Find the force produced by muscle under isokinetic conditions assuming
 * an infinitely stiff tendon.  That is, all the shortening velocity of the
 * actuator (the musculotendon unit) is assumed to be due to the shortening
 * of the muscle fibers alone.  This methods calls
 * computeIsometricForce and so alters the internal member variables of this
 * muscle.
 *
 *
 * Note that the current implementation approximates the effect of the
 * force-velocity curve.  It does not account for the shortening velocity
 * when it is solving for the equilibrium length of the muscle fibers.  And,
 * a generic representation of the force-velocity curve is used (as opposed
 * to the implicit force-velocity curve assumed by this model.
 *
 *
 * @param aActivation Activation of the muscle.
 * @return Isokinetic force generated by the actuator.
 * @todo Reimplement this methods with more accurate representation of the
 * force-velocity curve.
 */
double Schutte1993Muscle::
computeIsokineticForceAssumingInfinitelyStiffTendon(double aActivation)
{
	double isometricForce = computeIsometricForce(aActivation);

	double normalizedLength = _fiberLength / _optimalFiberLength;
	double normalizedVelocity = cos(_pennationAngle) * _speed / (_maxContractionVelocity * _optimalFiberLength);
	double normalizedForceVelocity = evaluateForceLengthVelocityCurve(1.0,normalizedLength,normalizedVelocity);

	return isometricForce * normalizedForceVelocity;
}
