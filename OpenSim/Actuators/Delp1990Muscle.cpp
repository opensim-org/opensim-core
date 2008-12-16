// Delp1990Muscle.cpp
// Author: Peter Loan
/*
 * Copyright (c)  2008, Stanford University. All rights reserved. 
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
#include "Delp1990Muscle.h"
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Common/rdMath.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

const int Delp1990Muscle::STATE_ACTIVATION = 0;
const int Delp1990Muscle::STATE_FIBER_LENGTH = 1;
const int Delp1990Muscle::STATE_FIBER_VELOCITY = 2;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Delp1990Muscle::Delp1990Muscle() :
   AbstractMuscle(),
	_timeScale(_timeScaleProp.getValueDbl()),
	_activation1(_activation1Prop.getValueDbl()),
	_activation2(_activation2Prop.getValueDbl()),
	_mass(_massProp.getValueDbl()),
	_maxIsometricForce(_maxIsometricForceProp.getValueDbl()),
	_optimalFiberLength(_optimalFiberLengthProp.getValueDbl()),
	_tendonSlackLength(_tendonSlackLengthProp.getValueDbl()),
	_pennationAngle(_pennationAngleProp.getValueDbl()),
	_maxContractionVelocity(_maxContractionVelocityProp.getValueDbl()),
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
Delp1990Muscle::~Delp1990Muscle()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMuscle Delp1990Muscle to be copied.
 */
Delp1990Muscle::Delp1990Muscle(const Delp1990Muscle &aMuscle) :
   AbstractMuscle(aMuscle),
	_timeScale(_timeScaleProp.getValueDbl()),
	_activation1(_activation1Prop.getValueDbl()),
	_activation2(_activation2Prop.getValueDbl()),
	_mass(_massProp.getValueDbl()),
	_maxIsometricForce(_maxIsometricForceProp.getValueDbl()),
	_optimalFiberLength(_optimalFiberLengthProp.getValueDbl()),
	_tendonSlackLength(_tendonSlackLengthProp.getValueDbl()),
	_pennationAngle(_pennationAngleProp.getValueDbl()),
	_maxContractionVelocity(_maxContractionVelocityProp.getValueDbl()),
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
 * @return Pointer to a copy of this Delp1990Muscle.
 */
Object* Delp1990Muscle::copy() const
{
	Delp1990Muscle *musc = new Delp1990Muscle(*this);
	return(musc);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one Delp1990Muscle to another.
 *
 * @param aMuscle Delp1990Muscle to be copied.
 */
void Delp1990Muscle::copyData(const Delp1990Muscle &aMuscle)
{
	_timeScale = aMuscle._timeScale;
	_activation1 = aMuscle._activation1;
	_activation2 = aMuscle._activation2;
	_mass = aMuscle._mass;
	_maxIsometricForce = aMuscle._maxIsometricForce;
	_optimalFiberLength = aMuscle._optimalFiberLength;
	_tendonSlackLength = aMuscle._tendonSlackLength;
	_pennationAngle = aMuscle._pennationAngle;
	_maxContractionVelocity = aMuscle._maxContractionVelocity;
	_tendonForceLengthCurve = (Function*)Object::SafeCopy(aMuscle._tendonForceLengthCurve);
	_activeForceLengthCurve = (Function*)Object::SafeCopy(aMuscle._activeForceLengthCurve);
	_passiveForceLengthCurve = (Function*)Object::SafeCopy(aMuscle._passiveForceLengthCurve);
	_forceVelocityCurve = (Function*)Object::SafeCopy(aMuscle._forceVelocityCurve);
}

//_____________________________________________________________________________
/**
 * Set the data members of this Delp1990Muscle to their null values.
 */
void Delp1990Muscle::setNull()
{
	setType("Delp1990Muscle");

	setNumControls(1); setNumStates(3); setNumPseudoStates(0);
	bindControl(0, _excitation, "excitation");
	bindState(STATE_ACTIVATION, _activation, "activation");
	bindState(STATE_FIBER_LENGTH, _fiberLength, "fiber_length");
	bindState(STATE_FIBER_VELOCITY, _fiberVelocity, "fiber_velocity");

	_excitation = 0.0;
	_activation = 0.0;
	_fiberLength = 0.0;
	_fiberVelocity = 0.0;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Delp1990Muscle::setupProperties()
{
	_timeScaleProp.setName("time_scale");
	_timeScaleProp.setComment("Scale factor for normalizing time");
	_timeScaleProp.setValue(0.0);
	_propertySet.append(&_timeScaleProp, "Parameters");

	_activation1Prop.setName("activation1");
	_activation1Prop.setComment("Parameter used in time constant of ramping up of muscle force");
	_activation1Prop.setValue(0.0);
	_propertySet.append(&_activation1Prop, "Parameters");

	_activation2Prop.setName("activation2");
	_activation2Prop.setComment("Parameter used in time constant of ramping up and ramping down of muscle force");
	_activation2Prop.setValue(0.0);
	_propertySet.append(&_activation2Prop, "Parameters");

	_massProp.setName("mass");
	_massProp.setComment("Normalized mass of the muscle between the tendon and muscle fibers");
	_massProp.setValue(0.0);
	_propertySet.append(&_massProp, "Parameters");

	_maxIsometricForceProp.setName("max_isometric_force");
	_maxIsometricForceProp.setComment("Maximum isometric force that the fibers can generate");
	_maxIsometricForceProp.setValue(0.0);
	_propertySet.append(&_maxIsometricForceProp, "Parameters");

	_optimalFiberLengthProp.setName("optimal_fiber_length");
	_optimalFiberLengthProp.setComment("Optimal length of the muscle fibers");
	_optimalFiberLengthProp.setValue(0.0);
	_propertySet.append(&_optimalFiberLengthProp, "Parameters");

	_tendonSlackLengthProp.setName("tendon_slack_length");
	_tendonSlackLengthProp.setComment("Resting length of the tendon");
	_tendonSlackLengthProp.setValue(0.0);
	_propertySet.append(&_tendonSlackLengthProp, "Parameters");

	_pennationAngleProp.setName("pennation_angle");
	_pennationAngleProp.setComment("Angle between tendon and fibers at optimal fiber length");
	_pennationAngleProp.setValue(0.0);
	_propertySet.append(&_pennationAngleProp, "Parameters");

	_maxContractionVelocityProp.setName("max_contraction_velocity");
	_maxContractionVelocityProp.setComment("Maximum contraction velocity of the fibers, in optimal fiberlengths per second");
	_maxContractionVelocityProp.setValue(0.0);
	_propertySet.append(&_maxContractionVelocityProp, "Parameters");

	_tendonForceLengthCurveProp.setName("tendon_force_length_curve");
	_tendonForceLengthCurveProp.setComment("Function representing force-length behavior of tendon");
	_propertySet.append(&_tendonForceLengthCurveProp, "Functions");

	_activeForceLengthCurveProp.setName("active_force_length_curve");
	_activeForceLengthCurveProp.setComment("Function representing active force-length behavior of muscle fibers");
	_propertySet.append(&_activeForceLengthCurveProp, "Functions");

	_passiveForceLengthCurveProp.setName("passive_force_length_curve");
	_passiveForceLengthCurveProp.setComment("Function representing passive force-length behavior of muscle fibers");
	_propertySet.append(&_passiveForceLengthCurveProp, "Functions");

	_forceVelocityCurveProp.setName("force_velocity_curve");
	_forceVelocityCurveProp.setComment("Function representing force-velocity behavior of muscle fibers");
	_propertySet.append(&_forceVelocityCurveProp, "Functions");
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this Delp1990Muscle.
 */
void Delp1990Muscle::setup(Model* aModel)
{
	// Base class
	AbstractMuscle::setup(aModel);

	// aModel will be NULL when objects are being registered.
	if (aModel == NULL)
		return;

	if(!getActiveForceLengthCurve()) 
		throw Exception("Delp1990Muscle.setup: ERROR- No active force length curve specified for muscle '"+getName()+"'",__FILE__,__LINE__);
	else if(!getPassiveForceLengthCurve())
		throw Exception("Delp1990Muscle.setup: ERROR- No passive force length curve specified for muscle '"+getName()+"'",__FILE__,__LINE__);
	else if(!getTendonForceLengthCurve())
		throw Exception("Delp1990Muscle.setup: ERROR- No tendon force length curve specified for muscle '"+getName()+"'",__FILE__,__LINE__);
	else if(!getForceVelocityCurve())
		throw Exception("Delp1990Muscle.setup: ERROR- No force velocity curve specified for muscle '"+getName()+"'",__FILE__,__LINE__);

	// Reasonable initial activation value
	_activation = 0.01;

	// Compute isometric force to get starting value
	// of _fiberLength.
	computeEquilibrium();
}

//_____________________________________________________________________________
/**
 * Copy the property values from another actuator, which may not be
 * a Delp1990Muscle.
 *
 * @param aActuator Actuator to copy property values from.
 */
void Delp1990Muscle::copyPropertyValues(AbstractActuator& aActuator)
{
	AbstractMuscle::copyPropertyValues(aActuator);

	const Property* prop = aActuator.getPropertySet().contains("time_scale");
	if (prop) _timeScaleProp.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("activation1");
	if (prop) _activation1Prop.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("activation2");
	if (prop) _activation2Prop.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("mass");
	if (prop) _massProp.setValue(prop->getValueDbl());

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
Delp1990Muscle& Delp1990Muscle::operator=(const Delp1990Muscle &aMuscle)
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
double Delp1990Muscle::getPennationAngle()
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
double Delp1990Muscle::getFiberLength()
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
double Delp1990Muscle::getNormalizedFiberLength()
{
	return _fiberLength / getOptimalFiberLength();
}
//_____________________________________________________________________________
/**
 * Get the velocity of the muscle fiber(s).
 *
 * @param Current velocity of the muscle fiber(s).
 */
double Delp1990Muscle::getFiberVelocity()
{
	return _fiberVelocity;
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
double Delp1990Muscle::getPassiveFiberForce()
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
void Delp1990Muscle::scale(const ScaleSet& aScaleSet)
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
void Delp1990Muscle::postScale(const ScaleSet& aScaleSet)
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
void Delp1990Muscle::computeStateDerivatives(double rDYDT[])
{
	if (!rDYDT)
		return;

	rDYDT[STATE_ACTIVATION] = _activationDeriv;
	rDYDT[STATE_FIBER_LENGTH] = _fiberLengthDeriv;
	rDYDT[STATE_FIBER_VELOCITY] = _fiberVelocityDeriv;
}

//_____________________________________________________________________________
/**
 * Compute the equilibrium states.  This method computes a fiber length
 * for the muscle that is consistent with the muscle's activation level.
 */
void Delp1990Muscle::computeEquilibrium()
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
void Delp1990Muscle::computeActuation()
{
	// Base Class (to calculate speed)
	AbstractMuscle::computeActuation();

   double normState[3], normStateDeriv[3], norm_tendon_length, ca, ta;
   double norm_muscle_tendon_length, pennation_angle;

   /* Normalize the muscle states */
   normState[STATE_ACTIVATION] = _activation;
   normState[STATE_FIBER_LENGTH] = _fiberLength / _optimalFiberLength;
   normState[STATE_FIBER_VELOCITY] = _fiberVelocity * (_timeScale / _optimalFiberLength);

	/* Compute normalized muscle state derivatives */
   if (_excitation >= normState[STATE_ACTIVATION])
      normStateDeriv[STATE_ACTIVATION] = (_excitation - normState[STATE_ACTIVATION]) * (_activation1 * _excitation + _activation2);
   else
      normStateDeriv[STATE_ACTIVATION] = (_excitation - normState[STATE_ACTIVATION]) * _activation2;
   normStateDeriv[STATE_FIBER_LENGTH] = normState[STATE_FIBER_VELOCITY];

	pennation_angle = calcPennation(normState[STATE_FIBER_LENGTH], 1.0, _pennationAngle);
   ca = cos(pennation_angle);
   ta = tan(pennation_angle);
   norm_muscle_tendon_length = getLength() / _optimalFiberLength;
   norm_tendon_length = norm_muscle_tendon_length - normState[STATE_FIBER_LENGTH] * ca;
   _tendonForce = calcTendonForce(norm_tendon_length);
	double fiberForce = calcFiberForce(normState[STATE_ACTIVATION], normState[STATE_FIBER_LENGTH], normState[STATE_FIBER_VELOCITY]);
	double muscleMass = _mass * (_optimalFiberLength / _timeScale) * (_optimalFiberLength / _timeScale);
	double massTerm = (_tendonForce * ca - fiberForce * ca * ca) / muscleMass;
	double velocityTerm = normState[STATE_FIBER_VELOCITY] * normState[STATE_FIBER_VELOCITY] * ta * ta / normState[STATE_FIBER_LENGTH];
	normStateDeriv[STATE_FIBER_VELOCITY] = massTerm + velocityTerm;
   _passiveForce = getPassiveForceLengthCurve()->evaluate(0, normState[STATE_FIBER_LENGTH]);
	_activeForce = getActiveForceLengthCurve()->evaluate(0, normState[STATE_FIBER_LENGTH]);
	if (_activeForce < 0.0)
		_activeForce = 0.0;

   /* Un-normalize the muscle state derivatives and forces. */
   _activationDeriv = normStateDeriv[STATE_ACTIVATION] / _timeScale;
   _fiberLengthDeriv = normStateDeriv[STATE_FIBER_LENGTH] * _optimalFiberLength / _timeScale;
   _fiberVelocityDeriv = normStateDeriv[STATE_FIBER_VELOCITY] * _optimalFiberLength / (_timeScale * _timeScale);

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
Function* Delp1990Muscle::getActiveForceLengthCurve() const
{
	return _activeForceLengthCurve;
}

//_____________________________________________________________________________
/**
 * Get the passive force-length curve.
 *
 * @return Pointer to the passive force-length curve (Function).
 */
Function* Delp1990Muscle::getPassiveForceLengthCurve() const
{
	return _passiveForceLengthCurve;
}

//_____________________________________________________________________________
/**
 * Get the tendon force-length curve.
 *
 * @return Pointer to the tendon force-length curve (Function).
 */
Function* Delp1990Muscle::getTendonForceLengthCurve() const
{
	return _tendonForceLengthCurve;
}

//_____________________________________________________________________________
/**
 * Get the force-velocity curve.
 *
 * @return Pointer to the force-velocity curve (Function).
 */
Function* Delp1990Muscle::getForceVelocityCurve() const
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
double Delp1990Muscle::calcTendonForce(double aNormTendonLength) const
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
 * Calculate the force in the muscle fibers, which includes the effects of active
 * force, passive force, activation, and contraction velocity.
 *
 * @param aNormTendonLength Normalized length of the tendon.
 * @return The force in the tendon.
 */
double Delp1990Muscle::calcFiberForce(double aActivation, double aNormFiberLength, double aNormFiberVelocity) const
{
   double activeForce = getActiveForceLengthCurve()->evaluate(0, aNormFiberLength);
   double passiveForce = getPassiveForceLengthCurve()->evaluate(0, aNormFiberLength);
   double velocityFactor = getForceVelocityCurve()->evaluate(0, aNormFiberVelocity);

   return aActivation * activeForce * velocityFactor + passiveForce;
}

//_____________________________________________________________________________
/**
 * Compute stress
 */
double Delp1990Muscle::getStress() const
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
double Delp1990Muscle::computeIsometricForce(double aActivation)
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
double Delp1990Muscle::
computeIsokineticForceAssumingInfinitelyStiffTendon(double aActivation)
{
	double isometricForce = computeIsometricForce(aActivation);

	double normalizedLength = _fiberLength / _optimalFiberLength;
	double normalizedVelocity = cos(_pennationAngle) * _speed / (_maxContractionVelocity * _optimalFiberLength);
	double normalizedForceVelocity = evaluateForceLengthVelocityCurve(1.0,normalizedLength,normalizedVelocity);

	return isometricForce * normalizedForceVelocity;
}
