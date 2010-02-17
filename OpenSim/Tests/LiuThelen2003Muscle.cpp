// LiuThelen2003Muscle.cpp
// Authors: Peter Loan, Darryl Thelen
/*
 * Copyright (c)  2009, Stanford University. All rights reserved. 
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
#include "LiuThelen2003Muscle.h"
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Common/DebugUtilities.h>
#include <OpenSim/Simulation/Model/Model.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

const int LiuThelen2003Muscle::STATE_ACTIVATION = 0;
const int LiuThelen2003Muscle::STATE_FIBER_LENGTH = 1;
const int LiuThelen2003Muscle::STATE_ACTIVE_MOTOR_UNITS = 2;
const int LiuThelen2003Muscle::STATE_FATIGUED_MOTOR_UNITS = 3;

//=============================================================================
//=============================================================================
/**
 * A Thelen2003Muscle that includes two states for modeling fatigue and
 * recovery of muscle fibers. The equations for these states are based
 * on the following paper:
 * Liu, Jing Z., Brown, Robert, Yue, Guang H., "A Dynamical Model of Muscle
 * Activation, Fatigue, and Recovery," Biophysical Journal, Vol. 82, Issue 5,
 * pp. 2344-2359, 2002.
 *
 * @author Peter Loan (based on Thelen2003Muscle by Darryl Thelen)
 * @version 1.0
 */

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
LiuThelen2003Muscle::LiuThelen2003Muscle() :
   Thelen2003Muscle(),
	_fatigueFactor(_fatigueFactorProp.getValueDbl()),
	_recoveryFactor(_recoveryFactorProp.getValueDbl()),
   _defaultActiveMotorUnits(0.0),
   _defaultFatiguedMotorUnits(0.0)
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Constructor.
 */
LiuThelen2003Muscle::LiuThelen2003Muscle(const std::string &aName, double aMaxIsometricForce,
													  double aOptimalFiberLength, double aTendonSlackLength,
													  double aPennationAngle, double aFatigueFactor,
													  double aRecoveryFactor) :
   Thelen2003Muscle(aName, aMaxIsometricForce, aOptimalFiberLength, aTendonSlackLength, aPennationAngle),
	_fatigueFactor(_fatigueFactorProp.getValueDbl()),
	_recoveryFactor(_recoveryFactorProp.getValueDbl()),
   _defaultActiveMotorUnits(0.0),
   _defaultFatiguedMotorUnits(0.0)
{
	setNull();
	setupProperties();
	setFatigueFactor(aFatigueFactor);
	setRecoveryFactor(aRecoveryFactor);
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
LiuThelen2003Muscle::~LiuThelen2003Muscle()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMuscle LiuThelen2003Muscle to be copied.
 */
LiuThelen2003Muscle::LiuThelen2003Muscle(const LiuThelen2003Muscle &aMuscle) :
   Thelen2003Muscle(aMuscle),
	_fatigueFactor(_fatigueFactorProp.getValueDbl()),
	_recoveryFactor(_recoveryFactorProp.getValueDbl()),
   _defaultActiveMotorUnits(0.0),
   _defaultFatiguedMotorUnits(0.0)
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
 * @return Pointer to a copy of this LiuThelen2003Muscle.
 */
Object* LiuThelen2003Muscle::copy() const
{
	LiuThelen2003Muscle *musc = new LiuThelen2003Muscle(*this);
	return(musc);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one LiuThelen2003Muscle to another.
 *
 * @param aMuscle LiuThelen2003Muscle to be copied.
 */
void LiuThelen2003Muscle::copyData(const LiuThelen2003Muscle &aMuscle)
{
	_fatigueFactor = aMuscle._fatigueFactor;
	_recoveryFactor = aMuscle._recoveryFactor;
}

//_____________________________________________________________________________
/**
 * Set the data members of this LiuThelen2003Muscle to their null values.
 */
void LiuThelen2003Muscle::setNull()
{
	setType("LiuThelen2003Muscle");

	setNumStateVariables(4);

	_stateVariableSuffixes[STATE_ACTIVE_MOTOR_UNITS]="active_motor_units";
	_stateVariableSuffixes[STATE_FATIGUED_MOTOR_UNITS]="fatigued_motor_units";
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void LiuThelen2003Muscle::setupProperties()
{
	_fatigueFactorProp.setName("fatigue_factor");
	_fatigueFactorProp.setValue(0.0);
	_fatigueFactorProp.setComment("percentage of active motor units that fatigue in unit time");
	_propertySet.append(&_fatigueFactorProp, "Parameters");

	_recoveryFactorProp.setName("recovery_factor");
	_recoveryFactorProp.setValue(0.0);
	_recoveryFactorProp.setComment("percentage of fatigued motor units that recover in unit time");
	_propertySet.append(&_recoveryFactorProp, "Parameters");
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this LiuThelen2003Muscle.
 */
void LiuThelen2003Muscle::setup(Model& aModel)
{
	// Base class
	Thelen2003Muscle::setup(aModel);
}

void LiuThelen2003Muscle::equilibrate(SimTK::State& state) const
{
	Thelen2003Muscle::equilibrate(state);

	// Reasonable initial activation value
	setActiveMotorUnits(state, 0.0);
	setFatiguedMotorUnits(state, 0.0);
	_model->getSystem().realize(state, SimTK::Stage::Velocity);

	// Compute isometric force to get starting value
	// of _fiberLength.
	computeEquilibrium(state);
}

void LiuThelen2003Muscle::initStateCache(SimTK::State& s, SimTK::SubsystemIndex subsystemIndex, Model& model) 
{
	Thelen2003Muscle::initStateCache(s, subsystemIndex, model);
}

//_____________________________________________________________________________
/**
 * Copy the property values from another actuator, which may not be
 * a LiuThelen2003Muscle.
 *
 * @param aActuator Actuator to copy property values from.
 */
void LiuThelen2003Muscle::copyPropertyValues(Actuator& aActuator)
{
	Thelen2003Muscle::copyPropertyValues(aActuator);

	const Property* prop = aActuator.getPropertySet().contains("fatigue_factor");
	if (prop) _fatigueFactorProp.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("recovery_factor");
	if (prop) _recoveryFactorProp.setValue(prop->getValueDbl());
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
LiuThelen2003Muscle& LiuThelen2003Muscle::operator=(const LiuThelen2003Muscle &aMuscle)
{
	// BASE CLASS
	Muscle::operator=(aMuscle);

	copyData(aMuscle);

	setup(aMuscle.getModel());

	return(*this);
}



//=============================================================================
// GET
//=============================================================================

//-----------------------------------------------------------------------------
// MAXIMUM ISOMETRIC FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the maximum isometric force that the fibers can generate.
 *
 * @param aMaxIsometricForce The maximum isometric force that the fibers can generate.
 * @return Whether the maximum isometric force was successfully changed.
 */
bool LiuThelen2003Muscle::setFatigueFactor(double aFatigueFactor)
{
	_fatigueFactor = aFatigueFactor;
	return true;
}

//-----------------------------------------------------------------------------
// OPTIMAL FIBER LENGTH
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the optimal length of the muscle fibers.
 *
 * @param aOptimalFiberLength The optimal length of the muscle fibers.
 * @return Whether the optimal length was successfully changed.
 */
bool LiuThelen2003Muscle::setRecoveryFactor(double aRecoveryFactor)
{
	_recoveryFactor = aRecoveryFactor;
	return true;
}

double LiuThelen2003Muscle::getDefaultActiveMotorUnits() const
{
	return _defaultActiveMotorUnits;
}

void LiuThelen2003Muscle::setDefaultActiveMotorUnits(double activeMotorUnits) {
    _defaultActiveMotorUnits = activeMotorUnits;
}

double LiuThelen2003Muscle::getDefaultFatiguedMotorUnits() const
{
	return _defaultFatiguedMotorUnits;
}

void LiuThelen2003Muscle::setDefaultFatiguedMotorUnits(double fatiguedMotorUnits) {
    _defaultFatiguedMotorUnits = fatiguedMotorUnits;
}

//=============================================================================
// COMPUTATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the derivatives of the muscle states.
 *
 * @param s  system state 
 * @param index 
 */
void LiuThelen2003Muscle::computeStateDerivatives(const SimTK::State& s)
{
	Thelen2003Muscle::computeStateDerivatives(s);

	s.updZDot(_subsystemIndex)[_zIndex+STATE_ACTIVE_MOTOR_UNITS] = getActiveMotorUnitsDeriv(s);
	s.updZDot(_subsystemIndex)[_zIndex+STATE_FATIGUED_MOTOR_UNITS] = getFatiguedMotorUnitsDeriv(s);
}

//_____________________________________________________________________________
/**
 * Compute the equilibrium states.  This method computes a fiber length
 * for the muscle that is consistent with the muscle's activation level.
 */
void LiuThelen2003Muscle::computeEquilibrium(SimTK::State& s) const
{
	Thelen2003Muscle::computeEquilibrium(s);
}

//_____________________________________________________________________________
/**
 * Compute the actuation for the muscle. This function assumes
 * that computeDerivatives has already been called.
 *
 * This function is based on muscle_deriv_func9 from derivs.c (old pipeline code)
 */
double  LiuThelen2003Muscle::computeActuation(const SimTK::State& s) const
{
	double tendonForce;
   double normState[4], normStateDeriv[4], norm_tendon_length, ca;
   double norm_muscle_tendon_length, pennation_angle;

   // Normalize the muscle states.
   normState[STATE_ACTIVATION] = getActivation(s);
   normState[STATE_FIBER_LENGTH] = getFiberLength(s) / _optimalFiberLength;
	normState[STATE_ACTIVE_MOTOR_UNITS] = getActiveMotorUnits(s);
	normState[STATE_FATIGUED_MOTOR_UNITS] = getFatiguedMotorUnits(s);

	// Maximum contraction velocity is an activation scaled value.
	double Vmax = _vmax;
	if (normState[STATE_ACTIVATION]<1.0)
		Vmax = _vmax0 + normState[STATE_ACTIVATION]*(Vmax-_vmax0);
	Vmax = Vmax*_optimalFiberLength;

   // Compute normalized muscle state derivatives.
   if (getExcitation(s) >= normState[STATE_ACTIVATION])
      normStateDeriv[STATE_ACTIVATION] = (getExcitation(s) - normState[STATE_ACTIVATION]) / _activationTimeConstant;
   else
      normStateDeriv[STATE_ACTIVATION] = (getExcitation(s) - normState[STATE_ACTIVATION]) / _deactivationTimeConstant;

	pennation_angle = Muscle::calcPennation( normState[STATE_FIBER_LENGTH], 1.0, _pennationAngle);
   ca = cos(pennation_angle);

   norm_muscle_tendon_length = getLength(s) / _optimalFiberLength;
   norm_tendon_length = norm_muscle_tendon_length - normState[STATE_FIBER_LENGTH] * ca;

   tendonForce = calcTendonForce(s,norm_tendon_length);
   setPassiveForce(s, calcPassiveForce(s,normState[STATE_FIBER_LENGTH]) );
   setActiveForce(s, calcActiveForce(s,normState[STATE_FIBER_LENGTH]) );
	
   // If pennation equals 90 degrees, fiber length equals muscle width and fiber
   // velocity goes to zero.  Pennation will stay at 90 until tendon starts to
   // pull, then "stiff tendon" approximation is used to calculate approximate
   // fiber velocity.
   if (EQUAL_WITHIN_ERROR(ca, 0.0))
   {
      if (EQUAL_WITHIN_ERROR(tendonForce, 0.0))
      {
         normStateDeriv[STATE_FIBER_LENGTH] = 0.0;
		}
		else 
		{
         double h = norm_muscle_tendon_length - _tendonSlackLength;
         double w = _optimalFiberLength * sin(_pennationAngle);
         double new_fiber_length = sqrt(h*h + w*w) / _optimalFiberLength;
			double new_pennation_angle = Muscle::calcPennation( new_fiber_length, 1.0, _pennationAngle);
         double new_ca = cos(new_pennation_angle);
         normStateDeriv[STATE_FIBER_LENGTH] = getSpeed(s) / (Vmax * new_ca);
		}
	}
   else
   {
      double velocity_dependent_force = tendonForce / ca - getPassiveForce(s);
      normStateDeriv[STATE_FIBER_LENGTH] = calcFiberVelocity(s, normState[STATE_ACTIVATION]*getActiveMotorUnits(s), getActiveForce(s), velocity_dependent_force);
   }

	double unrecruitedMotorUnits = 1.0 - getActiveMotorUnits(s) - getFatiguedMotorUnits(s);
	normStateDeriv[STATE_ACTIVE_MOTOR_UNITS] = getActivation(s) * unrecruitedMotorUnits - getFatigueFactor() * getActiveMotorUnits(s) + getRecoveryFactor() * getFatiguedMotorUnits(s);
	normStateDeriv[STATE_FATIGUED_MOTOR_UNITS]  = getFatigueFactor() * getActiveMotorUnits(s) - getRecoveryFactor() * getFatiguedMotorUnits(s);

   // Un-normalize the muscle state derivatives and forces.
   // Note: Do not need to Un-Normalize activation dynamics equation since activation, deactivation parameters
   // specified in muscle file are now independent of time scale
   setActivationDeriv(s, normStateDeriv[STATE_ACTIVATION]) ;
   setFiberLengthDeriv(s, normStateDeriv[STATE_FIBER_LENGTH] * Vmax );
	setActiveMotorUnitsDeriv(s, normStateDeriv[STATE_ACTIVE_MOTOR_UNITS]);
	setFatiguedMotorUnitsDeriv(s, normStateDeriv[STATE_FATIGUED_MOTOR_UNITS]);

	tendonForce = tendonForce *  _maxIsometricForce;
	setTendonForce( s, tendonForce );
	setPassiveForce( s, getPassiveForce(s) * _maxIsometricForce);
	setActiveForce( s, getActiveForce(s)*getActivation(s) * _maxIsometricForce);

	return tendonForce;
}

//_____________________________________________________________________________
/**
 * Find the force produced by an actuator (the musculotendon unit), assuming
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
double LiuThelen2003Muscle::
computeIsometricForce(SimTK::State& s, double aActivation) const
{
#define MAX_ITERATIONS 100
#define ERROR_LIMIT 0.01

   int i;
   double length,tendon_length, fiber_force, tmp_fiber_length, min_tendon_stiffness;
   double cos_factor, fiber_stiffness;
   double old_fiber_length, length_change, tendon_stiffness, percent;
   double error_force = 0.0, old_error_force, tendon_force, norm_tendon_length;

   if (_optimalFiberLength < ROUNDOFF_ERROR) {
      return 0.0;
   }

	// This muscle model includes two fatigue states, so this function assumes
	// that t=infinity in order to compute the [steady-state] isometric force.
	// The fatigue effects are used to scale down the activation.
	if (_fatigueFactor > 0.0 || _recoveryFactor > 0.0)
		aActivation *= _recoveryFactor / (_fatigueFactor + _recoveryFactor);

   _path.calcLengthAfterPathComputation(s);
	length = getLength(s);

	// rough initial guess of fiber length
	setStateVariable(s, STATE_FIBER_LENGTH,  length - _tendonSlackLength);

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
      cos_factor = cos(atan(muscle_width / length));
      setStateVariable(s, STATE_FIBER_LENGTH,  length / cos_factor);
      _model->getSystem().realize(s, SimTK::Stage::Velocity);

		setActiveForce(s,  calcActiveForce(s, getFiberLength(s) / _optimalFiberLength) * aActivation );
		if (getActiveForce(s) < 0.0)
			setActiveForce(s, 0.0 );

		setPassiveForce(s,  calcPassiveForce(s, getFiberLength(s) / _optimalFiberLength) );
		if (getPassiveForce(s) < 0.0)
			setPassiveForce(s, 0.0);

      return (getActiveForce(s) + getPassiveForce(s)) * _maxIsometricForce * cos_factor;
   } else if (length < _tendonSlackLength) {
      setStateVariable(s, STATE_FIBER_LENGTH, muscle_width);
      _model->getSystem().realize(s, SimTK::Stage::Velocity);
      tendon_length = length;
      return 0.0;
   } else {
      setStateVariable(s, STATE_FIBER_LENGTH,  _optimalFiberLength);
      _model->getSystem().realize(s, SimTK::Stage::Velocity);
      cos_factor = cos(calcPennation( getFiberLength(s), _optimalFiberLength,  _pennationAngle ));  
      tendon_length = length - getFiberLength(s) * cos_factor;

      // Check to make sure tendon is not shorter than its slack length. If it
      // is, set the length to its slack length and re-compute fiber length.
      if (tendon_length < _tendonSlackLength) {
         tendon_length = _tendonSlackLength;
         cos_factor = cos(atan(muscle_width / (length - tendon_length)));
         setStateVariable(s, STATE_FIBER_LENGTH,  (length - tendon_length) / cos_factor);
         if (getFiberLength(s) < muscle_width)
           setStateVariable(s, STATE_FIBER_LENGTH,  muscle_width);
        _model->getSystem().realize(s, SimTK::Stage::Velocity);
      }
   }

   // Muscle-tendon force is found using an iterative method. First, you guess
   // the length of the muscle fibers and the length of the tendon, and
   // calculate their respective forces. If the forces match (are within
   // ERROR_LIMIT of each other), stop; else change the length guesses based
   // on the error and try again.
   for (i = 0; i < MAX_ITERATIONS; i++) {
		setActiveForce(s, calcActiveForce(s, getFiberLength(s)/ _optimalFiberLength) * aActivation );
      if (getActiveForce(s) <  0.0) 
         setActiveForce(s, 0.0);

		setPassiveForce(s, calcPassiveForce(s, getFiberLength(s) / _optimalFiberLength));
      if (getPassiveForce(s) < 0.0)
         setPassiveForce(s, 0.0);

      fiber_force = (getActiveForce(s) + getPassiveForce(s) ) * _maxIsometricForce * cos_factor;

      norm_tendon_length = tendon_length / _optimalFiberLength;
      tendon_force = calcTendonForce(s, norm_tendon_length) * _maxIsometricForce;

      old_error_force = error_force;
 
      error_force = tendon_force - fiber_force;

      if (DABS(error_force) <= ERROR_LIMIT) // muscle-tendon force found!
         break;

      if (i == 0)
         old_error_force = error_force;

      if (DSIGN(error_force) != DSIGN(old_error_force)) {
         percent = DABS(error_force) / (DABS(error_force) + DABS(old_error_force));
         tmp_fiber_length = old_fiber_length;
         old_fiber_length = getFiberLength(s);
         setStateVariable(s, STATE_FIBER_LENGTH, getFiberLength(s) + percent * (tmp_fiber_length - getFiberLength(s)) );
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

         min_tendon_stiffness = (getActiveForce(s) + getPassiveForce(s)) *
	         tendon_elastic_modulus * _maxIsometricForce /
	         (tendon_max_stress * _tendonSlackLength);

         if (tendon_stiffness < min_tendon_stiffness)
            tendon_stiffness = min_tendon_stiffness;

         fiber_stiffness = _maxIsometricForce / _optimalFiberLength *
            (calcActiveForce(s, getFiberLength(s) / _optimalFiberLength)  +
            calcPassiveForce(s, getFiberLength(s) / _optimalFiberLength));

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
         old_fiber_length = getFiberLength(s);

         if (error_force > 0.0)
             setStateVariable(s, STATE_FIBER_LENGTH,  getFiberLength(s) + length_change);
         else
             setStateVariable(s, STATE_FIBER_LENGTH,  getFiberLength(s) - length_change);

      }
      cos_factor = cos(calcPennation(getFiberLength(s), _optimalFiberLength, _pennationAngle ));
      tendon_length = length - getFiberLength(s) * cos_factor;

      // Check to make sure tendon is not shorter than its slack length. If it is,
      // set the length to its slack length and re-compute fiber length.
      if (tendon_length < _tendonSlackLength) {
         tendon_length = _tendonSlackLength;
         cos_factor = cos(atan(muscle_width / (length - tendon_length)));
         setStateVariable(s, STATE_FIBER_LENGTH,  (length - tendon_length) / cos_factor );
      }
   }

   _model->getSystem().realize(s, SimTK::Stage::Position);

   return tendon_force;
}

//_____________________________________________________________________________
/**
 * Find the force produced by muscle under isokinetic conditions assuming
 * an infinitely stiff tendon.  That is, all the shortening velocity of the
 * actuator (the musculotendon unit) is assumed to be due to the shortening
 * of the muscle fibers alone.  This methods calls
 * computeIsometricForce() and so alters the internal member variables of this
 * muscle.
 *
 * Note that the current implementation approximates the effect of the
 * force-velocity curve.  It does not account for the shortening velocity
 * when it is solving for the equilibrium length of the muscle fibers.  And,
 * a generic representation of the force-velocity curve is used (as opposed
 * to the implicit force-velocity curve assumed by this model.
 *
 * @param aActivation Activation of the muscle.
 * @return Isokinetic force generated by the actuator.
 * @todo Reimplement this methods with more accurate representation of the
 * force-velocity curve.
 */
double LiuThelen2003Muscle::
computeIsokineticForceAssumingInfinitelyStiffTendon(SimTK::State& s, double aActivation)
{
	double isometricForce = computeIsometricForce(s, aActivation);
	double normalizedLength = getFiberLength(s) / _optimalFiberLength;
	double normalizedVelocity = - getSpeed(s) / (_vmax * _optimalFiberLength);
	normalizedVelocity *= cos(_pennationAngle);
	double normalizedForceVelocity = evaluateForceLengthVelocityCurve(1.0,1.0,normalizedVelocity);
	
	return isometricForce * normalizedForceVelocity;
}
