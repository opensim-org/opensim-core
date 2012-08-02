/* -------------------------------------------------------------------------- *
 *                     OpenSim:  LiuThelen2003Muscle.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan, Darryl G. Thelen                                    *
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
#include "LiuThelen2003Muscle.h"
#include <OpenSim/Common/SimmMacros.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

// States 0 and 1 are defined in the base class, Thelen2003Muscle.
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
LiuThelen2003Muscle::LiuThelen2003Muscle()
{
	setNull();
	constructProperties();
}

//_____________________________________________________________________________
/**
 * Constructor.
 */
LiuThelen2003Muscle::LiuThelen2003Muscle
   (const std::string &aName, double aMaxIsometricForce,
	double aOptimalFiberLength, double aTendonSlackLength,
	double aPennationAngle, double aFatigueFactor,
	double aRecoveryFactor) 
:   Super(aName, aMaxIsometricForce, aOptimalFiberLength, aTendonSlackLength, 
          aPennationAngle)
{
	setNull();
	constructProperties();
	setFatigueFactor(aFatigueFactor);
	setRecoveryFactor(aRecoveryFactor);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this LiuThelen2003Muscle to their null values.
 */
void LiuThelen2003Muscle::setNull()
{
   _defaultActiveMotorUnits = 0.0;
   _defaultFatiguedMotorUnits = 0.0;
}

//_____________________________________________________________________________
/**
 * Construct and initialize properties.
 *
 * Properties should be given a meaningful name and an informative comment.
 * The name you give each property is the tag that will be used in the XML
 * file. The comment will appear before the property in the XML file.
 * In addition, the comments are used for tool tips in the OpenSim GUI.
 *
 * All properties are added to the property set. Once added, they can be
 * read in and written to files.
*/
void LiuThelen2003Muscle::constructProperties()
{
    constructProperty_fatigue_factor(0.0);
    constructProperty_recovery_factor(0.0);
}

//_____________________________________________________________________________
void LiuThelen2003Muscle::addToSystem(SimTK::MultibodySystem& system) const
{
	Super::addToSystem(system);

	addStateVariable("active_motor_units");
	addStateVariable("fatigued_motor_units");
	addCacheVariable("active_motor_units_deriv", 0.0, SimTK::Stage::Dynamics);
	addCacheVariable("fatigued_motor_units_deriv", 0.0, SimTK::Stage::Dynamics);
}

void LiuThelen2003Muscle::equilibrate(SimTK::State& state) const
{
	// Reasonable initial activation value
	setActivation(state, 0.01);
	setFiberLength(state, getOptimalFiberLength());
	setActiveMotorUnits(state, 0.0);
	setFatiguedMotorUnits(state, 0.0);
	_model->getMultibodySystem().realize(state, SimTK::Stage::Velocity);

	// Compute isometric force to get starting value of _fiberLength.
	computeEquilibrium(state);
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
	set_fatigue_factor(aFatigueFactor);
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
	set_recovery_factor(aRecoveryFactor);
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
 */
SimTK::Vector LiuThelen2003Muscle::
computeStateVariableDerivatives(const SimTK::State& s) const
{
	SimTK::Vector derivs = Super::computeStateVariableDerivatives(s);
	derivs.resizeKeep(4);
    if (!isDisabled(s)) {
	    derivs[2] = getActiveMotorUnitsDeriv(s);
	    derivs[3] = getFatiguedMotorUnitsDeriv(s);
    } else
        derivs[2] = derivs[3] = 0;
	return derivs;
}

//_____________________________________________________________________________
/**
 * Compute the equilibrium states.  This method computes a fiber length
 * for the muscle that is consistent with the muscle's activation level.
 */
void LiuThelen2003Muscle::computeEquilibrium(SimTK::State& s) const
{
	computeIsometricForce(s, getActivation(s));
}

//_____________________________________________________________________________
/**
 * Compute the actuation for the muscle. This function assumes
 * that computeDerivatives has already been called.
 */
double LiuThelen2003Muscle::computeActuation(const SimTK::State& s) const
{
	double tendonForce;
	double normState[4], normStateDeriv[4], norm_tendon_length, ca;
	double norm_muscle_tendon_length, pennation_angle;

	const double maxIsometricForce = get_max_isometric_force();
    const double optimalFiberLength = get_optimal_fiber_length();
	const double tendonSlackLength = get_tendon_slack_length();
	const double pennationAngleAtOptimal = get_pennation_angle_at_optimal();
	const double activationTimeConstant = get_activation_time_constant();
	const double deactivationTimeConstant = get_deactivation_time_constant();
	const double vmax = get_Vmax();
	const double vmax0 = get_Vmax0();

	// Normalize the muscle states.
	normState[STATE_ACTIVATION] = getActivation(s);
	normState[STATE_FIBER_LENGTH] = getFiberLength(s) / optimalFiberLength;
	normState[STATE_ACTIVE_MOTOR_UNITS] = getActiveMotorUnits(s);
	normState[STATE_FATIGUED_MOTOR_UNITS] = getFatiguedMotorUnits(s);

	// Maximum contraction velocity is an activation scaled value.
	double Vmax = vmax;
	if (normState[STATE_ACTIVATION]<1.0)
		Vmax = vmax0 + normState[STATE_ACTIVATION]*(Vmax-vmax0);
	Vmax = Vmax*optimalFiberLength;

	// Compute normalized muscle state derivatives.
	if (getExcitation(s) >= normState[STATE_ACTIVATION])
      normStateDeriv[STATE_ACTIVATION] = (getExcitation(s) - normState[STATE_ACTIVATION]) / activationTimeConstant;
	else
      normStateDeriv[STATE_ACTIVATION] = (getExcitation(s) - normState[STATE_ACTIVATION]) / deactivationTimeConstant;

	pennation_angle = calcPennation( normState[STATE_FIBER_LENGTH], 1.0, pennationAngleAtOptimal);
	ca = cos(pennation_angle);

	norm_muscle_tendon_length = getLength(s) / optimalFiberLength;
	norm_tendon_length = norm_muscle_tendon_length - normState[STATE_FIBER_LENGTH] * ca;

	tendonForce = calcTendonForce(s,norm_tendon_length);
	setPassiveForce(s, calcPassiveForce(s,normState[STATE_FIBER_LENGTH]));
	double activeForce = calcActiveForce(s,normState[STATE_FIBER_LENGTH]);
	
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
         double h = norm_muscle_tendon_length - tendonSlackLength;
         double w = optimalFiberLength * sin(pennationAngleAtOptimal);
         double new_fiber_length = sqrt(h*h + w*w) / optimalFiberLength;
			double new_pennation_angle = calcPennation( new_fiber_length, 1.0, pennationAngleAtOptimal);
         double new_ca = cos(new_pennation_angle);
         normStateDeriv[STATE_FIBER_LENGTH] = getSpeed(s) / (Vmax * new_ca);
		}
	}
	else
	{
      double velocity_dependent_force = tendonForce / ca - getPassiveForce(s);
      normStateDeriv[STATE_FIBER_LENGTH] = calcFiberVelocity(s, getActiveMotorUnits(s), activeForce, velocity_dependent_force);
	}

	normStateDeriv[STATE_ACTIVE_MOTOR_UNITS] = normStateDeriv[STATE_ACTIVATION] - getFatigueFactor() * getActiveMotorUnits(s) + getRecoveryFactor() * getFatiguedMotorUnits(s);
	normStateDeriv[STATE_FATIGUED_MOTOR_UNITS]  = getFatigueFactor() * getActiveMotorUnits(s) - getRecoveryFactor() * getFatiguedMotorUnits(s);

	// Un-normalize the muscle state derivatives and forces.
	// Note: Do not need to Un-Normalize activation dynamics equation since activation, deactivation parameters
	// specified in muscle file are now independent of time scale
	setActivationDeriv(s, normStateDeriv[STATE_ACTIVATION]) ;
	setFiberLengthDeriv(s, normStateDeriv[STATE_FIBER_LENGTH] * Vmax );
	setActiveMotorUnitsDeriv(s, normStateDeriv[STATE_ACTIVE_MOTOR_UNITS]);
	setFatiguedMotorUnitsDeriv(s, normStateDeriv[STATE_FATIGUED_MOTOR_UNITS]);

	tendonForce = tendonForce *  maxIsometricForce;
	setForce( s, tendonForce );
	setTendonForce( s, tendonForce );
	setPassiveForce( s, getPassiveForce(s) * maxIsometricForce);
	
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
	const double optimalFiberLength = get_optimal_fiber_length();
	const double fatigueFactor = get_fatigue_factor();
	const double recoveryFactor = get_recovery_factor();

    if (optimalFiberLength < ROUNDOFF_ERROR) {
       return 0.0;
    }

	// This muscle model includes two fatigue states, so this function assumes
	// that t=infinity in order to compute the [steady-state] isometric force.
	// When t=infinity, the number of active motor units is independent of the
	// activation level (JPL: as long as activation > _recoveryFactor /
	// (_fatigueFactor + _recoveryFactor), I think). So the passed-in activation
	// is not used in this function (unless the fatigue and recovery factors are
	// both zero which means there is no fatigue).
	if ((fatigueFactor + recoveryFactor > 0.0) && (aActivation >= recoveryFactor / (fatigueFactor + recoveryFactor))) {
		setActiveMotorUnits(s, recoveryFactor / (fatigueFactor + recoveryFactor));
		setFatiguedMotorUnits(s, fatigueFactor / (fatigueFactor + recoveryFactor));
	} else {
		setActiveMotorUnits(s, aActivation);
		setFatiguedMotorUnits(s, 0.0);
	}

	aActivation = getActiveMotorUnits(s);

	// Now you can call the base class's function with the steady-state activation
	return Super::computeIsometricForce(s, aActivation);
}

