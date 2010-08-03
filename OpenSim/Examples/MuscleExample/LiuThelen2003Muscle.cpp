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
 *
 * Properties should be given a meaningful name and an informative comment.
 * The name you give each property is the tag that will be used in the XML
 * file. The comment will appear before the property in the XML file.
 * In addition, the comments are used for tool tips in the OpenSim GUI.
 *
 * All properties are added to the property set. Once added, they can be
 * read in and written to files.
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
	// Reasonable initial activation value
	setActivation(state, 0.01);
	setFiberLength(state, getOptimalFiberLength());
	setActiveMotorUnits(state, 0.0);
	setFatiguedMotorUnits(state, 0.0);
	_model->getSystem().realize(state, SimTK::Stage::Velocity);

	// Compute isometric force to get starting value of _fiberLength.
	computeEquilibrium(state);
}

void LiuThelen2003Muscle::initStateCache(SimTK::State& s, SimTK::SubsystemIndex subsystemIndex, Model& model) 
{
	Thelen2003Muscle::initStateCache(s, subsystemIndex, model);
}

//_____________________________________________________________________________
/**
 * Copy the property values from another actuator, which may not be
 * a LiuThelen2003Muscle. This method is most commonly used when
 * switching the type of a muscle in the OpenSim GUI. Because the
 * types of the two muscles are different, this method looks for
 * properties by name in the muscle to be copied.
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
	double force = computeIsometricForce(s, getActivation(s));
}

//_____________________________________________________________________________
/**
 * Compute the actuation for the muscle. This function assumes
 * that computeDerivatives has already been called.
 */
double  LiuThelen2003Muscle::computeActuation(const SimTK::State& s) const
{
   double tendonForce;
   double normState[4], normStateDeriv[4], norm_tendon_length, ca;
   double norm_muscle_tendon_length, pennation_angle;

	// Base Class (to calculate speed)
	Muscle::computeLengtheningSpeed(s);

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

	tendonForce = tendonForce *  _maxIsometricForce;
	setForce( s, tendonForce );
	setTendonForce( s, tendonForce );
	setPassiveForce( s, getPassiveForce(s) * _maxIsometricForce);
	
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
   if (_optimalFiberLength < ROUNDOFF_ERROR) {
      return 0.0;
   }

	// This muscle model includes two fatigue states, so this function assumes
	// that t=infinity in order to compute the [steady-state] isometric force.
	// When t=infinity, the number of active motor units is independent of the
	// activation level (JPL: as long as activation > _recoveryFactor /
	// (_fatigueFactor + _recoveryFactor), I think). So the passed-in activation
	// is not used in this function (unless the fatigue and recovery factors are
	// both zero which means there is no fatigue).
	if ((_fatigueFactor + _recoveryFactor > 0.0) && (aActivation >= _recoveryFactor / (_fatigueFactor + _recoveryFactor))) {
		setActiveMotorUnits(s, _recoveryFactor / (_fatigueFactor + _recoveryFactor));
		setFatiguedMotorUnits(s, _fatigueFactor / (_fatigueFactor + _recoveryFactor));
	} else {
		setActiveMotorUnits(s, aActivation);
		setFatiguedMotorUnits(s, 0.0);
	}

	aActivation = getActiveMotorUnits(s);

	// Now you can call the base class's function with the steady-state activation
	return Thelen2003Muscle::computeIsometricForce(s, aActivation);
}

int LiuThelen2003Muscle::getStateVariableYIndex(int index) const
{
	if (index<4 && index >=0)
		return _model->getSystem().getDefaultState().getZStart()+_zIndex+index;
	throw Exception("Trying to get State variable YIndex for LiuThelen2003Muscle "+getName()+" at undefined index"); 

}
