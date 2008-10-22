// ContDerivMuscle.cpp
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
#include "ContDerivMuscle.h"
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/DebugUtilities.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

const int ContDerivMuscle::STATE_ACTIVATION = 0;
const int ContDerivMuscle::STATE_FIBER_LENGTH = 1;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ContDerivMuscle::ContDerivMuscle() :
   AbstractMuscle(),
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
ContDerivMuscle::~ContDerivMuscle()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMuscle ContDerivMuscle to be copied.
 */
ContDerivMuscle::ContDerivMuscle(const ContDerivMuscle &aMuscle) :
   AbstractMuscle(aMuscle),
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
	setup(aMuscle.getModel());
}

//_____________________________________________________________________________
/**
 * Copy this muscle point and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this ContDerivMuscle.
 */
Object* ContDerivMuscle::copy() const
{
	ContDerivMuscle *musc = new ContDerivMuscle(*this);
	return(musc);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one ContDerivMuscle to another.
 *
 * @param aMuscle ContDerivMuscle to be copied.
 */
void ContDerivMuscle::copyData(const ContDerivMuscle &aMuscle)
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
 * Set the data members of this ContDerivMuscle to their null values.
 */
void ContDerivMuscle::setNull()
{
	setType("ContDerivMuscle");

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
void ContDerivMuscle::setupProperties()
{
	_maxIsometricForceProp.setName("max_isometric_force");
	_maxIsometricForceProp.setValue(546.0);
   _maxIsometricForceProp.setComment("maximum isometric force of the muscle fibers");
	_propertySet.append(&_maxIsometricForceProp, "Parameters");

	_optimalFiberLengthProp.setName("optimal_fiber_length");
	_optimalFiberLengthProp.setValue(0.0535);
	_optimalFiberLengthProp.setComment("optimal length of the muscle fibers");
	_propertySet.append(&_optimalFiberLengthProp, "Parameters");

	_tendonSlackLengthProp.setName("tendon_slack_length");
	_tendonSlackLengthProp.setValue(0.078);
	_tendonSlackLengthProp.setComment("resting length of the tendon");
	_propertySet.append(&_tendonSlackLengthProp, "Parameters");

	_pennationAngleProp.setName("pennation_angle");
	_pennationAngleProp.setComment("angle between tendon and fibers at optimal fiber length");
	_pennationAngleProp.setValue(0.0);
	_propertySet.append(&_pennationAngleProp, "Parameters");

	_activationTimeConstantProp.setName("activation_time_constant");
	_activationTimeConstantProp.setValue(0.01);
	_activationTimeConstantProp.setComment("time constant for ramping up of muscle activation");
	_propertySet.append(&_activationTimeConstantProp, "Parameters");

	_deactivationTimeConstantProp.setName("deactivation_time_constant");
	_deactivationTimeConstantProp.setValue(0.04);
	_deactivationTimeConstantProp.setComment("time constant for ramping down of muscle activation");
	_propertySet.append(&_deactivationTimeConstantProp, "Parameters");

	_vmaxProp.setName("Vmax");
	_vmaxProp.setValue(10.0);
	_vmaxProp.setComment("maximum contraction velocity at full activation in fiber lengths per second");
	_propertySet.append(&_vmaxProp, "Parameters");

	_vmax0Prop.setName("Vmax0");
	_vmax0Prop.setValue(5.0);
	_vmax0Prop.setComment("maximum contraction velocity at low activation in fiber lengths per second");
	_propertySet.append(&_vmax0Prop, "Parameters");

	_fmaxTendonStrainProp.setName("FmaxTendonStrain");
	_fmaxTendonStrainProp.setValue(0.033);
	_fmaxTendonStrainProp.setComment("tendon strain due to maximum isometric muscle force");
	_propertySet.append(&_fmaxTendonStrainProp, "Parameters");

	_fmaxMuscleStrainProp.setName("FmaxMuscleStrain");
	_fmaxMuscleStrainProp.setValue(0.6);
	_fmaxMuscleStrainProp.setComment("passive muscle strain due to maximum isometric muscle force");
	_propertySet.append(&_fmaxMuscleStrainProp, "Parameters");

	_kShapeActiveProp.setName("KshapeActive");
	_kShapeActiveProp.setValue(0.5);
	_kShapeActiveProp.setComment("shape factor for Gaussian active muscle force-length relationship");
	_propertySet.append(&_kShapeActiveProp, "Parameters");

	_kShapePassiveProp.setName("KshapePassive");
	_kShapePassiveProp.setValue(4.0);
	_kShapePassiveProp.setComment("exponential shape factor for passive force-length relationship");
	_propertySet.append(&_kShapePassiveProp, "Parameters");

	_dampingProp.setName("damping");
	_dampingProp.setValue(0.05);
	_dampingProp.setComment("passive damping in the force-velocity relationship");
	_propertySet.append(&_dampingProp, "Parameters");

	_afProp.setName("Af");
	_afProp.setValue(0.3);
	_afProp.setComment("force-velocity shape factor");
	_propertySet.append(&_afProp, "Parameters");

	_flenProp.setName("Flen");
	_flenProp.setValue(1.8);
	_flenProp.setComment("maximum normalized lengthening force");
	_propertySet.append(&_flenProp, "Parameters");
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this ContDerivMuscle.
 */
void ContDerivMuscle::setup(Model* aModel)
{
	// Base class
	AbstractMuscle::setup(aModel);

	// aModel will be NULL when objects are being registered.
	if (aModel == NULL)
		return;

	// Reasonable initial activation value
	_activation = 0.01;

	// Compute isometric force to get starting value
	// of _fiberLength.
   computeEquilibrium();
}

//_____________________________________________________________________________
/**
 * Copy the property values from another actuator, which may not be
 * a ContDerivMuscle.
 *
 * @param aActuator Actuator to copy property values from.
 */
void ContDerivMuscle::copyPropertyValues(AbstractActuator& aActuator)
{
	AbstractMuscle::copyPropertyValues(aActuator);

	const Property* prop = aActuator.getPropertySet().contains("max_isometric_force");
	if (prop) _maxIsometricForceProp.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("optimal_fiber_length");
	if (prop) _optimalFiberLengthProp.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("tendon_slack_length");
	if (prop) _tendonSlackLengthProp.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("pennation_angle");
	if (prop) _pennationAngleProp.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("activation_time_constant");
	if (prop) _activationTimeConstantProp.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("deactivation_time_constant");
	if (prop) _deactivationTimeConstantProp.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("Vmax");
	if (prop) _vmaxProp.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("Vmax0");
	if (prop) _vmax0Prop.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("FmaxTendonStrain");
	if (prop) _fmaxTendonStrainProp.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("FmaxMuscleStrain");
	if (prop) _fmaxMuscleStrainProp.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("KshapeActive");
	if (prop) _kShapeActiveProp.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("KshapePassive");
	if (prop) _kShapePassiveProp.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("damping");
	if (prop) _dampingProp.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("Af");
	if (prop) _afProp.setValue(prop->getValueDbl());

	prop = aActuator.getPropertySet().contains("Flen");
	if (prop) _flenProp.setValue(prop->getValueDbl());
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
ContDerivMuscle& ContDerivMuscle::operator=(const ContDerivMuscle &aMuscle)
{
	// BASE CLASS
	AbstractMuscle::operator=(aMuscle);

	copyData(aMuscle);

	setup(aMuscle.getModel());

	return(*this);
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
void ContDerivMuscle::scale(const ScaleSet& aScaleSet)
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
void ContDerivMuscle::postScale(const ScaleSet& aScaleSet)
{
	// Base class
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
double ContDerivMuscle::getPennationAngle()
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
double ContDerivMuscle::getFiberLength()
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
double ContDerivMuscle::getNormalizedFiberLength()
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
 * @param Current passive force of the muscle fiber(s).
 */
double ContDerivMuscle::getPassiveFiberForce()
{
	return _passiveForce;
}


//=============================================================================
// COMPUTATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the derivatives of the muscle states.
 *
 * @param rDYDT the state derivatives are returned here.
 */
void ContDerivMuscle::computeStateDerivatives(double rDYDT[])
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
void ContDerivMuscle::computeEquilibrium()
{
	double force = computeIsometricForce(_activation);

	//cout<<getName()<<": isometric force = "<<force<<endl;
	//cout<<getName()<<": fiber length = "<<_fiberLength<<endl;
}

//_____________________________________________________________________________
/**
 * Compute the actuation for the muscle. This function assumes
 * that computeDerivatives has already been called.
 *
 * This function is based on muscle_deriv_func9 from derivs.c (old pipeline code)
 */
void ContDerivMuscle::computeActuation()
{
	// Base Class (to calculate speed)
   AbstractMuscle::computeActuation();

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

	pennation_angle = AbstractMuscle::calcPennation(normState[STATE_FIBER_LENGTH], 1.0, _pennationAngle);
   ca = cos(pennation_angle);

   norm_muscle_tendon_length = getLength() / _optimalFiberLength; //REMEMBER - NORMALIZED IN OPTIMAL FIBER LENGTH UNITS
   norm_tendon_length = norm_muscle_tendon_length - normState[STATE_FIBER_LENGTH] * ca;


   _tendonForce = calcTendonForce(norm_tendon_length);
   _passiveForce = calcPassiveForce(normState[STATE_FIBER_LENGTH]);
   _activeForce = calcActiveForce(normState[STATE_FIBER_LENGTH]);
	
//cout << tendon_length << "," << _length << endl;
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
		 double new_pennation_angle = AbstractMuscle::calcPennation(new_fiber_length, 1.0, _pennationAngle);
         double new_ca = cos(new_pennation_angle);
         normStateDeriv[STATE_FIBER_LENGTH] = getSpeed() / (Vmax * new_ca);
	  }
   }
   else
   {
      double velocity_dependent_force = _tendonForce / ca - _passiveForce;
      normStateDeriv[STATE_FIBER_LENGTH] = calcFiberVelocity(normState[STATE_ACTIVATION],_activeForce,velocity_dependent_force);
   }
   /* Un-normalize the muscle state derivatives and forces. */
   /* Note: Do not need to Un-Normalize activation dynamics equation since activation, deactivation parameters
     specified in muscle file are now independent of time scale */
   _activationDeriv = normStateDeriv[STATE_ACTIVATION];
   _fiberLengthDeriv = normStateDeriv[STATE_FIBER_LENGTH] * Vmax;

	_tendonForce *= _maxIsometricForce;
	_passiveForce *= _maxIsometricForce;
	_activeForce *= normState[STATE_ACTIVATION] * _maxIsometricForce;
	//ms->tendon_length = norm_tendon_length*(*(ms->optimal_fiber_length));

	setForce(_tendonForce);
}

//_____________________________________________________________________________
/**
 * Tendon force is calculated by an integral-of-sigmoid function - zero for most negative inputs, linear
 * for most positive inputs, and a toe region that's continuous and smooth with both linear regions.
 *
 * The slope of the linear region is specified by the variable klin, and is taken from older code by Thelen.
 * Currently, this value is about 51.87, which is bigger than the value found in the spline usually specifying
 * the Schutte model (which has an effective slope of about 37.52).
 *
 * The size of the toe region (where the function transitions from one linear region to another) is parameterized
 * by "smoothcoef".  The bigger its value, the wider the toe region.  For smoothcoef=1, the area where the
 * discrepency between the linear models and the function is bigger than 0.1 is about [-0.04, 0.04], and the area
 * where it is bigger than 0.5 is smaller than [-0.01, 0.01], with maximum discrepency of about 0.7.
 *
 * Note that while the tendon strain depends on the tendon length normalized in tendon slack length units, the
 * input to this function is assumed to be the tendon length normalized in optimal fiber length units, hence the
 * transformations in the first two lines.
 *
 * FmaxTendonStrain - this function is parameterized by the tendon strain due to maximum isometric muscle force
 *     This should be specified as a dynamic parameter in the muscle file
 *
 * @param aNormTendonLength Normalized length of the tendon.
 * @return The force in the tendon.
 */
double ContDerivMuscle::calcTendonForce(double aNormTendonLength) const
{
    double norm_resting_length = _tendonSlackLength / _optimalFiberLength;
    double tendon_strain =  (aNormTendonLength - norm_resting_length) / norm_resting_length;

	double klin = 1.712/_fmaxTendonStrain;

	double tendon_force;

	double smoothcoef=1;
	tendon_force=smoothcoef*log(1+exp(klin*tendon_strain/smoothcoef));

   return tendon_force;
}

//_____________________________________________________________________________
/**
 *
 * Written by Darryl Thelen, modified by Tom Erez
 * this routine calculates the passive force in the muscle fibers using
 * an exponential-linear function instead of cubic splines.
 *
 * This equation is parameterized using the following dynamic parameters
 * which must be specified in the muscle file
 * Dynamic Parameters:
 *   FmaxMuscleStrain - passive muscle strain due to the application of 
 *                      maximum isometric muscle force
 *	 KshapePassive - exponential shape factor
 *
 * The normalized force due to passive stretch is given by
 *  For L < (1+maxStrain)*Lo
 *		f/f0 = exp(ks*(L-1)/maxStrain) / exp(ks)
 *
 * At higher normalized lengths, we transition to a linear model through a smoothed-sigmoidal transition.
 *
 * @param aNormFiberLength Normalized length of the muscle fiber.
 * @return The passive force in the muscle fibers.
 */
double ContDerivMuscle::calcPassiveForce(double aNormFiberLength) const
{
	double passive_force;

	double slope=_kShapePassive/_fmaxMuscleStrain;
	double decision1=aNormFiberLength-(1+_fmaxMuscleStrain);
	double passive_forceR = 1.0+slope*(aNormFiberLength-(1.0+_fmaxMuscleStrain));
	double passive_forceL = (exp(_kShapePassive*(aNormFiberLength-1.0)/_fmaxMuscleStrain)) / (exp(_kShapePassive));
	double decision2 = 1/(1+exp(-100*decision1));

	passive_force=decision2*passive_forceR+(1-decision2)*passive_forceL;


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
double ContDerivMuscle::calcActiveForce(double aNormFiberLength) const
{
	double x=-(aNormFiberLength-1.)*(aNormFiberLength-1.)/_kShapeActive;
	return exp(x);
}

//_____________________________________________________________________________
/**
 * This function is based on the equations in appendix 3 to Lisa Schutte's Ph.D. dissertation
 * (Stanford, Dec. 1992), as implemented in the Schutte1993Muscle.
 *  My modification involves smoothing the transition between the two formulas using a sigmoidal function.
 *
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




double ContDerivMuscle::calcFiberVelocity(double aActivation, double aActiveForce, double aVelocityDependentForce) const
{
   double b1, c1, fiber_velocity1;
   double b2, c2, fiber_velocity2;
   double fiber_velocity;
   double kv = 0.15, slope_k = 0.13, fmax = 1.4;
   double decision1, decision2;

   if (aVelocityDependentForce < -_damping)
	{
      fiber_velocity = aVelocityDependentForce / _damping;
	}
   else
   {
      c1 = kv * (aVelocityDependentForce - aActivation * aActiveForce) / _damping;
      b1 = -kv * (aVelocityDependentForce / kv + aActivation * aActiveForce +
			_damping) / _damping;
      fiber_velocity1 = (-b1 - sqrt(b1 * b1 - 4 * c1)) / 2.0;

      c2 = -(slope_k * kv / ((_damping * (kv + 1)))) *
	      (aVelocityDependentForce - aActivation * aActiveForce);
      b2 = -(aVelocityDependentForce / _damping
			-fmax * aActivation * aActiveForce / _damping - slope_k * kv / (kv + 1));
      fiber_velocity2 = (-b2 + sqrt(b2 * b2 - 4 * c2)) / 2.0;

	  decision1=aVelocityDependentForce - aActivation * aActiveForce;
	  decision2=1/(1+exp(-10*decision1));
      fiber_velocity = decision2*fiber_velocity2 + (1-decision2)*fiber_velocity1;
   }
   return fiber_velocity;
}
/*
//_____________________________________________________________________________
/**
 * Get the stress in this actuator.  It is calculated as the force divided
 * by the maximum isometric force (which is proportional to its area).
 */
double ContDerivMuscle::getStress() const
{
	return _force / _maxIsometricForce;
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
double ContDerivMuscle::
computeIsometricForce(double aActivation)
{
#define MAX_ITERATIONS 100
#define ERROR_LIMIT 0.01

   int i;
   double tendon_length, fiber_force, tmp_fiber_length, min_tendon_stiffness;
   double cos_factor, fiber_stiffness;
   double old_fiber_length, length_change, tendon_stiffness, percent;
   double error_force = 0.0, old_error_force, tendon_force, norm_tendon_length;
   
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

      norm_tendon_length = tendon_length / _optimalFiberLength;
      tendon_force = calcTendonForce(norm_tendon_length) * _maxIsometricForce;

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

         tendon_stiffness = calcTendonForce(norm_tendon_length) *
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
double ContDerivMuscle::
computeIsokineticForceAssumingInfinitelyStiffTendon(double aActivation)
{
	double isometricForce = computeIsometricForce(aActivation);
	double normalizedLength = _fiberLength / _optimalFiberLength;
	double normalizedVelocity = - _speed / (_vmax * _optimalFiberLength);
	normalizedVelocity *= cos(_pennationAngle);
	double normalizedForceVelocity = evaluateForceLengthVelocityCurve(1.0,1.0,normalizedVelocity);
	
	return isometricForce * normalizedForceVelocity;
}
