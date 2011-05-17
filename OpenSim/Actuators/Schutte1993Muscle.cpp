
// Author: Peter Loan, Jeff Reinbolt
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
#include <OpenSim/Common/NaturalCubicSpline.h>
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Simulation/Model/Model.h>

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
   ActivationFiberLengthMuscle(),
	_timeScale(_timeScaleProp.getValueDbl()),
	_activation1(_activation1Prop.getValueDbl()),
	_activation2(_activation2Prop.getValueDbl()),
	_damping(_dampingProp.getValueDbl()),
	_tendonForceLengthCurve(_tendonForceLengthCurveProp.getValueObjPtrRef()),
	_activeForceLengthCurve(_activeForceLengthCurveProp.getValueObjPtrRef()),
	_passiveForceLengthCurve(_passiveForceLengthCurveProp.getValueObjPtrRef())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Constructor.
 */
Schutte1993Muscle::Schutte1993Muscle(const std::string &aName,double aMaxIsometricForce,double aOptimalFiberLength,double aTendonSlackLength,double aPennationAngle) :
   ActivationFiberLengthMuscle(),
	_timeScale(_timeScaleProp.getValueDbl()),
	_activation1(_activation1Prop.getValueDbl()),
	_activation2(_activation2Prop.getValueDbl()),
	_damping(_dampingProp.getValueDbl()),
	_tendonForceLengthCurve(_tendonForceLengthCurveProp.getValueObjPtrRef()),
	_activeForceLengthCurve(_activeForceLengthCurveProp.getValueObjPtrRef()),
	_passiveForceLengthCurve(_passiveForceLengthCurveProp.getValueObjPtrRef())
{
	setNull();
	setupProperties();
	setName(aName);
	setMaxIsometricForce(aMaxIsometricForce);
	setOptimalFiberLength(aOptimalFiberLength);
	setTendonSlackLength(aTendonSlackLength);
	setPennationAngle(aPennationAngle);
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
   ActivationFiberLengthMuscle(aMuscle),
	_timeScale(_timeScaleProp.getValueDbl()),
	_activation1(_activation1Prop.getValueDbl()),
	_activation2(_activation2Prop.getValueDbl()),
	_damping(_dampingProp.getValueDbl()),
	_tendonForceLengthCurve(_tendonForceLengthCurveProp.getValueObjPtrRef()),
	_activeForceLengthCurve(_activeForceLengthCurveProp.getValueObjPtrRef()),
	_passiveForceLengthCurve(_passiveForceLengthCurveProp.getValueObjPtrRef())
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
	_damping = aMuscle._damping;
	_tendonForceLengthCurve = (Function*)Object::SafeCopy(aMuscle._tendonForceLengthCurve);
	_activeForceLengthCurve = (Function*)Object::SafeCopy(aMuscle._activeForceLengthCurve);
	_passiveForceLengthCurve = (Function*)Object::SafeCopy(aMuscle._passiveForceLengthCurve);
}

//_____________________________________________________________________________
/**
 * Set the data members of this Schutte1993Muscle to their null values.
 */
void Schutte1993Muscle::setNull()
{
	setType("Schutte1993Muscle");

	setNumStateVariables(2);

	_stateVariableSuffixes[STATE_ACTIVATION]="activation";
	_stateVariableSuffixes[STATE_FIBER_LENGTH]="fiber_length";

}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Schutte1993Muscle::setupProperties()
{
	_timeScaleProp.setName("time_scale");
	_timeScaleProp.setComment("Scale factor for normalizing time");
	_timeScaleProp.setValue(0.1);
	_propertySet.append(&_timeScaleProp, "Parameters");

	_activation1Prop.setName("activation1");
	_activation1Prop.setComment("Parameter used in time constant of ramping up of muscle force");
	_activation1Prop.setValue(7.667);
	_propertySet.append(&_activation1Prop, "Parameters");

	_activation2Prop.setName("activation2");
	_activation2Prop.setComment("Parameter used in time constant of ramping up and ramping down of muscle force");
	_activation2Prop.setValue(1.459854);
	_propertySet.append(&_activation2Prop, "Parameters");

	_dampingProp.setName("damping");
	_dampingProp.setComment("Damping factor related to maximum contraction velocity");
	_dampingProp.setValue(0.1);
	_propertySet.append(&_dampingProp, "Parameters");

	_tendonForceLengthCurveProp.setName("tendon_force_length_curve");
	_tendonForceLengthCurveProp.setComment("Function representing force-length behavior of tendon");
	int tendonForceLengthCurvePoints = 17;
	double tendonForceLengthCurveX[] = {-10.00000000, -0.00200000, -0.00100000,  0.00000000,  0.00131000,  0.00281000,  0.00431000,  0.00581000,  0.00731000,  0.00881000,  0.01030000,  0.01180000,  0.01230000,  9.20000000,  9.20100000,  9.20200000, 20.00000000};
	double tendonForceLengthCurveY[] = {0.00000000,  0.00000000,  0.00000000,  0.00000000,  0.01080000,  0.02570000,  0.04350000,  0.06520000,  0.09150000,  0.12300000,  0.16100000,  0.20800000,  0.22700000,  345.00000000,  345.00000000,  345.00000000,  345.00000000};
	NaturalCubicSpline *tendonForceLengthCurve = new NaturalCubicSpline(tendonForceLengthCurvePoints, tendonForceLengthCurveX, tendonForceLengthCurveY);
	_tendonForceLengthCurveProp.setValue(tendonForceLengthCurve);
	_propertySet.append(&_tendonForceLengthCurveProp, "Functions");

	_activeForceLengthCurveProp.setName("active_force_length_curve");
	_activeForceLengthCurveProp.setComment("Function representing active force-length behavior of muscle fibers");
	int activeForceLengthCurvePoints = 21;
	double activeForceLengthCurveX[] = {-5.30769200, -4.30769200, -1.92307700, -0.88461500, -0.26923100,  0.23076900,  0.46153800,  0.52725000,  0.62875000,  0.71875000,  0.86125000,  1.04500000,  1.21750000,  1.43875000,  1.50000000,  1.61538500,  2.00000000,  2.96153800,  3.69230800,  5.46153800,  9.90190200};
	double activeForceLengthCurveY[] = {0.01218800,  0.02189900,  0.03646600,  0.05249300,  0.07531200,  0.11415800,  0.15785900,  0.22666700,  0.63666700,  0.85666700,  0.95000000,  0.99333300,  0.77000000,  0.24666700,  0.19382100,  0.13325200,  0.07268300,  0.04441700,  0.03634100,  0.02189900,  0.00733200};
	NaturalCubicSpline *activeForceLengthCurve = new NaturalCubicSpline(activeForceLengthCurvePoints, activeForceLengthCurveX, activeForceLengthCurveY);
	_activeForceLengthCurveProp.setValue(activeForceLengthCurve);
	_propertySet.append(&_activeForceLengthCurveProp, "Functions");

	_passiveForceLengthCurveProp.setName("passive_force_length_curve");
	_passiveForceLengthCurveProp.setComment("Function representing passive force-length behavior of muscle fibers");
	int passiveForceLengthCurvePoints = 13;
	double passiveForceLengthCurveX[] = {-5.00000000,  0.99800000,  0.99900000,  1.00000000,  1.10000000,  1.20000000,  1.30000000,  1.40000000,  1.50000000,  1.60000000,  1.60100000,  1.60200000,  5.00000000};
	double passiveForceLengthCurveY[] = {0.00000000,  0.00000000,  0.00000000,  0.00000000,  0.03500000,  0.12000000,  0.26000000,  0.55000000,  1.17000000,  2.00000000,  2.00000000,  2.00000000,  2.00000000};
	NaturalCubicSpline *passiveForceLengthCurve = new NaturalCubicSpline(passiveForceLengthCurvePoints, passiveForceLengthCurveX, passiveForceLengthCurveY);
	_passiveForceLengthCurveProp.setValue(passiveForceLengthCurve);
	_propertySet.append(&_passiveForceLengthCurveProp, "Functions");
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this Schutte1993Muscle.
 */
void Schutte1993Muscle::setup(Model& aModel)
{
	// Base class
	ActivationFiberLengthMuscle::setup(aModel);

	// _model will be NULL when objects are being registered.
	if (_model == NULL)
		return;

	if(!getActiveForceLengthCurve()) 
		throw Exception("Schutte1993Muscle.setup: ERROR- No active force length curve specified for muscle '"+getName()+"'",__FILE__,__LINE__);
	else if(!getPassiveForceLengthCurve())
		throw Exception("Schutte1993Muscle.setup: ERROR- No passive force length curve specified for muscle '"+getName()+"'",__FILE__,__LINE__);
	else if(!getTendonForceLengthCurve())
		throw Exception("Schutte1993Muscle.setup: ERROR- No tendon force length curve specified for muscle '"+getName()+"'",__FILE__,__LINE__);

}

void Schutte1993Muscle::createSystem(SimTK::MultibodySystem& system) const
{
	ActivationFiberLengthMuscle::createSystem(system);
	Schutte1993Muscle* mutableThis = const_cast<Schutte1993Muscle *>(this);

	// Cache the computed passive muscle force
	// note the total muscle force is the tendon force and is already a cached variable of the actuator
	mutableThis->addCacheVariable<double>("passiveForce", 0.0, SimTK::Stage::Velocity);
}

void Schutte1993Muscle::setPassiveForce(const SimTK::State& s, double force ) const {
    setCacheVariable<double>(s, "passiveForce", force);
}
double Schutte1993Muscle::getPassiveForce( const SimTK::State& s) const {
    return getCacheVariable<double>(s, "passiveForce");
}

void Schutte1993Muscle::setTendonForce(const SimTK::State& s, double force) const {
	setForce(s, force);
}
double Schutte1993Muscle::getTendonForce(const SimTK::State& s) const {
	return getForce(s);
}

//_____________________________________________________________________________
/**
 * Copy the property values from another actuator, which may not be
 * a Schutte1993Muscle.
 *
 * @param aActuator Actuator to copy property values from.
 */
void Schutte1993Muscle::copyPropertyValues(Actuator& aActuator)
{
	ActivationFiberLengthMuscle::copyPropertyValues(aActuator);

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
	ActivationFiberLengthMuscle::operator=(aMuscle);

	copyData(aMuscle);

	return(*this);
}


//=============================================================================
// GET
//=============================================================================
//-----------------------------------------------------------------------------
// TIME SCALE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the scale factor for normalizing time.
 *
 * @param aTimeScale The scale factor for normalizing time.
 * @return Whether the scale factor was successfully changed.
 */
bool Schutte1993Muscle::setTimeScale(double aTimeScale)
{
	_timeScale = aTimeScale;
	return true;
}

//-----------------------------------------------------------------------------
// ACTIVATION 1
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the time constant of ramping up of muscle force.
 *
 * @param aActivation1 The time constant of ramping up of muscle force.
 * @return Whether the time constant was successfully changed.
 */
bool Schutte1993Muscle::setActivation1(double aActivation1)
{
	_activation1 = aActivation1;
	return true;
}

//-----------------------------------------------------------------------------
// ACTIVATION 2
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the time constant of ramping up and ramping down of muscle force.
 *
 * @param aActivation1 The time constant of ramping up and ramping down of muscle force.
 * @return Whether the time constant was successfully changed.
 */
bool Schutte1993Muscle::setActivation2(double aActivation2)
{
	_activation2 = aActivation2;
	return true;
}

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
bool Schutte1993Muscle::setMaxIsometricForce(double aMaxIsometricForce)
{
	_maxIsometricForce = aMaxIsometricForce;
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
bool Schutte1993Muscle::setOptimalFiberLength(double aOptimalFiberLength)
{
	_optimalFiberLength = aOptimalFiberLength;
	return true;
}

//-----------------------------------------------------------------------------
// TENDON SLACK LENGTH
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the resting length of the tendon.
 *
 * @param aTendonSlackLength The resting length of the tendon.
 * @return Whether the resting length was successfully changed.
 */
bool Schutte1993Muscle::setTendonSlackLength(double aTendonSlackLength)
{
	_tendonSlackLength = aTendonSlackLength;
	return true;
}

//-----------------------------------------------------------------------------
// PENNATION ANGLE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the angle between tendon and fibers at optimal fiber length.
 *
 * @param aPennationAngle The angle between tendon and fibers at optimal fiber length.
 * @return Whether the angle was successfully changed.
 */
bool Schutte1993Muscle::setPennationAngle(double aPennationAngle)
{
	_pennationAngle = aPennationAngle;
	return true;
}

//-----------------------------------------------------------------------------
// MAX CONTRACTION VELOCITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the maximum contraction velocity of the fibers, in optimal fiber lengths per second.
 *
 * @param aMaxContractionVelocity The maximum contraction velocity of the fibers, in optimal fiber lengths per second.
 * @return Whether the maximum contraction velocity was successfully changed.
 */
bool Schutte1993Muscle::setMaxContractionVelocity(double aMaxContractionVelocity)
{
	_maxContractionVelocity = aMaxContractionVelocity;
	return true;
}

//-----------------------------------------------------------------------------
// DAMPING
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the damping factor related to maximum contraction velocity.
 *
 * @param aDamping The damping factor related to maximum contraction velocity.
 * @return Whether the damping factor was successfully changed.
 */
bool Schutte1993Muscle::setDamping(double aDamping)
{
	_damping = aDamping;
	return true;
}


//-----------------------------------------------------------------------------
// PENNATION ANGLE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the current pennation angle of the muscle fiber(s).
 *
 * @param Pennation angle.
 */
double Schutte1993Muscle::getPennationAngle(const SimTK::State& s) const
{
	return calcPennation(getFiberLength(s),_optimalFiberLength,_pennationAngle);
}

//-----------------------------------------------------------------------------
// LENGTH
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the normalized length of the muscle fiber(s).  This is the current
 * fiber length(s) divided by the optimal fiber length.
 *
 * @param Current length of the muscle fiber(s).
 */
double Schutte1993Muscle::getNormalizedFiberLength(const SimTK::State& s) const
{
	return getFiberLength(s) / getOptimalFiberLength();
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
double Schutte1993Muscle::getPassiveFiberForce(const SimTK::State& s) const
{
	return getPassiveForce(s);
}




//_____________________________________________________________________________
/**
 * Compute the derivatives of the muscle states.
 *
 * @param rDYDT the state derivatives are returned here.
 */
SimTK::Vector Schutte1993Muscle::computeStateVariableDerivatives(const SimTK::State &s) const
{
	SimTK::Vector derivs(getNumStateVariables());
	derivs[0] = getActivationDeriv(s);
	derivs[1] = getFiberLengthDeriv(s);
	return derivs; 
}

//_____________________________________________________________________________
/**
 * Compute the equilibrium states.  This method computes a fiber length
 * for the muscle that is consistent with the muscle's activation level.
 */
void Schutte1993Muscle::computeEquilibrium(SimTK::State& s ) const
{
	double force = computeIsometricForce( s, getActivation(s));

	//cout<<getName()<<": isometric force = "<<force<<endl;
	//cout<<getName()<<": fiber length = "<<_fiberLength<<endl;
}

//_____________________________________________________________________________
/**
 * Compute the actuation for the muscle. This function assumes
 * that computeDerivatives has already been called.
 */
double Schutte1993Muscle::computeActuation(const SimTK::State& s) const
{
    double tendonForce;
    double passiveForce;
    double activeForce;
    double activationDeriv;
    double fiberLengthDeriv;

   double norm_tendon_length, ca;
   double norm_muscle_tendon_length, pennation_angle;
   double excitation = getExcitation(s);

   /* Normalize the muscle states */
   double activation = getActivation(s);
   double normFiberLength = getFiberLength(s) / _optimalFiberLength;

   /* Compute normalized muscle state derivatives */
   if (excitation >= activation) 
       activationDeriv = (excitation - activation) * (_activation1 * excitation + _activation2);
   else
      activationDeriv = (excitation - activation) * _activation2;

	pennation_angle = calcPennation(normFiberLength, 1.0, _pennationAngle);
   ca = cos(pennation_angle);
   norm_muscle_tendon_length = getLength(s) / _optimalFiberLength;
   norm_tendon_length = norm_muscle_tendon_length - normFiberLength * ca;

   tendonForce = calcTendonForce(s,norm_tendon_length);
   passiveForce =  calcNonzeroPassiveForce(s,normFiberLength, 0.0);
   activeForce = getActiveForceLengthCurve()->calcValue(SimTK::Vector(1, normFiberLength) );
	if (activeForce < 0.0) activeForce = 0.0;

   /* If pennation equals 90 degrees, fiber length equals muscle width and fiber
    * velocity goes to zero.  Pennation will stay at 90 until tendon starts to
    * pull, then "stiff tendon" approximation is used to calculate approximate
    * fiber velocity.
    */
   if (EQUAL_WITHIN_ERROR(ca, 0.0)) {
      if (EQUAL_WITHIN_ERROR(tendonForce, 0.0)) {
         fiberLengthDeriv = 0.0;;
      } else {
         double h = norm_muscle_tendon_length - _tendonSlackLength;
         double w = _optimalFiberLength * sin(_pennationAngle);
         double new_fiber_length = sqrt(h*h + w*w) / _optimalFiberLength;
         double new_pennation_angle = calcPennation(new_fiber_length, 1.0, _pennationAngle);
         double new_ca = cos(new_pennation_angle);
         fiberLengthDeriv = getLengtheningSpeed(s) * _timeScale / _optimalFiberLength * new_ca;
      }
   } else {
      double velocity_dependent_force = tendonForce / ca - passiveForce;
	  if (velocity_dependent_force < 0.0) velocity_dependent_force = 0.0;
      fiberLengthDeriv  = calcFiberVelocity(s,activation, activeForce, velocity_dependent_force);
   }

   /* Un-normalize the muscle state derivatives and forces. */
   setActivationDeriv(s,  activationDeriv / _timeScale);
   setFiberLengthDeriv(s, fiberLengthDeriv * _optimalFiberLength / _timeScale);

	tendonForce = tendonForce * _maxIsometricForce;
	setForce(s, tendonForce);
	setTendonForce(s, tendonForce);
	setPassiveForce(s, passiveForce * _maxIsometricForce);


    return( tendonForce );
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
 * Set the active force-length curve.
 *
 * @param aActiveForceLengthCurve Pointer to an active force-length curve (Function).
 * @return Whether active force-length curve was successfully changed.
 */
bool Schutte1993Muscle::setActiveForceLengthCurve(Function* aActiveForceLengthCurve)
{
	_activeForceLengthCurve = aActiveForceLengthCurve;
	return true;
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
 * Get the passive force-length curve.
 *
 * @param aPassiveForceLengthCurve Pointer to a passive force-length curve (Function).
 * @return Whether passive force-length curve was successfully changed.
 */
bool Schutte1993Muscle::setPassiveForceLengthCurve(Function* aPassiveForceLengthCurve)
{
	_passiveForceLengthCurve = aPassiveForceLengthCurve;
	return true;
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
 * Get the tendon force-length curve.
 *
 * @param aTendonForceLengthCurve Pointer to a tendon force-length curve (Function).
 * @return Whether tendon force-length curve was successfully changed.
 */
bool Schutte1993Muscle::setTendonForceLengthCurve(Function* aTendonForceLengthCurve)
{
	_tendonForceLengthCurve = aTendonForceLengthCurve;
	return true;
}

//_____________________________________________________________________________
/**
 * Calculate the force in tendon by finding tendon strain
 * and using it to interpolate the tendon force-length curve.
 *
 * @param aNormTendonLength Normalized length of the tendon.
 * @return The force in the tendon.
 */
double Schutte1993Muscle::calcTendonForce(const SimTK::State& s, double aNormTendonLength) const
{
   double tendon_force;
   double norm_resting_length = _tendonSlackLength / _optimalFiberLength;
   double tendon_strain =  (aNormTendonLength - norm_resting_length) / norm_resting_length;

   if (tendon_strain < 0.0)
      tendon_force = 0.0;
   else
      tendon_force = getTendonForceLengthCurve()->calcValue(SimTK::Vector(1, tendon_strain));

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
double Schutte1993Muscle::calcNonzeroPassiveForce(const SimTK::State& s, double aNormFiberLength, double aNormFiberVelocity) const
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
double Schutte1993Muscle::calcFiberVelocity(const SimTK::State& s, double aActivation, double aActiveForce, double aVelocityDependentForce) const
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
double Schutte1993Muscle::getStress(const SimTK::State& s ) const
{
	return getForce(s) / _maxIsometricForce;
}

//_____________________________________________________________________________
/**
 * computeIsometricForce: this function finds the force in a muscle, assuming
 * static equilibrium. Using the total muscle-tendon length, it finds the
 * fiber and tendon lengths so that the forces in each match. This routine
 * takes pennation angle into account, so its definition of static equilibrium
 * is when tendon_force = fiber_force * cos(pennation_angle). This funcion
 * will modify the object's values for length, fiberLength,
 * and passiveForce.
 *
 * @param aActivation Activation of the muscle.
 * @return The isometric force in the muscle.
 */
double Schutte1993Muscle::computeIsometricForce(SimTK::State& s, double aActivation) const
{
#define MAX_ITERATIONS 100
#define ERROR_LIMIT 0.01

   int i;
   double length,tendon_length, fiber_force, tmp_fiber_length, min_tendon_stiffness;
   double cos_factor, fiber_stiffness;
   double old_fiber_length, length_change, tendon_stiffness, percent;
   double error_force = 0.0, old_error_force, tendon_force, tendon_strain;
   double passiveForce, activeForce, tendonForce, fiberLength;

   if (_optimalFiberLength < ROUNDOFF_ERROR) {
      setStateVariable(s, STATE_FIBER_LENGTH, 0.0);
		setPassiveForce(s, 0.0);
      setForce(s, 0.0);
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

   double muscle_width = _optimalFiberLength * sin(_pennationAngle);

   if (_tendonSlackLength < ROUNDOFF_ERROR) {
      tendon_length = 0.0;
      cos_factor = cos(atan(muscle_width / length));
      fiberLength = length / cos_factor;

		activeForce =  getActiveForceLengthCurve()->calcValue(SimTK::Vector(1, fiberLength / _optimalFiberLength)) * aActivation * _maxIsometricForce;
       if (activeForce < 0.0) activeForce = 0.0;

		passiveForce = calcNonzeroPassiveForce(s, fiberLength / _optimalFiberLength, 0.0) * _maxIsometricForce;

        setPassiveForce(s, passiveForce );
		setStateVariable(s, STATE_FIBER_LENGTH,  fiberLength);
		tendonForce = (activeForce + passiveForce) * cos_factor;
		setForce(s, tendonForce);
		setTendonForce(s, tendonForce);
      return tendonForce;
   } else if (length < _tendonSlackLength) {
		setStateVariable(s, STATE_FIBER_LENGTH,  muscle_width);
		setPassiveForce(s, 0.0);
      setForce(s, 0.0);
      setTendonForce(s, 0.0);
      return 0.0;
   } else {
      fiberLength = _optimalFiberLength;
      cos_factor = cos(calcPennation(fiberLength, _optimalFiberLength, _pennationAngle));  
      tendon_length = length - fiberLength * cos_factor;

      /* Check to make sure tendon is not shorter than its slack length. If it
       * is, set the length to its slack length and re-compute fiber length.
       */
      if (tendon_length < _tendonSlackLength) {
         tendon_length = _tendonSlackLength;
         cos_factor = cos(atan(muscle_width / (length - tendon_length)));
         fiberLength = (length - tendon_length) / cos_factor;
         if (fiberLength < muscle_width) fiberLength = muscle_width;
      }

   }

   // Muscle-tendon force is found using an iterative method. First, you guess
   // the length of the muscle fibers and the length of the tendon, and
   // calculate their respective forces. If the forces match (are within
   // ERROR_LIMIT of each other), stop; else change the length guesses based
   // on the error and try again.
   for (i = 0; i < MAX_ITERATIONS; i++) {
		activeForce = getActiveForceLengthCurve()->calcValue(SimTK::Vector(1, fiberLength / _optimalFiberLength)) * aActivation;
      if (activeForce < 0.0) activeForce = 0.0;

		passiveForce = calcNonzeroPassiveForce(s, fiberLength / _optimalFiberLength, 0.0);
      if (passiveForce < 0.0) passiveForce = 0.0;

      fiber_force = (activeForce + passiveForce ) * _maxIsometricForce * cos_factor;

      tendon_strain = (tendon_length / _tendonSlackLength - 1.0);
      if (tendon_strain < 0.0)
         tendon_force = 0.0;
      else
         tendon_force = getTendonForceLengthCurve()->calcValue(SimTK::Vector(1, tendon_strain)) * _maxIsometricForce;
		setForce(s, tendon_force);
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
         fiberLength += percent * (tmp_fiber_length - fiberLength);
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

         tendon_stiffness = getTendonForceLengthCurve()->calcValue(SimTK::Vector(1, tendon_strain)) *
				_maxIsometricForce / _tendonSlackLength;

         min_tendon_stiffness = (activeForce + passiveForce) *
	         tendon_elastic_modulus * _maxIsometricForce /
	         (tendon_max_stress * _tendonSlackLength);

         if (tendon_stiffness < min_tendon_stiffness)
            tendon_stiffness = min_tendon_stiffness;

         fiber_stiffness = _maxIsometricForce / _optimalFiberLength *
			 (getActiveForceLengthCurve()->calcValue(SimTK::Vector(1, fiberLength / _optimalFiberLength))  +
            calcNonzeroPassiveForce(s, fiberLength / _optimalFiberLength, 0.0));

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

      cos_factor = cos(calcPennation(fiberLength, _optimalFiberLength, _pennationAngle));
      tendon_length = length - fiberLength * cos_factor;

      // Check to make sure tendon is not shorter than its slack length. If it is,
      // set the length to its slack length and re-compute fiber length.
      if (tendon_length < _tendonSlackLength) {
         tendon_length = _tendonSlackLength;
         cos_factor = cos(atan(muscle_width / (length - tendon_length)));
         fiberLength = (length - tendon_length) / cos_factor;
      }
   }

   setStateVariable(s, STATE_FIBER_LENGTH,  fiberLength);
   setPassiveForce(s, passiveForce * _maxIsometricForce);

	return tendon_force;
}



int Schutte1993Muscle::getStateVariableYIndex(int index) const
{
	if (index==0)
		return _model->getMultibodySystem().getDefaultState().getZStart()+_zIndex;
	if (index ==1)
		return _model->getMultibodySystem().getDefaultState().getZStart()+_zIndex+1;
	throw Exception("Trying to get Coordinate State variable YIndex for Coorindate "+getName()+" at undefined index"); 

}
