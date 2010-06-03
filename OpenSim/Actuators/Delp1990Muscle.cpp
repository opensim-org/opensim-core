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
#include <OpenSim/Common/NaturalCubicSpline.h>
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Simulation/Model/Model.h>

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
   Muscle(),
	_timeScale(_timeScaleProp.getValueDbl()),
	_activation1(_activation1Prop.getValueDbl()),
	_activation2(_activation2Prop.getValueDbl()),
	_maxIsometricForce(_maxIsometricForceProp.getValueDbl()),
	_optimalFiberLength(_optimalFiberLengthProp.getValueDbl()),
	_tendonSlackLength(_tendonSlackLengthProp.getValueDbl()),
	_pennationAngle(_pennationAngleProp.getValueDbl()),
	_maxContractionVelocity(_maxContractionVelocityProp.getValueDbl()),
	_mass(_massProp.getValueDbl()),
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
   Muscle(aMuscle),
	_timeScale(_timeScaleProp.getValueDbl()),
	_activation1(_activation1Prop.getValueDbl()),
	_activation2(_activation2Prop.getValueDbl()),
	_maxIsometricForce(_maxIsometricForceProp.getValueDbl()),
	_optimalFiberLength(_optimalFiberLengthProp.getValueDbl()),
	_tendonSlackLength(_tendonSlackLengthProp.getValueDbl()),
	_pennationAngle(_pennationAngleProp.getValueDbl()),
	_maxContractionVelocity(_maxContractionVelocityProp.getValueDbl()),
	_mass(_massProp.getValueDbl()),
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
	_maxIsometricForce = aMuscle._maxIsometricForce;
	_optimalFiberLength = aMuscle._optimalFiberLength;
	_tendonSlackLength = aMuscle._tendonSlackLength;
	_pennationAngle = aMuscle._pennationAngle;
	_maxContractionVelocity = aMuscle._maxContractionVelocity;
	_mass = aMuscle._mass;
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

	setNumStateVariables(3);

	_stateVariableSuffixes[STATE_ACTIVATION] = "activation";
	_stateVariableSuffixes[STATE_FIBER_LENGTH] = "fiber_length";
	_stateVariableSuffixes[STATE_FIBER_VELOCITY] = "fiber_velocity";
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Delp1990Muscle::setupProperties()
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

	_maxIsometricForceProp.setName("max_isometric_force");
	_maxIsometricForceProp.setComment("Maximum isometric force that the fibers can generate");
	_maxIsometricForceProp.setValue(1000.0);
	_propertySet.append(&_maxIsometricForceProp, "Parameters");

	_optimalFiberLengthProp.setName("optimal_fiber_length");
	_optimalFiberLengthProp.setComment("Optimal length of the muscle fibers");
	_optimalFiberLengthProp.setValue(0.1);
	_propertySet.append(&_optimalFiberLengthProp, "Parameters");

	_tendonSlackLengthProp.setName("tendon_slack_length");
	_tendonSlackLengthProp.setComment("Resting length of the tendon");
	_tendonSlackLengthProp.setValue(0.2);
	_propertySet.append(&_tendonSlackLengthProp, "Parameters");

	_pennationAngleProp.setName("pennation_angle");
	_pennationAngleProp.setComment("Angle between tendon and fibers at optimal fiber length");
	_pennationAngleProp.setValue(0.0);
	_propertySet.append(&_pennationAngleProp, "Parameters");

	_maxContractionVelocityProp.setName("max_contraction_velocity");
	_maxContractionVelocityProp.setComment("Maximum contraction velocity of the fibers, in optimal fiberlengths per second");
	_maxContractionVelocityProp.setValue(10.0);
	_propertySet.append(&_maxContractionVelocityProp, "Parameters");

	_massProp.setName("mass");
	_massProp.setComment("Normalized mass of the muscle between the tendon and muscle fibers");
	_massProp.setValue(0.00287);
	_propertySet.append(&_massProp, "Parameters");

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

	_forceVelocityCurveProp.setName("force_velocity_curve");
	_forceVelocityCurveProp.setComment("Function representing force-velocity behavior of muscle fibers");
	int forceVelocityLengthCurvePoints = 42;
	double forceVelocityLengthCurveX[] = {-1.001000000000, -1.000000000000, -0.950000000000, -0.900000000000, -0.850000000000, -0.800000000000, -0.750000000000, -0.700000000000, -0.650000000000, -0.600000000000, -0.550000000000, -0.500000000000, -0.450000000000, -0.400000000000, -0.350000000000, -0.300000000000, -0.250000000000, -0.200000000000, -0.150000000000, -0.100000000000, -0.050000000000,
		0.000000000000, 0.050000000000, 0.100000000000, 0.150000000000, 0.200000000000, 0.250000000000, 0.300000000000, 0.350000000000, 0.400000000000, 0.450000000000, 0.500000000000, 0.550000000000, 0.600000000000, 0.650000000000, 0.700000000000, 0.750000000000, 0.800000000000, 0.850000000000, 0.900000000000, 0.950000000000, 1.000000000000};
	double forceVelocityLengthCurveY[] = {0.000000000000, 0.000000000000, 0.010417000000, 0.021739000000, 0.034091000000, 0.047619000000, 0.062500000000, 0.078947000000, 0.097222000000, 0.117647000000, 0.140625000000, 0.166667000000, 0.196429000000, 0.230769000000, 0.270833000000, 0.318182000000, 0.375000000000, 0.444444000000, 0.531250000000, 0.642857000000, 0.791667000000, 1.000000000000,
		1.482014000000, 1.601571000000, 1.655791000000, 1.686739000000, 1.706751000000, 1.720753000000, 1.731099000000, 1.739055000000, 1.745365000000, 1.750490000000, 1.754736000000, 1.758312000000, 1.761364000000, 1.763999000000, 1.766298000000, 1.768321000000, 1.770115000000, 1.771717000000, 1.773155000000, 1.774455000000};
	NaturalCubicSpline *forceVelocityLengthCurve = new NaturalCubicSpline(forceVelocityLengthCurvePoints, forceVelocityLengthCurveX, forceVelocityLengthCurveY);
	_forceVelocityCurveProp.setValue(forceVelocityLengthCurve);
	_propertySet.append(&_forceVelocityCurveProp, "Functions");
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this Delp1990Muscle.
 */
void Delp1990Muscle::setup(Model& aModel)
{
	// Base class
	Muscle::setup(aModel);

	// aModel will be NULL when objects are being registered.
	if (_model == NULL)
		return;

	if(!getActiveForceLengthCurve()) 
		throw Exception("Delp1990Muscle.setup: ERROR- No active force length curve specified for muscle '"+getName()+"'",__FILE__,__LINE__);
	else if(!getPassiveForceLengthCurve())
		throw Exception("Delp1990Muscle.setup: ERROR- No passive force length curve specified for muscle '"+getName()+"'",__FILE__,__LINE__);
	else if(!getTendonForceLengthCurve())
		throw Exception("Delp1990Muscle.setup: ERROR- No tendon force length curve specified for muscle '"+getName()+"'",__FILE__,__LINE__);
	else if(!getForceVelocityCurve())
		throw Exception("Delp1990Muscle.setup: ERROR- No force velocity curve specified for muscle '"+getName()+"'",__FILE__,__LINE__);
}

void Delp1990Muscle::equilibrate(SimTK::State& state) const
{
	Muscle::equilibrate(state);

	// Reasonable initial activation value
	setActivation(state, 0.01);
	setFiberLength(state, getOptimalFiberLength());
	_model->getSystem().realize(state, SimTK::Stage::Velocity);

	// Compute isometric force to get starting value of fiber length.
	computeEquilibrium(state);
}

void Delp1990Muscle:: initStateCache(SimTK::State& s, SimTK::SubsystemIndex subsystemIndex, Model& model) {
	Muscle::initStateCache(s, subsystemIndex, model);

	_tendonForceIndex = s.allocateCacheEntry( subsystemIndex, SimTK::Stage::Topology, new SimTK::Value<double>() );
	_activeForceIndex = s.allocateCacheEntry( subsystemIndex, SimTK::Stage::Topology, new SimTK::Value<double>() );
	_passiveForceIndex = s.allocateCacheEntry( subsystemIndex, SimTK::Stage::Topology, new SimTK::Value<double>() );
}

void Delp1990Muscle::setTendonForce(const SimTK::State& s, double force) const {
	SimTK::Value<double>::downcast(s.updCacheEntry( _subsystemIndex, _tendonForceIndex)).upd() = force;
}
double Delp1990Muscle::getTendonForce(const SimTK::State& s) const {
	return SimTK::Value<double>::downcast(s.getCacheEntry( _subsystemIndex, _tendonForceIndex)).get();
}

void Delp1990Muscle::setActiveForce(const SimTK::State& s, double force) const {
	SimTK::Value<double>::downcast(s.updCacheEntry( _subsystemIndex, _activeForceIndex)).upd() = force;
}
double Delp1990Muscle::getActiveForce(const SimTK::State& s) const {
	return SimTK::Value<double>::downcast(s.getCacheEntry( _subsystemIndex, _activeForceIndex)).get();
}

void Delp1990Muscle::setPassiveForce(const SimTK::State& s, double force) const {
	SimTK::Value<double>::downcast(s.updCacheEntry( _subsystemIndex, _passiveForceIndex)).upd() = force;
}
double Delp1990Muscle::getPassiveForce(const SimTK::State& s) const {
	return SimTK::Value<double>::downcast(s.getCacheEntry( _subsystemIndex, _passiveForceIndex)).get();
}
//_____________________________________________________________________________
/**
 * Copy the property values from another actuator, which may not be
 * a Delp1990Muscle.
 *
 * @param aActuator Actuator to copy property values from.
 */
void Delp1990Muscle::copyPropertyValues(Actuator& aActuator)
{
	Muscle::copyPropertyValues(aActuator);

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

	prop = aActuator.getPropertySet().contains("mass");
	if (prop) _massProp.setValue(prop->getValueDbl());

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
	Muscle::operator=(aMuscle);

	copyData(aMuscle);

	setup(aMuscle.getModel());

	return(*this);
}


//=============================================================================
// GET AND SET
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
bool Delp1990Muscle::setTimeScale(double aTimeScale)
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
bool Delp1990Muscle::setActivation1(double aActivation1)
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
bool Delp1990Muscle::setActivation2(double aActivation2)
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
bool Delp1990Muscle::setMaxIsometricForce(double aMaxIsometricForce)
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
bool Delp1990Muscle::setOptimalFiberLength(double aOptimalFiberLength)
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
bool Delp1990Muscle::setTendonSlackLength(double aTendonSlackLength)
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
bool Delp1990Muscle::setPennationAngle(double aPennationAngle)
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
bool Delp1990Muscle::setMaxContractionVelocity(double aMaxContractionVelocity)
{
	_maxContractionVelocity = aMaxContractionVelocity;
	return true;
}

//-----------------------------------------------------------------------------
// MASS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the mass of the muscle.
 *
 * @param aMass The mass of the muscle.
 * @return Whether the mass was successfully changed.
 */
bool Delp1990Muscle::setMass(double aMass)
{
	_mass = aMass;
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
double Delp1990Muscle::getPennationAngle(const SimTK::State& s) const
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
double Delp1990Muscle::getNormalizedFiberLength(const SimTK::State& s) const
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
double Delp1990Muscle::getPassiveFiberForce(const SimTK::State& s) const
{
	return getPassiveForce(s);
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
void Delp1990Muscle::scale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	Muscle::scale(s, aScaleSet);

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
void Delp1990Muscle::postScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	Muscle::postScale(s, aScaleSet);

	if (_path.getPreScaleLength(s) > 0.0)
	{
		double scaleFactor = getLength(s) / _path.getPreScaleLength(s);

		_optimalFiberLength *= scaleFactor;
		_tendonSlackLength *= scaleFactor;
		//_maxIsometricForce *= scaleFactor;

		_path.setPreScaleLength(s, 0.0);
	}
}

//_____________________________________________________________________________
/**
 * Compute the derivatives of the muscle states.
 *
 * @param rDYDT the state derivatives are returned here.
 */
void Delp1990Muscle::computeStateDerivatives(const SimTK::State& s)
{
	s.updZDot(_subsystemIndex)[_zIndex+STATE_ACTIVATION] = getActivationDeriv(s);
	s.updZDot(_subsystemIndex)[_zIndex+STATE_FIBER_LENGTH] = getFiberLengthDeriv(s);
	s.updZDot(_subsystemIndex)[_zIndex+STATE_FIBER_VELOCITY] = getFiberVelocityDeriv(s);
}

//_____________________________________________________________________________
/**
 * Compute the equilibrium states.  This method computes a fiber length
 * for the muscle that is consistent with the muscle's activation level.
 */
void Delp1990Muscle::computeEquilibrium(SimTK::State& s) const
{
	double force = computeIsometricForce(s, getActivation(s));

	//cout<<getName()<<": isometric force = "<<force<<endl;
	//cout<<getName()<<": fiber length = "<<getFiberLength(s)<<endl;
}

//_____________________________________________________________________________
/**
 * Compute the actuation for the muscle. This function assumes
 * that computeDerivatives has already been called.
 */
double Delp1990Muscle::computeActuation(const SimTK::State& s) const
{
	double tendonForce;

	// Base Class (to calculate speed)
	Muscle::computeLengtheningSpeed(s);

   double normState[3], normStateDeriv[3], norm_tendon_length, ca, ta;
   double norm_muscle_tendon_length, pennation_angle;

   /* Normalize the muscle states */
   normState[STATE_ACTIVATION] = getActivation(s);
   normState[STATE_FIBER_LENGTH] = getFiberLength(s) / _optimalFiberLength;
   normState[STATE_FIBER_VELOCITY] = getFiberVelocity(s) * (_timeScale / _optimalFiberLength);

	/* Compute normalized muscle state derivatives */
   if (getExcitation(s) >= normState[STATE_ACTIVATION])
      normStateDeriv[STATE_ACTIVATION] = (getExcitation(s) - normState[STATE_ACTIVATION]) * (_activation1 * getExcitation(s) + _activation2);
   else
      normStateDeriv[STATE_ACTIVATION] = (getExcitation(s) - normState[STATE_ACTIVATION]) * _activation2;
   normStateDeriv[STATE_FIBER_LENGTH] = normState[STATE_FIBER_VELOCITY];

	pennation_angle = calcPennation(normState[STATE_FIBER_LENGTH], 1.0, _pennationAngle);
   ca = cos(pennation_angle);
   ta = tan(pennation_angle);
   norm_muscle_tendon_length = getLength(s) / _optimalFiberLength;
   norm_tendon_length = norm_muscle_tendon_length - normState[STATE_FIBER_LENGTH] * ca;
   tendonForce = calcTendonForce(s, norm_tendon_length);
	double fiberForce = calcFiberForce(normState[STATE_ACTIVATION], normState[STATE_FIBER_LENGTH], normState[STATE_FIBER_VELOCITY]);
	double muscleMass = _mass * (_optimalFiberLength / _timeScale) * (_optimalFiberLength / _timeScale);
	double massTerm = (tendonForce * ca - fiberForce * ca * ca) / muscleMass;
	double velocityTerm = normState[STATE_FIBER_VELOCITY] * normState[STATE_FIBER_VELOCITY] * ta * ta / normState[STATE_FIBER_LENGTH];
	normStateDeriv[STATE_FIBER_VELOCITY] = massTerm + velocityTerm;
   setPassiveForce(s, getPassiveForceLengthCurve()->calcValue(SimTK::Vector(1, normState[STATE_FIBER_LENGTH])));
	setActiveForce(s, getActiveForceLengthCurve()->calcValue(SimTK::Vector(1, normState[STATE_FIBER_LENGTH])) * getActivation(s));
	if (getActiveForce(s) < 0.0)
		setActiveForce(s, 0.0);

   /* Un-normalize the muscle state derivatives and forces. */
   setActivationDeriv(s,  normStateDeriv[STATE_ACTIVATION] / _timeScale);
   setFiberLengthDeriv(s, normStateDeriv[STATE_FIBER_LENGTH] * _optimalFiberLength / _timeScale);
   setFiberVelocityDeriv(s, normStateDeriv[STATE_FIBER_VELOCITY] * _optimalFiberLength / (_timeScale * _timeScale));

	tendonForce *= _maxIsometricForce; 
	setTendonForce(s, tendonForce);
	setForce(s, tendonForce);
	setPassiveForce(s, getPassiveForce(s) * _maxIsometricForce);
	setActiveForce(s, getActiveForce(s) * _maxIsometricForce);

	return tendonForce;
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
 * Set the active force-length curve.
 *
 * @param aActiveForceLengthCurve Pointer to an active force-length curve (Function).
 * @return Whether active force-length curve was successfully changed.
 */
bool Delp1990Muscle::setActiveForceLengthCurve(Function* aActiveForceLengthCurve)
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
Function* Delp1990Muscle::getPassiveForceLengthCurve() const
{
	return _passiveForceLengthCurve;
}

//_____________________________________________________________________________
/**
 * Set the passive force-length curve.
 *
 * @param aPassiveForceLengthCurve Pointer to a passive force-length curve (Function).
 * @return Whether passive force-length curve was successfully changed.
 */
bool Delp1990Muscle::setPassiveForceLengthCurve(Function* aPassiveForceLengthCurve)
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
Function* Delp1990Muscle::getTendonForceLengthCurve() const
{
	return _tendonForceLengthCurve;
}

//_____________________________________________________________________________
/**
 * Set the tendon force-length curve.
 *
 * @param aTendonForceLengthCurve Pointer to a tendon force-length curve (Function).
 * @return Whether tendon force-length curve was successfully changed.
 */
bool Delp1990Muscle::setTendonForceLengthCurve(Function* aTendonForceLengthCurve)
{
	_tendonForceLengthCurve = aTendonForceLengthCurve;
	return true;
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
 * Set the force-velocity curve.
 *
 * @param aForceVelocityCurve Pointer to a force-velocity curve (Function).
 * @return Whether force-velocity curve was successfully changed.
 */
bool Delp1990Muscle::setForceVelocityCurve(Function* aForceVelocityCurve)
{
	_forceVelocityCurve = aForceVelocityCurve;
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
double Delp1990Muscle::calcTendonForce(const SimTK::State& s, double aNormTendonLength) const
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
 * Calculate the force in the muscle fibers, which includes the effects of active
 * force, passive force, activation, and contraction velocity.
 *
 * @param aNormTendonLength Normalized length of the tendon.
 * @return The force in the tendon.
 */
double Delp1990Muscle::calcFiberForce(double aActivation, double aNormFiberLength, double aNormFiberVelocity) const
{
   double activeForce = getActiveForceLengthCurve()->calcValue(SimTK::Vector(1, aNormFiberLength));
   double passiveForce = getPassiveForceLengthCurve()->calcValue(SimTK::Vector(1, aNormFiberLength));
   double velocityFactor = getForceVelocityCurve()->calcValue(SimTK::Vector(1, aNormFiberVelocity));

   return aActivation * activeForce * velocityFactor + passiveForce;
}

//_____________________________________________________________________________
/**
 * Compute stress
 */
double Delp1990Muscle::getStress(const SimTK::State& s) const
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
 * will modify the object's values for length, fiber length, active force, 
 * and passive force.
 *
 * @param aActivation Activation of the muscle.
 * @return The isometric force in the muscle.
 */
double Delp1990Muscle::computeIsometricForce(SimTK::State& s, double aActivation) const
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

	double length = getLength(s);

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

		setActiveForce(s, getActiveForceLengthCurve()->calcValue(SimTK::Vector(1, getFiberLength(s) / _optimalFiberLength)) * aActivation * _maxIsometricForce);
      if (getActiveForce(s) < 0.0)
         setActiveForce(s, 0.0);

		setPassiveForce(s,  getPassiveForceLengthCurve()->calcValue(SimTK::Vector(1, getFiberLength(s) / _optimalFiberLength)) * _maxIsometricForce);
      if (getPassiveForce(s) < 0.0)
         setPassiveForce(s, 0.0);

		setTendonForce(s, (getActiveForce(s) + getPassiveForce(s)) * cos_factor);
		setForce(s, getTendonForce(s));
   } else if (length < _tendonSlackLength) {
      tendon_length = length;
		setStateVariable(s, STATE_FIBER_LENGTH,  muscle_width);
		setActiveForce(s, 0.0);
		setPassiveForce(s, 0.0);
		setTendonForce(s, 0.0);
		setForce(s, 0.0);
      return 0.0;
   } else {
      cos_factor = cos(calcPennation(getFiberLength(s), _optimalFiberLength, _pennationAngle));  
      tendon_length = length - getFiberLength(s) * cos_factor;

      /* Check to make sure tendon is not shorter than its slack length. If it
       * is, set the length to its slack length and re-compute fiber length.
       */
      if (tendon_length < _tendonSlackLength) {
         tendon_length = _tendonSlackLength;
         cos_factor = cos(atan(muscle_width / (length - tendon_length)));
         setStateVariable(s, STATE_FIBER_LENGTH,   (length - tendon_length) / cos_factor );
         if (getFiberLength(s) < muscle_width)
            setStateVariable(s, STATE_FIBER_LENGTH,  muscle_width);
      }
   }

   // Muscle-tendon force is found using an iterative method. First, you guess
   // the length of the muscle fibers and the length of the tendon, and
   // calculate their respective forces. If the forces match (are within
   // ERROR_LIMIT of each other), stop; else change the length guesses based
   // on the error and try again.
   for (i = 0; i < MAX_ITERATIONS; i++) {
		setActiveForce(s, getActiveForceLengthCurve()->calcValue(SimTK::Vector(1, getFiberLength(s) / _optimalFiberLength)) * aActivation);
      if (getActiveForce(s) < 0.0)
         setActiveForce(s,0.0);

		setPassiveForce(s, getPassiveForceLengthCurve()->calcValue(SimTK::Vector(1, getFiberLength(s) / _optimalFiberLength)));
      if (getPassiveForce(s) < 0.0)
         setPassiveForce(s, 0.0);

      fiber_force = (getActiveForce(s) + getPassiveForce(s) ) * _maxIsometricForce * cos_factor;

      tendon_strain = (tendon_length / _tendonSlackLength - 1.0);
      if (tendon_strain < 0.0)
         tendon_force = 0.0;
      else
         tendon_force = getTendonForceLengthCurve()->calcValue(SimTK::Vector(1, tendon_strain)) * _maxIsometricForce;
		setTendonForce(s, tendon_force);
		setForce(s, tendon_force);

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
         setStateVariable(s, STATE_FIBER_LENGTH,  getFiberLength(s)+ percent * (tmp_fiber_length - getFiberLength(s)) );
      } else {
         // Estimate the stiffnesses of the tendon and the fibers. If tendon
         // stiffness is too low, then the next length guess will overshoot
         // the equilibrium point. So we artificially raise it using the
         // normalized muscle force. (active force + passive force) is the
         // normalized force for the current fiber length, and we assume that
         // the equilibrium length is close to this current length. So we want
         // to get force = (active force + passive force) from the tendon as well.
         // We hope this will happen by setting the tendon stiffness to
         // (_activeForce+_passiveForce) times its maximum stiffness.
			double tendon_elastic_modulus = 1200.0;
			double tendon_max_stress = 32.0;

         tendon_stiffness = getTendonForceLengthCurve()->calcValue(SimTK::Vector(1, tendon_strain)) *
				_maxIsometricForce / _tendonSlackLength;

         min_tendon_stiffness = (getActiveForce(s) + getPassiveForce(s)) *
	         tendon_elastic_modulus * _maxIsometricForce /
	         (tendon_max_stress * _tendonSlackLength);

         if (tendon_stiffness < min_tendon_stiffness)
            tendon_stiffness = min_tendon_stiffness;

         fiber_stiffness = _maxIsometricForce / _optimalFiberLength *
			 (getActiveForceLengthCurve()->calcValue(SimTK::Vector(1, getFiberLength(s) / _optimalFiberLength))  +
            getPassiveForceLengthCurve()->calcValue(SimTK::Vector(1, getFiberLength(s) / _optimalFiberLength)));

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

      cos_factor = cos(calcPennation(getFiberLength(s), _optimalFiberLength, _pennationAngle));
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

	setPassiveForce(s, getPassiveForce(s) * _maxIsometricForce);
	setActiveForce(s, getActiveForce(s) * _maxIsometricForce);

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
computeIsokineticForceAssumingInfinitelyStiffTendon(SimTK::State& s, double aActivation)
{
	double isometricForce = computeIsometricForce(s, aActivation);

	double normalizedLength = getFiberLength(s) / _optimalFiberLength;
	double normalizedVelocity = cos(_pennationAngle) * getSpeed(s) / (_maxContractionVelocity * _optimalFiberLength);
	double normalizedForceVelocity = evaluateForceLengthVelocityCurve(1.0,normalizedLength,normalizedVelocity);

	return isometricForce * normalizedForceVelocity;
}
