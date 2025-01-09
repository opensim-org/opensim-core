/* -------------------------------------------------------------------------- *
 *                  OpenSim:  Delp1990Muscle_Deprecated.cpp                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//=============================================================================
// INCLUDES
//=============================================================================
#include "Delp1990Muscle_Deprecated.h"
#include <OpenSim/Common/SimmSpline.h>
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Simulation/Model/Model.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

const int Delp1990Muscle_Deprecated::STATE_FIBER_VELOCITY = 2;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Delp1990Muscle_Deprecated::Delp1990Muscle_Deprecated() :
   ActivationFiberLengthMuscle_Deprecated(),
    _timeScale(_timeScaleProp.getValueDbl()),
    _activation1(_activation1Prop.getValueDbl()),
    _activation2(_activation2Prop.getValueDbl()),
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
 * Constructor.
 */
Delp1990Muscle_Deprecated::Delp1990Muscle_Deprecated(const std::string &aName,double aMaxIsometricForce,double aOptimalFiberLength,double aTendonSlackLength,double aPennationAngle) :
   ActivationFiberLengthMuscle_Deprecated(),
    _timeScale(_timeScaleProp.getValueDbl()),
    _activation1(_activation1Prop.getValueDbl()),
    _activation2(_activation2Prop.getValueDbl()),
    _mass(_massProp.getValueDbl()),
    _tendonForceLengthCurve(_tendonForceLengthCurveProp.getValueObjPtrRef()),
    _activeForceLengthCurve(_activeForceLengthCurveProp.getValueObjPtrRef()),
    _passiveForceLengthCurve(_passiveForceLengthCurveProp.getValueObjPtrRef()),
    _forceVelocityCurve(_forceVelocityCurveProp.getValueObjPtrRef())
{
    setNull();
    setupProperties();
    setName(aName);
    setMaxIsometricForce(aMaxIsometricForce);
    setOptimalFiberLength(aOptimalFiberLength);
    setTendonSlackLength(aTendonSlackLength);
    setPennationAngleAtOptimalFiberLength(aPennationAngle);
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
Delp1990Muscle_Deprecated::~Delp1990Muscle_Deprecated()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMuscle Delp1990Muscle_Deprecated to be copied.
 */
Delp1990Muscle_Deprecated::Delp1990Muscle_Deprecated(const Delp1990Muscle_Deprecated &aMuscle) :
   ActivationFiberLengthMuscle_Deprecated(aMuscle),
    _timeScale(_timeScaleProp.getValueDbl()),
    _activation1(_activation1Prop.getValueDbl()),
    _activation2(_activation2Prop.getValueDbl()),
    _mass(_massProp.getValueDbl()),
    _tendonForceLengthCurve(_tendonForceLengthCurveProp.getValueObjPtrRef()),
    _activeForceLengthCurve(_activeForceLengthCurveProp.getValueObjPtrRef()),
    _passiveForceLengthCurve(_passiveForceLengthCurveProp.getValueObjPtrRef()),
    _forceVelocityCurve(_forceVelocityCurveProp.getValueObjPtrRef())
{
    setNull();
    setupProperties();
    copyData(aMuscle);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one Delp1990Muscle_Deprecated to another.
 *
 * @param aMuscle Delp1990Muscle_Deprecated to be copied.
 */
void Delp1990Muscle_Deprecated::copyData(const Delp1990Muscle_Deprecated &aMuscle)
{
    _timeScale = aMuscle._timeScale;
    _activation1 = aMuscle._activation1;
    _activation2 = aMuscle._activation2;
    _mass = aMuscle._mass;

    delete _tendonForceLengthCurve;
    _tendonForceLengthCurve = (Function*)Object::SafeCopy(aMuscle._tendonForceLengthCurve);

    delete _activeForceLengthCurve;
    _activeForceLengthCurve = (Function*)Object::SafeCopy(aMuscle._activeForceLengthCurve);

    delete _passiveForceLengthCurve;
    _passiveForceLengthCurve = (Function*)Object::SafeCopy(aMuscle._passiveForceLengthCurve);

    delete _forceVelocityCurve;
    _forceVelocityCurve = (Function*)Object::SafeCopy(aMuscle._forceVelocityCurve);
}

//_____________________________________________________________________________
/**
 * Set the data members of this Delp1990Muscle_Deprecated to their null values.
 */
void Delp1990Muscle_Deprecated::setNull()
{
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Delp1990Muscle_Deprecated::setupProperties()
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

    _massProp.setName("mass");
    _massProp.setComment("Normalized mass of the muscle between the tendon and muscle fibers");
    _massProp.setValue(0.00287);
    _propertySet.append(&_massProp, "Parameters");

    _tendonForceLengthCurveProp.setName("tendon_force_length_curve");
    _tendonForceLengthCurveProp.setComment("Function representing force-length behavior of tendon");
    int tendonForceLengthCurvePoints = 17;
    double tendonForceLengthCurveX[] = {-10.00000000, -0.00200000, -0.00100000,  0.00000000,  0.00131000,  0.00281000,  0.00431000,  0.00581000,  0.00731000,  0.00881000,  0.01030000,  0.01180000,  0.01230000,  9.20000000,  9.20100000,  9.20200000, 20.00000000};
    double tendonForceLengthCurveY[] = {0.00000000,  0.00000000,  0.00000000,  0.00000000,  0.01080000,  0.02570000,  0.04350000,  0.06520000,  0.09150000,  0.12300000,  0.16100000,  0.20800000,  0.22700000,  345.00000000,  345.00000000,  345.00000000,  345.00000000};
    SimmSpline *tendonForceLengthCurve = new SimmSpline(tendonForceLengthCurvePoints, tendonForceLengthCurveX, tendonForceLengthCurveY);
    _tendonForceLengthCurveProp.setValue(tendonForceLengthCurve);
    _propertySet.append(&_tendonForceLengthCurveProp, "Functions");

    _activeForceLengthCurveProp.setName("active_force_length_curve");
    _activeForceLengthCurveProp.setComment("Function representing active force-length behavior of muscle fibers");
    int activeForceLengthCurvePoints = 21;
    double activeForceLengthCurveX[] = {-5.30769200, -4.30769200, -1.92307700, -0.88461500, -0.26923100,  0.23076900,  0.46153800,  0.52725000,  0.62875000,  0.71875000,  0.86125000,  1.04500000,  1.21750000,  1.43875000,  1.50000000,  1.61538500,  2.00000000,  2.96153800,  3.69230800,  5.46153800,  9.90190200};
    double activeForceLengthCurveY[] = {0.01218800,  0.02189900,  0.03646600,  0.05249300,  0.07531200,  0.11415800,  0.15785900,  0.22666700,  0.63666700,  0.85666700,  0.95000000,  0.99333300,  0.77000000,  0.24666700,  0.19382100,  0.13325200,  0.07268300,  0.04441700,  0.03634100,  0.02189900,  0.00733200};
    SimmSpline *activeForceLengthCurve = new SimmSpline(activeForceLengthCurvePoints, activeForceLengthCurveX, activeForceLengthCurveY);
    _activeForceLengthCurveProp.setValue(activeForceLengthCurve);
    _propertySet.append(&_activeForceLengthCurveProp, "Functions");

    _passiveForceLengthCurveProp.setName("passive_force_length_curve");
    _passiveForceLengthCurveProp.setComment("Function representing passive force-length behavior of muscle fibers");
    int passiveForceLengthCurvePoints = 13;
    double passiveForceLengthCurveX[] = {-5.00000000,  0.99800000,  0.99900000,  1.00000000,  1.10000000,  1.20000000,  1.30000000,  1.40000000,  1.50000000,  1.60000000,  1.60100000,  1.60200000,  5.00000000};
    double passiveForceLengthCurveY[] = {0.00000000,  0.00000000,  0.00000000,  0.00000000,  0.03500000,  0.12000000,  0.26000000,  0.55000000,  1.17000000,  2.00000000,  2.00000000,  2.00000000,  2.00000000};
    SimmSpline *passiveForceLengthCurve = new SimmSpline(passiveForceLengthCurvePoints, passiveForceLengthCurveX, passiveForceLengthCurveY);
    _passiveForceLengthCurveProp.setValue(passiveForceLengthCurve);
    _propertySet.append(&_passiveForceLengthCurveProp, "Functions");

    _forceVelocityCurveProp.setName("force_velocity_curve");
    _forceVelocityCurveProp.setComment("Function representing force-velocity behavior of muscle fibers");
    int forceVelocityLengthCurvePoints = 42;
    double forceVelocityLengthCurveX[] = {-1.001000000000, -1.000000000000, -0.950000000000, -0.900000000000, -0.850000000000, -0.800000000000, -0.750000000000, -0.700000000000, -0.650000000000, -0.600000000000, -0.550000000000, -0.500000000000, -0.450000000000, -0.400000000000, -0.350000000000, -0.300000000000, -0.250000000000, -0.200000000000, -0.150000000000, -0.100000000000, -0.050000000000,
        0.000000000000, 0.050000000000, 0.100000000000, 0.150000000000, 0.200000000000, 0.250000000000, 0.300000000000, 0.350000000000, 0.400000000000, 0.450000000000, 0.500000000000, 0.550000000000, 0.600000000000, 0.650000000000, 0.700000000000, 0.750000000000, 0.800000000000, 0.850000000000, 0.900000000000, 0.950000000000, 1.000000000000};
    double forceVelocityLengthCurveY[] = {0.000000000000, 0.000000000000, 0.010417000000, 0.021739000000, 0.034091000000, 0.047619000000, 0.062500000000, 0.078947000000, 0.097222000000, 0.117647000000, 0.140625000000, 0.166667000000, 0.196429000000, 0.230769000000, 0.270833000000, 0.318182000000, 0.375000000000, 0.444444000000, 0.531250000000, 0.642857000000, 0.791667000000, 1.000000000000,
        1.482014000000, 1.601571000000, 1.655791000000, 1.686739000000, 1.706751000000, 1.720753000000, 1.731099000000, 1.739055000000, 1.745365000000, 1.750490000000, 1.754736000000, 1.758312000000, 1.761364000000, 1.763999000000, 1.766298000000, 1.768321000000, 1.770115000000, 1.771717000000, 1.773155000000, 1.774455000000};
    SimmSpline *forceVelocityLengthCurve = new SimmSpline(forceVelocityLengthCurvePoints, forceVelocityLengthCurveX, forceVelocityLengthCurveY);
    _forceVelocityCurveProp.setValue(forceVelocityLengthCurve);
    _propertySet.append(&_forceVelocityCurveProp, "Functions");
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel model containing this Delp1990Muscle_Deprecated.
 */
void Delp1990Muscle_Deprecated::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);

    // aModel will be NULL when objects are being registered.
    if (!_model)
        return;

    if(!getActiveForceLengthCurve()) 
        throw Exception("Delp1990Muscle_Deprecated::extendConnectToModel(): ERROR- No active force length curve specified for muscle '"+getName()+"'",__FILE__,__LINE__);
    else if(!getPassiveForceLengthCurve())
        throw Exception("Delp1990Muscle_Deprecated::extendConnectToModel(): ERROR- No passive force length curve specified for muscle '"+getName()+"'",__FILE__,__LINE__);
    else if(!getTendonForceLengthCurve())
        throw Exception("Delp1990Muscle_Deprecated::extendConnectToModel(): ERROR- No tendon force length curve specified for muscle '"+getName()+"'",__FILE__,__LINE__);
    else if(!getForceVelocityCurve())
        throw Exception("Delp1990Muscle_Deprecated::extendConnectToModel(): ERROR- No force velocity curve specified for muscle '"+getName()+"'",__FILE__,__LINE__);
}

void Delp1990Muscle_Deprecated::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
    addStateVariable("fiber_velocity");
}

void Delp1990Muscle_Deprecated::setActiveForce( const SimTK::State& s, double force ) const {
    setCacheVariableValue<double>(s, "activeForce", force);
}

double Delp1990Muscle_Deprecated::getActiveForce( const SimTK::State& s) const {
    return getCacheVariableValue<double>(s, "activeForce");
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
Delp1990Muscle_Deprecated& Delp1990Muscle_Deprecated::operator=(const Delp1990Muscle_Deprecated &aMuscle)
{
    // BASE CLASS
    ActivationFiberLengthMuscle_Deprecated::operator=(aMuscle);

    copyData(aMuscle);

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
bool Delp1990Muscle_Deprecated::setTimeScale(double aTimeScale)
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
bool Delp1990Muscle_Deprecated::setActivation1(double aActivation1)
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
bool Delp1990Muscle_Deprecated::setActivation2(double aActivation2)
{
    _activation2 = aActivation2;
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
bool Delp1990Muscle_Deprecated::setMass(double aMass)
{
    _mass = aMass;
    return true;
}





//_____________________________________________________________________________
/**
 * Compute the derivatives of the muscle states.
 *
 * @param s  system state
 */
void Delp1990Muscle_Deprecated::computeStateVariableDerivatives(const SimTK::State &s) const
{
    Super::computeStateVariableDerivatives(s);
    setStateVariableDerivativeValue(s, "fiber_velocity", getFiberVelocityDeriv(s));
}

//_____________________________________________________________________________
/**
 * Compute the actuation for the muscle. This function assumes
 * that computeDerivatives has already been called.
 */
double Delp1990Muscle_Deprecated::computeActuation(const SimTK::State& s) const
{
    double tendonForce;

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

    pennation_angle = calcPennation(normState[STATE_FIBER_LENGTH], 1.0, _pennationAngleAtOptimal);
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
    setActuation(s, tendonForce);
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
Function* Delp1990Muscle_Deprecated::getActiveForceLengthCurve() const
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
bool Delp1990Muscle_Deprecated::setActiveForceLengthCurve(Function* aActiveForceLengthCurve)
{
    delete _activeForceLengthCurve;
    _activeForceLengthCurve = aActiveForceLengthCurve->clone();
    return true;
}

//_____________________________________________________________________________
/**
 * Get the passive force-length curve.
 *
 * @return Pointer to the passive force-length curve (Function).
 */
Function* Delp1990Muscle_Deprecated::getPassiveForceLengthCurve() const
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
bool Delp1990Muscle_Deprecated::setPassiveForceLengthCurve(Function* aPassiveForceLengthCurve)
{
    delete _passiveForceLengthCurve;
    _passiveForceLengthCurve = aPassiveForceLengthCurve->clone();
    return true;
}

//_____________________________________________________________________________
/**
 * Get the tendon force-length curve.
 *
 * @return Pointer to the tendon force-length curve (Function).
 */
Function* Delp1990Muscle_Deprecated::getTendonForceLengthCurve() const
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
bool Delp1990Muscle_Deprecated::setTendonForceLengthCurve(Function* aTendonForceLengthCurve)
{
    delete _tendonForceLengthCurve;
    _tendonForceLengthCurve = aTendonForceLengthCurve->clone();
    return true;
}

//_____________________________________________________________________________
/**
 * Get the force-velocity curve.
 *
 * @return Pointer to the force-velocity curve (Function).
 */
Function* Delp1990Muscle_Deprecated::getForceVelocityCurve() const
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
bool Delp1990Muscle_Deprecated::setForceVelocityCurve(Function* aForceVelocityCurve)
{
    delete _forceVelocityCurve;
    _forceVelocityCurve = aForceVelocityCurve->clone();
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
double Delp1990Muscle_Deprecated::calcTendonForce(const SimTK::State& s, double aNormTendonLength) const
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
double Delp1990Muscle_Deprecated::calcFiberForce(double aActivation, double aNormFiberLength, double aNormFiberVelocity) const
{
    double activeForce = getActiveForceLengthCurve()->calcValue(SimTK::Vector(1, aNormFiberLength));
    double passiveForce = getPassiveForceLengthCurve()->calcValue(SimTK::Vector(1, aNormFiberLength));
    double velocityFactor = getForceVelocityCurve()->calcValue(SimTK::Vector(1, aNormFiberVelocity));

    return aActivation * activeForce * velocityFactor + passiveForce;
}

//_____________________________________________________________________________
/**
 * computeIsometricForce: this function finds the force in a muscle, assuming
 * static equilibrium. Using the total muscle-tendon length, it finds the
 * fiber and tendon lengths so that the forces in each match. This routine
 * takes pennation angle into account, so its definition of static equilibrium
 * is when tendon_force = fiber_force * cos(pennation_angle). This function
 * will modify the object's values for length, fiber length, active force, 
 * and passive force.
 *
 * @param aActivation Activation of the muscle.
 * @return The isometric force in the muscle.
 */
double Delp1990Muscle_Deprecated::computeIsometricForce(SimTK::State& s, double aActivation) const
{
#define MAX_ITERATIONS 100
#define ERROR_LIMIT 0.01

    int i;
    double tendon_length, fiber_force, tmp_fiber_length, min_tendon_stiffness;
    double cos_factor, fiber_stiffness;
    double old_fiber_length{SimTK::NaN}, length_change, tendon_stiffness, percent;
    double error_force = 0.0, old_error_force, tendon_force, tendon_strain;
   
    // If the muscle has no fibers, then treat it as a ligament.
    if (_optimalFiberLength < ROUNDOFF_ERROR) {
        // ligaments should be a separate class, so _optimalFiberLength should
        // never be zero.
        return 0.0;
    }

    double length = getLength(s);

    // rough initial guess of fiber length
    setStateVariableValue(s, STATE_FIBER_LENGTH_NAME,  length - _tendonSlackLength);

    // Make first guess of fiber and tendon lengths. Make fiber length equal to
    // optimal_fiber_length so that you start in the middle of the active+passive
    // force-length curve. Muscle_width is the width, or thickness, of the
    // muscle-tendon unit. It is the shortest allowable fiber length because if
    // the muscle-tendon length is very short, the pennation angle will be 90
    // degrees and the fibers will be vertical (assuming the tendon is horizontal).
    // When this happens, the fibers are as long as the muscle is wide.
    // If the resting tendon length is zero, then set the fiber length equal to
    // the muscle tendon length / cosine_factor, and find its force directly.

    double muscle_width = _optimalFiberLength * sin(_pennationAngleAtOptimal);

    if (_tendonSlackLength < ROUNDOFF_ERROR) {
        tendon_length = 0.0;
        cos_factor = cos(atan(muscle_width / length));
        setStateVariableValue(s, STATE_FIBER_LENGTH_NAME,  length / cos_factor);

        setActiveForce(s, getActiveForceLengthCurve()->calcValue(SimTK::Vector(1, getFiberLength(s) / _optimalFiberLength)) * aActivation * _maxIsometricForce);
        if (getActiveForce(s) < 0.0)
            setActiveForce(s, 0.0);

        setPassiveForce(s,  getPassiveForceLengthCurve()->calcValue(SimTK::Vector(1, getFiberLength(s) / _optimalFiberLength)) * _maxIsometricForce);
        if (getPassiveForce(s) < 0.0)
            setPassiveForce(s, 0.0);

        setTendonForce(s, (getActiveForce(s) + getPassiveForce(s)) * cos_factor);
        setActuation(s, getTendonForce(s));
    } else if (length < _tendonSlackLength) {
        tendon_length = length;
        setStateVariableValue(s, STATE_FIBER_LENGTH_NAME,  muscle_width);
        setActiveForce(s, 0.0);
        setPassiveForce(s, 0.0);
        setTendonForce(s, 0.0);
        setActuation(s, 0.0);
        return 0.0;
    } else {
        cos_factor = cos(calcPennation(getFiberLength(s), _optimalFiberLength, _pennationAngleAtOptimal));  
        tendon_length = length - getFiberLength(s) * cos_factor;

        /* Check to make sure tendon is not shorter than its slack length. If it
            * is, set the length to its slack length and re-compute fiber length.
            */
        if (tendon_length < _tendonSlackLength) {
            tendon_length = _tendonSlackLength;
            cos_factor = cos(atan(muscle_width / (length - tendon_length)));
            setStateVariableValue(s, STATE_FIBER_LENGTH_NAME,   (length - tendon_length) / cos_factor );
            if (getFiberLength(s) < muscle_width)
                setStateVariableValue(s, STATE_FIBER_LENGTH_NAME,  muscle_width);
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
        setActuation(s, tendon_force);

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
            setStateVariableValue(s, STATE_FIBER_LENGTH_NAME,  getFiberLength(s)+ percent * (tmp_fiber_length - getFiberLength(s)) );
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
                setStateVariableValue(s, STATE_FIBER_LENGTH_NAME,  getFiberLength(s) + length_change);
            else
                setStateVariableValue(s, STATE_FIBER_LENGTH_NAME,  getFiberLength(s) - length_change);

        }

        cos_factor = cos(calcPennation(getFiberLength(s), _optimalFiberLength, _pennationAngleAtOptimal));
        tendon_length = length - getFiberLength(s) * cos_factor;

        // Check to make sure tendon is not shorter than its slack length. If it is,
        // set the length to its slack length and re-compute fiber length.
        if (tendon_length < _tendonSlackLength) {
            tendon_length = _tendonSlackLength;
            cos_factor = cos(atan(muscle_width / (length - tendon_length)));
            setStateVariableValue(s, STATE_FIBER_LENGTH_NAME,  (length - tendon_length) / cos_factor );
        }
    }

    _model->getMultibodySystem().realize(s, SimTK::Stage::Position);

    setPassiveForce(s, getPassiveForce(s) * _maxIsometricForce);
    setActiveForce(s, getActiveForce(s) * _maxIsometricForce);

    return tendon_force;
}
