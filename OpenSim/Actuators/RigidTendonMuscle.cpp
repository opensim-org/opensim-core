/* -------------------------------------------------------------------------- *
 *                      OpenSim:  RigidTendonMuscle.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

//==============================================================================
// INCLUDES
//==============================================================================
#include "RigidTendonMuscle.h"
#include <OpenSim/Common/SimmSpline.h>



//==============================================================================
// STATICS
//==============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3; using SimTK::square; using SimTK::Eps; using SimTK::State;
using SimTK::Vector;

// static int counter=0;
//==============================================================================
// CONSTRUCTOR
//==============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy 
// assignment operator.

//_____________________________________________________________________________
// Default constructor.
RigidTendonMuscle::RigidTendonMuscle()
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
// Convenience Constructor.
RigidTendonMuscle::RigidTendonMuscle(   const std::string& aName,
                                        double aMaxIsometricForce,
                                        double aOptimalFiberLength,
                                        double aTendonSlackLength,
                                        double aPennationAngle)
{
    setNull();
    constructProperties();

    setName(aName);
    setMaxIsometricForce(aMaxIsometricForce);
    setOptimalFiberLength(aOptimalFiberLength);
    setTendonSlackLength(aTendonSlackLength);
    setPennationAngleAtOptimalFiberLength(aPennationAngle);
}

//_____________________________________________________________________________
// Set the data members of this RigidTendonMuscle to their null values.
void RigidTendonMuscle::setNull()
{
    setAuthors("Ajay Seth");
}


//_____________________________________________________________________________
// Allocate and initialize properties.
void RigidTendonMuscle::constructProperties()
{
    int activeForceLengthCurvePoints = 21;
    double activeForceLengthCurveX[] = {-5.30769200, -4.30769200, -1.92307700, -0.88461500, -0.26923100,  0.23076900,  0.46153800,  0.52725000,  0.62875000,  0.71875000,  0.86125000,  1.04500000,  1.21750000,  1.43875000,  1.50000000,  1.61538500,  2.00000000,  2.96153800,  3.69230800,  5.46153800,  9.90190200};
    double activeForceLengthCurveY[] = {0.01218800,  0.02189900,  0.03646600,  0.05249300,  0.07531200,  0.11415800,  0.15785900,  0.22666700,  0.63666700,  0.85666700,  0.95000000,  0.99333300,  0.77000000,  0.24666700,  0.19382100,  0.13325200,  0.07268300,  0.04441700,  0.03634100,  0.02189900,  0.00733200};
    SimmSpline activeForceLengthCurve
       (activeForceLengthCurvePoints, activeForceLengthCurveX, activeForceLengthCurveY);
    constructProperty_active_force_length_curve(activeForceLengthCurve);

    int passiveForceLengthCurvePoints = 13;
    double passiveForceLengthCurveX[] = {-5.00000000,  0.99800000,  0.99900000,  1.00000000,  1.10000000,  1.20000000,  1.30000000,  1.40000000,  1.50000000,  1.60000000,  1.60100000,  1.60200000,  5.00000000};
    double passiveForceLengthCurveY[] = {0.00000000,  0.00000000,  0.00000000,  0.00000000,  0.03500000,  0.12000000,  0.26000000,  0.55000000,  1.17000000,  2.00000000,  2.00000000,  2.00000000,  2.00000000};
    SimmSpline passiveForceLengthCurve
       (passiveForceLengthCurvePoints, passiveForceLengthCurveX, 
        passiveForceLengthCurveY);
    constructProperty_passive_force_length_curve(passiveForceLengthCurve);

    int forceVelocityLengthCurvePoints = 42;
    double forceVelocityLengthCurveX[] = {-1.00100000000, -1.00000000000, -0.95000000000, -0.90000000000, -0.85000000000, -0.80000000000, -0.75000000000, -0.70000000000, -0.65000000000, -0.60000000000, -0.55000000000, -0.50000000000, -0.45000000000, -0.40000000000, -0.35000000000, -0.30000000000, -0.25000000000, -0.20000000000, -0.15000000000, -0.10000000000, -0.05000000000, 0.000000000000,
        0.050000000000, 0.100000000000, 0.150000000000, 0.200000000000, 0.250000000000, 0.300000000000, 0.350000000000, 0.400000000000, 0.450000000000, 0.500000000000, 0.550000000000, 0.600000000000, 0.650000000000, 0.700000000000, 0.750000000000, 0.800000000000, 0.850000000000, 0.900000000000, 0.950000000000, 1.000000000000};
    double forceVelocityLengthCurveY[] = {0.000000000000, 0.000000000000, 0.010417000000, 0.021739000000, 0.034091000000, 0.047619000000, 0.062500000000, 0.078947000000, 0.097222000000, 0.117647000000, 0.140625000000, 0.166667000000, 0.196429000000, 0.230769000000, 0.270833000000, 0.318182000000, 0.375000000000, 0.444444000000, 0.531250000000, 0.642857000000, 0.791667000000, 1.000000000000,
        1.482014000000, 1.601571000000, 1.655791000000, 1.686739000000, 1.706751000000, 1.720753000000, 1.731099000000, 1.739055000000, 1.745365000000, 1.750490000000, 1.754736000000, 1.758312000000, 1.761364000000, 1.763999000000, 1.766298000000, 1.768321000000, 1.770115000000, 1.771717000000, 1.773155000000, 1.774455000000};
    SimmSpline forceVelocityLengthCurve
       (forceVelocityLengthCurvePoints, forceVelocityLengthCurveX, 
        forceVelocityLengthCurveY);
    constructProperty_force_velocity_curve(forceVelocityLengthCurve);
}

//-----------------------------------------------------------------------------
// FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________

//==============================================================================
// CALCULATIONS
//==============================================================================
/* calculate muscle's position related values such fiber and tendon lengths,
    normalized lengths, pennation angle, etc... */
void RigidTendonMuscle::
calcMuscleLengthInfo(const State& s, MuscleLengthInfo& mli) const
{
    mli.tendonLength = getTendonSlackLength();
    double zeroPennateLength = getLength(s) - mli.tendonLength;
    zeroPennateLength = zeroPennateLength < 0 ? 0 : zeroPennateLength;

    if (_muscleWidth > SimTK::SqrtEps) {
        mli.fiberLength = sqrt(square(zeroPennateLength) + square(_muscleWidth));
    mli.cosPennationAngle = zeroPennateLength/mli.fiberLength;
    }
    else { 
        mli.fiberLength = zeroPennateLength;
        //also avoid divide by zero if fiberLength is approaching zero
        mli.cosPennationAngle = 1.0;
    }
        
    mli.pennationAngle = acos(mli.cosPennationAngle);
    mli.normFiberLength = mli.fiberLength/getOptimalFiberLength();

    const Vector arg(1, mli.normFiberLength);
    mli.fiberActiveForceLengthMultiplier = 
        get_active_force_length_curve().calcValue(arg);
    mli.fiberPassiveForceLengthMultiplier = 
        SimTK::clamp(0, get_passive_force_length_curve().calcValue(arg), 10);

    mli.normTendonLength = 1.0;
    mli.tendonStrain = 0.0;
}

void RigidTendonMuscle::calcMusclePotentialEnergyInfo(const SimTK::State& s,
        MusclePotentialEnergyInfo& mpei) const
{
    mpei.fiberPotentialEnergy = 0;
    mpei.tendonPotentialEnergy = 0;
    mpei.musclePotentialEnergy = 0;
}

/* calculate muscle's velocity related values such fiber and tendon velocities,
    normalized velocities, pennation angular velocity, etc... */
void RigidTendonMuscle::calcFiberVelocityInfo(const State& s, FiberVelocityInfo& fvi) const
{
    /*const MuscleLengthInfo &mli = */getMuscleLengthInfo(s);
    fvi.fiberVelocity = getGeometryPath().getLengtheningSpeed(s);
    fvi.normFiberVelocity = fvi.fiberVelocity / 
                            (getOptimalFiberLength()*getMaxContractionVelocity());
    fvi.fiberForceVelocityMultiplier = 
        get_force_velocity_curve().calcValue(Vector(1, fvi.normFiberVelocity));
}

/* calculate muscle's active and passive force-length, force-velocity, 
    tendon force, relationships and their related values */
void RigidTendonMuscle::
calcMuscleDynamicsInfo(const State& s, MuscleDynamicsInfo& mdi) const
{
    const MuscleLengthInfo &mli = getMuscleLengthInfo(s);
    const FiberVelocityInfo &fvi = getFiberVelocityInfo(s);

    mdi.activation = getControl(s);
    double normActiveForce = mdi.activation 
                             * mli.fiberActiveForceLengthMultiplier 
                             * fvi.fiberForceVelocityMultiplier;
    mdi.activeFiberForce =  getMaxIsometricForce()*normActiveForce;
    mdi.passiveFiberForce = getMaxIsometricForce()
                            * mli.fiberPassiveForceLengthMultiplier;
    mdi.fiberForce = mdi.activeFiberForce + mdi.passiveFiberForce;
    mdi.fiberForceAlongTendon = mdi.fiberForce*mli.cosPennationAngle;
    mdi.tendonForce = mdi.fiberForceAlongTendon;

    mdi.normTendonForce = (normActiveForce+mli.fiberPassiveForceLengthMultiplier)
                          * mli.cosPennationAngle;

    mdi.fiberActivePower = -(mdi.activeFiberForce) * fvi.fiberVelocity;
    mdi.fiberPassivePower = -(mdi.passiveFiberForce) * fvi.fiberVelocity;
    mdi.tendonPower = 0;
    mdi.musclePower = -getMaxIsometricForce()*mdi.normTendonForce
                        * fvi.fiberVelocity;
}


//--------------------------------------------------------------------------
// COMPUTATIONS
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the actuation (i.e. activation causing tension) of this muscle. 
 */
double RigidTendonMuscle::computeActuation(const State& s) const
{
    const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
    double force = getFiberForce(s)*mli.cosPennationAngle;
    // store force in the system cache so if needed again it won't have to be
    // recalculated
    setActuation(s, force);

    return(force);
}


double RigidTendonMuscle::computeIsometricForce(State& s, double activation) const
{
    const double &aNormFiberLength = getNormalizedFiberLength(s);

    const Vector arg(1, aNormFiberLength);
    double activeForceLength = 
        get_active_force_length_curve().calcValue(arg);
    double passiveForceLength = 
        get_passive_force_length_curve().calcValue(arg);

    // Isometric means velocity is zero, so velocity "factor" is 1
    return (activation*activeForceLength + passiveForceLength)
           * cos(getPennationAngle(s));
}
