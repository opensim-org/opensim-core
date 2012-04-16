// RigidTendonMuscle.cpp
// Author: Ajay Seth
/*
 * Copyright (c)  2011, Stanford University. All rights reserved. 
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
#include "RigidTendonMuscle.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/NaturalCubicSpline.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

static int counter=0;
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
RigidTendonMuscle::RigidTendonMuscle() : Muscle()
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Convenience Constructor.
 */
RigidTendonMuscle::RigidTendonMuscle(const std::string &aName,double aMaxIsometricForce,double aOptimalFiberLength,double aTendonSlackLength,double aPennationAngle) :
   Muscle()
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
 * Copy constructor.
 *
 * @param aRigidTendonMuscle RigidTendonMuscle to be copied.
 */
RigidTendonMuscle::RigidTendonMuscle(const RigidTendonMuscle &aRigidTendonMuscle) : Muscle(aRigidTendonMuscle)
{
	setNull();
	setupProperties();
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________

//_____________________________________________________________________________
/**
 * Set the data members of this RigidTendonMuscle to their null values.
 */
void RigidTendonMuscle::setNull()
{
}


//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void RigidTendonMuscle::setupProperties()
{
	int activeForceLengthCurvePoints = 21;
	double activeForceLengthCurveX[] = {-5.30769200, -4.30769200, -1.92307700, -0.88461500, -0.26923100,  0.23076900,  0.46153800,  0.52725000,  0.62875000,  0.71875000,  0.86125000,  1.04500000,  1.21750000,  1.43875000,  1.50000000,  1.61538500,  2.00000000,  2.96153800,  3.69230800,  5.46153800,  9.90190200};
	double activeForceLengthCurveY[] = {0.01218800,  0.02189900,  0.03646600,  0.05249300,  0.07531200,  0.11415800,  0.15785900,  0.22666700,  0.63666700,  0.85666700,  0.95000000,  0.99333300,  0.77000000,  0.24666700,  0.19382100,  0.13325200,  0.07268300,  0.04441700,  0.03634100,  0.02189900,  0.00733200};
	NaturalCubicSpline activeForceLengthCurve
       (activeForceLengthCurvePoints, activeForceLengthCurveX, activeForceLengthCurveY);
	addProperty<Function>("active_force_length_curve",
		"Function representing active force-length behavior of muscle fibers",
		activeForceLengthCurve);

	int passiveForceLengthCurvePoints = 13;
	double passiveForceLengthCurveX[] = {-5.00000000,  0.99800000,  0.99900000,  1.00000000,  1.10000000,  1.20000000,  1.30000000,  1.40000000,  1.50000000,  1.60000000,  1.60100000,  1.60200000,  5.00000000};
	double passiveForceLengthCurveY[] = {0.00000000,  0.00000000,  0.00000000,  0.00000000,  0.03500000,  0.12000000,  0.26000000,  0.55000000,  1.17000000,  2.00000000,  2.00000000,  2.00000000,  2.00000000};
	NaturalCubicSpline passiveForceLengthCurve
       (passiveForceLengthCurvePoints, passiveForceLengthCurveX, 
        passiveForceLengthCurveY);
	addProperty<Function>("passive_force_length_curve",
		"Function representing passive force-length behavior of muscle fibers",
		passiveForceLengthCurve);

	int forceVelocityLengthCurvePoints = 42;
	double forceVelocityLengthCurveX[] = {-1.00100000000, -1.00000000000, -0.95000000000, -0.90000000000, -0.85000000000, -0.80000000000, -0.75000000000, -0.70000000000, -0.65000000000, -0.60000000000, -0.55000000000, -0.50000000000, -0.45000000000, -0.40000000000, -0.35000000000, -0.30000000000, -0.25000000000, -0.20000000000, -0.15000000000, -0.10000000000, -0.05000000000, 0.000000000000,
		0.050000000000, 0.100000000000, 0.150000000000, 0.200000000000, 0.250000000000, 0.300000000000, 0.350000000000, 0.400000000000, 0.450000000000, 0.500000000000, 0.550000000000, 0.600000000000, 0.650000000000, 0.700000000000, 0.750000000000, 0.800000000000, 0.850000000000, 0.900000000000, 0.950000000000, 1.000000000000};
	double forceVelocityLengthCurveY[] = {0.000000000000, 0.000000000000, 0.010417000000, 0.021739000000, 0.034091000000, 0.047619000000, 0.062500000000, 0.078947000000, 0.097222000000, 0.117647000000, 0.140625000000, 0.166667000000, 0.196429000000, 0.230769000000, 0.270833000000, 0.318182000000, 0.375000000000, 0.444444000000, 0.531250000000, 0.642857000000, 0.791667000000, 1.000000000000,
		1.482014000000, 1.601571000000, 1.655791000000, 1.686739000000, 1.706751000000, 1.720753000000, 1.731099000000, 1.739055000000, 1.745365000000, 1.750490000000, 1.754736000000, 1.758312000000, 1.761364000000, 1.763999000000, 1.766298000000, 1.768321000000, 1.770115000000, 1.771717000000, 1.773155000000, 1.774455000000};
	NaturalCubicSpline forceVelocityLengthCurve
       (forceVelocityLengthCurvePoints, forceVelocityLengthCurveX, 
        forceVelocityLengthCurveY);
	addProperty<Function>("force_velocity_curve",
		"Function representing force-velocity behavior of muscle fibers",
		forceVelocityLengthCurve);
}

//_____________________________________________________________________________
/**
 * Set the name of the RigidTendonMuscle. This method overrides the one in Object
 * so that the path points can be [re]named accordingly.
 *
 * @param aName The new name of the RigidTendonMuscle.
 */
void RigidTendonMuscle::setName(const string &aName)
{
	// base class
	Muscle::setName(aName);
}


//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @param aRigidTendonMuscle The RigidTendonMuscle from which to copy its data
 * @return Reference to this object.
 */
RigidTendonMuscle& RigidTendonMuscle::operator=(const RigidTendonMuscle &aRigidTendonMuscle)
{
	// base class
	Muscle::operator=(aRigidTendonMuscle);
	return(*this);
}


//-----------------------------------------------------------------------------
// FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________

//=============================================================================
// CALCULATIONS
//=============================================================================
/* calculate muscle's position related values such fiber and tendon lengths,
	normalized lengths, pennation angle, etc... */
void RigidTendonMuscle::calcMuscleLengthInfo(const SimTK::State& s, MuscleLengthInfo& mli) const
{
	mli.tendonLength = getTendonSlackLength();
	double zeroPennateLength = getLength(s) - mli.tendonLength;
	zeroPennateLength = zeroPennateLength < 0 ? 0 : zeroPennateLength;

	mli.fiberLength = sqrt(zeroPennateLength*zeroPennateLength + _muscleWidth*_muscleWidth) + SimTK::Eps;
	
	mli.cosPennationAngle = zeroPennateLength/mli.fiberLength;
	mli.pennationAngle = acos(mli.cosPennationAngle);
	
	mli.normFiberLength = mli.fiberLength/getOptimalFiberLength();

	mli.fiberActiveForceLengthMultiplier = getPropertyValue<Function>("active_force_length_curve").calcValue(SimTK::Vector(1, mli.normFiberLength));
	mli.fiberPassiveForceLengthMultiplier = getPropertyValue<Function>("passive_force_length_curve").calcValue(SimTK::Vector(1, mli.normFiberLength));

	mli.normTendonLength = 1.0;
	mli.tendonStrain = 0.0;

	mli.musclePotentialEnergy =0;
}

/* calculate muscle's velocity related values such fiber and tendon velocities,
	normalized velocities, pennation angular velocity, etc... */
void RigidTendonMuscle::calcFiberVelocityInfo(const SimTK::State& s, FiberVelocityInfo& fvi) const
{
	const MuscleLengthInfo &mli = getMuscleLengthInfo(s);
	fvi.fiberVelocity = getGeometryPath().getLengtheningSpeed(s);
	fvi.normFiberVelocity = fvi.fiberVelocity/(getOptimalFiberLength()*getMaxContractionVelocity());
	fvi.fiberForceVelocityMultiplier = getPropertyValue<Function>("force_velocity_curve").calcValue(SimTK::Vector(1, fvi.normFiberVelocity));
}

/* calculate muscle's active and passive force-length, force-velocity, 
	tendon force, relationships and their related values */
void RigidTendonMuscle::calcMuscleDynamicsInfo(const SimTK::State& s, MuscleDynamicsInfo& mdi) const
{
	const MuscleLengthInfo &mli = getMuscleLengthInfo(s);
	const FiberVelocityInfo &fvi = getFiberVelocityInfo(s);

	mdi.activation = getControl(s);
	double normActiveForce = mdi.activation * mli.fiberActiveForceLengthMultiplier * fvi.fiberForceVelocityMultiplier;
	mdi.activeFiberForce =  getMaxIsometricForce()*normActiveForce;
	mdi.passiveFiberForce = getMaxIsometricForce()*mli.fiberPassiveForceLengthMultiplier;

	mdi.normTendonForce = (normActiveForce+mli.fiberPassiveForceLengthMultiplier)*mli.cosPennationAngle;

	mdi.fiberPower = -(mdi.activeFiberForce + mdi.passiveFiberForce)*fvi.fiberVelocity;
	mdi.tendonPower = 0;
	mdi.musclePower = -getMaxIsometricForce()*mdi.normTendonForce*getSpeed(s);
}


//--------------------------------------------------------------------------
// COMPUTATIONS
//--------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the actuation (i.e. activation causing tenson) of this muscle. 
 */
double RigidTendonMuscle::computeActuation(const SimTK::State& s) const
{
	const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
	double force = getFiberForce(s)*mli.cosPennationAngle;
	// store force in the system cache so if needed again it won't have to be recalculated
	setForce(s, force);

	return(force);
}


double RigidTendonMuscle::computeIsometricForce(SimTK::State& s, double activation) const
{
	const double &aNormFiberLength = getNormalizedFiberLength(s);

	double activeForceLength = 
        getPropertyValue<Function>("active_force_length_curve").calcValue(SimTK::Vector(1, aNormFiberLength));
	double passiveForceLength = 
        getPropertyValue<Function>("passive_force_length_curve").calcValue(SimTK::Vector(1, aNormFiberLength));

	//Isometric means velocity is zero, so velocity "factor" is 1
	return (activation*activeForceLength + passiveForceLength)*cos(getPennationAngle(s));
}
