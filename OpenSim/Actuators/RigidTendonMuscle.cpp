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
RigidTendonMuscle::RigidTendonMuscle() : Muscle(),
	_activeForceLengthCurve(_activeForceLengthCurveProp.getValueObjPtrRef()),
	_passiveForceLengthCurve(_passiveForceLengthCurveProp.getValueObjPtrRef()),
	_forceVelocityCurve(_forceVelocityCurveProp.getValueObjPtrRef())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Convenience Constructor.
 */
RigidTendonMuscle::RigidTendonMuscle(const std::string &aName,double aMaxIsometricForce,double aOptimalFiberLength,double aTendonSlackLength,double aPennationAngle) :
   Muscle(),
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
RigidTendonMuscle::~RigidTendonMuscle()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aRigidTendonMuscle RigidTendonMuscle to be copied.
 */
RigidTendonMuscle::RigidTendonMuscle(const RigidTendonMuscle &aRigidTendonMuscle) : Muscle(aRigidTendonMuscle),
	_activeForceLengthCurve(_activeForceLengthCurveProp.getValueObjPtrRef()),
	_passiveForceLengthCurve(_passiveForceLengthCurveProp.getValueObjPtrRef()),
	_forceVelocityCurve(_forceVelocityCurveProp.getValueObjPtrRef())
{
	setNull();
	setupProperties();
	copyData(aRigidTendonMuscle);
}

//_____________________________________________________________________________
/**
 * Copy this muscle and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this Delp1990Muscle.
 */
Object* RigidTendonMuscle::copy() const
{
	RigidTendonMuscle *musc = new RigidTendonMuscle(*this);
	return(musc);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one RigidTendonMuscle to another.
 *
 * @param aRigidTendonMuscle RigidTendonMuscle to be copied.
 */
void RigidTendonMuscle::copyData(const RigidTendonMuscle &aRigidTendonMuscle)
{
}

//_____________________________________________________________________________
/**
 * Set the data members of this RigidTendonMuscle to their null values.
 */
void RigidTendonMuscle::setNull()
{
	setType("RigidTendonMuscle");
}

//_____________________________________________________________________________
/**
 * Override default implementation by object to intercept and fix the XML node
 * underneath the model to match current version
 */
void RigidTendonMuscle::updateFromXMLNode()
{
}

//_____________________________________________________________________________
/**
 * Perform set up functions after model has been deserialized or copied.
 *
 * @param aModel The model containing this RigidTendonMuscle.
 */
void RigidTendonMuscle::setup(Model& aModel)
{
	Muscle::setup(aModel);
}

//_____________________________________________________________________________
/**
 * allocate and initialize the SimTK state for this acuator.
 */
 void RigidTendonMuscle::createSystem(SimTK::MultibodySystem& system) const
{
	Muscle::createSystem(system);

 }

 void RigidTendonMuscle::initState( SimTK::State& s) const
{
    Muscle::initState(s);
}

void RigidTendonMuscle::setDefaultsFromState(const SimTK::State& state)
{
	Muscle::setDefaultsFromState(state);
}


//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void RigidTendonMuscle::setupProperties()
{
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
	double forceVelocityLengthCurveX[] = {-1.00100000000, -1.00000000000, -0.95000000000, -0.90000000000, -0.85000000000, -0.80000000000, -0.75000000000, -0.70000000000, -0.65000000000, -0.60000000000, -0.55000000000, -0.50000000000, -0.45000000000, -0.40000000000, -0.35000000000, -0.30000000000, -0.25000000000, -0.20000000000, -0.15000000000, -0.10000000000, -0.05000000000, 0.000000000000,
		0.050000000000, 0.100000000000, 0.150000000000, 0.200000000000, 0.250000000000, 0.300000000000, 0.350000000000, 0.400000000000, 0.450000000000, 0.500000000000, 0.550000000000, 0.600000000000, 0.650000000000, 0.700000000000, 0.750000000000, 0.800000000000, 0.850000000000, 0.900000000000, 0.950000000000, 1.000000000000};
	double forceVelocityLengthCurveY[] = {0.000000000000, 0.000000000000, 0.010417000000, 0.021739000000, 0.034091000000, 0.047619000000, 0.062500000000, 0.078947000000, 0.097222000000, 0.117647000000, 0.140625000000, 0.166667000000, 0.196429000000, 0.230769000000, 0.270833000000, 0.318182000000, 0.375000000000, 0.444444000000, 0.531250000000, 0.642857000000, 0.791667000000, 1.000000000000,
		1.482014000000, 1.601571000000, 1.655791000000, 1.686739000000, 1.706751000000, 1.720753000000, 1.731099000000, 1.739055000000, 1.745365000000, 1.750490000000, 1.754736000000, 1.758312000000, 1.761364000000, 1.763999000000, 1.766298000000, 1.768321000000, 1.770115000000, 1.771717000000, 1.773155000000, 1.774455000000};
	NaturalCubicSpline *forceVelocityLengthCurve = new NaturalCubicSpline(forceVelocityLengthCurvePoints, forceVelocityLengthCurveX, forceVelocityLengthCurveY);
	_forceVelocityCurveProp.setValue(forceVelocityLengthCurve);
	_propertySet.append(&_forceVelocityCurveProp, "Functions");
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
	copyData(aRigidTendonMuscle);
	return(*this);
}




//_____________________________________________________________________________
/**
 * Get the length of the tendon.
 *
 * @return Current length of the tendon.
 */
double RigidTendonMuscle::getTendonLength(const SimTK::State& s) const
{
	return getTendonSlackLength();
}


//-----------------------------------------------------------------------------
// FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the force generated by the RigidTendonMuscle fibers. This accounts for
 * pennation angle. That is, the fiber force is computed by dividing the
 * actuator force by the cosine of the pennation angle.
 *
 * @return Force in the RigidTendonMuscle fibers.
 */
double RigidTendonMuscle::getFiberForce(const SimTK::State& s) const
{
	const double &aNormFiberLength = getNormalizedFiberLength(s);
	const double &aNormFiberVelocity = getLengtheningSpeed(s)/(getMaxContractionVelocity() * cos(getPennationAngle(s)));

	double activeForceLength = _activeForceLengthCurve->calcValue(SimTK::Vector(1, aNormFiberLength));
	double passiveForceLength = _passiveForceLengthCurve->calcValue(SimTK::Vector(1, aNormFiberLength));
	double activeForceVelocity = _forceVelocityCurve->calcValue(SimTK::Vector(1, aNormFiberVelocity));

	return _maxIsometricForce*(getActivation(s) * activeForceLength * activeForceVelocity + passiveForceLength);
}


double RigidTendonMuscle::getFiberLength(const SimTK::State& s) const
{
	double zeroPennateLength = (getLength(s) - getTendonLength(s));
	double width = sin(_pennationAngleAtOptimal)*_optimalFiberLength;
	return sqrt(zeroPennateLength*zeroPennateLength + width*width);
}


double RigidTendonMuscle::getNormalizedFiberLength(const SimTK::State& s) const
{
	return getFiberLength(s)/getOptimalFiberLength();
}

double RigidTendonMuscle::getPassiveFiberForce(const SimTK::State& s) const
{
	const double &aNormFiberLength = getNormalizedFiberLength(s);
	return _passiveForceLengthCurve->calcValue(SimTK::Vector(1, aNormFiberLength));
}

//_____________________________________________________________________________
/**
 * Get the active force generated by the RigidTendonMuscle fibers.
 *
 * @return Current active force of the RigidTendonMuscle fibers.
 */
double RigidTendonMuscle::getActiveFiberForce(const SimTK::State& s) const
{
	return getFiberForce(s) - getPassiveFiberForce(s);
}
//_____________________________________________________________________________
/**
 * Get the active force generated by the RigidTendonMuscle fibers along the direction
 * of the tendon.
 *
 * @return Current active force of the RigidTendonMuscle fibers along tendon.
 */
double RigidTendonMuscle::getActiveFiberForceAlongTendon(const SimTK::State& s) const
{
	return getActiveFiberForce(s) * cos(getPennationAngle(s));
}
//_____________________________________________________________________________
/**
 * Get the passive force generated by the RigidTendonMuscle fibers along the direction
 * of the tendon.
 *
 * @return Current passive force of the RigidTendonMuscle fibers along tendon.
 */
double RigidTendonMuscle::getPassiveFiberForceAlongTendon(const SimTK::State& s) const
{
	return getPassiveFiberForce(s) * cos(getPennationAngle(s));
}

//_____________________________________________________________________________
/**
 * get the activation (control) value for this RigidTendonMuscle 
 */
double RigidTendonMuscle::getActivation(const SimTK::State& s) const 
{
    return( getControl(s) );
}

void RigidTendonMuscle::setActivation(SimTK::State& s, double activation) const
{
	setControls(SimTK::Vector(1, activation), _model->updControls(s));
}

//=============================================================================
// FORCE APPLICATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Apply the RigidTendonMuscle's force at its points of attachment to the bodies.
 */
void RigidTendonMuscle::computeForce(const SimTK::State& s, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const
{
	Muscle::computeForce(s, bodyForces, generalizedForces);
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
	double force = getFiberForce(s)*cos(getPennationAngle(s));
	// store force in the system cache so if needed again it won't have to be recalculated
	setForce(s, force);

	return(force);
}


double RigidTendonMuscle::computeIsometricForce(SimTK::State& s, double activation) const
{
	const double &aNormFiberLength = getNormalizedFiberLength(s);

	double activeForceLength = _activeForceLengthCurve->calcValue(SimTK::Vector(1, aNormFiberLength));
	double passiveForceLength = _passiveForceLengthCurve->calcValue(SimTK::Vector(1, aNormFiberLength));

	//Isometric means velocity is zero, so velocity "factor" is 1
	return (activation*activeForceLength + passiveForceLength)*cos(getPennationAngle(s));
}