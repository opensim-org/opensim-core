// CoordinateLimitForce.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHORS:  Ajay Seth
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2011, Stanford University, All rights reserved. 
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
#include "CoordinateLimitForce.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>

using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
CoordinateLimitForce::~CoordinateLimitForce()
{
	delete upStep;
	delete loStep;
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
CoordinateLimitForce::CoordinateLimitForce() : Force()
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Convenience constructor.
 */
CoordinateLimitForce::CoordinateLimitForce(const string &coordName, double q_upper, 
	double K_upper,	double q_lower, double K_lower, double damping, double dq, 
	bool computeDissipationEnergy) : Force()
{
	setNull();
	setPropertyValue("coordinate", coordName);
	setPropertyValue("upper_stiffness", K_upper);
	setPropertyValue("upper_limit", q_upper);
	setPropertyValue("lower_stiffness", K_lower);
	setPropertyValue("lower_limit", q_lower);
	setPropertyValue("damping", damping);
	setPropertyValue("transition", dq);

	setPropertyValue<bool>("compute_dissipation_energy", computeDissipationEnergy);

	setName(coordName + "_LimitForce");

}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aForce Actuator to be copied.
 */
CoordinateLimitForce::CoordinateLimitForce(const CoordinateLimitForce &aForce) : Force(aForce)
{
	setNull();
	copyData(aForce);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void CoordinateLimitForce::setNull()
{
	setupProperties();

	_coord = NULL;
	upStep = NULL;
	loStep = NULL;
	
	// Scaling for coordinate values in m or degrees (rotational) 
	_w = SimTK::NaN;

	// Coordinate limits in internal (SI) units (m or rad)
	_qup = SimTK::NaN;
	_qlow = SimTK::NaN;
	// Constant stiffnesses in internal (SI) N/m or Nm/rad
	_Kup = SimTK::NaN;;
	_Klow = SimTK::NaN;

	// Damping in internal (SI) units of N/(m/s) or Nm/(rad/s)
	_damp = SimTK::NaN;
}
//_____________________________________________________________________________
/**
 * Set up the serializable member variables.  This involves generating
 * properties and connecting local variables to those properties.
 */
void CoordinateLimitForce::setupProperties()
{
	addProperty<string>("coordinate", "Coordinate (name) to be limited.", "UNASSIGNED");
	addProperty<double>("upper_stiffness",
		"Stiffness of the passive limit force when coordinate exceeds upper limit."
		" Note, rotational stiffness expected in N*m/degree.",
		1.0);
	addProperty<double>("upper_limit",
		"The upper limit of the coordinate range of motion (rotations in degrees).",
		0.0);
	addProperty<double>("lower_stiffness",
		"Stiffness of the passive limit force when coordinate exceeds lower limit."
		" Note, rotational stiffness expected in N*m/degree.",
		1.0);
	addProperty<double>("lower_limit",
		"The lower limit of the coordinate range of motion (rotations in degrees).",
		0.0);
	addProperty<double>("damping",
		"Damping factor on the coordinate's speed applied only when limit is exceeded"
		" For translational has units N/(m/s) and rotational has Nm/(degree/s)",
		0.001);
	addProperty<double>("transition",
		"Transition region width in the units of the coordinate (rotations in degrees)."
		" Dictates the transition from zero to constant stiffness as coordinate exceeds its limit.",
		0.1);
	addProperty<bool>("compute_dissipation_energy", 
		"Option to compute the dissipation energy due to damping in the CoordinateLimitForce."
		"If true the dissipation power is automatically integrated to provide energy. Default is false.",
		false);
}


//_____________________________________________________________________________
/**
 * Copy the member data of the specified actuator.
 */
void CoordinateLimitForce::copyData(const CoordinateLimitForce &aForce)
{
    // We allow the coordinate to be missing so we might not get a value here.
	setPropertyValue("coordinate", aForce.getProperty<string>("coordinate"));

	setPropertyValue("upper_stiffness", aForce.getPropertyValue<double>("upper_stiffness"));
	setPropertyValue("upper_limit", aForce.getPropertyValue<double>("upper_limit"));
	setPropertyValue("lower_stiffness", aForce.getPropertyValue<double>("lower_stiffness"));
	setPropertyValue("lower_limit", aForce.getPropertyValue<double>("lower_limit"));
	setPropertyValue("damping", aForce.getPropertyValue<double>("damping"));
	setPropertyValue("transition", aForce.getPropertyValue<double>("transition"));
	setPropertyValue<bool>("compute_dissipation_energy", aForce.getPropertyValue<bool>("compute_dissipation_energy"));
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return  Reference to the altered object.
 */
CoordinateLimitForce& CoordinateLimitForce::operator=(const CoordinateLimitForce &aForce)
{
	// BASE CLASS
	Force::operator =(aForce);

	// DATA
	copyData(aForce);

	return(*this);
}


//=============================================================================
// GET AND SET CoordinateLimitForce Stiffness and Damping parameters
//=============================================================================
void CoordinateLimitForce::setUpperStiffness(double aUpperStiffness)
{
	setPropertyValue("upper_stiffness", aUpperStiffness);
}

void CoordinateLimitForce::setUpperLimit(double aUpperLimit)
{
	setPropertyValue("upper_limit", aUpperLimit);
}

void CoordinateLimitForce::setLowerStiffness(double aLowerStiffness)
{
	setPropertyValue("lower_stiffness", aLowerStiffness);
}

void CoordinateLimitForce::setLowerLimit(double aLowerLimit)
{
	setPropertyValue("lower_limit", aLowerLimit);
}

void CoordinateLimitForce::setDamping(double aDamping)
{
	setPropertyValue("damping", aDamping);
}

void CoordinateLimitForce::setTransition(double aTransition)
{
	setPropertyValue("transition", aTransition);
}

void CoordinateLimitForce::setComputeDissipationEnergy(bool flag)
{
	setPropertyValue<bool>("compute_dissipation_energy", flag);
}

//_____________________________________________________________________________
/**
 * Get the parameters.
 */
double CoordinateLimitForce::getUpperStiffness() const
{
	return getPropertyValue<double>("upper_stiffness");
}

double CoordinateLimitForce::getUpperLimit() const
{
	return getPropertyValue<double>("upper_limit");
}

double CoordinateLimitForce::getLowerStiffness() const
{
	return getPropertyValue<double>("lower_stiffness");
}
double CoordinateLimitForce::getLowerLimit() const
{
	return getPropertyValue<double>("lower_limit");
}

double CoordinateLimitForce::getDamping() const
{
	return getPropertyValue<double>("damping");
}

bool CoordinateLimitForce::isComputingDissipationEnergy() const
{
	return getPropertyValue<bool>("compute_dissipation_energy");
}

//_____________________________________________________________________________
/**
 * Perform some setup functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this CoordinateLimitForce.
 */
void CoordinateLimitForce::setup(Model& aModel)
{
	string errorMessage;

	const string &coordName = getPropertyValue<string>("coordinate");
	const double &upperStiffness = getPropertyValue<double>("upper_stiffness");
	const double &upperLimit = getPropertyValue<double>("upper_limit");
	const double &lowerStiffness = getPropertyValue<double>("lower_stiffness");
	const double &lowerLimit = getPropertyValue<double>("lower_limit");
	const double &transition = getPropertyValue<double>("transition");
	const double &damping = getPropertyValue<double>("damping");

	// Base class
	Super::setup(aModel);

	// Look up the coordinate
	if (!_model->updCoordinateSet().contains(coordName)) {
		errorMessage = "CoordinateLimitForce: Invalid coordinate (" + coordName + ") specified in Actuator " + getName();
		throw (Exception(errorMessage.c_str()));
	}
	_coord = &_model->updCoordinateSet().get(coordName);

	// scaling for units
	_w = (_coord->getMotionType() == Coordinate::Rotational) ? SimTK_DEGREE_TO_RADIAN : 1.0;

	_qup = _w*upperLimit;
	_qlow = _w*lowerLimit;
	_Kup = upperStiffness/_w;
	_Klow = lowerStiffness/_w;
	_damp = damping/_w;

	// Define the transition from no stiffness to the upperStiffness as coordinate increases 
	// beyond the upper limit
	upStep = new SimTK::Function::Step(0.0, _Kup, _qup, _qup+_w*transition);
	// Define the transition from lowerStiffness to zero as coordinate increases to the lower limit
	loStep = new SimTK::Function::Step(_Klow, 0.0, _qlow-_w*transition, _qlow);
}


/** Create the underlying Force that is part of the multibodysystem. */
void CoordinateLimitForce::createSystem(SimTK::MultibodySystem& system) const
{
	Super::createSystem(system);
	addCacheVariable<double>("dissipationPower", 0.0, SimTK::Stage::Dynamics);

	if(isComputingDissipationEnergy()){
		addStateVariable("dissipatedEnergy");
	}
}

//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute and apply the mobility force corrsponding to the passive limit force
 *
 */
void CoordinateLimitForce::computeForce( const SimTK::State& s, 
							   SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							   SimTK::Vector& mobilityForces) const
{
	applyGeneralizedForce(s, *_coord, calcLimitForce(s), mobilityForces);
}

double CoordinateLimitForce::calcLimitForce( const SimTK::State& s) const
{
	double q = _coord->getValue(s);
	SimTK::Vector qv(1,q);
	double K_up = upStep->calcValue(qv);
	double K_low = loStep->calcValue(qv);

	double qdot = _coord->getSpeedValue(s);
	double f_up = -K_up*(q - _qup);
	double f_low = K_low*(_qlow - q);

	// dividing the stiffness by the constant yields the transition function that can 
	// also be appplied to damping
	double f_damp = -_damp*(K_up/_Kup + K_low/_Klow)*qdot;

	// disspative power is negative power but is already implied by "dissipation"
	// so negate power so that dissipation power is a positive number
	double dissPower = -qdot*f_damp;
	setCacheVariable<double>(s, "dissipationPower", dissPower);

	double f_limit = f_up + f_low + f_damp;

	return f_limit;
}

// Potential energy stored in the limit spring
double CoordinateLimitForce::computePotentialEnergy(const SimTK::State& s) const
{
	double q = _coord->getValue(s);
	SimTK::Vector qv(1,q);

	double K=0;
	double delta = 0;
	if(q > _qup){ // against upper limit
		K = _Kup;
		delta = q-_qup;
	}
	else if(q < _qlow){ // against lower limit
		K = _Klow;
		delta = (_qlow-q);
	}
	else{
		// no limits being hit
		return 0.0;
	}
	
	const double &trans = _w*getPropertyValue<double>("transition");

	if(delta >= trans){
		// = 5/14*K*trans^2 - 1/2*K*trans^2 + 1/2*K*(delta)^2
		return K*((-2.0/14.0)*trans*trans + 0.5*(delta)*(delta));
	}
	else{
		double x = delta/trans;
		// This is the integral of K(x)*x*dx evaluated at
		// x = delta/trans, where 
		// K(x) = K*(10*x^3-15*x^4+6*x^5) is the definition of an
		// S curve continuous step function defined in Simbody
		// SimTK/include/SimTKcommon/Scalar.h
		return K*x*x*x*(2.0-2.5*x+(6.0/7.0)*x*x) * (delta*delta);
	}
}
// power dissipated by the damping term of the coodinate limit force
double CoordinateLimitForce::getPowerDissipation(const SimTK::State& s) const
{
	return  getCacheVariable<double>(s, "dissipationPower");
}

// energy dissipated by the damping term of the coodinate limit force
double CoordinateLimitForce::getDissipatedEnergy(const SimTK::State& s) const
{
	if(isComputingDissipationEnergy()){
		return getStateVariable(s, "dissipatedEnergy");
	} else {
		throw Exception("CoordinateLimitForce::getDissipatedEnergy() compute_dissipation_energy set to false");
		return SimTK::NaN;
	}
}

SimTK::Vector CoordinateLimitForce::computeStateVariableDerivatives(const SimTK::State& s) const
{
	return SimTK::Vector(1, getPowerDissipation(s));
}


/** 
 * Methods to query a Force for the value actually applied during simulation
 * The names of the quantities (column labels) is returned by this first function
 * getRecordLabels()
 */
Array<std::string> CoordinateLimitForce::getRecordLabels() const {
	OpenSim::Array<std::string> labels("");
	labels.append(getName());
	labels.append("PotentialEnergy");
	return labels;
}
/**
 * Given SimTK::State object extract all the values necessary to report forces, application location
 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
 */
Array<double> CoordinateLimitForce::getRecordValues(const SimTK::State& state) const {
	OpenSim::Array<double> values(0.0, 0, 2);
	values.append(calcLimitForce(state));
	values.append(computePotentialEnergy(state));
	return values;
}