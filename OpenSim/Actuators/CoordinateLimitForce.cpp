// CoordinateLimitForce.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHORS:  Ajay Seth
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University, All rights reserved. 
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
#include <OpenSim/Common/PropertyDbl.h>
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
	double K_upper,	double q_lower, double K_lower, double damping, double dq) 
	: Force()
{
	setNull();
	setPropertyValue("coordinate", coordName);
	setPropertyValue("upper_stiffness", K_upper);
	setPropertyValue("upper_limit", q_upper);
	setPropertyValue("lower_stiffness", K_lower);
	setPropertyValue("lower_limit", q_lower);
	setPropertyValue("damping", damping);
	setPropertyValue("transition", dq);

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
//_____________________________________________________________________________
/**
 * Copy this actuator and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this actuator.
 */
Object* CoordinateLimitForce::copy() const
{
	return new CoordinateLimitForce(*this);
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
	setType("CoordinateLimitForce");
	setupProperties();

	_coord = NULL;
	upStep = NULL;
	loStep = NULL;
	_w = 1.0;
}
//_____________________________________________________________________________
/**
 * Set up the serializable member variables.  This involves generating
 * properties and connecting local variables to those properties.
 */
void CoordinateLimitForce::setupProperties()
{
	addProperty<string>("coordinate",
		"",
		"");
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
		"Damping factor on the coordinate's speed applied only when limit is exceeded",
		0.001);
	addProperty<double>("transition",
		"Transition region width in the units of the coordinate (rotations in degrees)."
		" Dictates the transition from zero to constant stiffness as coordinate exceeds its limit.",
		0.1);
}


//_____________________________________________________________________________
/**
 * Copy the member data of the specified actuator.
 */
void CoordinateLimitForce::copyData(const CoordinateLimitForce &aForce)
{
	setPropertyValue("coordinate", aForce.getPropertyValue<string>("coordinate"));
	setPropertyValue("upper_stiffness", aForce.getPropertyValue<double>("upper_stiffness"));
	setPropertyValue("upper_limit", aForce.getPropertyValue<double>("upper_limit"));
	setPropertyValue("lower_stiffness", aForce.getPropertyValue<double>("lower_stiffness"));
	setPropertyValue("lower_limit", aForce.getPropertyValue<double>("lower_limit"));
	setPropertyValue("damping", aForce.getPropertyValue<double>("damping"));
	setPropertyValue("transition", aForce.getPropertyValue<double>("transition"));
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
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// Param1
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the parameters.
 *
 */
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

//_____________________________________________________________________________
/**
 * Get the parameters.
 *
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

//_____________________________________________________________________________
/**
 * Perform some setup functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this CoordinateActuator.
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

	// Base class
	Force::setup(aModel);

	// Look up the coordinate
	if (!_model->updCoordinateSet().contains(coordName)) {
		errorMessage = "CoordinateLimitForce: Invalid coordinate (" + coordName + ") specified in Actuator " + getName();
		throw (Exception(errorMessage.c_str()));
	}
	_coord = &_model->updCoordinateSet().get(coordName);

	// scaling for units
	_w = (_coord->getMotionType() == Coordinate::Rotational) ? SimTK_DEGREE_TO_RADIAN : 1.0;

	// Define the transition from no stiffness to the upperStiffness as coordinate increases 
	// beyond the upper limit
	upStep = new SimTK::Function::Step(0.0, upperStiffness/_w, _w*upperLimit, _w*(upperLimit+transition));
	// Define the transition from lowerStiffness to zero as coordinate increases to the lower limit
	loStep = new SimTK::Function::Step(lowerStiffness/_w, 0.0, _w*(lowerLimit-transition), _w*lowerLimit);
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
	double f_up = -K_up*(q - _w*getPropertyValue<double>("upper_limit"));
	double f_low = K_low*(_w*getPropertyValue<double>("lower_limit") - q);
	double f_damp = -getPropertyValue<double>("damping")*(K_up/getPropertyValue<double>("upper_stiffness")+K_low/getPropertyValue<double>("lower_stiffness"))*qdot;

	double f_limit = f_up + f_low + f_damp;

	return f_limit;
}


/** 
 * Methods to query a Force for the value actually applied during simulation
 * The names of the quantities (column labels) is returned by this first function
 * getRecordLabels()
 */
Array<std::string> CoordinateLimitForce::getRecordLabels() const {
	OpenSim::Array<std::string> labels("");
	labels.append(getName());
	return labels;
}
/**
 * Given SimTK::State object extract all the values necessary to report forces, application location
 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
 */
Array<double> CoordinateLimitForce::getRecordValues(const SimTK::State& state) const {
	OpenSim::Array<double> values(1);
	values.append(calcLimitForce(state));
	return values;
}