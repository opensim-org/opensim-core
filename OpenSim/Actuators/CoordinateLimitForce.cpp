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
CoordinateLimitForce::CoordinateLimitForce() : Force(),
	_coordName(_propCoordinateName.getValueStr()),
	_upperStiffness(_propUpperStiffness.getValueDbl()),
	_upperLimit(_propUpperLimit.getValueDbl()),
	_lowerStiffness(_propLowerStiffness.getValueDbl()),
	_lowerLimit(_propLowerLimit.getValueDbl()),
	_damping(_propDamping.getValueDbl()),
	_transition(_propTransition.getValueDbl())
{
	setNull();
}

//_____________________________________________________________________________
/**
 * Convenience constructor.
 */
CoordinateLimitForce::CoordinateLimitForce(const string &coordName, double q_upper, 
	double K_upper,	double q_lower, double K_lower, double damping, double dq) 
	: Force(),
	_coordName(_propCoordinateName.getValueStr()),
	_upperStiffness(_propUpperStiffness.getValueDbl()),
	_upperLimit(_propUpperLimit.getValueDbl()),
	_lowerStiffness(_propLowerStiffness.getValueDbl()),
	_lowerLimit(_propLowerLimit.getValueDbl()),
	_damping(_propDamping.getValueDbl()),
	_transition(_propTransition.getValueDbl())
{
	setNull();
	_coordName = coordName;
	_upperStiffness = K_upper;
	_upperLimit = q_upper;
	_lowerStiffness = K_lower;
	_lowerLimit = q_lower;
	_damping = damping;
	_transition = dq;

	setName(_coordName + "_LimitForce");

}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aForce Actuator to be copied.
 */
CoordinateLimitForce::CoordinateLimitForce(const CoordinateLimitForce &aForce) : Force(aForce),
	_coordName(_propCoordinateName.getValueStr()),
	_upperStiffness(_propUpperStiffness.getValueDbl()),
	_upperLimit(_propUpperLimit.getValueDbl()),
	_lowerStiffness(_propLowerStiffness.getValueDbl()),
	_lowerLimit(_propLowerLimit.getValueDbl()),
	_damping(_propDamping.getValueDbl()),
	_transition(_propTransition.getValueDbl())
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
	_propCoordinateName.setName("coordinate"); 
	_propertySet.append( &_propCoordinateName );

	_propUpperStiffness.setComment("Stiffness of the passive limit force when coordinate exceeds upper limit."
				" Note, rotational stiffness expected in N*m/degree.");
	_propUpperStiffness.setName("upper_stiffness");
	_propUpperStiffness.setValue(1);
	_propertySet.append( &_propUpperStiffness );
	
	_propUpperLimit.setComment("The upper limit of the coordinate range of motion (rotations in degrees).");
	_propUpperLimit.setName("upper_limit");
	_propUpperLimit.setValue(0);
	_propertySet.append( &_propUpperLimit );
	
	_propLowerStiffness.setComment("Stiffness of the passive limit force when coordinate exceeds lower limit."
				" Note, rotational stiffness expected in N*m/degree.");
	_propLowerStiffness.setName("lower_stiffness");
	_propLowerStiffness.setValue(1);
	_propertySet.append( &_propLowerStiffness );
	
	_propLowerLimit.setComment("The lower limit of the coordinate range of motion (rotations in degrees).");
	_propLowerLimit.setName("lower_limit");
	_propLowerLimit.setValue(0);
	_propertySet.append( &_propLowerLimit );
	
	_propDamping.setComment("Damping factor on the coordinate's speed applied only when limit is exceeded");
	_propDamping.setName("damping");
	_propDamping.setValue(0.001);
	_propertySet.append( &_propDamping );

	_propTransition.setComment("Transition region width in the units of the coordinate (rotations in degrees)."
		"Dictates the transition from zero to constant stiffness as coordinate exceeds its limit.");
	_propTransition.setName("transition");
	_propTransition.setValue(0.1);
	_propertySet.append( &_propTransition );
}


//_____________________________________________________________________________
/**
 * Copy the member data of the specified actuator.
 */
void CoordinateLimitForce::copyData(const CoordinateLimitForce &aForce)
{
	_coordName = aForce._coordName;
	_upperStiffness = aForce._upperStiffness;
	_upperLimit = aForce._upperLimit;
	_lowerStiffness = aForce._lowerStiffness;
	_lowerLimit = aForce._lowerLimit;
	_damping = aForce._damping;
	_transition = aForce._transition;
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
	_upperStiffness = aUpperStiffness;
}

void CoordinateLimitForce::setUpperLimit(double aUpperLimit)
{
	_upperLimit = aUpperLimit;
}

void CoordinateLimitForce::setLowerStiffness(double aLowerStiffness)
{
	_lowerStiffness = aLowerStiffness;
}

void CoordinateLimitForce::setLowerLimit(double aLowerLimit)
{
	_lowerLimit = aLowerLimit;
}

void CoordinateLimitForce::setDamping(double aDamping)
{
	_damping = aDamping;
}

void CoordinateLimitForce::setTransition(double aTransition)
{
	_transition = aTransition;
}

//_____________________________________________________________________________
/**
 * Get the parameters.
 *
 */
double CoordinateLimitForce::getUpperStiffness() const
{
	return(_upperStiffness);
}

double CoordinateLimitForce::getUpperLimit() const
{
	return(_upperLimit);
}

double CoordinateLimitForce::getLowerStiffness() const
{
	return(_lowerStiffness);
}
double CoordinateLimitForce::getLowerLimit() const
{
	return(_lowerLimit);
}

double CoordinateLimitForce::getDamping() const
{
	return(_damping);
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

	// Base class
	Force::setup(aModel);

	// Look up the coordinate
	if (!_model->updCoordinateSet().contains(_coordName)) {
		errorMessage = "CoordinateLimitForce: Invalid coordinate (" + _coordName + ") specified in Actuator " + getName();
		throw (Exception(errorMessage.c_str()));
	}
	_coord = &_model->updCoordinateSet().get(_coordName);

	// scaling for units
	_w = (_coord->getMotionType() == Coordinate::Rotational) ? SimTK_DEGREE_TO_RADIAN : 1.0;

	// Define the transition from no stiffness to the upperStiffness as coordinate increases 
	// beyond the upper limit
	upStep = new SimTK::Function::Step(0.0, _upperStiffness/_w, _w*_upperLimit, _w*(_upperLimit+_transition));
	// Define the transition from lowerStiffness to zero as coordinate increases to the lower limit
	loStep = new SimTK::Function::Step(_lowerStiffness/_w, 0.0, _w*(_lowerLimit-_transition), _w*_lowerLimit);
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
	double f_up = -K_up*(q - _w*_upperLimit);
	double f_low = K_low*(_w*_lowerLimit - q);
	double f_damp = -_damping*(K_up/_upperStiffness+K_low/_lowerStiffness)*qdot;

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