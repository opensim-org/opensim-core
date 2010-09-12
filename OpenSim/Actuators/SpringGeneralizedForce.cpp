// SpringGeneralizedForce.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHORS:  Frank C. Anderson
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
#include "SpringGeneralizedForce.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>

using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
SpringGeneralizedForce::~SpringGeneralizedForce()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SpringGeneralizedForce::
SpringGeneralizedForce(string aCoordinateName) :
	Force(),
	_coordName(_propCoordinateName.getValueStr()),
	_stiffness(_propStiffness.getValueDbl()),
	_restLength(_propRestLength.getValueDbl()),
	_viscosity(_propViscosity.getValueDbl()),
	_coord(NULL)
{
	// NULL
	setNull();
	_coordName=aCoordinateName;
	if (_model) {
		_coord = &_model->updCoordinateSet().get(_coordName);
	} 
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aforce force to be copied.
 */
SpringGeneralizedForce::
SpringGeneralizedForce(const SpringGeneralizedForce &aForce) :
	Force(aForce),
	_coordName(_propCoordinateName.getValueStr()),
	_stiffness(_propStiffness.getValueDbl()),
	_restLength(_propRestLength.getValueDbl()),
	_viscosity(_propViscosity.getValueDbl()),
	_coord(NULL)
{
	setNull();

	// MEMBER VARIABLES
	_coordName = aForce._coordName;
	setStiffness(aForce.getStiffness());
	setRestLength(aForce.getRestLength());
	setViscosity(aForce.getViscosity());
}

//_____________________________________________________________________________
/**
 * Copy this force and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this force.
 */
Object* SpringGeneralizedForce::
copy() const
{
	SpringGeneralizedForce *force = new SpringGeneralizedForce(*this);
	return force;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this force to their null values.
 */
void SpringGeneralizedForce::
setNull()
{
	setType("SpringGeneralizedForce");
	setupProperties();

	// Defualt VALUES
	_stiffness = 0.0;
	_restLength = 0.0;
	_viscosity = 0.0;
	_coordName="";
}

	
//_____________________________________________________________________________
//
/**
 * Set the data members of this force to their null values.
 */
void SpringGeneralizedForce::
setupProperties()
{
	_propCoordinateName.setName("coordinate"); 
	_propertySet.append( &_propCoordinateName );

	_propStiffness.setName("stiffness");
	_propStiffness.setValue(0.0);
	_propertySet.append( &_propStiffness );

	_propRestLength.setName("rest_length");
	_propRestLength.setValue(0.0);
	_propertySet.append( &_propRestLength );

	_propViscosity.setName("viscosity");
	_propViscosity.setValue(0.0);
	_propertySet.append( &_propViscosity );
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
SpringGeneralizedForce& SpringGeneralizedForce::
operator=(const SpringGeneralizedForce &aForce)
{
	// BASE CLASS
	Force::operator =(aForce);

	// MEMBER VARIABLES
	_coordName = aForce._coordName;
	setStiffness(aForce.getStiffness());
	setRestLength(aForce.getRestLength());
	setViscosity(aForce.getViscosity());

	return(*this);
}

//_____________________________________________________________________________
/**
 * setup sets the _model pointer to proper value
 * _coordinate is actually set inside _createSystem
 */
void SpringGeneralizedForce::setup(Model& aModel)
{
	Force::setup( aModel);

	if (_model) {
		_coord = &_model->updCoordinateSet().get(_coordName);
	}
}

//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// REST LENGTH
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the rest length of the spring.
 *
 * @param aRestLength Rest length of the spring.
 */
void SpringGeneralizedForce::
setRestLength(double aRestLength)
{
	_restLength = aRestLength;
}
//_____________________________________________________________________________
/**
 * Get the rest length of the spring.
 *
 * @return Rest length of the spring.
 */
double SpringGeneralizedForce::
getRestLength() const
{
	return(_restLength);
}

//-----------------------------------------------------------------------------
// VISCOSITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the viscosity of the spring.  Normally the viscosity should be a
 * positive number.  Negative viscosities will put energy into the system
 * rather than apply a damping force.
 *
 * @param aViscosity Viscosity of the spring.
 */
void SpringGeneralizedForce::
setViscosity(double aViscosity)
{
	_viscosity = aViscosity;
}
//_____________________________________________________________________________
/**
 * Get the viscosity of the spring.
 *
 * @return Stiffness of the spring.
 */
double SpringGeneralizedForce::
getViscosity() const
{
	return(_viscosity);
}

//-----------------------------------------------------------------------------
// STIFFNESS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the stiffness of the spring.  Normally the stiffness is a positive
 * quantity.  Negative stiffnessess will result in an unstable system- the
 * force will push away from the rest length instead of pulling toward it.
 *
 * @param aStiffness Stiffness of the spring force.
 */
void SpringGeneralizedForce::
setStiffness(double aStiffness)
{
	_stiffness = aStiffness;
}
//_____________________________________________________________________________
/**
 * Get the stiffness of the force.
 *
 * @return Stiffness of the force.
 */
double SpringGeneralizedForce::
getStiffness() const
{

	return _stiffness;
}


//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the spring force to the
 * model.
 * Force applied = -stiffness * (_coordinateValue - restLength) - viscosity * _coordinateSpeed
 */
void SpringGeneralizedForce::computeForce(const SimTK::State& s, 
							      SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							      SimTK::Vector& generalizedForces) const
{
	if(_model==NULL || _coord == NULL) return;

	// FORCE
	applyGeneralizedForce(s, *_coord, computeForceMagnitude(s), generalizedForces);
}
//_____________________________________________________________________________
/**
 * setup sets the actual Coordinate reference _coord
 */
 void  SpringGeneralizedForce::
createSystem(SimTK::MultibodySystem& system) const {

     Force::createSystem( system );

	if (_model) 
		_coord = &_model->updCoordinateSet().get(_coordName);
     
}
/** 
 * Methods to query a Force for the value actually applied during simulation
 * The names of the quantities (column labels) is returned by this first function
 * getRecordLabels()
 */
OpenSim::Array<std::string> SpringGeneralizedForce::getRecordLabels() const {
	OpenSim::Array<std::string> labels("");
	labels.append(getName()+"_Force");
	return labels;
}
/**
 * Given SimTK::State object extract all the values necessary to report forces, application location
 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
 */
OpenSim::Array<double> SpringGeneralizedForce::getRecordValues(const SimTK::State& state) const {
	OpenSim::Array<double> values(1);

	values.append(computeForceMagnitude(state));
	return values;
};

/**
 * Given SimTK::State object Compute the (signed) magnitude of the force applied
 * along the _coordinate
 */
double SpringGeneralizedForce::
computeForceMagnitude(const SimTK::State& s) const
{
	double q = _coord->getValue(s);
	double speed =  _coord->getSpeedValue(s);
	double force = -getStiffness()*(q - _restLength) - _viscosity*speed;
	return force;
}
